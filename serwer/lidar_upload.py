"""
Odbior nagran ze skanera LiDAR w BMS API (FastAPI).

Wlaczenie w api_server.py:
    from lidar_upload import router as lidar_router
    app.include_router(lidar_router)

Zmienne srodowiskowe (np. w .env uslugi bms-api):
    LIDAR_TOKEN=<dlugi losowy ciag>      # wymagany; bez niego endpointy odpowiadaja 503
    LIDAR_DIR=/mnt/backup/lidar          # gdzie zapisywac nagrania

Protokol (dopisywanie z kontrola pozycji, wiec przerwane wysylki mozna bezpiecznie wznawiac):
    GET /lidar/rozmiar?sesja=S&plik=P            -> {"rozmiar": N}
    PUT /lidar/dopisz?sesja=S&plik=P&offset=N    (tresc = surowe bajty) -> {"rozmiar": N2}
        409 + {"rozmiar": N} gdy offset nie zgadza sie z aktualnym rozmiarem pliku
Naglowek: X-Lidar-Token
"""
import hmac
import os
import re
import threading
from pathlib import Path

from fastapi import APIRouter, Header, HTTPException, Request
from fastapi.responses import JSONResponse

router = APIRouter(prefix="/lidar", tags=["lidar"])

KATALOG = Path(os.environ.get("LIDAR_DIR", "/mnt/backup/lidar"))
MAX_KAWALEK = 8 * 1024 * 1024
PLIK_RE = re.compile(r"^[a-z0-9_]{1,40}\.(csv|txt|bin)$")       # pliki sesji; nowe rodzaje nie wymagaja zmian na serwerze
SESJA_RE = re.compile(r"^[A-Za-z0-9_+\-]{1,100}$")
_blokada = threading.Lock()


def _sprawdz(token, sesja, plik):
    wzor = os.environ.get("LIDAR_TOKEN", "")
    if not wzor:
        raise HTTPException(503, "LIDAR_TOKEN nie jest ustawiony na serwerze")
    if not token or not hmac.compare_digest(token.encode(), wzor.encode()):
        raise HTTPException(401, "zly token")
    if not SESJA_RE.match(sesja) or not PLIK_RE.match(plik):
        raise HTTPException(400, "niedozwolona nazwa sesji lub pliku")
    return KATALOG / sesja / plik


@router.get("/rozmiar")
def rozmiar(sesja: str, plik: str, x_lidar_token: str = Header(None)):
    sciezka = _sprawdz(x_lidar_token, sesja, plik)
    return {"rozmiar": sciezka.stat().st_size if sciezka.exists() else 0}


@router.put("/dopisz")
async def dopisz(sesja: str, plik: str, offset: int, request: Request, x_lidar_token: str = Header(None)):
    sciezka = _sprawdz(x_lidar_token, sesja, plik)
    dane = await request.body()
    if len(dane) > MAX_KAWALEK:
        raise HTTPException(413, "za duzy kawalek")
    with _blokada:
        sciezka.parent.mkdir(parents=True, exist_ok=True)
        obecny = sciezka.stat().st_size if sciezka.exists() else 0
        if offset != obecny:
            return JSONResponse({"rozmiar": obecny}, status_code=409)
        with open(sciezka, "ab") as f:
            f.write(dane)
            f.flush()
            os.fsync(f.fileno())
        return {"rozmiar": obecny + len(dane)}

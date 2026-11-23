#!/usr/bin/env python3
"""
sync_http.py - wysylka nagran z ~/skany do BMS API przez HTTPS (dopisywanie z kontrola pozycji).
Konfiguracja w sync.conf:  URL="https://adres-api"   TOKEN="..."
Wywolywane przez sync.sh; mozna tez recznie: python3 sync_http.py
"""
import json, os, sys, time, urllib.error, urllib.parse, urllib.request

import re
PIERWSZE = ("info.txt", "znaczniki.txt", "pomieszczenia.csv", "grzejniki.csv")      # male pliki ida przed duzymi
WZOR = re.compile(r"^[a-z0-9_]{1,40}\.(csv|txt|bin)$")
KAWALEK = 512 * 1024


def wczytaj_conf(sciezka):
    conf = {}
    with open(sciezka) as f:
        for linia in f:
            linia = linia.strip()
            if linia and not linia.startswith("#") and "=" in linia:
                k, v = linia.split("=", 1)
                conf[k.strip()] = v.strip().strip('"').strip("'")
    return conf


def zapytanie(url, token, metoda="GET", dane=None):
    req = urllib.request.Request(url, data=dane, method=metoda,
                                 headers={"X-Lidar-Token": token, "Content-Type": "application/octet-stream",
                                          "User-Agent": "GOLlidar-sync/1"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b"{}")
        except ValueError:
            return e.code, {}


PL = str.maketrans("ąćęłńóśźżĄĆĘŁŃÓŚŹŻ", "acelnoszzACELNOSZZ")


def bezpieczna(nazwa):
    """Nazwa sesji dla serwera: tylko litery ASCII, cyfry i _ + - (serwer inne odrzuca)."""
    n = nazwa.translate(PL)
    return "".join(ch if (ch.isascii() and ch.isalnum()) or ch in "_+-" else "_" for ch in n)[:100]


def main():
    tu = os.path.dirname(os.path.abspath(__file__))
    conf = wczytaj_conf(os.path.join(tu, "sync.conf"))
    url, token = conf.get("URL", "").rstrip("/"), conf.get("TOKEN", "")
    zrodlo = os.path.expanduser(conf.get("ZRODLO", "~/skany"))
    if not url or not token:
        sys.exit("W sync.conf brakuje URL lub TOKEN")
    limit = time.time() + 50                      # jedno uruchomienie pracuje najwyzej ok. 50 s
    wyslano, bledy = 0, []
    for sesja in sorted(os.listdir(zrodlo)):
        kat = os.path.join(zrodlo, sesja)
        if not os.path.isdir(kat):
            continue
        pliki = [p for p in os.listdir(kat) if WZOR.match(p) and os.path.isfile(os.path.join(kat, p))]
        for plik in sorted(pliki, key=lambda p: (p not in PIERWSZE, p.endswith(".bin"), p)):
            sciezka = os.path.join(kat, plik)
            q = urllib.parse.urlencode({"sesja": bezpieczna(sesja), "plik": plik})
            kod, odp = zapytanie(f"{url}/lidar/rozmiar?{q}", token)
            if kod in (401, 503):
                sys.exit(f"Serwer odpowiedzial {kod}: {odp}")
            if kod != 200:                          # klopot z jedna sesja nie blokuje pozostalych
                bledy.append(f"{sesja}/{plik}: {kod} {odp}")
                continue
            zdalny = int(odp.get("rozmiar", 0))
            with open(sciezka, "rb") as f:
                while time.time() < limit:
                    if os.path.getsize(sciezka) <= zdalny:
                        break
                    f.seek(zdalny)
                    dane = f.read(KAWALEK)
                    if not dane:
                        break
                    kod, odp = zapytanie(f"{url}/lidar/dopisz?{q}&offset={zdalny}", token, "PUT", dane)
                    if kod == 409:
                        zdalny = int(odp.get("rozmiar", 0))
                        continue
                    if kod != 200:
                        bledy.append(f"{sesja}/{plik}: {kod} {odp}")
                        break
                    zdalny = int(odp["rozmiar"])
                    wyslano += len(dane)
    print(f"wyslano {wyslano} bajtow")
    if bledy:
        sys.exit("Bledy:\n  " + "\n  ".join(bledy[:10]))


if __name__ == "__main__":
    try:
        main()
    except (urllib.error.URLError, OSError) as e:
        sys.exit(f"Brak polaczenia: {e}")

#!/bin/bash
# Jednorazowa instalacja odbioru nagran LiDAR w BMS API + konfiguracja skanera.
# Uruchomic NA SERWERZE jako root:
#     sudo bash instaluj_na_serwerze.sh https://ADRES-API  ADRES-IP-MALINY  [uzytkownik-maliny]
# Co robi: dokleja router do api_server.py (z kopia zapasowa), ustawia LIDAR_TOKEN/LIDAR_DIR w uslugi,
# restartuje API, sprawdza endpoint (przy bledzie cofa zmiany), a na koniec wysyla sync.conf na maline
# i wlacza tam kopie co minute. Token nie jest nigdzie wypisywany.
set -euo pipefail
URL="${1:?podaj adres API, np. https://api.example.pl}"; URL="${URL%/}"
RPI="${2:?podaj adres IP maliny}"
RPI_USER="${3:-anakreont}"
USLUGA="${USLUGA:-bms-api}"
TU="$(cd "$(dirname "$0")" && pwd)"
[ "$(id -u)" = 0 ] || { echo "Uruchom przez sudo"; exit 1; }

KAT="$(systemctl show -p WorkingDirectory --value "$USLUGA")"
UZYT="$(systemctl show -p User --value "$USLUGA")"; UZYT="${UZYT:-root}"
API="$KAT/api_server.py"
[ -f "$API" ] || { echo "Nie znalazlem $API (usluga $USLUGA)"; exit 1; }
PORT_API="$(systemctl cat "$USLUGA" | grep -oE -- '--port[ =][0-9]+' | grep -oE '[0-9]+' | head -1)"; PORT_API="${PORT_API:-8080}"
DANE="${LIDAR_DIR:-$(eval echo "~$UZYT")/skany-lidar}"
DROPIN="/etc/systemd/system/$USLUGA.service.d/lidar.conf"
KOPIA="$API.przed-lidar.$(date +%Y%m%d-%H%M%S)"
echo "API: $API (uzytkownik $UZYT, port $PORT_API), nagrania: $DANE"

cofnij() {
    echo "!! Blad - cofam zmiany"
    [ -f "$KOPIA" ] && cp -p "$KOPIA" "$API"
    rm -f "$DROPIN" "$KAT/lidar_upload.py"
    systemctl daemon-reload; systemctl restart "$USLUGA" || true
    exit 1
}

cp -p "$API" "$KOPIA"
install -o "$UZYT" -m 644 "$TU/lidar_upload.py" "$KAT/lidar_upload.py"
if ! grep -q "lidar_upload" "$API"; then
    python3 - "$API" <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
blok = "\n# --- skaner LiDAR: odbior nagran (GOLlidar/serwer/lidar_upload.py)\nfrom lidar_upload import router as _lidar_router\napp.include_router(_lidar_router)\n\n"
m = re.search(r'^if __name__ *== *[\'"]__main__[\'"]', s, re.M)
s = s[:m.start()] + blok.lstrip("\n") + s[m.start():] if m else s.rstrip("\n") + "\n" + blok
open(p, "w").write(s)
PY
fi
if [ -f "$DROPIN" ] && grep -q LIDAR_TOKEN "$DROPIN"; then
    TOKEN="$(grep -oP 'LIDAR_TOKEN=\K[0-9a-f]+' "$DROPIN")"
else
    TOKEN="$(openssl rand -hex 32)"
    mkdir -p "$(dirname "$DROPIN")"
    ( umask 077; printf '[Service]\nEnvironment=LIDAR_TOKEN=%s\nEnvironment=LIDAR_DIR=%s\n' "$TOKEN" "$DANE" > "$DROPIN" )
fi
install -d -o "$UZYT" -m 750 "$DANE"
systemctl daemon-reload
systemctl restart "$USLUGA" || cofnij
for i in $(seq 1 20); do
    KOD="$(curl -s -o /dev/null -w '%{http_code}' -H "X-Lidar-Token: $TOKEN" "http://127.0.0.1:$PORT_API/lidar/rozmiar?sesja=test&plik=info.txt" || true)"
    [ "$KOD" = 200 ] && break; sleep 1
done
[ "$KOD" = 200 ] || { echo "Endpoint lokalnie odpowiada: $KOD"; cofnij; }
echo "OK: endpoint dziala lokalnie"
KOD="$(curl -s -o /dev/null -w '%{http_code}' -H "X-Lidar-Token: $TOKEN" "$URL/lidar/rozmiar?sesja=test&plik=info.txt" || true)"
if [ "$KOD" = 200 ]; then echo "OK: endpoint dziala przez $URL"
else echo "UWAGA: przez $URL odpowiedz $KOD - sprawdz tunel / Cloudflare Access dla sciezki /lidar/*. Lokalnie dziala, zmian nie cofam."; fi

echo "--- Konfiguracja maliny $RPI_USER@$RPI (moze zapytac o haslo maliny, takze do sudo)"
printf 'URL="%s"\nTOKEN="%s"\n' "$URL" "$TOKEN" | ssh -o StrictHostKeyChecking=accept-new "$RPI_USER@$RPI" 'umask 077; cat > ~/skaner/sync.conf'
ssh -t "$RPI_USER@$RPI" 'cd ~/skaner && git pull --ff-only -q; ./sync.sh; echo "stan kopii: $(cat ~/skany/.sync_status 2>/dev/null)"; sudo cp skaner-sync.service skaner-sync.timer /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now skaner-sync.timer && echo "kopia co minute wlaczona"'
echo "Gotowe. Kopia zapasowa api_server.py: $KOPIA"

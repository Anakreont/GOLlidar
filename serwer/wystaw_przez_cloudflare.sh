#!/bin/bash
# Wystawia przegladarke pomiarow LiDAR na zewnatrz przez istniejacy tunel Cloudflare BMS (np. https://lidar.bms.co.pl)
# i wlacza logowanie haslem dla ruchu z internetu (w sieci domowej dalej bez hasla).
# Uruchomienie na Dellu:   sudo bash ~/GOLlidar/serwer/wystaw_przez_cloudflare.sh [lidar.bms.co.pl]
set -euo pipefail
HOST="${1:-lidar.bms.co.pl}"
TUNEL="${TUNEL:-39cbefae-8630-4b49-ad52-4354d5c6f9a4}"
UZYT="${SUDO_USER:-$USER}"
DOM="$(eval echo "~$UZYT")"
[ "$(id -u)" = 0 ] || { echo "Uruchom przez sudo"; exit 1; }
systemctl cat lidar-przegladarka >/dev/null 2>&1 || { echo "Najpierw zainstaluj przegladarke: sudo bash serwer/instaluj_przegladarke.sh"; exit 1; }
PORT="$(systemctl cat lidar-przegladarka | sed -n 's/.*--port \([0-9]*\).*/\1/p' | head -1)"
[ -n "$PORT" ] || PORT=8150

echo "== 1. Haslo do logowania z internetu (w domu haslo nie jest potrzebne)"
while true; do
    read -rsp "Haslo: " H1; echo
    read -rsp "Powtorz: " H2; echo
    [ "$H1" = "$H2" ] && [ ${#H1} -ge 8 ] && break
    echo "Hasla rozne albo krotsze niz 8 znakow - jeszcze raz."
done
HASH="$(printf '%s' "$H1" | sha256sum | cut -d' ' -f1)"; unset H1 H2
SEKRET="$(head -c 32 /dev/urandom | sha256sum | cut -d' ' -f1)"
mkdir -p /etc/systemd/system/lidar-przegladarka.service.d
( umask 077; printf '[Service]\nEnvironment=LIDAR_HASLO_HASH=%s\nEnvironment=LIDAR_SEKRET=%s\n' "$HASH" "$SEKRET" \
    > /etc/systemd/system/lidar-przegladarka.service.d/haslo.conf )
systemctl daemon-reload && systemctl restart lidar-przegladarka
echo "  haslo zapisane (jako skrot), usluga zrestartowana"

echo "== 2. Tunel Cloudflare: $HOST -> http://localhost:$PORT"
CFG=""
for c in /etc/cloudflared/config.yml /etc/cloudflared/config.yaml "$DOM/.cloudflared/config.yml" "$DOM/.cloudflared/config.yaml"; do
    [ -f "$c" ] && { CFG="$c"; break; }
done
if [ -z "$CFG" ]; then
    echo "  Na tym komputerze nie ma konfiguracji cloudflared (moze tunel dziala jeszcze na Toshibie)."
    echo "  Na komputerze z tunelem dopisz w config.yml w sekcji ingress, PRZED linia '- service: http_status:404':"
    echo "    - hostname: $HOST"
    echo "      service: http://192.168.1.73:$PORT"
    echo "  potem:  cloudflared tunnel route dns $TUNEL $HOST  &&  sudo systemctl restart cloudflared"
    exit 0
fi
if grep -q "hostname: *$HOST" "$CFG"; then
    echo "  $HOST juz jest w $CFG - pomijam dopisywanie"
else
    cp -p "$CFG" "$CFG.przed-lidar.$(date +%Y%m%d-%H%M%S)"
    WCIECIE="$(grep -E '^[[:space:]]*- *service: *http_status:404' "$CFG" | head -1 | sed 's/-.*//')"
    [ -n "$(grep -E 'http_status:404' "$CFG")" ] || { echo "  Nie znalazlem linii http_status:404 w $CFG - dopisz recznie (patrz wyzej)"; exit 1; }
    python3 - "$CFG" "$HOST" "$PORT" "$WCIECIE" <<'PY'
import sys
cfg, host, port, w = sys.argv[1:5]
linie = open(cfg).read().split('\n')
i = next(k for k, l in enumerate(linie) if 'http_status:404' in l and l.strip().startswith('-'))
linie[i:i] = [f'{w}- hostname: {host}', f'{w}  service: http://localhost:{port}']
open(cfg, 'w').write('\n'.join(linie))
PY
    echo "  dopisano do $CFG (kopia zapasowa obok)"
fi
if sudo -u "$UZYT" cloudflared tunnel route dns "$TUNEL" "$HOST" 2>&1 | tail -2; then :; fi
cloudflared tunnel ingress validate --config "$CFG" 2>&1 | tail -1 || true
systemctl restart cloudflared 2>/dev/null || systemctl restart "cloudflared@*" 2>/dev/null || echo "  zrestartuj cloudflared recznie"
sleep 3
echo
echo "Gotowe: https://$HOST  (z internetu - haslo; w domu dalej http://$(hostname -I | awk '{print $1}'):$PORT bez hasla)"
echo "Zalecane dodatkowo: Cloudflare Access (Zero Trust) dla $HOST - logowanie kodem z e-maila przed haslem."

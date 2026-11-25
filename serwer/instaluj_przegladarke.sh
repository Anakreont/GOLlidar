#!/bin/bash
# Instaluje przegladarke pomiarow LiDAR na serwerze (Dell) jako usluge systemd, dostepna tylko w sieci domowej.
# Uruchomienie (na Dellu, ze swojego konta):   sudo bash ~/GOLlidar/serwer/instaluj_przegladarke.sh
# Potem w przegladarce w domu:  http://<IP Della>:8090
set -euo pipefail
UZYT="${SUDO_USER:-$USER}"
DOM="$(eval echo "~$UZYT")"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DANE="${LIDAR_DIR:-$DOM/skany-lidar}"
PORT="${PORT:-8090}"

echo "== biblioteki (numpy, scipy, matplotlib, scikit-image, ezdxf)"
apt-get update -qq
apt-get install -y -qq python3-numpy python3-scipy python3-matplotlib python3-skimage >/dev/null
if ! sudo -u "$UZYT" python3 -c "import ezdxf" 2>/dev/null; then
    apt-get install -y -qq python3-ezdxf >/dev/null 2>&1 || \
    sudo -u "$UZYT" python3 -m pip install --user --break-system-packages -q ezdxf || \
    sudo -u "$UZYT" python3 -m pip install --user -q ezdxf
fi
sudo -u "$UZYT" python3 -c "import numpy, scipy, matplotlib, skimage, ezdxf; print('  biblioteki OK')"

echo "== usluga lidar-przegladarka (port $PORT, katalog $DANE)"
cat > /etc/systemd/system/lidar-przegladarka.service <<USL
[Unit]
Description=Przegladarka pomiarow LiDAR (tylko siec lokalna)
After=network.target

[Service]
User=$UZYT
WorkingDirectory=$REPO
Environment=LIDAR_DIR=$DANE
ExecStart=/usr/bin/python3 $REPO/serwer/przegladarka.py --port $PORT
Restart=always
RestartSec=5
Nice=10

[Install]
WantedBy=multi-user.target
USL
systemctl daemon-reload
systemctl enable --now lidar-przegladarka
systemctl restart lidar-przegladarka

if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
    ufw allow from 192.168.0.0/16 to any port "$PORT" proto tcp >/dev/null && echo "  ufw: port $PORT otwarty tylko dla 192.168.x.x"
fi
sleep 2
systemctl is-active --quiet lidar-przegladarka && echo "Gotowe: http://$(hostname -I | awk '{print $1}'):$PORT" \
    || { echo "Usluga nie wstala:"; journalctl -u lidar-przegladarka -n 20 --no-pager; }
echo "Mapy licza sie same po zakonczeniu sesji (ok. 1 min na 8 min nagrania)."

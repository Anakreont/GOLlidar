#!/bin/bash
# Pobiera aktualizacje z GitHuba i (re)startuje rejestrator.
# Gdy zainstalowana jest usluga skaner (autostart), rejestrator zawsze dziala jako usluga - nie w terminalu.
# Uzycie:  ./start.sh                      (wszystkie czujniki)
#          ./start.sh --bez-imu --bez-baro (sam LiDAR, tylko bez uslugi)
cd "$(dirname "$0")" || exit 1

# stara kopia uruchomiona recznie (moze byc uspiona Ctrl+Z - wtedy zwykly sygnal do niej nie dociera)
zamknij_reczne() {
    local pidy
    pidy=$(pgrep -f "python3 .*skan_web.py" | while read -r p; do
        [ "$(systemctl show -p MainPID --value skaner 2>/dev/null)" = "$p" ] || echo "$p"; done)
    [ -z "$pidy" ] && return
    kill -CONT $pidy 2>/dev/null; kill -TERM $pidy 2>/dev/null; sleep 1; kill -KILL $pidy 2>/dev/null; sleep 0.5
}

git pull --ff-only
zamknij_reczne
ADRES="http://$(hostname -I | awk '{print $1}'):8080"

if systemctl is-enabled --quiet skaner 2>/dev/null; then
    [ $# -gt 0 ] && echo "Uwaga: opcje $* pominiete - usluga startuje z ustawieniami z skaner.service"
    sudo systemctl restart skaner
    sleep 2
    if systemctl is-active --quiet skaner; then
        echo "Usluga skaner dziala. Podglad: $ADRES"
    else
        echo "Usluga skaner nie wstala. Log:"; journalctl -u skaner -n 20 --no-pager
    fi
    exit 0
fi

echo "Podglad: $ADRES   (Ctrl+C konczy; nie uzywaj Ctrl+Z)"
exec python3 skan_web.py "$@"

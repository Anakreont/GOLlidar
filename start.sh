#!/bin/bash
# Zamyka stara kopie rejestratora, pobiera aktualizacje z GitHuba i uruchamia nowa wersje.
# Uzycie:  ./start.sh                      (wszystkie czujniki)
#          ./start.sh --bez-imu --bez-baro (sam LiDAR)
cd "$(dirname "$0")" || exit 1
if systemctl is-active --quiet skaner 2>/dev/null; then
    git pull --ff-only
    sudo systemctl restart skaner
    echo "Usluga skaner zrestartowana. Podglad: http://$(hostname -I | awk '{print $1}'):8080"
    exit 0
fi
# stara kopia moze byc uspiona (Ctrl+Z) - wtedy zwykly sygnal do niej nie dociera
if pgrep -f skan_web.py >/dev/null; then
    pkill -CONT -f skan_web.py 2>/dev/null
    pkill -TERM -f skan_web.py 2>/dev/null
    sleep 1
    pkill -KILL -f skan_web.py 2>/dev/null
    sleep 0.5
fi
git pull --ff-only
echo "Podglad: http://$(hostname -I | awk '{print $1}'):8080"
exec python3 skan_web.py "$@"

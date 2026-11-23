#!/bin/bash
# Udostepnia nagrania Claude'owi: wypycha katalog nagran na osobna galaz prywatnego repozytorium (domyslnie SQ9GOL, galaz lidar-dane).
# Uruchamiac NA SERWERZE jako zwykly uzytkownik, po zakonczeniu pomiarow:
#     bash ~/GOLlidar/serwer/wyslij_do_claude.sh                 # wszystkie sesje
#     bash ~/GOLlidar/serwer/wyslij_do_claude.sh 20261126        # tylko sesje, ktorych nazwa zaczyna sie od ...
# Galaz ma zawsze jeden commit (poprzednia zawartosc jest zastepowana), pliki ponad 45 MB sa dzielone na czesci *.czescNNN.
set -euo pipefail
FILTR="${1:-}"
ZR="${LIDAR_DIR:-$HOME/skany-lidar}"
URL="${LIDAR_REPO:-https://github.com/Anakreont/SQ9GOL.git}"
GALAZ="${LIDAR_GALAZ:-lidar-dane}"
R="${LIDAR_ROBOCZY:-$HOME/repos/lidar-dane}"
[ -d "$ZR" ] || { echo "Brak katalogu $ZR"; exit 1; }
rm -rf "$R"; mkdir -p "$R"; cd "$R"
git init -q; git checkout -q --orphan "$GALAZ"; git remote add origin "$URL"
ile=0
for s in "$ZR"/"$FILTR"*/; do
    [ -d "$s" ] || continue
    n="$(basename "$s")"; mkdir -p "$R/$n"; ile=$((ile + 1))
    for f in "$s"*; do
        [ -f "$f" ] || continue
        if [ "$(stat -c %s "$f")" -gt $((45 * 1024 * 1024)) ]; then
            split -b 45m -d -a 3 "$f" "$R/$n/$(basename "$f").czesc"
        else
            cp -p "$f" "$R/$n/"
        fi
    done
done
[ "$ile" -gt 0 ] || { echo "Nie znaleziono sesji pasujacych do '$FILTR'"; exit 1; }
git add -A
git -c user.name="Dell lidar" -c user.email="dell@localhost" commit -q -m "nagrania LiDAR $(date '+%F %H:%M') ($ile sesji)"
git push -q -f origin "$GALAZ"
echo "Wyslano $ile sesji ($(du -sh --exclude=.git . | cut -f1)) na galaz $GALAZ"

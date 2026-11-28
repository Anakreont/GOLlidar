#!/usr/bin/env bash
# Przykladowe nagrania Unitree L2 (rosbag) jako sesje w przegladarce - do obejrzenia, jak strona pracuje z chmura 3D.
#   bash ~/GOLlidar/l2/przyklad_l2.sh ["~/L2 Indoor Point Cloud Data.bag" ...]
# Kazdy bag -> ~/skany-lidar/<data>_Przyklad_Unitree_L2_<nazwa>/l2.bag (dowiazanie twarde, bez kopiowania)
# + info.txt (budynek "Przyklad Unitree L2"). Jesli jest juz policzony wynik z wczesniejszej proby (~/l2_indoor, ~/l2_park),
# kopiuje chmure i rzut, zeby chmura 3D byla widoczna od razu. Pelne liczenie (trasa, pomieszczenia, petle) - przycisk
# "Policz mape L2" w zakladce L2 albo L2_LICZ=1 (zlecenie do przegladarki na localhost).
set -euo pipefail
DANE="${LIDAR_DIR:-$HOME/skany-lidar}"
PORT="${PORT:-8150}"
MM="$HOME/.local/bin/micromamba"; export MAMBA_ROOT_PREFIX="$HOME/.micromamba"
[ $# -gt 0 ] || set -- "$HOME/L2 Indoor Point Cloud Data.bag" "$HOME/L2 Park Point Cloud Data.bag"
for BAG in "$@"; do
  [ -s "$BAG" ] || { echo "brak $BAG - pomijam"; continue; }
  case "$(basename "$BAG")" in
    *ndoor*) NAZ=wnetrze; KOND="wnętrze (przykład Unitree)"; STARE="$HOME/l2_indoor" ;;
    *ark*)   NAZ=park;    KOND="park, na zewnątrz (przykład Unitree)"; STARE="$HOME/l2_park" ;;
    *)       NAZ="$(basename "$BAG" .bag | tr -c 'A-Za-z0-9\n' _ | cut -c1-30)"; KOND="$NAZ"; STARE="" ;;
  esac
  ISTN="$(ls -d "$DANE"/*_Przyklad_Unitree_L2_"$NAZ" 2>/dev/null | head -1 || true)"
  K="${ISTN:-$DANE/$(date +%Y%m%d_%H%M%S)_Przyklad_Unitree_L2_$NAZ}"
  mkdir -p "$K/_wynik/l2"
  [ -e "$K/l2.bag" ] || ln "$BAG" "$K/l2.bag" 2>/dev/null || cp "$BAG" "$K/l2.bag"
  # czas nagrania z naglowka bag (rosbag info z srodowiska ROS 1), w razie bledu bez konca
  SEK="$("$MM" run -p "$HOME/ros1_l2" rosbag info -k duration -y "$K/l2.bag" 2>/dev/null | tr -dc '0-9.' || true)"
  START="$(date -Iseconds | cut -c1-19)"
  {
    echo "nazwa=Przyklad_Unitree_L2_$NAZ"; echo "budynek=Przykład Unitree L2"; echo "kondygnacja=$KOND"
    echo "start=$START"
    [ -n "$SEK" ] && echo "koniec=$(date -d "$START + ${SEK%.*} seconds" -Iseconds | cut -c1-19)"
    echo "l2=rosbag Unitree: $(basename "$BAG")"
  } > "$K/info.txt"
  if [ -n "$STARE" ] && [ -f "$STARE/chmura.pcd" ] && [ ! -f "$K/_wynik/l2/chmura.pcd" ]; then
    for p in chmura.pcd rzut.png rzut.dxf wysokosci.txt otwory.csv otwory.json; do [ -f "$STARE/$p" ] && cp "$STARE/$p" "$K/_wynik/l2/"; done
    echo "  wynik z wczesniejszej proby skopiowany ($STARE)"
  fi
  [ -f "$K/_wynik/l2/chmura.pcd" ] && python3 "$(dirname "$0")/l2_podglad3d.py" "$K/_wynik/l2/chmura.pcd" "$K/_wynik/l2/podglad3d.bin" || true
  echo "sesja: $K ($(du -sh "$K/l2.bag" | cut -f1), ${SEK:-?} s)"
  if [ "${L2_LICZ:-0}" = 1 ]; then
    curl -fsS -X POST "http://127.0.0.1:$PORT/api/l2_licz?n=$(basename "$K")" && echo "  zlecono liczenie L2"
  fi
done

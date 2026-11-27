#!/usr/bin/env bash
# Wywolywane przez ~/bin/l2_mapa wewnatrz srodowiska ~/ros1_l2.  Argumenty: <ws> <bag> [wynik]
set -eo pipefail
WS="$1"; WE="$(realpath "$2")"; TU="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$WE" ]; then                     # katalog sesji skanera z l2.bin -> najpierw rosbag
  [ -f "$WE/l2.bin" ] || { echo "brak $WE/l2.bin"; exit 1; }
  WY="$(realpath -m "${3:-$WE/_wynik/l2}")"; mkdir -p "$WY"
  BAG="$WY/l2.bag"
  python "$TU/l2_bag.py" "$WE" "$BAG"
else
  BAG="$WE"; WY="$(realpath -m "${3:-$(dirname "$BAG")/l2_wynik}")"; mkdir -p "$WY"
fi
source "$WS/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11411 ROS_HOSTNAME=localhost
PCD="$WS/src/point_lio_unilidar/PCD/scans.pcd"; rm -f "$PCD"
roscore -p 11411 > "$WY/roscore.log" 2>&1 & RC=$!
sleep 4
roslaunch point_lio_unilidar mapping_unilidar_l2.launch rviz:=false > "$WY/pointlio.log" 2>&1 & PL=$!
sleep 4
# zapis wyniku Point-LIO (trasa + skany w ukladzie mapy) do zamykania petli (l2_petle.py); kasowany po uzyciu
RB=""
if [ "${L2_PETLE:-1}" = 1 ]; then
  rosbag record -O "$WY/pointlio_wynik.bag" /pointlio/odom /pointlio/cloud_registered __name:=zapis_wyniku > "$WY/zapis.log" 2>&1 & RB=$!
  sleep 2
fi
echo "== odtwarzam $BAG (tyle, ile trwalo nagranie)"
rosbag play --quiet "$BAG" > "$WY/rosbag.log" 2>&1
sleep 5
if [ -n "$RB" ]; then kill -INT $RB; wait $RB || true; fi
kill -INT $PL; wait $PL || true         # Point-LIO zapisuje mape (scans.pcd) przy zamknieciu
kill -INT $RC; wait $RC || true
[ -f "$PCD" ] || { echo "brak $PCD - zobacz $WY/pointlio.log"; exit 1; }
cp "$PCD" "$WY/chmura.pcd"
python "$TU/l2_rzut.py" "$WY/chmura.pcd" "$WY"
# druga wersja mapy: z zamknieciem petli (mapa bez petli zostaje w $WY, z petlami w $WY/petle) - blad nie psuje wyniku
if [ -n "$RB" ] && [ -f "$WY/pointlio_wynik.bag" ]; then
  echo "== zamykanie petli"
  if python "$TU/l2_petle.py" "$WY/pointlio_wynik.bag" "$WY/petle" && python "$TU/l2_rzut.py" "$WY/petle/chmura.pcd" "$WY/petle"; then
    echo "== wynik z petlami: $WY/petle"
  else
    echo "zamykanie petli nieudane - zostaje mapa bez petli"; mkdir -p "$WY/petle"; echo "zamykanie petli nieudane - szczegoly w przebieg.log" > "$WY/petle/blad.txt"
  fi
  if [ "${L2_ZOSTAW_BAG:-0}" != 1 ]; then rm -f "$WY/pointlio_wynik.bag"; fi
fi
# rosbag zrobiony z l2.bin jest tak duzy jak nagranie i da sie go odtworzyc w kazdej chwili - kasujemy (miejsce na dysku)
if [ -d "$WE" ] && [ "${L2_ZOSTAW_BAG:-0}" != 1 ]; then rm -f "$BAG"; fi
echo "== wynik: $WY (rzut.png, rzut.dxf, wysokosci.txt)"

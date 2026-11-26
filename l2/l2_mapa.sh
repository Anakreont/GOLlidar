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
echo "== odtwarzam $BAG (tyle, ile trwalo nagranie)"
rosbag play --quiet "$BAG" > "$WY/rosbag.log" 2>&1
sleep 5
kill -INT $PL; wait $PL || true         # Point-LIO zapisuje mape (scans.pcd) przy zamknieciu
kill -INT $RC; wait $RC || true
[ -f "$PCD" ] || { echo "brak $PCD - zobacz $WY/pointlio.log"; exit 1; }
cp "$PCD" "$WY/chmura.pcd"
python "$TU/l2_rzut.py" "$WY/chmura.pcd" "$WY"
echo "== wynik: $WY (rzut.png, rzut.dxf, wysokosci.txt)"

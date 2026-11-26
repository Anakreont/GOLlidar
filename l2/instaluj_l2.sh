#!/usr/bin/env bash
# Point-LIO dla Unitree L2 na Dellu: ROS 1 Noetic z robostack (micromamba, bez Dockera i bez sudo) do ~/ros1_l2,
# kompilacja point_lio_unilidar (Unitree) do ~/catkin_l2 i polecenie ~/bin/l2_mapa.
#   bash ~/GOLlidar/l2/instaluj_l2.sh
set -euo pipefail
CEL="${CEL:-$HOME/ros1_l2}"
WS="${WS:-$HOME/catkin_l2}"
MM="$HOME/.local/bin/micromamba"
TU="$(cd "$(dirname "$0")" && pwd)"
export MAMBA_ROOT_PREFIX="$HOME/.micromamba"
if [ ! -x "$MM" ]; then
  mkdir -p "$(dirname "$MM")"
  curl -fsSL -o "$MM" https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64
  chmod +x "$MM"
fi
if [ ! -x "$CEL/bin/roscore" ]; then
  echo "== instaluje ROS 1 Noetic + PCL + kompilatory do $CEL (kilka minut)"
  "$MM" create -y -p "$CEL" -c conda-forge -c robostack-noetic python=3.11 \
    ros-noetic-ros-base ros-noetic-pcl-ros ros-noetic-tf ros-noetic-rosbag \
    compilers cmake make pkg-config eigen numpy scipy scikit-image matplotlib ezdxf
fi
mkdir -p "$WS/src"
if [ ! -d "$WS/src/point_lio_unilidar" ]; then
  git clone --depth 1 https://github.com/unitreerobotics/point_lio_unilidar "$WS/src/point_lio_unilidar"
fi
echo "== kompiluje point_lio_unilidar (kilka minut)"
"$MM" run -p "$CEL" bash -c "cd '$WS' && catkin_make -DCMAKE_BUILD_TYPE=Release -j4"
mkdir -p "$HOME/bin"
cat > "$HOME/bin/l2_mapa" <<EOS
#!/usr/bin/env bash
# l2_mapa <nagranie.bag> [katalog_wyniku]  - Point-LIO na nagraniu L2, potem rzut (sciany spod sufitu, ciecie 1,0-1,2 m)
exec "$MM" run -p "$CEL" bash "$TU/l2_mapa.sh" "$WS" "\$@"
EOS
chmod +x "$HOME/bin/l2_mapa"
echo "Gotowe. Uzycie: ~/bin/l2_mapa ~/L2_Indoor.bag ~/l2_wynik"

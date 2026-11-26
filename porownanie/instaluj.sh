#!/usr/bin/env bash
# Instaluje na Dellu srodowisko ROS 2 Humble z slam_toolbox i Cartographerem (robostack, bez Dockera i bez roota)
# do katalogu ~/ros2_slam oraz polecenie ~/bin/porownaj.
#   bash ~/GOLlidar/porownanie/instaluj.sh
set -euo pipefail
CEL="${CEL:-$HOME/ros2_slam}"
MM="$HOME/.local/bin/micromamba"
TU="$(cd "$(dirname "$0")" && pwd)"

if [ ! -x "$MM" ]; then
  echo "== pobieram micromamba"
  mkdir -p "$(dirname "$MM")"
  curl -fsSL -o "$MM" https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64
  chmod +x "$MM"
fi
export MAMBA_ROOT_PREFIX="$HOME/.micromamba"

if [ ! -x "$CEL/lib/slam_toolbox/sync_slam_toolbox_node" ] || [ ! -x "$CEL/lib/cartographer_ros/cartographer_offline_node" ]; then
  echo "== instaluje ROS 2 Humble + slam_toolbox + Cartographer do $CEL (ok. 1,5 GB, kilka minut)"
  "$MM" create -y -p "$CEL" -c conda-forge -c robostack-staging \
    python=3.11 ros-humble-ros-base ros-humble-slam-toolbox ros-humble-cartographer-ros \
    ros-humble-nav2-map-server ros-humble-rosbag2 ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
    numpy scipy matplotlib ezdxf scikit-image pyyaml psutil
else
  echo "== srodowisko $CEL juz jest"
fi

mkdir -p "$HOME/bin"
cat > "$HOME/bin/porownaj" <<EOF
#!/usr/bin/env bash
# porownaj <katalog_sesji> [--metody carto_imu,carto,slam_toolbox] [--kondygnacja N]
set -e
set +u; source "$CEL/setup.bash"; set -u
export PATH="$CEL/bin:\$PATH" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-77}
exec python "$TU/uruchom.py" "\$@"
EOF
chmod +x "$HOME/bin/porownaj"

echo "== test srodowiska"
set +u; source "$CEL/setup.bash"; set -u
"$CEL/bin/python" -c "import rclpy, rosbag2_py, ezdxf, scipy, psutil; print('python: OK')"
ls "$CEL/lib/slam_toolbox/sync_slam_toolbox_node" "$CEL/lib/cartographer_ros/cartographer_offline_node" >/dev/null && echo "slam_toolbox, Cartographer: OK"
echo
echo "Gotowe. Uzycie (sesje sa w /home/anakreont/skany-lidar/):"
echo "  ~/bin/porownaj /home/anakreont/skany-lidar/<sesja>"
echo "Wyniki: <sesja>/_porownanie/ (k0_zestawienie.png, mapy PGM/PNG/DXF) i w przegladarce: zakladka Pliki -> porownanie_k0.png"

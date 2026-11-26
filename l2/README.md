# Unitree L2: Point-LIO i rzut z chmury 3D

1. Instalacja na Dellu, jednorazowo. Instalator stawia ROS 1 Noetic z robostack w `~/ros1_l2` i kompiluje `point_lio_unilidar` od Unitree w `~/catkin_l2`. Nie potrzebuje sudo ani Dockera.

       bash ~/GOLlidar/l2/instaluj_l2.sh

2. Nagranie L2 (.bag, np. przykładowe z https://www.unitree.com/download/L2):

       ~/bin/l2_mapa "~/L2 Indoor Point Cloud Data.bag" ~/l2_indoor

   Program odtwarza nagranie w czasie rzeczywistym (Point-LIO), zapisuje `chmura.pcd`, a potem uruchamia `l2_rzut.py`.

3. `l2_rzut.py`: podłogę i sufit znajduje z histogramu wysokości. Daje:
   - **SCIANY**: pas 15–45 cm pod sufitem,
   - **CIECIE_1M**: pas 1,0–1,2 m nad podłogą,
   - wysokość w świetle.

   Pliki wynikowe: `rzut.png`, `rzut.dxf` (w metrach) i `wysokosci.txt`.

Stan na 26.11.2026: `l2_rzut.py` sprawdzony na sztucznej chmurze. Instalator i Point-LIO nie były uruchamiane, pierwsza próba odbywa się na Dellu.

## Nagrywanie L2 skanerem (malina) — od 27.11.2026

1. **Adres L2** (raz): `l2/l2_ustaw_ip.py` — opis w nagłówku pliku. Docelowo L2 192.168.2.62 → malina 192.168.2.2 (eth0, bez bramy).
2. **Rejestrator** (`skan_web.py`, opcja `--l2 auto`, domyślnie): wątek nasłuchuje UDP na porcie 6201, w razie ciszy wysyła do L2 polecenie startu pracy. Jeśli L2 przysyła dane w chwili START, każdy datagram trafia surowo do `l2.bin` (format jak `lidar.bin`), a w info.txt jest `l2=<ip:port>`. Na stronie kafelek „Unitree L2 (3D)” z liczbą pakietów/s (ok. 200 chmury + IMU).
3. **Mapa na Dellu:** `~/bin/l2_mapa <katalog_sesji>` — `l2_bag.py` robi rosbag (`/unilidar/cloud`, `/unilidar/imu`, 18 pakietów na chmurę, czas sprzętowy L2 przeliczony na czas maliny), potem Point-LIO i `l2_rzut.py`. Wynik w `<sesja>/_wynik/l2/`.
   - Do istniejącego środowiska raz: `~/.local/bin/micromamba run -p ~/ros1_l2 pip install rosbags`.
4. Protokół i przeliczenie punktów: `l2_protokol.py` (wg unilidar_sdk2 i discodyer/unilidar_driver). Sprawdzone na sztucznych pakietach; na prawdziwym L2 jeszcze nie.

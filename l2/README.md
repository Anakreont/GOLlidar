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

   - **OKNA / DRZWI** (od 27.11.2026): dla każdego odcinka ściany siatka 8 × 8 cm w płaszczyźnie ściany na całej wysokości; otwór = brak punktów w ścianie i brak przeszkody tuż przed nią (meble zasłaniające ścianę nie są otworem), punkty za ścianą potwierdzają otwór. Drzwi = od podłogi, wys. ≥ 1,7 m; okno = parapet ≥ 0,3 m. Krawędzie doprecyzowane z punktów ściany. W DXF warstwy OKNA, DRZWI (z łukiem skrzydła), OTWORY_OPIS; lista w `otwory.json` i `otwory.csv` (szerokość, wysokość, parapet, nadproże). `--bez-otworow` wyłącza. Sprawdzone na sztucznym pokoju (okno 1,50 × 1,50 p. 0,90 → 1,52 × 1,49 p. 0,89; drzwi 0,90 × 2,05 → 0,92 × 2,05; szafa przy ścianie nie wykryta jako otwór).

   Pliki wynikowe: `rzut.png`, `rzut.dxf` (w metrach), `wysokosci.txt`, `otwory.json`, `otwory.csv`.

Stan na 26.11.2026: `l2_rzut.py` sprawdzony na sztucznej chmurze. Instalator i Point-LIO nie były uruchamiane, pierwsza próba odbywa się na Dellu.

## Nagrywanie L2 skanerem (malina) — od 27.11.2026

1. **Adres L2** (raz): `l2/l2_ustaw_ip.py` — opis w nagłówku pliku. Docelowo L2 192.168.2.62 → malina 192.168.2.2 (eth0, bez bramy).
2. **Rejestrator** (`skan_web.py`, opcja `--l2 auto`, domyślnie): wątek nasłuchuje UDP na porcie 6201, w razie ciszy wysyła do L2 polecenie startu pracy. Jeśli L2 przysyła dane w chwili START, każdy datagram trafia surowo do `l2.bin` (format jak `lidar.bin`), a w info.txt jest `l2=<ip:port>`. Na stronie kafelek „Unitree L2 (3D)” z liczbą pakietów/s (ok. 200 chmury + IMU).
3. **Mapa na Dellu:** `~/bin/l2_mapa <katalog_sesji>` — `l2_bag.py` robi rosbag (`/unilidar/cloud`, `/unilidar/imu`, 18 pakietów na chmurę, czas sprzętowy L2 przeliczony na czas maliny), potem Point-LIO i `l2_rzut.py`. Wynik w `<sesja>/_wynik/l2/`.
   - Do istniejącego środowiska raz: `~/.local/bin/micromamba run -p ~/ros1_l2 pip install rosbags`.
4. Protokół i przeliczenie punktów: `l2_protokol.py` (wg unilidar_sdk2 i discodyer/unilidar_driver). Sprawdzone na sztucznych pakietach; na prawdziwym L2 jeszcze nie.
5. **Przeglądarka na Dellu** (od 27.11.2026): zakładka „Unitree L2 (3D)” przy nagraniu – przycisk „Policz mapę L2” (kolejka, jedno liczenie naraz, w tym samym limicie pamięci co mapy 2D; trwa mniej więcej tyle, co nagranie), podgląd rzutu, wysokość w świetle, pobieranie `rzut.dxf` (w m/cm/mm) i `chmura.pcd`. Nagrania bez D500 mają stan „tylko L2” – nie są traktowane jak puste i automat nie liczy dla nich mapy 2D. Usuwanie/przeliczanie mapy 2D nie kasuje wyników L2. Lista nagrań: budynek → dzień → nagranie (przełącznik „wg kondygnacji”).

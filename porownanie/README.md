# Porównanie z gotowymi programami SLAM

Te same nagrania skanera liczone przez **slam_toolbox** (Karto) i **Cartographera** (sam LiDAR oraz z kursem z BNO085). Wynik stawiamy obok naszego potoku z `mapa/`.

Instalacja na Dellu, jednorazowo, bez Dockera i bez sudo. Instalator kładzie ROS 2 Humble z robostack do `~/ros2_slam` i zajmuje ok. 1,5 GB:

    bash ~/GOLlidar/porownanie/instaluj.sh

Liczenie jednej sesji. Domyślnie wszystkie kondygnacje z barometru i oba warianty Cartographera:

    ~/bin/porownaj /home/anakreont/skany-lidar/<sesja>
    ~/bin/porownaj <sesja> --metody carto_imu --kondygnacja 0
    ~/bin/porownaj <sesja> --metody slam_toolbox      # eksperymentalnie, patrz niżej

Wyniki trafiają do `<sesja>/_porownanie/`:
- `k<N>_zestawienie.png` z metodami obok siebie; kopia jest w przeglądarce w zakładce Pliki jako `porownanie_k<N>.png`,
- `k<N>_<metoda>.png` / `.pgm` / `.yaml` z mapą zajętości w siatce 2,5 cm,
- `k<N>_<metoda>.dxf` z punktami ścian (warstwa SCIANY_PUNKTY) do nałożenia na rzut,
- `*.log` z logami programów i `wynik.json`.

Do programów trafia to samo, co do naszego potoku: obroty D500 po korekcji ruchu (kurs z IMU), bez operatora i słabych odbić. Skany z PAUZY i z przechyłem powyżej 12° są pominięte. Odometria to sam kurs z BNO085, bez przesunięcia.

Uwaga: Cartographer z robostack nie rozwiązuje `include` w plikach .lua, więc `uruchom.py` skleja konfigurację w `_porownanie/_lua/`. Globalne szukanie pętli jest wyłączone (`global_sampling_ratio = 0`), bo podobne sale łączyły się błędnie.

Test na sesji syntetycznej (korytarz 30 m i 4 sale różnej szerokości, 254 s): Cartographer odtworzył układ poprawnie w obu wariantach (sam LiDAR 85 s, z IMU 44 s na 2 rdzeniach). slam_toolbox na tym samym nagraniu się gubi: korytarz wychodzi skrócony, a sale zlewają się w jedną. Karto potrzebuje odometrii z przesunięciem (np. z kół robota), a my podajemy tylko kurs. Dlatego slam_toolbox nie jest w metodach domyślnych.

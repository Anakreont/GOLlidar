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

## Zamykanie pętli (`l2_petle.py`) — od 27.11.2026

Point-LIO sam nie zamyka pętli, więc na długiej trasie (cała szkoła) mapa może się
lekko rozjechać: ten sam korytarz przy powrocie wychodzi przesunięty o kilkadziesiąt cm.
`l2_mapa.sh` liczy więc **dwie wersje**:

| wersja | gdzie | co |
|---|---|---|
| bez pętli | `_wynik/l2/` | czysty Point-LIO (jak dotąd) |
| z pętlami | `_wynik/l2/petle/` | to samo + korekta trasy po powrotach w znane miejsca |

Jak działa: co ~1 m / 15° klatka kluczowa → przy powrocie (< 5 m od miejsca sprzed
> 15 m trasy) dopasowanie ICP 3D → odrzucenie słabych i „korytarzowych” (zdegenerowanych)
dopasowań → graf pozycji (x, y, z, kurs) z odporną stratą → poprawka rozłożona na wszystkie
skany → nowa chmura → rzut DXF + okna i drzwi.

Pliki w `_wynik/l2/petle/`: `rzut.dxf`, `rzut.png`, `trasa.png` (trasa przed/po, pętle na
czerwono), `petle.json` (raport), `otwory.csv`, `chmura.pcd`. Gdy coś pójdzie nie tak —
`blad.txt`, a wersja bez pętli zostaje nietknięta.

W przeglądarce (zakładka L2) wybierasz, którą wersję eksportować (zapis w `_wynik/l2/wybor.txt`);
przycisk „Rzut L2 DXF” w nagłówku pobiera wybraną. **Wybieraj po porównaniu z CAD lub
pomiarami kontrolnymi** — w D500 pętle (`posegraf.py`) są wyłączone, bo tam pogarszały wynik.

Zmienne: `L2_PETLE=0` wyłącza, `L2_ZOSTAW_BAG=1` zostawia `pointlio_wynik.bag`.
Ręcznie: `python3 l2_petle.py pointlio_wynik.bag wyjscie/`.

Test na symulacji (pierścień korytarzy 40×24 m, dryf 2°/min): błąd max **150 → 21 cm**,
na końcu trasy 126 → 9 cm; bez dryfu i na prostej trasie mapa się nie zmienia.

## Kilka kondygnacji w jednym nagraniu i skosy (od 28.11.2026)

Można przejść cały dom (parter → schody → poddasze) **jednym nagraniem**, bez zatrzymywania.
- `l2_trasa.py` zapisuje trasę Point-LIO (`trasa.csv`: czas, x, y, z głowicy). `l2_mapa.sh` robi to samo i przekazuje trasę do `l2_rzut.py --trasa`.
- **Kondygnacje** to poziomy, na których chodził operator (co najmniej 1,5 m od siebie).
  - Podłoga: najwyższa duża płaszczyzna 1,0–2,7 m pod głowicą. Niższa płaszczyzna to sufit kondygnacji poniżej.
  - Sufit: najniższa duża płaszczyzna nad głowicą (pod stropem). Na ostatniej kondygnacji: najwyższa.
  - Rzuty kondygnacji leżą obok siebie na jednym arkuszu (odstęp 5 m), w tej samej skali. Otwory mają w CSV kolumnę `kondygnacja`.
- **Skosy** (najwyższa kondygnacja, gdy co najmniej 8 % rzutu jest niższe od płaskiego sufitu o 30 cm):
  - mapa wysokości pod dachem (siatka 10 cm, 75. percentyl punktów);
  - warstwy `SKOS_1_40` i `SKOS_2_20` (linie h = 1,40 i 2,20 m);
  - powierzchnia wg PN-ISO 9836: ≥ 2,20 m liczona w 100 %, 1,40–2,20 m w 50 %, < 1,40 m pomijana;
  - kubatura; wyniki w `skosy.json` i `wysokosci.txt`.
  - Pas ścian na poddaszu: 1,0–1,4 m nad podłogą, bo pod skosem pas pod sufitem byłby dachem.
- Bez trasy (stare nagrania, `L2_PETLE=0`) działa jak dawniej: jedna kondygnacja.
- Test (sztuczny dom 10 × 8 m, parter 2,70 m, poddasze ze ścianką kolankową 1,0 m i dachem 40°, jętki 2,60 m):
  - podłogi 0,00 / 2,99 m (wzorzec 0 / 3,0 m);
  - powierzchnia ≥ 2,20 m: 51,67 m² (wzorzec 51,40);
  - 1,40–2,20 m: 19,28 m² (wzorzec 19,07);
  - PN-ISO 9836: 61,31 m² (wzorzec 60,93; +0,6 %);
  - kubatura: 179,0 m³ (wzorzec 177,5).
- **Jak chodzić:**
  - start i koniec w tym samym miejscu;
  - schody powoli, bez obracania się na stopniu;
  - głowica z dala od stropu nad schodami i od dachu pod skosem;
  - w każdym pokoju „Jestem tu”.

## Szafy i meble (od 28.11.2026)

- **Ściany** w rzucie L2 to pionowe powierzchnie, które dochodzą do sufitu albo dachu nad nimi:
  - mają punkty w pasie 5–25 cm pod lokalnym sufitem;
  - mają ciągłość w dół: punkty 30–70 cm pod sufitem.
  - Szafy, regały i komody mają u góry szczelinę, nad którą skaner widzi sufit, więc nie są ścianami. Zostają tylko na warstwie CIECIE_1M jako meble.
  - Lampy i kanały pod sufitem nie mają ciągłości w dół, więc też nie są ścianami.
  - Ścianka kolankowa dochodzi do dachu, więc jest ścianą.
  - Mebel zabudowany do 5 cm pod sufit będzie wyglądał jak ściana.
- **Wysokość pod sufitem lub dachem** (`mapa_sufitu`):
  - komórki 15 cm; mediana najwyższej warstwy punktów (15 cm pod drugim najwyższym punktem);
  - góra szafy pod widocznym sufitem nie jest brana za sufit;
  - na tym liczą się też skosy.
- Test (ten sam sztuczny dom; szafa 2,35 m w pokoju 2,70 m, regał 2,0 m, szafa 1,9 m na poddaszu, komoda 0,9 m pod skosem):
  - ściany bez mebli;
  - powierzchnia PN-ISO 9836 61,11 m² z meblami i bez (wzorzec 60,93);
  - kubatura 178,0 m³ (wzorzec 177,5).

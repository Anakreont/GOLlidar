# Skaner 2D na Raspberry Pi 3B+ — instrukcja

## Podłączenie

- **LiDAR D500**: przez dołączoną płytkę-adapter USB do portu USB w Raspberry Pi. Pojawi się jako `/dev/ttyUSB0`.
- **IMU BNO085 (Adafruit)** w trybie UART-RVC, przez konwerter USB-UART (np. CH343):
  - na płytce BNO085 połącz pin **P0 (PS0) z 3V** — to włącza tryb UART-RVC (z tyłu płytki jest też zworka do zalutowania),
  - **VIN → 3V3** konwertera, **GND → GND**, **SDA (wyjście danych w tym trybie) → RX** konwertera,
  - konwerter ustaw na 3,3 V. Pojawi się jako `/dev/ttyUSB1` albo `/dev/ttyACM0`.
- **Barometr BMP280 lub BME280 (opcjonalnie, do automatycznego rozróżniania kondygnacji)** — wprost do złącza GPIO Raspberry Pi, cztery przewody:
  - VCC → pin 1 (3,3 V), GND → pin 6, SDA → pin 3 (GPIO2), SCL → pin 5 (GPIO3),
  - włącz I2C: `sudo raspi-config` → Interface Options → I2C, potem `sudo apt install -y python3-smbus2 i2c-tools`,
  - sprawdź adres: `i2cdetect -y 1` powinno pokazać `76` albo `77`,
  - nagrywanie z barometrem: `python3 skan_log.py szkola --baro` (dla adresu 0x77 dodaj `--baro-adres 0x77`).
  - Barometr może leżeć w plecaku przy Raspberry Pi; ważne, żeby nie był w przeciągu ani na słońcu.
- **Zasilanie**: Raspberry Pi z powerbanku (wyjście min. 2,4 A). Drugi powerbank jako zapas.
- IMU przymocuj sztywno do tej samej płytki co LiDAR, płasko, osią Z do góry.

Uwaga: opis pinów BNO085 podaję z pamięci — przed lutowaniem porównaj z opisem trybu „UART-RVC" na stronie produktu Adafruit.

## Instalacja (raz)

```bash
sudo apt update && sudo apt install -y python3-serial python3-matplotlib
mkdir -p ~/skaner ~/skany
# skopiuj skan_log.py i sprawdz.py do ~/skaner
sudo usermod -aG dialout $USER   # potem wyloguj i zaloguj się ponownie
```

## Podgląd i sterowanie z telefonu (zalecane)

```bash
cd ~/skaner
python3 skan_web.py                      # LiDAR /dev/ttyUSB0, IMU /dev/ttyACM0, barometr 0x76
python3 skan_web.py --bez-imu --bez-baro # sam LiDAR
```

W przeglądarce telefonu (ta sama sieć lub hotspot): `http://<adres-IP-raspberry>:8080`.
Strona pokazuje bieżący obrót LiDAR-u (kontur pomieszczenia wokół Ciebie), przechył, wysokość względną,
stan zapisu i wolne miejsce. Przyciski: START / STOP, PAUZA, ZNACZNIK (np. numer sali), Zeruj wysokość, Wyłącz RPi.

**PAUZA** – naciśnij, zanim się schylisz, kucniesz albo odstawisz czujnik inaczej niż pionowo; po powrocie do
normalnej pozycji naciśnij WZNÓW. Surowe dane zapisują się dalej, ale odcinek jest oznaczony do pominięcia.
Wznawiaj mniej więcej w tym samym miejscu i zwrócony w tę samą stronę, w której zacząłeś pauzę.
Przechył powyżej 15° jest oznaczany automatycznie (na kafelku „Zapis" pojawia się „PRZECHYŁ!").

**ZAPISZ POMIESZCZENIE** – inwentaryzacja oświetlenia: nazwa lub numer pomieszczenia, rodzaj oprawy (świetlówki, LED,
żarówki, inne), liczba punktów świetlnych w oprawie, ilość opraw i uwagi (np. moc). Każdy zapis trafia do `pomieszczenia.csv` w katalogu sesji (otwiera się w Excelu) i jednocześnie
stawia znacznik czasu, więc pomieszczenie zostanie podpisane na rzucie. Kilka typów opraw w jednej sali zapisz jako
osobne wiersze z tą samą nazwą pomieszczenia.

Autostart po włączeniu zasilania (wtedy SSH w terenie nie jest potrzebne):

```bash
sudo cp ~/skaner/skaner.service /etc/systemd/system/
sudo systemctl enable --now skaner
```

Plik `skaner.service` zakłada użytkownika `anakreont` i katalog `/home/anakreont/skaner` — przy innym loginie popraw trzy linie w pliku.
Inne porty lub opcje dopisz w linii `ExecStart`.

## Sterowanie przez SSH (wariant zapasowy)

Raspberry Pi łączy się z hotspotem telefonu; z telefonu logujesz się przez SSH (np. aplikacja Termux lub JuiceSSH).

```bash
ls /dev/ttyUSB* /dev/ttyACM*            # sprawdź, które porty są widoczne
cd ~/skaner
python3 skan_log.py parter_A            # start nagrania; Ctrl+C kończy
python3 skan_log.py test --bez-imu      # wariant bez IMU
python3 sprawdz.py ~/skany/<katalog> --png   # kontrola po nagraniu
```

W trakcie nagrania co 2 s widać przepływ danych. D500 powinien dawać ok. 17–18 kB/s, IMU ok. 1,9 kB/s. Napis „BRAK DANYCH!" oznacza zły port lub kabel.

Enter w trakcie nagrania zapisuje znacznik czasu — możesz wpisać numer sali przed Enterem.

Żeby nagranie nie przerwało się po zerwaniu SSH, uruchamiaj w `tmux` (`sudo apt install tmux`, potem `tmux`, a po ponownym połączeniu `tmux attach`).

## Inne czujniki

| Czujnik | Parametr |
|---|---|
| D500 / LD19 | `--lidar-baud 230400` (domyślnie) |
| STL27L | `--lidar-baud 921600` |
| RPLIDAR C1 | `--lidar-baud 460800` (wymaga komendy startu — daj znać, dopiszę) |

## Jak chodzić

1. Czujnik na ok. 1,9 m, tyczka pionowo, tempo spacerowe wolne (ok. 0,5 m/s).
2. Bez barometru: każda kondygnacja to osobne nagranie. Z barometrem (`--baro`): można nagrywać ciągle, kondygnacje zostaną rozdzielone po wysokości, a odcinki na schodach odrzucone. W obu przypadkach każdą kondygnację zaczynaj i kończ na tym samym spoczniku klatki schodowej.
3. Przed startem otwórz drzwi do sal. W każdej sali dojdź do środka i stań na 2–3 s.
4. Na początku i końcu nagrania postój 5 s bez ruchu.
5. Sala gimnastyczna: obejdź dookoła 3–4 m od ścian.
6. Na każdej kondygnacji zmierz dalmierzem 3–4 długie wymiary kontrolne i zapisz, gdzie.

## Co podesłać

Cały katalog sesji z `~/skany/` (pliki `lidar.bin`, `imu.bin`, `info.txt`, `znaczniki.txt`). Godzina nagrania to ok. 65 MB.

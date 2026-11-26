#!/usr/bin/env python3
"""
skan_log.py - rejestrator surowych danych z LiDAR-u 2D (D500 / LD19 / STL27L / RPLIDAR)
i opcjonalnie z IMU BNO085 (tryb UART-RVC) na Raspberry Pi.

Zapisuje surowe bajty z portow szeregowych ze znacznikami czasu - bez dekodowania,
wiec dziala niezaleznie od protokolu czujnika. Mape liczy sie pozniej z tych plikow.

Uzycie:
    python3 skan_log.py parter_skrzydlo_A
    python3 skan_log.py pietro1 --lidar /dev/ttyUSB0 --imu /dev/ttyUSB1
    python3 skan_log.py test --bez-imu

Zatrzymanie: Ctrl+C (albo `sudo systemctl stop skan`).
Pliki laduja w ~/skany/<data>_<nazwa>/ :
    lidar.bin, imu.bin  - rekordy: [czas_ns uint64][dlugosc uint16][dane]
    info.txt            - parametry sesji
    znaczniki.txt       - znaczniki wpisywane z klawiatury (Enter = "jestem w nowej sali")
"""
import argparse, os, struct, sys, threading, time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("Brak pyserial. Zainstaluj: sudo apt install python3-serial")

STOP = threading.Event()
LICZNIK = {}


def loguj_port(nazwa, port, baud, sciezka, shtp=False):
    """Czyta port szeregowy i dopisuje rekordy do pliku binarnego. shtp: BNO085 w trybie UART-SHTP -
    po otwarciu portu (i co 3 s bez danych) wysyla konfiguracje raportow."""
    LICZNIK[nazwa] = 0
    while not STOP.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.05) as s, open(sciezka, "ab") as f:
                ost = 0.0
                while not STOP.is_set():
                    if shtp and time.time() - ost > 3.0:
                        import imu_shtp
                        imu_shtp.wyslij_konfiguracje(s); ost = time.time()
                    dane = s.read(4096)
                    if dane:
                        ost = time.time()
                        f.write(struct.pack("<QH", time.time_ns(), len(dane)))
                        f.write(dane)
                        LICZNIK[nazwa] += len(dane)
        except (serial.SerialException, OSError) as e:
            print(f"\n[{nazwa}] problem z portem {port}: {e} - ponawiam za 1 s", flush=True)
            time.sleep(1)


def loguj_baro(adres, sciezka, bus_nr=1):
    """Barometr BMP280/BME280 na I2C Raspberry Pi: zapisuje surowe rejestry (10 Hz).
    Pierwszy rekord: b'CAL' + id ukladu + 24 bajty kalibracji; kolejne: 6 bajtow pomiaru."""
    LICZNIK["baro"] = 0
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            print("\n[baro] brak biblioteki smbus2 (sudo apt install python3-smbus2) - pomijam barometr")
            return
    while not STOP.is_set():
        try:
            with SMBus(bus_nr) as bus, open(sciezka, "ab") as f:
                chip = bus.read_byte_data(adres, 0xD0)
                kal = bytes(bus.read_i2c_block_data(adres, 0x88, 24))
                bus.write_byte_data(adres, 0xF5, 0x10)   # filtr IIR x16
                bus.write_byte_data(adres, 0xF4, 0x57)   # temp x2, cisnienie x16, tryb ciagly
                rek = b"CAL" + bytes([chip]) + kal
                f.write(struct.pack("<QH", time.time_ns(), len(rek)) + rek)
                while not STOP.is_set():
                    d = bytes(bus.read_i2c_block_data(adres, 0xF7, 6))
                    f.write(struct.pack("<QH", time.time_ns(), 6) + d)
                    LICZNIK["baro"] += 6 * 20      # skala tylko dla wskaznika "OK"
                    time.sleep(0.1)
        except OSError as e:
            print(f"\n[baro] problem z I2C (adres 0x{adres:02x}): {e} - ponawiam za 2 s", flush=True)
            time.sleep(2)


def main():
    ap = argparse.ArgumentParser(description="Rejestrator LiDAR 2D + IMU + barometr")
    ap.add_argument("--baro", action="store_true", help="loguj barometr BMP280/BME280 na I2C (wykrywanie kondygnacji)")
    ap.add_argument("--baro-adres", type=lambda x: int(x, 0), default=0x76, help="0x76 (domyslnie) lub 0x77")
    ap.add_argument("nazwa", help="nazwa sesji, np. parter_skrzydlo_A")
    ap.add_argument("--lidar", default="/dev/ttyUSB0")
    ap.add_argument("--lidar-baud", type=int, default=230400,
                    help="D500/LD19: 230400, STL27L: 921600, RPLIDAR C1: 460800, A1: 115200")
    ap.add_argument("--imu", default="/dev/ttyUSB1")
    ap.add_argument("--imu-baud", type=int, default=115200, help="BNO085 UART-RVC: 115200")
    ap.add_argument("--bez-imu", action="store_true")
    ap.add_argument("--imu-shtp", action="store_true", help="BNO085 w trybie UART-SHTP (mostek P1, 3 Mbps, surowy zyroskop)")
    ap.add_argument("--katalog", default=os.path.expanduser("~/skany"))
    a = ap.parse_args()
    if a.imu_shtp:
        a.imu_baud = 3000000

    kat = os.path.join(a.katalog, datetime.now().strftime("%Y%m%d_%H%M%S_") + a.nazwa)
    os.makedirs(kat, exist_ok=True)
    with open(os.path.join(kat, "info.txt"), "w") as f:
        f.write(f"nazwa={a.nazwa}\nstart={datetime.now().isoformat()}\n"
                f"lidar={a.lidar}@{a.lidar_baud}\n"
                f"imu={'brak' if a.bez_imu else a.imu + '@' + str(a.imu_baud)}\n"
                f"imu_tryb={'shtp' if a.imu_shtp else 'rvc'}\n")

    watki = [threading.Thread(target=loguj_port, daemon=True,
                              args=("lidar", a.lidar, a.lidar_baud, os.path.join(kat, "lidar.bin")))]
    if not a.bez_imu:
        watki.append(threading.Thread(target=loguj_port, daemon=True,
                                      args=("imu", a.imu, a.imu_baud, os.path.join(kat, "imu.bin"), a.imu_shtp)))
    if a.baro:
        watki.append(threading.Thread(target=loguj_baro, daemon=True,
                                      args=(a.baro_adres, os.path.join(kat, "baro.bin"))))
    for w in watki:
        w.start()

    def znaczniki():
        # Enter (z opcjonalnym opisem) zapisuje znacznik czasu - przydatne przy wejsciu do sali
        with open(os.path.join(kat, "znaczniki.txt"), "a") as f:
            while not STOP.is_set():
                try:
                    opis = input()
                except EOFError:
                    return
                f.write(f"{time.time_ns()}\t{opis}\n")
                f.flush()
                print(f"  znacznik: {opis or '(bez opisu)'}")

    if sys.stdin.isatty():
        threading.Thread(target=znaczniki, daemon=True).start()

    print(f"Zapis do {kat}\nEnter = znacznik (mozna wpisac np. numer sali). Ctrl+C = koniec.")
    t0 = time.time()
    poprzedni = {}
    try:
        while True:
            time.sleep(2)
            czesci = []
            for k, v in LICZNIK.items():
                przyrost = (v - poprzedni.get(k, 0)) / 2
                poprzedni[k] = v
                stan = "OK" if przyrost > 100 else "BRAK DANYCH!"
                czesci.append(f"{k}: {przyrost / 1000:5.1f} kB/s {stan}")
            print(f"\r{int(time.time() - t0):5d} s | " + " | ".join(czesci) + "   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    STOP.set()
    time.sleep(0.3)
    with open(os.path.join(kat, "info.txt"), "a") as f:
        f.write(f"koniec={datetime.now().isoformat()}\n" +
                "".join(f"{k}_bajty={v}\n" for k, v in LICZNIK.items()))
    print(f"\nKoniec. Zapisano: " + ", ".join(f"{k} {v / 1e6:.2f} MB" for k, v in LICZNIK.items()))


if __name__ == "__main__":
    main()

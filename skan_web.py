#!/usr/bin/env python3
"""
skan_web.py - rejestrator LiDAR 2D (D500 / LD19 / STL-19P) + IMU BNO085 (UART-RVC) + barometr BMP280
z podgladem i sterowaniem w przegladarce telefonu.

Uruchomienie na Raspberry Pi:
    python3 skan_web.py                       # LiDAR /dev/ttyUSB0, IMU /dev/ttyACM0, barometr 0x76
    python3 skan_web.py --imu /dev/ttyUSB1 --baro-adres 0x77
    python3 skan_web.py --bez-imu --bez-baro

Potem w telefonie (ta sama siec / hotspot):  http://<adres-IP-raspberry>:8080

Na stronie: podglad biezacego obrotu LiDAR-u, przechyl, wysokosc, stan zapisu, przyciski
Start / Stop / Znacznik. Dane zapisywane sa w tym samym formacie co skan_log.py:
~/skany/<data>_<nazwa>/{lidar.bin,imu.bin,baro.bin,info.txt,znaczniki.txt}
"""
import argparse, collections, json, math, os, re, shutil, struct, subprocess, sys, threading, time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import serial
except ImportError:
    sys.exit("Brak pyserial. Zainstaluj: sudo apt install python3-serial")

# ----------------------------------------------------------------------------- stan wspolny
LOCK = threading.Lock()
STAN = {
    "nagrywa": False, "nazwa": "", "katalog": "", "t_start": 0.0,
    "bajty": {"lidar": 0, "lidar2": 0, "imu": 0, "baro": 0},
    "ost": {"lidar": 0.0, "lidar2": 0.0, "imu": 0.0, "baro": 0.0},        # czas ostatnich danych
    "podglad": "lidar",        # ktory LiDAR jest poziomy (podglad na stronie, przekroje); drugi jest pionowy
    "foto": 0,                 # liczba zdjec w biezacej sesji
    "obrot": [], "hz": 0.0, "pkt_obrot": 0,
    "yaw": None, "pitch": None, "roll": None, "max_przechyl": 0.0,
    "wys": None, "p0": None, "temp": None,
    "znaczniki": 0, "komunikat": "",
    "pauza": False, "auto_pauza": False,
    "pom": [],                 # pomieszczenia zapisane w biezacej sesji
    "budynek": "", "kondygnacja": "",
    "grz": [],                 # grzejniki zapisane w biezacej sesji
    "okn": [],                 # okna i drzwi zewnetrzne zapisane w biezacej sesji
    "przechyl": None,          # kat odchylenia od pionu [st.]
    "prz_faza": 0,             # 0 brak, 1 trzymaj nieruchomo, 2 przekroj zapisany (postaw do pionu)
    "prz_zostalo": 0.0,        # sekundy do zapisania przekroju
    "prz_nr": 0,               # liczba przekrojow w sesji
    "prz_wynik": "",           # wymiary ostatniego przekroju (szacunek na zywo)
    "prz_skany": [],           # obroty zebrane podczas trzymania
    "pion0": (0.0, 0.0),       # pitch, roll zmierzone przy tyczce w pionie (montaz BNO wzgledem LiDAR-u)
    "ip": "",
}
PLIK_PION0 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_zero.json")
IMU_HIST = collections.deque(maxlen=200)   # ostatnie 2 s (pitch, roll) do zerowania pionu


def wczytaj_pion0():
    try:
        with open(PLIK_PION0) as f:
            d = json.load(f)
        STAN["pion0"] = (float(d["pitch0"]), float(d["roll0"]))
    except (OSError, ValueError, KeyError):
        pass


def zeruj_pion():
    """Zapamietuje srednie pitch/roll z ostatnich 2 s jako pion; surowe dane w imu.bin zostaja bez zmian."""
    with LOCK:
        h = list(IMU_HIST)
    if len(h) < 100:
        return None, "Brak danych z IMU"
    p = [x[0] for x in h]; r = [x[1] for x in h]
    if max(p) - min(p) > 1.5 or max(r) - min(r) > 1.5:
        return None, "Za duży ruch – postaw tyczkę pionowo i nie ruszaj 2 s"
    p0, r0 = sum(p) / len(p), sum(r) / len(r)
    if math.hypot(p0, r0) > 20:
        return None, f"Odchylenie {math.hypot(p0, r0):.0f}° – za duże jak na pion, sprawdź montaż"
    with LOCK:
        STAN["pion0"] = (p0, r0)
    with open(PLIK_PION0, "w") as f:
        json.dump({"pitch0": round(p0, 2), "roll0": round(r0, 2), "kiedy": datetime.now().isoformat()}, f)
    znacznik(f"IMU_PION0 pitch0={p0:.2f} roll0={r0:.2f}", licz=False)
    return (p0, r0), None


def adres_ip():
    while not KONIEC.is_set():
        try:
            ip = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=3).stdout.split()
            STAN["ip"] = " ".join(a for a in ip if ":" not in a)
        except (OSError, subprocess.SubprocessError):
            pass
        KONIEC.wait(10)
PROG_PRZECHYLU = 15.0          # stopnie; powyzej tego skany sa oznaczane jako do pominiecia
PRZ_WEJSCIE, PRZ_WYJSCIE = 60.0, 45.0   # przekroj pionowy: glowica polozona (60-125 st.), koniec ponizej 45 st.
PRZ_MAX = 125.0
PRZ_CZAS = 3.0                 # tyle sekund nieruchomo i przekroj jest zapisany
PRZ_DRGANIA = 6.0              # dopuszczalna zmiana przechylu w czasie trzymania [st.]
PLIKI = {}                      # nazwa strumienia -> otwarty plik (gdy nagrywa)
KONIEC = threading.Event()
ARGS = None

CRC_TAB = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c << 1) ^ 0x4D) & 0xFF if _c & 0x80 else (_c << 1) & 0xFF
    CRC_TAB.append(_c)


def zapisz(strumien, dane, t=None):
    """Dopisuje rekord [czas_ns][dlugosc][dane] jesli trwa nagrywanie."""
    with LOCK:
        f = PLIKI.get(strumien)
        if f is not None:
            f.write(struct.pack("<QH", t or time.time_ns(), len(dane)))
            f.write(dane)
            STAN["bajty"][strumien] += len(dane)
        STAN["ost"][strumien] = time.time()


# ----------------------------------------------------------------------------- LiDAR
def watek_lidar(port, baud, nazwa="lidar"):
    buf = bytearray()
    obrot, ost_kat, t_obr = [], None, time.time()
    while not KONIEC.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.05) as s:
                if STAN["podglad"] == nazwa:
                    STAN["komunikat"] = ""
                while not KONIEC.is_set():
                    d = s.read(4096)
                    if not d:
                        continue
                    zapisz(nazwa, d)
                    buf += d
                    while len(buf) >= 47:
                        if buf[0] != 0x54 or buf[1] != 0x2C:
                            del buf[0]
                            continue
                        crc = 0
                        for b in buf[:46]:
                            crc = CRC_TAB[crc ^ b]
                        if crc != buf[46]:
                            del buf[0]
                            continue
                        p = bytes(buf[:47])
                        del buf[:47]
                        start = struct.unpack_from("<H", p, 4)[0] / 100.0
                        kon = struct.unpack_from("<H", p, 42)[0] / 100.0
                        krok = ((kon - start) % 360) / 11
                        for i in range(12):
                            odl, inten = struct.unpack_from("<HB", p, 6 + 3 * i)
                            kat = (start + krok * i) % 360
                            if ost_kat is not None and kat < ost_kat - 180:
                                teraz = time.time()
                                with LOCK:
                                    if STAN["podglad"] == nazwa:
                                        if STAN["prz_faza"] == 1 and len(STAN["prz_skany"]) < 40:
                                            STAN["prz_skany"].append(obrot)
                                        STAN["obrot"] = obrot
                                        STAN["pkt_obrot"] = len(obrot)
                                        STAN["hz"] = 1.0 / max(teraz - t_obr, 1e-3)
                                t_obr, obrot = teraz, []
                            ost_kat = kat
                            if odl > 30:
                                obrot.append((round(kat, 1), odl))
        except (serial.SerialException, OSError) as e:
            if STAN["podglad"] == nazwa:
                STAN["komunikat"] = f"LiDAR: {e}"
            time.sleep(1)


# ----------------------------------------------------------------------------- przekroj pionowy
def wymiary_przekroju(skany):
    """Szacuje dwa wymiary prostokata (np. wysokosc i szerokosc sali) z obrotow zebranych przy polozonej glowicy.
    Zwraca (a, b) w metrach, mniejszy pierwszy; None gdy nie widac pary przeciwleglych plaszczyzn."""
    pkt = []
    for ob in skany[-12:]:
        for kat, odl in ob:
            if 150 < odl < 12000:
                a = math.radians(kat)
                pkt.append((odl / 1000 * math.sin(a), odl / 1000 * math.cos(a)))
    if len(pkt) < 150:
        return None, None
    pkt = pkt[::max(1, len(pkt) // 700)]
    BIN = 0.04
    najl = (-1, 0, None, None)
    for st in range(0, 90):
        c, s_ = math.cos(math.radians(st)), math.sin(math.radians(st))
        hu, hv = {}, {}
        for x, y in pkt:
            u, v = int((x * c + y * s_) // BIN), int((-x * s_ + y * c) // BIN)
            hu[u] = hu.get(u, 0) + 1
            hv[v] = hv.get(v, 0) + 1
        ocena = sum(n * n for n in hu.values()) + sum(n * n for n in hv.values())
        if ocena > najl[0]:
            najl = (ocena, st, hu, hv)
    prog = max(6, len(pkt) * 0.03)

    def rozpietosc(h):
        uj = [(n + h.get(k - 1, 0) + h.get(k + 1, 0), k) for k, n in h.items() if k < -3]
        do = [(n + h.get(k - 1, 0) + h.get(k + 1, 0), k) for k, n in h.items() if k > 3]
        if not uj or not do:
            return None
        (n1, k1), (n2, k2) = max(uj), max(do)
        if n1 < prog or n2 < prog:
            return None
        return (k2 - k1) * BIN
    w = sorted(x for x in (rozpietosc(najl[2]), rozpietosc(najl[3])) if x)
    return (w + [None, None])[:2]


class Przekroj:
    """Automat: glowica polozona i nieruchoma przez PRZ_CZAS -> przekroj zapisany. Zwraca zdarzenia do znacznikow."""

    def __init__(self):
        self.faza, self.t0, self.okno = 0, 0.0, []

    def krok(self, t, kat):
        zd = []
        if self.faza == 0:
            if PRZ_WEJSCIE <= kat <= PRZ_MAX:
                self.faza, self.t0, self.okno = 1, t, [(t, kat)]
                zd.append("START")
        elif kat < PRZ_WYJSCIE:
            zd.append("KONIEC" if self.faza == 2 else "PRZERWANY")
            self.faza = 0
        elif self.faza == 1:
            self.okno = [(a, b) for a, b in self.okno if t - a <= 1.0] + [(t, kat)]
            k = [b for _, b in self.okno]
            if max(k) - min(k) > PRZ_DRGANIA or not (PRZ_WEJSCIE <= kat <= PRZ_MAX):
                self.t0, self.okno = t, [(t, kat)]
                zd.append("OD_NOWA")
            elif t - self.t0 >= PRZ_CZAS:
                self.faza = 2
                zd.append("OK")
        return zd

    def zostalo(self, t):
        return max(0.0, PRZ_CZAS - (t - self.t0)) if self.faza == 1 else 0.0


def policz_przekroj(nr, skany):
    a, b = wymiary_przekroju(skany)
    wynik = " × ".join(f"{x:.2f}".replace(".", ",") for x in (a, b) if x) + (" m" if a else "")
    with LOCK:
        if STAN["prz_nr"] == nr:
            STAN["prz_wynik"] = wynik or "brak wymiarów na żywo"
    znacznik(f"PRZEKROJ_WYNIK nr={nr} a={a or ''} b={b or ''}", licz=False)


# ----------------------------------------------------------------------------- IMU (BNO085 UART-RVC)
def imu_probka(yaw, pitch, roll, prz):
    """Jedna probka orientacji (stopnie) - wspolne dla trybu RVC i SHTP: stan strony, auto-pauza, przekroje."""
    with LOCK:
        STAN["yaw"], STAN["pitch"], STAN["roll"] = yaw, pitch, roll
        IMU_HIST.append((pitch, roll))
        p0, r0 = STAN["pion0"]
        m = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(pitch - p0)) *
                                                     math.cos(math.radians(roll - r0))))))
        STAN["przechyl"] = m
        if STAN["nagrywa"]:
            STAN["max_przechyl"] = max(STAN["max_przechyl"], m)
        zmiana = None
        if m > PROG_PRZECHYLU and not STAN["auto_pauza"]:
            STAN["auto_pauza"], zmiana = True, "AUTO_PRZECHYL_START"
        elif m < PROG_PRZECHYLU - 3 and STAN["auto_pauza"]:
            STAN["auto_pauza"], zmiana = False, "AUTO_PRZECHYL_KONIEC"
        teraz = time.time()
        zdarzenia = prz.krok(teraz, m) if STAN["nagrywa"] else []   # przekroj takze w pauzie (zdjety plecak)
        if not STAN["nagrywa"] and prz.faza:
            prz.faza = 0
        skany, nr = None, 0
        for z in zdarzenia:
            if z in ("START", "OD_NOWA"):
                STAN["prz_skany"] = []
                STAN["prz_wynik"] = ""
            elif z == "OK":
                STAN["prz_nr"] += 1
                nr, skany = STAN["prz_nr"], STAN["prz_skany"]
                STAN["prz_skany"] = []
        STAN["prz_faza"], STAN["prz_zostalo"] = prz.faza, prz.zostalo(teraz)
    if zmiana:
        znacznik(zmiana, licz=False)
    for z in zdarzenia:
        if z == "OK":
            znacznik(f"PRZEKROJ_OK nr={nr} przechyl={m:.0f}", licz=False)
            threading.Thread(target=policz_przekroj, args=(nr, skany), daemon=True).start()
        elif z != "OD_NOWA":
            znacznik(f"PRZEKROJ_{z} przechyl={m:.0f}", licz=False)


def watek_imu(port, baud):
    """BNO085: tryb wybierany zworka na plytce. --imu-tryb auto sprawdza najpierw UART-SHTP (3 Mbps, surowy
    zyroskop), a gdy czujnik nie odpowiada - UART-RVC (115200)."""
    prz = Przekroj()
    tryb = ARGS.imu_tryb
    while not KONIEC.is_set():
        if tryb in ("auto", "shtp"):
            ok = _imu_shtp(port, prz, probny=(tryb == "auto"))
            if ok:
                tryb = "shtp"
                continue
            if tryb == "shtp":
                time.sleep(1)
                continue
        if not _imu_rvc(port, baud, prz) and ARGS.imu_tryb == "auto":
            tryb = "auto"                       # brak ramek RVC przez 3 s - moze czujnik jednak w SHTP


def _imu_rvc(port, baud, prz):
    """Zwraca False po 3 s bez poprawnej ramki RVC."""
    buf = bytearray()
    try:
        with serial.Serial(port, baud, timeout=0.05) as s:
            ost = time.time()
            while not KONIEC.is_set():
                d = s.read(1024)
                if time.time() - ost > 3.0:
                    return False
                if not d:
                    continue
                zapisz("imu", d)
                buf += d
                while len(buf) >= 19:
                    if buf[0] != 0xAA or buf[1] != 0xAA:
                        del buf[0]
                        continue
                    if sum(buf[2:18]) & 0xFF != buf[18]:
                        del buf[0]
                        continue
                    yaw, pitch, roll = struct.unpack_from("<hhh", buf, 3)
                    del buf[:19]
                    ost = time.time(); STAN["imu_tryb"] = "rvc"
                    imu_probka(yaw / 100, pitch / 100, roll / 100, prz)
    except (serial.SerialException, OSError):
        time.sleep(1)
    return True


def _imu_shtp(port, prz, probny=False):
    """Zwraca False, gdy czujnik nie odpowiada w trybie SHTP (wtedy probujemy RVC)."""
    import imu_shtp as H
    st = H.Strumien(); byly = False
    try:
        with serial.Serial(port, H.BAUD, timeout=0.05) as s:
            H.wyslij_konfiguracje(s)
            ost = time.time(); seq = 3; n = 0
            while not KONIEC.is_set():
                d = s.read(4096)
                teraz = time.time()
                if d and byly:
                    zapisz("imu", d)
                for _, kan, tr in st.feed(d or b""):
                    if kan != 3:
                        continue
                    for rid, _, _, w in H.raporty(tr):
                        if rid != H.GRV:
                            continue
                        if not byly:
                            byly = True
                            STAN["imu_tryb"] = "shtp"
                            zapisz("imu", d)
                        ost = teraz; n += 1
                        if n % 2 == 0:            # 200 Hz -> 100 Hz dla strony i historii pionu
                            imu_probka(*H.katy(w), prz)
                if teraz - ost > 2.0:
                    if probny and not byly:
                        return False
                    H.wyslij_konfiguracje(s, seq)    # czujnik po resecie (np. chwilowy zanik zasilania)
                    seq += 3; ost = teraz
        return byly
    except (serial.SerialException, OSError):
        time.sleep(1)
        return byly


# ----------------------------------------------------------------------------- barometr BMP280
def bmp280(kal, d):
    T1, T2, T3 = struct.unpack_from("<Hhh", kal, 0)
    P = struct.unpack_from("<Hhhhhhhhh", kal, 6)
    adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
    adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
    v1 = (adc_t / 16384.0 - T1 / 1024.0) * T2
    v2 = (adc_t / 131072.0 - T1 / 8192.0) ** 2 * T3
    tf = v1 + v2
    v1 = tf / 2.0 - 64000.0
    v2 = v1 * v1 * P[5] / 32768.0 + v1 * P[4] * 2.0
    v2 = v2 / 4.0 + P[3] * 65536.0
    v1 = (P[2] * v1 * v1 / 524288.0 + P[1] * v1) / 524288.0
    v1 = (1.0 + v1 / 32768.0) * P[0]
    if v1 == 0:
        return tf / 5120.0, 0.0
    p = 1048576.0 - adc_p
    p = (p - v2 / 4096.0) * 6250.0 / v1
    p = p + (P[8] * p * p / 2147483648.0 + p * P[7] / 32768.0 + P[6]) / 16.0
    return tf / 5120.0, p


def watek_baro(adres, bus_nr=1):
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            return
    filtr = None
    while not KONIEC.is_set():
        try:
            with SMBus(bus_nr) as bus:
                chip = bus.read_byte_data(adres, 0xD0)
                kal = bytes(bus.read_i2c_block_data(adres, 0x88, 24))
                bus.write_byte_data(adres, 0xF5, 0x10)
                bus.write_byte_data(adres, 0xF4, 0x57)
                STAN["_baro_kal"] = b"CAL" + bytes([chip]) + kal
                zapisz("baro", STAN["_baro_kal"])
                time.sleep(0.5)
                while not KONIEC.is_set():
                    d = bytes(bus.read_i2c_block_data(adres, 0xF7, 6))
                    zapisz("baro", d)
                    temp, p = bmp280(kal, d)
                    if p > 0:
                        filtr = p if filtr is None else 0.9 * filtr + 0.1 * p
                        with LOCK:
                            if STAN["p0"] is None:
                                STAN["p0"] = filtr
                            STAN["wys"] = 44330.0 * (1 - (filtr / STAN["p0"]) ** 0.1903)
                            STAN["temp"] = temp
                    time.sleep(0.1)
        except OSError:
            time.sleep(2)


# ----------------------------------------------------------------------------- sterowanie nagrywaniem
def czysc(t, n=200):
    return str(t).replace(";", ",").replace("\n", " ").replace("\r", " ").replace("\t", " ").strip()[:n]


def ustawienia(nowe=None):
    """Zapamietuje ostatni budynek i kondygnacje miedzy uruchomieniami."""
    p = os.path.join(ARGS.katalog, ".ustawienia")
    try:
        with open(p, encoding="utf-8") as f:
            u = json.load(f)
    except (OSError, ValueError):
        u = {}
    if nowe:
        u.update(nowe)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(u, f, ensure_ascii=False)
    return u


def start(budynek, kondygnacja):
    budynek, kondygnacja = czysc(budynek, 40), czysc(kondygnacja, 30)
    nazwa = "_".join(x for x in (budynek, kondygnacja) if x) or "sesja"
    nazwa = nazwa.translate(str.maketrans("ąćęłńóśźżĄĆĘŁŃÓŚŹŻ", "acelnoszzACELNOSZZ"))
    nazwa = "".join(ch if (ch.isascii() and ch.isalnum()) or ch in "-_+" else "_" for ch in nazwa)[:60]
    ustawienia({"budynek": budynek, "kondygnacja": kondygnacja})
    with LOCK:
        if STAN["nagrywa"]:
            return False
        kat = os.path.join(ARGS.katalog, datetime.now().strftime("%Y%m%d_%H%M%S_") + nazwa)
        os.makedirs(kat, exist_ok=True)
        with open(os.path.join(kat, "info.txt"), "w") as f:
            f.write(f"nazwa={nazwa}\nbudynek={budynek}\nkondygnacja={kondygnacja}\nstart={datetime.now().isoformat()}\n"
                    f"lidar={ARGS.lidar}@{ARGS.lidar_baud}\nimu={ARGS.imu}@{ARGS.imu_baud}\n"
                    f"baro=0x{ARGS.baro_adres:02x}\nlidar2={ARGS.lidar2 or ''}\nlidar_poziomy={STAN['podglad']}\n"
                    f"imu_pitch0={STAN['pion0'][0]:.2f}\nimu_roll0={STAN['pion0'][1]:.2f}\n"
                    f"imu_tryb={STAN.get('imu_tryb', '')}\n")
        for s in ("lidar", "imu", "baro") + (("lidar2",) if ARGS.lidar2 else ()):
            PLIKI[s] = open(os.path.join(kat, s + ".bin"), "ab")
            STAN["bajty"][s] = 0
        kal = STAN.get("_baro_kal")
        if kal:
            PLIKI["baro"].write(struct.pack("<QH", time.time_ns(), len(kal)) + kal)
        STAN.update(nagrywa=True, nazwa=nazwa, katalog=kat, t_start=time.time(), znaczniki=0, max_przechyl=0.0, pom=[], grz=[], okn=[], foto=0, prz_nr=0, prz_wynik="", prz_faza=0,
                    budynek=budynek, kondygnacja=kondygnacja)
    return True


def stop():
    with LOCK:
        if not STAN["nagrywa"]:
            return False
        for s in list(PLIKI):
            PLIKI.pop(s).close()
        STAN["nagrywa"] = False
        STAN["pauza"] = False
        with open(os.path.join(STAN["katalog"], "info.txt"), "a") as f:
            f.write(f"koniec={datetime.now().isoformat()}\n" +
                    "".join(f"{k}_bajty={v}\n" for k, v in STAN["bajty"].items()) +
                    f"max_przechyl={STAN['max_przechyl']:.1f}\n")
    return True


def znacznik(opis, licz=True):
    with LOCK:
        if not STAN["nagrywa"]:
            return False
        with open(os.path.join(STAN["katalog"], "znaczniki.txt"), "a") as f:
            f.write(f"{time.time_ns()}\t{opis}\t{STAN['wys'] if STAN['wys'] is not None else ''}\n")
        if licz:
            STAN["znaczniki"] += 1
    return True


KOL_POM = ("czas_ns;godzina;pomieszczenie;rodzaj_oprawy;punktow_w_oprawie;liczba_opraw;punktow_razem;uwagi;"
           "wysokosc_wzgl_m;budynek;kondygnacja;moc_punktu_W;moc_razem_W;wpis")
ZN_SYSTEMOWE = ("AUTO_", "PRZEKROJ_", "PAUZA_", "KONDYGNACJA ", "FOTO ", "LIDAR_POZIOMY ", "POM ", "GRZ ", "OKN ", "EDYCJA ", "USUNIETO ")
KOL_OKN = "czas_ns;godzina;pomieszczenie;typ;rodzaj;szer_cm;wys_cm;rama;szyby;sztuk;uwagi;wysokosc_wzgl_m;budynek;kondygnacja;wpis"
KOL_GRZ = "czas_ns;godzina;pomieszczenie;rodzaj;sztuk;zeberek;szerokosc_cm;dlugosc_cm;uwagi;wysokosc_wzgl_m;budynek;kondygnacja;wpis"


def _liczba(t):
    try:
        return float(str(t or 0).replace(",", "."))
    except ValueError:
        return 0.0


def wiersze_pom(kat):
    """Aktualny stan pomieszczen sesji. Plik jest tylko dopisywany: poprawka to nowy wiersz z tym samym czas_ns
    (wygrywa ostatni), usuniecie to wiersz z wpis=usuniety. Dzieki temu kopia na serwer zawsze sie zgadza."""
    try:
        with open(os.path.join(kat, "pomieszczenia.csv"), encoding="utf-8-sig") as f:
            linie = f.read().splitlines()[1:]
    except OSError:
        return []
    stan = {}
    for w in linie:
        k = (w.split(";") + [""] * 14)[:14]
        if not k[0].isdigit():
            continue
        if k[13] == "usuniety":
            stan.pop(k[0], None)
            continue
        poprz = stan.pop(k[0], None) if k[13] == "poprawka" and k[0] in stan else None
        rek = {"id": k[0], "godz": k[1], "nr": k[2], "typ": k[3], "punktow": int(_liczba(k[4])), "opraw": int(_liczba(k[5])),
               "uwagi": k[7], "wys": k[8], "bud": k[9], "kond": k[10], "moc": _liczba(k[11]), "moc_razem": _liczba(k[12])}
        if poprz is not None:                       # poprawka zostaje na miejscu pierwotnego wpisu
            stan = {**{a: b for a, b in stan.items() if int(a) < int(k[0])}, k[0]: rek,
                    **{a: b for a, b in stan.items() if int(a) > int(k[0])}}
        else:
            stan[k[0]] = rek
    return list(stan.values())


def wiersze_grz(kat):
    """Aktualny stan grzejnikow sesji (plik tylko dopisywany, zasady jak w wiersze_pom)."""
    try:
        with open(os.path.join(kat, "grzejniki.csv"), encoding="utf-8-sig") as f:
            linie = f.read().splitlines()[1:]
    except OSError:
        return []
    stan = {}
    for w in linie:
        k = (w.split(";") + [""] * 13)[:13]
        if not k[0].isdigit():
            continue
        if k[12] == "usuniety":
            stan.pop(k[0], None)
            continue
        stan[k[0]] = {"id": k[0], "godz": k[1], "nr": k[2], "rodzaj": k[3], "sztuk": int(_liczba(k[4])), "zeberek": int(_liczba(k[5])),
                      "szer": _liczba(k[6]), "dl": _liczba(k[7]), "uwagi": k[8], "wys": k[9], "bud": k[10], "kond": k[11]}
    return sorted(stan.values(), key=lambda x: int(x["id"]))


def grzejnik(nr, rodzaj, sztuk, zeberek, szer, dl, uwagi, n=None, idg=None, usun=False):
    """Dopisuje wiersz do grzejniki.csv. Z n i idg: poprawka albo usuniecie wpisu."""
    try:
        sztuk, zeberek = int(sztuk or 0), int(zeberek or 0)
        szer, dl = float(str(szer or 0).replace(",", ".")), float(str(dl or 0).replace(",", "."))
    except ValueError:
        return "Sztuki, żeberka i wymiary muszą być liczbami"
    if min(sztuk, zeberek, szer, dl) < 0:
        return "Liczby nie mogą być ujemne"
    if not usun and sztuk <= 0:
        return "Podaj liczbę sztuk"
    lw = lambda x: (f"{x:.1f}".rstrip("0").rstrip(".").replace(".", ",")) if x else ""
    with LOCK:
        if idg:
            kat = sesja_sciezka(n)
            stary = next((x for x in wiersze_grz(kat) if x["id"] == idg), None) if kat else None
            if not stary:
                return "Nie ma takiego wpisu"
            czas, godz, wys, bud, kond, wpis = idg, stary["godz"], stary["wys"], stary["bud"], stary["kond"], "usuniety" if usun else "poprawka"
            if usun:
                nr, rodzaj, uwagi = stary["nr"], stary["rodzaj"], stary["uwagi"]
        else:
            if not STAN["nagrywa"]:
                return "Zapis grzejników działa tylko podczas nagrywania"
            kat, czas, godz, wpis = STAN["katalog"], str(time.time_ns()), datetime.now().strftime("%H:%M:%S"), ""
            if not czysc(nr):
                nr = f"bez nazwy {len(wiersze_grz(kat)) + 1}"
            wys = "" if STAN["wys"] is None else f"{STAN['wys']:.1f}".replace(".", ",")
            bud, kond = STAN["budynek"], STAN["kondygnacja"]
        sciezka = os.path.join(kat, "grzejniki.csv")
        nowy = not os.path.exists(sciezka)
        with open(sciezka, "a", encoding="utf-8-sig" if nowy else "utf-8") as f:
            if nowy:
                f.write(KOL_GRZ + "\n")
            f.write(f"{czas};{godz};{czysc(nr)};{czysc(rodzaj)};{sztuk or ''};{zeberek or ''};{lw(szer)};{lw(dl)};{czysc(uwagi)};{wys};"
                    f"{bud};{kond};{wpis}\n")
        if STAN["nagrywa"] and os.path.abspath(kat) == os.path.abspath(STAN["katalog"]):
            STAN["grz"] = wiersze_grz(kat)
    if not idg:
        znacznik(f"GRZ {czysc(nr)}", licz=True)
    return None


def wiersze_okn(kat):
    """Aktualny stan stolarki sesji (okna i drzwi zewnetrzne); plik tylko dopisywany."""
    try:
        with open(os.path.join(kat, "okna.csv"), encoding="utf-8-sig") as f:
            linie = f.read().splitlines()[1:]
    except OSError:
        return []
    stan = {}
    for w in linie:
        k = (w.split(";") + [""] * 15)[:15]
        if not k[0].isdigit():
            continue
        if k[14] == "usuniety":
            stan.pop(k[0], None)
            continue
        stan[k[0]] = {"id": k[0], "godz": k[1], "nr": k[2], "typ": k[3], "rodzaj": k[4], "szer": _liczba(k[5]), "wys_cm": _liczba(k[6]),
                      "rama": k[7], "szyby": k[8], "sztuk": int(_liczba(k[9])), "uwagi": k[10], "wys": k[11], "bud": k[12], "kond": k[13]}
    return sorted(stan.values(), key=lambda x: int(x["id"]))


def typy_okien(budynek):
    """Typy stolarki uzyte dotad w budynku (we wszystkich jego nagraniach): litera -> cechy i liczba sztuk."""
    typy = {}
    for n in sorted(os.listdir(ARGS.katalog)):
        p = sesja_sciezka(n)
        if not p or info_sesji(p).get("budynek", "") != budynek:
            continue
        for o in wiersze_okn(p):
            t = typy.setdefault(o["typ"], {"typ": o["typ"], "rodzaj": o["rodzaj"], "szer": o["szer"], "wys_cm": o["wys_cm"],
                                           "rama": o["rama"], "szyby": o["szyby"], "sztuk": 0})
            t["sztuk"] += o["sztuk"]
    return sorted(typy.values(), key=lambda t: (len(t["typ"]), t["typ"]))


def _litera(i):
    return (chr(65 + i // 26 - 1) if i >= 26 else "") + chr(65 + i % 26)


def okno(nr, rodzaj, szer, wys_cm, rama, szyby, sztuk, uwagi, n=None, ido=None, usun=False):
    """Dopisuje wiersz do okna.csv. Litera typu nadawana jest sama: te same cechy = ten sam typ w calym budynku."""
    try:
        sztuk = int(sztuk or 0)
        szer, wys_cm = float(str(szer or 0).replace(",", ".")), float(str(wys_cm or 0).replace(",", "."))
    except ValueError:
        return "Sztuki i wymiary muszą być liczbami"
    if min(sztuk, szer, wys_cm) < 0:
        return "Liczby nie mogą być ujemne"
    if not usun and sztuk <= 0:
        return "Podaj liczbę sztuk"
    if not usun and (szer <= 0 or wys_cm <= 0):
        return "Podaj szerokość i wysokość w cm"
    lw = lambda x: (f"{x:.1f}".rstrip("0").rstrip(".").replace(".", ",")) if x else ""
    rodzaj, rama, szyby = czysc(rodzaj, 30), czysc(rama, 30), czysc(szyby, 10)
    if ido:
        kat = sesja_sciezka(n)
        stary = next((x for x in wiersze_okn(kat) if x["id"] == ido), None) if kat else None
        if not stary:
            return "Nie ma takiego wpisu"
        bud = stary["bud"]
    else:
        if not STAN["nagrywa"]:
            return "Zapis stolarki działa tylko podczas nagrywania"
        kat, stary, bud = STAN["katalog"], None, STAN["budynek"]
    typy = typy_okien(bud)
    if usun:
        typ = stary["typ"]
    else:
        pas = next((t for t in typy if (t["rodzaj"], t["szer"], t["wys_cm"], t["rama"], t["szyby"]) == (rodzaj, szer, wys_cm, rama, szyby)), None)
        uzyte = {t["typ"] for t in typy}
        typ = pas["typ"] if pas else next(_litera(i) for i in range(700) if _litera(i) not in uzyte)
    with LOCK:
        if ido:
            czas, godz, wys, kond, wpis = ido, stary["godz"], stary["wys"], stary["kond"], "usuniety" if usun else "poprawka"
            if usun:
                nr, uwagi = stary["nr"], stary["uwagi"]
        else:
            czas, godz, wpis = str(time.time_ns()), datetime.now().strftime("%H:%M:%S"), ""
            if not czysc(nr):
                nr = f"bez nazwy {len(wiersze_okn(kat)) + 1}"
            wys = "" if STAN["wys"] is None else f"{STAN['wys']:.1f}".replace(".", ",")
            kond = STAN["kondygnacja"]
        sciezka = os.path.join(kat, "okna.csv")
        nowy = not os.path.exists(sciezka)
        with open(sciezka, "a", encoding="utf-8-sig" if nowy else "utf-8") as f:
            if nowy:
                f.write(KOL_OKN + "\n")
            f.write(f"{czas};{godz};{czysc(nr)};{typ};{rodzaj};{lw(szer)};{lw(wys_cm)};{rama};{szyby};{sztuk or ''};{czysc(uwagi)};{wys};"
                    f"{bud};{kond};{wpis}\n")
        if STAN["nagrywa"] and os.path.abspath(kat) == os.path.abspath(STAN["katalog"]):
            STAN["okn"] = wiersze_okn(kat)
    if not ido:
        znacznik(f"OKN {czysc(nr)} typ {typ}", licz=True)
    return None


def znaczniki_stan(kat):
    """Aktualny stan znacznikow: uwzglednia dopisane wiersze 'EDYCJA <id> <tekst>' i 'USUNIETO <id>'."""
    try:
        with open(os.path.join(kat, "znaczniki.txt"), encoding="utf-8") as f:
            linie = f.read().splitlines()
    except OSError:
        return []
    stan = {}
    for w in linie:
        cz = w.split("\t")
        if len(cz) < 2 or not cz[0].isdigit():
            continue
        opis = cz[1]
        if opis.startswith("EDYCJA "):
            _, idz, *reszta = opis.split(" ", 2)
            if idz in stan:
                stan[idz]["opis"] = reszta[0] if reszta else ""
        elif opis.startswith("USUNIETO "):
            stan.pop(opis.split(" ", 1)[1].strip(), None)
        else:
            stan[cz[0]] = {"id": cz[0], "godz": datetime.fromtimestamp(int(cz[0]) / 1e9).strftime("%H:%M:%S"),
                           "opis": opis, "sys": opis.startswith(ZN_SYSTEMOWE)}
    return list(stan.values())


def _dopisz_znacznik(kat, opis):
    with open(os.path.join(kat, "znaczniki.txt"), "a", encoding="utf-8") as f:
        f.write(f"{time.time_ns()}\t{opis}\t\n")


def znacznik_zmien(n, idz, opis=None):
    """opis=None usuwa znacznik, inaczej zmienia jego tresc. Dotyczy tylko znacznikow wpisanych recznie."""
    kat = sesja_sciezka(n)
    if not kat:
        return "Nie ma takiej sesji"
    with LOCK:
        z = next((x for x in znaczniki_stan(kat) if x["id"] == idz), None)
        if not z:
            return "Nie ma takiego znacznika"
        if z["sys"]:
            return "Znaczników systemowych nie można zmieniać"
        _dopisz_znacznik(kat, f"USUNIETO {idz}" if opis is None else f"EDYCJA {idz} {czysc(opis, 500)}")
    return None


def pomieszczenie(nr, typ, punktow, opraw, uwagi, moc="", n=None, idp=None, usun=False):
    """Dopisuje wiersz do pomieszczenia.csv (inwentaryzacja oswietlenia). Z n i idp: poprawka albo usuniecie wpisu."""
    try:
        opraw = int(opraw or 0)
        punktow = int(punktow or 0)
        moc = float(str(moc or 0).replace(",", "."))
    except ValueError:
        return "Liczba opraw, liczba punktów i moc muszą być liczbami"
    if moc < 0 or opraw < 0 or punktow < 0:
        return "Liczby nie mogą być ujemne"
    if opraw == 0:
        typ, punktow, moc = "", 0, 0.0            # pomieszczenie bez opraw: sama nazwa i uwagi
    moc_razem = moc * max(punktow, 1) * opraw
    lw = lambda x: (f"{x:.1f}".rstrip("0").rstrip(".").replace(".", ",")) if x else ""
    with LOCK:
        if idp:
            kat = sesja_sciezka(n)
            stary = next((x for x in wiersze_pom(kat) if x["id"] == idp), None) if kat else None
            if not stary:
                return "Nie ma takiego wpisu"
            czas, godz, wys, bud, kond = idp, stary["godz"], stary["wys"], stary["bud"], stary["kond"]
            wpis = "usuniety" if usun else "poprawka"
            if usun:
                nr, typ, uwagi, punktow, opraw, moc, moc_razem = stary["nr"], stary["typ"], stary["uwagi"], 0, 0, 0, 0
        else:
            if not STAN["nagrywa"]:
                return "Zapis pomieszczeń działa tylko podczas nagrywania"
            kat, czas, godz, wpis = STAN["katalog"], str(time.time_ns()), datetime.now().strftime("%H:%M:%S"), ""
            if not czysc(nr):                     # bez nazwy: miejsce i tak wyznacza znacznik czasu w nagraniu
                nr = f"bez nazwy {len(wiersze_pom(kat)) + 1}"
            wys = "" if STAN["wys"] is None else f"{STAN['wys']:.1f}".replace(".", ",")
            bud, kond = STAN["budynek"], STAN["kondygnacja"]
        sciezka = os.path.join(kat, "pomieszczenia.csv")
        nowy = not os.path.exists(sciezka)
        with open(sciezka, "a", encoding="utf-8-sig" if nowy else "utf-8") as f:
            if nowy:
                f.write(KOL_POM + "\n")
            f.write(f"{czas};{godz};{czysc(nr)};{czysc(typ)};{punktow or ''};{opraw};{punktow * opraw or ''};{czysc(uwagi)};{wys};"
                    f"{bud};{kond};{lw(moc)};{lw(moc_razem)};{wpis}\n")
        if STAN["nagrywa"] and os.path.abspath(kat) == os.path.abspath(STAN["katalog"]):
            STAN["pom"] = wiersze_pom(kat)
    if not idp:
        znacznik(f"POM {czysc(nr)}", licz=True)
    return None


def zmien_kondygnacje(k):
    k = czysc(k, 30)
    with LOCK:
        if not STAN["nagrywa"]:
            STAN["kondygnacja"] = k
            return True
        STAN["kondygnacja"] = k
    ustawienia({"kondygnacja": k})
    return znacznik(f"KONDYGNACJA {k}", licz=False)


def sesja_sciezka(n):
    """Zwraca sciezke katalogu sesji albo None (ochrona przed wyjsciem poza katalog nagran)."""
    if not n or n.startswith(".") or "/" in n or "\\" in n:
        return None
    p = os.path.join(ARGS.katalog, n)
    return p if os.path.isdir(p) else None


def info_sesji(p):
    d = {}
    try:
        with open(os.path.join(p, "info.txt"), encoding="utf-8") as f:
            for w in f:
                if "=" in w:
                    k, v = w.rstrip("\n").split("=", 1)
                    d[k] = v
    except OSError:
        pass
    return d


def lista_sesji():
    wyn = []
    for n in sorted(os.listdir(ARGS.katalog), reverse=True):
        p = sesja_sciezka(n)
        if not p:
            continue
        inf, pom, grz = info_sesji(p), wiersze_pom(p), wiersze_grz(p)
        wyn.append({"n": n, "mb": round(sum(os.path.getsize(os.path.join(p, x)) for x in os.listdir(p)) / 1e6, 1),
                    "bud": inf.get("budynek", ""), "kond": inf.get("kondygnacja", ""),
                    "data": n[6:8] + "." + n[4:6] + "." + n[:4] + " " + n[9:11] + ":" + n[11:13] if n[:8].isdigit() else "",
                    "zn": sum(1 for z in znaczniki_stan(p) if not z["sys"]), "pom": len(pom),
                    "grz": sum(x["sztuk"] for x in grz), "okn": sum(x["sztuk"] for x in wiersze_okn(p)),
                    "opraw": sum(x["opraw"] for x in pom), "w": round(sum(x["moc_razem"] for x in pom)),
                    "aktywna": STAN["nagrywa"] and os.path.abspath(p) == os.path.abspath(STAN["katalog"])})
    return wyn


def usun_budynek(b):
    """Usuwa z karty wszystkie sesje danego budynku (poza wlasnie nagrywana)."""
    ile = 0
    for x in lista_sesji():
        if x["bud"] == b and not x["aktywna"]:
            if usun_sesje(x["n"]) is None:
                ile += 1
    return ile


def sesja_tresc(n):
    p = sesja_sciezka(n)
    if not p:
        return None
    try:
        with open(os.path.join(p, "info.txt"), encoding="utf-8") as f:
            info = f.read()[-4000:]
    except OSError:
        info = ""
    return {"n": n, "info": info, "znaczniki": znaczniki_stan(p), "pom": wiersze_pom(p), "grz": wiersze_grz(p), "okn": wiersze_okn(p),
            "foto": sorted(x for x in os.listdir(p) if re.fullmatch(r"foto_[0-9_]+\.jpg", x))}


def usun_sesje(n):
    p = sesja_sciezka(n)
    if not p:
        return "Nie ma takiej sesji"
    with LOCK:
        if STAN["nagrywa"] and os.path.abspath(p) == os.path.abspath(STAN["katalog"]):
            return "Nie można usunąć sesji, która właśnie się nagrywa"
    shutil.rmtree(p)
    return None


def zamien_lidary():
    """Zamienia role: ktory LiDAR jest poziomy (podglad), a ktory pionowy."""
    if not ARGS.lidar2:
        return "Drugi LiDAR nie jest włączony"
    with LOCK:
        STAN["podglad"] = "lidar2" if STAN["podglad"] == "lidar" else "lidar"
        STAN["obrot"], p = [], STAN["podglad"]
    ustawienia({"podglad": p})
    znacznik(f"LIDAR_POZIOMY {p}", licz=False)
    return None


def zapisz_zdjecie(dane, opis):
    """Zapisuje zdjecie JPEG z telefonu w katalogu sesji i dopisuje znacznik FOTO."""
    if dane[:2] != b"\xff\xd8":
        return "To nie jest zdjęcie JPEG"
    with LOCK:
        if not STAN["nagrywa"]:
            return "Zdjęcia zapisują się tylko podczas nagrywania"
        STAN["foto"] += 1
        nazwa = f"foto_{datetime.now().strftime('%H%M%S')}_{STAN['foto']:03d}.jpg"
        with open(os.path.join(STAN["katalog"], nazwa), "wb") as f:
            f.write(dane)
    znacznik(f"FOTO {nazwa} {czysc(opis, 120)}".strip(), licz=False)
    return None


def pauza():
    """Przelacza pauze: zapis surowych danych trwa, ale odcinek jest oznaczony do pominiecia."""
    with LOCK:
        if not STAN["nagrywa"]:
            return None
        STAN["pauza"] = not STAN["pauza"]
        nowy = STAN["pauza"]
    znacznik("PAUZA_START" if nowy else "PAUZA_KONIEC", licz=False)
    return nowy


def watek_flush():
    while not KONIEC.is_set():
        time.sleep(5)
        with LOCK:
            for f in PLIKI.values():
                f.flush()
                os.fsync(f.fileno())


# ----------------------------------------------------------------------------- strona WWW
STRONA = r"""<!doctype html><html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#111"><title>Skaner</title><style>
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;font-size:16px;padding-bottom:76px}
section{display:none}section.on{display:block}
#tb{position:fixed;left:0;right:0;bottom:0;z-index:6;display:flex;gap:3px;padding:6px 6px calc(6px + env(safe-area-inset-bottom));background:#181818;border-top:1px solid #333}
#bfoto{position:fixed;right:10px;bottom:calc(74px + env(safe-area-inset-bottom));z-index:7;flex:none;width:62px;height:62px;min-height:0;border-radius:31px;background:#2b62c9;font-size:26px;box-shadow:0 2px 8px #000a;padding:0}
#bfoto small{position:absolute;right:-2px;top:-4px;background:#c33;border-radius:10px;font-size:12px;padding:1px 6px}#bfoto small:empty{display:none}
.siatka{flex-wrap:wrap}.siatka button{flex:1 0 30%}
#tb button{min-height:52px;font-size:13px;padding:0;background:#2a2a2a;border-radius:10px}#tb button.on{background:#2b62c9}
body:not(.nagr) .rec{display:none!important}body.nagr .nrec{display:none!important}
.lb{flex:0 0 96px;align-self:center;font-size:13px;color:#bbb;line-height:1.2}.zaw{flex-wrap:wrap;overflow:visible}
#pasek{position:sticky;top:0;z-index:5;background:#181818;display:flex;gap:4px;padding:6px;border-bottom:1px solid #333}
#pasek div{flex:1;text-align:center;border-radius:8px;background:#242424;padding:5px 2px;min-width:0}
#pasek small{display:block;color:#999;font-size:10px;line-height:1.1}
#pasek b{font-size:15px;white-space:nowrap}
.ok{color:#5d5}.zle{color:#f55}.uw{color:#fb3}
#mapa{position:relative}
#c{width:100vw;height:min(100vw,36vh);display:block;background:#000}
#zas{position:absolute;right:6px;top:6px;width:auto;font-size:15px;padding:6px 8px;border-radius:8px;border:0;background:#333c;color:#fff}
#prk{display:none;position:absolute;left:0;right:0;bottom:0;padding:12px 10px;text-align:center;font-size:20px;font-weight:700;color:#fff}
#prk small{display:block;font-weight:400;font-size:14px}
.lst{padding:2px 10px;font-size:14px;line-height:1.5}
.lst .r{display:flex;gap:8px;align-items:center;border-bottom:1px solid #333;padding:6px 0}
.lst .r span{flex:1;min-width:0;overflow-wrap:anywhere}
.lst .r button{flex:0 0 auto;min-height:40px;font-size:14px;font-weight:400;padding:0 12px;background:#444}
.lst pre{white-space:pre-wrap;word-break:break-word;background:#1a1a1a;padding:8px;border-radius:8px;font-size:12px;margin:6px 0}
#kom{padding:4px 10px;color:#fb3;font-size:13px}
#kom:empty{display:none}
.w{padding:4px 8px;display:flex;gap:8px}
button{flex:1;font-size:17px;font-weight:600;min-height:50px;border:0;border-radius:12px;color:#fff;touch-action:manipulation}
button:active{filter:brightness(1.35)}
input,select,textarea{width:100%;font-size:17px;padding:11px 12px;border-radius:12px;border:1px solid #444;background:#222;color:#fff;font-family:inherit}
textarea{min-height:80px;resize:vertical}
h2{font-size:13px;color:#9ab;margin:14px 10px 2px;text-transform:uppercase;letter-spacing:.05em}
#bs{background:#1f9d68;min-height:60px}#bp{background:#c33}#bpa{background:#b70}.nb{background:#2b62c9}.sz{background:#444;font-weight:400;font-size:16px}
.chip{flex:0 0 auto;min-height:40px;font-size:15px;font-weight:400;background:#333;padding:0 14px;border-radius:22px}
.chips{padding:5px 8px;display:flex;gap:8px;overflow-x:auto}
.seg button{min-height:46px;font-size:15px;font-weight:400;background:#2a2a2a;border:1px solid #444}
.seg button.on{background:#2b62c9;border-color:#2b62c9;font-weight:600}
.seg{gap:6px}
.et{font-size:13px;color:#aaa;padding:6px 12px 0}
.licz{display:flex;gap:8px;align-items:stretch;flex:1}
.licz button{flex:0 0 52px;background:#444;font-size:24px;min-height:46px}
.licz input{text-align:center}
#plista{padding:2px 10px 4px;font-size:14px;color:#bbb;line-height:1.5}
.k{flex:1;background:#1c1c1c;border-radius:10px;padding:8px 10px}
.k small{color:#999;font-size:11px;display:block}.k b{font-size:17px}
</style></head><body>
<div id="pasek">
 <div><small>Zapis</small><b id="nag">–</b></div>
 <div><small>LiDAR</small><b id="lid">–</b></div>
 <div><small>Przechył</small><b id="prz">–</b></div>
 <div><small>Wysokość</small><b id="wys">–</b></div>
</div>
<div id="kom"></div>

<section id="t1">
<div id="mapa"><canvas id="c"></canvas><div id="prk"></div>
 <select id="zas"><option value="3">3 m</option><option value="6" selected>6 m</option><option value="12">12 m</option><option value="20">20 m</option></select></div>
<div id="pkt" style="display:none;background:#1c2430;padding:4px 0 6px">
<div class="et" id="pktet"></div>
<div class="chips zaw"><button class="chip" onclick="$('popis').value='pion';zapPkt()">pion</button><button class="chip" onclick="$('popis').value='grzejnik';zapPkt()">grzejnik</button><button class="chip" onclick="$('popis').value='okno';zapPkt()">okno</button><button class="chip" onclick="$('popis').value='szafa';zapPkt()">szafa</button><button class="chip" onclick="$('popis').value='drzwi';zapPkt()">drzwi</button><button class="chip" onclick="$('popis').value='rozdzielnica';zapPkt()">rozdzielnica</button></div>
<div class="w"><input id="psr" type="number" inputmode="numeric" placeholder="Ø mm" style="flex:0 0 92px;width:92px"><input id="popis" placeholder="inny opis" autocomplete="off">
 <button class="nb" style="flex:0 0 84px" onclick="zapPkt()">Zapisz</button><button class="sz" style="flex:0 0 46px" onclick="sel=null;$('pkt').style.display='none'">✕</button></div>
</div>
<div class="w nrec"><input id="bud" list="budl" placeholder="budynek (z listy albo nowy)" autocomplete="off"><datalist id="budl"></datalist></div>
<div class="w"><input id="kon" placeholder="kondygnacja" autocomplete="off">
 <button id="bk" class="sz rec" style="flex:0 0 96px" onclick="cmd('kondygnacja?k='+encodeURIComponent($('kon').value),wibr)">Zmień</button></div>
<div class="chips"><button class="chip" onclick="$('kon').value='piwnica'">piwnica</button><button class="chip" onclick="$('kon').value='parter'">parter</button><button class="chip" onclick="$('kon').value='1 piętro'">1 piętro</button><button class="chip" onclick="$('kon').value='2 piętro'">2 piętro</button><button class="chip" onclick="$('kon').value='3 piętro'">3 piętro</button><button class="chip" onclick="$('kon').value='poddasze'">poddasze</button></div>
<div class="w nrec"><button id="bs" onclick="cmd('start?budynek='+encodeURIComponent($('bud').value)+'&kondygnacja='+encodeURIComponent($('kon').value),()=>{if($('snr').value.trim())setTimeout(zapSala,700)})">START</button></div>
<div class="w"><input id="snr" placeholder="pomieszczenie, w którym stoisz, np. sala 12" autocomplete="off" enterkeyhint="done" oninput="$('pnr').value=$('gnr').value=$('onr').value=this.value" onkeydown="if(event.key=='Enter'){zapSala();this.blur()}">
 <button class="nb rec" style="flex:0 0 110px" onclick="zapSala()">Jestem tu</button></div>
<div class="w"><small id="sost" style="color:#9ab;padding:0 6px"></small></div>
<div class="w rec"><button id="bpa" onclick="cmd('pauza')">PAUZA</button>
 <button id="bp" style="flex:0 0 34%" onclick="if(confirm('Zakończyć nagranie?'))cmd('stop')">STOP</button></div>
</section>

<section id="t2">
<div class="w"><input id="pnr" placeholder="pomieszczenie, np. sala 12 (może być puste)" autocomplete="off" oninput="$('snr').value=$('gnr').value=$('onr').value=this.value"></div>
<div class="w seg" id="ptyp"><button class="on" onclick="typ(this)">świetlówki</button><button onclick="typ(this)">LED</button>
 <button onclick="typ(this)">żarówki</button><button onclick="typ(this)">inne</button></div>
<div class="w"><label class="lb">1. Opraw</label><div class="licz"><button onclick="krok('popr',-1)">−</button>
 <input id="popr" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('popr',1)">+</button></div></div>
<div class="w"><label class="lb">2. Punktów w oprawie</label><div class="licz"><button onclick="krok('ppkt',-1)">−</button>
 <input id="ppkt" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('ppkt',1)">+</button></div></div>
<div class="w"><label class="lb">3. Moc punktu W</label><div class="licz"><button onclick="krok('pmoc',-1)">−</button>
 <input id="pmoc" type="number" inputmode="decimal" placeholder="0"><button onclick="krok('pmoc',1)">+</button></div></div>
<div class="chips"><button class="chip" onclick="$('pmoc').value=9">9</button><button class="chip" onclick="$('pmoc').value=18">18</button><button class="chip" onclick="$('pmoc').value=36">36</button><button class="chip" onclick="$('pmoc').value=58">58</button><button class="chip" onclick="$('pmoc').value=60">60</button><button class="chip" onclick="$('pmoc').value=100">100</button></div>
<div class="w"><input id="puw" placeholder="uwagi, np. 2 nie świecą" autocomplete="off"></div>
<div id="edb" class="lst" style="display:none;color:#fb3"></div>
<div class="w"><button class="nb" id="bzp" onclick="zapPom()">ZAPISZ POMIESZCZENIE</button></div>
<div id="plista" class="lst"></div>
<div class="w" id="bed" style="display:none"><button class="sz" onclick="tab(4);sesja(encodeURIComponent(aktN))">Popraw / usuń wpisy</button></div>
</section>

<section id="t5">
<div class="w"><input id="gnr" placeholder="pomieszczenie, np. sala 12 (może być puste)" autocomplete="off" oninput="$('snr').value=$('pnr').value=$('onr').value=this.value"></div>
<div class="w seg siatka" id="grodz"><button class=on onclick="typG(this)">żeliwny</button><button onclick="typG(this)">płytowy</button><button onclick="typG(this)">aluminiowy</button><button onclick="typG(this)">łazienkowy</button><button onclick="typG(this)">rurowy</button><button onclick="typG(this)">favier</button></div>
<div class="w"><label class="lb">1. Sztuk</label><div class="licz"><button onclick="krok('gszt',-1)">−</button>
 <input id="gszt" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('gszt',1)">+</button></div></div>
<div class="w"><label class="lb">2. Żeberek (w jednym)</label><div class="licz"><button onclick="krok('gzeb',-1)">−</button>
 <input id="gzeb" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('gzeb',1)">+</button></div></div>
<div class="w"><label class="lb">lub szerokość cm</label><div class="licz"><button onclick="krok('gszer',-1)">−</button>
 <input id="gszer" type="number" inputmode="decimal" placeholder="0"><button onclick="krok('gszer',1)">+</button></div></div>
<div class="w"><label class="lb">i długość cm</label><div class="licz"><button onclick="krok('gdl',-1)">−</button>
 <input id="gdl" type="number" inputmode="decimal" placeholder="0"><button onclick="krok('gdl',1)">+</button></div></div>
<div class="w"><input id="guw" placeholder="uwagi, np. bez zaworu, zabudowany" autocomplete="off"></div>
<div id="edg" class="lst" style="display:none;color:#fb3"></div>
<div class="w"><button class="nb" id="bzg" onclick="zapGrz()">ZAPISZ GRZEJNIK</button></div>
<div id="glista" class="lst"></div>
</section>

<section id="t6">
<div class="w"><input id="onr" placeholder="pomieszczenie / elewacja (może być puste)" autocomplete="off" oninput="$('snr').value=$('pnr').value=$('gnr').value=this.value"></div>
<div class="et">Typy w tym budynku (dotknij, żeby wczytać wymiary)</div>
<div class="chips" id="otypy"><span style="color:#888;font-size:13px;padding:10px 4px">brak – pierwszy wpis utworzy typ A</span></div>
<div class="w seg" id="orodz"><button class=on onclick="segO(this)">okno</button><button onclick="segO(this)">drzwi zewn.</button><button onclick="segO(this)">drzwi balk.</button><button onclick="segO(this)">inne</button></div>
<div class="w"><label class="lb">Szer. × wys. cm</label><input id="oszer" type="number" inputmode="decimal" placeholder="szer."><input id="owys" type="number" inputmode="decimal" placeholder="wys."></div>
<div class="w seg" id="orama"><button class=on onclick="segO(this)">PCV</button><button onclick="segO(this)">drewno</button><button onclick="segO(this)">aluminium</button><button onclick="segO(this)">stal</button></div>
<div class="w seg" id="oszyb"><button class=on onclick="segO(this)">1 szyba</button><button onclick="segO(this)">2 szyby</button><button onclick="segO(this)">3 szyby</button><button onclick="segO(this)">pełne</button></div>
<div class="w"><label class="lb">Sztuk</label><div class="licz"><button onclick="krok('oszt',-1)">−</button>
 <input id="oszt" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('oszt',1)">+</button></div></div>
<div class="w"><input id="ouw" placeholder="uwagi, np. nieszczelne, rok 1998, nawiewniki" autocomplete="off"></div>
<div id="edo" class="lst" style="display:none;color:#fb3"></div>
<div class="w"><button class="nb" id="bzo" onclick="zapOkn()">ZAPISZ OKNA / DRZWI</button></div>
<div id="olista" class="lst"></div>
</section>

<section id="t3">
<div class="w"><label class="lb">Wysokość w świetle cm</label><input id="zwys" type="number" inputmode="decimal" placeholder="np. 285">
 <button class="nb" style="flex:0 0 90px" onclick="if($('zwys').value)szybki('WYSOKOŚĆ W ŚWIETLE '+$('zwys').value+' cm'+nazwaP());$('zwys').value=''">Zapisz</button></div>
<div class="w"><label class="lb">Klimatyzacja</label><input id="zkli" placeholder="np. 2 szt. split 3,5 kW" autocomplete="off">
 <button class="nb" style="flex:0 0 90px" onclick="if($('zkli').value)szybki('KLIMATYZACJA '+$('zkli').value+nazwaP());$('zkli').value=''">Zapisz</button></div>
<div class="w"><textarea id="opis" placeholder="Notatka do tego miejsca, np. zabudowa na całej ścianie północnej"></textarea></div>
<div class="w"><button class="nb" onclick="zapZn()">ZAPISZ ZNACZNIK</button></div>
<div class="et">Szybkie znaczniki (jedno dotknięcie)</div>
<div class="chips zaw"><button class="chip" onclick="szybki('drzwi zamknięte')">drzwi zamknięte</button><button class="chip" onclick="szybki('szafa na ścianie')">szafa na ścianie</button><button class="chip" onclick="szybki('okna zasłonięte')">okna zasłonięte</button><button class="chip" onclick="szybki('pomiar kontrolny')">pomiar kontrolny</button><button class="chip" onclick="szybki('klatka schodowa')">klatka schodowa</button><button class="chip" onclick="szybki('ludzie w kadrze')">ludzie w kadrze</button></div>
</section>

<section id="t4">
<div class="w"><button class="sz" onclick="sesje()">Odśwież listę budynków</button></div>
<div id="sl" class="lst"></div><div id="sd" class="lst"></div>
<h2>Stan</h2>
<div class="w"><div class="k"><small>Sesja</small><b id="roz">–</b><small id="zn"></small></div>
 <div class="k"><small>Karta – wolne</small><b id="dysk">–</b><small id="ses"></small></div></div>
<div class="w"><div class="k"><small>Kopia na serwer</small><b id="kop">–</b><small id="kop2"></small>
  <button class="btn" style="margin-top:6px;padding:6px 10px;font-size:13px" onclick="cmd('wyslij_teraz',()=>alert('Wysyłka ruszyła – wysyła wszystko, czego brakuje na serwerze (także sesje skasowane na Dellu). Stan zobaczysz w tym kafelku.'))">⟳ Wyślij teraz</button></div>
 <div class="k"><small>Kopia na pendrive</small><b id="pen">–</b><small id="pen2"></small></div></div>
<div class="w"><div class="k"><small>LiDAR</small><b id="lid2">–</b></div>
 <div class="k"><small>Max przechył w sesji</small><b id="prz2">–</b><small id="wys2"></small></div></div>
<div class="w" id="l2" style="display:none"><div class="k"><small>Drugi LiDAR (pionowy)</small><b id="l2s">–</b></div>
 <button class="sz" onclick="if(confirm('Zamienić role LiDAR-ów? Podgląd w zakładce Skan ma pokazywać ten poziomy.'))cmd('zamien_lidary')">Zamień poziomy / pionowy</button></div>
<div class="w"><div class="k"><small>Adres strony</small><b id="ip">–</b></div>
 <div class="k"><small>Pion IMU (pitch / roll)</small><b id="p0">–</b></div></div>
<div class="w"><button class="sz" onclick="if(confirm('Tyczka stoi pionowo i nieruchomo?'))cmd('zeruj_pion',()=>alert('Pion zapisany'))">Zeruj pion (tyczka pionowo, 2 s)</button></div>
<div class="w"><button class="sz" onclick="if(confirm('Zerować wysokość w tym miejscu?'))cmd('zeruj')">Zeruj wysokość</button>
 <button class="sz" onclick="if(confirm('Zatrzymać zapis i wyłączyć Raspberry Pi?'))cmd('wylacz')">Wyłącz RPi</button></div>
</section>

<input type="file" id="fplik" accept="image/*" capture="environment" style="display:none" onchange="wyslijFoto(this)">
<button id="bfoto" class="rec" onclick="$('fplik').click()">📷<small id="fn"></small></button>
<nav id="tb"><button onclick="tab(1)">Skan</button><button onclick="tab(2)">Oprawy</button><button onclick="tab(5)">Grzejniki</button><button onclick="tab(6)">Okna</button><button onclick="tab(3)">Notatka</button><button onclick="tab(4)">Dane</button></nav>
<script>
const $=id=>document.getElementById(id),c=$('c'),g=c.getContext('2d');let ost=null;
function cmd(u,ok){fetch('/'+u,{method:'POST'}).then(r=>r.json()).then(j=>{if(j.blad)alert(j.blad);else if(ok)ok()}).catch(()=>alert('Brak połączenia z Raspberry Pi'))}
function wibr(){if(navigator.vibrate)navigator.vibrate(40)}
let ac=null,pf=0;
function dzw(f,ms,kiedy){if(!ac)return;const o=ac.createOscillator(),w=ac.createGain();o.frequency.value=f;o.connect(w);w.connect(ac.destination);
 const t=ac.currentTime+(kiedy||0);w.gain.setValueAtTime(.5,t);o.start(t);o.stop(t+ms/1000)}
addEventListener('pointerdown',()=>{try{ac=ac||new (window.AudioContext||window.webkitAudioContext)();ac.resume()}catch(e){}});
function przekroj(s){const e=$('prk'),f=s.nagrywa?s.prz_faza:0;
 if(f==1){e.style.display='block';e.style.background='#b70d';e.innerHTML='PRZEKRÓJ – trzymaj nieruchomo '+Math.ceil(s.prz_zostalo)+' s'}
 else if(f==2){e.style.display='block';e.style.background='#1f9d68e6';e.innerHTML='PRZEKRÓJ nr '+s.prz_nr+' ZAPISANY<small>'+(s.prz_wynik||'liczę…')+' · postaw do pionu</small>'}
 else e.style.display='none';
 if(f!=pf){if(f==1){dzw(660,90)}else if(f==2){dzw(880,150);dzw(1320,250,.18);if(navigator.vibrate)navigator.vibrate([150,80,150])}else if(pf==1){dzw(220,250)}pf=f}}
function zapZn(){cmd('znacznik?opis='+encodeURIComponent($('opis').value),()=>{$('opis').value='';wibr()})}
function zapSala(){const v=$('snr').value.trim();if(!v){alert('Wpisz nazwę pomieszczenia');return}
 cmd('znacznik?opis='+encodeURIComponent('SALA '+v),()=>{$('sost').textContent='Zapisano: '+v+' – wpisy w Oprawy / Grzejniki / Okna pójdą do tego pomieszczenia';wibr()})}
function szybki(t){cmd('znacznik?opis='+encodeURIComponent(t),wibr)}
function krok(id,d){const e=$(id);e.value=Math.max(0,(parseInt(e.value)||0)+d)}
function typ(b){for(const x of $('ptyp').children)x.className='';b.className='on'}
let ed=null,aktN='',otw='';
const TB=[1,2,5,6,3,4];
function nazwaP(){return $('pnr').value?' ('+$('pnr').value+')':''}
function tab(n){for(let i=1;i<=6;i++){$('t'+i).className=i==n?'on':'';$('tb').children[TB.indexOf(i)].className=i==n?'on':''}
 try{localStorage.setItem('tab',n)}catch(e){} if(n==4)sesje();if(n==6)typyO();scrollTo(0,0)}
function zapPom(){const e=encodeURIComponent,t=$('ptyp').querySelector('.on').textContent;
 cmd('pomieszczenie?nr='+e($('pnr').value)+'&typ='+e(t)+'&punktow='+e($('ppkt').value)+'&opraw='+e($('popr').value)+'&moc='+e($('pmoc').value)+'&uwagi='+e($('puw').value)+
 (ed?'&n='+e(ed.n)+'&id='+ed.id:''),()=>{$('popr').value='';$('puw').value='';wibr();if(ed){const n=ed.n;koniecEd();tab(4);sesja(e(n))}})}
function koniecEd(){ed=null;$('edb').style.display='none';$('bzp').textContent='ZAPISZ POMIESZCZENIE';$('popr').value='';$('puw').value=''}
function wyslijFoto(inp){const f=inp.files[0];inp.value='';if(!f)return;$('bfoto').style.opacity=.4;
 const wyslij=b=>fetch('/zdjecie?opis='+encodeURIComponent($('pnr').value),{method:'POST',body:b}).then(r=>r.json()).then(j=>{$('bfoto').style.opacity=1;if(j.blad)alert(j.blad);else wibr()})
  .catch(()=>{$('bfoto').style.opacity=1;alert('Nie udało się wysłać zdjęcia')});
 const im=new Image();im.onload=()=>{const k=Math.min(1,1600/Math.max(im.width,im.height)),cv=document.createElement('canvas');cv.width=Math.round(im.width*k);cv.height=Math.round(im.height*k);
  cv.getContext('2d').drawImage(im,0,0,cv.width,cv.height);URL.revokeObjectURL(im.src);cv.toBlob(b=>wyslij(b||f),'image/jpeg',.82)};
 im.onerror=()=>wyslij(f);im.src=URL.createObjectURL(f)}
let edo=null,typyL=[];
function segO(b){for(const x of b.parentNode.children)x.className='';b.className='on'}
function ustSeg(id,t){let z=false;for(const b of $(id).children){b.className=b.textContent==t?'on':'';z=z||b.textContent==t}if(!z)$(id).children[0].className='on'}
function wybSeg(id){return $(id).querySelector('.on').textContent}
function opisOkn(o){return o.sztuk+' × typ '+esc(o.typ)+' ('+esc(o.rodzaj)+' '+o.szer+'×'+o.wys_cm+', '+esc(o.rama)+', '+esc(o.szyby)+')'}
function typyO(){fetch('/typy_okien?b='+encodeURIComponent($('bud').value)).then(r=>r.json()).then(l=>{typyL=l;
 if(l.length)$('otypy').innerHTML=l.map((t,i)=>'<button class="chip" onclick="wczytajTyp('+i+')"><b>'+esc(t.typ)+'</b>&nbsp;'+esc(t.rodzaj)+' '+t.szer+'×'+t.wys_cm+' · '+t.sztuk+' szt.</button>').join('')})}
function wczytajTyp(i){const t=typyL[i];ustSeg('orodz',t.rodzaj);ustSeg('orama',t.rama);ustSeg('oszyb',t.szyby);$('oszer').value=t.szer;$('owys').value=t.wys_cm;$('oszt').focus()}
function zapOkn(){const e=encodeURIComponent;
 cmd('okno?nr='+e($('onr').value)+'&rodzaj='+e(wybSeg('orodz'))+'&szer='+e($('oszer').value)+'&wys='+e($('owys').value)+'&rama='+e(wybSeg('orama'))+'&szyby='+e(wybSeg('oszyb'))+'&sztuk='+e($('oszt').value)+'&uwagi='+e($('ouw').value)+
 (edo?'&n='+e(edo.n)+'&id='+edo.id:''),()=>{$('oszt').value='';$('ouw').value='';wibr();typyO();if(edo){const n=edo.n;koniecEdO();tab(4);sesja(e(n))}})}
function koniecEdO(){edo=null;$('edo').style.display='none';$('bzo').textContent='ZAPISZ OKNA / DRZWI';$('oszt').value='';$('ouw').value=''}
function edOkn(i){const o=podgl.okn[i];edo={n:podgl.n,id:o.id};$('onr').value=o.nr;ustSeg('orodz',o.rodzaj);ustSeg('orama',o.rama);ustSeg('oszyb',o.szyby);$('oszer').value=o.szer;$('owys').value=o.wys_cm;$('oszt').value=o.sztuk;$('ouw').value=o.uwagi;
 $('edo').style.display='block';$('edo').innerHTML='Poprawiasz wpis: <b>'+esc(o.nr)+'</b> ('+o.godz+') &nbsp; <a href="#" style="color:#9cf" onclick="koniecEdO();return false">anuluj</a>';
 $('bzo').textContent='ZAPISZ POPRAWKĘ';tab(6)}
function usOkn(i){const o=podgl.okn[i];if(!confirm('Usunąć wpis stolarki w „'+o.nr+'”?'))return;
 cmd('okno?usun=1&n='+encodeURIComponent(podgl.n)+'&id='+o.id,()=>sesja(encodeURIComponent(podgl.n)))}
let edg=null;
function typG(b){for(const x of $('grodz').children)x.className='';b.className='on'}
function opisGrz(g){return g.sztuk+' × '+esc(g.rodzaj)+(g.zeberek?', '+g.zeberek+' żeb.':'')+(g.szer||g.dl?', '+(g.szer||'?')+' × '+(g.dl||'?')+' cm':'')}
function zapGrz(){const e=encodeURIComponent,r=$('grodz').querySelector('.on').textContent;
 cmd('grzejnik?nr='+e($('gnr').value)+'&rodzaj='+e(r)+'&sztuk='+e($('gszt').value)+'&zeberek='+e($('gzeb').value)+'&szer='+e($('gszer').value)+'&dl='+e($('gdl').value)+'&uwagi='+e($('guw').value)+
 (edg?'&n='+e(edg.n)+'&id='+edg.id:''),()=>{$('gszt').value='';$('guw').value='';wibr();if(edg){const n=edg.n;koniecEdG();tab(4);sesja(e(n))}})}
function koniecEdG(){edg=null;$('edg').style.display='none';$('bzg').textContent='ZAPISZ GRZEJNIK';$('gszt').value='';$('guw').value=''}
function edGrz(i){const g=podgl.grz[i];edg={n:podgl.n,id:g.id};$('gnr').value=g.nr;$('gszt').value=g.sztuk;$('gzeb').value=g.zeberek||'';$('gszer').value=g.szer||'';$('gdl').value=g.dl||'';$('guw').value=g.uwagi;
 for(const b of $('grodz').children)b.className=b.textContent==g.rodzaj?'on':'';if(!$('grodz').querySelector('.on'))$('grodz').children[0].className='on';
 $('edg').style.display='block';$('edg').innerHTML='Poprawiasz wpis: <b>'+esc(g.nr)+'</b> ('+g.godz+') &nbsp; <a href="#" style="color:#9cf" onclick="koniecEdG();return false">anuluj</a>';
 $('bzg').textContent='ZAPISZ POPRAWKĘ';tab(5)}
function usGrz(i){const g=podgl.grz[i];if(!confirm('Usunąć wpis grzejnika w „'+g.nr+'”?'))return;
 cmd('grzejnik?usun=1&n='+encodeURIComponent(podgl.n)+'&id='+g.id,()=>sesja(encodeURIComponent(podgl.n)))}
function edPom(i){const p=podgl.pom[i];ed={n:podgl.n,id:p.id};$('pnr').value=p.nr;$('ppkt').value=p.punktow||'';$('pmoc').value=p.moc||'';$('popr').value=p.opraw;$('puw').value=p.uwagi;
 for(const b of $('ptyp').children)b.className=(b.textContent==p.typ||(!p.typ&&b==$('ptyp').children[0]))?'on':'';
 $('edb').style.display='block';$('edb').innerHTML='Poprawiasz wpis: <b>'+esc(p.nr)+'</b> ('+p.godz+') &nbsp; <a href="#" style="color:#9cf" onclick="koniecEd();return false">anuluj</a>';
 $('bzp').textContent='ZAPISZ POPRAWKĘ';tab(2)}
function usPom(i){const p=podgl.pom[i];if(!confirm('Usunąć wpis pomieszczenia „'+p.nr+'”?'))return;
 cmd('pomieszczenie?usun=1&n='+encodeURIComponent(podgl.n)+'&id='+p.id,()=>sesja(encodeURIComponent(podgl.n)))}
function edZn(i){const z=podgl.znaczniki[i],t=prompt('Treść znacznika:',z.opis);if(t===null)return;
 cmd('znacznik_zmien?n='+encodeURIComponent(podgl.n)+'&id='+z.id+'&opis='+encodeURIComponent(t),()=>sesja(encodeURIComponent(podgl.n)))}
function usZn(i){const z=podgl.znaczniki[i];if(!confirm('Usunąć znacznik „'+z.opis+'”?'))return;
 cmd('znacznik_zmien?usun=1&n='+encodeURIComponent(podgl.n)+'&id='+z.id,()=>sesja(encodeURIComponent(podgl.n)))}
function opisPom(p){return (p.opraw?p.opraw+' × '+esc(p.typ)+(p.punktow?' ('+p.punktow+' pkt)':'')+(p.moc?' × '+p.moc+' W = '+Math.round(p.moc_razem)+' W':''):'bez opraw')}
let podgl=null;
let kopOK=false,ustW=false;
function esc(t){return String(t).replace(/[&<>]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[m]))}
let rozw={};
function sesje(){fetch('/sesje').then(r=>r.json()).then(l=>{$('sd').innerHTML='';otw='';const g={};
 for(const x of l){const b=x.bud||'(bez nazwy budynku)';(g[b]=g[b]||[]).push(x)}
 $('budl').innerHTML=Object.keys(g).filter(b=>b[0]!='(').map(b=>'<option value="'+esc(b).replace(/"/g,'&quot;')+'">').join('');
 const B=Object.keys(g);window._bud=B;
 $('sl').innerHTML=B.length?B.map((b,bi)=>{const w=g[b],sum=k=>w.reduce((a,x)=>a+x[k],0),o=rozw[b];
  return '<div class="r" style="background:#1c2430;padding:8px;border-radius:8px;margin-top:6px"><span onclick="rozw[_bud['+bi+']]=!rozw[_bud['+bi+']];sesje()"><b>'+(o?'▾ ':'▸ ')+esc(b)+'</b><br>'+
   'nagrań: '+w.length+' · '+sum('mb').toFixed(1)+' MB · pomieszczeń: '+sum('pom')+' · grzejników: '+sum('grz')+' · okien/drzwi: '+sum('okn')+' · opraw: '+sum('opraw')+(sum('w')?' · '+sum('w')+' W':'')+'</span>'+
   (b[0]=='('?'':'<button style="background:#933" onclick="usunBud('+bi+')">Usuń budynek</button>')+'</div>'+
   (o?w.slice().sort((a,c)=>(a.kond||'').localeCompare(c.kond||'')||a.n.localeCompare(c.n)).map(x=>
   '<div class="r" style="padding-left:14px"><span><b>'+esc(x.kond||x.n)+'</b> · '+x.data+(x.aktywna?' · <b class="ok">NAGRYWA</b>':'')+'<br>'+x.mb+' MB · znaczniki: '+x.zn+' · pomieszczenia: '+x.pom+' · grzejniki: '+x.grz+' · okna: '+x.okn+(x.w?' · '+x.w+' W':'')+'</span>'+
   '<button data-n="'+encodeURIComponent(x.n)+'" onclick="sesja(this.dataset.n)">Podgląd</button>'+
   (x.aktywna?'':'<button data-n="'+encodeURIComponent(x.n)+'" style="background:#933" onclick="usun(this.dataset.n)">Usuń</button>')+'</div>').join(''):'')}).join(''):'brak zapisanych danych'})}
function usunBud(bi){const b=_bud[bi];
 if(!confirm('Usunąć z karty WSZYSTKIE nagrania budynku\n'+b+' ?\n\n'+(kopOK?'Kopia na serwerze zostaje.':'UWAGA: brak potwierdzonej kopii na serwerze – dane mogą przepaść bezpowrotnie!')))return;
 if(!confirm('Na pewno usunąć cały budynek „'+b+'”? Tego nie da się cofnąć.'))return;cmd('usun_budynek?b='+encodeURIComponent(b),sesje)}
function sesja(n){fetch('/sesja?n='+n).then(r=>r.json()).then(t=>{if(t.blad){alert(t.blad);return}podgl=t;
 const prz='<button onclick="';
 $('sd').innerHTML='<b>'+esc(t.n)+'</b><pre>'+esc(t.info)+'</pre><b>Pomieszczenia ('+t.pom.length+')</b>'+
 (t.pom.map((p,i)=>'<div class="r"><span>'+p.godz+' '+(p.kond?'['+esc(p.kond)+'] ':'')+'<b>'+esc(p.nr)+'</b> – '+opisPom(p)+(p.uwagi?'<br>'+esc(p.uwagi):'')+'</span>'+
  prz+'edPom('+i+')">Edytuj</button>'+prz+'usPom('+i+')" style="background:#933">Usuń</button></div>').join('')||'<div>brak</div>')+
 '<br><b>Grzejniki ('+(t.grz||[]).length+')</b>'+
 ((t.grz||[]).map((g,i)=>'<div class="r"><span>'+g.godz+' '+(g.kond?'['+esc(g.kond)+'] ':'')+'<b>'+esc(g.nr)+'</b> – '+opisGrz(g)+(g.uwagi?'<br>'+esc(g.uwagi):'')+'</span>'+
  prz+'edGrz('+i+')">Edytuj</button>'+prz+'usGrz('+i+')" style="background:#933">Usuń</button></div>').join('')||'<div>brak</div>')+
 '<br><b>Zdjęcia ('+(t.foto||[]).length+')</b><div>'+((t.foto||[]).map(p=>'<a href="/foto?n='+encodeURIComponent(t.n)+'&p='+p+'" target="_blank" style="color:#9cf;display:inline-block;padding:6px 8px 6px 0">'+p.slice(5,7)+':'+p.slice(7,9)+':'+p.slice(9,11)+'</a>').join(' ')||'brak')+'</div>'+
 '<br><b>Okna i drzwi zewn. ('+(t.okn||[]).length+')</b>'+
 ((t.okn||[]).map((o,i)=>'<div class="r"><span>'+o.godz+' '+(o.kond?'['+esc(o.kond)+'] ':'')+'<b>'+esc(o.nr)+'</b> – '+opisOkn(o)+(o.uwagi?'<br>'+esc(o.uwagi):'')+'</span>'+
  prz+'edOkn('+i+')">Edytuj</button>'+prz+'usOkn('+i+')" style="background:#933">Usuń</button></div>').join('')||'<div>brak</div>')+
 '<br><b>Znaczniki</b>'+(t.znaczniki.map((z,i)=>'<div class="r"><span'+(z.sys?' style="color:#888"':'')+'>'+z.godz+' '+esc(z.opis)+'</span>'+
  (z.sys?'':prz+'edZn('+i+')">Edytuj</button>'+prz+'usZn('+i+')" style="background:#933">Usuń</button>')+'</div>').join('')||'<div>brak</div>');
 if(otw!=t.n){otw=t.n;$('sd').scrollIntoView({behavior:'smooth'})}})}
function usun(n){const nz=decodeURIComponent(n);
 if(!confirm('Usunąć z karty sesję\n'+nz+' ?\n\n'+(kopOK?'Kopia na serwerze zostaje.':'UWAGA: brak potwierdzonej kopii na serwerze – dane mogą przepaść bezpowrotnie!')))return;
 if(!confirm('Na pewno? Tego nie da się cofnąć.'))return;cmd('usun?n='+n,sesje)}
let sel=null,ostS=null;
c.addEventListener('click',ev=>{if(!ostS||!ostS.nagrywa){return}const r=c.getBoundingClientRect(),R=+$('zas').value,sk=Math.min(r.width,r.height)/2/R;
 let x=(ev.clientX-r.left-r.width/2)/sk,y=(r.height/2-(ev.clientY-r.top))/sk,najl=null,dm=0.35;
 for(const p of ostS.obrot||[]){const a=p[0]*Math.PI/180,d=p[1]/1000,dd=Math.hypot(d*Math.sin(a)-x,d*Math.cos(a)-y);if(dd<dm){dm=dd;najl=p}}
 sel=najl?{kat:najl[0],odl:najl[1],lid:true}:{kat:(Math.atan2(x,y)*180/Math.PI+360)%360,odl:Math.round(Math.hypot(x,y)*1000),lid:false};
 $('pkt').style.display='block';$('pktet').textContent='Punkt: '+(sel.odl/1000).toFixed(2)+' m od skanera, kierunek '+sel.kat.toFixed(0)+'°'+(sel.lid?' (przyciągnięty do pomiaru)':' (wolne miejsce)');wibr()});
function zapPkt(){if(!sel)return;const o=($('popis').value||'punkt')+($('psr').value?' Ø'+$('psr').value:'');
 cmd('znacznik?opis='+encodeURIComponent('PUNKT kat='+sel.kat.toFixed(1)+' odl='+sel.odl+' '+o),()=>{sel=null;$('pkt').style.display='none';$('psr').value='';$('popis').value='';wibr()})}
function rys(s){ostS=s;const dp=devicePixelRatio||1,W=c.width=c.clientWidth*dp,H=c.height=c.clientHeight*dp;
 const R=+$('zas').value,sk=Math.min(W,H)/2/R,cx=W/2,cy=H/2;g.fillStyle='#000';g.fillRect(0,0,W,H);
 g.strokeStyle='#263';g.fillStyle='#484';g.lineWidth=1;g.font=11*dp+'px sans-serif';
 for(let r=1;r<=R;r+=(R>6?(R>12?5:2):1)){g.beginPath();g.arc(cx,cy,r*sk,0,7);g.stroke();g.fillText(r+' m',cx+3,cy-r*sk+12*dp)}
 g.beginPath();g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy);g.stroke();
 g.fillStyle='#fff';const p=s.obrot||[],k=2*dp;
 for(let i=0;i<p.length;i++){const a=p[i][0]*Math.PI/180,d=p[i][1]/1000;g.fillRect(cx+d*Math.sin(a)*sk-k/2,cy-d*Math.cos(a)*sk-k/2,k,k)}
 if(sel){const a=sel.kat*Math.PI/180,d=sel.odl/1000;g.strokeStyle='#fd0';g.lineWidth=2*dp;g.beginPath();g.arc(cx+d*Math.sin(a)*sk,cy-d*Math.cos(a)*sk,9*dp,0,7);g.stroke()}
 g.fillStyle='#f33';g.beginPath();g.moveTo(cx,cy-9*dp);g.lineTo(cx-5*dp,cy+5*dp);g.lineTo(cx+5*dp,cy+5*dp);g.fill()}
function kl(e,t,k){e.textContent=t;e.className=k||''}
function fmt(t){const m=Math.floor(t/60),s=Math.floor(t%60);return m+':'+(s<10?'0':'')+s}
function odswiez(){fetch('/stan').then(r=>r.json()).then(s=>{rys(s);przekroj(s);
 kopOK=!!(s.kopia&&s.kopia.ok&&s.kopia.wiek<300);$('bud').disabled=s.nagrywa;
 if(!ustW&&s.ust){ustW=true;if(!$('bud').value)$('bud').value=s.ust.budynek||'';if(!$('kon').value)$('kon').value=s.ust.kondygnacja||''}
 if(s.nagrywa&&document.activeElement!=$('kon')&&!$('kon').dataset.e){$('bud').value=s.budynek;$('kon').value=s.kondygnacja;$('kon').dataset.e=1}
 if(!s.nagrywa)delete $('kon').dataset.e;
 kl($('nag'),!s.nagrywa?'stop':s.pauza?'PAUZA':s.prz_faza?'PRZEKRÓJ':s.auto_pauza?'PRZECHYŁ':'● '+fmt(s.czas),!s.nagrywa?'uw':s.pauza?'uw':s.auto_pauza?'zle':'ok');
 $('bpa').textContent=s.pauza?'WZNÓW (plecak założony, stoję w tym samym miejscu)':'PAUZA (zdejmuję plecak / schylam się)';document.body.classList.toggle('nagr',s.nagrywa);$('bpa').style.background=s.pauza?'#1f9d68':'#b70';
 
 kl($('lid'),s.lidar_ok?s.hz.toFixed(1)+' Hz':'BRAK',s.lidar_ok?'ok':'zle');$('lid2').textContent=s.pkt_obrot+' pkt/obrót';$('fn').textContent=s.foto||'';
 $('l2').style.display=s.lidar2_wl?'':'none';if(s.lidar2_wl)kl($('l2s'),s.lidar2_ok?'działa':'BRAK',s.lidar2_ok?'ok':'zle');
 if(s.pitch===null){kl($('prz'),s.imu_wl?'BRAK':'wył.',s.imu_wl?'zle':'')}else{const m=s.przechyl;kl($('prz'),m.toFixed(1)+'°'+(s.imu_tryb==='shtp'?' żyro':''),m>15?'zle':m>8?'uw':'ok')}
 $('prz2').textContent=s.max_przechyl.toFixed(1)+'°';
 $('ip').textContent=s.ip?s.ip.split(' ')[0]+':8080':'–';$('p0').textContent=s.pion0[0].toFixed(1)+'° / '+s.pion0[1].toFixed(1)+'°';
 if(s.wys===null){kl($('wys'),s.baro_wl?'BRAK':'wył.',s.baro_wl?'zle':'');$('wys2').textContent=''}else{kl($('wys'),(s.wys>=0?'+':'')+s.wys.toFixed(1)+' m');$('wys2').textContent='temp. '+s.temp.toFixed(0)+' °C'}
 $('roz').textContent=(s.rozmiar/1e6).toFixed(1)+' MB';$('zn').textContent='znaczniki: '+s.znaczniki+(s.prz_nr?' · przekroje: '+s.prz_nr:'');$('ses').textContent=(s.nazwa||'')+(s.nagrywa&&s.kondygnacja?' · kond. '+s.kondygnacja:'');
 kl($('dysk'),(s.wolne/1e9).toFixed(1)+' GB',s.wolne<1e9?'zle':'ok');
 if(!s.kopia){kl($('kop'),'wyłączona');$('kop2').textContent=''}else{const w=s.kopia.wiek,st=w>300;
  kl($('kop'),s.kopia.ok?(st?'brak od '+Math.round(w/60)+' min':'OK'):'BŁĄD',s.kopia.ok&&!st?'ok':s.kopia.ok?'uw':'zle');
  $('kop2').textContent=s.kopia.ok?'ostatnia '+(w<90?w+' s':Math.round(w/60)+' min')+' temu':'sprawdź zasięg i serwer – dane są na karcie'}
 if(!s.pendrive){kl($('pen'),'wyłączona');$('pen2').textContent=''}else{const p=s.pendrive,st=p.wiek>300;
  kl($('pen'),p.wynik=='BRAK'?'BRAK PENDRIVE':!p.ok?'BŁĄD':st?'brak od '+Math.round(p.wiek/60)+' min':'OK',p.ok&&!st?'ok':'zle');
  $('pen2').textContent=p.ok?'ostatnia '+(p.wiek<90?p.wiek+' s':Math.round(p.wiek/60)+' min')+' temu':'dane są na karcie'}
 $('plista').innerHTML=(s.pom_n?'<b>Pomieszczeń: '+s.pom_sal+', opraw: '+s.pom_opraw+', punktów świetlnych: '+s.pom_pkt+(s.pom_w?', moc: '+s.pom_w+' W':'')+'</b><br>':'')+
  (s.pom||[]).slice().reverse().map(x=>esc(x.nr)+' – '+opisPom(x)).join('<br>');aktN=s.sesja_n||'';$('bed').style.display=aktN?'':'none';
 $('olista').innerHTML=(s.okn_szt?'<b>Okien i drzwi w tej sesji: '+s.okn_szt+'</b><br>':'')+(s.okn||[]).slice().reverse().map(x=>esc(x.nr)+' – '+opisOkn(x)).join('<br>');
 $('glista').innerHTML=(s.grz_szt?'<b>Grzejników w tej sesji: '+s.grz_szt+'</b><br>':'')+(s.grz||[]).slice().reverse().map(x=>esc(x.nr)+' – '+opisGrz(x)).join('<br>');
 $('kom').textContent=s.komunikat||''}).catch(()=>{$('kom').textContent='Brak połączenia z Raspberry Pi'})}
let t0=1;try{t0=+localStorage.getItem('tab')||1}catch(e){}tab(t0);setInterval(odswiez,500);odswiez();sesje();
let radarZajety=false;setInterval(()=>{if(radarZajety||document.hidden||!ostS||$('t1').className!='on')return;radarZajety=true;
 fetch('/obrot').then(r=>r.json()).then(j=>{if(ostS){ostS.obrot=j.obrot;rys(ostS)}}).catch(()=>{}).finally(()=>{radarZajety=false})},120);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, kod=200):
        b = json.dumps(obj).encode()
        self.send_response(kod)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/obrot":                 # lekki podglad radaru - tylko ostatni obrot (strona pyta ok. 8 razy/s)
            with LOCK:
                o = STAN["obrot"]
            return self._json({"obrot": o})
        if u.path == "/stan":
            teraz = time.time()
            with LOCK:
                s = {k: STAN[k] for k in ("nagrywa", "nazwa", "hz", "pkt_obrot", "yaw", "pitch", "roll",
                                          "max_przechyl", "wys", "temp", "znaczniki", "komunikat", "pauza", "auto_pauza",
                                          "przechyl", "prz_faza", "prz_zostalo", "prz_nr", "prz_wynik",
                                          "budynek", "kondygnacja", "ip")}
                s["pion0"] = [round(v, 1) for v in STAN["pion0"]]
                s["imu_tryb"] = STAN.get("imu_tryb", "")
                s["obrot"] = STAN["obrot"]
                s["pom"] = STAN["pom"][-5:]
                s["grz"] = STAN["grz"][-5:]
                s["okn"] = STAN["okn"][-4:]
                s["okn_szt"] = sum(x["sztuk"] for x in STAN["okn"])
                s["grz_szt"] = sum(x["sztuk"] for x in STAN["grz"])
                s["sesja_n"] = os.path.basename(STAN["katalog"]) if STAN["nagrywa"] else ""
                s["pom_n"] = len(STAN["pom"])
                s["pom_w"] = round(sum(x.get("moc_razem", 0) for x in STAN["pom"]))
                s["pom_opraw"] = sum(x["opraw"] for x in STAN["pom"])
                s["pom_pkt"] = sum(x["opraw"] * x["punktow"] for x in STAN["pom"])
                s["pom_sal"] = len({x["nr"] for x in STAN["pom"]})
                s["czas"] = teraz - STAN["t_start"] if STAN["nagrywa"] else 0
                s["rozmiar"] = sum(STAN["bajty"].values())
                s["lidar_ok"] = teraz - STAN["ost"][STAN["podglad"]] < 1.5
                s["lidar2_wl"] = bool(ARGS.lidar2)
                drugi = "lidar2" if STAN["podglad"] == "lidar" else "lidar"
                s["lidar2_ok"] = teraz - STAN["ost"][drugi] < 1.5
                s["podglad"] = STAN["podglad"]
                s["foto"] = STAN["foto"]
                s["imu_ok"] = teraz - STAN["ost"]["imu"] < 1.5
            s["imu_wl"], s["baro_wl"] = not ARGS.bez_imu, not ARGS.bez_baro
            if not s["imu_ok"]:
                s["pitch"] = s["roll"] = None
            if teraz - STAN["ost"]["baro"] > 3:
                s["wys"] = None
            s["wolne"] = shutil.disk_usage(ARGS.katalog).free
            if not s["nagrywa"]:
                s["ust"] = ustawienia()
            for klucz, plik in (("kopia", ".sync_status"), ("pendrive", ".sync_pendrive")):   # stan kopii (sync.sh)
                s[klucz] = None
                try:
                    with open(os.path.join(ARGS.katalog, plik)) as f:
                        t, wynik = f.read().split()[:2]
                    s[klucz] = {"wiek": int(teraz - int(t)), "ok": wynik == "OK", "wynik": wynik}
                except (OSError, ValueError):
                    pass
            self._json(s)
        elif u.path == "/foto":
            qq = parse_qs(u.query)
            kat, plik = sesja_sciezka(qq.get("n", [""])[0]), qq.get("p", [""])[0]
            sc = os.path.join(kat, plik) if kat and re.fullmatch(r"foto_[0-9_]+\.jpg", plik) else ""
            if not sc or not os.path.isfile(sc):
                self._json({"blad": "Nie ma takiego zdjęcia"}, 404)
                return
            with open(sc, "rb") as f:
                b = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif u.path == "/typy_okien":
            self._json(typy_okien(parse_qs(u.query).get("b", [""])[0]))
        elif u.path == "/sesje":
            self._json(lista_sesji())
        elif u.path == "/sesja":
            t = sesja_tresc(parse_qs(u.query).get("n", [""])[0])
            self._json(t if t else {"blad": "Nie ma takiej sesji"}, 200 if t else 404)
        elif u.path == "/":
            b = STRONA.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        else:
            self._json({"blad": "nie ma takiej strony"}, 404)

    def do_POST(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/start":
            ok = start(q.get("budynek", [""])[0], q.get("kondygnacja", [""])[0])
            self._json({"ok": ok} if ok else {"blad": "Nagrywanie już trwa"})
        elif u.path == "/stop":
            self._json({"ok": stop()})
        elif u.path == "/znacznik":
            ok = znacznik(q.get("opis", [""])[0])
            self._json({"ok": ok} if ok else {"blad": "Znacznik działa tylko podczas nagrywania"})
        elif u.path == "/pomieszczenie":
            g = lambda k: q.get(k, [""])[0]
            blad = pomieszczenie(g("nr"), g("typ"), g("punktow"), g("opraw"), g("uwagi"), g("moc"),
                                 n=g("n") or None, idp=g("id") or None, usun=g("usun") == "1")
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/grzejnik":
            g = lambda k: q.get(k, [""])[0]
            blad = grzejnik(g("nr"), g("rodzaj"), g("sztuk"), g("zeberek"), g("szer"), g("dl"), g("uwagi"),
                            n=g("n") or None, idg=g("id") or None, usun=g("usun") == "1")
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/okno":
            g = lambda k: q.get(k, [""])[0]
            blad = okno(g("nr"), g("rodzaj"), g("szer"), g("wys"), g("rama"), g("szyby"), g("sztuk"), g("uwagi"),
                        n=g("n") or None, ido=g("id") or None, usun=g("usun") == "1")
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/znacznik_zmien":
            g = lambda k: q.get(k, [None])[0]
            blad = znacznik_zmien(g("n") or "", g("id") or "", None if g("usun") == "1" else (g("opis") or ""))
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/kondygnacja":
            self._json({"ok": zmien_kondygnacje(q.get("k", [""])[0])})
        elif u.path == "/zdjecie":
            try:
                ile = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                ile = 0
            if not 0 < ile <= 8 * 1024 * 1024:
                self._json({"blad": "Zdjęcie jest puste albo za duże (limit 8 MB)"})
                return
            blad = zapisz_zdjecie(self.rfile.read(ile), q.get("opis", [""])[0])
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/zamien_lidary":
            blad = zamien_lidary()
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/usun_budynek":
            self._json({"ok": True, "usunieto": usun_budynek(q.get("b", [""])[0])})
        elif u.path == "/usun":
            blad = usun_sesje(q.get("n", [""])[0])
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/wyslij_teraz":
            # reczna wysylka: sync.sh porownuje rozmiary plikow z serwerem i dosyla wszystko, czego tam brakuje
            sk = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sync.sh")
            if not os.path.exists(os.path.join(os.path.dirname(sk), "sync.conf")):
                self._json({"blad": "Kopia na serwer nie jest skonfigurowana (brak sync.conf) – dane pobiera Dell sam, gdy malina jest w sieci"})
            else:
                subprocess.Popen(["bash", sk], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                self._json({"ok": True})
        elif u.path == "/pauza":
            r = pauza()
            self._json({"ok": True, "pauza": r} if r is not None else {"blad": "Pauza działa tylko podczas nagrywania"})
        elif u.path == "/zeruj":
            with LOCK:
                STAN["p0"] = None
            self._json({"ok": True})
        elif u.path == "/zeruj_pion":
            w, blad = zeruj_pion()
            self._json({"blad": blad} if blad else {"ok": True, "pitch0": round(w[0], 1), "roll0": round(w[1], 1)})
        elif u.path == "/wylacz":
            stop()
            self._json({"ok": True})
            subprocess.Popen(["sudo", "poweroff"])
        else:
            self._json({"blad": "nieznane polecenie"}, 404)


def main():
    global ARGS
    ap = argparse.ArgumentParser(description="Rejestrator LiDAR z podgladem WWW")
    ap.add_argument("--lidar", default="/dev/ttyUSB0")
    ap.add_argument("--lidar-baud", type=int, default=230400)
    ap.add_argument("--lidar2", default="auto", help="port drugiego (pionowego) LiDAR-u; auto = /dev/ttyUSB1, jesli istnieje; pusty = brak")
    ap.add_argument("--imu", default="/dev/ttyACM0")
    ap.add_argument("--imu-baud", type=int, default=115200, help="tylko tryb RVC")
    ap.add_argument("--imu-tryb", default="auto", choices=("auto", "rvc", "shtp"),
                    help="BNO085: shtp = surowy zyroskop (mostek P1), rvc = mostek P0; auto rozpoznaje")
    ap.add_argument("--bez-imu", action="store_true")
    ap.add_argument("--baro-adres", type=lambda x: int(x, 0), default=0x76)
    ap.add_argument("--bez-baro", action="store_true")
    ap.add_argument("--katalog", default=os.path.expanduser("~/skany"))
    ap.add_argument("--port", type=int, default=8080)
    ARGS = ap.parse_args()
    os.makedirs(ARGS.katalog, exist_ok=True)
    if ARGS.lidar2 == "auto":
        ARGS.lidar2 = "/dev/ttyUSB1" if os.path.exists("/dev/ttyUSB1") and ARGS.lidar != "/dev/ttyUSB1" else ""
    wczytaj_pion0()
    watki = [(watek_lidar, (ARGS.lidar, ARGS.lidar_baud)), (watek_flush, ()), (adres_ip, ())]
    if ARGS.lidar2:
        watki.append((watek_lidar, (ARGS.lidar2, ARGS.lidar_baud, "lidar2")))
        if ustawienia().get("podglad") in ("lidar", "lidar2"):
            STAN["podglad"] = ustawienia()["podglad"]
    if not ARGS.bez_imu:
        watki.append((watek_imu, (ARGS.imu, ARGS.imu_baud)))
    if not ARGS.bez_baro:
        watki.append((watek_baro, (ARGS.baro_adres,)))
    for f, a in watki:
        threading.Thread(target=f, args=a, daemon=True).start()
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", ARGS.port), Handler)
    except OSError as e:
        sys.exit(f"Nie moge otworzyc portu {ARGS.port}: {e}\n"
                 "Prawdopodobnie dziala juz inna kopia rejestratora. Zamknij ja: pkill -9 -f skan_web.py\n"
                 "(albo, przy wlaczonym autostarcie: sudo systemctl stop skaner)")
    print(f"Podglad: http://<adres-IP>:{ARGS.port}   (Ctrl+C konczy)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    KONIEC.set()
    stop()


if __name__ == "__main__":
    main()

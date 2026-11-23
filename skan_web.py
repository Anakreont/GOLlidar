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
import argparse, json, math, os, shutil, struct, subprocess, sys, threading, time
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
    "bajty": {"lidar": 0, "imu": 0, "baro": 0},
    "ost": {"lidar": 0.0, "imu": 0.0, "baro": 0.0},        # czas ostatnich danych
    "obrot": [], "hz": 0.0, "pkt_obrot": 0,
    "yaw": None, "pitch": None, "roll": None, "max_przechyl": 0.0,
    "wys": None, "p0": None, "temp": None,
    "znaczniki": 0, "komunikat": "",
    "pauza": False, "auto_pauza": False,
    "pom": [],                 # pomieszczenia zapisane w biezacej sesji
    "budynek": "", "kondygnacja": "",
    "przechyl": None,          # kat odchylenia od pionu [st.]
    "prz_faza": 0,             # 0 brak, 1 trzymaj nieruchomo, 2 przekroj zapisany (postaw do pionu)
    "prz_zostalo": 0.0,        # sekundy do zapisania przekroju
    "prz_nr": 0,               # liczba przekrojow w sesji
    "prz_wynik": "",           # wymiary ostatniego przekroju (szacunek na zywo)
    "prz_skany": [],           # obroty zebrane podczas trzymania
}
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
def watek_lidar(port, baud):
    buf = bytearray()
    obrot, ost_kat, t_obr = [], None, time.time()
    while not KONIEC.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.05) as s:
                STAN["komunikat"] = ""
                while not KONIEC.is_set():
                    d = s.read(4096)
                    if not d:
                        continue
                    zapisz("lidar", d)
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
def watek_imu(port, baud):
    buf = bytearray()
    prz = Przekroj()
    while not KONIEC.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.05) as s:
                while not KONIEC.is_set():
                    d = s.read(1024)
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
                        with LOCK:
                            STAN["yaw"], STAN["pitch"], STAN["roll"] = yaw / 100, pitch / 100, roll / 100
                            m = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(pitch / 100)) *
                                                                         math.cos(math.radians(roll / 100))))))
                            STAN["przechyl"] = m
                            if STAN["nagrywa"]:
                                STAN["max_przechyl"] = max(STAN["max_przechyl"], m)
                            zmiana = None
                            if m > PROG_PRZECHYLU and not STAN["auto_pauza"]:
                                STAN["auto_pauza"], zmiana = True, "AUTO_PRZECHYL_START"
                            elif m < PROG_PRZECHYLU - 3 and STAN["auto_pauza"]:
                                STAN["auto_pauza"], zmiana = False, "AUTO_PRZECHYL_KONIEC"
                            teraz = time.time()
                            zdarzenia = prz.krok(teraz, m) if STAN["nagrywa"] and not STAN["pauza"] else []
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
        except (serial.SerialException, OSError):
            time.sleep(1)


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
                    f"baro=0x{ARGS.baro_adres:02x}\n")
        for s in ("lidar", "imu", "baro"):
            PLIKI[s] = open(os.path.join(kat, s + ".bin"), "ab")
            STAN["bajty"][s] = 0
        kal = STAN.get("_baro_kal")
        if kal:
            PLIKI["baro"].write(struct.pack("<QH", time.time_ns(), len(kal)) + kal)
        STAN.update(nagrywa=True, nazwa=nazwa, katalog=kat, t_start=time.time(), znaczniki=0, max_przechyl=0.0, pom=[], prz_nr=0, prz_wynik="", prz_faza=0,
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


def pomieszczenie(nr, typ, punktow, opraw, uwagi, moc=""):
    """Dopisuje wiersz do pomieszczenia.csv w katalogu sesji (inwentaryzacja oswietlenia)."""
    try:
        opraw = int(opraw or 0)
        punktow = int(punktow or 0)
        moc = float(str(moc or 0).replace(",", "."))
    except ValueError:
        return "Liczba opraw, liczba punktów i moc muszą być liczbami"
    if moc < 0:
        return "Liczby nie mogą być ujemne"
    if opraw < 0 or punktow < 0:
        return "Liczby nie mogą być ujemne"
    if not czysc(nr):
        return "Podaj nazwę lub numer pomieszczenia"
    if opraw == 0:
        typ, punktow, moc = "", 0, 0.0            # pomieszczenie bez opraw: sama nazwa i uwagi
    moc_razem = moc * max(punktow, 1) * opraw
    lw = lambda x: (f"{x:.1f}".rstrip("0").rstrip(".").replace(".", ",")) if x else ""
    with LOCK:
        if not STAN["nagrywa"]:
            return "Zapis pomieszczeń działa tylko podczas nagrywania"
        sciezka = os.path.join(STAN["katalog"], "pomieszczenia.csv")
        nowy = not os.path.exists(sciezka)
        with open(sciezka, "a", encoding="utf-8-sig" if nowy else "utf-8") as f:
            if nowy:
                f.write("czas_ns;godzina;pomieszczenie;rodzaj_oprawy;punktow_w_oprawie;liczba_opraw;punktow_razem;uwagi;wysokosc_wzgl_m;budynek;kondygnacja;moc_punktu_W;moc_razem_W\n")
            wys = "" if STAN["wys"] is None else f"{STAN['wys']:.1f}".replace(".", ",")
            f.write(f"{time.time_ns()};{datetime.now().strftime('%H:%M:%S')};{czysc(nr)};{czysc(typ)};"
                    f"{punktow or ''};{opraw};{punktow * opraw or ''};{czysc(uwagi)};{wys};"
                    f"{STAN['budynek']};{STAN['kondygnacja']};{lw(moc)};{lw(moc_razem)}\n")
        STAN["pom"].append({"nr": czysc(nr), "typ": czysc(typ), "punktow": punktow, "opraw": opraw, "moc": moc, "moc_razem": moc_razem})
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


def lista_sesji():
    wyn = []
    for n in sorted(os.listdir(ARGS.katalog), reverse=True):
        p = sesja_sciezka(n)
        if not p:
            continue
        def linie(plik):
            try:
                with open(os.path.join(p, plik), encoding="utf-8-sig") as f:
                    return sum(1 for _ in f)
            except OSError:
                return 0
        wyn.append({"n": n, "mb": round(sum(os.path.getsize(os.path.join(p, x)) for x in os.listdir(p)) / 1e6, 1),
                    "zn": linie("znaczniki.txt"), "pom": max(0, linie("pomieszczenia.csv") - 1),
                    "aktywna": STAN["nagrywa"] and os.path.abspath(p) == os.path.abspath(STAN["katalog"])})
    return wyn


def sesja_tresc(n):
    p = sesja_sciezka(n)
    if not p:
        return None
    def czytaj(plik):
        try:
            with open(os.path.join(p, plik), encoding="utf-8-sig") as f:
                return f.read()[-20000:]
        except OSError:
            return ""
    zn = []
    for w in czytaj("znaczniki.txt").splitlines():
        cz = w.split("\t")
        try:
            zn.append(datetime.fromtimestamp(int(cz[0]) / 1e9).strftime("%H:%M:%S") + "  " + (cz[1] if len(cz) > 1 else ""))
        except ValueError:
            zn.append(w)
    pom = [w.split(";") for w in czytaj("pomieszczenia.csv").splitlines()[1:]]
    return {"n": n, "info": czytaj("info.txt"), "znaczniki": zn,
            "pom": [{"godz": w[1], "nr": w[2], "typ": w[3], "pkt": w[4], "opraw": w[5], "uwagi": w[7],
                     "kond": w[10] if len(w) > 10 else "", "moc": w[11] if len(w) > 11 else ""} for w in pom if len(w) >= 8]}


def usun_sesje(n):
    p = sesja_sciezka(n)
    if not p:
        return "Nie ma takiej sesji"
    with LOCK:
        if STAN["nagrywa"] and os.path.abspath(p) == os.path.abspath(STAN["katalog"]):
            return "Nie można usunąć sesji, która właśnie się nagrywa"
    shutil.rmtree(p)
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
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;font-size:16px;padding-bottom:24px}
#pasek{position:sticky;top:0;z-index:5;background:#181818;display:flex;gap:4px;padding:6px;border-bottom:1px solid #333}
#pasek div{flex:1;text-align:center;border-radius:8px;background:#242424;padding:5px 2px;min-width:0}
#pasek small{display:block;color:#999;font-size:10px;line-height:1.1}
#pasek b{font-size:15px;white-space:nowrap}
.ok{color:#5d5}.zle{color:#f55}.uw{color:#fb3}
#mapa{position:relative}
#c{width:100vw;height:min(100vw,40vh);display:block;background:#000}
#zas{position:absolute;right:6px;top:6px;width:auto;font-size:15px;padding:6px 8px;border-radius:8px;border:0;background:#333c;color:#fff}
#prk{display:none;position:absolute;left:0;right:0;bottom:0;padding:12px 10px;text-align:center;font-size:20px;font-weight:700;color:#fff}
#prk small{display:block;font-weight:400;font-size:14px}
.lst{padding:2px 10px;font-size:14px;line-height:1.5}
.lst .r{display:flex;gap:8px;align-items:center;border-bottom:1px solid #333;padding:6px 0}
.lst .r span{flex:1;min-width:0;word-break:break-all}
.lst .r button{flex:0 0 auto;min-height:40px;font-size:14px;font-weight:400;padding:0 12px;background:#444}
.lst pre{white-space:pre-wrap;word-break:break-word;background:#1a1a1a;padding:8px;border-radius:8px;font-size:12px;margin:6px 0}
#kom{padding:4px 10px;color:#fb3;font-size:13px}
#kom:empty{display:none}
.w{padding:5px 8px;display:flex;gap:8px}
button{flex:1;font-size:18px;font-weight:600;min-height:56px;border:0;border-radius:12px;color:#fff;touch-action:manipulation}
button:active{filter:brightness(1.35)}
input,select,textarea{width:100%;font-size:18px;padding:14px 12px;border-radius:12px;border:1px solid #444;background:#222;color:#fff;font-family:inherit}
textarea{min-height:92px;resize:vertical}
h2{font-size:13px;color:#9ab;margin:14px 10px 2px;text-transform:uppercase;letter-spacing:.05em}
#bs{background:#1f9d68}#bp{background:#c33}#bpa{background:#b70}.nb{background:#2b62c9}.sz{background:#444;font-weight:400;font-size:16px}
.chip{flex:0 0 auto;min-height:44px;font-size:15px;font-weight:400;background:#333;padding:0 14px;border-radius:22px}
.chips{padding:5px 8px;display:flex;gap:8px;overflow-x:auto}
.seg button{min-height:52px;font-size:16px;font-weight:400;background:#2a2a2a;border:1px solid #444}
.seg button.on{background:#2b62c9;border-color:#2b62c9;font-weight:600}
.seg{gap:6px}
.et{font-size:13px;color:#aaa;padding:6px 12px 0}
.licz{display:flex;gap:8px;align-items:stretch;flex:1}
.licz button{flex:0 0 56px;background:#444;font-size:26px}
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
<div id="mapa"><canvas id="c"></canvas><div id="prk"></div>
 <select id="zas"><option value="3">3 m</option><option value="6" selected>6 m</option><option value="12">12 m</option><option value="20">20 m</option></select></div>
<div id="kom"></div>

<div class="w"><input id="bud" placeholder="budynek, np. SP162" autocomplete="off"></div>
<div class="w"><input id="kon" placeholder="kondygnacja, np. parter, +1, -1" autocomplete="off">
 <button id="bk" class="sz" style="flex:0 0 96px;display:none" onclick="cmd('kondygnacja?k='+encodeURIComponent($('kon').value),wibr)">Zmień</button></div>
<div class="w"><button id="bs" onclick="cmd('start?budynek='+encodeURIComponent($('bud').value)+'&kondygnacja='+encodeURIComponent($('kon').value))">START</button>
 <button id="bp" onclick="if(confirm('Zakończyć nagranie?'))cmd('stop')">STOP</button></div>
<div class="w"><button id="bpa" onclick="cmd('pauza')">PAUZA (schylam się / kucam)</button></div>

<h2>Znacznik / notatka</h2>
<div class="w"><textarea id="opis" placeholder="Notatka do tego miejsca, np. sala 12 – zabudowa na całej ścianie północnej, okna zasłonięte roletami"></textarea></div>
<div class="chips">
 <button class="chip" onclick="szybki('drzwi zamknięte')">drzwi zamknięte</button>
 <button class="chip" onclick="szybki('szafa na ścianie')">szafa na ścianie</button>
 <button class="chip" onclick="szybki('okna zasłonięte')">okna zasłonięte</button>
 <button class="chip" onclick="szybki('pomiar kontrolny')">pomiar kontrolny</button>
 <button class="chip" onclick="szybki('klatka schodowa')">klatka schodowa</button>
 <button class="chip" onclick="szybki('ludzie w kadrze')">ludzie w kadrze</button>
</div>
<div class="w"><button class="nb" onclick="zapZn()">ZAPISZ ZNACZNIK</button></div>

<h2>Pomieszczenie – nazwa i oświetlenie</h2>
<div class="w"><input id="pnr" placeholder="pomieszczenie, np. sala 12" autocomplete="off"></div>
<div class="et">Rodzaj oprawy</div>
<div class="w seg" id="ptyp"><button class="on" onclick="typ(this)">świetlówki</button><button onclick="typ(this)">LED</button>
 <button onclick="typ(this)">żarówki</button><button onclick="typ(this)">inne</button></div>
<div class="et">Punktów świetlnych w jednej oprawie</div>
<div class="w"><div class="licz"><button onclick="krok('ppkt',-1)">−</button>
 <input id="ppkt" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('ppkt',1)">+</button></div></div>
<div class="et">Moc jednego punktu świetlnego [W]</div>
<div class="w"><div class="licz"><button onclick="krok('pmoc',-1)">−</button>
 <input id="pmoc" type="number" inputmode="decimal" placeholder="0"><button onclick="krok('pmoc',1)">+</button></div></div>
<div class="chips"><button class="chip" onclick="$('pmoc').value=9">9 W</button><button class="chip" onclick="$('pmoc').value=18">18 W</button>
 <button class="chip" onclick="$('pmoc').value=36">36 W</button><button class="chip" onclick="$('pmoc').value=58">58 W</button>
 <button class="chip" onclick="$('pmoc').value=60">60 W</button><button class="chip" onclick="$('pmoc').value=100">100 W</button></div>
<div class="et">Ilość opraw w pomieszczeniu (0 = zapis samej nazwy, bez oświetlenia)</div>
<div class="w"><div class="licz"><button onclick="krok('popr',-1)">−</button>
 <input id="popr" type="number" inputmode="numeric" placeholder="0"><button onclick="krok('popr',1)">+</button></div></div>
<div class="w"><textarea id="puw" style="min-height:64px" placeholder="uwagi, np. 4×18 W, 2 nie świecą"></textarea></div>
<div class="w"><button class="nb" onclick="zapPom()">ZAPISZ POMIESZCZENIE</button></div>
<div id="plista"></div>

<h2>Zapisane sesje</h2>
<div class="w"><button class="sz" onclick="sesje()">Pokaż / odśwież listę sesji</button></div>
<div id="sl" class="lst"></div><div id="sd" class="lst"></div>

<h2>Stan</h2>
<div class="w"><div class="k"><small>Sesja</small><b id="roz">–</b><small id="zn"></small></div>
 <div class="k"><small>Karta – wolne</small><b id="dysk">–</b><small id="ses"></small></div></div>
<div class="w"><div class="k"><small>Kopia na serwer</small><b id="kop">–</b><small id="kop2"></small></div>
 <div class="k"><small>Kopia na pendrive</small><b id="pen">–</b><small id="pen2"></small></div></div>
<div class="w"><div class="k"><small>LiDAR</small><b id="lid2">–</b></div>
 <div class="k"><small>Max przechył w sesji</small><b id="prz2">–</b><small id="wys2"></small></div></div>
<div class="w"><button class="sz" onclick="if(confirm('Zerować wysokość w tym miejscu?'))cmd('zeruj')">Zeruj wysokość</button>
 <button class="sz" onclick="if(confirm('Zatrzymać zapis i wyłączyć Raspberry Pi?'))cmd('wylacz')">Wyłącz RPi</button></div>
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
function szybki(t){cmd('znacznik?opis='+encodeURIComponent(t),wibr)}
function krok(id,d){const e=$(id);e.value=Math.max(0,(parseInt(e.value)||0)+d)}
function typ(b){for(const x of $('ptyp').children)x.className='';b.className='on'}
function zapPom(){const e=encodeURIComponent,t=$('ptyp').querySelector('.on').textContent;
 cmd('pomieszczenie?nr='+e($('pnr').value)+'&typ='+e(t)+'&punktow='+e($('ppkt').value)+'&opraw='+e($('popr').value)+'&moc='+e($('pmoc').value)+'&uwagi='+e($('puw').value),
 ()=>{$('popr').value='';$('puw').value='';wibr()})}
let kopOK=false,ustW=false;
function esc(t){return String(t).replace(/[&<>]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[m]))}
function sesje(){fetch('/sesje').then(r=>r.json()).then(l=>{$('sd').innerHTML='';$('sl').innerHTML=l.length?l.map(x=>
 '<div class="r"><span><b>'+esc(x.n)+'</b><br>'+x.mb+' MB · znaczniki: '+x.zn+' · pomieszczenia: '+x.pom+(x.aktywna?' · <b class="ok">NAGRYWA</b>':'')+'</span>'+
 '<button data-n="'+encodeURIComponent(x.n)+'" onclick="sesja(this.dataset.n)">Podgląd</button>'+
 (x.aktywna?'':'<button data-n="'+encodeURIComponent(x.n)+'" style="background:#933" onclick="usun(this.dataset.n)">Usuń</button>')+'</div>').join(''):'brak sesji'})}
function sesja(n){fetch('/sesja?n='+n).then(r=>r.json()).then(t=>{if(t.blad){alert(t.blad);return}
 $('sd').innerHTML='<b>'+esc(t.n)+'</b><pre>'+esc(t.info)+'</pre><b>Pomieszczenia ('+t.pom.length+')</b><pre>'+
 (esc(t.pom.map(p=>p.godz+'  '+(p.kond?'['+p.kond+'] ':'')+p.nr+' – '+(+p.opraw?p.opraw+' × '+p.typ+(p.pkt?' ('+p.pkt+' pkt)':'')+(p.moc?' × '+p.moc+' W':''):'bez opraw')+(p.uwagi?' · '+p.uwagi:'')).join('\n'))||'brak')+
 '</pre><b>Znaczniki ('+t.znaczniki.length+')</b><pre>'+(esc(t.znaczniki.join('\n'))||'brak')+'</pre>';$('sd').scrollIntoView({behavior:'smooth'})})}
function usun(n){const nz=decodeURIComponent(n);
 if(!confirm('Usunąć z karty sesję\n'+nz+' ?\n\n'+(kopOK?'Kopia na serwerze zostaje.':'UWAGA: brak potwierdzonej kopii na serwerze – dane mogą przepaść bezpowrotnie!')))return;
 if(!confirm('Na pewno? Tego nie da się cofnąć.'))return;cmd('usun?n='+n,sesje)}
function rys(s){const dp=devicePixelRatio||1,W=c.width=c.clientWidth*dp,H=c.height=c.clientHeight*dp;
 const R=+$('zas').value,sk=Math.min(W,H)/2/R,cx=W/2,cy=H/2;g.fillStyle='#000';g.fillRect(0,0,W,H);
 g.strokeStyle='#263';g.fillStyle='#484';g.lineWidth=1;g.font=11*dp+'px sans-serif';
 for(let r=1;r<=R;r+=(R>6?(R>12?5:2):1)){g.beginPath();g.arc(cx,cy,r*sk,0,7);g.stroke();g.fillText(r+' m',cx+3,cy-r*sk+12*dp)}
 g.beginPath();g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy);g.stroke();
 g.fillStyle='#fff';const p=s.obrot||[],k=2*dp;
 for(let i=0;i<p.length;i++){const a=p[i][0]*Math.PI/180,d=p[i][1]/1000;g.fillRect(cx+d*Math.sin(a)*sk-k/2,cy-d*Math.cos(a)*sk-k/2,k,k)}
 g.fillStyle='#f33';g.beginPath();g.moveTo(cx,cy-9*dp);g.lineTo(cx-5*dp,cy+5*dp);g.lineTo(cx+5*dp,cy+5*dp);g.fill()}
function kl(e,t,k){e.textContent=t;e.className=k||''}
function fmt(t){const m=Math.floor(t/60),s=Math.floor(t%60);return m+':'+(s<10?'0':'')+s}
function odswiez(){fetch('/stan').then(r=>r.json()).then(s=>{rys(s);przekroj(s);
 kopOK=!!(s.kopia&&s.kopia.ok&&s.kopia.wiek<300);$('bk').style.display=s.nagrywa?'':'none';$('bud').disabled=s.nagrywa;
 if(!ustW&&s.ust){ustW=true;if(!$('bud').value)$('bud').value=s.ust.budynek||'';if(!$('kon').value)$('kon').value=s.ust.kondygnacja||''}
 if(s.nagrywa&&document.activeElement!=$('kon')&&!$('kon').dataset.e){$('bud').value=s.budynek;$('kon').value=s.kondygnacja;$('kon').dataset.e=1}
 if(!s.nagrywa)delete $('kon').dataset.e;
 kl($('nag'),!s.nagrywa?'stop':s.pauza?'PAUZA':s.prz_faza?'PRZEKRÓJ':s.auto_pauza?'PRZECHYŁ':'● '+fmt(s.czas),!s.nagrywa?'uw':s.pauza?'uw':s.auto_pauza?'zle':'ok');
 $('bpa').textContent=s.pauza?'WZNÓW (stoję prosto)':'PAUZA (schylam się / kucam)';$('bpa').style.background=s.pauza?'#1f9d68':'#b70';
 $('bs').style.opacity=s.nagrywa?.35:1;$('bp').style.opacity=s.nagrywa?1:.35;
 kl($('lid'),s.lidar_ok?s.hz.toFixed(1)+' Hz':'BRAK',s.lidar_ok?'ok':'zle');$('lid2').textContent=s.pkt_obrot+' pkt/obrót';
 if(s.pitch===null){kl($('prz'),s.imu_wl?'BRAK':'wył.',s.imu_wl?'zle':'')}else{const m=s.przechyl;kl($('prz'),m.toFixed(1)+'°',m>15?'zle':m>8?'uw':'ok')}
 $('prz2').textContent=s.max_przechyl.toFixed(1)+'°';
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
  (s.pom||[]).slice().reverse().map(x=>esc(x.nr)+' – '+(x.opraw?x.opraw+' × '+x.typ+(x.punktow?' ('+x.punktow+' pkt)':'')+(x.moc?' × '+x.moc+' W = '+Math.round(x.moc_razem)+' W':''):'bez opraw')).join('<br>');
 $('kom').textContent=s.komunikat||''}).catch(()=>{$('kom').textContent='Brak połączenia z Raspberry Pi'})}
setInterval(odswiez,500);odswiez();
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
        if u.path == "/stan":
            teraz = time.time()
            with LOCK:
                s = {k: STAN[k] for k in ("nagrywa", "nazwa", "hz", "pkt_obrot", "yaw", "pitch", "roll",
                                          "max_przechyl", "wys", "temp", "znaczniki", "komunikat", "pauza", "auto_pauza",
                                          "przechyl", "prz_faza", "prz_zostalo", "prz_nr", "prz_wynik",
                                          "budynek", "kondygnacja")}
                s["obrot"] = STAN["obrot"]
                s["pom"] = STAN["pom"][-5:]
                s["pom_n"] = len(STAN["pom"])
                s["pom_w"] = round(sum(x.get("moc_razem", 0) for x in STAN["pom"]))
                s["pom_opraw"] = sum(x["opraw"] for x in STAN["pom"])
                s["pom_pkt"] = sum(x["opraw"] * x["punktow"] for x in STAN["pom"])
                s["pom_sal"] = len({x["nr"] for x in STAN["pom"]})
                s["czas"] = teraz - STAN["t_start"] if STAN["nagrywa"] else 0
                s["rozmiar"] = sum(STAN["bajty"].values())
                s["lidar_ok"] = teraz - STAN["ost"]["lidar"] < 1.5
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
            blad = pomieszczenie(g("nr"), g("typ"), g("punktow"), g("opraw"), g("uwagi"), g("moc"))
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/kondygnacja":
            self._json({"ok": zmien_kondygnacje(q.get("k", [""])[0])})
        elif u.path == "/usun":
            blad = usun_sesje(q.get("n", [""])[0])
            self._json({"blad": blad} if blad else {"ok": True})
        elif u.path == "/pauza":
            r = pauza()
            self._json({"ok": True, "pauza": r} if r is not None else {"blad": "Pauza działa tylko podczas nagrywania"})
        elif u.path == "/zeruj":
            with LOCK:
                STAN["p0"] = None
            self._json({"ok": True})
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
    ap.add_argument("--imu", default="/dev/ttyACM0")
    ap.add_argument("--imu-baud", type=int, default=115200)
    ap.add_argument("--bez-imu", action="store_true")
    ap.add_argument("--baro-adres", type=lambda x: int(x, 0), default=0x76)
    ap.add_argument("--bez-baro", action="store_true")
    ap.add_argument("--katalog", default=os.path.expanduser("~/skany"))
    ap.add_argument("--port", type=int, default=8080)
    ARGS = ap.parse_args()
    os.makedirs(ARGS.katalog, exist_ok=True)
    watki = [(watek_lidar, (ARGS.lidar, ARGS.lidar_baud)), (watek_flush, ())]
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

"""BNO085 w trybie UART-SHTP (mostek P1 -> 3Vo, P0 wolny; 3 000 000 bps, 8N1).

W odroznieniu od UART-RVC czujnik oddaje surowy zyroskop (kalibrowany, rad/s), przyspieszenia
i kwaternion Game Rotation Vector (bez magnetometru) z wlasnymi opoznieniami probek.

Ramka UART: 0x7E | 0x01 (SHTP) | dane z podmiana bajtow (0x7E -> 7D 5E, 0x7D -> 7D 5D) | 0x7E.
Dane SHTP: naglowek 4 B (dlugosc LSB, MSB z bitem kontynuacji, kanal, numer) + tresc.
Kanal 2 = sterowanie (Set Feature 0xFD), kanal 3 = raporty czujnikow.
Od hosta do czujnika bajty trzeba wysylac z odstepem >= 100 us.

Modul wspolny dla rejestratora (skan_web.py, skan_log.py) i dekodera (mapa/dekoduj.py).
"""
import math, struct, time

BAUD = 3000000

# raporty: id -> (dlugosc w bajtach, nazwa)
DLUGOSC = {0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10, 0x07: 16, 0x08: 12, 0x09: 14,
           0x0A: 16, 0x0B: 6, 0x0E: 6, 0x0F: 16, 0x10: 6, 0x11: 12, 0x12: 16, 0x13: 6, 0x14: 16, 0x15: 16,
           0x16: 14, 0x28: 14, 0x2A: 16, 0xFA: 5, 0xFB: 5}
ACC, ZYRO, GRV, MAG, RV = 0x01, 0x02, 0x08, 0x03, 0x05

# czestotliwosci w rejestratorze: kwaternion i zyroskop 200 Hz, przyspieszenia 100 Hz; dodatkowo 50 Hz
# Rotation Vector (z magnetometrem) i magnetometr - tylko zapis, do sprawdzenia czy pole w budynku jest stabilne
RAPORTY = ((GRV, 5000), (ZYRO, 5000), (ACC, 10000), (RV, 20000), (MAG, 20000))


def _stuff(b):
    out = bytearray()
    for x in b:
        if x == 0x7E: out += b'\x7d\x5e'
        elif x == 0x7D: out += b'\x7d\x5d'
        else: out.append(x)
    return bytes(out)


def ramka_set_feature(rid, okres_us, seq=0):
    """Ramka UART z poleceniem Set Feature (kanal sterowania 2)."""
    tresc = struct.pack('<BBBHIII', 0xFD, rid, 0, 0, okres_us, 0, 0)
    shtp = struct.pack('<HBB', 4 + len(tresc), 2, seq & 0xFF) + tresc
    return b'\x7e\x01' + _stuff(shtp) + b'\x7e'


def wyslij_konfiguracje(port, seq0=0):
    """Wlacza raporty RAPORTY; port = otwarty serial.Serial na 3 Mbps."""
    for k, (rid, okres) in enumerate(RAPORTY):
        for b in ramka_set_feature(rid, okres, seq0 + k):
            port.write(bytes([b]))
            port.flush()
            time.sleep(0.0002)              # >= 100 us miedzy bajtami
        time.sleep(0.02)


class Strumien:
    """Skleja bajty z portu w pakiety SHTP: feed(dane, t_ns) -> [(t_ns, kanal, tresc)]."""
    def __init__(self):
        self.buf = bytearray(); self.w = False; self.esc = False

    def feed(self, dane, t=0):
        out = []
        for x in dane:
            if x == 0x7E:
                if self.w and len(self.buf) >= 5 and self.buf[0] == 0x01:
                    s = bytes(self.buf[1:])
                    n = (s[0] | s[1] << 8) & 0x7FFF
                    if 4 <= n <= len(s):
                        out.append((t, s[2], s[4:n]))
                self.buf = bytearray(); self.w = True; self.esc = False
            elif self.w:
                if x == 0x7D: self.esc = True; continue
                if self.esc: x ^= 0x20; self.esc = False
                self.buf.append(x)
                if len(self.buf) > 600: self.buf = bytearray(); self.w = False
        return out


def raporty(tresc):
    """Raporty z pakietu kanalu 3: [(id, seq, dt_s, wartosci)]; dt_s = czas probki wzgledem odbioru pakietu (<0)."""
    out = []; i = 0; baza = 0
    while i < len(tresc):
        rid = tresc[i]; n = DLUGOSC.get(rid)
        if n is None or i + n > len(tresc): break
        r = tresc[i:i + n]; i += n
        if rid == 0xFB: baza = -struct.unpack_from('<i', r, 1)[0]; continue
        if rid == 0xFA: baza += struct.unpack_from('<i', r, 1)[0]; continue
        opozn = ((r[2] & 0xFC) << 6) | r[3]
        dt = (baza + opozn) * 1e-4
        if rid in (ACC, 0x04, 0x06): w = tuple(v / 256.0 for v in struct.unpack_from('<hhh', r, 4))      # m/s2, Q8
        elif rid in (ZYRO,): w = tuple(v / 512.0 for v in struct.unpack_from('<hhh', r, 4))             # rad/s, Q9
        elif rid in (GRV, RV): w = tuple(v / 16384.0 for v in struct.unpack_from('<hhhh', r, 4))        # i, j, k, real (Q14)
        elif rid == MAG: w = tuple(v / 16.0 for v in struct.unpack_from('<hhh', r, 4))                  # uT, Q4
        else: continue
        out.append((rid, r[1], dt, w))
    return out


def katy(q):
    """Kwaternion (i, j, k, real) -> (yaw, pitch, roll) w stopniach, Tait-Bryan Z-Y-X (jak w trybie RVC)."""
    x, y, z, w = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def wyglada_na_shtp(dane):
    """Rozpoznanie zawartosci imu.bin: True gdy sa ramki SHTP z kanalem 3, False gdy ramki RVC (AA AA)."""
    rvc = dane.count(b'\xaa\xaa')
    s = Strumien(); n3 = sum(1 for _, k, _ in s.feed(dane) if k == 3)
    return n3 > rvc

"""Protokol Unitree L2 (unilidar_sdk2) - wspolny dla rejestratora na malinie i dekodera na Dellu.

Ramka: naglowek 55 AA 05 0A | typ uint32 | rozmiar calej ramki uint32 | dane | crc32(dane) uint32 |
       msg_type_check uint32 | 2 B rezerwy | 00 FF.  Little-endian.
Typy: 102 chmura (300 punktow), 103 skan 2D, 104 IMU, 105 wersja, 106 znacznik czasu, 108 konfiguracja IP.
Przeliczenie odleglosci na XYZ jak w unilidar_driver (github.com/discodyer/unilidar_driver,
unitree_lidar_utilities.h: parseFromPacketToPointCloud).

Fabryczne adresy: L2 192.168.1.62:6101 -> komputer 192.168.1.2:6201 (UDP).
"""
import struct, zlib

NAGLOWEK = b'\x55\xaa\x05\x0a'
CHMURA, SKAN2D, IMU, WERSJA, CZAS, IP = 102, 103, 104, 105, 106, 108
POLECENIE = 100
PORT_L2, PORT_HOST = 6101, 6201


# --------------------------------------------------------------------------- ramki
def ramka(typ, dane):
    """Ramka do wyslania do L2."""
    return (NAGLOWEK + struct.pack('<II', typ, 24 + len(dane)) + dane +
            struct.pack('<II', zlib.crc32(dane) & 0xFFFFFFFF, 0) + b'\x00\x00\x00\xff')


def start_obrotu():
    """Polecenie uzytkownika: STANDBY = 0 (start pracy / obrotu)."""
    return ramka(POLECENIE, struct.pack('<II', 2, 0))


def ustaw_ip(lidar_ip, host_ip, maska='255.255.255.0', brama='0.0.0.0', port_l2=PORT_L2, port_host=PORT_HOST):
    """Ramka 108: nowy adres L2 i adres komputera, do ktorego L2 wysyla dane. Po wyslaniu wylaczyc i wlaczyc L2."""
    b = lambda s: bytes(int(x) for x in s.split('.'))
    return ramka(IP, b(lidar_ip) + b(host_ip) + b(brama) + b(maska) + struct.pack('<HH', port_l2, port_host))


class Strumien:
    """Wycina poprawne ramki ze strumienia bajtow: feed(dane) -> [(typ, dane_bez_naglowka_i_ogona)]."""
    def __init__(self):
        self.buf = bytearray(); self.zle = 0

    def feed(self, dane):
        self.buf += dane; out = []
        while True:
            i = self.buf.find(NAGLOWEK)
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 3)]; break
            if i: del self.buf[:i]
            if len(self.buf) < 12: break
            typ, n = struct.unpack_from('<II', self.buf, 4)
            if n < 24 or n > 20000:
                del self.buf[:1]; self.zle += 1; continue
            if len(self.buf) < n: break
            r = bytes(self.buf[:n])
            d = r[12:n - 12]
            crc = struct.unpack_from('<I', r, n - 12)[0]
            if r[-2:] != b'\x00\xff' or (zlib.crc32(d) & 0xFFFFFFFF) != crc:
                del self.buf[:1]; self.zle += 1; continue
            del self.buf[:n]
            out.append((typ, d))
        return out


# --------------------------------------------------------------------------- dane
def info(d):
    """(seq, czas_s) z DataInfo na poczatku danych chmury/IMU."""
    seq, _, s, ns = struct.unpack_from('<IIII', d, 0)
    return seq, s + ns * 1e-9


def imu(d):
    """IMU: (seq, czas_s, kwaternion w,x,y,z, zyroskop rad/s xyz, przyspieszenie m/s2 xyz)."""
    seq, t = info(d)
    v = struct.unpack_from('<10f', d, 16)
    return seq, t, v[0:4], v[4:7], v[7:10]


def chmura(d, np=None):
    """Pakiet chmury -> slownik: seq, t (czas pakietu, s), x, y, z, i (intensywnosc), dt (czas punktu wzgledem
    pakietu, s). Z numpy (np podany) zwraca tablice, inaczej listy."""
    seq, t = info(d)
    o = 16 + 36                                   # DataInfo + LidarInsideState
    a_dist, b_dist, th_bias, al_bias, beta, xi, r_bias, r_scale = struct.unpack_from('<8f', d, o); o += 32
    n_dl = len(d)
    if n_dl >= o + 32 + 4 + 900:                  # L2: z polami com_horizontal_angle_start/step
        h0, hstep, okres, rmin, rmax, a0, astep, tstep = struct.unpack_from('<8f', d, o); o += 32
    else:                                         # starszy uklad bez com_* (na wszelki wypadek)
        h0 = hstep = 0.0
        okres, rmin, rmax, a0, astep, tstep = struct.unpack_from('<6f', d, o); o += 24
    n = struct.unpack_from('<I', d, o)[0]; o += 4
    n = min(n, 300)
    zasieg = struct.unpack_from(f'<{300}H', d, o); inten = d[o + 600: o + 900]
    import math
    sb, cb, sx, cx = math.sin(beta), math.cos(beta), math.sin(xi), math.cos(xi)
    out = {'seq': seq, 't': t, 'okres': okres, 'x': [], 'y': [], 'z': [], 'i': [], 'dt': []}
    if np is not None:
        j = np.arange(n); r = np.asarray(zasieg[:n], float)
        rf = r_scale * (r + r_bias)
        m = (r >= 1) & (rf >= rmin) & (rf <= rmax)
        al = a0 + al_bias + j * astep; th = h0 + th_bias + j * hstep
        A = (-cb * sx + sb * cx * np.sin(al)) * rf + b_dist
        B = np.cos(al) * cx * rf
        C = (sb * sx + cb * cx * np.sin(al)) * rf
        out.update(x=(np.cos(th) * A - np.sin(th) * B)[m], y=(np.sin(th) * A + np.cos(th) * B)[m],
                   z=(C + a_dist)[m], i=np.frombuffer(inten, np.uint8)[:n][m].astype(float), dt=(j * tstep)[m])
        return out
    for j in range(n):
        if zasieg[j] < 1: continue
        rf = r_scale * (zasieg[j] + r_bias)
        if rf < rmin or rf > rmax: continue
        al = a0 + al_bias + j * astep; th = h0 + th_bias + j * hstep
        A = (-cb * sx + sb * cx * math.sin(al)) * rf + b_dist
        B = math.cos(al) * cx * rf
        C = (sb * sx + cb * cx * math.sin(al)) * rf
        out['x'].append(math.cos(th) * A - math.sin(th) * B); out['y'].append(math.sin(th) * A + math.cos(th) * B)
        out['z'].append(C + a_dist); out['i'].append(inten[j]); out['dt'].append(j * tstep)
    return out

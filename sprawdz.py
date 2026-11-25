#!/usr/bin/env python3
"""
sprawdz.py - szybka kontrola nagrania z skan_log.py (dla D500 / LD19 / STL27L i BNO085 RVC).

Uzycie:
    python3 sprawdz.py ~/skany/20261126_090000_parter
    python3 sprawdz.py ~/skany/20261126_090000_parter --png   (zapisuje podglad jednego obrotu)

Wypisuje: czas nagrania, liczbe poprawnych pakietow, predkosc obrotowa, liczbe punktow,
zasieg min/max, a dla IMU - zakres przechylu. Jesli liczby wygladaja sensownie, nagranie jest dobre.
"""
import math, os, struct, sys


def rekordy(sciezka):
    with open(sciezka, "rb") as f:
        while True:
            h = f.read(10)
            if len(h) < 10:
                return
            t, n = struct.unpack("<QH", h)
            yield t, f.read(n)


def crc8_ld(dane):
    # CRC-8 uzywane w protokole LD19/STL27L (wielomian 0x4D)
    crc = 0
    for b in dane:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pakiety_ld(sciezka):
    """Dekoduje pakiety LD19: 0x54 0x2C, predkosc, kat startu, 12 x (odl, intens), kat konca, czas, crc."""
    buf = bytearray()
    for t, d in rekordy(sciezka):
        buf += d
        while len(buf) >= 47:
            if buf[0] != 0x54 or buf[1] != 0x2C:
                del buf[0]
                continue
            p = bytes(buf[:47])
            if crc8_ld(p[:46]) != p[46]:
                del buf[0]
                continue
            del buf[:47]
            predkosc, start = struct.unpack_from("<HH", p, 2)
            koniec, _ts = struct.unpack_from("<HH", p, 42)
            a0, a1 = start / 100.0, koniec / 100.0
            krok = ((a1 - a0) % 360) / 11
            pts = []
            for i in range(12):
                odl, intens = struct.unpack_from("<HB", p, 6 + 3 * i)
                pts.append(((a0 + krok * i) % 360, odl / 1000.0, intens))
            yield t, predkosc, pts


def ramki_rvc(sciezka):
    """BNO085 UART-RVC: 0xAA 0xAA, indeks, yaw, pitch, roll (0,01 st.), ax, ay, az, 3 bajty, suma."""
    buf = bytearray()
    for t, d in rekordy(sciezka):
        buf += d
        while len(buf) >= 19:
            if buf[0] != 0xAA or buf[1] != 0xAA:
                del buf[0]
                continue
            p = bytes(buf[:19])
            if sum(p[2:18]) & 0xFF != p[18]:
                del buf[0]
                continue
            del buf[:19]
            yaw, pitch, roll = struct.unpack_from("<hhh", p, 3)
            yield t, yaw / 100.0, pitch / 100.0, roll / 100.0


def bmp280(kal, d):
    """Kompensacja BMP280 wg noty Boscha: zwraca (temp C, cisnienie Pa)."""
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


def wysokosci(sciezka):
    """Zwraca liste (czas_ns, wysokosc wzgledna [m]) z pliku baro.bin."""
    kal, p0, out = None, None, []
    for t, d in rekordy(sciezka):
        if d[:3] == b"CAL":
            kal = d[4:28]
            continue
        if kal is None or len(d) != 6:
            continue
        _, p = bmp280(kal, d)
        if p <= 0:
            continue
        if p0 is None:
            p0 = p
        out.append((t, 44330.0 * (1 - (p / p0) ** 0.1903)))
    return out


def poziomy(wys, okno_s=8.0, prog=1.2):
    """Dzieli nagranie na odcinki o stalej wysokosci (kondygnacje); zwraca [(t_od, t_do, wys_srednia)]."""
    if not wys:
        return []
    # mediana kroczaca co 1 s
    sek, i = [], 0
    while i < len(wys):
        j = i
        while j < len(wys) and wys[j][0] - wys[i][0] < 1e9:
            j += 1
        vals = sorted(h for _, h in wys[i:j])
        sek.append((wys[i][0], vals[len(vals) // 2]))
        i = j
    odc, start, ref = [], 0, sek[0][1]
    for k in range(1, len(sek)):
        if abs(sek[k][1] - ref) > prog:
            if (sek[k - 1][0] - sek[start][0]) / 1e9 >= okno_s:
                hs = [h for _, h in sek[start:k]]
                odc.append((sek[start][0], sek[k - 1][0], sum(hs) / len(hs)))
            start, ref = k, sek[k][1]
        else:
            ref = 0.9 * ref + 0.1 * sek[k][1]
    if (sek[-1][0] - sek[start][0]) / 1e9 >= okno_s:
        hs = [h for _, h in sek[start:]]
        odc.append((sek[start][0], sek[-1][0], sum(hs) / len(hs)))
    return odc


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    kat = sys.argv[1]
    lp = os.path.join(kat, "lidar.bin")
    if not os.path.exists(lp):
        sys.exit(f"Brak pliku {lp}")
    n = 0
    t0 = t1 = None
    predkosci, odl = [], []
    obrot, ostatni_kat, obrot_gotowy = [], None, None
    for t, v, pts in pakiety_ld(lp):
        n += 1
        t0 = t0 or t
        t1 = t
        predkosci.append(v)
        for kat_, d, i in pts:
            if d > 0.05:
                odl.append(d)
            if ostatni_kat is not None and kat_ < ostatni_kat - 180 and len(obrot) > 200 and obrot_gotowy is None and n > 300:
                obrot_gotowy = obrot
            if ostatni_kat is not None and kat_ < ostatni_kat - 180:
                obrot = []
            obrot.append((kat_, d))
            ostatni_kat = kat_
    if n == 0:
        sys.exit("Nie znaleziono poprawnych pakietow LD19. Zly baud albo inny czujnik "
                 "(surowe dane i tak sa zapisane - podeslij plik).")
    czas = (t1 - t0) / 1e9
    odl.sort()
    print(f"LiDAR: {czas:.0f} s nagrania, {n} pakietow, {n * 12 / max(czas, 1):.0f} pkt/s")
    print(f"  obroty: {sum(predkosci) / len(predkosci) / 360:.1f} Hz")
    print(f"  poprawne odleglosci: {len(odl)} ({100 * len(odl) / (n * 12):.0f}%), "
          f"mediana {odl[len(odl) // 2]:.2f} m, 95% < {odl[int(len(odl) * .95)]:.2f} m, max {odl[-1]:.2f} m")
    ip = os.path.join(kat, "imu.bin")
    if os.path.exists(ip):
        r = list(ramki_rvc(ip))
        if r:
            p0 = r0 = None
            try:
                with open(os.path.join(kat, "info.txt")) as f:
                    info = dict(l.strip().split("=", 1) for l in f if "=" in l)
                p0, r0 = float(info["imu_pitch0"]), float(info["imu_roll0"])
            except (OSError, KeyError, ValueError):
                pass
            zrodlo = "z info.txt (Zeruj pion)"
            if p0 is None or (p0 == 0 and r0 == 0):
                # brak zerowania: pion = mediana z pierwszych 10 s (nagranie zaczyna sie od postoju)
                pocz = [x for x in r if x[0] - r[0][0] < 10e9]
                p0 = sorted(x[2] for x in pocz)[len(pocz) // 2]
                r0 = sorted(x[3] for x in pocz)[len(pocz) // 2]
                zrodlo = "szacunek z pierwszych 10 s - zrob 'Zeruj pion' przed nagraniem"
            prz = [math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(x[2] - p0)) *
                                                                 math.cos(math.radians(x[3] - r0)))))) for x in r]
            chod = sorted(m for m in prz if m < 45)          # bez przekrojow pionowych (glowica polozona)
            n = len(chod) or 1
            print(f"IMU: {len(r)} ramek ({len(r) / max(czas, 1):.0f} Hz), pion pitch {p0:+.1f} / roll {r0:+.1f} st. ({zrodlo})")
            if chod:
                print(f"  przechyl od pionu (bez przekrojow): mediana {chod[n // 2]:.1f}, 95% < {chod[int(n * .95)]:.1f}, "
                      f"max {chod[-1]:.1f} st.; > 8 st.: {100 * sum(m > 8 for m in chod) / n:.0f}%, "
                      f"> 12 st.: {100 * sum(m > 12 for m in chod) / n:.0f}%")
                if chod[int(n * .95)] > 8:
                    print("  UWAGA: duzo skanow z przechylem > 8 st. - trzymaj tyczke bardziej pionowo.")
            yaw = [x[1] for x in r if x[0] - r[0][0] < 120e9]
            if len(yaw) > 100:
                u = [yaw[0]]
                for y in yaw[1:]:
                    u.append(u[-1] + ((y - u[-1] + 180) % 360 - 180))
                print(f"  yaw w pierwszych 2 min: zakres {max(u) - min(u):.1f} st. (przy postoju = dryf)")
        else:
            print("IMU: plik jest, ale brak poprawnych ramek RVC (sprawdz zworke PS0 i baud 115200).")
    bp = os.path.join(kat, "baro.bin")
    if os.path.exists(bp):
        w = wysokosci(bp)
        if w:
            hs = [h for _, h in w]
            print(f"Barometr: {len(w)} pomiarow, zmiana wysokosci od {min(hs):+.1f} do {max(hs):+.1f} m wzgledem startu")
            for a, b, h in poziomy(w):
                print(f"  poziom {h:+5.1f} m: od {(a - w[0][0]) / 1e9:5.0f} s do {(b - w[0][0]) / 1e9:5.0f} s")
        else:
            print("Barometr: plik jest, ale brak danych (sprawdz adres 0x76/0x77 i polaczenie I2C).")
    if "--png" in sys.argv and obrot_gotowy:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            xs = [d * math.cos(math.radians(a)) for a, d in obrot_gotowy if d > 0.05]
            ys = [d * math.sin(math.radians(a)) for a, d in obrot_gotowy if d > 0.05]
            plt.figure(figsize=(6, 6))
            plt.plot(xs, ys, "k.", ms=2)
            plt.plot(0, 0, "r+")
            plt.axis("equal")
            plt.grid(lw=0.3)
            plt.title("Jeden obrot [m]")
            out = os.path.join(kat, "podglad.png")
            plt.savefig(out, dpi=110)
            print(f"Podglad zapisany: {out}")
        except ImportError:
            print("Brak matplotlib - pomijam podglad (sudo apt install python3-matplotlib).")


if __name__ == "__main__":
    main()

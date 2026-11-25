"""Dekoduje lidar.bin, imu.bin, baro.bin do npz z czasem maliny dla kazdego pakietu/ramki."""
import sys, struct, numpy as np
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))
import sprawdz as S

def dekoduj(kat, wy=None):
    """Zwraca slownik tablic; zapisuje do wy (npz), jesli podano."""
    # LiDAR: czas odczytu porcji (Pi) + znacznik LD19 (ms, przepelnienie 30000)
    T, TS, A0, A1, D, I = [], [], [], [], [], []
    buf = bytearray(); tpi = None
    for t, d in S.rekordy(kat + '/lidar.bin'):
        buf += d
        while len(buf) >= 47:
            if buf[0] != 0x54 or buf[1] != 0x2C: del buf[0]; continue
            p = bytes(buf[:47])
            if S.crc8_ld(p[:46]) != p[46]: del buf[0]; continue
            del buf[:47]
            _, st = struct.unpack_from('<HH', p, 2); kon, ts = struct.unpack_from('<HH', p, 42)
            odl = [struct.unpack_from('<HB', p, 6 + 3 * i) for i in range(12)]
            T.append(t); TS.append(ts); A0.append(st / 100); A1.append(kon / 100)
            D.append([o[0] for o in odl]); I.append([o[1] for o in odl])
    T = np.array(T, np.int64); TS = np.array(TS, np.int64)
    # rozwiniecie licznika LD19 z pomoca czasu maliny: krotnosc 30000 ms dobrana tak, by przyrost zgadzal sie
    # z czasem odczytu. Restart LiDAR-a (zanik zasilania - np. luzny kabel przy przechyle) zeruje licznik:
    # wtedy przyrost nie pasuje do czasu maliny -> nowy odcinek z wlasnym przesunieciem.
    tp = (T - T[0]) / 1e9
    dpi = np.diff(tp) * 1000; d = np.diff(TS).astype(float)
    d += 30000 * np.round((dpi - d) / 30000)
    zle = np.abs(d - dpi) > 300
    d[zle] = dpi[zle]
    tsu = np.concatenate([[0], np.cumsum(d)]).astype(float)
    gr = np.concatenate([[0], np.where(zle)[0] + 1, [len(T)]])
    # skala zegara z najdluzszego odcinka (odporna regresja po dolnej obwiedni opoznien odczytu)
    dl = np.argmax(np.diff(gr)); m0 = slice(gr[dl], gr[dl + 1])
    x, y = tsu[m0] / 1000, tp[m0]
    a, b = np.polyfit(x, y, 1)
    for _ in range(5):
        r = y - (a * x + b); m = r < np.percentile(r, 20)
        a, b = np.polyfit(x[m], y[m], 1)
    tl = np.empty(len(T))
    for i in range(len(gr) - 1):                   # przesuniecie osobno dla kazdego odcinka
        s_ = slice(gr[i], gr[i + 1]); r = tp[s_] - a * tsu[s_] / 1000
        tl[s_] = a * tsu[s_] / 1000 + np.percentile(r, 10)
    if zle.any():
        przerwy = [(round(float(tp[j]), 1), round(float(dpi[j - 1]) / 1000, 1)) for j in gr[1:-1]]
        print(f'LiDAR: {int(zle.sum())} przerw/restartow licznika (czas s, przerwa s): {przerwy[:12]}')
    print(f'LiDAR: {len(T)} pakietow, skala zegara {(a-1)*1e6:+.0f} ppm, rozrzut opoznien {np.percentile(tp-tl,95)*1000:.1f} ms (95%)')
    # IMU
    import os
    if not os.path.exists(kat + '/imu.bin'):
        print('IMU: brak imu.bin - mapa bez IMU (kurs tylko z dopasowania skanow, bez filtra przechylu)')
        ti = np.arange(tl[0], tl[-1], 0.01)
        z = dict(tl=tl, a0=np.array(A0), a1=np.array(A1), d=np.array(D, float) / 1000, i=np.array(I),
                 ti=ti, yaw=np.zeros_like(ti), pitch=np.zeros_like(ti), roll=np.zeros_like(ti), acc=np.zeros((len(ti), 3)),
                 t0=T[0], bez_imu=True, wersja_dek=2)
        if wy:
            np.savez_compressed(wy, **z)
        return z
    Ti, R = [], []
    buf = bytearray()
    for t, d in S.rekordy(kat + '/imu.bin'):
        buf += d
        while len(buf) >= 19:
            if buf[0] != 0xAA or buf[1] != 0xAA: del buf[0]; continue
            p = bytes(buf[:19])
            if sum(p[2:18]) & 0xFF != p[18]: del buf[0]; continue
            del buf[:19]
            Ti.append(t); R.append((p[2],) + struct.unpack_from('<hhhhhh', p, 3))
    Ti = np.array(Ti, np.int64); R = np.array(R, float)
    dc = np.diff(R[:, 0]) % 256; cnt = np.concatenate([[0], np.cumsum(dc)])
    tpi = (Ti - T[0]) / 1e9
    ai, bi = np.polyfit(cnt, tpi, 1)
    for _ in range(5):
        r = tpi - (ai * cnt + bi); m = r < np.percentile(r, 20)
        ai, bi = np.polyfit(cnt[m], tpi[m], 1)
    print(f'IMU: {len(Ti)} ramek, okres {ai*1000:.3f} ms, zgubione {int(cnt[-1]+1-len(Ti))}')
    ti = ai * cnt + bi
    yaw = np.degrees(np.unwrap(np.radians(R[:, 1] / 100)))
    z = dict(tl=tl, a0=np.array(A0), a1=np.array(A1), d=np.array(D, float) / 1000, i=np.array(I),
             ti=ti, yaw=yaw, pitch=R[:, 2] / 100, roll=R[:, 3] / 100, acc=R[:, 4:7], t0=T[0], wersja_dek=2)
    if wy:
        np.savez_compressed(wy, **z)
    return z


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    dekoduj(k, k + '_dek.npz')

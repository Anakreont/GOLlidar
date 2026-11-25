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
    # rozwiniecie licznika LD19
    d = np.diff(TS); d[d < -15000] += 30000
    tsu = np.concatenate([[0], np.cumsum(d)]).astype(float)
    # odporna regresja t_pi = a*ts + b (opoznienie odczytu tylko dodatnie -> dolna obwiednia)
    tp = (T - T[0]) / 1e9
    a, b = np.polyfit(tsu / 1000, tp, 1)
    for _ in range(5):
        r = tp - (a * tsu / 1000 + b); m = r < np.percentile(r, 20)
        a, b = np.polyfit(tsu[m] / 1000, tp[m], 1)
    tl = a * tsu / 1000 + b
    print(f'LiDAR: {len(T)} pakietow, skala zegara {(a-1)*1e6:+.0f} ppm, rozrzut opoznien {np.percentile(tp-tl,95)*1000:.1f} ms (95%)')
    # IMU
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
             ti=ti, yaw=yaw, pitch=R[:, 2] / 100, roll=R[:, 3] / 100, acc=R[:, 4:7], t0=T[0])
    if wy:
        np.savez_compressed(wy, **z)
    return z


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    dekoduj(k, k + '_dek.npz')

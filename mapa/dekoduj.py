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
                 t0=T[0], bez_imu=True, wersja_dek=3)
        if wy:
            np.savez_compressed(wy, **z)
        return z
    import imu_shtp
    probka = b''.join(d for _, d in zip(range(400), (x for _, x in S.rekordy(kat + '/imu.bin'))))
    if imu_shtp.wyglada_na_shtp(probka):
        z = dekoduj_shtp(kat, T[0], tl, A0, A1, D, I)
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
             ti=ti, yaw=yaw, pitch=R[:, 2] / 100, roll=R[:, 3] / 100, acc=R[:, 4:7], t0=T[0], wersja_dek=3)
    if wy:
        np.savez_compressed(wy, **z)
    return z


def _os_czasu(t_odb, seq, t_ref, okres=0.005):
    """Czas probek z numerow kolejnych (8 bit) i czasu odbioru skorygowanego o opoznienia raportu: odporna
    regresja po dolnej obwiedni (jak dla ramek RVC). Licznik rozwijany z pomoca czasu odbioru, bo przy
    dluzszej przerwie (zanik zasilania, odlaczenie USB) gubi sie wiecej niz 256 probek naraz.
    Zwraca czas w s od t_ref, okres i liczbe zgubionych probek."""
    tp = (np.asarray(t_odb, float) - t_ref)
    d = (np.diff(seq) % 256).astype(float)
    d += 256 * np.round((np.diff(tp) / okres - d) / 256)
    d = np.maximum(d, 1)
    n = np.concatenate([[0], np.cumsum(d)])
    a, b = np.polyfit(n, tp, 1)
    for _ in range(5):
        r = tp - (a * n + b); m = r < np.percentile(r, 20)
        a, b = np.polyfit(n[m], tp[m], 1)
    return a * n + b, a, int(n[-1] + 1 - len(n))


def dekoduj_shtp(kat, t0, tl, A0, A1, D, I):
    """imu.bin z trybu UART-SHTP: kwaternion GRV -> yaw/pitch/roll (jak RVC), zyroskop -> yaw_zyro (calka
    predkosci obrotu wokol pionu), przyspieszenia. Wszystko na osi czasu kwaternionu (~200 Hz)."""
    import imu_shtp as H
    st = H.Strumien(); G, W, Acc, RVm, Mg = [], [], [], [], []
    pk = 0
    for t, d in S.rekordy(kat + '/imu.bin'):
        for tt, kan, tr in st.feed(d, t):
            if kan != 3: continue
            pk += 1
            for rid, seq, dt, w in H.raporty(tr):
                rek = ((tt - t0) / 1e9 + dt, seq, w, pk, dt)
                {H.GRV: G, H.ZYRO: W, H.ACC: Acc, H.RV: RVm, H.MAG: Mg}.get(rid, []).append(rek)
    if len(G) < 100:
        raise RuntimeError('IMU SHTP: za malo kwaternionow w imu.bin - czy czujnik dostal konfiguracje?')
    tg, a, zg = _os_czasu([g[0] for g in G], np.array([g[1] for g in G]), 0.0, H.RAPORTY[0][1] * 1e-6)
    # pozostale raporty na osi czasu kwaternionu: czas pakietu z kwaternionu w tym samym (lub sasiednim)
    # pakiecie + wlasne opoznienie raportu; osobne rozwijanie licznikow rozjezdzalo sie po przerwach
    pkG = np.array([g[3] for g in G], float); refG = tg - np.array([g[4] for g in G])
    def os_raportu(L):
        return np.interp(np.array([v[3] for v in L], float), pkG, refG) + np.array([v[4] for v in L])
    q = np.array([g[2] for g in G])
    q /= np.linalg.norm(q, axis=1)[:, None]
    x, y, zq, w = q.T
    # znak kursu jak w trybie RVC (sprawdzone na nagraniu Gol1 26.11: kurs z ICP = -yaw RVC, a +yaw kwaternionu)
    yaw = -np.degrees(np.unwrap(np.arctan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq))))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - zq * x), -1, 1)))
    roll = np.degrees(np.arctan2(2 * (w * x + y * zq), 1 - 2 * (x * x + y * y)))
    print(f'IMU SHTP: {len(G)} kwaternionow, okres {a*1000:.3f} ms, zgubione {zg}; zyroskop {len(W)}, przysp. {len(Acc)}')
    yaw_z = yaw.copy(); om = np.zeros((len(tg), 3))
    if len(W) > 100:
        tw = os_raportu(W)
        Wv = np.array([v[2] for v in W])
        om = np.stack([np.interp(tg, tw, Wv[:, k]) for k in range(3)], 1)
        # predkosc obrotu w ukladzie swiata: R(q) * omega; skladowa pionowa calkowana daje kurs niezalezny
        # od filtra BNO (przekroj z glowica na boku nie psuje go jak katy Eulera blisko 90 st.)
        wz = (2 * (x * zq - w * y)) * om[:, 0] + (2 * (y * zq + w * x)) * om[:, 1] + (1 - 2 * (x * x + y * y)) * om[:, 2]
        inc = -np.degrees(0.5 * (wz[1:] + wz[:-1]) * np.diff(tg))
        # przerwy w danych (zgubione probki kwaternionu lub zyroskopu): przyrost kursu z kwaternionu
        przerwa = np.diff(tg) > 0.02
        gw = np.where(np.diff(tw) > 0.02)[0]
        for j in gw:
            przerwa[max(0, np.searchsorted(tg, tw[j]) - 1): np.searchsorted(tg, tw[j + 1]) + 1] = True
        inc[przerwa] = np.diff(yaw)[przerwa]
        yaw_z = yaw[0] + np.concatenate([[0], np.cumsum(inc)])
        print(f'IMU SHTP: kurs zyroskop - kwaternion na koncu {yaw_z[-1] - yaw[-1]:+.1f} st., przerw w danych {int(przerwa.sum())}')
    acc = np.zeros((len(tg), 3))
    if len(Acc) > 100:
        ta = os_raportu(Acc)
        Av = np.array([v[2] for v in Acc])
        acc = np.stack([np.interp(tg, ta, Av[:, k]) for k in range(3)], 1) * (1000 / 9.80665)   # mg jak w RVC
    # kurs z magnetometrem (Rotation Vector) i natezenie pola - na razie tylko do oceny zaklocen w budynku
    yaw_mag = np.full(len(tg), np.nan); pole = np.full(len(tg), np.nan)
    if len(RVm) > 100:
        qm = np.array([v[2] for v in RVm]); xm, ym_, zm, wm = qm.T
        ymg = -np.degrees(np.unwrap(np.arctan2(2 * (wm * zm + xm * ym_), 1 - 2 * (ym_ * ym_ + zm * zm))))
        yaw_mag = np.interp(tg, os_raportu(RVm), ymg)
    if len(Mg) > 100:
        pole = np.interp(tg, os_raportu(Mg), np.linalg.norm([v[2] for v in Mg], axis=1))
        print(f'IMU SHTP: pole magnetyczne mediana {np.nanmedian(pole):.0f} uT, 5-95% {np.nanpercentile(pole,5):.0f}-{np.nanpercentile(pole,95):.0f} uT')
    return dict(tl=tl, a0=np.array(A0), a1=np.array(A1), d=np.array(D, float) / 1000, i=np.array(I),
                ti=tg, yaw=yaw, yaw_mag=yaw_mag, pole_uT=pole, yaw_zyro=yaw_z, pitch=pitch, roll=roll, acc=acc, omega=om, t0=t0,
                imu_tryb='shtp', wersja_dek=3)


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    dekoduj(k, k + '_dek.npz')

"""Zamykanie petli dla mapy z Unitree L2 (po Point-LIO, offline na Dellu).

    python l2_petle.py <wynik_pointlio.bag> <katalog_wyniku> [--bez-zapisu-chmury]

Wejscie: rosbag nagrany w czasie liczenia Point-LIO (l2_mapa.sh): /pointlio/odom (nav_msgs/Odometry, pozycja
w ukladzie mapy) i /pointlio/cloud_registered (sensor_msgs/PointCloud2, skan po filtrze w ukladzie mapy, znacznik =
koniec skanu) - nazwy z point_lio_unilidar (laserMapping.cpp); starsze nazwy FAST-LIO tez sa obslugiwane.

Kroki:
 1. kazdy skan wraca do ukladu czujnika (odwrotnosc pozycji z Point-LIO),
 2. klatki kluczowe co ~1 m / 15 st.: chmura z +-1 s trasy w ukladzie klatki (gestsza niz pojedynczy skan),
 3. kandydaci na petle: klatki blisko siebie w przestrzeni (< 5 m, |dz| < 1,5 m), a daleko po trasie (> 15 m),
 4. weryfikacja ICP punkt-plaszczyzna (3D), odrzucenie: slabe dopasowanie, za duza poprawka, degeneracja
    (dlugi gladki korytarz - ICP nie wie, gdzie jest wzdluz),
 5. graf pozycji 4 stopnie swobody (x, y, z, kurs): przechyly zostaja z Point-LIO (IMU zna kierunek grawitacji),
    odporna funkcja straty - pojedyncza falszywa petla nie wykrzywia calej mapy,
 6. poprawka rozlozona na wszystkie skany (interpolacja miedzy klatkami) -> nowa chmura.
Wynik: chmura.pcd (mapa z petlami), petle.json (liczby, poprawki), trasa.png (przed / po + petle).
Mapa bez petli (scans.pcd z Point-LIO) zostaje osobno - w przegladarce wybiera sie lepsza (porownanie z CAD).
"""
import argparse, json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

# ---------------------------------------------------------------------------------------------- geometria
def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def kurs(R):
    return math.atan2(R[1, 0], R[0, 0])


def T_z(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def odwr(T):
    R, t = T[:3, :3], T[:3, 3]
    return T_z(R.T, -R.T @ t)


def kwat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def przeksztalc(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def woksel(P, k):
    if len(P) == 0:
        return P
    q = np.floor(P / k).astype(np.int64)
    _, i = np.unique(q, axis=0, return_index=True)
    return P[np.sort(i)]


def exp_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


# ---------------------------------------------------------------------------------------------- ICP 3D
def normalne(P, k=12):
    tr = cKDTree(P)
    _, idx = tr.query(P, k=min(k, len(P)))
    Q = P[idx] - P[idx].mean(1, keepdims=True)
    C = np.einsum('nki,nkj->nij', Q, Q)
    w, v = np.linalg.eigh(C)
    n = v[:, :, 0]
    plas = w[:, 0] / np.maximum(w.sum(1), 1e-12)                      # 0 = idealna plaszczyzna
    return tr, n, plas


def icp(src, dst, T0, iters=40, progi=(0.6, 0.3, 0.15, 0.08)):
    """ICP punkt-plaszczyzna src -> dst od T0. Zwraca (T, fitness, rmse, min_wart_wlasna_translacji)."""
    tr, N, plas = normalne(dst)
    dobre = plas < 0.08
    T = T0.copy(); H = np.eye(6)
    for etap, dmax in enumerate(progi):
        for _ in range(iters // len(progi)):
            P = przeksztalc(T, src)
            d, i = tr.query(P, distance_upper_bound=dmax)
            m = np.isfinite(d)
            m[m] &= dobre[i[m]]
            if m.sum() < 30:
                return T, 0.0, 1.0, 0.0
            p, q, n = P[m], dst[i[m]], N[i[m]]
            r = ((p - q) * n).sum(1)
            J = np.hstack([np.cross(p, n), n])
            w = 1.0 / np.maximum(1.0, np.abs(r) / 0.03)                  # Huber
            H = J.T @ (J * w[:, None]); g = J.T @ (w * r)
            dx = -np.linalg.solve(H + 1e-6 * np.eye(6), g)
            dT = T_z(exp_so3(dx[:3]), dx[3:])
            T = dT @ T
            if np.linalg.norm(dx) < 1e-5:
                break
    P = przeksztalc(T, src)
    d, i = tr.query(P, distance_upper_bound=0.1)
    m = np.isfinite(d)
    fit = float(m.mean())
    rmse = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else 1.0
    Ht = H[3:, 3:] / max(1, len(src))
    ev = float(np.linalg.eigvalsh(Ht)[0])
    return T, fit, rmse, ev


# ---------------------------------------------------------------------------------------------- klatki kluczowe
def klatki(poz, co_m=1.0, co_st=15.0):
    out = [0]
    for i in range(1, len(poz)):
        a = poz[out[-1]]; b = poz[i]
        if np.linalg.norm(b[:3, 3] - a[:3, 3]) >= co_m or abs(math.degrees(kurs(odwr(a) @ b))) >= co_st:
            out.append(i)
    return out


def chmura_klatki(k, poz, skany, czasy, okno=1.0, wok=0.08):
    Tk = odwr(poz[k]); t0 = czasy[k]; czesci = []
    for s in range(len(skany)):
        if abs(czasy[s] - t0) <= okno:
            czesci.append(przeksztalc(Tk @ poz[s], skany[s]))
    return woksel(np.vstack(czesci), wok) if czesci else np.zeros((0, 3))


# ---------------------------------------------------------------------------------------------- graf 4-DoF
def graf(pozK, odo, petle, s_odo_t=0.02, s_odo_a=math.radians(0.3), s_pet_t=0.05, s_pet_a=math.radians(0.5)):
    """pozK: pozycje klatek (4x4, Point-LIO). odo/petle: [(i, j, T_ij)]. Zwraca nowe pozycje 4x4."""
    n = len(pozK)
    Rrp = [rot_z(-kurs(T[:3, :3])) @ T[:3, :3] for T in pozK]           # przechyly bez kursu (stale)
    x0 = np.array([[T[0, 3], T[1, 3], T[2, 3], kurs(T[:3, :3])] for T in pozK]).ravel()
    kr = [(i, j, T, s_odo_t * (1 + np.linalg.norm(T[:3, 3])), s_odo_a, False) for i, j, T in odo] + \
         [(i, j, T, s_pet_t, s_pet_a, True) for i, j, T in petle]

    def poza(x, i):
        return T_z(rot_z(x[4 * i + 3]) @ Rrp[i], x[4 * i:4 * i + 3])

    def res(x):
        r = [(x[:4] - x0[:4]) * 1e3]                                      # pierwsza klatka nieruchoma
        for i, j, Tm, st, sa, _ in kr:
            E = odwr(Tm) @ odwr(poza(x, i)) @ poza(x, j)
            r.append(np.r_[E[:3, 3] / st, kurs(E[:3, :3]) / sa])
        return np.concatenate(r)

    S = lil_matrix((4 + 4 * len(kr), 4 * n), dtype=int)
    S[:4, :4] = 1
    for e, (i, j, *_r) in enumerate(kr):
        S[4 + 4 * e:8 + 4 * e, 4 * i:4 * i + 4] = 1; S[4 + 4 * e:8 + 4 * e, 4 * j:4 * j + 4] = 1
    wyn = least_squares(res, x0, jac_sparsity=S, loss='soft_l1', f_scale=3.0, x_scale='jac', max_nfev=200)
    x = wyn.x
    # ktore petle zostaly "odrzucone" przez odporna strate (duzy blad po optymalizacji)
    r = res(x)[4:].reshape(-1, 4)
    odrz = [bool(np.abs(r[e]).max() > 6) for e in range(len(kr)) if kr[e][5]]
    return [poza(x, i) for i in range(n)], odrz


# ---------------------------------------------------------------------------------------------- calosc
def zamknij_petle(poz, skany, czasy, log=print, min_trasa=15.0, max_odl=5.0):
    """poz: pozycje skanow (4x4, uklad mapy), skany: punkty w ukladzie czujnika, czasy [s].
    Zwraca (nowe pozycje skanow, raport)."""
    K = klatki(poz)
    trasa = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.array([T[:3, 3] for T in poz]), axis=0), axis=1))]
    log(f'skanow {len(poz)}, klatek kluczowych {len(K)}, trasa {trasa[-1]:.0f} m')
    chm = {}
    def ch(k):
        if k not in chm:
            chm[k] = chmura_klatki(k, poz, skany, czasy)
        return chm[k]
    pK = np.array([poz[k][:3, 3] for k in K])
    odo = [(a, a + 1, odwr(poz[K[a]]) @ poz[K[a + 1]]) for a in range(len(K) - 1)]
    petle, kand, odrzucone = [], 0, {'dopasowanie': 0, 'degeneracja': 0, 'poprawka': 0}
    ostatnia = -10**9
    tr = cKDTree(pK[:, :2])
    for b in range(len(K)):
        if trasa[K[b]] - ostatnia < 3.0:                                  # nie czesciej niz co 3 m trasy
            continue
        bl = [a for a in tr.query_ball_point(pK[b, :2], max_odl)
              if trasa[K[b]] - trasa[K[a]] > min_trasa and abs(pK[a, 2] - pK[b, 2]) < 1.5]
        if not bl:
            continue
        a = min(bl, key=lambda a: np.linalg.norm(pK[a] - pK[b]))
        kand += 1
        T0 = odwr(poz[K[a]]) @ poz[K[b]]
        T, fit, rmse, ev = icp(ch(K[b]), ch(K[a]), T0)
        dT = odwr(T0) @ T
        popr_t, popr_a = np.linalg.norm(dT[:3, 3]), abs(math.degrees(kurs(dT[:3, :3])))
        if fit < 0.45 or rmse > 0.06:
            odrzucone['dopasowanie'] += 1; continue
        if ev < 0.02:
            odrzucone['degeneracja'] += 1; continue
        if popr_t > 3.0 or popr_a > 15:
            odrzucone['poprawka'] += 1; continue
        petle.append((a, b, T, dict(fit=round(fit, 2), rmse_mm=round(rmse * 1000, 1), poprawka_cm=round(popr_t * 100, 1),
                                    poprawka_st=round(popr_a, 2), trasa_m=round(trasa[K[b]] - trasa[K[a]], 1))))
        ostatnia = trasa[K[b]]
    log(f'kandydatow {kand}, petli przyjetych {len(petle)}, odrzuconych {odrzucone}')
    raport = dict(skanow=len(poz), klatek=len(K), trasa_m=round(float(trasa[-1]), 1), kandydatow=kand,
                  petli=len(petle), odrzucone=odrzucone, petle=[p[3] for p in petle])
    if not petle:
        raport['uwaga'] = 'brak petli - mapa bez zmian'
        return poz, raport
    nowe, odrz = graf([poz[k] for k in K], odo, [(a, b, T) for a, b, T, _ in petle])
    for p, o in zip(raport['petle'], odrz):
        p['odrzucona_w_grafie'] = o
    # poprawka kazdego skanu: interpolacja (czas) poprawek sasiednich klatek
    C = [n @ odwr(poz[k]) for n, k in zip(nowe, K)]
    tK = np.array([czasy[k] for k in K])
    dA = np.unwrap([kurs(c[:3, :3]) for c in C]); dT = np.array([c[:3, 3] for c in C])
    wyn = []
    for s in range(len(poz)):
        a_ = np.interp(czasy[s], tK, dA); t_ = np.array([np.interp(czasy[s], tK, dT[:, i]) for i in range(3)])
        wyn.append(T_z(rot_z(a_), t_) @ poz[s])
    zm = [np.linalg.norm(w[:3, 3] - p[:3, 3]) for w, p in zip(wyn, poz)]
    raport['max_przesuniecie_trasy_cm'] = round(float(max(zm)) * 100, 1)
    raport['srednie_przesuniecie_trasy_cm'] = round(float(np.mean(zm)) * 100, 1)
    log(f"trasa poprawiona: srednio {raport['srednie_przesuniecie_trasy_cm']} cm, max {raport['max_przesuniecie_trasy_cm']} cm")
    raport['_klatki'] = K; raport['_petle_ij'] = [(K[a], K[b]) for a, b, _, _ in petle]
    return wyn, raport


# ---------------------------------------------------------------------------------------------- wejscie / wyjscie
def czytaj_bag(sciezka):
    """Pozycje i skany z rosbaga Point-LIO. Skany w ukladzie mapy przeliczane do ukladu czujnika."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    from pathlib import Path
    odo, chm = [], {}
    with AnyReader([Path(sciezka)], default_typestore=get_typestore(Stores.ROS1_NOETIC)) as r:
        tematy = {c.topic for c in r.connections}
        t_odo = next((t for t in ('/pointlio/odom', '/aft_mapped_to_init', '/Odometry') if t in tematy), None)
        t_body = next((t for t in ('/pointlio/cloud_registered_body', '/cloud_registered_body') if t in tematy), None)
        t_swiat = next((t for t in ('/pointlio/cloud_registered', '/cloud_registered') if t in tematy), None)
        if not t_odo or not (t_body or t_swiat):
            raise SystemExit(f'w nagraniu brak tematow pozycji / chmury: {sorted(tematy)}')
        temat_ch = t_body or t_swiat
        conns = [c for c in r.connections if c.topic in (t_odo, temat_ch)]
        for c, t, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if c.topic == t_odo:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                odo.append((st, T_z(kwat_R((q.x, q.y, q.z, q.w)), np.array([p.x, p.y, p.z]))))
            else:
                pola = {f.name: f.offset for f in m.fields}
                A = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
                P = np.stack([A[:, pola[k]:pola[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1).astype(float)
                chm[round(st, 4)] = P[np.isfinite(P).all(1)]
    odo.sort(key=lambda x: x[0])
    tod = np.array([t for t, _ in odo])
    poz, skany, czasy = [], [], []
    t_swiat = None if temat_ch.endswith('_body') else temat_ch
    for st, P in sorted(chm.items()):
        i = int(np.argmin(np.abs(tod - st)))
        if abs(tod[i] - st) > 0.02 or len(P) < 50:
            continue
        T = odo[i][1]
        if temat_ch == t_swiat:
            P = przeksztalc(odwr(T), P)                                    # uklad mapy -> czujnik
        poz.append(T); skany.append(woksel(P, 0.03)); czasy.append(st)
    return poz, skany, np.array(czasy), temat_ch


def zapisz_pcd(sciezka, P):
    P = np.asarray(P, np.float32)
    with open(sciezka, 'wb') as f:
        f.write((f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(P)}\nHEIGHT 1\n'
                 f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(P)}\nDATA binary\n').encode())
        f.write(P.tobytes())


def rysuj_trase(sciezka, poz, nowe, raport):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    a = np.array([T[:3, 3] for T in poz]); b = np.array([T[:3, 3] for T in nowe])
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(a[:, 0], a[:, 1], '-', color='#9ca3af', lw=1, label='trasa Point-LIO (bez petli)')
    ax.plot(b[:, 0], b[:, 1], '-', color='#2563eb', lw=1.2, label='po zamknieciu petli')
    for i, j in raport.get('_petle_ij', []):
        ax.plot([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], '-', color='#dc2626', lw=0.8)
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='best')
    ax.set_title(f"petle: {raport['petli']} (czerwone), poprawka trasy max {raport.get('max_przesuniecie_trasy_cm', 0)} cm")
    fig.tight_layout(); fig.savefig(sciezka, dpi=100); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('wyjscie')
    a = ap.parse_args()
    os.makedirs(a.wyjscie, exist_ok=True)
    poz, skany, czasy, temat = czytaj_bag(a.bag)
    print(f'z {os.path.basename(a.bag)}: {len(poz)} skanow ({temat})')
    if len(poz) < 20:
        raise SystemExit('za malo skanow w nagraniu wyniku Point-LIO')
    nowe, raport = zamknij_petle(poz, skany, czasy)
    rysuj_trase(os.path.join(a.wyjscie, 'trasa.png'), poz, nowe, raport)
    mapa = woksel(np.vstack([przeksztalc(T, P) for T, P in zip(nowe, skany)]), 0.02)
    zapisz_pcd(os.path.join(a.wyjscie, 'chmura.pcd'), mapa)
    with open(os.path.join(a.wyjscie, 'petle.json'), 'w', encoding='utf-8') as f:
        json.dump({k: v for k, v in raport.items() if not k.startswith('_')}, f, ensure_ascii=False, indent=1)
    print(f"zapisano chmura.pcd ({len(mapa)} pkt), petle.json, trasa.png")


if __name__ == '__main__':
    main()

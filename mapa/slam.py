"""Pierwszy potok mapy 2D: obroty LD19 -> deskew z yaw IMU -> ICP point-to-line scan-to-submap."""
import sys, numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))

# parametry dopasowania (strojone na rzucie CAD domu Piotra, sesja 25.11 18:57)
PAR = dict(imu_w=200.0, promien=8.0, kf_dl=0.25, kf_kat=8.0, co_ktory=2, przechyl_max=12.0, huber=0.03, max_d=0.3,
           intens=0.3, sub_max=150, iters=15, luka_s=2.0, luka_frac=0.6, luka_rms=0.05)
LOG_LUK = []
TEST_LUKI = []   # testy: [(t_od, czas_s, dx, dy, dth_st)] - wyciecie skanow i sztuczny blad pozycji po luce


def wczytaj(kat):
    z = np.load(kat + '_dek.npz')
    return {k: z[k] for k in z.files}

WERSJA_DEK = 2   # zmiana dekodowania -> stara pamiec podreczna dek.npz liczona od nowa

def wczytaj_lub_dekoduj(kat):
    """Zdekodowane dane z pamieci podrecznej <sesja>/_wynik/dek.npz (odswiezanej, gdy lidar.bin jest nowszy)."""
    import os
    from dekoduj import dekoduj
    p = os.path.join(kat, '_wynik', 'dek.npz')
    if os.path.exists(p) and os.path.getmtime(p) >= os.path.getmtime(os.path.join(kat, 'lidar.bin')):
        z = np.load(p)
        if (int(z['wersja_dek']) if 'wersja_dek' in z.files else 0) == WERSJA_DEK:
            return {k: z[k] for k in z.files}
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return dekoduj(kat, p)

def pion_z_info(kat):
    """Pion zapisany przyciskiem 'Zeruj pion' (info.txt), jesli byl ustawiony."""
    import os
    try:
        d = dict(l.strip().split('=', 1) for l in open(os.path.join(kat, 'info.txt'), encoding='utf-8') if '=' in l)
        p, r = float(d['imu_pitch0']), float(d['imu_roll0'])
        return (p, r) if (p or r) else None
    except (OSError, KeyError, ValueError):
        return None


def przerwy(kat, t0):
    """Przedzialy PAUZA_START..PAUZA_KONIEC w sekundach od poczatku nagrania (z znaczniki.txt)."""
    import os
    p = os.path.join(kat, 'znaczniki.txt'); out = []; a = None
    if not os.path.exists(p): return out
    for l in open(p, encoding='utf-8'):
        k = l.split('\t')
        if len(k) < 2 or not k[0].isdigit(): continue
        t = (int(k[0]) - t0) / 1e9
        if k[1].startswith('PAUZA_START'): a = t
        elif k[1].startswith('PAUZA_KONIEC') and a is not None: out.append((a - 0.5, t + 1.0)); a = None
    if a is not None: out.append((a - 0.5, 1e12))
    return out


def obroty(z, rmin=0.3, rmax=9.0, pion=None):
    """Dzieli pakiety na obroty; kazdy punkt z czasem z interpolacji wewnatrz pakietu."""
    a0, a1, d, tl = z['a0'], z['a1'], z['d'], z['tl']
    inten = z['i'].astype(float)
    # slabe odbicia (przez szybe, po skosie, ciemne powierzchnie): intensywnosc ponizej 30 % mediany dla tej odleglosci
    przedz = np.arange(0, 16.5, 0.5)
    dd, ii = d.ravel(), inten.ravel()
    med = np.array([np.median(ii[(dd >= a) & (dd < a + 0.5)]) if np.sum((dd >= a) & (dd < a + 0.5)) > 50 else np.nan for a in przedz])
    med = np.where(np.isfinite(med), med, np.nanmin(med) if np.isfinite(med).any() else 0)
    krok = ((a1 - a0) % 360) / 11
    ang = (a0[:, None] + krok[:, None] * np.arange(12)) % 360
    dtp = np.median(np.diff(tl))
    tp = tl[:, None] + (np.arange(12) - 11) * dtp / 12      # znacznik pakietu ~ ostatni punkt
    nowy = np.where(np.diff(a0) < -180)[0] + 1
    gr = np.split(np.arange(len(a0)), nowy)
    gr = gr[1:-1]
    # operator (glowa, plecak, tyczka, rece przy niesieniu w dloni) porusza sie razem z czujnikiem i zatruwa
    # dopasowanie. Wycinki katowe (10 st.), w ktorych w ok. 10 s wiekszosc obrotow ma punkt blizej niz 0,7 m,
    # uznajemy za operatora i odrzucamy w tym oknie; dodatkowo zawsze tyl glowicy (130-230 st.).
    blisko = np.zeros((len(gr), 36), bool)
    for j, g in enumerate(gr):
        A, D = ang[g].ravel(), d[g].ravel()
        k = (D > 0.1) & (D < 0.7)
        blisko[j, (A[k] // 10).astype(int) % 36] = True
    okno = 100
    maska = np.zeros_like(blisko)
    for j0 in range(0, len(gr), okno // 2):
        b = blisko[max(0, j0 - okno // 2): j0 + okno]
        if len(b): maska[j0: j0 + okno // 2] = b.mean(0) > 0.6
    # rzutowanie na poziom: przy przechyle wiazka biegnie skosnie i odleglosc do pionowej sciany jest za duza o 1/cos(elewacji).
    # Os BNO wzgledem LiDAR-u ustalona z przekrojow: roll +85 st. = kierunek 0 st. LD19 w dol, wiec elewacja wiazki o kacie A
    # wynosi ok. -roll*cos(A) + pitch*sin(A) (przechyl liczony od pionu z postoju / Zeruj pion).
    rzut = pion is not None and not bool(z.get('bez_imu', False))
    if rzut:
        ti = z['ti']; pr = z['pitch'] - pion[0]; rr = z['roll'] - pion[1]
    out = []
    for j, g in enumerate(gr):
        A, D, Tt = ang[g].ravel(), d[g].ravel(), tp[g].ravel()
        I_ = inten[g].ravel()
        slabe = I_ < PAR['intens'] * np.interp(D, przedz + 0.25, med)
        if rzut:
            p_ = np.radians(np.interp(Tt, ti, pr)); r_ = np.radians(np.interp(Tt, ti, rr)); Ar = np.radians(A)
            elew = -r_ * np.cos(Ar) + p_ * np.sin(Ar)
            D = D * np.cos(np.clip(elew, -0.35, 0.35))
        m = (D > rmin) & (D < rmax) & ~slabe
        m &= ~((D < 0.7) & (A > 130) & (A < 230))
        m &= ~((D < 0.9) & maska[j][(A // 10).astype(int) % 36])
        if m.sum() < 150: continue
        out.append((Tt[m], np.radians(A[m]), D[m]))
    return out

def normalne(p):
    """Normalne z sasiadow w kolejnosci kata (punkty i-2..i+2); NaN gdy brak liniowosci."""
    n = np.full_like(p, np.nan)
    for i in range(2, len(p) - 2):
        q = p[i - 2:i + 3]
        if np.max(np.linalg.norm(q - p[i], axis=1)) > 0.25: continue
        c = np.cov((q - q.mean(0)).T)
        w, v = np.linalg.eigh(c)
        if w[0] > 0.1 * w[1]: continue
        n[i] = v[:, 0]
    return n

def icp(src, tgt, ntg, tree, T0, iters=15, huber=0.05, max_d=0.5, prior=None):
    x, y, th = T0
    for _ in range(iters):
        c, s = np.cos(th), np.sin(th)
        q = src @ np.array([[c, s], [-s, c]]) + [x, y]
        dist, j = tree.query(q, distance_upper_bound=max_d)
        m = np.isfinite(dist)
        if m.sum() < 50: return None
        qm, nm, tm, sm = q[m], ntg[j[m]], tgt[j[m]], src[m]
        r = np.sum((qm - tm) * nm, 1)
        w = np.where(np.abs(r) < huber, 1.0, huber / np.abs(r))
        # jakobian dla (dx, dy, dth)
        dq = np.stack([-s * sm[:, 0] - c * sm[:, 1], c * sm[:, 0] - s * sm[:, 1]], 1)
        J = np.stack([nm[:, 0], nm[:, 1], np.sum(nm * dq, 1)], 1)
        H = (J * w[:, None]).T @ J; g = (J * w[:, None]).T @ r
        if prior is not None:
            H[2, 2] += prior[1]; g[2] += prior[1] * (th - prior[0])
        try: dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
        except np.linalg.LinAlgError: return None
        x, y, th = x + dx[0], y + dx[1], th + dx[2]
        if np.abs(dx).max() < 1e-4: break
    ev = np.linalg.eigvalsh(H[:2, :2])
    return np.array([x, y, th]), np.sqrt(np.mean(r ** 2 * w)), ev, m.mean()

def mapa(kat, znak=+1, t_od=0, t_do=1e9, imu_w=None, pion=None, log=False, segmenty=None, promien=None, dane=None, rzutuj=True):
    imu_w = PAR['imu_w'] if imu_w is None else imu_w; promien = PAR['promien'] if promien is None else promien
    z = dane if dane is not None else wczytaj(kat)
    if bool(z.get('bez_imu', False)): imu_w = 0.0
    ti, yaw = z['ti'], np.radians(z['yaw'])
    pit, rol = z['pitch'], z['roll']
    if pion is None:
        pion = pion_z_info(kat)
    if pion is None:
        m10 = ti < ti[0] + 10; pion = (np.median(pit[m10]), np.median(rol[m10]))
    sk = obroty(z, pion=pion if rzutuj else None)
    prz = np.degrees(np.arccos(np.clip(np.cos(np.radians(pit - pion[0])) * np.cos(np.radians(rol - pion[1])), -1, 1)))
    poses, kf, info = [], [], []
    T = np.zeros(3); v = np.zeros(2); ostT = None; yaw_ost = None
    seg_ost = None; kfw = []   # kluczowe w ukladzie swiata (punkty, normalne, pozycja, segment)
    pauzy = przerwy(kat, int(z['t0'])); po_pauzie = False
    frag = []; frag_luka = {}; fr_id = 0; t_ok = None; zaburz = None   # fragmenty: nowy po luce w sledzeniu (nie po PAUZIE)
    for (tt, ang, D) in sk:
        tm = tt.mean()
        if tm < t_od or tm > t_do: continue
        if any(pa <= tm <= pb for pa, pb in pauzy):     # PAUZA: zdjety plecak, przekroj z reki - nie do rzutu
            info.append((tm, 'pauza')); po_pauzie = True; continue
        seg = 0 if segmenty is None else int(np.searchsorted(segmenty, tm))
        if seg != seg_ost:
            kf_seg = []; seg_ost = seg
        pz = np.interp(tt, ti, prz)
        if pz.max() > PAR['przechyl_max']: info.append((tm, 'przechyl')); continue
        tl_ = [l for l in TEST_LUKI if l[0] <= tm <= l[0] + l[1]]
        if tl_: info.append((tm, 'test')); zaburz = tl_[0]; continue
        yp = np.interp(tt, ti, yaw); ym = np.interp(tm, ti, yaw)
        a = -ang + znak * (yp - ym)              # LD19 zgodnie z zegarem -> matematycznie przeciwnie; deskew obrotu
        p = np.stack([D * np.cos(a), D * np.sin(a)], 1)
        w = np.exp(-(pz.mean() ** 2) / (2 * 8 ** 2))
        def dodaj_kf():
            n = normalne(p); mk = np.isfinite(n[:, 0]); c, s_ = np.cos(T[2]), np.sin(T[2]); Rm = np.array([[c, s_], [-s_, c]])
            kf.append((T.copy(), p, n)); frag.append((seg, fr_id)); kf_seg.append((T.copy(), p[mk] @ Rm + T[:2], n[mk] @ Rm))
        if not kf_seg:
            dodaj_kf(); poses.append((tm, *T, w)); yaw_ost = ym; t_ok = tm; continue
        dth = znak * (ym - yaw_ost)
        if zaburz is not None:
            T = T + np.array([zaburz[2], zaburz[3], np.radians(zaburz[4])]); zaburz = None
        luka = (not po_pauzie and t_ok is not None and tm - t_ok > PAR['luka_s'])
        if luka: v = np.zeros(2)
        T0 = np.array([T[0] + v[0], T[1] + v[1], T[2] + dth])
        poz = np.array([x[0][:2] for x in kf_seg])
        bl = np.where(np.hypot(*(poz - T0[:2]).T) < promien)[0]
        wyb = sorted(set(bl[-PAR['sub_max']:].tolist()) | set(range(max(0, len(kf_seg) - 10), len(kf_seg))))
        P = np.vstack([kf_seg[i][1] for i in wyb]); N = np.vstack([kf_seg[i][2] for i in wyb])
        ps = p[::PAR['co_ktory']]
        tree = cKDTree(P)
        if po_pauzie:
            # po pauzie operator stoi mniej wiecej w tym samym miejscu: szukamy pozycji w promieniu ok. 0,6 m
            v = np.zeros(2); wyn = None
            for ddx in (0, -0.4, 0.4):
                for ddy in (0, -0.4, 0.4):
                    r_ = icp(ps, P, N, tree, T0 + [ddx, ddy, 0], prior=(T0[2], imu_w), max_d=0.8, iters=25)
                    if r_ is not None and (wyn is None or r_[3] - 2 * r_[1] > wyn[3] - 2 * wyn[1]): wyn = r_
            po_pauzie = False
            if wyn is None: info.append((tm, 'icp')); continue
            Tn, rms, ev, frac = wyn
        else:
            wyn = icp(ps, P, N, tree, T0, prior=(T0[2], imu_w), huber=PAR['huber'], max_d=PAR['max_d'], iters=PAR['iters'])
            if luka:
                # po luce w sledzeniu (przechyl, brak IMU): sprawdzamy, czy skan pewnie pasuje do starej mapy.
                # Jesli nie - nie doklejamy na sile (to przesuwalo kuchnie/korytarz), tylko zaczynamy nowy
                # fragment od zera; fragmenty zszywa potem posegraf.zszyj po wspolnych scianach.
                q = None if wyn is None else (wyn[3], wyn[1], float(np.linalg.norm(wyn[0][:2] - T0[:2])))
                if q is not None:
                    # jednoznacznosc: start z +-0,5 m nie moze dawac innej, rownie dobrej pozycji (rownolegle sciany)
                    for ddx, ddy in ((0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5)):
                        r_ = icp(ps, P, N, tree, T0 + [ddx, ddy, 0], prior=(T0[2], imu_w), huber=PAR['huber'], max_d=0.6, iters=25)
                        if (r_ is not None and np.linalg.norm(r_[0][:2] - wyn[0][:2]) > 0.15 and r_[3] > wyn[3] - 0.05
                                and r_[1] < 1.5 * wyn[1] + 0.005):
                            q = (q[0], q[1], q[2], 'niejednoznaczne'); break
                LOG_LUK.append((tm, tm - t_ok, q))
                if q is None or len(q) > 3 or q[0] < PAR['luka_frac'] or q[1] > PAR['luka_rms'] or q[2] > 0.5 + 0.6 * (tm - t_ok):
                    fr_id += 1; info.append((tm, 'fragment')); frag_luka[fr_id] = tm - t_ok
                    # start fragmentu w najlepszej znanej pozycji (wynik ICP, jesli byl) - zszywanie zmienia ja
                    # tylko wtedy, gdy znajdzie wyraznie lepsze i jednoznaczne dopasowanie
                    T = wyn[0].copy() if wyn is not None else np.array([T[0], T[1], T0[2]]); kf_seg = []
                    dodaj_kf(); poses.append((tm, *T, w)); yaw_ost = ym; t_ok = tm; continue
            if wyn is None: info.append((tm, 'icp')); continue
            Tn, rms, ev, frac = wyn
            if frac < 0.3 or np.linalg.norm(Tn[:2] - T0[:2]) > 0.4: info.append((tm, 'skok')); Tn = T0
        v = 0.5 * v + 0.5 * (Tn[:2] - T[:2]); T = Tn; yaw_ost = ym; t_ok = tm
        poses.append((tm, *T, w))
        Tk = kf_seg[-1][0]
        if np.hypot(*(T[:2] - Tk[:2])) > PAR['kf_dl'] or abs(T[2] - Tk[2]) > np.radians(PAR['kf_kat']):
            dodaj_kf()
    mapa.frag = np.array(frag, int).reshape(-1, 2); mapa.frag_luka = frag_luka
    return np.array(poses), kf, info, pion

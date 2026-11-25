"""Pierwszy potok mapy 2D: obroty LD19 -> deskew z yaw IMU -> ICP point-to-line scan-to-submap."""
import sys, numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))

def wczytaj(kat):
    z = np.load(kat + '_dek.npz')
    return {k: z[k] for k in z.files}

def wczytaj_lub_dekoduj(kat):
    """Zdekodowane dane z pamieci podrecznej <sesja>/_wynik/dek.npz (odswiezanej, gdy lidar.bin jest nowszy)."""
    import os
    from dekoduj import dekoduj
    p = os.path.join(kat, '_wynik', 'dek.npz')
    if os.path.exists(p) and os.path.getmtime(p) >= os.path.getmtime(os.path.join(kat, 'lidar.bin')):
        z = np.load(p); return {k: z[k] for k in z.files}
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return dekoduj(kat, p)

def obroty(z, rmin=0.3, rmax=9.0):
    """Dzieli pakiety na obroty; kazdy punkt z czasem z interpolacji wewnatrz pakietu."""
    a0, a1, d, tl = z['a0'], z['a1'], z['d'], z['tl']
    krok = ((a1 - a0) % 360) / 11
    ang = (a0[:, None] + krok[:, None] * np.arange(12)) % 360
    dtp = np.median(np.diff(tl))
    tp = tl[:, None] + (np.arange(12) - 11) * dtp / 12      # znacznik pakietu ~ ostatni punkt
    nowy = np.where(np.diff(a0) < -180)[0] + 1
    gr = np.split(np.arange(len(a0)), nowy)
    out = []
    for g in gr[1:-1]:
        A, D, Tt = ang[g].ravel(), d[g].ravel(), tp[g].ravel()
        m = (D > rmin) & (D < rmax)
        # operator: glowa/plecak/tyczka widziane z tylu glowicy (kat 130-230 st., < 0,7 m) poruszaja sie razem
        # z czujnikiem - zatruwaja dopasowanie, wiec ten wycinek odrzucamy
        m &= ~((D < 0.7) & (A > 130) & (A < 230))
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

def mapa(kat, znak=+1, t_od=0, t_do=1e9, imu_w=200.0, pion=None, log=False, segmenty=None, promien=8.0, dane=None):
    z = dane if dane is not None else wczytaj(kat); sk = obroty(z)
    ti, yaw = z['ti'], np.radians(z['yaw'])
    pit, rol = z['pitch'], z['roll']
    if pion is None:
        m10 = ti < ti[0] + 10; pion = (np.median(pit[m10]), np.median(rol[m10]))
    prz = np.degrees(np.arccos(np.clip(np.cos(np.radians(pit - pion[0])) * np.cos(np.radians(rol - pion[1])), -1, 1)))
    poses, kf, info = [], [], []
    T = np.zeros(3); v = np.zeros(2); ostT = None; yaw_ost = None
    seg_ost = None; kfw = []   # kluczowe w ukladzie swiata (punkty, normalne, pozycja, segment)
    for (tt, ang, D) in sk:
        tm = tt.mean()
        if tm < t_od or tm > t_do: continue
        seg = 0 if segmenty is None else int(np.searchsorted(segmenty, tm))
        if seg != seg_ost:
            kf_seg = []; seg_ost = seg
        pz = np.interp(tt, ti, prz)
        if pz.max() > 12: info.append((tm, 'przechyl')); continue
        yp = np.interp(tt, ti, yaw); ym = np.interp(tm, ti, yaw)
        a = -ang + znak * (yp - ym)              # LD19 zgodnie z zegarem -> matematycznie przeciwnie; deskew obrotu
        p = np.stack([D * np.cos(a), D * np.sin(a)], 1)
        w = np.exp(-(pz.mean() ** 2) / (2 * 8 ** 2))
        def dodaj_kf():
            n = normalne(p); mk = np.isfinite(n[:, 0]); c, s_ = np.cos(T[2]), np.sin(T[2]); Rm = np.array([[c, s_], [-s_, c]])
            kf.append((T.copy(), p, n)); kf_seg.append((T.copy(), p[mk] @ Rm + T[:2], n[mk] @ Rm))
        if not kf_seg:
            dodaj_kf(); poses.append((tm, *T, w)); yaw_ost = ym; continue
        dth = znak * (ym - yaw_ost)
        T0 = np.array([T[0] + v[0], T[1] + v[1], T[2] + dth])
        poz = np.array([x[0][:2] for x in kf_seg])
        bl = np.where(np.hypot(*(poz - T0[:2]).T) < promien)[0]
        wyb = sorted(set(bl[-150:].tolist()) | set(range(max(0, len(kf_seg) - 10), len(kf_seg))))
        P = np.vstack([kf_seg[i][1] for i in wyb]); N = np.vstack([kf_seg[i][2] for i in wyb])
        ps = p[::2]
        tree = cKDTree(P)
        wyn = icp(ps, P, N, tree, T0, prior=(T0[2], imu_w))
        if wyn is None: info.append((tm, 'icp')); continue
        Tn, rms, ev, frac = wyn
        if frac < 0.3 or np.linalg.norm(Tn[:2] - T0[:2]) > 0.4: info.append((tm, 'skok')); Tn = T0
        v = 0.5 * v + 0.5 * (Tn[:2] - T[:2]); T = Tn; yaw_ost = ym
        poses.append((tm, *T, w))
        Tk = kf_seg[-1][0]
        if np.hypot(*(T[:2] - Tk[:2])) > 0.25 or abs(T[2] - Tk[2]) > np.radians(8):
            dodaj_kf()
    return np.array(poses), kf, info, pion

"""Porownanie mapy LiDAR z rzutem CAD (wzorzec): dopasowanie sztywne chmury do lic scian z CAD
i statystyki odchylek. Uzywane do oceny i strojenia algorytmu.

    python3 porownaj_cad.py <sesja> <rzut.dxf> [--warstwa Z_POWIERZCHNIE] [--pomin x0,y0,x1,y1 ...]
"""
import argparse, json, os, sys
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))


def odcinki_cad(plik, warstwy=('Z_POWIERZCHNIE',), pomin=(), obszar=None):
    """Odcinki lic scian z CAD (metry). pomin: prostokaty (x0,y0,x1,y1) wylaczone z porownania (np. przerobki)."""
    import ezdxf
    from ezdxf import path as P_
    d = ezdxf.readfile(plik)
    S = []
    for e in d.modelspace():
        if e.dxf.layer not in warstwy or e.dxftype() not in ('LINE', 'LWPOLYLINE', 'POLYLINE'):
            continue
        try:
            pts = np.array([(q.x, q.y) for q in P_.make_path(e).flattening(0.01)])
        except Exception:
            continue
        if e.dxftype() == 'LWPOLYLINE' and e.closed and len(pts) > 2 and np.linalg.norm(pts[0] - pts[-1]) > 1e-6:
            pts = np.vstack([pts, pts[:1]])
        for a, b in zip(pts[:-1], pts[1:]):
            if np.linalg.norm(b - a) > 0.05:
                S.append((a, b))
    S = np.array(S)
    if obszar:
        x0, y0, x1, y1 = obszar; sr = S.mean(1)
        S = S[(sr[:, 0] > x0) & (sr[:, 0] < x1) & (sr[:, 1] > y0) & (sr[:, 1] < y1)]
    # odcinki w obszarach przerobek - wylaczone
    if len(pomin):
        sr = S.mean(1)
        m = np.ones(len(S), bool)
        for x0, y0, x1, y1 in pomin:
            m &= ~((sr[:, 0] > x0) & (sr[:, 0] < x1) & (sr[:, 1] > y0) & (sr[:, 1] < y1))
        S = S[m]
    return S


def probki(S, krok=0.02):
    """Gesto probkowane punkty na odcinkach CAD + normalne (do dopasowania point-to-line)."""
    P, N = [], []
    for a, b in S:
        L = np.linalg.norm(b - a); n = max(2, int(L / krok))
        t = np.linspace(0, 1, n)[:, None]
        P.append(a + (b - a) * t)
        u = (b - a) / L; N.append(np.repeat([[-u[1], u[0]]], n, 0))
    return np.vstack(P), np.vstack(N)


def dopasuj(Q, S, pomin=(), log=print):
    """Dopasowanie sztywne (obrot, przesuniecie) chmury Q do lic CAD. Zwraca (R, t, statystyki)."""
    C, Cn = probki(S)
    tree = cKDTree(C)
    # obszary przerobek wylaczamy tez z chmury przy ocenie
    def wewn_pomin(X):
        m = np.zeros(len(X), bool)
        for x0, y0, x1, y1 in pomin:
            m |= (X[:, 0] > x0) & (X[:, 0] < x1) & (X[:, 1] > y0) & (X[:, 1] < y1)
        return m
    # siatka odleglosci do CAD (5 cm) do zgrubnego szukania
    kom = 0.05; o = C.min(0) - 3; sh = ((C.max(0) + 3 - o) / kom).astype(int) + 1
    G = np.ones(sh, bool); ij = ((C - o) / kom).astype(int); G[ij[:, 0], ij[:, 1]] = False
    DT = ndimage.distance_transform_edt(G) * kom
    def ocena(R, t, X):
        Y = X @ R.T + t; k = ((Y - o) / kom).astype(int)
        ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < sh[0]) & (k[:, 1] < sh[1])
        d = np.full(len(Y), 1.0); d[ok] = DT[k[ok, 0], k[ok, 1]]
        return np.mean(np.minimum(d, 0.5))
    # zgrubnie: obrot co 1 st. (mapa jest wyprostowana, CAD w osiach - wystarcza ok. 0/90/180/270 +-4 st.),
    # przesuniecie z korelacji rastrow (FFT) chmury i lic CAD
    from scipy.signal import fftconvolve
    kr = 0.05
    Cr = np.zeros(sh, float); Cr[ij[:, 0], ij[:, 1]] = 1; Cr = ndimage.gaussian_filter(Cr, 1.5)
    best = None
    for baza in (0, 90, 180, 270):
        for st in np.arange(-6, 6.1, 1.0):
            a_ = np.radians(baza + st); R = np.array([[np.cos(a_), -np.sin(a_)], [np.sin(a_), np.cos(a_)]])
            Y = Q @ R.T; m0 = Y.min(0)
            kq = ((Y - m0) / kr).astype(int); shq = kq.max(0) + 1
            Qr = np.zeros(shq, float); np.add.at(Qr, (kq[:, 0], kq[:, 1]), 1); Qr = np.minimum(Qr, 3)
            K = fftconvolve(Cr, Qr[::-1, ::-1], mode='full')
            i, j = np.unravel_index(np.argmax(K), K.shape)
            # polozenie: komorka (0,0) chmury trafia w (i - shq0 + 1, j - shq1 + 1) siatki CAD
            t = o + np.array([i - shq[0] + 1, j - shq[1] + 1]) * kr - m0
            sc = ocena(R, t, Q[::7])
            if best is None or sc < best[0]:
                best = (sc, R, t)
    _, R, t = best
    # dokladnie: point-to-line ICP, odporne
    X = Q
    for it in range(40):
        Y = X @ R.T + t
        d, j = tree.query(Y, distance_upper_bound=0.25 if it < 10 else 0.12)
        m = np.isfinite(d) & ~wewn_pomin(Y)
        if m.sum() < 100: break
        n = Cn[j[m]]; r = np.sum((Y[m] - C[j[m]]) * n, 1)
        w = 1 / np.maximum(np.abs(r), 0.01)
        Jm = np.stack([n[:, 0], n[:, 1], n[:, 1] * Y[m, 0] - n[:, 0] * Y[m, 1]], 1)
        H = (Jm * w[:, None]).T @ Jm; g = (Jm * w[:, None]).T @ r
        dx = -np.linalg.solve(H + 1e-9 * np.eye(3), g)
        da = dx[2]; Rd = np.array([[np.cos(da), -np.sin(da)], [np.sin(da), np.cos(da)]])
        R = Rd @ R; t = Rd @ t + dx[:2]
        if np.abs(dx).max() < 1e-5: break
    Y = X @ R.T + t
    d, j = tree.query(Y)
    m = ~wewn_pomin(Y)
    dd = d[m]
    wew = dd < 0.20
    stat = {'punktow': int(m.sum()), 'zgodnych_20cm_proc': round(100 * wew.mean(), 1),
            'mediana_mm': round(1000 * np.median(dd[wew]), 1), 'rms_mm': round(1000 * np.sqrt(np.mean(dd[wew] ** 2)), 1),
            'p90_mm': round(1000 * np.percentile(dd[wew], 90), 1), 'obrot_st': round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 2)}
    # pokrycie CAD: jaka czesc lic ma punkty skanu w 10 cm
    tq = cKDTree(Y[m]); dc, _ = tq.query(C)
    stat['pokrycie_cad_proc'] = round(100 * float(np.mean(dc < 0.10)), 1)
    # blad lic: dla kazdej probki sciany CAD odleglosc do najblizszego punktu skanu (meble stoja dalej, nie psuja)
    stat['blad_lic_mm'] = round(1000 * float(np.median(dc[dc < 0.15])), 1)
    stat['blad_lic_p90_mm'] = round(1000 * float(np.percentile(dc[dc < 0.15], 90)), 1)
    return R, t, stat, Y


def chmura_sesji(kat, **kw):
    """Chmura kondygnacji (punkty widziane z >=3 skanow) z pelnego przetwarzania - uklad mapy."""
    import slam, posegraf, sprawdz as S_
    from eksport_dxf import kondygnacja, KOM, MIN_KF
    z = slam.wczytaj_lub_dekoduj(kat); t0 = int(z['t0'])
    pozi = list(S_.poziomy(S_.wysokosci(os.path.join(kat, 'baro.bin'))))
    seg = kw.pop('segment', None)
    pet = kw.pop('petle', False); pro = kw.pop('prostuj', True); kw_szlif = kw.pop('szlif', 0); kw_zsz = kw.pop('zszyj', True); kw.setdefault('rzutuj', False)
    P, kf, info, pion = slam.mapa(kat, znak=-1, segmenty=[(b - t0) / 1e9 for a, b, h in pozi[:-1]], dane=z, **kw)
    kt = np.array([P[np.argmin(np.abs(P[:, 1:4] - Tk).sum(1)), 0] for Tk, _, _ in kf])
    if kw_zsz:
        Xz = posegraf.zszyj(kf, slam.mapa.frag, slam.mapa.frag_luka, log=print)
        kf = [(Xz[i], kf[i][1], kf[i][2]) for i in range(len(kf))]
    granice = [(a - t0) / 1e9 for a, b, h in pozi] + [1e12]
    s_ = seg if seg is not None else int(np.argmax([np.sum((kt >= granice[i]) & (kt < granice[i + 1])) for i in range(len(pozi))]))
    idx = np.where((kt >= granice[s_]) & (kt < granice[s_ + 1]))[0]
    sub = [kf[i] for i in idx]
    if pet:
        Xg, L = posegraf.graf(sub, kt[idx], log=lambda *a: None)
        sub = [(Xg[j], sub[j][1], sub[j][2]) for j in range(len(sub))]
    if pro:
        X = posegraf.prostuj(sub, log=lambda *a: None)
        sub = [(X[j], sub[j][1], sub[j][2]) for j in range(len(sub))]
    if kw_szlif:
        X = posegraf.doszlifuj(sub, rund=kw_szlif, log=lambda *a: None)
        sub = [(X[j], sub[j][1], sub[j][2]) for j in range(len(sub))]
    Q, N, ile = kondygnacja(sub, kt[idx], -1e12, 1e12)
    kom = np.round(Q / KOM).astype(np.int64)
    _, odw = np.unique(kom[:, 0] * 1000003 + kom[:, 1], return_inverse=True)
    par = np.unique(np.stack([odw, ile], 1), axis=0)
    widz = np.bincount(par[:, 0], minlength=odw.max() + 1)
    return Q[widz[odw] >= MIN_KF]


def pomieszczenia_cad(plik, warstwa='Z_POWIERZCHNIE', obszar=None, pomin=()):
    """Prostokatne obrysy pomieszczen z CAD (zamkniete polilinie) - do porownania wymiarow."""
    import ezdxf
    d = ezdxf.readfile(plik); out = []
    for e in d.modelspace().query(f'LWPOLYLINE[layer=="{warstwa}"]'):
        p = np.array([(v[0], v[1]) for v in e.get_points()])
        if len(p) < 4: continue
        x0, y0 = p.min(0); x1, y1 = p.max(0)
        if (x1 - x0) * (y1 - y0) < 1.0: continue
        if obszar and not (obszar[0] < x0 and x1 < obszar[2] and obszar[1] < y0 and y1 < obszar[3]): continue
        if any(not (x1 < a or x0 > c_ or y1 < b or y0 > d_) for a, b, c_, d_ in pomin): continue
        out.append((x0, y0, x1, y1))
    return sorted(set(out))


def wymiary(Y, prost, pas=0.12, margines=0.3):
    """Dla prostokata z CAD: polozenie kazdego boku zmierzone w skanie (mediana punktow w pasie) -> dlugosc i szerokosc."""
    x0, y0, x1, y1 = prost; wyn = {}
    def bok(os_, wart, a, b):
        o = 1 - os_
        m = (np.abs(Y[:, os_] - wart) < pas) & (Y[:, o] > a + margines) & (Y[:, o] < b - margines)
        return float(np.median(Y[m, os_])) if m.sum() > 30 else None
    L, R_, D, G = bok(0, x0, y0, y1), bok(0, x1, y0, y1), bok(1, y0, x0, x1), bok(1, y1, x0, x1)
    return {'cad_x': x1 - x0, 'cad_y': y1 - y0, 'skan_x': (R_ - L) if L is not None and R_ is not None else None,
            'skan_y': (G - D) if D is not None and G is not None else None}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja'); ap.add_argument('dxf')
    ap.add_argument('--warstwa', default='Z_POWIERZCHNIE')
    ap.add_argument('--pomin', nargs='*', default=[])
    ap.add_argument('--segment', type=int)
    ap.add_argument('--png')
    ap.add_argument('--obszar', default='55.5,53,75.5,62.5', help='prostokat rzutu w CAD x0,y0,x1,y1')
    a = ap.parse_args()
    pomin = [tuple(map(float, p.split(','))) for p in a.pomin]
    S = odcinki_cad(a.dxf, (a.warstwa,), pomin, tuple(map(float, a.obszar.split(','))) if a.obszar else None)
    Q = chmura_sesji(a.sesja.rstrip('/'), segment=a.segment)
    R, t, stat, Y = dopasuj(Q, S, pomin)
    print(json.dumps(stat, ensure_ascii=False))
    ob = tuple(map(float, a.obszar.split(','))) if a.obszar else None
    print('Pomieszczenia (CAD -> skan, cm):')
    for pr in pomieszczenia_cad(a.dxf, a.warstwa, ob, pomin):
        w = wymiary(Y, pr)
        txt = []
        for o in ('x', 'y'):
            if w['skan_' + o] is not None:
                txt.append(f"{o}: {100 * w['cad_' + o]:.1f} -> {100 * w['skan_' + o]:.1f} ({100 * (w['skan_' + o] - w['cad_' + o]):+.1f})")
        print(f"  {pr[0]:.2f},{pr[1]:.2f}: " + '; '.join(txt))
    if a.png:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        C, _ = probki(S); d, _ = cKDTree(C).query(Y)
        fig, ax = plt.subplots(figsize=(16, 8))
        for s in S: ax.plot(s[:, 0], s[:, 1], 'k-', lw=1)
        sc = ax.scatter(Y[:, 0], Y[:, 1], c=np.minimum(d, 0.2) * 100, s=0.5, cmap='turbo', vmin=0, vmax=20)
        for x0, y0, x1, y1 in pomin: ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls='--', color='gray'))
        plt.colorbar(sc, label='odległość od ściany CAD [cm]'); ax.set_aspect('equal')
        ax.set_xlim(S[:, :, 0].min() - 1, S[:, :, 0].max() + 1); ax.set_ylim(S[:, :, 1].min() - 1, S[:, :, 1].max() + 1)
        ax.set_title(f"{os.path.basename(a.sesja)}: mediana {stat['mediana_mm']} mm, RMS {stat['rms_mm']} mm, zgodnych {stat['zgodnych_20cm_proc']} %, pokrycie CAD {stat['pokrycie_cad_proc']} %")
        plt.tight_layout(); plt.savefig(a.png, dpi=100)

"""Naklada mape z gotowego SLAM (Cartographer, slam_toolbox) na nasza mape i liczy odchylki w cm.

    python3 nakladka.py <katalog_sesji> [--metody carto_imu,carto]

Uzywa map juz policzonych przez uruchom.py (<sesja>/_porownanie/k<N>_<metoda>.pgm/.yaml)
i naszej mapy (<sesja>/_wynik/mapa.json, chmura scian najwiekszej kondygnacji).
Uklady obu map sa dowolne, wiec najpierw szukamy obrotu (co 2 st., tez odbicie lustrzane)
i przesuniecia (korelacja FFT rastrow), potem dokladnie ICP z obcinaniem dalekich par.

Wyniki w <sesja>/_porownanie/:
    k<N>_nakladka_<metoda>.png   - nasza mapa (szara) + ich sciany kolorem wg odchylki
    nakladka.json                - statystyki (mediana, 90 %, udzialy < 5/10/20 cm, pokrycie)
"""
import argparse, json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU)
NAZWY = {'slam_toolbox': 'slam_toolbox', 'carto': 'Cartographer (sam LiDAR)', 'carto_imu': 'Cartographer + IMU'}


def woksel(P, k=0.05):
    if not len(P): return P
    _, i = np.unique(np.floor(P / k).astype(np.int64), axis=0, return_index=True)
    return P[np.sort(i)]


def nasza_chmura(kat):
    M = json.load(open(os.path.join(kat, '_wynik', 'mapa.json'), encoding='utf-8'))
    k = max(M['kondygnacje'], key=lambda k: len(k['chmura_cm']))
    return np.array(k['chmura_cm'], float).reshape(-1, 2) / 100


def raster(P, x0, y0, w, h, kom):
    from scipy.ndimage import gaussian_filter
    g = np.zeros((h, w), np.float32)
    c = ((P[:, 0] - x0) / kom).astype(int); r = ((P[:, 1] - y0) / kom).astype(int)
    ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
    np.add.at(g, (r[ok], c[ok]), 1.0)
    g = np.minimum(g, 3.0)
    return gaussian_filter(g, 1.2)


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def zgrubnie(A, B, kom=0.10, krok=2.0, log=print):
    """Najlepszy obrot (i odbicie) + przesuniecie B na A: korelacja FFT rastrow."""
    cA = A.mean(0); A0 = A - cA
    r = max(np.abs(A0).max(), np.abs(B - B.mean(0)).max()) + 1.0
    n = int(2 * r / kom) + 1
    N = 1 << int(math.ceil(math.log2(2 * n)))
    ga = raster(A0, -r, -r, n, n, kom)
    FA = np.fft.rfft2(ga, (N, N))
    naj = (-1, None)
    for lustro in (False, True):
        Bl = B * [1, -1] if lustro else B
        Bl = Bl - Bl.mean(0)
        for st in np.arange(0, 360, krok):
            Br = Bl @ rot(math.radians(st)).T
            gb = raster(Br, -r, -r, n, n, kom)
            kor = np.fft.irfft2(FA * np.conj(np.fft.rfft2(gb, (N, N))), (N, N))
            i = int(np.argmax(kor)); w = float(kor.flat[i])
            if w > naj[0]:
                dy, dx = divmod(i, N)
                dy = dy - N if dy > N // 2 else dy; dx = dx - N if dx > N // 2 else dx
                naj = (w, (lustro, st, np.array([dx, dy]) * kom))
    lustro, st, t = naj[1]
    log(f'  zgrubnie: obrot {st:.0f} st.{" + odbicie" if lustro else ""}, przesuniecie {t[0]:.2f}, {t[1]:.2f} m')
    # przeksztalcenie calkowite: x -> R (L x - mB) + t + cA
    L = np.diag([1.0, -1.0]) if lustro else np.eye(2)
    R = rot(math.radians(st)) @ L
    mB = (B * [1, -1] if lustro else B).mean(0)
    t_cal = -rot(math.radians(st)) @ mB + t + cA
    return R, t_cal, lustro


def icp_dokladny(A, B, R, t, progi=(0.5, 0.3, 0.2, 0.12, 0.08), it=15):
    """ICP punkt-punkt 2D z obcinaniem par dalszych niz prog (sciany widziane tylko w jednej mapie nie ciagna)."""
    drzewo = cKDTree(A)
    for prog in progi:
        for _ in range(it):
            Y = B @ R.T + t
            d, j = drzewo.query(Y, distance_upper_bound=prog)
            ok = np.isfinite(d)
            if ok.sum() < 20:
                break
            p, q = Y[ok], A[j[ok]]
            mp, mq = p.mean(0), q.mean(0)
            U, _, Vt = np.linalg.svd((p - mp).T @ (q - mq))
            D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
            Rk = Vt.T @ D @ U.T
            R = Rk @ R
            t = Rk @ t - Rk @ mp + mq
            if np.linalg.norm(mq - mp) < 1e-4:
                break
    return R, t


def statystyki(A, Y):
    d_ich = cKDTree(A).query(Y)[0]          # ich sciany -> nasze
    d_nas = cKDTree(Y).query(A)[0]          # nasze sciany -> ich
    wsp = d_ich < 0.30                       # tylko sciany widoczne w obu mapach
    dw = d_ich[wsp] if wsp.any() else d_ich
    pr = lambda x, p: round(float((dw < x).mean() * 100), 1)
    return d_ich, {
        'mediana_cm': round(float(np.median(dw)) * 100, 1),
        'p90_cm': round(float(np.percentile(dw, 90)) * 100, 1),
        'do_5cm_proc': pr(0.05, 0), 'do_10cm_proc': pr(0.10, 0), 'do_20cm_proc': pr(0.20, 0),
        'wspolne_proc': round(float(wsp.mean() * 100), 1),
        'pokrycie_naszej_proc': round(float((d_nas < 0.10).mean() * 100), 1),
    }


def rysuj(A, Y, d, plik, tytul, st):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    w = np.ptp(np.r_[A[:, 0], Y[:, 0]]); h = np.ptp(np.r_[A[:, 1], Y[:, 1]])
    s = min(18.0 / max(w, 1), 14.0 / max(h, 1))
    fig, ax = plt.subplots(figsize=(max(6, w * s) + 1.5, max(5, h * s) + 1.2))
    ax.scatter(A[:, 0], A[:, 1], s=3, c='#606060', lw=0, label='nasza mapa')
    cm = ListedColormap(['#1a9850', '#91cf60', '#fdae61', '#d73027', '#7b3294'])
    nrm = BoundaryNorm([0, 5, 10, 20, 30, 1000], cm.N)
    o = np.argsort(d)
    sc = ax.scatter(Y[o, 0], Y[o, 1], s=1.6, c=d[o] * 100, cmap=cm, norm=nrm, lw=0)
    cb = fig.colorbar(sc, ax=ax, shrink=0.7, ticks=[2.5, 7.5, 15, 25, 40])
    cb.ax.set_yticklabels(['< 5 cm', '5–10', '10–20', '20–30', '> 30 (brak w naszej)'])
    ax.set_aspect('equal'); ax.grid(alpha=0.25)
    ax.set_title(f"{tytul}\nmediana {st['mediana_cm']} cm · 90 %: {st['p90_cm']} cm · "
                 f"< 10 cm: {st['do_10cm_proc']} % · wspólne ściany: {st['wspolne_proc']} %", fontsize=11)
    ax.legend(loc='upper right', markerscale=8, fontsize=9)
    plt.tight_layout(); plt.savefig(plik, dpi=110); plt.close(fig)


def nakladka(kat, metody=None, log=print):
    from uruchom import wczytaj_mape, punkty_scian
    wy = os.path.join(kat, '_porownanie')
    A = woksel(nasza_chmura(kat))
    plik_json = os.path.join(wy, 'nakladka.json')
    try:
        wynik = json.load(open(plik_json, encoding='utf-8'))
    except (OSError, ValueError):
        wynik = {}
    for f in sorted(os.listdir(wy)):
        if not (f.startswith('k') and f.endswith('.yaml')): continue
        stem = f[:-5]; nr, m = stem[1:].split('_', 1)
        if metody and m not in metody: continue
        log(f'== nakladka: kondygnacja {nr}, {NAZWY.get(m, m)}')
        img, rz, org = wczytaj_mape(os.path.join(wy, stem))
        B = woksel(punkty_scian(img, rz, org))
        if len(B) < 50 or len(A) < 50:
            log('  za malo punktow - pomijam'); continue
        R, t, lustro = zgrubnie(A, B, log=log)
        R, t = icp_dokladny(A, B, R, t)
        Y = B @ R.T + t
        d, st = statystyki(A, Y)
        st.update(lustro=bool(lustro), obrot_st=round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 1))
        log(f"  mediana {st['mediana_cm']} cm, 90 % {st['p90_cm']} cm, < 10 cm {st['do_10cm_proc']} %, wspolne {st['wspolne_proc']} %")
        rysuj(A, Y, d, os.path.join(wy, f'k{nr}_nakladka_{m}.png'),
              f'{NAZWY.get(m, m)} na naszej mapie – kondygnacja {nr}', st)
        wynik[f'k{nr}_{m}'] = dict(st, metoda=m, kondygnacja=int(nr))
    json.dump(wynik, open(plik_json, 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    return wynik


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja')
    ap.add_argument('--metody', default='')
    a = ap.parse_args()
    nakladka(os.path.abspath(a.sesja.rstrip('/')), [m for m in a.metody.split(',') if m])

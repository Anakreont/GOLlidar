"""Graf pozycji 2D z zamykaniem petli (bez zewnetrznych bibliotek SLAM).
Wezly = kluczowe skany z slam.mapa(); krawedzie = kolejne skany (odometria ICP) + petle wykryte przez ICP
skanu do mapy lokalnej z innego czasu. Optymalizacja: scipy least_squares z odporna funkcja straty."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
import slam


def _R(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def wzgl(Ta, Tb):
    """Pozycja b w ukladzie a."""
    d = _R(Ta[2]).T @ (Tb[:2] - Ta[:2])
    return np.array([d[0], d[1], (Tb[2] - Ta[2] + np.pi) % (2 * np.pi) - np.pi])


def swiat(T, p):
    """Punkty skanu (uklad czujnika) w ukladzie mapy - ta sama konwencja co slam.py (p @ [[c,s],[-s,c]] + t)."""
    c, s = np.cos(T[2]), np.sin(T[2])
    return p @ np.array([[c, s], [-s, c]]) + T[:2]


def optymalizuj(X0, kraw, staly=0, f_scale=1.0):
    n = len(X0)
    I = np.array([k[0] for k in kraw]); J = np.array([k[1] for k in kraw])
    Z = np.array([k[2] for k in kraw]); Wg = np.array([k[3] for k in kraw])

    def res(x):
        X = x.reshape(-1, 3)
        Xa, Xb = X[I], X[J]
        c, s = np.cos(Xa[:, 2]), np.sin(Xa[:, 2])
        dx, dy = Xb[:, 0] - Xa[:, 0], Xb[:, 1] - Xa[:, 1]
        ex = c * dx + s * dy - Z[:, 0]
        ey = -s * dx + c * dy - Z[:, 1]
        et = (Xb[:, 2] - Xa[:, 2] - Z[:, 2] + np.pi) % (2 * np.pi) - np.pi
        r = np.stack([ex / 0.05, ey / 0.05, et / np.radians(1.0)], 1) * Wg[:, None]
        kot = (X[staly] - X0[staly]) * 1e3
        return np.concatenate([r.ravel(), kot])

    A = lil_matrix((3 * len(kraw) + 3, 3 * n), dtype=int)
    for e, (i, j) in enumerate(zip(I, J)):
        for k in range(3):
            A[3 * e + k, 3 * i:3 * i + 3] = 1
            A[3 * e + k, 3 * j:3 * j + 3] = 1
    A[3 * len(kraw):, 3 * staly:3 * staly + 3] = 1
    w = least_squares(res, X0.ravel(), jac_sparsity=A, loss='cauchy', f_scale=f_scale, x_scale='jac', max_nfev=200)
    return w.x.reshape(-1, 3)


def petle(X, kf, kt, juz=set(), odstep_s=8.0, promien=2.5, log=print):
    """Szuka zamkniec petli: skan i dopasowany do mapy lokalnej skanow j z innego czasu."""
    nowe = []
    poz = X[:, :2]; drzewo = cKDTree(poz)
    for i in range(len(kf)):
        if i % 2:                       # co drugi skan - wystarczy, a dwa razy szybciej
            continue
        kand = [j for j in drzewo.query_ball_point(poz[i], promien) if abs(kt[i] - kt[j]) > odstep_s and j < i]
        if not kand:
            continue
        j = min(kand, key=lambda j: np.hypot(*(poz[i] - poz[j])))
        if (i, j) in juz:
            continue
        sas = [k for k in range(max(0, j - 4), min(len(kf), j + 5)) if abs(kt[k] - kt[j]) < 4]
        P, N = [], []
        for k in sas:
            nk = kf[k][2]; mk = np.isfinite(nk[:, 0])
            P.append(swiat(X[k], kf[k][1][mk]))
            c, s = np.cos(X[k][2]), np.sin(X[k][2]); N.append(nk[mk] @ np.array([[c, s], [-s, c]]))
        P, N = np.vstack(P), np.vstack(N)
        if len(P) < 300:
            continue
        tree = cKDTree(P); src = kf[i][1][::2]
        best = None
        for dd in np.radians([0, -4, 4, -8, 8, -12, 12, -16, 16, -20, 20]):
            for dxy in ([0, 0], [0.3, 0], [-0.3, 0], [0, 0.3], [0, -0.3]):
                T0 = X[i] + [dxy[0], dxy[1], dd]
                r = slam.icp(src, P, N, tree, T0, iters=25, max_d=0.3)
                if r is None:
                    continue
                Tn, rms, ev, frac = r
                ocena = frac - 3 * rms
                if best is None or ocena > best[0]:
                    best = (ocena, Tn, rms, frac, ev)
        if best is None:
            continue
        _, Tn, rms, frac, ev = best
        if frac > 0.75 and rms < 0.03 and ev[0] > 50:
            nowe.append((j, i, wzgl(X[j], Tn), 1.0))
            juz.add((i, j))
    log(f'  nowe petle: {len(nowe)}')
    return nowe


def graf(kf, kt, rund=3, log=print):
    X = np.array([k[0] for k in kf], float)
    kraw = [(i, i + 1, wzgl(X[i], X[i + 1]), 1.0) for i in range(len(kf) - 1)]
    juz = set(); wszystkie_petle = []
    for r in range(rund):
        nowe = petle(X, kf, kt, juz, log=log)
        if not nowe and r:
            break
        wszystkie_petle += nowe
        X = optymalizuj(X, kraw + wszystkie_petle)
    log(f'  petli razem: {len(wszystkie_petle)}')
    return X, wszystkie_petle


def kierunek_skanu(n):
    """Glowny kierunek normalnych skanu modulo 90 st. i jego sila (ulamek normalnych w +-3 st.)."""
    n = n[np.isfinite(n[:, 0])]
    if len(n) < 40:
        return None, 0.0
    a = np.arctan2(n[:, 1], n[:, 0]) % (np.pi / 2)
    h, e = np.histogram(a, bins=90, range=(0, np.pi / 2))
    h = np.convolve(np.r_[h[-3:], h, h[:3]], np.ones(7), 'same')[3:-3]
    i = np.argmax(h)
    return e[i] + e[1] / 2, h[i] / len(n)


def prostuj(kf, sigma_st=1.5, log=print):
    """Budynki maja katy proste: kurs kazdego skanu z wyraznym ukladem scian dociagamy (miekko) do wspolnego
    kierunku modulo 90 st., a odometria ICP rozklada poprawke na trase. Skany bez wyraznego ukladu
    (skosy, meble) i z odchylka > 8 st. nie wplywaja na wynik."""
    X0 = np.array([k[0] for k in kf], float)
    kraw = [(i, i + 1, wzgl(X0[i], X0[i + 1]), 1.0) for i in range(len(kf) - 1)]
    fi, sila = [], []
    for T, p, n in kf:
        f, s = kierunek_skanu(n)
        fi.append(np.nan if f is None else f); sila.append(s)
    fi, sila = np.array(fi), np.array(sila)
    # konwencja slam.py: p_swiat = R(th) p, wiec kierunek scian w ukladzie mapy = fi + th
    kat = (fi + X0[:, 2]) % (np.pi / 2)
    ok = np.isfinite(kat) & (sila > 0.35)
    if ok.sum() < 10:
        log('  prostowanie: za malo skanow z wyraznym ukladem scian'); return X0
    z = np.exp(4j * kat[ok]) @ sila[ok]
    PHI = (np.angle(z) / 4) % (np.pi / 2)
    n = len(X0)
    I = np.array([k[0] for k in kraw]); J = np.array([k[1] for k in kraw]); Z = np.array([k[2] for k in kraw])
    idx = np.where(ok)[0]
    def res(x):
        X = x.reshape(-1, 3)
        Xa, Xb = X[I], X[J]
        c, s = np.cos(Xa[:, 2]), np.sin(Xa[:, 2])
        dx, dy = Xb[:, 0] - Xa[:, 0], Xb[:, 1] - Xa[:, 1]
        ex = c * dx + s * dy - Z[:, 0]; ey = -s * dx + c * dy - Z[:, 1]
        et = (Xb[:, 2] - Xa[:, 2] - Z[:, 2] + np.pi) % (2 * np.pi) - np.pi
        r = np.stack([ex / 0.03, ey / 0.03, et / np.radians(0.7)], 1).ravel()
        e = ((fi[idx] + X[idx, 2]) - PHI + np.pi / 4) % (np.pi / 2) - np.pi / 4
        m = (np.sqrt(sila[idx] / 0.35) * e / np.radians(sigma_st))
        kot = (X[0, :2] - X0[0, :2]) * 1e3
        return np.concatenate([r, m, kot])
    A = lil_matrix((3 * len(kraw) + len(idx) + 2, 3 * n), dtype=int)
    for e_, (i, j) in enumerate(zip(I, J)):
        A[3 * e_:3 * e_ + 3, 3 * i:3 * i + 3] = 1; A[3 * e_:3 * e_ + 3, 3 * j:3 * j + 3] = 1
    for q, i in enumerate(idx):
        A[3 * len(kraw) + q, 3 * i + 2] = 1
    A[3 * len(kraw) + len(idx):, 0:2] = 1
    w = least_squares(res, X0.ravel(), jac_sparsity=A, loss='cauchy', f_scale=2.0, x_scale='jac', max_nfev=100)
    X = w.x.reshape(-1, 3)
    e0 = np.degrees(((fi[idx] + X0[idx, 2]) - PHI + np.pi / 4) % (np.pi / 2) - np.pi / 4)
    e1 = np.degrees(((fi[idx] + X[idx, 2]) - PHI + np.pi / 4) % (np.pi / 2) - np.pi / 4)
    log(f'  prostowanie: {len(idx)} skanow, odchylka od katow prostych mediana {np.median(np.abs(e0)):.2f} -> {np.median(np.abs(e1)):.2f} st.')
    return X


def przenies(P, kt, X0, X):
    """Poprawka kluczowych skanow przeniesiona na wszystkie pozycje trasy (wg poprzedzajacego kluczowego)."""
    P = P.copy()
    j = np.clip(np.searchsorted(kt, P[:, 0], side='right') - 1, 0, len(kt) - 1)
    for i in range(len(P)):
        a, b = X0[j[i]], X[j[i]]
        loc = _R(a[2]).T @ (P[i, 1:3] - a[:2])
        P[i, 1:3] = b[:2] + _R(b[2]) @ loc
        P[i, 3] = P[i, 3] + (b[2] - a[2])
    return P

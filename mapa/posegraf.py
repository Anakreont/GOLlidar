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


def petle(X, kf, kt, juz=None, odstep_s=8.0, promien=1.5, krok=3, log=print):
    """Szuka zamkniec petli: skan i dopasowany do mapy lokalnej skanow j z innego czasu."""
    juz = set() if juz is None else juz
    nowe = []
    poz = X[:, :2]; drzewo = cKDTree(poz)
    for i in range(len(kf)):
        if i % krok:
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
        for dd in np.radians([0, -3, 3, -6, 6]):
            for dxy in ([0, 0],):
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


def graf(kf, kt, rund=2, log=print):
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


def obroc_fragmenty(kf, frag, min_skanow=8, min_zgodnosc=0.5, min_kat=1.5, log=print):
    """Fragment po luce w sledzeniu (przechyl, glowica polozona na bok) bywa doklejony pod zlym katem (10-30 st.),
    bo kurs IMU po takiej przerwie jest niepewny. Budynek ma katy proste: kierunek scian fragmentu (modulo 90 st.)
    powinien byc taki sam jak w dotychczasowej mapie. Fragment i wszystkie nastepne obracane sa wokol pierwszego
    skanu fragmentu o roznice kierunkow (do +-45 st.). Tylko gdy oba kierunki sa wyrazne."""
    X = np.array([k[0] for k in kf], float)
    fi, sila = [], []
    for T, p, n in kf:
        f, s = kierunek_skanu(n)
        fi.append(np.nan if f is None else f); sila.append(s)
    fi, sila = np.array(fi), np.array(sila)
    ok = np.isfinite(fi) & (sila > 0.35)
    def kier(idx):
        idx = idx[ok[idx]]
        if len(idx) < min_skanow: return None
        z = np.exp(4j * (fi[idx] + X[idx, 2])) @ sila[idx]
        return (np.angle(z) / 4) % (np.pi / 2), abs(z) / sila[idx].sum()
    for seg in np.unique(frag[:, 0]):
        fr_s = list(dict.fromkeys(frag[frag[:, 0] == seg, 1].tolist()))
        for f in fr_s[1:]:
            A_idx = np.where((frag[:, 0] == seg) & (frag[:, 1] < f))[0]; B_idx = np.where((frag[:, 0] == seg) & (frag[:, 1] == f))[0]
            ka, kb = kier(A_idx), kier(B_idx)
            if ka is None or kb is None or ka[1] < min_zgodnosc or kb[1] < min_zgodnosc: continue
            d = (ka[0] - kb[0] + np.pi / 4) % (np.pi / 2) - np.pi / 4
            if abs(d) < np.radians(min_kat): continue
            nast = np.where((frag[:, 0] == seg) & (frag[:, 1] >= f))[0]; c = X[B_idx[0], :2].copy()
            X[nast, :2] = (X[nast, :2] - c) @ _R(d).T + c; X[nast, 2] += d
            log(f'  fragment {f}: obrot do kierunku scian o {np.degrees(d):+.1f} st. ({len(B_idx)} skanow, zgodnosc {kb[1]:.2f})')
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


def doszlifuj(kf, rund=2, promien=6.0, log=print):
    """Drugi przebieg: kazdy kluczowy skan dopasowany do mapy zlozonej z WSZYSTKICH pozostalych skanow
    (nie tylko poprzednich) - wyrownuje bledy narastajace w pierwszym przebiegu. Zmiany ograniczone (<10 cm, <1,5 st.)."""
    import slam
    X = np.array([k[0] for k in kf], float)
    for r_ in range(rund):
        W = []; Nw = []
        for (T, p, n), x in zip(kf, X):
            mk = np.isfinite(n[:, 0]); c, s = np.cos(x[2]), np.sin(x[2])
            W.append(swiat(x, p[mk])); Nw.append(n[mk] @ np.array([[c, s], [-s, c]]))
        idx = np.concatenate([np.full(len(w), i) for i, w in enumerate(W)]); P = np.vstack(W); N = np.vstack(Nw)
        tree_all = cKDTree(P)
        zm = []
        Xn = X.copy()
        for i, (T, p, n) in enumerate(kf):
            bl = tree_all.query_ball_point(X[i, :2], promien)
            bl = np.asarray(bl); bl = bl[np.abs(idx[bl] - i) > 2]
            if len(bl) < 300: continue
            Pt, Nt = P[bl], N[bl]
            r = slam.icp(p[::2], Pt, Nt, cKDTree(Pt), X[i], iters=10, max_d=0.15, huber=0.03)
            if r is None: continue
            Tn, rms, ev, frac = r
            d = Tn - X[i]; d[2] = (d[2] + np.pi) % (2 * np.pi) - np.pi
            if frac > 0.5 and np.hypot(d[0], d[1]) < 0.10 and abs(d[2]) < np.radians(1.5):
                Xn[i] = Tn; zm.append(np.hypot(d[0], d[1]))
        X = Xn
        log(f'  szlif {r_ + 1}: poprawiono {len(zm)} skanow, srednio {1000 * np.mean(zm) if zm else 0:.1f} mm')
    return X


def dopracuj(kf, kt, rund=None, promien=8.0, odstep=8.0, log=print):
    """Po pierwszym przebiegu: kazdy kluczowy skan dopasowany do scian widzianych w INNYM czasie (o > odstep s
    wczesniej lub pozniej), z kontrola widocznosci lica. Sasiednie w czasie skany sa pomijane, bo niosa ten sam
    blad. Usuwa podwojne sciany po odjezdzie trasy w ciasnym pomieszczeniu (skany stamtad widza hol przez drzwi).
    W kierunku slabo okreslonym pozycja zostaje (wiez jak przy degeneracji). Zmiana przyjmowana do 0,35 m / 8 st."""
    import slam
    X = np.array([k[0] for k in kf], float); kt = np.asarray(kt, float)
    rund = int(slam.PAR.get('dopr_rund', 3)) if rund is None else rund
    for r_ in range(rund):
        W = []; Nw = []
        for (T, p, n), x in zip(kf, X):
            mk = np.isfinite(n[:, 0]); c, s = np.cos(x[2]), np.sin(x[2])
            Pw = swiat(x, p[mk]); W.append(Pw); Nw.append(slam._do_czujnika(Pw, n[mk] @ np.array([[c, s], [-s, c]]), x))
        zm = []; Xn = X.copy()
        for i, (T, p, n) in enumerate(kf):
            o = np.where((np.abs(kt - kt[i]) > odstep) & (np.hypot(*(X[:, :2] - X[i, :2]).T) < promien))[0]
            if len(o) < 3: continue
            if len(o) > 80: o = o[np.argsort(np.hypot(*(X[o, :2] - X[i, :2]).T))[:80]]
            Pt = np.vstack([W[j] for j in o]); Nt = np.vstack([Nw[j] for j in o])
            if len(Pt) < 300: continue
            r = slam.icp_widok(p[::2], Pt, Nt, cKDTree(Pt), X[i].copy(), max_d=0.4 if r_ == 0 else 0.2,
                               prior=(X[i, 2], 5.0), deg=(X[i, 0], X[i, 1], slam.PAR['deg_prog']))
            if r is None: continue
            Tn, rms, ev, inl = r
            d = Tn - X[i]
            if inl >= 0.35 and rms <= 0.03 and np.hypot(d[0], d[1]) < 0.35 and abs(d[2]) < np.radians(8):
                Xn[i] = Tn; zm.append(np.hypot(d[0], d[1]))
                if slam.PAR.get('dopr_gs'):       # Gauss-Seidel: poprawiony skan od razu jest celem dla nastepnych
                    X[i] = Tn; mk = np.isfinite(n[:, 0]); c, s = np.cos(Tn[2]), np.sin(Tn[2])
                    W[i] = swiat(Tn, p[mk]); Nw[i] = slam._do_czujnika(W[i], n[mk] @ np.array([[c, s], [-s, c]]), Tn)
        X = Xn
        log(f'  dopracowanie {r_ + 1}: poprawiono {len(zm)} skanow, srednio {1000 * np.mean(zm) if zm else 0:.1f} mm, najw. {1000 * max(zm) if zm else 0:.0f} mm')
    return X


def _chmura(kf, idx, co=3):
    P = [swiat(kf[i][0], kf[i][1][::co]) for i in idx]
    N = []
    for i in idx:
        c, s = np.cos(kf[i][0][2]), np.sin(kf[i][0][2])
        N.append(kf[i][2][::co] @ np.array([[c, s], [-s, c]]))
    P, N = np.vstack(P), np.vstack(N); m = np.isfinite(N[:, 0])
    return P[m], N[m]


def zszyj(kf, frag, luki=None, v_max=1.2, katy=6.0, poczatek=40, log=print):
    """Fragmenty trasy rozdzielone luka w sledzeniu (przechyl, brak IMU) dopasowuje sztywno do wczesniejszych
    fragmentow tej samej kondygnacji po wspolnych scianach. Zasieg szukania rosnie z dlugoscia luki
    (operator idzie dalej, gdy skaner nie sledzi): 0,5 m + v_max * luka. Dla kazdego obrotu (+-katy st.)
    przesuniecie z korelacji rastrow (FFT) poczatku fragmentu z wczesniejsza mapa, potem ICP punkt-prosta.
    Poprawka idzie na caly fragment i wszystkie nastepne. Przyjmowana tylko, gdy wyraznie lepsza i jednoznaczna."""
    from scipy.spatial import cKDTree
    from scipy import ndimage
    from scipy.signal import fftconvolve
    luki = luki or {}
    X = np.array([k[0] for k in kf], float)
    raport = []
    for seg in np.unique(frag[:, 0]):
        fr_s = list(dict.fromkeys(frag[frag[:, 0] == seg, 1].tolist()))
        for f in fr_s[1:]:
            A_idx = np.where((frag[:, 0] == seg) & (frag[:, 1] < f))[0]
            B_idx = np.where((frag[:, 0] == seg) & (frag[:, 1] == f))[0]
            if len(A_idx) < 15 or len(B_idx) < 8:
                raport.append((int(seg), f, 'za maly')); continue
            kf = [(X[i], kf[i][1], kf[i][2]) for i in range(len(kf))]
            A, An = _chmura(kf, A_idx, 2); B, Bn = _chmura(kf, B_idx, 3); B0, _ = _chmura(kf, B_idx[:poczatek], 2)
            zas = min(0.5 + v_max * float(luki.get(f, 3.0)), 10.0)
            c0 = X[B_idx[0], :2].copy()
            kom = 0.05; o = A.min(0) - zas - 1; sh = ((A.max(0) + zas + 1 - o) / kom).astype(int) + 1
            G = np.ones(sh, bool); ij = ((A - o) / kom).astype(int); G[ij[:, 0], ij[:, 1]] = False
            DT = np.minimum(ndimage.distance_transform_edt(G) * kom, 0.25)
            Ar = ndimage.gaussian_filter((~G).astype(float), 1.0)
            # wolna przestrzen wczesniejszej mapy: komorki, przez ktore przeszly promienie (do 15 cm przed sciana).
            # Zle dopasowanie kladzie sciany fragmentu w srodku pokoju, ktory byl widziany jako pusty.
            WOL = np.zeros(sh, bool)
            for i in A_idx[::2]:
                T_, p_ = X[i], kf[i][1][::4]
                Pw = swiat(T_, p_); d_ = np.linalg.norm(p_, axis=1)
                for f_ in np.arange(0.0, 1.0, 0.02):
                    q_ = T_[:2] + (Pw - T_[:2]) * f_
                    mq = d_ * (1 - f_) > 0.15
                    k = ((q_[mq] - o) / kom).astype(int)
                    WOL[k[:, 0], k[:, 1]] = True
            WOL &= G
            WOL = ndimage.binary_erosion(WOL, iterations=1)
            rng = np.random.default_rng(0); Bs = B0[rng.choice(len(B0), min(len(B0), 3000), replace=False)]
            Bc = B[rng.choice(len(B), min(len(B), 4000), replace=False)]
            def ulamek(th, t, Q, wolne=False):
                Y = (Q - c0) @ _R(th).T + c0 + t; k = ((Y - o) / kom).astype(int)
                ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < sh[0]) & (k[:, 1] < sh[1])
                d = np.full(len(Y), 0.25); d[ok] = DT[k[ok, 0], k[ok, 1]]
                hit = float(np.mean(d < 0.06))
                if not wolne: return hit
                w_ = np.zeros(len(Y), bool); w_[ok] = WOL[k[ok, 0], k[ok, 1]]
                return hit - 2.0 * float(np.mean(w_))
            kand = []
            for a, Qk in [(a, Q_) for a in np.arange(-katy, katy + 1e-9, 1.0) for Q_ in (Bs, Bc)]:
                th = np.radians(a); Y = (Qk - c0) @ _R(th).T + c0
                m0 = Y.min(0); kq = ((Y - m0) / kom).astype(int); shq = kq.max(0) + 1
                Qr = np.zeros(shq); np.add.at(Qr, (kq[:, 0], kq[:, 1]), 1); Qr = np.minimum(Qr, 2)
                K = fftconvolve(Ar, Qr[::-1, ::-1], mode='full')
                # przesuniecie t dla komorki (i,j): o + (i - shq + 1)*kom - m0
                tx = (o[0] + (np.arange(K.shape[0]) - shq[0] + 1) * kom - m0[0])[:, None]
                ty = (o[1] + (np.arange(K.shape[1]) - shq[1] + 1) * kom - m0[1])[None, :]
                # tylko okno +-zasieg (oszczednosc pamieci)
                wi = np.where(np.abs(tx[:, 0]) <= zas)[0]; wj = np.where(np.abs(ty[0]) <= zas)[0]
                if len(wi) == 0 or len(wj) == 0: continue
                K = K[wi[0]:wi[-1] + 1, wj[0]:wj[-1] + 1].copy(); tx = tx[wi[0]:wi[-1] + 1]; ty = ty[:, wj[0]:wj[-1] + 1]
                K[np.hypot(tx, ty) > zas] = -1
                for _ in range(4):                                  # kilka maksimow na obrot
                    i, j = np.unravel_index(np.argmax(K), K.shape)
                    if K[i, j] <= 0: break
                    t = np.array([tx[i, 0], ty[0, j]])
                    kand.append((ulamek(th, t, Bc, True), th, t))
                    K[np.hypot(tx - t[0], ty - t[1]) < 0.4] = -1
            kand.sort(key=lambda k: -k[0])
            s0 = ulamek(0.0, np.zeros(2), Bc, True)
            best = kand[0]
            drugi = max([k[0] for k in kand[1:] if np.hypot(*(k[2] - best[2])) > 0.4] + [0.0])
            th, t = best[1], best[2].copy()
            tree = cKDTree(A)
            for it in range(30):
                Y = (B - c0) @ _R(th).T + c0 + t
                d, j = tree.query(Y, distance_upper_bound=0.15 if it < 10 else 0.08)
                m = np.isfinite(d)
                if m.sum() < 50: break
                n = An[j[m]]; r = np.sum((Y[m] - A[j[m]]) * n, 1); wv = 1 / np.maximum(np.abs(r), 0.01)
                Yc = Y[m] - c0 - t
                Jm = np.stack([n[:, 0], n[:, 1], n[:, 1] * Yc[:, 0] - n[:, 0] * Yc[:, 1]], 1)
                H = (Jm * wv[:, None]).T @ Jm; g = (Jm * wv[:, None]).T @ r
                dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
                t = t + dx[:2]; th += dx[2]
                if np.abs(dx).max() < 1e-5: break
            s1 = ulamek(th, t, Bc, True)
            import slam as _sl
            ok_ = (s1 > s0 + 0.10) and (s1 > 0.15) and (best[0] - drugi > _sl.PAR.get('zszyj_margines', 0.15))
            raport.append((int(seg), f, f'luka {luki.get(f, 0):.1f} s, zasieg {zas:.1f} m: {s0*100:.0f}% -> {s1*100:.0f}% (drugie {drugi*100:.0f}%), '
                           f'popr {100*t[0]:+.0f},{100*t[1]:+.0f} cm {np.degrees(th):+.1f} st. ' + ('PRZYJETA' if ok_ else 'odrzucona')))
            if not ok_: continue
            dal_idx = np.where((frag[:, 0] == seg) & (frag[:, 1] >= f))[0]
            X[dal_idx, :2] = (X[dal_idx, :2] - c0) @ _R(th).T + c0 + t
            X[dal_idx, 2] += th
    for r in raport: log(f'  zszywanie kond.{r[0]} fragm.{r[1]}: {r[2]}')
    return X

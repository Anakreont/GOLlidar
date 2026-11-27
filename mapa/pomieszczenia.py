"""Podzial kondygnacji na pomieszczenia z mapy (mapa.json): obrys, powierzchnia, kubatura, drzwi, ocena wiarygodnosci.

Metoda (bez znajomosci rzutu CAD):
  1. siatka 4 cm obrocona do kierunku scian; przeszkody = sciany z mapy + chmura punktow,
  2. wolna przestrzen = to, co widac z trasy operatora (promienie do pierwszej przeszkody, zasieg 8 m),
     ograniczona do obrysu budynku (domkniecie przeszkod) - promienie przez okna nie wychodza na zewnatrz,
  3. watershed na mapie odleglosci od scian -> drobne obszary, potem laczenie sasiadow, gdy przejscie
     miedzy nimi jest szerokie (> 1,6 m albo prawie tak szerokie jak sam obszar - np. kolejne odcinki korytarza);
     waskie przejscia (0,6-1,6 m) zostaja jako drzwi,
  4. dla kazdego pomieszczenia: obrys (wielokat), powierzchnia, wysokosc kondygnacji, kubatura, typ
     (pomieszczenie / korytarz / male), nazwa ze znacznika "sala" z telefonu oraz ocena zamkniecia obrysu:
     jaka czesc granicy to zmierzona sciana, a jaka nieznane (niewidziane, szyba, brak punktow).

Wynik: <sesja>/_wynik/pomieszczenia.json (+ warstwy POMIESZCZENIA/DRZWI w rzut.dxf przez przetworz.dxf_i_podglad,
       pomieszczenia.png, pomieszczenia.csv).
Uzycie: python3 pomieszczenia.py <katalog_sesji>   albo   python3 pomieszczenia.py --mapa mapa.json --wy katalog
"""
import csv, json, os, sys, time
import numpy as np
from scipy import ndimage
from skimage.segmentation import watershed
from skimage.feature import peak_local_max
from skimage.measure import find_contours, approximate_polygon
from skimage.morphology import remove_small_objects

WERSJA = 1
KOM = 0.04            # komorka siatki [m]
ZASIEG = 8.0          # zasieg promieni widocznosci [m]
KROK_TRASY = 0.25     # co ile metrow trasy liczyc widocznosc
OBRYS_R = (0.8, 1.1, 1.5, 2.0, 2.5)   # kolejno probowane promienie domkniecia obrysu budynku [m]
DRZWI_MIN, DRZWI_MAX = 0.55, 1.6   # szerokosc przejscia uznawanego za drzwi [m]
OSCIEZE = 0.35        # tyle sciany w linii przejscia (z kazdej strony) oznacza drzwi/otwor [m]
OTWOR_MAX = 2.2       # najszerszy otwor z oscieznicami po obu stronach, ktory dzieli pomieszczenia [m]
MEBEL_MAX = 1.5       # najwieksze wolnostojace skupisko traktowane jako mebel [m]
LICO = 0.08          # o ile obrys wolnej przestrzeni jest przed licem sciany (szum punktow + poszerzenie) [m]
BEZ_ZNACZNIKA_MAX = 3.0   # obszar bez znacznika 'sala' (gdy sa znaczniki) mniejszy niz to - dolaczany do sasiada [m2]
SCIANA_ZEWN = 0.38    # grubosc scian zewnetrznych na rysunku (lica zewnetrznego nie widac ze srodka) [m]
TOL_RYS = 0.06        # dokladnosc upraszczania linii rysunku [m]
SCIANA_WEWN_MAX = 0.6 # najgrubsza sciana miedzy pomieszczeniami wypelniana na rysunku [m]
MIN_POW = 0.8         # mniejsze obszary dolaczane do sasiada (nie przez drzwi) [m2]
WYS_DOMYSLNA = 2.70


# ---------------------------------------------------------------- geometria pomocnicza
def kierunek_scian(sciany):
    """Dominujacy kierunek scian (modulo 90 st.) z odcinkow polilinii, waga = dlugosc."""
    a, w = [], []
    for pl in sciany:
        p = np.asarray(pl, float)
        if len(p) < 2:
            continue
        d = np.diff(p, axis=0)
        a.append(np.arctan2(d[:, 1], d[:, 0]) % (np.pi / 2)); w.append(np.hypot(d[:, 0], d[:, 1]))
    if not a:
        return 0.0
    a, w = np.concatenate(a), np.concatenate(w)
    # srednia kolowa na okregu o okresie 90 st.
    z = np.sum(w * np.exp(4j * a))
    return float(np.angle(z) / 4)


def _rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def rysuj_odcinki(G, P):
    """Linie miedzy kolejnymi punktami P (w komorkach) na siatce G."""
    for (x1, y1), (x2, y2) in zip(P[:-1], P[1:]):
        n = int(max(abs(x2 - x1), abs(y2 - y1)) * 2) + 2
        xs = np.round(np.linspace(x1, x2, n)).astype(int); ys = np.round(np.linspace(y1, y2, n)).astype(int)
        ok = (xs >= 0) & (ys >= 0) & (xs < G.shape[0]) & (ys < G.shape[1])
        G[xs[ok], ys[ok]] = True


def widocznosc(prz, punkty, zasieg_k, n_kat=720):
    """Komorki widoczne z punktow trasy (promien konczy sie na pierwszej przeszkodzie)."""
    H, W = prz.shape
    kat = np.linspace(0, 2 * np.pi, n_kat, endpoint=False)
    r = np.arange(0.5, zasieg_k, 0.7)
    dx = np.cos(kat)[:, None] * r; dy = np.sin(kat)[:, None] * r
    wolne = np.zeros_like(prz)
    ir = np.arange(len(r))[None, :]
    for px, py in punkty:
        X = np.round(px + dx).astype(int); Y = np.round(py + dy).astype(int)
        poza = (X < 0) | (Y < 0) | (X >= H) | (Y >= W)
        Xc = np.clip(X, 0, H - 1); Yc = np.clip(Y, 0, W - 1)
        traf = prz[Xc, Yc] | poza
        pierwszy = np.where(traf.any(1), traf.argmax(1), len(r))
        m = ir < pierwszy[:, None]
        wolne[Xc[m], Yc[m]] = True
    return wolne


def _prostuj(p, tol=0.2):
    """Krawedzie prawie rownolegle do osi siatki (uklad scian) -> dokladnie rownolegle."""
    p = np.array(p, float)
    for i in range(len(p) - 1):
        d = p[i + 1] - p[i]
        if abs(d[0]) < tol * abs(d[1]):
            p[i + 1, 0] = p[i, 0]
        elif abs(d[1]) < tol * abs(d[0]):
            p[i + 1, 1] = p[i, 1]
    p[-1] = p[0]
    q = [p[0]]
    for i in range(1, len(p) - 1):                         # usun wierzcholki w srodku prostego odcinka
        a, b, c = q[-1], p[i], p[i + 1]
        if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) > 1e-6 and np.hypot(*(b - a)) > 0:
            q.append(b)
    q.append(p[-1])
    return np.array(q)


def granice(L):
    """Slownik (a,b) -> liczba sasiadujacych par komorek miedzy obszarami a<b."""
    wyn = {}
    for A, B in ((L[1:, :], L[:-1, :]), (L[:, 1:], L[:, :-1])):
        m = (A > 0) & (B > 0) & (A != B)
        if not m.any():
            continue
        a = np.minimum(A[m], B[m]); b = np.maximum(A[m], B[m])
        para, ile = np.unique(np.stack([a, b], 1), axis=0, return_counts=True)
        for (x, y), n in zip(para, ile):
            wyn[(int(x), int(y))] = wyn.get((int(x), int(y)), 0) + int(n)
    return wyn


def polacz(L, a, b):
    L[L == b] = a


def przejscie(L, a, b, prz, wolne=None):
    """Przejscie miedzy obszarami a i b: szerokosc [m] i liczba oscieznic (0-2) - przeszkoda biegnaca dalej w linii
    przejscia po danej stronie. Drzwi/otwor maja sciane w tej samej linii po obu stronach; kolejne odcinki korytarza
    albo zwezenie przy meblu - sciany w poprzek albo nic. 'wneka' = komorki otworu w grubosci sciany
    (miedzy licami), ktore nie wchodza do powierzchni pomieszczen."""
    A, B = L == a, L == b
    pts = np.argwhere((A & ndimage.binary_dilation(B)) | (B & ndimage.binary_dilation(A)))
    if len(pts) == 0:
        return {'szer': 0.0, 'osc': 0}
    ex = pts.max(0) - pts.min(0)
    o = 0 if ex[0] >= ex[1] else 1                       # przejscie biegnie wzdluz osi o
    lo, hi = int(pts[:, o].min()), int(pts[:, o].max())
    pop = int(np.median(pts[:, 1 - o]))
    t, d, sz = int(round(0.12 / KOM)), int(round(OSCIEZE / KOM)), int(round(0.5 / KOM))
    def pas(i0, i1, c0, c1):
        sl = [None, None]
        sl[o] = slice(max(i0, 0), max(i1, 0)); sl[1 - o] = slice(max(c0, 0), max(c1, 0))
        P = prz[tuple(sl)]
        return P if o == 0 else P.T                        # [pozycja wzdluz, poprzek]
    def pelne(i0, i1):
        P = pas(i0, i1, pop - t, pop + t + 1)
        return P.size > 0 and P.any(axis=1).mean() >= 0.75
    strony = [(lo - d, lo), (hi + 1, hi + 1 + d)]
    jest = [pelne(*s_) for s_ in strony]
    wyn = {'szer': (hi - lo + 1) * KOM, 'osc': int(sum(jest)), 'o': o, 'lo': lo, 'hi': hi, 'pop': pop}
    # koniec przejscia oparty o przeszkode (np. drzwi przy narozniku - krotkie oscieze i sciana prostopadla)
    dotyk = [pas(lo - 3, lo, pop - t, pop + t + 1).any(), pas(hi + 1, hi + 4, pop - t, pop + t + 1).any()]
    wyn['dotyk'] = int(sum(dotyk))
    # grubosc sciany przy oscieznicach: wiersze poprzeczne z przeszkoda na >= polowie dlugosci pasa
    prof = [pas(i0, i1, pop - sz, pop + sz + 1).mean(axis=0) for (i0, i1), j in zip(strony, jest) if j]
    if prof:
        p_ = np.mean([p[:min(map(len, prof))] for p in prof], axis=0) >= 0.5
        c = pop - max(pop - sz, 0)                          # indeks linii przejscia w profilu
        if p_.any():
            idx = np.where(p_)[0]
            blisko = idx[np.abs(idx - c) <= t + 2]
            if len(blisko):
                lab_, _ = ndimage.label(p_)
                k_ = lab_[blisko[np.argmin(np.abs(blisko - c))]]
                rz = np.where(lab_ == k_)[0]
                wyn['c0'], wyn['c1'] = int(rz.min() + max(pop - sz, 0)), int(rz.max() + max(pop - sz, 0))
    # czy za przejsciem przestrzen sie rozszerza po obu stronach (drzwi) - w przekroju korytarza nie
    if wolne is not None:
        sr = (lo + hi) // 2; gap = hi - lo + 1
        gr_ = (wyn['c1'] - wyn['c0'] + 1) * KOM / 2 if 'c1' in wyn else 0.15
        off = int(round((gr_ + 0.25) / KOM))
        rozs = []
        for zn in (-1, 1):
            r_ = pop + zn * off
            if not (0 <= r_ < (prz.shape[1 - o])):
                rozs.append(False); continue
            linia = wolne[:, r_] if o == 0 else wolne[r_, :]
            if not linia[min(max(sr, 0), len(linia) - 1)]:
                rozs.append(False); continue
            lab_, _ = ndimage.label(linia)
            rozs.append((lab_ == lab_[sr]).sum() >= gap + int(round(0.08 / KOM)))   # drzwi wezsze od przestrzeni po obu stronach
        wyn['rozszerza'] = all(rozs)
    return wyn


# ---------------------------------------------------------------- glowne liczenie
def podziel(k, log=print):
    """k = kondygnacja z mapa.json. Zwraca slownik z pomieszczeniami i drzwiami (wspolrzedne mapy, metry)."""
    t0 = time.time()
    ch = np.array(k['chmura_cm'], float).reshape(-1, 2) / 100
    sc = [np.asarray(pl, float) for pl in k['sciany'] if len(pl) >= 2]
    tr = np.asarray(k['trasa'], float).reshape(-1, 2)
    if len(tr) < 2 or (len(ch) < 50 and not sc):
        return {'pomieszczenia': [], 'drzwi': [], 'uwagi': ['za malo danych (trasa/chmura)']}
    kat = kierunek_scian(sc)
    R = _rot(-kat)                                   # do ukladu, w ktorym sciany sa rownolegle do osi
    wsz = np.vstack([ch] + sc + [tr])
    srodek = wsz.mean(0)
    f = lambda p: (np.asarray(p, float) - srodek) @ R.T
    chr_, scr, trr = f(ch), [f(p) for p in sc], f(tr)
    wr = np.vstack([chr_] + scr + [trr])
    m0 = wr.min(0) - 1.0
    shape = tuple(np.ceil((wr.max(0) + 1.0 - m0) / KOM).astype(int))
    dok = lambda p: (np.asarray(p) - m0) / KOM       # metry (uklad obrocony) -> komorki

    # przeszkody
    prz = np.zeros(shape, bool)
    ij = np.round(dok(chr_)).astype(int)
    prz[ij[:, 0], ij[:, 1]] = True
    for p in scr:
        rysuj_odcinki(prz, dok(p))
    prz_g = ndimage.binary_dilation(prz, iterations=1)   # zatyka szczeliny 1 komorki miedzy punktami
    # wolnostojace przedmioty (stoly, szafki, slupy, ludzie): zwarte skupiska < MEBEL_MAX nieprzylegajace do scian.
    # Nie zaslaniaja widocznosci i nie dziela pomieszczen - podloga pod nimi wchodzi do powierzchni.
    lp, npz = ndimage.label(prz_g, np.ones((3, 3)))
    meble = np.zeros(shape, bool)
    for i, sl in enumerate(ndimage.find_objects(lp), 1):
        wym_ = np.array([sl[0].stop - sl[0].start, sl[1].stop - sl[1].start]) * KOM
        if wym_.max() < MEBEL_MAX and wym_.min() > 0.35:
            meble[sl] |= lp[sl] == i
    prz_g &= ~meble

    # obrys budynku: tylko geste przeszkody (sciany z mapy + punkty majace >= 4 sasiadow w promieniu ~12 cm),
    # domkniecie kolem OBRYS_R (zamyka okna i drzwi zewnetrzne do 2 m) i wypelnienie dziur
    gesty = ndimage.uniform_filter(prz.astype(float), 7) * 49 >= 5
    sc_g = np.zeros(shape, bool)
    for p in scr:
        rysuj_odcinki(sc_g, dok(p))
    baza = ndimage.binary_dilation(sc_g | (prz & gesty), iterations=1)
    lb, nb = ndimage.label(baza, np.ones((3, 3)))
    if nb:                                         # odrzuc drobne skupiska (smieci, odbicia) przed domknieciem
        rb = ndimage.sum(np.ones(shape), lb, range(1, nb + 1))
        baza = np.isin(lb, 1 + np.where(rb * KOM >= 0.6)[0])
    # promien domkniecia: najmniejszy, przy ktorym trasa operatora (>= 90 %) lezy wewnatrz obrysu
    pk_tr = np.round(dok(trr)).astype(int)
    budynek, r_uzyty = None, None
    for r_m in OBRYS_R:
        r_k = int(r_m / KOM)
        yy, xx = np.mgrid[-r_k:r_k + 1, -r_k:r_k + 1]; kolo = xx ** 2 + yy ** 2 <= r_k ** 2
        pad = r_k + 2
        P = ndimage.binary_closing(np.pad(baza, pad), kolo)[pad:-pad, pad:-pad]
        B = ndimage.binary_fill_holes(P)
        if B[pk_tr[:, 0], pk_tr[:, 1]].mean() >= 0.9:
            budynek, r_uzyty = B, r_m
            break
    if budynek is None:                            # budynek bardzo otwarty - otoczka wypukla gestych przeszkod
        from skimage.morphology import convex_hull_image
        budynek, r_uzyty = convex_hull_image(baza), None

    # wolna przestrzen widziana z trasy
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(trr, axis=0).T))]
    wyb = np.searchsorted(d, np.arange(0, d[-1] + 1e-6, KROK_TRASY))
    pkt = dok(trr[np.clip(wyb, 0, len(trr) - 1)])
    wolne = widocznosc(prz_g, pkt, ZASIEG / KOM)
    wolne = ndimage.binary_closing(wolne, iterations=2) & ~prz_g & budynek
    wolne = ndimage.binary_opening(wolne, iterations=1)
    lab, n = ndimage.label(wolne)
    if n == 0:
        return {'pomieszczenia': [], 'drzwi': [], 'uwagi': ['brak wolnej przestrzeni']}
    roz = ndimage.sum(np.ones(shape), lab, range(1, n + 1))
    wolne = np.isin(lab, 1 + np.where(roz * KOM ** 2 >= 0.5)[0])

    # meble i slupy otoczone wolna przestrzenia nie dziela pomieszczenia: liczymy na wolnej przestrzeni z wypelnionymi dziurami
    wolne = ndimage.binary_fill_holes(wolne)
    # watershed na odleglosci od przeszkod
    dist = ndimage.distance_transform_edt(wolne)
    dg = ndimage.gaussian_filter(dist, 2)
    piki = peak_local_max(dg, min_distance=int(0.6 / KOM), threshold_abs=0.3 / KOM, labels=ndimage.label(wolne)[0])
    mk = np.zeros(shape, int)
    mk[tuple(piki.T)] = np.arange(1, len(piki) + 1)
    L = watershed(-dg, mk, mask=wolne)
    # skladowe bez piku (waskie zakamarki) - jako osobne obszary, dalej dolacza je laczenie malych
    reszta, nr_ = ndimage.label(wolne & (L == 0))
    if nr_:
        L[reszta > 0] = reszta[reszta > 0] + L.max()

    # laczenie obszarow: dzielimy tylko w drzwiach/otworach (sciana w linii przejscia po obu stronach,
    # albo po jednej stronie przy waskim przejsciu); reszta granic watershedu to kawalki tej samej przestrzeni
    OPIS = {}
    def drzwi_(a, b):
        if (a, b) not in OPIS:
            OPIS[(a, b)] = przejscie(L, a, b, prz_g, wolne)
        p = OPIS[(a, b)]
        if not (DRZWI_MIN - 2 * LICO <= p['szer'] <= OTWOR_MAX):
            return False
        return bool(p.get('rozszerza')) and (p['osc'] == 2 or (p['osc'] == 1 and p['dotyk'] == 2 and p['szer'] <= 1.2))
    while True:
        do = [(nk, a, b) for (a, b), nk in granice(L).items() if not drzwi_(a, b)]
        if not do:
            break
        _, a, b = max(do)
        polacz(L, a, b)
        OPIS = {k_: v for k_, v in OPIS.items() if a not in k_ and b not in k_}
    # male obszary (wneki, resztki) do sasiada, ale nie przez drzwi; < 0,4 m2 bez sasiada - usuwane
    while True:
        ids = np.unique(L[L > 0])
        pow_ = dict(zip(ids, np.asarray(ndimage.sum(np.ones(shape), L, ids)) * KOM ** 2))
        gr = granice(L); zmiana = False
        for p, i in sorted((p, i) for i, p in pow_.items() if p < MIN_POW):
            sas = [(nk, b if a == i else a) for (a, b), nk in gr.items() if i in (a, b) and (p < 0.4 or not drzwi_(a, b))]
            if sas:
                polacz(L, max(sas)[1], i); zmiana = True; break
            if p < 0.4:
                L[L == i] = 0; zmiana = True; break
        if not zmiana:
            break
        OPIS.clear()

    # znaczniki "Jestem tu": maly obszar bez znacznika (kabina WC, wneka, resztka) dolaczany do sasiada ze znacznikiem
    sala_ = [r for r in k.get('znaczniki', []) if r.get('rodzaj') == 'sala']
    if len(sala_) >= 3:
        def lab_zn():
            z = set()
            for r in sala_:
                qi = np.round(((np.array([r['x'], r['y']]) - srodek) @ R.T - m0) / KOM).astype(int)
                if 0 <= qi[0] < shape[0] and 0 <= qi[1] < shape[1] and L[qi[0], qi[1]] > 0:
                    z.add(int(L[qi[0], qi[1]]))
            return z
        while True:
            zn = lab_zn(); gr = granice(L); zm = False
            ids = np.unique(L[L > 0])
            pw_ = dict(zip(ids, np.asarray(ndimage.sum(np.ones(shape), L, ids)) * KOM ** 2))
            for i in sorted(ids, key=lambda i: pw_[i]):
                if i in zn or pw_[i] > BEZ_ZNACZNIKA_MAX:
                    continue
                sas = [(nk, b if a == i else a) for (a, b), nk in gr.items() if i in (a, b) and (b if a == i else a) in zn]
                if sas:
                    polacz(L, max(sas)[1], i); zm = True; break
            if not zm:
                break

    # drzwi = pozostale przejscia; wneka drzwiowa (otwor w grubosci sciany) wycinana z obu pomieszczen
    drzwi, WNEKI = [], np.zeros(shape, bool)
    for (a, b), nk in granice(L).items():
        p = przejscie(L, a, b, prz_g, wolne)
        mA = ndimage.binary_dilation(L == a) & (L == b)
        if not mA.any():
            continue
        c = np.argwhere(mA).mean(0) * KOM + m0
        jest_dr = bool(p.get('rozszerza')) and (p['osc'] == 2 or (p['osc'] == 1 and p.get('dotyk') == 2))
        szer = p['szer'] + 2 * LICO if jest_dr else p['szer']           # w swietle oscieznic
        dr = {'a': a, 'b': b, 'szer_m': round(float(szer), 2), 'xy': np.round(c @ R + srodek, 2).tolist(),
              'typ': ('drzwi' if szer <= 1.4 else 'otwor') if jest_dr else 'przejscie', 'oscieza': p['osc']}
        if jest_dr and 'c0' in p:
            sl = [None, None]
            sl[p['o']] = slice(p['lo'] - 1, p['hi'] + 2); sl[1 - p['o']] = slice(p['c0'], p['c1'] + 1)
            w_ = np.zeros(shape, bool); w_[tuple(sl)] = True; w_ &= (L > 0)
            WNEKI |= w_
            dr['gl_sciany_m'] = round((p['c1'] - p['c0'] + 1) * KOM, 2)
            a_ = np.zeros(2); b_ = np.zeros(2)
            a_[p['o']], b_[p['o']] = p['lo'], p['hi'] + 1; a_[1 - p['o']], b_[1 - p['o']] = p['c0'], p['c1'] + 1
            nar = np.array([[a_[0], a_[1]], [b_[0], a_[1]], [b_[0], b_[1]], [a_[0], b_[1]]]) * KOM + m0
            dr['naroza'] = np.round(nar @ R + srodek, 3).tolist()          # prostokat otworu w scianie
            dr['os'] = 'x' if p['o'] == 0 else 'y'
            dr['wneka_m2'] = round(float(w_.sum() * KOM ** 2), 2)
        drzwi.append(dr)
    L[WNEKI] = 0

    # pomieszczenia
    sala = [r for r in k.get('znaczniki', []) if r.get('rodzaj') == 'sala']
    wys = k.get('wysokosc_m') or WYS_DOMYSLNA
    wyn = []
    RM = np.zeros(shape, int)                              # pomieszczenia do lica (do rysunku scian)
    for i in np.unique(L[L > 0]):
        m = ndimage.binary_fill_holes(L == i)                # meble/slupy w srodku nie zmniejszaja powierzchni
        # zakamarki niewidziane (cien za meblem przy scianie), zamkniete przez to pomieszczenie i przeszkody
        H = ndimage.binary_fill_holes(m | (prz_g & ndimage.binary_dilation(m, iterations=int(0.6 / KOM))))
        m = m | (H & ~prz_g & ~(L > 0) & ~WNEKI)
        # do lica sciany: wolna przestrzen konczy sie ~LICO przed licem
        for _ in range(int(round(LICO / KOM))):
            m = m | (ndimage.binary_dilation(m) & prz_g & ~(L > 0) & ~WNEKI)
        pw = float(m.sum() * KOM ** 2)
        if pw < 0.5:
            continue
        RM[m & (RM == 0)] = i
        # granica: sciana / sasiednie pomieszczenie / nieznane
        pierscien = ndimage.binary_dilation(m) & ~m
        sciana = (pierscien & ndimage.binary_dilation(prz_g, iterations=1)).sum()
        sasiad = (pierscien & (((L > 0) & (L != i)) | WNEKI)).sum()
        nieznane = max(0, pierscien.sum() - sciana - sasiad)
        zamk = min(1.0, float(sciana / max(1, sciana + nieznane)))
        # obrys
        cs = find_contours(np.pad(m, 1).astype(float), 0.5)
        c = max(cs, key=len) - 1
        poly = _prostuj(approximate_polygon(c, tolerance=TOL_RYS / KOM))
        poly_m = (poly * KOM + m0) @ R + srodek
        # wymiary w ukladzie scian
        ij_ = np.argwhere(m) * KOM
        dl, sz = np.sort(ij_.max(0) - ij_.min(0) + KOM)[::-1]
        sr_ = float(2 * KOM * dist[L == i].max())
        typ = 'male' if pw < 2.5 else 'korytarz' if (dl / max(sz, 1e-6) > 2.8 and sr_ < 2.2) else 'pomieszczenie'
        cm = (np.argwhere(m).mean(0) * KOM + m0) @ R + srodek
        nazwy = []
        for r in sala:
            q = (np.array([r['x'], r['y']]) - srodek) @ R.T
            qi = np.round((q - m0) / KOM).astype(int)
            if 0 <= qi[0] < shape[0] and 0 <= qi[1] < shape[1] and m[qi[0], qi[1]]:
                nazwy.append(r.get('opis', '').strip())
        wyn.append({'id': int(i), 'nazwa': ' / '.join(n for n in nazwy if n), 'typ': typ,
                    'pow_m2': round(pw, 2), 'wys_m': wys, 'kub_m3': round(pw * wys, 1),
                    'dl_m': round(float(dl), 2), 'szer_m': round(float(sz), 2),
                    'zamkniecie': round(zamk, 2),
                    'ocena': 'pewne' if zamk >= 0.85 else 'do sprawdzenia' if zamk >= 0.6 else 'niepewne',
                    'srodek': np.round(cm, 2).tolist(), 'obrys': np.round(poly_m, 3).tolist()})
    # sciany do rysunku: miedzy licami sasiednich pomieszczen (szczeliny do SCIANA_WEWN_MAX) + sciany zewnetrzne
    # o grubosci SCIANA_ZEWN na zewnatrz obrysu (lica zewnetrznego skaner nie widzi); otwory drzwiowe zostaja puste
    sciany_arch = []
    try:
        INT = RM > 0
        pad = int(SCIANA_ZEWN / KOM) + 4
        def kolo(r_):                                         # kwadrat: w ukladzie scian narozniki zostaja proste
            r_ = max(1, int(round(r_ / KOM))); return np.ones((2 * r_ + 1, 2 * r_ + 1), bool)
        Ip = np.pad(INT, pad)
        Z = ndimage.binary_closing(Ip, kolo(SCIANA_WEWN_MAX / 2))
        wew = Z & ~Ip
        zew = ndimage.binary_dilation(Z, kolo(SCIANA_ZEWN)) & ~Z
        Wm = (wew | zew) & ~np.pad(WNEKI, pad)
        Wm = remove_small_objects(Wm, max_size=int(0.05 / KOM ** 2))
        for c in find_contours(np.pad(Wm, 1).astype(float), 0.5):
            if len(c) < 4:
                continue
            p = approximate_polygon(c - 1 - pad, tolerance=TOL_RYS / KOM)
            p = _prostuj(p)
            if len(p) < 4:
                continue
            sciany_arch.append(np.round((p * KOM + m0) @ R + srodek, 3).tolist())
    except Exception as e:
        log(f'sciany do rysunku: blad {e}')
    # numeracja: od lewej gory, wierszami co ~3 m
    wyn.sort(key=lambda p: (-round(p['srodek'][1] / 3), p['srodek'][0]))
    stary = {}
    for n_, p in enumerate(wyn, 1):
        stary[p['id']] = n_; p['nr'] = n_; del p['id']
    for dr in drzwi:
        dr['a'], dr['b'] = stary.get(dr['a']), stary.get(dr['b'])
    drzwi = [d_ for d_ in drzwi if d_['a'] and d_['b']]
    log(f"obrys budynku r={r_uzyty} m; pomieszczenia: {len(wyn)}, drzwi/przejscia: {len(drzwi)}, kierunek scian {np.degrees(kat):.1f} st., {time.time() - t0:.1f} s")
    return {'wersja': WERSJA, 'kierunek_st': round(float(np.degrees(kat)), 2), 'wys_zrodlo': k.get('wysokosc_zrodlo', ''), 'obrys_r_m': r_uzyty,
            'pomieszczenia': wyn, 'drzwi': drzwi, 'sciany_arch': sciany_arch, 'sciana_zewn_m': SCIANA_ZEWN}


# ---------------------------------------------------------------- rysunek architektoniczny (jak inwentaryzacja w ArCADii)
WARSTWY_ARCH = (('ARCH_SCIANY', 2), ('ARCH_SCIANY_KRESKOWANIE', 8), ('ARCH_STOLARKA_DRZWI', 6), ('ARCH_OPIS POMIESZCZEN', 1),
                ('ARCH_OPIS WYS POM', 4), ('Kontur wewnętrzny', 6))


def warstwy_arch(doc):
    for n, k in WARSTWY_ARCH:
        if n not in doc.layers:
            doc.layers.add(n, color=k)
    doc.layers.get('ARCH_SCIANY').dxf.lineweight = 50


def rysuj_arch(msp, wynik, dx=0.0, wys=None, kreskuj=True):
    """Sciany obiema licami z kreskowaniem, otwory drzwiowe ze skrzydlem i lukiem, kontury i opisy pomieszczen
    (nr | nazwa, powierzchnia, hsw). Wspolrzedne wyniku w metrach (uklad mapy); dx - przesuniecie kondygnacji na arkuszu."""
    przes = lambda P: [(x + dx, y) for x, y in P]
    for pl in wynik.get('sciany_arch', []):
        msp.add_lwpolyline(przes(pl), close=True, dxfattribs={'layer': 'ARCH_SCIANY'})
    if kreskuj and wynik.get('sciany_arch'):
        h = msp.add_hatch(dxfattribs={'layer': 'ARCH_SCIANY_KRESKOWANIE'})
        h.set_pattern_fill('ANSI31', scale=0.02)
        for pl in wynik['sciany_arch']:
            h.paths.add_polyline_path(przes(pl), is_closed=True)
    for d_ in wynik.get('drzwi', []):
        if not d_.get('naroza'):
            continue
        n = np.array(przes(d_['naroza']))
        zawias, oscieze = n[0], n[1]
        w_ = np.linalg.norm(oscieze - zawias)
        t = n[3] - n[0]; t = t / (np.linalg.norm(t) or 1)
        koniec = zawias - t * w_                              # skrzydlo otwarte na 90 st. od lica
        msp.add_line(tuple(zawias), tuple(koniec), dxfattribs={'layer': 'ARCH_STOLARKA_DRZWI'})
        a1 = np.degrees(np.arctan2(*(koniec - zawias)[::-1])); a2 = np.degrees(np.arctan2(*(oscieze - zawias)[::-1]))
        if (a2 - a1) % 360 > 180:
            a1, a2 = a2, a1
        msp.add_arc(tuple(zawias), w_, a1, a2, dxfattribs={'layer': 'ARCH_STOLARKA_DRZWI'})
        for a_, b_ in ((n[0], n[3]), (n[1], n[2])):                  # oscieza (linie lica w otworze)
            msp.add_line(tuple(a_), tuple(b_), dxfattribs={'layer': 'ARCH_STOLARKA_DRZWI'})
    for p in wynik.get('pomieszczenia', []):
        msp.add_lwpolyline(przes(p['obrys']), close=True, dxfattribs={'layer': 'Kontur wewnętrzny'})
        x, y = p['srodek'][0] + dx, p['srodek'][1]
        t1 = f"{p['nr']} | {p['nazwa']}" if p['nazwa'] else str(p['nr'])
        msp.add_text(t1, height=0.15, dxfattribs={'layer': 'ARCH_OPIS POMIESZCZEN'}).set_placement((x, y + 0.05), align=_TA.BOTTOM_CENTER)
        msp.add_text(f"{p['pow_m2']:.2f} m²".replace('.', ','), height=0.15,
                     dxfattribs={'layer': 'ARCH_OPIS POMIESZCZEN'}).set_placement((x, y - 0.05), align=_TA.TOP_CENTER)
        h_ = wys or p.get('wys_m')
        if h_:
            msp.add_text(f"hśw={h_:.2f}m".replace('.', ','), height=0.10,
                         dxfattribs={'layer': 'ARCH_OPIS WYS POM'}).set_placement((x, y - 0.32), align=_TA.TOP_CENTER)


from ezdxf.enums import TextEntityAlignment as _TA


# ---------------------------------------------------------------- zapis
def podglad(k, wynik, plik):
    import matplotlib
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 8))
    ch = np.array(k['chmura_cm']).reshape(-1, 2) / 100
    ax.scatter(ch[:, 0], ch[:, 1], s=0.2, c='0.75', lw=0)
    for pl in k['sciany']:
        p = np.asarray(pl); ax.plot(p[:, 0], p[:, 1], 'k-', lw=1)
    kol = {'pewne': '#2a9d8f', 'do sprawdzenia': '#e9c46a', 'niepewne': '#e76f51'}
    for p in wynik['pomieszczenia']:
        o = np.asarray(p['obrys'])
        ax.fill(o[:, 0], o[:, 1], color=kol[p['ocena']], alpha=0.35, lw=0)
        ax.plot(np.r_[o[:, 0], o[0, 0]], np.r_[o[:, 1], o[0, 1]], color=kol[p['ocena']], lw=1.2)
        ax.text(*p['srodek'], f"{p['nr']}{' ' + p['nazwa'] if p['nazwa'] else ''}\n{p['pow_m2']:.2f} m²".replace('.', ','),
                ha='center', va='center', fontsize=8)
    for d_ in wynik['drzwi']:
        ax.plot(*d_['xy'], 's', color='#264653' if d_['typ'] == 'drzwi' else '#8ab17d', ms=5)
    ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title('Pomieszczenia: zielone = obrys zamknięty ścianami, żółte = do sprawdzenia, czerwone = niepewne')
    fig.tight_layout(); fig.savefig(plik, dpi=110); plt.close(fig)


def zapisz(kat_wy, M, wyniki, log=print):
    with open(os.path.join(kat_wy, 'pomieszczenia.json'), 'w', encoding='utf-8') as f:
        json.dump({'wersja': WERSJA, 'sesja': M.get('sesja'), 'policzono': time.strftime('%Y-%m-%d %H:%M'),
                   'kondygnacje': wyniki}, f, ensure_ascii=False, separators=(',', ':'))
    with open(os.path.join(kat_wy, 'pomieszczenia.csv'), 'w', encoding='utf-8-sig', newline='') as f:
        w = csv.writer(f, delimiter=';')
        w.writerow(['kondygnacja', 'nr', 'nazwa', 'typ', 'powierzchnia m2', 'wysokosc m', 'kubatura m3', 'dlugosc m', 'szerokosc m',
                    'zamkniecie obrysu', 'ocena'])
        for kw in wyniki:
            for p in kw['pomieszczenia']:
                w.writerow([kw['nr'], p['nr'], p['nazwa'], p['typ'], f"{p['pow_m2']:.2f}".replace('.', ','), str(p['wys_m']).replace('.', ','),
                            f"{p['kub_m3']:.1f}".replace('.', ','), str(p['dl_m']).replace('.', ','), str(p['szer_m']).replace('.', ','),
                            f"{p['zamkniecie']:.0%}", p['ocena']])


def policz(kat, log=print):
    wy = os.path.join(kat, '_wynik')
    with open(os.path.join(wy, 'mapa.json'), encoding='utf-8') as f:
        M = json.load(f)
    wyniki = []
    for k in M['kondygnacje']:
        w = podziel(k, log); w['nr'] = k['nr']; wyniki.append(w)
        podglad(k, w, os.path.join(wy, f"pomieszczenia{'_' + str(k['nr']) if len(M['kondygnacje']) > 1 else ''}.png"))
    zapisz(wy, M, wyniki, log)
    return wyniki


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == '--mapa':
        M = json.load(open(a[1], encoding='utf-8')); wy = a[a.index('--wy') + 1] if '--wy' in a else '.'
        os.makedirs(wy, exist_ok=True); W = []
        for k in M['kondygnacje']:
            w = podziel(k); w['nr'] = k['nr']; W.append(w)
            podglad(k, w, os.path.join(wy, f"pomieszczenia_{k['nr']}.png"))
        zapisz(wy, M, W)
    else:
        policz(a[0].rstrip('/'))

"""Rzut kondygnacji z chmury 3D (Point-LIO / Unitree L2).

    python l2_rzut.py <chmura.pcd> <katalog_wyniku> [--siatka 0.02] [--trasa trasa.csv]

- z --trasa (l2_trasa.py): jedno nagranie przez kilka kondygnacji -> osobny rzut kazdej (poziomy, na ktorych
  chodzil operator), kondygnacje obok siebie na arkuszu; na najwyzszej kondygnacji skosy (poddasze): linie
  h=1,40 i 2,20 m, powierzchnia wg PN-ISO 9836 (100 % / 50 % / pomijana), kubatura -> skosy.json,
- podloga i sufit z histogramu wysokosci (najnizsze i najwyzsze duze skupisko),
- SCIANY: pas 15-45 cm pod sufitem (meble tam nie siegaja) -> czysty obrys,
- CIECIE_1M: pas 1,0-1,2 m nad podloga (okna jako przerwy w scianie, meble, drzwi),
- wysokosc w swietle = sufit - podloga,
- OKNA / DRZWI: otwory w scianach na calej wysokosci (funkcja otwory(); --bez-otworow wylacza).
Wynik: rzut.png, rzut.dxf (metry; warstwy SCIANY, SCIANY_PUNKTY, CIECIE_1M, OKNA, DRZWI, OTWORY_OPIS, OPIS, SKALA),
wysokosci.txt, otwory.json i otwory.csv (szerokosc, wysokosc, parapet, nadproze, wspolrzedne).
"""
import argparse, os, sys
import numpy as np
from scipy import ndimage

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TU, '..', 'mapa')); sys.path.insert(0, os.path.join(TU, '..'))


def czytaj_pcd(plik):
    """Minimalny czytnik PCD (ascii / binary), zwraca x, y, z."""
    with open(plik, 'rb') as f:
        nag = {}
        while True:
            l = f.readline().decode('ascii', 'replace').strip()
            if not l or l.startswith('#'): continue
            k, *v = l.split(); nag[k.upper()] = v
            if k.upper() == 'DATA': break
        pola = nag['FIELDS']; roz = list(map(int, nag['SIZE'])); typ = nag['TYPE']; ile = list(map(int, nag.get('COUNT', ['1'] * len(pola))))
        n = int(nag['POINTS'][0])
        if nag['DATA'][0] == 'ascii':
            A = np.loadtxt(f, dtype=float, ndmin=2)
            ix = [sum(ile[:pola.index(c)]) for c in 'xyz']
            return A[:, ix]
        if nag['DATA'][0] != 'binary':
            raise SystemExit('PCD binary_compressed - zapisz jako binary')
        mapa = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = np.dtype([(p if p != '_' else f'_{i}', mapa[(t, s)], (c,) if c > 1 else ()) for i, (p, t, s, c) in enumerate(zip(pola, typ, roz, ile))])
        A = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        return np.stack([A['x'], A['y'], A['z']], 1).astype(float)


def poziomy(z, krok=0.02):
    """Podloga = najnizsze duze skupisko wysokosci, sufit = najwyzsze."""
    h, e = np.histogram(z, bins=np.arange(np.percentile(z, 0.5) - 0.2, np.percentile(z, 99.5) + 0.2, krok))
    h = ndimage.uniform_filter1d(h.astype(float), 3)
    prog = 0.25 * h.max()
    piki = [i for i in range(1, len(h) - 1) if h[i] >= prog and h[i] >= h[i - 1] and h[i] >= h[i + 1]]
    if len(piki) < 2:
        return float(np.percentile(z, 2)), float(np.percentile(z, 98))
    return float(e[piki[0]] + krok / 2), float(e[piki[-1]] + krok / 2)


def raster(P, kom, min_pkt):
    o = P.min(0) - 1
    ij = ((P - o) / kom).astype(int)
    G = np.zeros(ij.max(0) + 2, int); np.add.at(G, (ij[:, 0], ij[:, 1]), 1)
    return G >= min_pkt, o


def segmenty(sciany, min_dl=0.5):
    """Odcinki scian (kolejne wierzcholki polilinii) dluzsze niz min_dl."""
    out = []
    for pl in sciany:
        for a, b in zip(pl[:-1], pl[1:]):
            if np.hypot(*(b - a)) >= min_dl:
                out.append((np.asarray(a, float), np.asarray(b, float)))
    return out


def otwory(P, sciany, podl, suf, kom=0.08, tol=0.07):
    """Okna i drzwi w scianach. Dla kazdego odcinka sciany (z pasa pod sufitem) siatka w plaszczyznie sciany:
    wzdluz x wysokosc (kom x kom). Komorka 'otwarta' = brak punktow w plaszczyznie (+-tol) i brak przeszkody tuz
    przed sciana od strony pomieszczenia (meble zaslaniajace sciane to nie otwor); punkty ZA sciana (inne
    pomieszczenie, ulica za szyba) potwierdzaja otwor. Spojne obszary otwarte -> prostokaty:
    drzwi = od podlogi, wys. >= 1,7 m; okno = parapet >= 0,3 m. Zwraca liste slownikow (metry, wzgl. podlogi)."""
    wyn = []
    H = suf - podl
    nz = int(np.ceil(H / kom))
    for a, b in segmenty(sciany):
        t = (b - a) / np.hypot(*(b - a)); n = np.array([-t[1], t[0]])
        s = (P[:, :2] - a) @ t; d = (P[:, :2] - a) @ n; z = P[:, 2] - podl
        L = float(np.hypot(*(b - a)))
        wok = (s > -0.05) & (s < L + 0.05) & (z > 0.02) & (z < H - 0.02) & (np.abs(d) < 3.0)
        s, d, z = s[wok], d[wok], z[wok]
        if len(s) < 50:
            continue
        pas = np.abs(d) < 0.3                                          # dokladna plaszczyzna: mediana d punktow bliskich
        if pas.sum() > 20:
            d = d - np.median(d[pas & (np.abs(d) < 0.15)]) if (pas & (np.abs(d) < 0.15)).any() else d
        strona = 1.0 if ((d > 0.15) & (d < 1.5)).sum() >= ((d < -0.15) & (d > -1.5)).sum() else -1.0
        d = d * strona                                                 # + = pomieszczenie (przed sciana), - = za sciana
        ns = max(1, int(np.ceil(L / kom)))
        i_s = np.clip((s / kom).astype(int), 0, ns - 1); i_z = np.clip((z / kom).astype(int), 0, nz - 1)
        def siatka(m):
            G = np.zeros((ns, nz), int); np.add.at(G, (i_s[m], i_z[m]), 1); return G
        sc = siatka(np.abs(d) < tol)
        przed = siatka((d >= tol) & (d < 0.8))
        za = siatka(d <= -tol)
        ok_sciana = sc > 0
        otw = ~ok_sciana & ((przed == 0) | (za > 0))
        otw = ndimage.binary_opening(otw, np.ones((3, 3)))
        lab, nl = ndimage.label(otw)
        for k in range(1, nl + 1):
            ii, jj = np.nonzero(lab == k)
            s0, s1 = ii.min() * kom, (ii.max() + 1) * kom
            z0, z1 = jj.min() * kom, (jj.max() + 1) * kom
            # dokladne krawedzie z punktow sciany (siatka kom zaokragla i zjada brzeg)
            w = np.abs(d) < tol
            sm, zm = (s0 + s1) / 2, (z0 + z1) / 2
            pz = w & (z > z0 + 0.1) & (z < z1 - 0.1)
            l_ = s[pz & (s < sm) & (s > s0 - 0.3)]; r_ = s[pz & (s > sm) & (s < s1 + 0.3)]
            if len(l_): s0 = float(np.percentile(l_, 98))
            if len(r_): s1 = float(np.percentile(r_, 2))
            ps = w & (s > s0 + 0.1) & (s < s1 - 0.1)
            g_ = z[ps & (z > zm) & (z < z1 + 0.3)]; d_ = z[ps & (z < zm) & (z > z0 - 0.3)]
            if len(g_): z1 = float(np.percentile(g_, 2))
            z0 = float(np.percentile(d_, 98)) if len(d_) else (0.0 if z0 < 0.15 else z0)
            szer, wys = s1 - s0, z1 - z0
            wypel = len(ii) / ((ii.max() - ii.min() + 1) * (jj.max() - jj.min() + 1))
            if szer < 0.5 or szer > 4.5 or wys < 0.5 or wypel < 0.6 or s0 < 0.05 or s1 > L - 0.05:
                continue
            if z1 > H - 0.25:                                          # siega pod sufit = nie otwor (brak sciany/korytarz)
                continue
            if z0 < 0.15 and wys >= 1.7:
                typ = 'drzwi'
            elif z0 >= 0.3:
                typ = 'okno'
            else:
                continue
            p0, p1 = a + t * s0, a + t * s1
            wyn.append({'typ': typ, 'szer': round(szer, 2), 'wys': round(wys, 2), 'parapet': round(z0, 2) if typ == 'okno' else 0.0,
                        'nadproze': round(z1, 2), 'za_sciana': int(za[lab == k].sum()),
                        'x0': round(float(p0[0]), 3), 'y0': round(float(p0[1]), 3), 'x1': round(float(p1[0]), 3), 'y1': round(float(p1[1]), 3),
                        'nx': round(float(n[0] * strona), 3), 'ny': round(float(n[1] * strona), 3)})
    return wyn


def czytaj_trase(plik):
    if not plik or not os.path.exists(plik):
        return None
    A = np.loadtxt(plik, delimiter=';', skiprows=1, ndmin=2)
    return A if len(A) >= 10 else None


def kondygnacje(P, trasa=None):
    """Kondygnacje [(podloga, sufit, zt)] - z poziomow trasy operatora (jedno nagranie przez kilka pieter) albo,
    bez trasy, jedna kondygnacja z histogramu wysokosci (jak dawniej). Sufit ostatniej kondygnacji moze byc
    skosny (dach) - wtedy 'sufit' to plaska czesc, jesli jest; skosy liczy funkcja skosy()."""
    z = P[:, 2]
    if trasa is None:
        podl, suf = poziomy(z)
        return [(podl, suf, None)]
    zt = trasa[:, 3]
    h, e = np.histogram(zt, bins=np.arange(zt.min() - 0.25, zt.max() + 0.35, 0.1))
    h = ndimage.uniform_filter1d(h.astype(float), 3)
    piki = [i for i in range(len(h)) if h[i] >= 0.05 * len(zt) and h[i] == h[max(0, i - 6):i + 7].max()]
    lv = []
    for i in sorted(piki, key=lambda i: -h[i]):                  # poziomy operatora co najmniej 1,5 m od siebie
        zc = e[i] + 0.05
        if all(abs(zc - q) > 1.5 for q in lv):
            lv.append(zc)
    lv.sort()
    wyn = []
    for j, zl in enumerate(lv):
        okno = z[(z > zl - 2.7) & (z < zl - 1.0)]                # podloga: 1,0-2,7 m pod glowica
        if len(okno) < 200:
            continue
        hh, ee = np.histogram(okno, bins=np.arange(okno.min(), okno.max() + 0.02, 0.02))
        hh = ndimage.uniform_filter1d(hh.astype(float), 3)
        pk = [i for i in range(1, len(hh) - 1) if hh[i] >= 0.4 * hh.max() and hh[i] >= hh[i - 1] and hh[i] >= hh[i + 1]]
        podl = float(ee[pk[-1] if pk else int(np.argmax(hh))] + 0.01)   # najwyzsza duza plaszczyzna pod stopami (nizej: sufit pietra ponizej)
        gora = (lv[j + 1] - 1.0) if j + 1 < len(lv) else z.max() + 0.01
        okno = z[(z > zl) & (z < gora)]
        suf = None
        if len(okno) > 200:
            hh, ee = np.histogram(okno, bins=np.arange(okno.min(), okno.max() + 0.02, 0.02))
            hh = ndimage.uniform_filter1d(hh.astype(float), 3)
            pk = [i for i in range(1, len(hh) - 1) if hh[i] >= 0.25 * hh.max() and hh[i] >= hh[i - 1] and hh[i] >= hh[i + 1]]
            if pk:
                suf = float(ee[pk[0] if j + 1 < len(lv) else pk[-1]] + 0.01)   # miedzy pietrami: spod stropu = najnizszy duzy pik
        if suf is None or suf - podl < 1.8:
            suf = float(np.percentile(okno, 99)) if len(okno) else podl + 2.5
        wyn.append((podl, suf, float(zl)))
    return wyn or [(*poziomy(z), None)]


def mapa_sufitu(P, podl, kom=0.15):
    """Wysokosc stropu/dachu nad kazda komorka: najwyzsza warstwa punktow (mediana punktow do 15 cm pod drugim najwyzszym
    komorki), wiec gorna plyta szafy pod widocznym sufitem nie jest brana za sufit. Luki - najblizsza znana komorka.
    Zwraca (siatka z [m n.p.m. ukladu], poczatek, kom, maska znanych)."""
    Q = P[P[:, 2] > podl + 1.0]
    o = P[:, :2].min(0) - 0.5
    sh = tuple(((P[:, :2].max(0) - o) / kom).astype(int) + 2)
    ij = ((Q[:, :2] - o) / kom).astype(int)
    idk = ij[:, 0] * sh[1] + ij[:, 1]; zz = Q[:, 2]
    k_ = np.lexsort((zz, idk)); idk, zz = idk[k_], zz[k_]
    u, st, n_ = np.unique(idk, return_index=True, return_counts=True)
    S = np.full(sh, np.nan); ile = np.zeros(sh, int)
    g = zz[st + np.maximum(n_ - 2, 0)]                                # drugi najwyzszy punkt (pojedynczy odstajacy nie wystarczy);
    # sufitu nad szafa jest w komorce malo (1-2 punkty na 5 cm) wobec setek z boku szafy - percentyl trafialby w szafe
    gg = np.repeat(g, n_)
    m = np.add.reduceat((zz > gg - 0.15).astype(int), st)           # ile punktow w gornej warstwie (na koncu - posortowane)
    S.flat[u] = zz[st + n_ - m + (m - 1) // 2]; ile.flat[u] = n_    # mediana gornej warstwy
    znane = ile >= 2
    _, (ii, jj) = ndimage.distance_transform_edt(~znane, return_indices=True)
    return np.where(znane, S, S[ii, jj]), o, kom, znane


def punkty_scian(P, podl, M, pas_z=None):
    """Punkty scian: pionowa powierzchnia od ~1 m nad podloga do samego sufitu/dachu nad nia (pas 5-25 cm pod lokalnym
    sufitem + ciaglosc w dol). Szafy, regaly i komody maja nad soba szczeline (widac sufit) - nie sa scianami; lampy i
    kanaly pod sufitem nie maja ciaglosci w dol."""
    S, o, kom, _ = M
    ij = np.clip(((P[:, :2] - o) / kom).astype(int), 0, np.array(S.shape) - 1)
    c = S[ij[:, 0], ij[:, 1]]
    pas = (P[:, 2] > c - 0.25) & (P[:, 2] < c - 0.05) & (P[:, 2] > podl + 0.3)
    gora = pas
    if pas_z is not None:                                       # te same sciany, ale inny pas wysokosci (np. 1,0-1,9 m:
        # drzwi jako otwory, nadproza pominiete); pod skosem pas schodzi nizej (scianka kolankowa), a sam dach odpada
        pas = (P[:, 2] > np.maximum(podl + 0.3, np.minimum(podl + pas_z[0], c - 0.8))) & (P[:, 2] < np.minimum(podl + pas_z[1], c - 0.05))
    dol = (P[:, 2] > np.maximum(podl + 0.3, c - 0.7)) & (P[:, 2] < c - 0.3)   # ta sama pionowa powierzchnia 30-70 cm pod
    # sufitem (scianka kolankowa tez); mebel ponizej ma tam pusta szczeline;
    # sam skos dachu w komorce nie ma punktow 30 cm pod soba - nie jest sciana
    G = np.zeros(S.shape, bool); G[ij[dol, 0], ij[dol, 1]] = True
    G = ndimage.binary_dilation(G)                                  # sciana moze byc lekko nachylona / szum
    if pas_z is not None:                                           # inny pas: komorka musi miec tez punkty tuz pod sufitem
        Gg = np.zeros(S.shape, bool); Gg[ij[gora, 0], ij[gora, 1]] = True   # (wysoka szafa ma ciaglosc, ale nie siega sufitu)
        G &= ndimage.binary_dilation(Gg)
    return P[pas & G[ij[:, 0], ij[:, 1]], :2]


def skosy(P, podl, suf, kom=0.10):
    """Mapa wysokosci pod stropem/dachem kondygnacji. Gdy czesc pomieszczen jest nizsza niz plaski sufit (poddasze),
    zwraca slownik: powierzchnie wg PN-ISO 9836 (>= 2,20 m w 100 %, 1,40-2,20 m w 50 %, < 1,40 m pomijana),
    kubature, kontury 1,40 i 2,20 m. Bez skosow - None."""
    Q = P[(P[:, 2] > podl - 0.1) & (P[:, 2] < suf + 0.6)]
    Sm, o, _, znane = mapa_sufitu(Q, podl, kom)
    sh = Sm.shape
    ij = ((Q[:, :2] - o) / kom).astype(int)
    pod = np.zeros(sh, bool); f = np.abs(Q[:, 2] - podl) < 0.06; pod[ij[f, 0], ij[f, 1]] = True
    wnetrze = ndimage.binary_fill_holes(ndimage.binary_closing(pod, iterations=3))
    if (wnetrze & znane).sum() < 50:
        return None
    Hf = np.where(wnetrze, Sm - podl, np.nan)
    plaski = suf - podl
    nisko = np.nanmean(Hf[wnetrze] < plaski - 0.3)
    if nisko < 0.08:
        return None
    a = kom * kom
    w = Hf[wnetrze]
    A100, A50, A0 = float((w >= 2.2).sum() * a), float(((w >= 1.4) & (w < 2.2)).sum() * a), float((w < 1.4).sum() * a)
    from skimage.measure import find_contours
    kont = {}
    for prog in (1.4, 2.2):
        M = np.where(wnetrze, Hf, 0.0)
        kont[prog] = [c * kom + o + kom / 2 for c in find_contours(np.pad(M, 1), prog) if len(c) > 8]
        kont[prog] = [c - kom for c in kont[prog]]
    return {'pow_100': round(A100, 2), 'pow_50': round(A50, 2), 'pow_pominieta': round(A0, 2),
            'pow_uzytkowa_pn_iso_9836': round(A100 + 0.5 * A50, 2), 'pow_rzutu': round(A100 + A50 + A0, 2),
            'kubatura_m3': round(float(np.nansum(w) * a), 1), 'wys_max_m': round(float(np.nanmax(w)), 2),
            'kontury': kont}


def rzut_kondygnacji(P, podl, suf, siatka, bez_otworow=False, sk=None):
    """Sciany, ciecie 1 m i otwory jednej kondygnacji (P - punkty tej kondygnacji)."""
    try:
        A = punkty_scian(P, podl, mapa_sufitu(P, podl))           # sciany dochodzace do sufitu/dachu (bez szaf, lamp)
    except Exception as e:
        print('sciany wg mapy sufitu pominiete:', e); A = np.zeros((0, 2))
    if len(A) < 100:
        A = P[(P[:, 2] > suf - 0.45) & (P[:, 2] < suf - 0.15), :2]      # zapas: pas pod plaskim sufitem
    B = P[(P[:, 2] > podl + 1.0) & (P[:, 2] < podl + 1.2), :2]       # ciecie 1 m
    if len(A) < 100:
        A = P[(P[:, 2] > podl + 1.6) & (P[:, 2] < podl + 2.0), :2]  # niski sufit / brak sufitu - pas 1,6-2,0 m
    G0, oa = raster(A, siatka, 1)
    ij = ((A - oa) / siatka).astype(int); H = np.zeros(G0.shape, int); np.add.at(H, (ij[:, 0], ij[:, 1]), 1)
    prog = max(1, int(np.percentile(H[H > 0], 30)))
    Ga = ndimage.binary_closing(H >= prog, np.ones((3, 3)))        # rzadkie punkty sciany -> ciagla linia
    from skimage.morphology import remove_small_objects
    Ga = remove_small_objects(Ga, max_size=12) & (H > 0)
    ia, ja = np.nonzero(Ga); Wp = np.stack([ia, ja], 1) * siatka + oa + siatka / 2
    sciany = []
    try:
        from eksport_dxf import szkielet
        sciany = szkielet(Wp, kom=0.03)
    except Exception as e:
        print('szkielet scian pominiety:', e)
    ot = [] if bez_otworow else otwory(P, sciany, podl, suf)
    return {'Wp': Wp, 'A': A, 'B': B, 'sciany': sciany, 'otwory': ot}


def sale_l2(plik_zn, trasa, zt):
    """Znaczniki "Jestem tu" (SALA <nazwa>) z rejestratora -> punkty na trasie tej kondygnacji (czas maliny w ns =
    czas trasy w s; trasa z l2_trasa.py). Zwraca liste {'rodzaj':'sala','opis','x','y'}."""
    if not plik_zn or not os.path.exists(plik_zn) or trasa is None:
        return []
    wyn = []
    for l in open(plik_zn, encoding='utf-8'):
        k = l.rstrip('\n').split('\t')
        if len(k) < 2 or not k[0].isdigit() or not k[1].startswith('SALA'):
            continue
        t = int(k[0]) / 1e9; nz = (k[1][4:].strip() or (k[2] if len(k) > 2 else '')).strip()
        if not (trasa[0, 0] - 2 <= t <= trasa[-1, 0] + 2):
            continue
        i = int(np.clip(np.searchsorted(trasa[:, 0], t), 0, len(trasa) - 1))
        if zt is not None and abs(trasa[i, 3] - zt) > 0.8:          # znacznik z innej kondygnacji
            continue
        wyn.append({'rodzaj': 'sala', 'opis': nz, 'x': float(trasa[i, 1]), 'y': float(trasa[i, 2])})
    return wyn


def pomieszczenia_kondygnacji(r, trasa, plik_zn=None, log=print):
    """Podzial kondygnacji L2 na pomieszczenia tym samym algorytmem co D500 (mapa/pomieszczenia.py): przeszkody = tylko
    sciany dochodzace do sufitu (bez szaf), trasa operatora = punkty widzenia. Na poddaszu powierzchnia kazdego
    pomieszczenia takze wg PN-ISO 9836 (z mapy wysokosci pod dachem) i kubatura z rzeczywistej wysokosci."""
    import pomieszczenia as PM
    zt = r.get('zt')
    if trasa is not None:
        tk = trasa if zt is None else trasa[np.abs(trasa[:, 3] - zt) < 0.8]
        tr = tk[:, 1:3]
    else:
        tr = np.zeros((0, 2))
    if len(tr) < 10:                                         # bez trasy: srodek kazdej wolnej przestrzeni nie jest znany
        log(f'kondygnacja {r["nr"]}: brak trasy - pomieszczenia pominiete'); return None
    # przeszkody do podzialu: sciany dochodzace do sufitu (bez szaf, lamp), ale ich punkty z pasa 1,0-1,9 m nad podloga -
    # wtedy drzwi sa otworami z osciezami (jak w D500), a nie zamkniete nadprozem
    try:
        A = punkty_scian(r['P'], r['podl'], mapa_sufitu(r['P'], r['podl']), pas_z=(1.0, 1.9))
    except Exception:
        A = r.get('A', r['Wp'])
    # rdzen sciany: komorki 2 cm z liczba punktow >= 35 % lokalnego maksimum (szum L2 2-3 cm rozmywa lico; ogon rozkladu
    # pogrubialby sciany i zmniejszal pomieszczenia)
    q = np.round(A / 0.02).astype(np.int64); q0 = q.min(0); sh = tuple((q.max(0) - q0 + 1).tolist())
    Hc = np.zeros(sh, np.int32); np.add.at(Hc, (q[:, 0] - q0[0], q[:, 1] - q0[1]), 1)
    rdzen = (Hc > 0) & (Hc >= 0.35 * ndimage.maximum_filter(Hc, 9))
    A = (np.argwhere(rdzen) + q0) * 0.02
    k = {'nr': r['nr'], 'chmura_cm': np.round(A * 100).astype(int).ravel().tolist(), 'lico_m': 0.04,
         'sciany': [np.asarray(pl).tolist() for pl in r['sciany']], 'trasa': tr[::3].tolist(),
         'wysokosc_m': round(float(r['wys']), 2), 'wysokosc_zrodlo': 'L2: podloga i sufit z chmury',
         'znaczniki': sale_l2(plik_zn, trasa, zt)}
    w = PM.podziel(k, log)
    w['nr'] = r['nr']
    # wysokosc kazdego pomieszczenia z mapy sufitu (mediana), skosy wg PN-ISO 9836
    try:
        from matplotlib.path import Path
        S_, o, kom, _ = mapa_sufitu(r['P'], r['podl'], 0.10)
        ii, jj = np.mgrid[0:S_.shape[0], 0:S_.shape[1]]
        C = np.c_[ii.ravel(), jj.ravel()] * kom + o + kom / 2
        H = (S_ - r['podl']).ravel()
        for p in w['pomieszczenia']:
            m = Path(np.asarray(p['obrys'])).contains_points(C)
            if m.sum() < 5:
                continue
            h = H[m]; a = kom * kom; skala = p['pow_m2'] / (m.sum() * a)     # raster -> dokladny obrys
            p['wys_m'] = round(float(np.median(h)), 2)
            p['kub_m3'] = round(float(np.sum(np.clip(h, 0, None)) * a * skala), 1)
            if np.mean(h < r['wys'] - 0.3) > 0.08:                          # pomieszczenie pod skosem
                A100 = float((h >= 2.2).sum() * a * skala); A50 = float(((h >= 1.4) & (h < 2.2)).sum() * a * skala)
                p['pow_pn_iso_9836'] = round(A100 + 0.5 * A50, 2)
                p['pow_ponizej_1_40'] = round(float((h < 1.4).sum() * a * skala), 2)
                p['wys_max_m'] = round(float(np.percentile(h, 98)), 2)
    except Exception as e:
        log(f'wysokosci pomieszczen pominiete: {e}')
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('wyjscie')
    ap.add_argument('--siatka', type=float, default=0.02)
    ap.add_argument('--trasa', help='trasa.csv z l2_trasa.py - podzial nagrania na kondygnacje')
    ap.add_argument('--bez-otworow', action='store_true', help='nie szukaj okien i drzwi')
    ap.add_argument('--znaczniki', help='znaczniki.txt z rejestratora (SALA <nazwa> -> nazwy pomieszczen)')
    ap.add_argument('--bez-pomieszczen', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.wyjscie, exist_ok=True)
    P = czytaj_pcd(a.pcd)
    P = P[np.isfinite(P).all(1)]
    print(f'chmura: {len(P)} punktow, zakres {np.ptp(P, 0).round(1).tolist()} m')
    TR = czytaj_trase(a.trasa)
    KD = kondygnacje(P, TR)
    wyniki = []
    for nr, (podl, suf, zt) in enumerate(KD):
        gora = KD[nr + 1][0] - 0.05 if nr + 1 < len(KD) else np.inf
        Pk = P[(P[:, 2] > podl - 0.15) & (P[:, 2] < gora)]
        sk = skosy(Pk, podl, suf) if nr == len(KD) - 1 else None    # skosy szukamy na najwyzszej kondygnacji
        print(f'kondygnacja {nr}: podloga {podl:+.2f} m, sufit {suf:+.2f} m, wysokosc w swietle {suf - podl:.2f} m'
              + (f", skosy: pow. wg PN-ISO 9836 {sk['pow_uzytkowa_pn_iso_9836']} m2 z {sk['pow_rzutu']} m2 rzutu" if sk else ''))
        r = rzut_kondygnacji(Pk, podl, suf, a.siatka, a.bez_otworow, sk)
        r.update(nr=nr, podl=podl, suf=suf, wys=suf - podl, sk=sk, P=Pk, zt=zt)
        r['pom'] = None if a.bez_pomieszczen else pomieszczenia_kondygnacji(r, TR, a.znaczniki)
        wyniki.append(r)
    import json
    ot_all = [dict(o, kondygnacja=r['nr']) for r in wyniki for o in r['otwory']]
    with open(os.path.join(a.wyjscie, 'otwory.json'), 'w', encoding='utf-8') as f:
        json.dump(ot_all, f, ensure_ascii=False, indent=1)
    with open(os.path.join(a.wyjscie, 'otwory.csv'), 'w', encoding='utf-8') as f:
        f.write('nr;kondygnacja;typ;szerokosc_m;wysokosc_m;parapet_m;nadproze_m;x0;y0;x1;y1\n')
        for i, o in enumerate(ot_all, 1):
            f.write(f"{i};{o['kondygnacja']};{o['typ']};{o['szer']};{o['wys']};{o['parapet']};{o['nadproze']};{o['x0']};{o['y0']};{o['x1']};{o['y1']}\n".replace('.', ','))
    print(f"otwory: {sum(o['typ'] == 'okno' for o in ot_all)} okien, {sum(o['typ'] == 'drzwi' for o in ot_all)} drzwi")
    with open(os.path.join(a.wyjscie, 'wysokosci.txt'), 'w', encoding='utf-8') as f:
        r0 = wyniki[0]
        f.write(f"podloga_m={r0['podl']:.3f}\nsufit_m={r0['suf']:.3f}\nwysokosc_w_swietle_m={r0['wys']:.3f}\npunktow={len(P)}\n"
                f"kondygnacji={len(wyniki)}\n")
        for r in wyniki:
            f.write(f"k{r['nr']}_podloga_m={r['podl']:.3f}\nk{r['nr']}_sufit_m={r['suf']:.3f}\nk{r['nr']}_wysokosc_w_swietle_m={r['wys']:.3f}\n")
            if r['sk']:
                for k_, v in r['sk'].items():
                    if k_ != 'kontury':
                        f.write(f"k{r['nr']}_skosy_{k_}={v}\n")
    if any(r['sk'] for r in wyniki):
        with open(os.path.join(a.wyjscie, 'skosy.json'), 'w', encoding='utf-8') as f:
            json.dump({r['nr']: {k_: v for k_, v in r['sk'].items() if k_ != 'kontury'} for r in wyniki if r['sk']}, f, indent=1)
    POMW = [r['pom'] for r in wyniki if r.get('pom')]
    if POMW:
        import pomieszczenia as PM
        PM.zapisz(a.wyjscie, {'sesja': os.path.basename(os.path.dirname(os.path.abspath(a.wyjscie)))}, POMW)
        with open(os.path.join(a.wyjscie, 'pomieszczenia.csv'), 'a', encoding='utf-8') as f:
            pod = [(w['nr'], p) for w in POMW for p in w['pomieszczenia'] if 'pow_pn_iso_9836' in p]
            if pod:
                f.write('\npomieszczenia pod skosami (PN-ISO 9836: >= 2,20 m w 100 %, 1,40-2,20 m w 50 %, < 1,40 m pomijane)\n'
                        'kondygnacja;nr;nazwa;powierzchnia rzutu m2;powierzchnia PN-ISO 9836 m2;ponizej 1,40 m m2;wys. max m;kubatura m3\n')
                for kn, p in pod:
                    f.write(f"{kn};{p['nr']};{p['nazwa']};{p['pow_m2']:.2f};{p['pow_pn_iso_9836']:.2f};{p['pow_ponizej_1_40']:.2f};"
                            f"{p['wys_max_m']:.2f};{p['kub_m3']:.1f}\n".replace('.', ','))
        print('pomieszczenia: ' + ', '.join(f"k{w['nr']}: {len(w['pomieszczenia'])} ({sum(p['pow_m2'] for p in w['pomieszczenia']):.1f} m2)" for w in POMW))
    # kondygnacje obok siebie na arkuszu (odstep 5 m), ta sama skala
    dxs, x = [], 0.0
    for r in wyniki:
        lo, hi = r['P'][:, :2].min(0), r['P'][:, :2].max(0)
        dxs.append(x - lo[0]); x += (hi[0] - lo[0]) + 5.0
    # obraz
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    ext = np.array([x, max(np.ptp(r['P'][:, 1]) for r in wyniki)])
    fig, ax = plt.subplots(figsize=(min(30, 4 + ext[0] / 3), min(24, 3 + ext[1] / 3)))
    for r, dx in zip(wyniki, dxs):
        ax.scatter(r['B'][::2, 0] + dx, r['B'][::2, 1], s=0.3, c='tab:red', lw=0, label='cięcie 1,0–1,2 m (okna, meble)' if r['nr'] == 0 else None)
        ax.scatter(r['Wp'][:, 0] + dx, r['Wp'][:, 1], s=1.5, c='k', lw=0, label='ściany' if r['nr'] == 0 else None)
        for pl in r['sciany']: ax.plot(pl[:, 0] + dx, pl[:, 1], '-', color='tab:blue', lw=0.8)
        for i, o in enumerate(r['otwory'], 1):
            kol = 'tab:cyan' if o['typ'] == 'okno' else 'tab:green'
            ax.plot([o['x0'] + dx, o['x1'] + dx], [o['y0'], o['y1']], '-', color=kol, lw=4, solid_capstyle='butt')
        if r['sk']:
            for prog, kol in ((1.4, 'tab:orange'), (2.2, 'tab:purple')):
                for c in r['sk']['kontury'][prog]:
                    ax.plot(c[:, 0] + dx, c[:, 1], '--', color=kol, lw=1, label=f'skos h={prog:.2f} m'.replace('.', ',') if c is r['sk']['kontury'][prog][0] else None)
        if r.get('pom'):
            for p in r['pom']['pomieszczenia']:
                o_ = np.asarray(p['obrys']); ax.fill(o_[:, 0] + dx, o_[:, 1], alpha=0.18, color='tab:green', lw=0)
                ax.text(p['srodek'][0] + dx, p['srodek'][1], (f"{p['nr']} {p['nazwa']}\n{p['pow_m2']:.2f} m²" +
                        (f"\nPN-ISO {p['pow_pn_iso_9836']:.2f}" if 'pow_pn_iso_9836' in p else '')).replace('.', ','), ha='center', va='center', fontsize=8)
        opis = f"kondygnacja {r['nr']}: h {r['wys']:.2f} m" + (f" | PN-ISO 9836: {r['sk']['pow_uzytkowa_pn_iso_9836']:.2f} m²" if r['sk'] else '')
        ax.text(r['P'][:, 0].min() + dx, r['P'][:, 1].max() + 0.5, opis.replace('.', ','), fontsize=10, color='g')
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper right', markerscale=10)
    ax.set_title(f'{os.path.basename(a.pcd)}: {len(wyniki)} kondygnacji')
    fig.tight_layout(); fig.savefig(os.path.join(a.wyjscie, 'rzut.png'), dpi=110); plt.close(fig)
    # DXF
    import ezdxf
    from dxf_wspolne import podzialka, warstwy
    d = ezdxf.new('R2010', setup=True); d.units = ezdxf.units.M
    warstwy(d, (('SCIANY', 7), ('SCIANY_PUNKTY', 8), ('CIECIE_1M', 1), ('OKNA', 4), ('DRZWI', 3), ('OTWORY_OPIS', 2), ('OPIS', 3), ('SKALA', 7),
                ('SKOS_1_40', 30), ('SKOS_2_20', 200)))
    msp = d.modelspace()
    if any(r.get('pom') for r in wyniki):
        import pomieszczenia as PM
        PM.warstwy_arch(d)
        for r, dx in zip(wyniki, dxs):
            if r.get('pom'):
                PM.rysuj_arch(msp, r['pom'], dx, None)
    nr_ot = 0
    for r, dx in zip(wyniki, dxs):
        for pl in r['sciany']:
            msp.add_lwpolyline([(float(x_ + dx), float(y)) for x_, y in pl], dxfattribs={'layer': 'SCIANY'})
        for x_, y in r['Wp'][::2]:
            msp.add_point((float(x_ + dx), float(y)), dxfattribs={'layer': 'SCIANY_PUNKTY'})
        Gb, ob = raster(r['B'], a.siatka, 2); ib, jb = np.nonzero(Gb)
        for x_, y in (np.stack([ib, jb], 1) * a.siatka + ob)[::2]:
            msp.add_point((float(x_ + dx), float(y)), dxfattribs={'layer': 'CIECIE_1M'})
        for o in r['otwory']:
            nr_ot += 1; i = nr_ot
            p0, p1 = np.array([o['x0'] + dx, o['y0']]), np.array([o['x1'] + dx, o['y1']]); nn = np.array([o['nx'], o['ny']])
            war = 'OKNA' if o['typ'] == 'okno' else 'DRZWI'
            for off in (-0.06, 0.06):                                      # dwie linie w grubosci sciany + zamkniecia
                msp.add_line(tuple(p0 + nn * off), tuple(p1 + nn * off), dxfattribs={'layer': war})
            msp.add_line(tuple(p0 - nn * 0.06), tuple(p0 + nn * 0.06), dxfattribs={'layer': war})
            msp.add_line(tuple(p1 - nn * 0.06), tuple(p1 + nn * 0.06), dxfattribs={'layer': war})
            if o['typ'] == 'drzwi':                                        # luk skrzydla (kierunek otwierania nieznany - do pokoju)
                rr = float(np.hypot(*(p1 - p0))); kat0 = np.degrees(np.arctan2(*(p1 - p0)[::-1])); kn = np.degrees(np.arctan2(nn[1], nn[0]))
                msp.add_line(tuple(p0), tuple(p0 + nn * rr), dxfattribs={'layer': war})
                a0, a1 = sorted((kat0, kn)); a1 = a1 if a1 - a0 <= 180 else a1 - 360
                msp.add_arc(tuple(p0), rr, min(a0, a1), max(a0, a1), dxfattribs={'layer': war})
            txt = (f"O{i} {o['szer']:.2f}x{o['wys']:.2f} p.{o['parapet']:.2f}" if o['typ'] == 'okno' else f"D{i} {o['szer']:.2f}x{o['wys']:.2f}").replace('.', ',')
            c_ = (p0 + p1) / 2 + nn * 0.35
            msp.add_text(txt, height=0.12, dxfattribs={'layer': 'OTWORY_OPIS'}).set_placement((float(c_[0]), float(c_[1])))
        if r['sk']:
            for prog, war in ((1.4, 'SKOS_1_40'), (2.2, 'SKOS_2_20')):
                for c in r['sk']['kontury'][prog]:
                    msp.add_lwpolyline([(float(x_ + dx), float(y)) for x_, y in c[::2]], dxfattribs={'layer': war, 'linetype': 'DASHED'})
        m0 = r['P'][:, :2].min(0)
        opis = f"kondygnacja {r['nr']}: wysokosc w swietle {r['wys']:.2f} m"
        if r['sk']:
            s_ = r['sk']
            opis += (f"; pod skosami: >=2,20 m {s_['pow_100']:.2f} m2, 1,40-2,20 m {s_['pow_50']:.2f} m2 (50 %), <1,40 m {s_['pow_pominieta']:.2f} m2; "
                     f"pow. wg PN-ISO 9836 {s_['pow_uzytkowa_pn_iso_9836']:.2f} m2, kubatura {s_['kubatura_m3']:.1f} m3")
        msp.add_text(opis.replace('.', ','), height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((float(m0[0] + dx), float(r['P'][:, 1].max() + 0.8)))
        podzialka(msp, float(m0[0] + dx), float(m0[1]) - 1.5)
    d.saveas(os.path.join(a.wyjscie, 'rzut.dxf'))
    print('zapisano rzut.png, rzut.dxf, wysokosci.txt, otwory.json, otwory.csv' + (', pomieszczenia.json/csv' if any(r.get('pom') for r in wyniki) else '') + (', skosy.json' if any(r['sk'] for r in wyniki) else ''))


if __name__ == '__main__':
    main()

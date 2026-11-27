"""Rzut kondygnacji z chmury 3D (Point-LIO / Unitree L2).

    python l2_rzut.py <chmura.pcd> <katalog_wyniku> [--siatka 0.02]

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('wyjscie')
    ap.add_argument('--siatka', type=float, default=0.02)
    ap.add_argument('--bez-otworow', action='store_true', help='nie szukaj okien i drzwi')
    a = ap.parse_args()
    os.makedirs(a.wyjscie, exist_ok=True)
    P = czytaj_pcd(a.pcd)
    P = P[np.isfinite(P).all(1)]
    print(f'chmura: {len(P)} punktow, zakres {np.ptp(P, 0).round(1).tolist()} m')
    podl, suf = poziomy(P[:, 2])
    wys = suf - podl
    print(f'podloga {podl:+.2f} m, sufit {suf:+.2f} m, wysokosc w swietle {wys:.2f} m')
    A = P[(P[:, 2] > suf - 0.45) & (P[:, 2] < suf - 0.15), :2]      # sciany pod sufitem
    B = P[(P[:, 2] > podl + 1.0) & (P[:, 2] < podl + 1.2), :2]       # ciecie 1 m
    if len(A) < 100:
        A = P[(P[:, 2] > podl + 1.6) & (P[:, 2] < podl + 2.0), :2]  # niski sufit / brak sufitu - pas 1,6-2,0 m
    # prog gestosci dopasowany do chmury: komorka sciany ma miec wiecej punktow niz typowy szum
    G0, oa = raster(A, a.siatka, 1)
    ij = ((A - oa) / a.siatka).astype(int); H = np.zeros(G0.shape, int); np.add.at(H, (ij[:, 0], ij[:, 1]), 1)
    prog = max(1, int(np.percentile(H[H > 0], 30)))
    Ga = ndimage.binary_closing(H >= prog, np.ones((3, 3)))        # rzadkie punkty sciany -> ciagla linia
    from skimage.morphology import remove_small_objects
    Ga = remove_small_objects(Ga, max_size=12) & (H > 0)
    ia, ja = np.nonzero(Ga); Wp = np.stack([ia, ja], 1) * a.siatka + oa + a.siatka / 2
    sciany = []
    try:
        from eksport_dxf import szkielet
        sciany = szkielet(Wp, kom=0.03)
    except Exception as e:
        print('szkielet scian pominiety:', e)
    ot = [] if a.bez_otworow else otwory(P, sciany, podl, suf)
    import json
    with open(os.path.join(a.wyjscie, 'otwory.json'), 'w', encoding='utf-8') as f:
        json.dump(ot, f, ensure_ascii=False, indent=1)
    with open(os.path.join(a.wyjscie, 'otwory.csv'), 'w', encoding='utf-8') as f:
        f.write('nr;typ;szerokosc_m;wysokosc_m;parapet_m;nadproze_m;x0;y0;x1;y1\n')
        for i, o in enumerate(ot, 1):
            f.write(f"{i};{o['typ']};{o['szer']};{o['wys']};{o['parapet']};{o['nadproze']};{o['x0']};{o['y0']};{o['x1']};{o['y1']}\n".replace('.', ','))
    print(f"otwory: {sum(o['typ'] == 'okno' for o in ot)} okien, {sum(o['typ'] == 'drzwi' for o in ot)} drzwi")
    with open(os.path.join(a.wyjscie, 'wysokosci.txt'), 'w', encoding='utf-8') as f:
        f.write(f'podloga_m={podl:.3f}\nsufit_m={suf:.3f}\nwysokosc_w_swietle_m={wys:.3f}\npunktow={len(P)}\n')
    # obraz
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    ext = np.ptp(P[:, :2], 0)
    fig, ax = plt.subplots(figsize=(min(24, 4 + ext[0] / 3), min(24, 3 + ext[1] / 3)))
    ax.scatter(B[::2, 0], B[::2, 1], s=0.3, c='tab:red', lw=0, label='cięcie 1,0–1,2 m (okna, meble)')
    ax.scatter(Wp[:, 0], Wp[:, 1], s=1.5, c='k', lw=0, label='ściany (pas pod sufitem)')
    for pl in sciany: ax.plot(pl[:, 0], pl[:, 1], '-', color='tab:blue', lw=0.8)
    for i, o in enumerate(ot, 1):
        kol = 'tab:cyan' if o['typ'] == 'okno' else 'tab:green'
        ax.plot([o['x0'], o['x1']], [o['y0'], o['y1']], '-', color=kol, lw=4, solid_capstyle='butt')
        ax.annotate(f"{'O' if o['typ'] == 'okno' else 'D'}{i}", ((o['x0'] + o['x1']) / 2 + o['nx'] * 0.3, (o['y0'] + o['y1']) / 2 + o['ny'] * 0.3),
                    fontsize=7, color=kol, ha='center', va='center')
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper right', markerscale=10)
    ax.set_title(f'{os.path.basename(a.pcd)}: wysokość w świetle {wys:.2f} m, {ext[0]:.1f} × {ext[1]:.1f} m')
    fig.tight_layout(); fig.savefig(os.path.join(a.wyjscie, 'rzut.png'), dpi=110); plt.close(fig)
    # DXF
    import ezdxf
    from dxf_wspolne import podzialka, warstwy
    d = ezdxf.new('R2010', setup=True); d.units = ezdxf.units.M
    warstwy(d, (('SCIANY', 7), ('SCIANY_PUNKTY', 8), ('CIECIE_1M', 1), ('OKNA', 4), ('DRZWI', 3), ('OTWORY_OPIS', 2), ('OPIS', 3), ('SKALA', 7)))
    msp = d.modelspace()
    for pl in sciany:
        msp.add_lwpolyline([(float(x), float(y)) for x, y in pl], dxfattribs={'layer': 'SCIANY'})
    for x, y in Wp[::2]:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'SCIANY_PUNKTY'})
    Gb, ob = raster(B, a.siatka, 2); ib, jb = np.nonzero(Gb)
    for x, y in (np.stack([ib, jb], 1) * a.siatka + ob)[::2]:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'CIECIE_1M'})
    for i, o in enumerate(ot, 1):
        p0, p1 = np.array([o['x0'], o['y0']]), np.array([o['x1'], o['y1']]); nn = np.array([o['nx'], o['ny']])
        war = 'OKNA' if o['typ'] == 'okno' else 'DRZWI'
        for off in (-0.06, 0.06):                                      # dwie linie w grubosci sciany + zamkniecia
            msp.add_line(tuple(p0 + nn * off), tuple(p1 + nn * off), dxfattribs={'layer': war})
        msp.add_line(tuple(p0 - nn * 0.06), tuple(p0 + nn * 0.06), dxfattribs={'layer': war})
        msp.add_line(tuple(p1 - nn * 0.06), tuple(p1 + nn * 0.06), dxfattribs={'layer': war})
        if o['typ'] == 'drzwi':                                        # luk skrzydla (kierunek otwierania nieznany - do pokoju)
            r = float(np.hypot(*(p1 - p0))); kat0 = np.degrees(np.arctan2(*(p1 - p0)[::-1])); kn = np.degrees(np.arctan2(nn[1], nn[0]))
            msp.add_line(tuple(p0), tuple(p0 + nn * r), dxfattribs={'layer': war})
            a0, a1 = sorted((kat0, kn)); a1 = a1 if a1 - a0 <= 180 else a1 - 360
            msp.add_arc(tuple(p0), r, min(a0, a1), max(a0, a1), dxfattribs={'layer': war})
        txt = (f"O{i} {o['szer']:.2f}x{o['wys']:.2f} p.{o['parapet']:.2f}" if o['typ'] == 'okno' else f"D{i} {o['szer']:.2f}x{o['wys']:.2f}").replace('.', ',')
        c_ = (p0 + p1) / 2 + nn * 0.35
        msp.add_text(txt, height=0.12, dxfattribs={'layer': 'OTWORY_OPIS'}).set_placement((float(c_[0]), float(c_[1])))
    m0 = P[:, :2].min(0)
    msp.add_text(f'wysokosc w swietle {wys:.2f} m', height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((float(m0[0]), float(P[:, 1].max() + 0.8)))
    podzialka(msp, float(m0[0]), float(m0[1]) - 1.5)
    d.saveas(os.path.join(a.wyjscie, 'rzut.dxf'))
    print('zapisano rzut.png, rzut.dxf, wysokosci.txt, otwory.json, otwory.csv')


if __name__ == '__main__':
    main()

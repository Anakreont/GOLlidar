"""Rzut kondygnacji z chmury 3D (Point-LIO / Unitree L2).

    python l2_rzut.py <chmura.pcd> <katalog_wyniku> [--siatka 0.02]

- podloga i sufit z histogramu wysokosci (najnizsze i najwyzsze duze skupisko),
- SCIANY: pas 15-45 cm pod sufitem (meble tam nie siegaja) -> czysty obrys,
- CIECIE_1M: pas 1,0-1,2 m nad podloga (okna jako przerwy w scianie, meble, drzwi),
- wysokosc w swietle = sufit - podloga.
Wynik: rzut.png, rzut.dxf (metry; warstwy SCIANY, SCIANY_PUNKTY, CIECIE_1M, OPIS, SKALA), wysokosci.txt.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('wyjscie')
    ap.add_argument('--siatka', type=float, default=0.02)
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
    with open(os.path.join(a.wyjscie, 'wysokosci.txt'), 'w', encoding='utf-8') as f:
        f.write(f'podloga_m={podl:.3f}\nsufit_m={suf:.3f}\nwysokosc_w_swietle_m={wys:.3f}\npunktow={len(P)}\n')
    # obraz
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    ext = np.ptp(P[:, :2], 0)
    fig, ax = plt.subplots(figsize=(min(24, 4 + ext[0] / 3), min(24, 3 + ext[1] / 3)))
    ax.scatter(B[::2, 0], B[::2, 1], s=0.3, c='tab:red', lw=0, label='cięcie 1,0–1,2 m (okna, meble)')
    ax.scatter(Wp[:, 0], Wp[:, 1], s=1.5, c='k', lw=0, label='ściany (pas pod sufitem)')
    for pl in sciany: ax.plot(pl[:, 0], pl[:, 1], '-', color='tab:blue', lw=0.8)
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper right', markerscale=10)
    ax.set_title(f'{os.path.basename(a.pcd)}: wysokość w świetle {wys:.2f} m, {ext[0]:.1f} × {ext[1]:.1f} m')
    fig.tight_layout(); fig.savefig(os.path.join(a.wyjscie, 'rzut.png'), dpi=110); plt.close(fig)
    # DXF
    import ezdxf
    from dxf_wspolne import podzialka, warstwy
    d = ezdxf.new('R2010', setup=True); d.units = ezdxf.units.M
    warstwy(d, (('SCIANY', 7), ('SCIANY_PUNKTY', 8), ('CIECIE_1M', 1), ('OPIS', 3), ('SKALA', 7)))
    msp = d.modelspace()
    for pl in sciany:
        msp.add_lwpolyline([(float(x), float(y)) for x, y in pl], dxfattribs={'layer': 'SCIANY'})
    for x, y in Wp[::2]:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'SCIANY_PUNKTY'})
    Gb, ob = raster(B, a.siatka, 2); ib, jb = np.nonzero(Gb)
    for x, y in (np.stack([ib, jb], 1) * a.siatka + ob)[::2]:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'CIECIE_1M'})
    m0 = P[:, :2].min(0)
    msp.add_text(f'wysokosc w swietle {wys:.2f} m', height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((float(m0[0]), float(P[:, 1].max() + 0.8)))
    podzialka(msp, float(m0[0]), float(m0[1]) - 1.5)
    d.saveas(os.path.join(a.wyjscie, 'rzut.dxf'))
    print('zapisano rzut.png, rzut.dxf, wysokosci.txt')


if __name__ == '__main__':
    main()

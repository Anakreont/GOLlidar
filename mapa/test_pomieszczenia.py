"""Test podzialu na pomieszczenia na syntetycznych domach: python3 test_pomieszczenia.py [png]
Sciany jako linie (lico), punkty z szumem 2 cm, okna bez punktow, wolnostojace meble, smieci poza budynkiem,
obrot 7 st. Wzorzec = powierzchnie pomieszczen miedzy licami; dopuszczalny blad 3 %."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pomieszczenia


def dom(sciany, meble, trasa, sala=(), obrot=7, seed=1):
    rng = np.random.default_rng(seed)
    W = []
    for x1, y1, x2, y2, *pr in sciany:
        pr = pr[0] if pr else []
        L = np.hypot(x2 - x1, y2 - y1); u = np.array([x2 - x1, y2 - y1]) / L; a = 0
        for p, q in sorted(pr) + [(L, L)]:
            if p > a:
                W.append((x1 + u[0] * a, y1 + u[1] * a, x1 + u[0] * p, y1 + u[1] * p))
            a = q
    th = np.radians(obrot); R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    pts = []
    for x1, y1, x2, y2 in W:
        n = int(np.hypot(x2 - x1, y2 - y1) / 0.02); t = rng.random(n)
        pts.append(np.c_[x1 + (x2 - x1) * t, y1 + (y2 - y1) * t] + rng.normal(0, 0.02, (n, 2)))
    for cx, cy, r in meble:
        a = rng.random(300) * 2 * np.pi; pts.append(np.c_[cx + r * np.cos(a), cy + r * np.sin(a)])
    P = np.vstack(pts); pts.append(np.c_[rng.uniform(P[:, 0].min() - 4, P[:, 0].max() + 4, 200),
                                         rng.uniform(P[:, 1].min() - 4, P[:, 1].max() + 4, 200)])
    P = np.vstack(pts) @ R.T
    tr = np.array(trasa, float)
    tr = np.vstack([np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / 0.1))) for a, b in zip(tr[:-1], tr[1:])]) @ R.T
    zn = [{'rodzaj': 'sala', 'opis': n, 'x': float((np.array(xy) @ R.T)[0]), 'y': float((np.array(xy) @ R.T)[1])} for n, xy in sala]
    return {'nr': 0, 'sciany': [(np.array([[a, b], [c, d]]) @ R.T).tolist() for a, b, c, d in W],
            'chmura_cm': np.round(P * 100).astype(int).ravel().tolist(), 'trasa': tr.round(2).tolist(), 'znaczniki': zn, 'wysokosc_m': 2.6}


# A: korytarz 1,5 m, drzwi 0,9 m, salon wejscie 1,0 m, duze okna (do 2 m)
A = dom([(0, 0, 15, 0, [(1.2, 2.7), (5, 6.5), (12, 13.5)]), (15, 0, 15, 8.5, [(3, 5)]), (15, 8.5, 0, 8.5, [(2, 3.5), (9, 10.5)]),
         (0, 8.5, 0, 0, [(3, 4.2)]), (0, 3.5, 11, 3.5, [(1.5, 2.4), (5.5, 6.4), (9, 9.9)]), (0, 5.0, 11, 5.0, [(2, 2.9), (7, 7.9)]),
         (4, 0, 4, 3.5), (8, 0, 8, 3.5), (5, 5, 5, 8.5), (11, 0, 11, 8.5, [(3.7, 4.7)])],
        [(2, 1.5, 0.4), (13, 6, 0.6), (7, 7, 0.5)],
        [[0.6, 4.25], [1.95, 4.25], [1.95, 2], [1.95, 4.25], [5.9, 4.25], [5.9, 2], [5.9, 4.25], [9.45, 4.25], [9.45, 2], [9.45, 4.25],
         [7.45, 4.25], [7.45, 5.6], [8.5, 6.5], [7.45, 5.6], [7.45, 4.25], [2.45, 4.25], [2.45, 7], [2.45, 4.25], [10.5, 4.25], [13, 4.2],
         [13, 2], [13, 4.2], [14, 7.5], [13, 4.2]], sala=[('Salon', (13, 6.5))])
WZOR_A = [14.0, 14.0, 10.5, 17.5, 21.0, 34.0, 16.5]

# B: waski korytarz 1,05 m, drzwi 0,85 m, WC 1,2 x 1,3 m, salon z kuchnia przez otwor 2,0 m z oscieznicami
B = dom([(0, 0, 12, 0, [(1.0, 2.2), (8, 9.5)]), (12, 0, 12, 9, [(2, 3.5), (6, 7.5)]), (12, 9, 0, 9, [(1.5, 3)]), (0, 9, 0, 0, [(6, 7.2)]),
         (0, 4, 7, 4, [(1.0, 1.85), (4.5, 5.35)]),          # sciana korytarz / pokoje dolne (drzwi 0,85)
         (0, 5.05, 3.0, 5.05, [(1.2, 2.05)]),                # korytarz / pokoj gorny
         (3.0, 5.05, 3.0, 9), (3.0, 5.05, 4.2, 5.05),
         (4.2, 5.05, 4.2, 6.35, [(0.1, 0.95)]),              # WC 1,2 x 1,3 m, drzwi z korytarza? (sciana pionowa)
         (3.0, 6.35, 4.2, 6.35), (4.2, 5.05, 7.0, 5.05, [(1.0, 1.85)]),
         (7, 0, 7, 9, [(4.07, 4.98)]),                      # koniec korytarza -> salon drzwi 0,91 m
         (3.5, 0, 3.5, 4),
         (7, 6.5, 12, 6.5, [(1.5, 3.5)])],                   # salon / kuchnia: otwor 2,0 m
        [(10, 3, 0.5)],
        [[0.5, 4.5], [1.4, 4.5], [1.4, 2], [1.4, 4.5], [1.6, 4.5], [1.6, 7], [1.6, 4.5], [3.6, 4.5], [3.6, 5.7], [3.6, 4.5],
         [4.9, 4.5], [4.9, 2], [4.9, 4.5], [5.4, 4.5], [5.4, 7], [5.4, 4.5], [6.5, 4.5], [8.5, 4.5], [9, 2], [9, 5], [9.5, 6.5], [9.5, 8], [9.5, 5]],
        sala=[('Salon', (9.5, 3)), ('Kuchnia', (9.5, 7.7)), ('WC', (3.6, 5.7))], obrot=-4, seed=3)
# pokoje: dol L 3,5x4 =14; dol P 3,5x4=14; korytarz 7x1,05 = 7,35; gora L 3x3,95=11,85; WC 1,2x1,3=1,56;
# gora P: (7-3)x(9-5,05) - WC 1,56 - wneka obok WC = 4*3,95-1,56 = 14,24; salon 5x6,5 = 32,5; kuchnia 5x2,5 = 12,5
WZOR_B = [14.0, 14.0, 7.35, 11.85, 1.56, 14.24, 32.5, 12.5]


def sprawdz(nazwa, k, wzor, png=None):
    w = pomieszczenia.podziel(k)
    if png:
        pomieszczenia.podglad(k, w, png)
    wyn = sorted(p['pow_m2'] for p in w['pomieszczenia']); wz = sorted(wzor)
    print(nazwa, 'wykryte:', wyn); print(nazwa, 'wzorzec:', wz)
    print(nazwa, 'drzwi:', [(d['typ'], d['szer_m'], d['oscieza']) for d in w['drzwi']])
    assert len(wyn) == len(wz), f'{nazwa}: zla liczba pomieszczen ({len(wyn)} zamiast {len(wz)})'
    bl = max(abs(a / b - 1) for a, b in zip(wyn, wz))
    print(nazwa, 'maks. blad powierzchni: %.1f %%' % (100 * bl))
    # wzorzec liczony do osi linii scian bez grubosci, a program wycina wneki drzwiowe (pas punktow ~15 cm) - stad tolerancja
    assert bl < 0.035 or max(abs(a - b) for a, b in zip(wyn, wz)) < 0.3, f'{nazwa}: blad powierzchni'


if __name__ == '__main__':
    png = len(sys.argv) > 1
    sprawdz('A', A, WZOR_A, 'test_A.png' if png else None)
    sprawdz('B', B, WZOR_B, 'test_B.png' if png else None)
    print('OK')

"""Test podzialu na pomieszczenia na rysunkach CAD: symulowane przejscie ze skanerem (D500: 450 promieni/obrot, zasieg 12 m,
szum 2 cm, poza co 0,4 m) przez wszystkie etykiety pomieszczen, potem mapa/pomieszczenia.py i porownanie z powierzchniami
z rysunku. Wzorzec "fizyczny": drzwi i otwory zamkniete, wiec pomieszczenia rozdzielone w rysunku tylko linia (np. VirtWalls
ArCADii) porownywane razem - skaner ich nie rozrozni.
Uzycie: python3 wzorzec.py rysunek.dxf rys.pkl ; python3 symulacja.py rys.pkl [rys2.pkl ...] [--png]
Wynik 27.11.2026 (P29, do_Thermo, Kielce PB-A): 105/166 pomieszczen +-5 %, mediana |bledu| 1,5 %.
"""
import os, sys, pickle, numpy as np
from collections import Counter, defaultdict
from scipy import ndimage
from skimage.graph import MCP_Geometric
from matplotlib.path import Path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'mapa'))
import pomieszczenia as PM

K_ = 0.03


def symuluj(Kd, seed=0):
    rng = np.random.default_rng(seed)
    S = Kd['S'] + Kd['okna']                     # okna jako przeszkoda (rama/szyba odbija czesc wiazki)
    P = np.array([p for s in S for p in s]); m0 = P.min(0) - 2
    shape = tuple(np.ceil((P.max(0) + 2 - m0) / K_).astype(int))
    G0 = np.zeros(shape, bool)
    for a, b in S:
        PM.rysuj_odcinki(G0, np.array([(a - m0) / K_, (b - m0) / K_]))
    G = ndimage.binary_dilation(G0)
    obw = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(G, 60), np.ones((45, 45)))[60:-60, 60:-60])
    DR = []
    for a, b in Kd['DR']:
        if (b - a).max() > 3.5:
            continue
        DR.append((np.floor((a - m0) / K_).astype(int), np.ceil((b - m0) / K_).astype(int)))
    Gz = G.copy()
    for a, b in DR:
        Gz[a[0]:b[0] + 1, a[1]:b[1] + 1] = True
    FIZ, _ = ndimage.label(~Gz & obw)
    for a, b in DR:
        G[a[0]:b[0] + 1, a[1]:b[1] + 1] = False; G0[a[0]:b[0] + 1, a[1]:b[1] + 1] = False
    wolne = ~G & obw
    E, pkt = [], []
    for e in Kd['E']:
        p = np.round((e['xy'] - m0) / K_).astype(int)
        if not ((p >= 0).all() and (p < np.array(shape)).all() and obw[p[0], p[1]]):
            continue
        if not wolne[p[0], p[1]]:
            o = 17; c0 = np.maximum(p - o, 0); sub = wolne[c0[0]:p[0]+o, c0[1]:p[1]+o]; q = np.argwhere(sub)
            if not len(q):
                continue
            p = c0 + q[np.argmin(((q - (p - c0)) ** 2).sum(1))]
        f = FIZ[p[0], p[1]]
        if f == 0:
            o = 27; c0 = np.maximum(p - o, 0); sub = FIZ[c0[0]:p[0]+o, c0[1]:p[1]+o]; q = np.argwhere(sub > 0)
            f = sub[tuple(q[np.argmin(((q - (p - c0)) ** 2).sum(1))])] if len(q) else -1
        E.append(dict(e, fiz=int(f))); pkt.append(p)
    if not pkt:
        return None, [], 0
    lab_, _ = ndimage.label(wolne)
    glowna = Counter(lab_[p[0], p[1]] for p in pkt).most_common(1)[0][0]
    osiag = [lab_[p[0], p[1]] == glowna for p in pkt]
    pomin = len(pkt) - sum(osiag)
    E = [e for e, o in zip(E, osiag) if o]; pkt = [p for p, o in zip(pkt, osiag) if o]
    dist = ndimage.distance_transform_edt(wolne) * K_
    koszt = np.where(wolne, 1 + 8 * np.exp(-dist / 0.4), -1.0)
    trasa = [pkt[0]]; zost = pkt[1:]; cur = pkt[0]
    while zost:
        mcp = MCP_Geometric(koszt); cum, _ = mcp.find_costs([tuple(cur)])
        c = [cum[p[0], p[1]] for p in zost]; i = int(np.argmin(c))
        if not np.isfinite(c[i]):
            zost.pop(i); continue
        trasa += mcp.traceback(tuple(zost[i])); cur = zost.pop(i)
    tr = np.array(trasa, float)
    d_ = np.r_[0, np.cumsum(np.hypot(*np.diff(tr, axis=0).T))] * K_
    poz_ = tr[np.searchsorted(d_, np.arange(0, d_[-1], 0.4))]
    kat = np.linspace(0, 2 * np.pi, 450, endpoint=False); r = np.arange(1, 12 / K_, 0.8)
    dx = np.cos(kat)[:, None] * r; dy = np.sin(kat)[:, None] * r; chm = []
    for px, py in poz_:
        X = np.clip(np.round(px + dx).astype(int), 0, shape[0] - 1); Y = np.clip(np.round(py + dy).astype(int), 0, shape[1] - 1)
        h = G0[X, Y] | ~obw[X, Y]; tr_ = G0[X, Y]
        ok = h.any(1); i = h.argmax(1); ok &= tr_[np.arange(len(kat)), i]
        rr = r[i[ok]] * K_ + rng.normal(0, 0.02, ok.sum())
        chm.append(np.c_[px * K_ + m0[0] + rr * np.cos(kat[ok]), py * K_ + m0[1] + rr * np.sin(kat[ok])])
    if not chm or len(E) < 2:
        return None, E, pomin
    chm = np.vstack(chm)[::2]
    k = {'nr': 0, 'sciany': [], 'chmura_cm': np.round(chm * 100).astype(int).ravel().tolist(),
         'trasa': (tr[::4] * K_ + m0).round(2).tolist(), 'wysokosc_m': 3.0,
         'znaczniki': [{'rodzaj': 'sala', 'opis': str(e['nr']), 'x': float(e['xy'][0]), 'y': float(e['xy'][1])} for e in E]}
    return k, E, pomin


def porownaj(nazwa, k, E, png=False, log=True, w=None):
    if w is None:
        w = PM.podziel(k, log=lambda *a: None)
    if png:
        PM.podglad(k, w, f'sym_{nazwa}.png')
    gr = defaultdict(list)
    for e in E:
        gr[e['fiz']].append(e)
    pom_et = defaultdict(set); wyn = []
    for g, es in gr.items():
        traf = []
        for e in es:
            t = [p for p in w['pomieszczenia'] if Path(p['obrys']).contains_point(e['xy'])]
            traf.append(t[0]['nr'] if t else None)
            if t:
                pom_et[t[0]['nr']].add(g)
        wyn.append((g, es, traf))
    st = Counter(); bledy = []; E5 = []
    for g, es, traf in wyn:
        opis = ' + '.join(f"{e['nr']} {e['nazwa']}" for e in es); wz = sum(e['pole'] for e in es)
        if all(t is None for t in traf):
            st['nie wykryte'] += 1; bledy.append(f'{opis} {wz:.2f}: nie wykryte'); continue
        if len(set(traf)) > 1:
            st['rozdzielone'] += 1; bledy.append(f'{opis} {wz:.2f}: podzielone na {len(set(traf))}'); continue
        if len(pom_et[traf[0]]) > 1:
            st['sklejone'] += 1; bledy.append(f'{opis} {wz:.2f}: sklejone z innym'); continue
        pw = next(p['pow_m2'] for p in w['pomieszczenia'] if p['nr'] == traf[0])
        e_ = (pw - wz) / wz; E5.append(e_)
        if abs(e_) <= 0.05:
            st['zgodne'] += 1
        else:
            st['zla pow.'] += 1; bledy.append(f'{opis} {wz:.2f} -> {pw} ({e_:+.0%})')
    if log:
        print(f"{nazwa}: fizycznych {len(gr)}: zgodne {st['zgodne']}, zla pow. {st['zla pow.']}, sklejone {st['sklejone']}, "
              f"rozdzielone {st['rozdzielone']}, nie wykryte {st['nie wykryte']}"
              + (f'; mediana |bledu| {np.median(np.abs(E5)) * 100:.1f} %, sredni blad {np.mean(E5) * 100:+.1f} %' if E5 else ''))
        for b in bledy:
            print('   ', b)
    return st, len(gr), E5


if __name__ == '__main__':
    png = '--png' in sys.argv
    RAZEM = Counter(); N = 0; WSZ = []
    for plik in [a for a in sys.argv[1:] if not a.startswith('--')]:
        for Kd in pickle.load(open(plik, 'rb')):
            nazwa = os.path.basename(plik).replace('.pkl', '') + '_' + Kd['nazwa'].replace(' ', '')
            k, E, pomin = symuluj(Kd)
            if k is None or len(E) < 2:
                continue
            st, n, E5 = porownaj(nazwa, k, E, png)
            if pomin:
                print(f'    ({pomin} etykiet pominietych - nieosiagalne na rysunku)')
            RAZEM.update(st); N += n; WSZ += E5
    print(f"\nRAZEM: {N} pomieszczen fizycznych: zgodne +-5 % {RAZEM['zgodne']} ({RAZEM['zgodne'] / max(N, 1):.0%}), zla pow. {RAZEM['zla pow.']}, "
          f"sklejone {RAZEM['sklejone']}, rozdzielone {RAZEM['rozdzielone']}, nie wykryte {RAZEM['nie wykryte']}; "
          f"mediana |bledu| {np.median(np.abs(WSZ)) * 100:.1f} %, sredni {np.mean(WSZ) * 100:+.1f} %")

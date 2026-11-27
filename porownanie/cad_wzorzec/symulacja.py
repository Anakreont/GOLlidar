"""Symulacja przejscia ze skanerem (D500: 450 promieni, 12 m, szum 2 cm) po rzucie z ArCADii (dane.pkl z arcadia_czytaj.py),
podzial na pomieszczenia i porownanie z powierzchniami z rysunku. Pomieszczenia rozdzielone w ArCADii tylko linia (VirtWalls)
porownywane razem. Uzycie: python3 symulacja.py  (w katalogu z dane.pkl). Kody kondygnacji w NAZWY - dopasowac do rysunku."""
import pickle, re, sys, json, numpy as np
from scipy import ndimage
from skimage.graph import MCP_Geometric
from matplotlib.path import Path
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'mapa'))
import pomieszczenia as PM
D = pickle.load(open('dane.pkl', 'rb'))
NAZWY = {'L28DBAD06C4D4C098': 'piwnica', 'L28D8F10432558039': 'parter', 'L25B6C0843594C17B': 'pietro', 'L2A0DBA4FB415419A': 'poddasze'}
K = 0.02; rng = np.random.default_rng(0)
czysc = lambda s: re.sub(r'\\[^;]*;|[{}]', '', s).strip()

def odcinki(poz, typy=('Walls', 'Walls_10C148FA32D4A199', 'Pillars', 'Windows')):
    S = []
    for d in D:
        if d['poz'] != poz or d['typ'] not in typy: continue
        for t, g in d['geo']:
            if t in ('P', 'L') and g:
                p = [q for q in np.asarray(g, float)[:, :2] if abs(q[0]) > 1e-6 or abs(q[1]) > 1e-6]
                S += [(a, b) for a, b in zip(p[:-1], p[1:])]
    return [(a / 100, b / 100) for a, b in S]           # cm -> m

def etykiety(poz):
    W = []
    for d in D:
        if d['poz'] != poz or not d['typ'].startswith('Rooms') or d['typ'].endswith('_I'): continue
        T = [(np.array(g[0][:2]) / 100, czysc(g[1])) for t, g in d['geo'] if t == 'T']
        pole = [float(s.replace(' m', '')) for _, s in T if re.match(r'^\d+\.\d+ m$', s)]
        if not pole: continue
        nr = T[0][1]; nazwa = T[1][1] if len(T) > 1 else ''
        xy = np.mean([p for p, _ in T[:2]], axis=0)
        W.append({'nr': nr, 'nazwa': nazwa, 'pole': pole[0], 'xy': xy})
    return W

def symuluj(poz, okna_szklo=False):
    S = odcinki(poz, ('Walls', 'Walls_10C148FA32D4A199', 'Pillars') + (() if okna_szklo else ('Windows',)))
    E = etykiety(poz)
    P = np.array([p for s in S for p in s]); m0 = P.min(0) - 2; shape = tuple(np.ceil((P.max(0) + 2 - m0) / K).astype(int))
    G = np.zeros(shape, bool); DR = []
    for a, b in S:
        PM.rysuj_odcinki(G, np.array([(a - m0) / K, (b - m0) / K]))
    G0 = G.copy()                             # do promieni: lico bez pogrubienia
    G = ndimage.binary_dilation(G)
    obw = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(G, 80), np.ones((60, 60)))[80:-80, 80:-80])
    # drzwi i otwory otwarte: w prostokacie obiektu Doors/Holes usuwamy przeszkody (oscieznica, prog, skrzydlo)
    for d in D:
        if d['poz'] != poz or d['typ'] not in ('Doors', 'Holes'): continue
        Q = [q for t, g in d['geo'] if t in ('P', 'L') and g for q in np.asarray(g, float)[:, :2] if abs(q[0]) > 1e-6 or abs(q[1]) > 1e-6]
        if not Q: continue
        Q = (np.array(Q) / 100 - m0) / K; a = np.floor(Q.min(0)).astype(int); b = np.ceil(Q.max(0)).astype(int)
        if ((b - a) * K).max() > 3.5: continue
        DR.append((a, b))
    Gz = G.copy()
    for a, b in DR:
        Gz[a[0]:b[0] + 1, a[1]:b[1] + 1] = True
    FIZ, _ = ndimage.label(~Gz & obw)        # pomieszczenia fizyczne: drzwi/otwory zamkniete, linie wirtualne ArCADii pominiete
    for a, b in DR:
        # otwieramy tylko pas sciany: komorki przeszkod w prostokacie drzwi, ktore leza w linii otworu
        G[a[0]:b[0] + 1, a[1]:b[1] + 1] = False; G0[a[0]:b[0] + 1, a[1]:b[1] + 1] = False
    wolne = ~G & obw
    dist = ndimage.distance_transform_edt(wolne) * K
    koszt = np.where(wolne, 1 + 8 * np.exp(-dist / 0.4), -1.0)   # ujemny = nieprzejezdne (MCP)
    E = [e for e in E if np.all((e['xy'] - m0) / K >= 0) and np.all((e['xy'] - m0) / K < np.array(shape) - 1)
         and obw[tuple(np.round((e['xy'] - m0) / K).astype(int))]]
    pkt = []
    for e in E:                                   # etykieta na scianie/linii -> najblizsza wolna komorka
        p = np.round((e['xy'] - m0) / K).astype(int)
        if not wolne[p[0], p[1]]:
            o = 25; sub = wolne[p[0]-o:p[0]+o, p[1]-o:p[1]+o]
            if sub.any():
                q = np.argwhere(sub); p = p - o + q[np.argmin(((q - o) ** 2).sum(1))]
        pkt.append(p)
    for e, p in zip(E, pkt):
        f = FIZ[p[0], p[1]]
        if f == 0:                            # etykieta w prostokacie drzwi - najblizsza skladowa
            o = 40; sub = FIZ[p[0]-o:p[0]+o, p[1]-o:p[1]+o]; q = np.argwhere(sub > 0)
            f = sub[tuple(q[np.argmin(((q - o) ** 2).sum(1))])] if len(q) else -1
        e['fiz'] = int(f)
    # start w skladowej z najwieksza liczba etykiet; etykiety nieosiagalne (zamkniete drzwi w rysunku) pomijamy
    lab_, _ = ndimage.label(wolne)
    from collections import Counter
    glowna = Counter(lab_[p[0], p[1]] for p in pkt).most_common(1)[0][0]
    osiag = [lab_[p[0], p[1]] == glowna for p in pkt]
    E = [e for e, o in zip(E, osiag) if o]; pomin = len(osiag) - sum(osiag)
    pkt = [p for p, o in zip(pkt, osiag) if o]
    print(f'  {poz}: pominiete etykiety (nieosiagalne): {pomin}')
    trasa = [pkt[0]]; zost = pkt[1:]; cur = pkt[0]
    while zost:
        mcp = MCP_Geometric(koszt); cum, _ = mcp.find_costs([tuple(cur)])
        c = [cum[p[0], p[1]] for p in zost]; i = int(np.argmin(c))
        if not np.isfinite(c[i]): zost.pop(i); continue
        trasa += mcp.traceback(tuple(zost[i])); cur = zost.pop(i)
    tr = np.array(trasa, float)
    # D500: 450 promieni/obrot, zasieg 12 m, szum 2 cm, poza co 0,3 m
    d_ = np.r_[0, np.cumsum(np.hypot(*np.diff(tr, axis=0).T))] * K
    poz_ = tr[np.searchsorted(d_, np.arange(0, d_[-1], 0.3))]
    kat = np.linspace(0, 2 * np.pi, 450, endpoint=False); r = np.arange(1, 12 / K, 0.7)
    dx = np.cos(kat)[:, None] * r; dy = np.sin(kat)[:, None] * r; chm = []
    for px, py in poz_:
        X = np.clip(np.round(px + dx).astype(int), 0, shape[0] - 1); Y = np.clip(np.round(py + dy).astype(int), 0, shape[1] - 1)
        h = G0[X, Y]; ok = h.any(1); i = h.argmax(1)
        rr = r[i[ok]] * K + rng.normal(0, 0.02, ok.sum())
        chm.append(np.c_[px * K + m0[0] + rr * np.cos(kat[ok]), py * K + m0[1] + rr * np.sin(kat[ok])])
    chm = np.vstack(chm)[::2] if chm else np.zeros((0, 2))
    trm = tr[::5] * K + m0
    k = {'nr': 0, 'sciany': [], 'chmura_cm': np.round(chm * 100).astype(int).ravel().tolist(), 'trasa': trm.round(2).tolist(),
         'znaczniki': [{'rodzaj': 'sala', 'opis': e['nr'], 'x': float(e['xy'][0]), 'y': float(e['xy'][1])} for e in E], 'wysokosc_m': 3.0}
    return k, E

def porownaj(nazwa, k, E):
    w = PM.podziel(k)
    PM.podglad(k, w, f'sym_{nazwa}.png')
    from collections import defaultdict
    gr = defaultdict(list)
    for e in E:
        gr[e['fiz']].append(e)
    pom_et = defaultdict(set)                  # pomieszczenie programu -> grupy fizyczne, ktorych etykiety w nim leza
    wyn = []
    for g, es in gr.items():
        traf = []
        for e in es:
            t = [p for p in w['pomieszczenia'] if Path(p['obrys']).contains_point(e['xy'])]
            traf.append(t[0]['nr'] if t else None)
            if t: pom_et[t[0]['nr']].add(g)
        wyn.append((g, es, traf))
    ok = zle = sklej = podz = brak = 0; bledy = []; E5 = []
    for g, es, traf in wyn:
        opis = ' + '.join(f"{e['nr']} {e['nazwa']}" for e in es); wz = sum(e['pole'] for e in es)
        if all(t is None for t in traf):
            brak += 1; bledy.append(f'{opis} {wz:.2f}: nie wykryte'); continue
        if len(set(traf)) > 1:
            podz += 1; bledy.append(f'{opis} {wz:.2f}: podzielone na {len(set(traf))}'); continue
        nr = traf[0]
        if len(pom_et[nr]) > 1:
            sklej += 1; bledy.append(f'{opis} {wz:.2f}: sklejone z innym pomieszczeniem'); continue
        pw = next(p['pow_m2'] for p in w['pomieszczenia'] if p['nr'] == nr)
        e_ = (pw - wz) / wz; E5.append(e_)
        if abs(e_) <= 0.05: ok += 1
        else: zle += 1; bledy.append(f'{opis} {wz:.2f} -> {pw} ({e_:+.0%})')
    bez = len([p for p in w['pomieszczenia'] if p['nr'] not in pom_et])
    print(f'{nazwa}: pomieszczen fizycznych {len(gr)} (etykiet {len(E)}): zgodne +-5%: {ok}, zla powierzchnia: {zle}, '
          f'sklejone: {sklej}, rozdzielone: {podz}, nie wykryte: {brak}; nadmiarowe w programie: {bez}'
          + (f'; mediana bledu zgodnych {np.median(np.abs(E5)) * 100:.1f} %' if E5 else ''))
    for b in bledy: print('   ', b)
    return w, wyn

if __name__ == '__main__':
    R = {}
    for poz, nz in NAZWY.items():
        k, E = symuluj(poz)
        json.dump(k, open(f'sym_{nz}.json', 'w'))
        R[nz] = porownaj(nz, k, E)

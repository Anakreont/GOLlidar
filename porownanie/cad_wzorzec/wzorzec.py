"""Wzorzec do testu podzialu na pomieszczenia z rysunkow CAD (rysunkow klientow NIE dodawac do repo - jest publiczne).

Czyta rzut i zwraca kondygnacje: sciany (odcinki w m), drzwi/otwory (prostokaty w m), etykiety pomieszczen
(numer, nazwa, powierzchnia z rysunku, polozenie). Obslugiwane:
  * ArCADia-Architektura 3D (obiekty ARCADIA_RXENTITY z grafika zastepcza; warstwy isa_..._<kondygnacja><Typ>),
  * zwykly rysunek 2D z warstwami (domyslnie jak w KIELCE_PB-A: ARCH_SCIANY, ARCH_STOLARKA_DRZWI, ARCH_STOLARKA_OKNA,
    ARCH_OPIS POMIESZCZEN z tekstem "72.69 m2"); kondygnacje = osobne rzuty rozlozone na arkuszu.
Uzycie: python3 wzorzec.py rysunek.dxf [wynik.pkl]  ->  podsumowanie + pickle dla symulacja.py
"""
import re, sys, pickle, numpy as np
from collections import defaultdict
import ezdxf
from scipy import ndimage

CZYSC = lambda s: re.sub(r'\\[A-Za-z][^;\\]*;|[{}]|\^', '', s).replace('\\P', ' ').strip()
POLE_RE = re.compile(r'(\d+[.,]\d+)\s*m\s*2?')


def _warstwa(e):
    try:
        return e.dxf.layer
    except Exception:
        for sub in e.xtags.subclasses:
            for t in sub:
                if t.code == 8:
                    return t.value
    return ''


def _skala(drzwi_wym):
    """Jednostka rysunku z typowej szerokosci drzwi (0,8-1,2 m): zwraca mnoznik do metrow."""
    if not drzwi_wym:
        return 0.01
    m = np.median(drzwi_wym)
    return min((0.001, 0.01, 0.0254, 1.0), key=lambda s: abs(np.log(m * s / 1.0)))


# ------------------------------------------------------------------ ArCADia
def czytaj_arcadia(d):
    from ezdxf.proxygraphic import ProxyGraphic
    RE = re.compile(r'isa_V[0-9A-F]+_B([0-9A-F]{16})([LR][0-9A-F]{16})(\w+)')
    poz = defaultdict(lambda: {'S': [], 'DR': [], 'E': [], 'okna': []})
    for e in d.modelspace():
        if e.dxftype() not in ('ARCADIA_RXENTITY', 'ACAD_PROXY_ENTITY'):   # ArCADia zapisana z ArCADii / z AutoCADa
            continue
        m = RE.match(_warstwa(e))
        if not m or m.group(2)[0] != 'L':
            continue
        typ = m.group(3)
        if typ.endswith('_I'):
            continue
        klucz = m.group(1) + m.group(2)
        try:
            V = list(ProxyGraphic(e.proxy_graphic, d).virtual_entities())
        except Exception:
            continue
        pts, txt = [], []
        for v in V:
            t = v.dxftype()
            if t == 'LINE':
                pts.append([tuple(v.dxf.start)[:2], tuple(v.dxf.end)[:2]])
            elif t == 'LWPOLYLINE':
                pts.append([tuple(p)[:2] for p in v.get_points('xy')])
            elif t == 'POLYLINE':
                pts.append([tuple(p.dxf.location)[:2] for p in v.vertices])
            elif t == 'TEXT':
                txt.append((tuple(v.dxf.insert)[:2], CZYSC(v.dxf.text)))
            elif t == 'MTEXT':
                txt.append((tuple(v.dxf.insert)[:2], CZYSC(v.text)))
        pts = [[q for q in p if abs(q[0]) > 1e-6 or abs(q[1]) > 1e-6] for p in pts]   # grafika ArCADii ma odcinki do (0,0)
        pts = [p for p in pts if len(p) > 1]
        K = poz[klucz]
        if typ.startswith('Walls') or typ.startswith('Pillars'):
            K['S'] += [(a, b) for p in pts for a, b in zip(p[:-1], p[1:])]
        elif typ.startswith('Windows'):
            K['okna'] += [(a, b) for p in pts for a, b in zip(p[:-1], p[1:])]
        elif typ.startswith('Doors') or typ.startswith('Holes'):
            Q = np.array([q for p in pts for q in p])
            if len(Q):
                K['DR'].append((Q.min(0), Q.max(0)))
        elif typ.startswith('Rooms'):
            pola = [float(POLE_RE.match(s).group(1).replace(',', '.')) for _, s in txt if POLE_RE.match(s)]
            if pola and txt:
                xy = np.mean([p for p, _ in txt[:2]], axis=0)
                K['E'].append({'nr': txt[0][1], 'nazwa': txt[1][1] if len(txt) > 1 else '', 'pole': pola[0], 'xy': xy})
    wyn = []
    for klucz, K in poz.items():
        if not K['E'] or not K['S']:
            continue
        wyn.append(K | {'nazwa': klucz[-17:]})
    s = _skala([max(b - a) for x in wyn for a, b in x['DR'] if max(b - a) > 0])
    for K in wyn:
        _do_m(K, s)
    return wyn


# ------------------------------------------------------------------ zwykle warstwy
def czytaj_warstwy(d, sciany=('ARCH_SCIANY',), drzwi=('ARCH_STOLARKA_DRZWI',), okna=('ARCH_STOLARKA_OKNA',),
                   opisy=('ARCH_OPIS POMIESZCZEN',)):
    S, O, Dp, T = [], [], [], []
    def odc(e):
        t = e.dxftype()
        if t == 'LINE':
            return [(tuple(e.dxf.start)[:2], tuple(e.dxf.end)[:2])]
        if t == 'LWPOLYLINE':
            p = [tuple(q)[:2] for q in e.get_points('xy')]
            if e.closed:
                p.append(p[0])
            return list(zip(p[:-1], p[1:]))
        if t in ('ARC', 'CIRCLE', 'ELLIPSE', 'SPLINE'):
            try:
                p = [tuple(q)[:2] for q in e.flattening(5)]
                return list(zip(p[:-1], p[1:]))
            except Exception:
                return []
        if t == 'INSERT':
            r = []
            try:
                for v in e.virtual_entities():
                    r += odc(v)
            except Exception:
                pass
            return r
        return []
    for e in d.modelspace():
        L = _warstwa(e)
        if L in sciany:
            S += odc(e)
        elif L in okna:
            O += odc(e)
        elif L in drzwi:
            Dp += odc(e)
        elif L in opisy and e.dxftype() in ('MTEXT', 'TEXT'):
            T.append((tuple(e.dxf.insert)[:2], CZYSC(e.text if e.dxftype() == 'MTEXT' else e.dxf.text)))
    # kondygnacje = skupiska scian (przerwa > ~15 m przy jednostce cm)
    P = np.array([p for s in S for p in s])
    rozm = np.ptp(P, axis=0).max()
    kom = rozm / 400
    ij = np.floor((P - P.min(0)) / kom).astype(int)
    G = np.zeros(ij.max(0) + 1, bool); G[ij[:, 0], ij[:, 1]] = True
    lab, n = ndimage.label(ndimage.binary_fill_holes(ndimage.binary_dilation(G, iterations=10)))
    _, (ii, jj) = ndimage.distance_transform_edt(lab == 0, return_indices=True)
    lab = lab[ii, jj]                              # kazdy punkt arkusza -> najblizszy rzut
    def nr_sk(xy):
        q = np.floor((np.asarray(xy) - P.min(0)) / kom).astype(int)
        return lab[tuple(np.clip(q, 0, np.array(lab.shape) - 1))]
    # drzwi: skupiska geometrii warstwy drzwi
    DR = []
    if Dp:
        Q = np.array([p for s in Dp for p in s]); kd = 2.0 if rozm > 5000 else 0.02
        ij = np.floor((Q - Q.min(0)) / kd).astype(int); Gd = np.zeros(ij.max(0) + 1, bool)
        for a, b in Dp:
            for t in np.linspace(0, 1, 12):
                q = np.floor((np.array(a) + (np.array(b) - np.array(a)) * t - Q.min(0)) / kd).astype(int); Gd[q[0], q[1]] = True
        ld, _ = ndimage.label(ndimage.binary_dilation(Gd, iterations=2))
        for sl in ndimage.find_objects(ld):
            a = np.array([sl[0].start, sl[1].start]) * kd + Q.min(0); b = np.array([sl[0].stop, sl[1].stop]) * kd + Q.min(0)
            DR.append((a, b))
    # etykiety: tekst z powierzchnia + najblizsza nazwa i numer
    E = []
    pola = [(np.array(p), float(POLE_RE.search(s).group(1).replace(',', '.'))) for p, s in T if POLE_RE.fullmatch(s.strip()) or re.fullmatch(r'\d+[.,]\d+\s*m2', s.strip())]
    inne = [(np.array(p), s) for p, s in T if not POLE_RE.search(s)]
    for p, pole in pola:
        if pole <= 0:
            continue
        bl = sorted(inne, key=lambda x: np.hypot(*(x[0] - p)))[:4]
        nr = next((s for q, s in bl if re.fullmatch(r'-?\d+[A-Za-z]?', s)), '')
        nz = next((s for q, s in bl if not re.fullmatch(r'-?\d+[A-Za-z]?', s) and len(s) > 1), '')
        bliskie = [q for q, s in bl[:2]] + [p]
        E.append({'nr': nr, 'nazwa': nz, 'pole': pole, 'xy': np.mean(bliskie, axis=0)})
    wyn = []
    for k in range(1, n + 1):
        Ek = [e for e in E if nr_sk(e['xy']) == k]
        if not Ek:
            continue
        Sk = [s for s in S if nr_sk(s[0]) == k]
        x0, x1 = np.min([p for s in Sk for p in s], axis=0), np.max([p for s in Sk for p in s], axis=0)
        w = lambda q: (np.asarray(q) >= x0).all() and (np.asarray(q) <= x1).all()
        wyn.append({'nazwa': f'rzut {k}', 'S': Sk, 'okna': [s for s in O if w(s[0])], 'DR': [r for r in DR if w(r[0])], 'E': Ek})
    s = _skala([max(b - a) for a, b in DR if 0 < max(b - a)])
    for K in wyn:
        _do_m(K, s)
    wyn.sort(key=lambda K: -np.mean([e['xy'][1] for e in K['E']]))
    return wyn


def _do_m(K, s):
    K['S'] = [(np.array(a) * s, np.array(b) * s) for a, b in K['S']]
    K['okna'] = [(np.array(a) * s, np.array(b) * s) for a, b in K['okna']]
    K['DR'] = [(np.array(a) * s, np.array(b) * s) for a, b in K['DR']]
    for e in K['E']:
        e['xy'] = np.array(e['xy']) * s
    K['skala'] = s


def czytaj(plik):
    d = ezdxf.readfile(plik)
    if sum(1 for e in d.modelspace() if e.dxftype() in ('ARCADIA_RXENTITY', 'ACAD_PROXY_ENTITY')) > 200:
        return czytaj_arcadia(d)
    return czytaj_warstwy(d)


if __name__ == '__main__':
    K = czytaj(sys.argv[1])
    for k in K:
        print(f"{k['nazwa']}: scian {len(k['S'])}, okien {len(k['okna'])}, drzwi/otworow {len(k['DR'])}, etykiet {len(k['E'])}, "
              f"skala {k['skala']}, suma pow. {sum(e['pole'] for e in k['E']):.1f} m2")
    if len(sys.argv) > 2:
        pickle.dump(K, open(sys.argv[2], 'wb'))

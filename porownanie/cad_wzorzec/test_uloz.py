"""Test mapa/uloz_kawalki.py na rysunkach CAD: symulowane przejscie D500 przez wszystkie pomieszczenia, pociete na
kawalki (jak fragmenty SP162), kazdy kawalek obrocony i przesuniety losowo, znaczniki SALA przy etykietach pomieszczen.
Sprawdza, czy algorytm odtworzy prawdziwe polozenie kawalkow.
Uzycie: python3 test_uloz.py rys.pkl [--kondygnacja N] [--dl 30] [--ukryj 0.3] [--bez-sal] [--zgiecie st] [--png wynik.png]
"""
import os, sys, pickle, argparse, time, numpy as np
from collections import Counter
from scipy import ndimage
from skimage.graph import MCP_Geometric
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'mapa'))
import pomieszczenia as PM
import uloz_kawalki as U

K_ = 0.03


def przejscie(Kd, seed=0):
    """Trasa przez etykiety pomieszczen + skany z kazdej pozy (co 0,4 m). Zwraca pozy [m], skany (lista), etykiety na trasie."""
    rng = np.random.default_rng(seed)
    S = Kd['S'] + Kd['okna']
    P = np.array([p for s in S for p in s]); m0 = P.min(0) - 2
    shape = tuple(np.ceil((P.max(0) + 2 - m0) / K_).astype(int))
    G0 = np.zeros(shape, bool)
    for a, b in S:
        PM.rysuj_odcinki(G0, np.array([(a - m0) / K_, (b - m0) / K_]))
    G = ndimage.binary_dilation(G0)
    obw = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(G, 60), np.ones((45, 45)))[60:-60, 60:-60])
    for a, b in Kd['DR']:
        if (b - a).max() > 3.5:
            continue
        a = np.floor((a - m0) / K_).astype(int); b = np.ceil((b - m0) / K_).astype(int)
        G[a[0]:b[0] + 1, a[1]:b[1] + 1] = False; G0[a[0]:b[0] + 1, a[1]:b[1] + 1] = False
    wolne = ~G & obw
    E, pkt = [], []
    for e in Kd['E']:
        p = np.round((e['xy'] - m0) / K_).astype(int)
        if (p >= 0).all() and (p < np.array(shape)).all() and wolne[p[0], p[1]]:
            E.append(e); pkt.append(p)
    lab_, _ = ndimage.label(wolne)
    gl = Counter(lab_[p[0], p[1]] for p in pkt).most_common(1)[0][0]
    E = [e for e, p in zip(E, pkt) if lab_[p[0], p[1]] == gl]; pkt = [p for p in pkt if lab_[p[0], p[1]] == gl]
    dist = ndimage.distance_transform_edt(wolne) * K_
    koszt = np.where(wolne, 1 + 8 * np.exp(-dist / 0.4), -1.0)
    trasa = [pkt[0]]; zost = list(range(1, len(pkt))); cur = pkt[0]; wizyty = [(0, 0)]
    while zost:
        mcp = MCP_Geometric(koszt); cum, _ = mcp.find_costs([tuple(cur)])
        c = [cum[pkt[i][0], pkt[i][1]] for i in zost]; j = int(np.argmin(c))
        if not np.isfinite(c[j]):
            zost.pop(j); continue
        trasa += mcp.traceback(tuple(pkt[zost[j]])); wizyty.append((len(trasa) - 1, zost[j])); cur = pkt[zost.pop(j)]
    tr = np.array(trasa, float)
    d_ = np.r_[0, np.cumsum(np.hypot(*np.diff(tr, axis=0).T))] * K_
    ip = np.searchsorted(d_, np.arange(0, d_[-1], 0.4))
    poz = tr[ip] * K_ + m0
    etyk = [(int(np.searchsorted(ip, i)), E[k]['nr']) for i, k in wizyty]     # (indeks pozy, nr sali)
    kat = np.linspace(0, 2 * np.pi, 450, endpoint=False); r = np.arange(1, 12 / K_, 0.8)
    dx = np.cos(kat)[:, None] * r; dy = np.sin(kat)[:, None] * r; skany = []
    for px, py in tr[ip]:
        X = np.clip(np.round(px + dx).astype(int), 0, shape[0] - 1); Y = np.clip(np.round(py + dy).astype(int), 0, shape[1] - 1)
        h = G0[X, Y] | ~obw[X, Y]; tr_ = G0[X, Y]
        ok = h.any(1); i = h.argmax(1); ok &= tr_[np.arange(len(kat)), i]
        rr = r[i[ok]] * K_ + rng.normal(0, 0.02, ok.sum())
        skany.append(np.c_[px * K_ + m0[0] + rr * np.cos(kat[ok]), py * K_ + m0[1] + rr * np.sin(kat[ok])][::3])
    return poz, skany, etyk, E


def potnij(poz, skany, etyk, dl=30.0, ukryj=0.3, seed=0, zgiecie=0.0):
    """Kawalki ~dl metrow trasy (losowo 0,5-1,5 dl), przerwy 1-4 s miedzy nimi, losowy obrot i przesuniecie kazdego."""
    rng = np.random.default_rng(seed)
    n = len(poz); KAW, PRAWDA = [], []
    i = 0; t = 0.0; nr = 1
    etyk_ok = [e for e in etyk if rng.random() >= ukryj]                     # czesc znacznikow "zapomniana"
    while i < n - 10:
        m = int(rng.uniform(0.5, 1.5) * dl / 0.4)
        j = min(n, i + max(m, 10))
        czasy = t + np.arange(j - i) * 0.5                                    # 0,8 m/s
        a = rng.uniform(-np.pi, np.pi); dt = rng.uniform(-30, 30, 2)
        Rm = U._R(a)
        c0 = poz[i:j].mean(0)
        g = np.radians(rng.uniform(-zgiecie, zgiecie)); sk_ = []; poz_ = []
        for q in range(i, j):                                                 # dryf: kazda poza obrocona o g*postep
            Rq = U._R(g * (q - i) / max(1, j - i - 1)); o_ = poz[q]
            sk_.append((skany[q] - o_) @ Rq.T + (poz[q] - poz[i]) @ Rq.T + poz[i]); poz_.append((poz[q] - poz[i]) @ Rq.T + poz[i])
        Pd = np.vstack(sk_); pozd = np.array(poz_); P = np.vstack(skany[i:j])
        loc = lambda X: (X - c0) @ Rm.T + c0 + dt                             # prawda -> uklad kawalka
        sale = [(czasy[k - i], nz) for k, nz in etyk_ok if i <= k < j]
        KAW.append({'nr': nr, 'P': loc(Pd), 'trasa': loc(pozd), 'czasy': czasy, 'od_s': czasy[0], 'do_s': czasy[-1],
                    'sale': sale})
        PRAWDA.append((P, poz[i:j]))
        przerwa = int(rng.integers(2, 8))                                     # 2-8 poz zgubionych (0,8-3,2 m)
        t = czasy[-1] + 0.5 * przerwa; i = j + przerwa; nr += 1
    return KAW, PRAWDA


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pkl'); ap.add_argument('--kondygnacja', type=int, default=None)
    ap.add_argument('--dl', type=float, default=30.0); ap.add_argument('--ukryj', type=float, default=0.3)
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--png')
    ap.add_argument('--bez-sal', action='store_true'); ap.add_argument('--zgiecie', type=float, default=0.0)
    ap.add_argument('--smieci', type=float, default=0.0, help='udzial punktow-mebli (losowe skupiska)')
    ap.add_argument('--usun-cad', type=float, default=0.0, help='udzial scian usunietych z rysunku (przebudowa)')
    a = ap.parse_args()
    D = pickle.load(open(a.pkl, 'rb'))
    idx = [a.kondygnacja] if a.kondygnacja is not None else range(len(D))
    for ki in idx:
        Kd = D[ki]
        if len(Kd['E']) < 4:
            continue
        t0 = time.time()
        poz, skany, etyk, E = przejscie(Kd, a.seed)
        KAW, PR = potnij(poz, skany, etyk, a.dl, 1.0 if a.bez_sal else a.ukryj, a.seed, a.zgiecie)
        sale = {e['nr']: e['xy'] for e in Kd['E']}
        rng = np.random.default_rng(a.seed + 7)
        for k in KAW:                                   # meble/ludzie: odcinki 0,3-1,5 m w losowych miejscach trasy
            n = int(a.smieci * len(k['P']) / (1 - a.smieci)) if a.smieci else 0
            if n:
                srod = k['trasa'][rng.integers(0, len(k['trasa']), n // 40 + 1)] + rng.normal(0, 1.0, (n // 40 + 1, 2))
                kier = rng.uniform(0, np.pi, len(srod)); dl = rng.uniform(0.3, 1.5, len(srod))
                w = rng.integers(0, len(srod), n); t_ = rng.uniform(-0.5, 0.5, n)
                X = srod[w] + (t_ * dl[w])[:, None] * np.c_[np.cos(kier[w]), np.sin(kier[w])] + rng.normal(0, 0.02, (n, 2))
                k['P'] = np.vstack([k['P'], X])
        Sc = np.array(Kd['S'])
        if a.usun_cad:
            Sc = Sc[rng.random(len(Sc)) >= a.usun_cad]
        rz = U.Rzut(Sc, sale)
        print(f"== {Kd['nazwa']}: trasa {len(poz) * 0.4:.0f} m, {len(KAW)} kawalkow, znacznikow {sum(len(k['sale']) for k in KAW)}")
        W = U.uloz(KAW, rz, log=lambda s: None)
        bl = []
        for k, w, (Ptr, _) in zip(KAW, W, PR):
            if 'a' not in w:
                e = None
            else:
                X = U.przeksztalc(k['P'][:len(Ptr)], np.array(w['c']), w['a'], np.array(w['t']))
                e = float(np.median(np.linalg.norm(X - Ptr, axis=1)))
            w['blad_m'] = e; bl.append(e)
            print(f"  K{k['nr']:02d} {w['klasa']:9s} udzial {w['udzial']:.2f} marg {w['margines']:5.2f}  "
                  f"blad {'-' if e is None else f'{e:.2f} m'}")
        ok = [w for w in W if w.get('blad_m') is not None and w['blad_m'] < 0.2]
        zle = [w for w in W if w['klasa'] == 'pewny' and (w.get('blad_m') is None or w['blad_m'] >= 0.2)]
        print(f"  poprawnie (<20 cm): {len(ok)}/{len(W)}; pewne: {sum(w['klasa'] == 'pewny' for w in W)}, "
              f"w tym BLEDNYCH pewnych: {len(zle)}; czas {time.time() - t0:.0f} s")
        if a.png:
            import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(14, 9))
            for s in Kd['S']:
                ax.plot(*np.array(s).T, color='0.6', lw=0.5)
            kol = {'pewny': 'tab:green', 'niepewny': 'tab:orange', 'nieznany': 'tab:red'}
            for k, w in zip(KAW, W):
                if 'a' in w:
                    X = U.przeksztalc(k['P'][::4], np.array(w['c']), w['a'], np.array(w['t']))
                    ax.scatter(*X.T, s=0.3, color=kol[w['klasa']])
                    ax.text(*X.mean(0), f"K{k['nr']}", fontsize=7)
            ax.set_aspect('equal'); ax.set_title(f"{Kd['nazwa']}: zielone pewne, pomaranczowe niepewne")
            fig.savefig(a.png.replace('.png', f'_{ki}.png'), dpi=110, bbox_inches='tight')


if __name__ == '__main__':
    main()

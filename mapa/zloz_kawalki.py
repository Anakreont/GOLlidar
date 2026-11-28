"""Skladanie kawalkow mapy D500 MIEDZY SOBA - bez rysunku CAD (budynki bez dokumentacji).

Kawalki (kawalki.py) leza mniej wiecej tam, gdzie zgadl SLAM: zwykle przesuniete o 0,5-3 m i obrocone o kilka stopni.
Ta sama sala widziana w kilku kawalkach daje kilka przesunietych kopii scian. Algorytm:
 1. Start: najwiekszy kawalek = mapa odniesienia.
 2. Kolejny kawalek: ten, ktory najbardziej naklada sie na juz zlozone (wspolne miejsca).
 3. Szukanie w poblizu obecnego polozenia: obroty +-15 st. co 1 st., przesuniecia +-3 m co 10 cm (FFT),
    ocena = udzial punktow kawalka na scianach odniesienia, LICZONY TYLKO tam, gdzie odniesienie cokolwiek widzialo
    (nowa sala nie jest bledem), mala kara za odejscie od polozenia ze SLAM.
 4. Dociagniecie ICP punkt-prosta (Cauchy 5 cm).
 5. Przyjecie, gdy wspolna czesc >= 15 % punktow, zgodnosc >= 50 % i wynik wyraznie lepszy od drugiego miejsca
    (powtarzalne sale/korytarze); inaczej kawalek zostaje jak byl ("niepewny").
 6. Druga runda: kazdy kawalek jeszcze raz wzgledem wszystkich pozostalych (poprawia wczesne decyzje).
Wynik: _wynik/kawalki_zlozone.dxf/.png/.json (kawalki na warstwach KAWALEK_NN_zlozony/niepewny).
Uzycie: python3 mapa/zloz_kawalki.py <sesja> [dlugosc_odcinka_m, domyslnie 12]
"""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
from scipy.optimize import least_squares

RES = 0.10
OBC = 0.30          # obciecie mapy odleglosci [m]
INL = 0.08
PRZES = 3.0         # zasieg szukania przesuniecia [m]
OBROT = 15          # zasieg szukania obrotu [st.]
WSPOLNE_MIN = 0.15
ZGODNOSC_MIN = 0.40
DZIEL_M = 12.0      # kawalki ze SLAM dzielone na odcinki trasy ~12 m (powolny rozjazd wewnatrz kawalka)
KARA_ODEJSCIA = 0.02  # za kazdy metr od polozenia ze SLAM (w jednostkach udzialu)


def _R(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def rzedz(P, krok=0.05):
    return P[np.unique(np.round(P / krok).astype(np.int64), axis=0, return_index=True)[1]]


class Odniesienie:
    def __init__(self, Q):
        self.Q = rzedz(Q, 0.03)
        self.drzewo = cKDTree(self.Q)
        # normalne odniesienia (do ICP punkt-prosta)
        _, j = self.drzewo.query(self.Q, k=min(10, len(self.Q)))
        X = self.Q[j] - self.Q[j].mean(1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', X, X))
        self.N = v[:, :, 0]
        from uloz_kawalki import kier90
        self.phi = kier90(self.Q, ile=1)[0]

    def siatki(self, o, sh):
        G = np.zeros(sh, bool)
        ij = np.round((self.Q - o) / RES).astype(int)
        m = (ij >= 0).all(1) & (ij[:, 0] < sh[0]) & (ij[:, 1] < sh[1])
        G[ij[m, 0], ij[m, 1]] = True
        koszt = np.minimum(ndimage.distance_transform_edt(~G) * RES, OBC)
        # "tu odniesienie cos widzialo": 1 m wokol jego punktow
        pokr = ndimage.binary_dilation(G, iterations=int(1.0 / RES))
        return koszt, pokr.astype(float)


def oceny(P, ref, c, a0):
    """Najlepsze (a, t) dla kawalka P obroconego wokol c, przesuniecie wzgledem obecnego polozenia."""
    Q = rzedz(P, 0.05)
    if len(Q) > 3000:
        Q = Q[np.random.default_rng(0).choice(len(Q), 3000, replace=False)]
    o = Q.min(0) - PRZES - 2.0
    sh = tuple(((Q.max(0) + PRZES + 2.0 - o) / RES).astype(int) + 1)
    koszt, pokr = ref.siatki(o, sh)
    kp = koszt * pokr
    kand = []
    r = int(PRZES / RES)
    # obroty: +-OBROT wokol obecnego oraz +-4 st. wokol obrotu, ktory ustawia sciany kawalka rownolegle do scian
    # odniesienia (budynki prawie zawsze maja katy proste; SLAM po zgubieniu bywa obrocony o 10-30 st.)
    katy = set(range(-OBROT, OBROT + 1))
    try:
        from uloz_kawalki import kier90
        fr = ref.phi
        for fp in kier90(Q):
            d = np.degrees(fr - fp); d = (d + 45) % 90 - 45
            katy |= set(range(int(round(d)) - 4, int(round(d)) + 5))
    except Exception:
        pass
    for st in sorted(katy):
        a = np.radians(st)
        Qr = (Q - c) @ _R(a).T + c
        lo = Qr.min(0)
        ij = np.round((Qr - lo) / RES).astype(int)
        B = np.zeros(ij.max(0) + 1); np.add.at(B, (ij[:, 0], ij[:, 1]), 1.0)
        i0 = np.round((lo - o) / RES).astype(int) - r    # polozenie B w siatce przy przesunieciu -PRZES
        okno = (slice(i0[0], i0[0] + B.shape[0] + 2 * r), slice(i0[1], i0[1] + B.shape[1] + 2 * r))
        if okno[0].stop > sh[0] or okno[1].stop > sh[1] or i0.min() < 0:
            continue
        num = fftconvolve(kp[okno], B[::-1, ::-1], mode='valid')
        den = fftconvolve(pokr[okno], B[::-1, ::-1], mode='valid')
        den = np.maximum(den, 1e-6)
        wsp = den / len(Q)
        zg = 1.0 - num / den / OBC
        yy, xx = np.mgrid[0:zg.shape[0], 0:zg.shape[1]]
        odej = np.hypot((yy - r) * RES, (xx - r) * RES) + min(abs(st), 15) * 0.03
        W = np.where(wsp >= WSPOLNE_MIN, zg * np.sqrt(np.minimum(wsp, 1.0)) - KARA_ODEJSCIA * odej, -1)   # wiecej wspolnego = pewniej
        i, j = np.unravel_index(np.argmax(W), W.shape)
        t = (np.array([i, j]) - r) * RES
        kand.append((float(W[i, j]), a, t, float(wsp[i, j]), W, st))
    if not kand:
        return None
    kand.sort(key=lambda x: -x[0])
    best = kand[0]
    # drugie miejsce: najlepszy wynik >= 0,7 m albo >= 4 st. od zwyciezcy (powtarzalne sale)
    drugi = -1.0
    for w, a, t, wsp, W, st in kand:
        if abs(st - best[5]) >= 4 and abs(abs(st - best[5]) - 90) >= 4:
            drugi = max(drugi, w)
        else:
            yy, xx = np.mgrid[0:W.shape[0], 0:W.shape[1]]
            ti = np.round(best[2] / RES).astype(int) + int(PRZES / RES)
            daleko = np.hypot(yy - ti[0], xx - ti[1]) * RES >= 0.7
            if daleko.any():
                drugi = max(drugi, float(W[daleko].max()))
    return {'a': best[1] + a0, 't': best[2], 'wynik': best[0], 'wspolne': best[3], 'drugi': drugi}


def icp(P, ref, c, a, t, iters=10):
    Q = rzedz(P, 0.05)
    x = np.array([a, *t])
    for md in np.linspace(0.4, 0.12, iters):
        X = (Q - c) @ _R(x[0]).T + c + x[1:]
        d, j = ref.drzewo.query(X, distance_upper_bound=md)
        m = np.isfinite(d)
        if m.sum() < 30:
            break
        Qm, Pm, Nm = Q[m], ref.Q[j[m]], ref.N[j[m]]
        x = least_squares(lambda y: np.sum(((Qm - c) @ _R(y[0]).T + c + y[1:] - Pm) * Nm, 1), x,
                          loss='cauchy', f_scale=0.05, max_nfev=20).x
    X = (Q - c) @ _R(x[0]).T + c + x[1:]
    d, _ = ref.drzewo.query(X, distance_upper_bound=1.0)
    w = d < 1.0
    zg = float(np.mean(d[w] < INL)) if w.any() else 0.0
    return x[0], x[1:], zg, float(w.mean())


def przeksztalc(P, c, a, t):
    return (np.asarray(P) - c) @ _R(a).T + c + t


def zloz(K, rundy=2, log=print):
    """K: lista kawalkow {'nr','P','trasa'} we wspolnym ukladzie (z SLAM). Zwraca liste {nr,a,t,c,stan,...}."""
    n = len(K)
    stan = [{'nr': k['nr'], 'a': 0.0, 't': np.zeros(2), 'c': k['P'].mean(0), 'stan': 'niepewny'} for k in K]
    poz = lambda i: przeksztalc(K[i]['P'], stan[i]['c'], stan[i]['a'], stan[i]['t'])
    zlozone = [int(np.argmax([len(k['P']) for k in K]))]
    stan[zlozone[0]]['stan'] = 'start'
    reszta = set(range(n)) - set(zlozone)
    proby = {}                 # kawalek -> liczba zlozonych przy ostatniej nieudanej probie (ponowna proba, gdy przybedzie)
    while True:
        # odniesienie TYLKO z kawalkow juz pewnie zlozonych (odrzucone w zlym miejscu psulyby kolejne)
        ref_P = np.vstack([poz(i) for i in zlozone])
        dr = cKDTree(rzedz(ref_P, 0.2))
        kand = [i for i in reszta if proby.get(i, -1) < len(zlozone)]
        if not kand:
            break
        # najpierw najwieksza wspolna czesc z juz zlozonymi; sasiad w czasie dostaje premie (na pewno widzial to samo)
        def wsp(i):
            P_ = rzedz(przeksztalc(K[i]['P'], stan[i]['c'], *_start(stan, zlozone, i)), 0.2)
            w = float(np.mean(dr.query(P_, distance_upper_bound=1.0)[0] < 1.0))
            return w + (0.3 if (i - 1 in zlozone or i + 1 in zlozone) else 0.0)
        i = max(kand, key=wsp)
        stan[i]['a'], stan[i]['t'] = _start(stan, zlozone, i)
        _proba(K, stan, i, Odniesienie(ref_P), log)
        if stan[i]['stan'] == 'zlozony':
            zlozone.append(i); reszta.discard(i)
        else:
            proby[i] = len(zlozone)
    for r in range(1, rundy):
        for i in sorted(range(n), key=lambda i: -len(K[i]['P'])):
            if stan[i]['stan'] == 'start':
                continue
            if stan[i]['stan'] == 'niepewny':
                continue
            ref = Odniesienie(np.vstack([poz(j) for j in range(n) if j != i and stan[j]['stan'] != 'niepewny']))
            _proba(K, stan, i, ref, log, runda=r)
    for s in stan:
        s['t'] = s['t'].tolist(); s['c'] = s['c'].tolist()
    return stan


def _start(stan, zlozone, i):
    """Punkt wyjscia: poprawka najblizszego w czasie zlozonego kawalka (bledy SLAM narastaja wzdluz trasy)."""
    j = min(zlozone, key=lambda j: abs(j - i))
    sn = stan[j]; Rn = _R(sn['a'])
    return sn['a'], (stan[i]['c'] - sn['c']) @ Rn.T + sn['c'] + sn['t'] - stan[i]['c']


def _proba(K, stan, i, ref, log, runda=0):
    s = stan[i]; k = K[i]
    Pc = przeksztalc(k['P'], s['c'], s['a'], s['t'])        # obecne polozenie
    h = oceny(Pc, ref, s['c'] + s['t'], 0.0)
    if h is None:
        log(f"  K{k['nr']:02d}: brak wspolnej czesci"); return
    a, t, zg, wsp = icp(Pc, ref, s['c'] + s['t'], h['a'], h['t'])
    margines = h['wynik'] - h['drugi']
    ok = wsp >= WSPOLNE_MIN and zg >= ZGODNOSC_MIN and margines >= 0.05
    log(f"  {'R' + str(runda) if runda else '  '} K{k['nr']:02d}: wspolne {wsp:.2f} zgodnosc {zg:.2f} margines {margines:.2f} "
        f"przesuniecie {np.hypot(*t):.2f} m obrot {np.degrees(a):+.1f} st. -> {'ZLOZONY' if ok else 'zostaje'}")
    if ok:
        # zlozenie przeksztalcen: najpierw obecne (c, a0, t0), potem obrot a wokol c+t0 i przesuniecie t
        s['a'] = s['a'] + a
        s['t'] = s['t'] + t
        s['stan'] = 'zlozony'; s['zgodnosc'] = round(zg, 3); s['wspolne'] = round(wsp, 3); s['margines'] = round(margines, 3)


def zapisz(kat, K, S, nazwa='kawalki_zlozone'):
    import ezdxf
    wy = os.path.join(kat, '_wynik')
    d = ezdxf.new('R2010'); d.header['$INSUNITS'] = 6; msp = d.modelspace()
    kol = {'start': 5, 'zlozony': 3, 'niepewny': 30}
    for k, s in zip(K, S):
        war = f"KAWALEK_{k['nr']:02d}_{s['stan']}"
        d.layers.add(war, color=kol[s['stan']])
        f = lambda X: przeksztalc(X, np.array(s['c']), s['a'], np.array(s['t']))
        for p in rzedz(f(k['P']), 0.03):
            msp.add_point(tuple(p), dxfattribs={'layer': war})
        for pl in k.get('sciany', []):
            msp.add_lwpolyline(f(np.asarray(pl)).tolist(), dxfattribs={'layer': war})
        msp.add_lwpolyline(f(k['trasa']).tolist(), dxfattribs={'layer': war})
        msp.add_text(f"K{k['nr']:02d}", height=0.25, dxfattribs={'layer': war}).set_placement(tuple(f(k['trasa']).mean(0)))
    d.saveas(os.path.join(wy, nazwa + '.dxf'))
    json.dump(S, open(os.path.join(wy, nazwa + '.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(22, 10))
        cm = plt.get_cmap('tab20')
        for k, s in zip(K, S):
            for x, (A, T) in enumerate([(0.0, np.zeros(2)), (s['a'], np.array(s['t']))]):
                X = przeksztalc(k['P'][::3], np.array(s['c']), A, T)
                ax[x].scatter(*X.T, s=0.2, color=cm(k['nr'] % 20))
                ax[x].text(*X.mean(0), f"K{k['nr']}" + ('' if x == 0 or s['stan'] != 'niepewny' else '?'), fontsize=7)
        ax[0].set_title('przed (jak ze SLAM)'); ax[1].set_title('po zlozeniu (? = niepewny, zostal jak byl)')
        for x in ax:
            x.set_aspect('equal')
        fig.savefig(os.path.join(wy, nazwa + '.png'), dpi=100, bbox_inches='tight'); plt.close(fig)
    except Exception as e:
        print('png:', e)


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import kawalki as KW
    kat = sys.argv[1].rstrip('/')
    dziel = float(sys.argv[2]) if len(sys.argv) > 2 else DZIEL_M
    K, _ = KW.kawalki(kat, dziel_m=dziel)
    print(f'{len(K)} odcinkow po ~{dziel:.0f} m trasy', flush=True)
    S = zloz(K)
    zapisz(kat, K, S)
    ile = {x: sum(s['stan'] == x for s in S) for x in ('start', 'zlozony', 'niepewny')}
    print(json.dumps(ile))


if __name__ == '__main__':
    main()

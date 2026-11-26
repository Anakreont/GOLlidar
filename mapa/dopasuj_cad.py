"""Dopasowanie mapy sesji do rzutu CAD budynku (DXF wgrany przez przegladarke).

    python3 dopasuj_cad.py <katalog_sesji> <katalog_cad> [--klaster N] [--zrodlo nasz|carto_imu|carto]

<katalog_cad>/rzut.dxf + cad.json {"warstwa": ..., "jedn": "auto|m|cm|mm"}.
Rysunek moze zawierac kilka kondygnacji obok/nad soba - dzielony jest na klastry (osobne rzuty) i skan
dopasowywany do kazdego; wygrywa najlepszy (mozna wymusic --klaster).
Obrot skanu nieznany: probowane 0/90/180/270 st. (+-6 st. co 1) po wyprostowaniu do kierunku scian,
przesuniecie z korelacji rastrow (FFT), na koniec odporny ICP do lic.
Wyniki w <sesja>/_wynik/: cad.json (statystyki, obrot, przesuniecie, klastry), cad_porownanie.png,
rzut_cad.dxf (skan w ukladzie i jednostkach rysunku CAD - wstawic w 0,0).
"""
import argparse, json, os, sys
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))
from porownaj_cad import probki, dopasuj

SKALA = {'m': 1.0, 'cm': 0.01, 'mm': 0.001}
KOD_INS = {4: 'mm', 5: 'cm', 6: 'm'}


def warstwy_dxf(plik):
    """[(warstwa, liczba odcinkow)] - do wyboru warstwy scian na stronie."""
    import ezdxf
    from collections import Counter
    d = ezdxf.readfile(plik)
    c = Counter(e.dxf.layer for e in d.modelspace() if e.dxftype() in ('LINE', 'LWPOLYLINE', 'POLYLINE'))
    jedn = KOD_INS.get(int(d.header.get('$INSUNITS', 0) or 0), '')
    return c.most_common(), jedn


def domyslna_warstwa(lista):
    for n, _ in lista:
        l = n.lower()
        if any(s in l for s in ('ścian', 'scian', 'sciana', 'ściana', 'wall', 'mur')):
            return n
    return lista[0][0] if lista else ''


def odcinki(plik, warstwa):
    import ezdxf
    from ezdxf import path as P_
    d = ezdxf.readfile(plik)
    S = []
    for e in d.modelspace():
        if e.dxf.layer != warstwa or e.dxftype() not in ('LINE', 'LWPOLYLINE', 'POLYLINE'):
            continue
        try:
            pts = np.array([(q.x, q.y) for q in P_.make_path(e).flattening(0.5)])
        except Exception:
            continue
        if e.dxftype() == 'LWPOLYLINE' and e.closed and len(pts) > 2:
            pts = np.vstack([pts, pts[:1]])
        for a, b in zip(pts[:-1], pts[1:]):
            S.append((a, b))
    jedn = KOD_INS.get(int(d.header.get('$INSUNITS', 0) or 0), '')
    return np.array(S, float), jedn


def zgadnij_jednostke(S, Q=None, kolejnosc=('cm', 'm', 'mm')):
    """Jednostka rysunku: najwiekszy rzut (klaster) ma miec 5-400 m i miescic skan (skan to czesc kondygnacji).
    $INSUNITS bywa bledny (np. SP 162: naglowek mm, rysunek w cm), wiec tez jest tylko pierwsza propozycja."""
    sq = np.sort(np.ptp(Q, 0)) if Q is not None and len(Q) else np.zeros(2)
    for j in kolejnosc:
        K = klastry(S * SKALA[j])
        if not K: continue
        wym = [np.sort(np.ptp(k.reshape(-1, 2), 0)) for k in K]
        if max(w[1] for w in wym) > 400: continue
        # skan (krotszy i dluzszy bok, w dowolnym obrocie) musi sie miescic w ktoryms rzucie; zapas na wygiecie mapy
        if any(5 <= w[1] and w[0] >= 0.5 * sq[0] and w[1] >= 0.7 * sq[1] for w in wym):
            return j
    return kolejnosc[0]


def klastry(S, przerwa=12.0, min_dl=40.0):
    """Dzieli odcinki (metry) na osobne rzuty: raster 1 m, pogrubienie o 'przerwa', skladowe spojne."""
    if not len(S): return []
    P = S.reshape(-1, 2); o = P.min(0); sh = ((P.max(0) - o) / 1.0).astype(int) + 2
    if sh[0] * sh[1] > 4e7:                  # bardzo rozlegly rysunek - zgrubniej
        return [S]
    G = np.zeros(sh, bool)
    for a, b in S:
        n = int(np.linalg.norm(b - a)) + 2
        q = ((a + (b - a) * np.linspace(0, 1, n)[:, None] - o) / 1.0).astype(int)
        G[q[:, 0], q[:, 1]] = True
    L, n = ndimage.label(ndimage.binary_dilation(G, iterations=int(przerwa)))
    sr = ((S.mean(1) - o) / 1.0).astype(int)
    lab = L[sr[:, 0], sr[:, 1]]
    out = []
    for i in range(1, n + 1):
        Si = S[lab == i]
        if len(Si) and np.linalg.norm(Si[:, 1] - Si[:, 0], axis=1).sum() >= min_dl:
            out.append(Si)
    out.sort(key=lambda s: (-s.reshape(-1, 2)[:, 1].mean(), s.reshape(-1, 2)[:, 0].mean()))   # od gory, od lewej
    return out


def kierunek_scian(Q):
    """Dominujacy kierunek scian skanu modulo 90 st. (z lokalnych normalnych)."""
    t = cKDTree(Q); _, j = t.query(Q[::5], k=8)
    kat = []
    for nb in j:
        X = Q[nb] - Q[nb].mean(0); w, v = np.linalg.eigh(X.T @ X)
        if w[0] < 0.05 * w[1]:
            kat.append(np.degrees(np.arctan2(v[1, 1], v[0, 1])) % 90)
    if not kat: return 0.0
    h, e = np.histogram(kat, bins=90, range=(0, 90))
    h = ndimage.uniform_filter1d(h.astype(float), 3, mode='wrap')
    return float(e[np.argmax(h)] + 0.5)


def skan_sesji(kat, zrodlo='nasz'):
    """Punkty scian skanu w metrach (uklad mapy) + polilinie scian."""
    if zrodlo == 'nasz':
        M = json.load(open(os.path.join(kat, '_wynik', 'mapa.json'), encoding='utf-8'))
        k = max(M['kondygnacje'], key=lambda k: len(k['chmura_cm']))
        Q = np.array(k['chmura_cm'], float).reshape(-1, 2) / 100
        return Q, [np.array(pl, float) for pl in k['sciany']]
    sys.path.insert(0, os.path.join(TU, '..', 'porownanie'))
    from uruchom import wczytaj_mape, punkty_scian
    img, rz, org = wczytaj_mape(os.path.join(kat, '_porownanie', f'k0_{zrodlo}'))
    return punkty_scian(img, rz, org), []


def statystyki(Y, d, C):
    wew = d < 0.20
    dc, _ = cKDTree(Y).query(C)
    return {'punktow': int(len(Y)), 'zgodnych_20cm_proc': round(100 * float(wew.mean()), 1),
            'mediana_mm': round(1000 * float(np.median(d[wew])), 1), 'rms_mm': round(1000 * float(np.sqrt(np.mean(d[wew] ** 2))), 1),
            'p90_mm': round(1000 * float(np.percentile(d[wew], 90)), 1), 'pokrycie_cad_proc': round(100 * float(np.mean(dc < 0.10)), 1),
            'blad_lic_mm': round(1000 * float(np.median(dc[dc < 0.15])), 1) if np.any(dc < 0.15) else None}


def _rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, s], [-s, c]])          # p @ _rot(t) = obrot o +t (konwencja slam.py)


def wczytaj_kf(kat):
    z = np.load(os.path.join(kat, '_wynik', 'kf.npz'))
    gr = np.cumsum(z['dl'])[:-1]
    return z['X'], np.split(z['p'].astype(float), gr), np.split(z['n'].astype(float), gr), z['kt']


def punkty_kf(X, Pl, Nl, maks=60, zasieg=7.0):
    """Punkty kazdego kluczowego skanu (lokalne, przerzedzone) - do dociagania i do wynikowej chmury."""
    out = []
    rng = np.random.default_rng(0)
    for pk, nk in zip(Pl, Nl):
        m = np.isfinite(nk[:, 0]) & (np.hypot(*pk.T) < zasieg)
        idx = np.where(m)[0]
        if len(idx) > maks: idx = np.sort(rng.choice(idx, maks, replace=False))
        out.append(pk[idx])
    return out


def chmura_pozycji(Y, Lp):
    return [pk @ _rot(y[2]) + y[:2] for y, pk in zip(Y, Lp)]


def odcinki_trasy(Y, luka, L):
    """Numer odcinka dla kazdego kluczowego skanu: nowy co L metrow trasy i zawsze po przerwie w sledzeniu."""
    s_ = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Y[:, :2], axis=0).T))])
    ch = np.zeros(len(Y), int); c = 0; s0 = 0.0
    for i in range(1, len(Y)):
        if luka[i] or s_[i] - s0 >= L:
            c += 1; s0 = s_[i]
        ch[i] = c
    return ch


def zgrubnie(Y, Lp, Si, luka, log=print, kom=0.1, okno=6.0, katy=np.arange(-40, 40.1, 2.0)):
    """Kawalki miedzy przerwami, ktore slabo leza na rzucie, szukane zgrubnie: obrot +-40 st. wokol srodka kawalka
    i przesuniecie +-6 m (korelacja rastra kawalka z rozmytymi scianami CAD). Zmienia Y w miejscu."""
    from scipy.signal import fftconvolve
    C, _ = probki(Si, krok=kom / 2)
    o = C.min(0) - okno - 2; sh = ((C.max(0) + okno + 2 - o) / kom).astype(int) + 1
    G = np.zeros(sh); ij = ((C - o) / kom).astype(int); G[ij[:, 0], ij[:, 1]] = 1
    G = ndimage.gaussian_filter(G, 1.5); G /= G.max()
    tree = cKDTree(C)
    gr = np.concatenate([[0], np.where(luka)[0], [len(Y)]])
    poprawione = 0
    for a, b in zip(gr[:-1], gr[1:]):
        if b - a < 5: continue
        Q = np.vstack([pk @ _rot(y[2]) + y[:2] for y, pk in zip(Y[a:b], Lp[a:b])])
        if len(Q) < 200: continue
        d, _ = tree.query(Q); przed = float(np.mean(d < 0.25))
        if przed > 0.6: continue
        c = Q.mean(0); best = (przed, 0.0, np.zeros(2))
        for kat in katy:
            t = np.radians(kat); R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            Qr = (Q - c) @ R.T + c
            m0 = Qr.min(0) - okno; kq = ((Qr - m0) / kom).astype(int); shq = kq.max(0) + 1 + int(okno / kom)
            Qg = np.zeros(shq); np.add.at(Qg, (kq[:, 0], kq[:, 1]), 1); Qg = np.minimum(Qg, 2)
            # wycinek rzutu wokol kawalka (+- okno)
            i0 = np.clip(((m0 - o) / kom).astype(int), 0, None)
            Gw = G[i0[0]: i0[0] + shq[0] + int(okno / kom), i0[1]: i0[1] + shq[1] + int(okno / kom)]
            if min(Gw.shape) < 3: continue
            K = fftconvolve(Gw, Qg[::-1, ::-1], mode='valid')
            if not K.size: continue
            ii, jj = np.unravel_index(np.argmax(K), K.shape)
            t2 = o + (i0 + [ii, jj]) * kom - m0
            dd, _ = tree.query(Qr[::3] + t2); sc = float(np.mean(dd < 0.25))
            if sc > best[0]: best = (sc, t, t2)
        if best[0] > max(1.3 * przed, przed + 0.1):
            t, t2 = best[1], best[2]; R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            for i in range(a, b):
                Y[i, :2] = (Y[i, :2] - c) @ R.T + c + t2; Y[i, 2] += t
            poprawione += 1
            log(f'  zgrubnie: kawalek {a}-{b} ({b - a} skanow): {100 * przed:.0f}% -> {100 * best[0]:.0f}%, obrot {np.degrees(t):+.0f} st., przesuniecie {t2.round(1).tolist()} m', flush=True)
    log(f'  zgrubnie: poprawionych kawalkow {poprawione}', flush=True)


def dociagnij(Y, Lp, Si, kt=None, log=print, dlugosci=(30.0, 12.0, 5.0), luka_s=4.5):
    """Dociaganie mapy do rzutu kawalkami: trasa dzielona na odcinki o zadanej dlugosci (od dlugich do krotkich);
    kazdy odcinek dostaje wlasna poprawke sztywna (dx, dy, obrot wokol srodka odcinka) dopasowujaca go do lic CAD,
    a sasiednie odcinki musza sie stykac (ciaglosc trasy). Usuwa wygiecie korytarzy i wachlarz po przerwach.
    Y: pozycje kluczowych skanow w ukladzie CAD (n x 3), Lp: punkty lokalne. Zwraca nowe Y."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import lsqr
    C, Cn = probki(Si); tree = cKDTree(C)
    Y = Y.copy(); n = len(Y)
    # przerwy w sledzeniu (przekroj, przechyl, zgubiona pozycja): tam odcinek sie konczy i ciaglosc jest slaba,
    # bo wlasnie w tych miejscach mapa doklejala kawalki w zlym miejscu
    luka = np.zeros(n, bool)
    if kt is not None and len(kt) == n:
        luka[1:] = np.diff(np.asarray(kt, float)) > luka_s
    zgrubnie(Y, Lp, Si, luka, log)
    for L in dlugosci:
        for it, dmax in enumerate((1.2, 0.8, 0.5, 0.35, 0.25, 0.2)):
            ch = odcinki_trasy(Y, luka, L); nc = ch.max() + 1
            Q = chmura_pozycji(Y, Lp)
            srodek = np.array([np.vstack([Y[i, :2] for i in np.where(ch == c)[0]]).mean(0) for c in range(nc)])
            rows, cols, vals, b, w = [], [], [], [], []
            r = 0
            for i in range(n):
                q = Q[i]
                if not len(q): continue
                d, j = tree.query(q, distance_upper_bound=dmax)
                m = np.isfinite(d)
                if not m.any(): continue
                q = q[m]; j = j[m]; nn = Cn[j]; c = ch[i]
                v = q - srodek[c]; perp = np.stack([-v[:, 1], v[:, 0]], 1)
                res = np.sum(nn * (q - C[j]), 1)
                k = len(q)
                for col, val in ((3 * c, nn[:, 0]), (3 * c + 1, nn[:, 1]), (3 * c + 2, np.sum(nn * perp, 1))):
                    rows.append(np.arange(r, r + k)); cols.append(np.full(k, col)); vals.append(val / 0.03)
                b.append(-res / 0.03); w.append(np.ones(k)); r += k
            nd = r
            # ciaglosc: kluczowy skan na granicy odcinkow musi wyladowac tak samo w obu odcinkach
            for i in range(n - 1):
                a_, c_ = ch[i], ch[i + 1]
                if a_ == c_: continue
                p = Y[i + 1, :2]
                for os_ in (0, 1):
                    va = p - srodek[a_]; vc = p - srodek[c_]
                    pa = (-va[1], va[0])[os_]; pc = (-vc[1], vc[0])[os_]
                    rows.append(np.array([r] * 4)); cols.append(np.array([3 * a_ + os_, 3 * a_ + 2, 3 * c_ + os_, 3 * c_ + 2]))
                    vals.append(np.array([1, pa, -1, -pc]) / 0.03); b.append(np.zeros(1)); w.append(np.ones(1) * 3); r += 1
                rows.append(np.array([r, r])); cols.append(np.array([3 * a_ + 2, 3 * c_ + 2]))
                vals.append(np.array([1, -1]) / np.radians(0.5)); b.append(np.zeros(1)); w.append(np.ones(1) * 3); r += 1
                if luka[i + 1]:                       # po przerwie ciaglosc tylko slabo
                    for q_ in range(1, 4): w[-q_] = w[-q_] * 0.01
            rows = np.concatenate(rows); cols = np.concatenate(cols); vals = np.concatenate(vals)
            b = np.concatenate(b); w0 = np.concatenate(w)
            A0 = coo_matrix((vals, (rows, cols)), shape=(r, 3 * nc)).tocsr()
            x = np.zeros(3 * nc)
            for _ in range(4):                        # odporne (Huber) - meble i przerobki nie ciagna mapy
                e = A0 @ x - b
                wh = np.where(np.abs(e) <= 1.5, 1.0, 1.5 / np.maximum(np.abs(e), 1e-9)); wh[nd:] = 1.0
                sw = np.sqrt(wh * w0)
                A = A0.multiply(sw[:, None]).tocsr()
                x = lsqr(A, b * sw, damp=1e-3, atol=1e-8, btol=1e-8, iter_lim=2000)[0]
            D = x.reshape(-1, 3)
            for i in range(n):
                c = ch[i]; t = D[c]
                v = Y[i, :2] - srodek[c]
                cs, sn = np.cos(t[2]), np.sin(t[2])
                Y[i, :2] = srodek[c] + [cs * v[0] - sn * v[1], sn * v[0] + cs * v[1]] + t[:2]
                Y[i, 2] += t[2]
        Qa = np.vstack(chmura_pozycji(Y, Lp)); d, _ = tree.query(Qa)
        log(f'  dociaganie odcinkami {L:.0f} m ({nc} odc.): zgodnych {100 * np.mean(d < 0.2):.1f} %, mediana {1000 * np.median(d[d < 0.2]):.0f} mm', flush=True)
    return Y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja'); ap.add_argument('cad')
    ap.add_argument('--klaster', type=int)
    ap.add_argument('--zrodlo', default='nasz')
    ap.add_argument('--pietro', type=int, help='numer kondygnacji sesji (z nazwy), np. -2')
    ap.add_argument('--pietra', help='kondygnacje budynku od najwyzszej, np. 2,1,0,-1,-2 (z nazw sesji)')
    ap.add_argument('--sztywno', action='store_true', help='tylko dopasowanie calosci, bez dociagania odcinkami')
    a = ap.parse_args()
    kat = a.sesja.rstrip('/'); wy = os.path.join(kat, '_wynik'); os.makedirs(wy, exist_ok=True)
    ust = json.load(open(os.path.join(a.cad, 'cad.json'), encoding='utf-8'))
    S0, jedn_plik = odcinki(os.path.join(a.cad, 'rzut.dxf'), ust['warstwa'])
    if not len(S0):
        raise SystemExit(f"brak linii na warstwie {ust['warstwa']}")
    Q, sciany = skan_sesji(kat, a.zrodlo)
    KF = None
    if a.zrodlo == 'nasz' and not a.sztywno and os.path.exists(os.path.join(wy, 'kf.npz')):
        X, Pl, Nl, ktk = wczytaj_kf(kat)
        Lp = punkty_kf(X, Pl, Nl, zasieg=9.0)
        Q = np.vstack(chmura_pozycji(X, Lp)); sciany = []          # uklad swiata kluczowych skanow
        KF = (X, Pl, Nl, Lp, ktk)
    jedn = ust.get('jedn') or 'auto'
    if jedn == 'auto':
        kol = tuple(dict.fromkeys(([jedn_plik] if jedn_plik else []) + ['cm', 'm', 'mm']))
        jedn = zgadnij_jednostke(S0, Q, kol)
    s = SKALA[jedn]; S = S0 * s
    S = S[np.linalg.norm(S[:, 1] - S[:, 0], axis=1) > 0.05]
    K = klastry(S)
    print(f'CAD: {len(S)} odcinkow, jednostka {jedn} (naglowek: {jedn_plik or "brak"}), rzutow (klastrow): {len(K)}, '
          f'skan {np.ptp(Q, 0).round(1).tolist()} m', flush=True)
    if len(Q) > 60000:
        Q = Q[np.random.default_rng(0).choice(len(Q), 60000, replace=False)]
    # wyprostowanie skanu do kierunku scian (dalej dopasuj probuje 0/90/180/270 +-6 st.)
    k0 = kierunek_scian(Q); a0 = np.radians(-k0 if k0 < 45 else 90 - k0)
    R0 = np.array([[np.cos(a0), -np.sin(a0)], [np.sin(a0), np.cos(a0)]])
    Qp = Q @ R0.T
    print(f'skan: {len(Q)} punktow, kierunek scian {k0:.1f} st.', flush=True)
    wyniki = []
    if a.klaster is None and a.pietro is not None and a.pietra:
        lista = [int(x) for x in a.pietra.split(',') if x.strip()]
        if len(lista) == len(K) and a.pietro in lista:     # rzuty w rysunku od gory = kondygnacje od najwyzszej
            yk = [k.reshape(-1, 2)[:, 1].mean() for k in K]
            kol = list(np.argsort(yk)[::-1])
            a.klaster = int(kol[lista.index(a.pietro)])
            print(f'  kondygnacja {a.pietro} z nazwy sesji -> rzut {a.klaster}', flush=True)
    for i, Si in enumerate(K):
        if a.klaster is not None and i != a.klaster: continue
        try:
            R, t, st, Y = dopasuj(Qp, Si, log=lambda *x: None)
        except Exception as e:
            print(f'  klaster {i}: blad {e}', flush=True); continue
        wyniki.append((i, R, t, st, Y))
        print(f"  rzut {i}: zgodnych {st['zgodnych_20cm_proc']} %, mediana {st['mediana_mm']} mm, pokrycie CAD {st['pokrycie_cad_proc']} %", flush=True)
    if not wyniki:
        raise SystemExit('nie udalo sie dopasowac')
    wyniki.sort(key=lambda w: -w[3]['zgodnych_20cm_proc'])
    i, R, t, st, Y = wyniki[0]
    Rc = R @ R0                                   # pelny obrot ze skanu (uklad mapy) do CAD
    niepewne = len(wyniki) > 1 and wyniki[1][3]['zgodnych_20cm_proc'] > 0.85 * st['zgodnych_20cm_proc']
    Si = K[i]; C, _ = probki(Si); d, _ = cKDTree(C).query(Y)
    st_sztywno = st
    if KF is not None:                            # dociaganie odcinkami (usuwa wygiecie i wachlarz)
        X, Pl, Nl, Lp, ktk = KF
        phi = np.arctan2(Rc[1, 0], Rc[0, 0])
        Yp = X.copy(); Yp[:, :2] = X[:, :2] @ Rc.T + t; Yp[:, 2] += phi
        Yp = dociagnij(Yp, Lp, Si, kt=ktk)
        Y = np.vstack(chmura_pozycji(Yp, punkty_kf(X, Pl, Nl, maks=200)))
        d, _ = cKDTree(C).query(Y)
        st = statystyki(Y, d, C)
        # sciany z dociagnietych skanow (lica jak w rzucie mapy)
        try:
            from eksport_dxf import lica
            Lf = punkty_kf(X, Pl, Nl, maks=200)
            Pn, Nn = [], []
            for y, pk, nk in zip(Yp, Pl, Nl):
                m = np.isfinite(nk[:, 0]) & (np.hypot(*pk.T) < 7.0)
                if m.sum() > 200: m[np.where(m)[0][200:]] = False
                nn = nk[m] * np.where(np.sum(nk[m] * pk[m], 1) > 0, -1.0, 1.0)[:, None]
                Pn.append(pk[m] @ _rot(y[2]) + y[:2]); Nn.append(nn @ _rot(y[2]))
            Pn, Nn = np.vstack(Pn), np.vstack(Nn); o = Pn.min(0)
            sciany_cad = [pl + o for pl in lica(Pn - o, Nn)[0]]
        except Exception as e:
            print('  sciany: pominiete -', e, flush=True); sciany_cad = []
    # wynik
    wynik = {'klaster': i, 'jednostka_cad': jedn, 'warstwa': ust['warstwa'], 'zrodlo': a.zrodlo,
             'obrot_st': round(float(np.degrees(np.arctan2(Rc[1, 0], Rc[0, 0]))), 2),
             'przesuniecie_m': [round(float(x), 3) for x in t], 'statystyki': st, 'niepewne': bool(niepewne),
             'klastry': [{'nr': w[0], 'zgodnych_proc': w[3]['zgodnych_20cm_proc'], 'mediana_mm': w[3]['mediana_mm']} for w in wyniki],
             'odchylki_pkt_10cm_proc': round(100 * float(np.mean(d > 0.10)), 1),
             'dociagniete_odcinkami': KF is not None, 'statystyki_sztywno': st_sztywno}
    json.dump(wynik, open(os.path.join(wy, 'cad.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    # obraz
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    P = Si.reshape(-1, 2); ext = np.ptp(P, 0)
    fig, ax = plt.subplots(figsize=(min(24, 4 + ext[0] / 5), min(24, 3 + ext[1] / 5)))
    for sg in Si: ax.plot(sg[:, 0], sg[:, 1], '-', color='0.35', lw=0.8)
    sc = ax.scatter(Y[:, 0], Y[:, 1], c=np.minimum(d, 0.3) * 100, s=0.6, cmap='turbo', vmin=0, vmax=30)
    plt.colorbar(sc, label='odległość skanu od ściany CAD [cm]', shrink=0.6)
    ax.set_aspect('equal'); ax.set_xlim(P[:, 0].min() - 2, P[:, 0].max() + 2); ax.set_ylim(P[:, 1].min() - 2, P[:, 1].max() + 2)
    ax.set_title(f"{os.path.basename(kat)} → rzut {i}" + ('  [NIEPEWNE - sprawdź inny rzut]' if niepewne else '') +
                 f"\nzgodnych (<20 cm) {st['zgodnych_20cm_proc']} %, mediana {st['mediana_mm']} mm, p90 {st['p90_mm']} mm, "
                 f"pokrycie ścian CAD {st['pokrycie_cad_proc']} %", fontsize=10)
    plt.tight_layout(); plt.savefig(os.path.join(wy, 'cad_porownanie.png'), dpi=90); plt.close(fig)
    # DXF w ukladzie i jednostkach rysunku CAD (wstawic w 0,0)
    import ezdxf
    from dxf_wspolne import warstwy
    doc = ezdxf.new('R2010', setup=True)
    doc.header['$INSUNITS'] = {'m': 6, 'cm': 5, 'mm': 4}[jedn]
    warstwy(doc, (('SKAN_SCIANY', 1), ('SKAN_PUNKTY', 8), ('SKAN_ODCHYLKI', 6), ('SKAN_OPIS', 3)))
    msp = doc.modelspace(); f = 1 / s
    if KF is not None:
        for pl in sciany_cad:
            msp.add_lwpolyline([(float(x * f), float(y * f)) for x, y in pl], dxfattribs={'layer': 'SKAN_SCIANY'})
    for pl in sciany:
        q = ((pl @ R0.T) @ R.T + t) * f
        msp.add_lwpolyline([(float(x), float(y)) for x, y in q], dxfattribs={'layer': 'SKAN_SCIANY'})
    for (x, y), dd in zip(Y[::2], d[::2]):
        msp.add_point((float(x * f), float(y * f)), dxfattribs={'layer': 'SKAN_ODCHYLKI' if dd > 0.10 else 'SKAN_PUNKTY'})
    msp.add_text(f"{os.path.basename(kat)}: mediana {st['mediana_mm']} mm, zgodnych {st['zgodnych_20cm_proc']} %",
                 height=0.3 * f, dxfattribs={'layer': 'SKAN_OPIS'}).set_placement((float(P[:, 0].min() * f), float((P[:, 1].max() + 1.0) * f)))
    doc.saveas(os.path.join(wy, 'rzut_cad.dxf'))
    print(json.dumps(wynik, ensure_ascii=False))


if __name__ == '__main__':
    main()

"""Rzut kondygnacji do DXF: punkty z kluczowych skanow -> obrot do kierunku scian -> siatka zajetosci
-> odcinki scian rownolegle do osi (dom/szkola: katy proste) -> DXF (warstwy SCIANY, PUNKTY, TRASA, OPIS).
Uzycie: python3 eksport_dxf.py <katalog_sesji> <wyjscie.dxf> [numery odcinkow do pominiecia, np. 2]"""
import os, sys, numpy as np, ezdxf
from scipy import ndimage
from skimage.morphology import skeletonize, remove_small_objects
from skimage.measure import approximate_polygon
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam, sprawdz as S

KOM = 0.025           # rozmiar komorki siatki [m]
NAZWY = {}
MIN_KF = 3            # komorka musi byc widziana z tylu kluczowych skanow (odsiewa szyby, ludzi, odbicia)
MIN_DL = 0.40         # najkrotszy odcinek sciany [m]
PRZERWA = 0.12        # przerwa w scianie, ktora jeszcze laczymy [m]

def kierunek(q, n):
    """Dominujacy kierunek scian z normalnych (modulo 90 st.)."""
    a = np.arctan2(n[:, 1], n[:, 0]) % (np.pi / 2)
    h, e = np.histogram(a, bins=180, range=(0, np.pi / 2))
    h = np.convolve(np.r_[h[-3:], h, h[:3]], np.ones(7), 'same')[3:-3]
    return (e[np.argmax(h)] + e[1] / 2)

def odcinki(Q, os_):
    """Odcinki scian rownoleglych do osi X (os_=0) albo Y (os_=1) z siatki zajetosci."""
    k = Q.copy() if os_ == 0 else Q[:, ::-1].copy()
    wiersz = np.round(k[:, 1] / KOM).astype(int)
    wyn = []
    for w in np.unique(wiersz):
        xs = np.unique(np.round(k[wiersz == w, 0] / KOM).astype(int))
        if len(xs) < MIN_DL / KOM: continue
        przer = np.where(np.diff(xs) * KOM > PRZERWA)[0]
        for g in np.split(xs, przer + 1):
            if (g[-1] - g[0]) * KOM >= MIN_DL:
                wyn.append([w * KOM, g[0] * KOM, g[-1] * KOM, len(g)])
    # scal sasiednie wiersze (ta sama sciana rozmyta na 2-4 komorki), waga = liczba komorek
    wyn.sort(key=lambda r: (r[0], r[1]))
    scal = []
    for r in wyn:
        for s in scal:
            if abs(s[0] - r[0]) <= 0.08 and r[1] <= s[2] + PRZERWA and r[2] >= s[1] - PRZERWA:
                s[0] = (s[0] * s[3] + r[0] * r[3]) / (s[3] + r[3]); s[1] = min(s[1], r[1]); s[2] = max(s[2], r[2]); s[3] += r[3]
                break
        else:
            scal.append(list(r))
    scal = [s for s in scal if s[2] - s[1] >= MIN_DL and s[3] * KOM >= 0.6 * MIN_DL]
    if os_ == 0: return [((s[1], s[0]), (s[2], s[0])) for s in scal]
    return [((s[0], s[1]), (s[0], s[2])) for s in scal]

def szkielet(Qd, kom=0.03, min_kf_mask=None):
    """Sciany jako polilinie po srodku smugi punktow: siatka -> domkniecie -> szkielet -> sledzenie -> uproszczenie."""
    ij = np.round(Qd / kom).astype(int); ij -= ij.min(0) - 3
    G = np.zeros(ij.max(0) + 4, bool); G[ij[:, 0], ij[:, 1]] = True
    G = ndimage.binary_closing(G, np.ones((3, 3)), iterations=2)
    G = remove_small_objects(G, max_size=40)
    Sk = skeletonize(G)
    sasiedzi = ndimage.convolve(Sk.astype(int), np.ones((3, 3), int), mode='constant') - 1
    wezly = Sk & (sasiedzi != 2)
    odw = np.zeros_like(Sk); linie = []
    kier = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    def sledz(start, nast):
        sc = [start, nast]; odw[nast] = True
        while not wezly[sc[-1]]:
            i, j = sc[-1]; dalej = None
            for di, dj in kier:
                q = (i + di, j + dj)
                if 0 <= q[0] < Sk.shape[0] and 0 <= q[1] < Sk.shape[1] and Sk[q] and q != sc[-2] and not odw[q]:
                    dalej = q; break
            if dalej is None: break
            odw[dalej] = True; sc.append(dalej)
        return sc
    for w in zip(*np.nonzero(wezly)):
        odw[w] = True
        for di, dj in kier:
            q = (w[0] + di, w[1] + dj)
            if 0 <= q[0] < Sk.shape[0] and 0 <= q[1] < Sk.shape[1] and Sk[q] and not odw[q]:
                linie.append(sledz(w, q))
    for p in zip(*np.nonzero(Sk & ~odw)):          # zamkniete petle bez wezlow
        if not odw[p]:
            odw[p] = True
            nb = [(p[0] + a, p[1] + b) for a, b in kier if Sk[p[0] + a, p[1] + b] and not odw[p[0] + a, p[1] + b]]
            if nb: linie.append(sledz(p, nb[0]))
    off = (np.round(Qd / kom).astype(int).min(0) - 3)
    wyn = []
    for l in linie:
        a = (np.array(l) + off) * kom
        if len(a) < 2 or np.linalg.norm(a[-1] - a[0]) < 0.15 and len(a) < 8: continue
        a = approximate_polygon(a, tolerance=0.04)
        # dociaganie do katow prostych (odchylka < 6 st.)
        for k in range(len(a) - 1):
            d = a[k + 1] - a[k]
            if abs(d[1]) < np.tan(np.radians(6)) * abs(d[0]):
                y = (a[k, 1] + a[k + 1, 1]) / 2; a[k, 1] = a[k + 1, 1] = y
            elif abs(d[0]) < np.tan(np.radians(6)) * abs(d[1]):
                x = (a[k, 0] + a[k + 1, 0]) / 2; a[k, 0] = a[k + 1, 0] = x
        if np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)) >= 0.2: wyn.append(a)
    return wyn


def kondygnacja(kf, kt, a, b):
    P, N, ile = [], [], []
    for i, ((Tk, pk, nk), t) in enumerate(zip(kf, kt)):
        if not a <= t <= b: continue
        c, s = np.cos(Tk[2]), np.sin(Tk[2]); R = np.array([[c, s], [-s, c]])
        m = np.isfinite(nk[:, 0]) & (np.hypot(*pk.T) < 7.0)
        P.append(pk[m] @ R + Tk[:2]); N.append(nk[m] @ R); ile.append(np.full(m.sum(), i))
    return np.vstack(P), np.vstack(N), np.concatenate(ile)

def main():
    kat, wy = sys.argv[1], sys.argv[2]
    pomin = {int(x) for x in sys.argv[3:] if x.isdigit()}
    global NAZWY
    NAZWY = {int(x.split('=')[0]): x.split('=', 1)[1] for x in sys.argv[3:] if '=' in x}
    z = slam.wczytaj(kat); t0 = z['t0']
    pozi = list(S.poziomy(S.wysokosci(kat + '/baro.bin')))
    P, kf, info, pion = slam.mapa(kat, znak=-1, segmenty=[(b - t0) / 1e9 for a, b, h in pozi[:-1]])
    kt = [P[np.argmin(np.abs(P[:, 1:4] - Tk).sum(1)), 0] for Tk, _, _ in kf]
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    for n, kol in (('SCIANY', 7), ('PUNKTY', 8), ('TRASA', 1), ('OPIS', 3), ('ZDJECIA', 5), ('PRZEKROJE', 6), ('ZNACZNIKI', 2)): doc.layers.add(n, color=kol)
    zn = []
    if os.path.exists(kat + '/znaczniki.txt'):
        for l in open(kat + '/znaczniki.txt', encoding='utf-8'):
            k = l.rstrip('\n').split('\t')
            zn.append(((int(k[0]) - t0) / 1e9, k[1]))
    nazwy = sys.argv[4:] if False else None
    msp = doc.modelspace(); dx = 0.0
    for nr, (a, b, h) in enumerate(pozi):
        if nr in pomin: continue
        a, b = (a - t0) / 1e9, (b - t0) / 1e9
        Q, Nn, ile = kondygnacja(kf, kt, a, b)
        th = kierunek(Q, Nn); c, s = np.cos(-th), np.sin(-th); R = np.array([[c, s], [-s, c]])
        Qr = Q @ R
        # komorki widziane z >= MIN_KF skanow
        kom = np.round(Qr / KOM).astype(np.int64)
        klucz = kom[:, 0] * 1000003 + kom[:, 1]
        _, odw = np.unique(klucz, return_inverse=True)
        par = np.unique(np.stack([odw, ile], 1), axis=0)
        widz = np.bincount(par[:, 0], minlength=odw.max() + 1)
        Qd = Qr[widz[odw] >= MIN_KF]
        x0, y0 = Qd.min(0)
        Qd = Qd - [x0, y0]
        sc = szkielet(Qd)
        tr = P[(P[:, 0] >= a) & (P[:, 0] <= b), 1:3] @ R - [x0, y0]
        for pl in sc: msp.add_lwpolyline([(x + dx, y) for x, y in pl], dxfattribs={'layer': 'SCIANY'})
        for p in Qd[::4]: msp.add_point((p[0] + dx, p[1]), dxfattribs={'layer': 'PUNKTY'})
        msp.add_lwpolyline([(x + dx, y) for x, y in tr[::5]], dxfattribs={'layer': 'TRASA'})
        def poz(t):
            i = min(np.searchsorted(P[:, 0], t), len(P) - 1)
            return P[i, 1:4]
        def doswiata(xy):
            return np.asarray(xy) @ R - [x0, y0] + [dx, 0]
        for t, opis_z in zn:
            if not a <= t <= b: continue
            T_ = poz(t)
            if opis_z.startswith('FOTO '):
                q = doswiata(T_[:2]); msp.add_circle(q, 0.12, dxfattribs={'layer': 'ZDJECIA'})
                msp.add_text(opis_z[5:].replace('.jpg', ''), height=0.12, dxfattribs={'layer': 'ZDJECIA'}).set_placement((q[0] + 0.15, q[1]))
            elif opis_z.startswith('PRZEKROJ_WYNIK'):
                q = doswiata(T_[:2]); d_ = dict(x.split('=') for x in opis_z.split()[1:])
                tx = f"przekroj {d_['nr']}: " + (f"{float(d_['a']):.2f} x {float(d_['b']):.2f} m" if d_['a'] else 'brak wymiarow')
                msp.add_text(tx, height=0.15, dxfattribs={'layer': 'PRZEKROJE'}).set_placement((q[0] + 0.15, q[1] - 0.25))
            elif opis_z.startswith('PUNKT '):
                d_ = dict(x.split('=') for x in opis_z.split()[1:3]); al = -np.radians(float(d_['kat'])); r_ = float(d_['odl']) / 1000
                c_, s_ = np.cos(T_[2]), np.sin(T_[2])
                pw = T_[:2] + np.array([[c_, -s_], [s_, c_]]) @ [r_ * np.cos(al), r_ * np.sin(al)]
                q = doswiata(pw); msp.add_circle(q, 0.08, dxfattribs={'layer': 'ZNACZNIKI'})
                msp.add_text(' '.join(opis_z.split()[3:]), height=0.15, dxfattribs={'layer': 'ZNACZNIKI'}).set_placement((q[0] + 0.1, q[1]))
        nazwa = NAZWY.get(nr, f'Odcinek {nr}')
        opis = f'{nazwa} ({a:.0f}-{b:.0f} s, poziom baro {h:+.1f} m)'
        msp.add_text(opis, height=0.25, dxfattribs={'layer': 'OPIS'}).set_placement((dx, Qd[:, 1].max() + 0.6))
        szer = Qd[:, 0].max()
        print(f'{opis}: {len(sc)} odcinkow scian, obrys {szer:.2f} x {Qd[:, 1].max():.2f} m, punktow {len(Qd)}')
        dx += szer + 5.0
    doc.saveas(wy)

if __name__ == '__main__':
    main()

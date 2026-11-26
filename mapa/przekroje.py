"""Przekroje pionowe z polozonej glowicy (znaczniki PRZEKROJ_OK): rysunek przekroju z wysokoscia w swietle
i szerokoscia pomieszczenia -> <sesja>/_wynik/przekroje.dxf, przekroj_NN.jpg, przekroje.json.

Zasada: obroty z ok. 2,5 s trzymania glowicy przed znacznikiem PRZEKROJ_OK sumujemy (glowica nieruchoma),
obracamy do glownych kierunkow plaszczyzn, a os pionowa wybieramy jako te, na ktorej po jednej stronie lezy
podloga ok. 0,6-1,35 m od glowicy (polozona tyczka obraca sie wokol dolu plecaka; sprawdzone na skosach poddasza)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import sprawdz as S

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU)
import slam
from eksport_dxf import szkielet

WYS_GLOWICY = (0.6, 1.35)       # glowica polozona: tyczka obraca sie wokol dolu plecaka, glowica ok. 0,95 m nad podloga


def _punkty(z, t0, t1):
    a0, a1, d, tl = z['a0'], z['a1'], z['d'], z['tl']
    m = (tl >= t0) & (tl <= t1)
    if m.sum() < 50:
        return None
    krok = ((a1[m] - a0[m]) % 360) / 11
    ang = np.radians((a0[m][:, None] + krok[:, None] * np.arange(12)) % 360).ravel()
    D = d[m].ravel()
    k = (D > 0.5) & (D < 9)
    return np.stack([D[k] * np.cos(-ang[k]), D[k] * np.sin(-ang[k])], 1)


def _kierunek(p):
    """Glowny kierunek plaszczyzn (modulo 90 st.) z histogramu kierunkow miedzy sasiednimi punktami."""
    dv = np.diff(p, axis=0)
    dl = np.hypot(*dv.T)
    dv = dv[(dl > 0.005) & (dl < 0.15)]
    a = np.arctan2(dv[:, 1], dv[:, 0]) % (np.pi / 2)
    h, e = np.histogram(a, bins=90, range=(0, np.pi / 2))
    h = np.convolve(np.r_[h[-2:], h, h[:2]], np.ones(5), 'same')[2:-2]
    return e[np.argmax(h)] + e[1] / 2


def _plaszczyzny(x):
    """Najsilniejsza plaszczyzna po stronie ujemnej i dodatniej osi (polozenie [m], liczba punktow)."""
    wyn = []
    for znak in (-1, 1):
        v = x[(x * znak) > 0.3] * znak
        if len(v) < 20:
            wyn.append(None); continue
        h, e = np.histogram(v, bins=np.arange(0.3, 9.0, 0.02))
        h = np.convolve(h, np.ones(3), 'same')
        i = np.argmax(h)
        if h[i] < 15:
            wyn.append(None); continue
        sel = v[(v > e[i] - 0.04) & (v < e[i] + 0.06)]
        wyn.append((float(np.median(sel)) * znak, int(h[i])))
    return wyn


def policz(kat, z=None, log=print):
    if z is None:
        z = slam.wczytaj_lub_dekoduj(kat)
    t0 = int(z['t0'])
    zn = []
    p = os.path.join(kat, 'znaczniki.txt')
    if os.path.exists(p):
        for l in open(p, encoding='utf-8'):
            k = l.rstrip('\n').split('\t')
            if len(k) >= 2 and k[0].isdigit() and k[1].startswith('PRZEKROJ_OK'):
                nr = dict(x.split('=') for x in k[1].split()[1:] if '=' in x).get('nr', '?')
                zn.append(((S.popraw_czas(kat, int(k[0])) - t0) / 1e9, nr))
    out = []
    for t, nr in zn:
        P = _punkty(z, t - 2.6, t - 0.1)
        if P is None:
            continue
        th = _kierunek(P)
        c, s = np.cos(-th), np.sin(-th)
        Q = P @ np.array([[c, -s], [s, c]]).T
        kand = []
        for os_ in (0, 1):
            pl = _plaszczyzny(Q[:, os_])
            if None in pl:
                continue
            (a, na), (b, nb) = pl
            for znak, podl, sufit in ((1, a, b), (-1, b, a)):    # podloga po stronie ujemnej albo dodatniej
                hg = abs(podl)
                if WYS_GLOWICY[0] <= hg <= WYS_GLOWICY[1] and 1.8 < abs(b - a) < 6:
                    kand.append((abs(hg - 0.95), os_, znak, podl, sufit))
        if not kand:
            log(f'  przekroj {nr}: nie znaleziono podlogi i sufitu')
            continue
        _, os_, znak, podl, sufit = min(kand)
        # uklad rysunku: Y w gore od podlogi, X w poprzek
        if os_ == 0:
            X, Y = Q[:, 1], Q[:, 0]
        else:
            X, Y = Q[:, 0], Q[:, 1]
        if podl > sufit:
            Y = -Y; podl, sufit = -podl, -sufit
        Y = Y - podl
        wys = sufit - podl
        sc = _plaszczyzny(X)
        szer = (sc[1][0] - sc[0][0]) if None not in sc else None
        x0 = sc[0][0] if sc[0] else X.min()
        X = X - x0
        m = (Y > -0.3) & (Y < wys + 0.4)
        R = np.stack([X[m], Y[m]], 1)
        out.append({'nr': nr, 't': round(t, 1), 'wysokosc': round(float(wys), 3),
                    'szerokosc': round(float(szer), 3) if szer else None, 'glowica_nad_podloga': round(float(-podl if podl < 0 else podl), 2),
                    'punkty': np.round(R[::2], 3).tolist()})
        log(f'  przekroj {nr}: wysokosc {wys:.2f} m' + (f', szerokosc {szer:.2f} m' if szer else ''))
    return out


def zapisz(kat, prz):
    import ezdxf, matplotlib
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    wy = os.path.join(kat, '_wynik'); os.makedirs(wy, exist_ok=True)
    for f in os.listdir(wy):
        if f.startswith('przekroj_') and f.endswith('.jpg'):
            os.remove(os.path.join(wy, f))
    with open(os.path.join(wy, 'przekroje.json'), 'w', encoding='utf-8') as f:
        json.dump([{k: v for k, v in p.items() if k != 'punkty'} for p in prz], f, ensure_ascii=False)
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    for n, kol in (('PRZEKROJ_KONTUR', 7), ('PRZEKROJ_PUNKTY', 8), ('PRZEKROJ_WYMIARY', 1), ('OPIS', 3)):
        doc.layers.add(n, color=kol)
    msp = doc.modelspace(); dx = 0.0
    for p in prz:
        R = np.array(p['punkty'])
        if len(R) < 30:
            continue
        kontur = szkielet(R - R.min(0)) if len(R) > 200 else []
        kontur = [pl + R.min(0) for pl in kontur]
        for pl in kontur:
            msp.add_lwpolyline([(x + dx, y) for x, y in pl], dxfattribs={'layer': 'PRZEKROJ_KONTUR'})
        for x, y in R[::2]:
            msp.add_point((x + dx, y), dxfattribs={'layer': 'PRZEKROJ_PUNKTY'})
        h, w = p['wysokosc'], p['szerokosc']
        xw = (w or R[:, 0].max()) + 0.3
        msp.add_line((dx + xw, 0), (dx + xw, h), dxfattribs={'layer': 'PRZEKROJ_WYMIARY'})
        msp.add_text(f'h = {h:.2f} m', height=0.15, dxfattribs={'layer': 'PRZEKROJ_WYMIARY'}).set_placement((dx + xw + 0.1, h / 2))
        if w:
            msp.add_line((dx, -0.3), (dx + w, -0.3), dxfattribs={'layer': 'PRZEKROJ_WYMIARY'})
            msp.add_text(f'{w:.2f} m', height=0.15, dxfattribs={'layer': 'PRZEKROJ_WYMIARY'}).set_placement((dx + w / 2 - 0.2, -0.55))
        msp.add_text(f"Przekroj {p['nr']} ({p['t']:.0f} s)", height=0.2, dxfattribs={'layer': 'OPIS'}).set_placement((dx, h + 0.5))
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.scatter(R[:, 0], R[:, 1], s=1, c='0.5', lw=0)
        for pl in kontur:
            ax.plot(pl[:, 0], pl[:, 1], 'k-', lw=1.5)
        ax.annotate('', (xw, 0), (xw, h), arrowprops=dict(arrowstyle='<->', color='r'))
        ax.text(xw + 0.08, h / 2, f'{h:.2f} m', color='r', va='center')
        if w:
            ax.annotate('', (0, -0.3), (w, -0.3), arrowprops=dict(arrowstyle='<->', color='r'))
            ax.text(w / 2, -0.5, f'{w:.2f} m', color='r', ha='center')
        ax.set_aspect('equal'); ax.grid(alpha=.3)
        ax.set_title(f"Przekrój {p['nr']}: wysokość w świetle {h:.2f} m" + (f', szerokość {w:.2f} m' if w else ''))
        fig.tight_layout(); fig.savefig(os.path.join(wy, f"przekroj_{int(p['nr']):02d}.jpg"), dpi=110); plt.close(fig)
        dx += xw + 2.0
    doc.saveas(os.path.join(wy, 'przekroje.dxf'))


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    zapisz(k, policz(k))

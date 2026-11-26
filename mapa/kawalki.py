"""Mapa pocieta na kawalki w miejscach przerw w sledzeniu (przekroj, duzy przechyl, PAUZA) - do recznego
sklejania w CAD. Kazdy kawalek jest lokalnie poprawny (krotki odcinek trasy), a bledy powstaja na przerwach.

    python3 kawalki.py <katalog_sesji>

Wymaga _wynik/kf.npz (przetworz.py od WERSJI 6). Wynik:
    _wynik/kawalki.dxf - kazdy kawalek jako osobny BLOK (K01, K02, ...) wstawiony tam, gdzie go polozyl program;
                         w CAD przesuwasz/obracasz caly blok (MOVE/ROTATE), sciany i trasa ida razem.
                         Warstwa wstawienia KAWALEK_NN (kolory na zmiane), w bloku: sciany, punkty, trasa, opis.
    _wynik/kawalki.png - podglad z numerami kawalkow.
Jednostki: metry (przegladarka przelicza na cm/mm przy pobieraniu).
"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import sprawdz as S

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))

LUKA_S = 4.5          # przerwa miedzy kluczowymi skanami, od ktorej zaczyna sie nowy kawalek
MIN_KF = 8            # mniejsze kawalki pomijane (pojedyncze skany po przerwie)


def _rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, s], [-s, c]])


def przekroje_czasy(kat):
    """Czasy PRZEKROJ_START i PAUZA_START (s od poczatku nagrania) - tam tez tniemy."""
    import slam
    z = np.load(os.path.join(kat, '_wynik', 'dek.npz'))
    t0 = int(z['t0']); out = []
    p = os.path.join(kat, 'znaczniki.txt')
    if os.path.exists(p):
        for l in open(p, encoding='utf-8'):
            k = l.split('\t')
            if len(k) >= 2 and k[0].isdigit() and k[1].startswith(('PRZEKROJ_START', 'PAUZA_START', 'AUTO_PRZECHYL_START')):
                out.append((S.popraw_czas(kat, int(k[0])) - t0) / 1e9)
    return np.array(sorted(out)), t0


def podziel(kt, ciecia):
    """Numer kawalka dla kazdego kluczowego skanu."""
    nr = np.zeros(len(kt), int); c = 0
    for i in range(1, len(kt)):
        if kt[i] - kt[i - 1] > LUKA_S or np.any((ciecia > kt[i - 1]) & (ciecia <= kt[i])):
            c += 1
        nr[i] = c
    return nr


def kawalki(kat):
    from eksport_dxf import lica
    z = np.load(os.path.join(kat, '_wynik', 'kf.npz'))
    X, kt = z['X'], z['kt']
    gr = np.cumsum(z['dl'])[:-1]
    Pl = np.split(z['p'].astype(float), gr); Nl = np.split(z['n'].astype(float), gr)
    try:
        ciecia, t0 = przekroje_czasy(kat)
    except (OSError, KeyError):
        ciecia, t0 = np.array([]), 0
    nr = podziel(kt, ciecia)
    wyn = []
    for c in range(nr.max() + 1):
        idx = np.where(nr == c)[0]
        if len(idx) < MIN_KF: continue
        P, N = [], []
        for i in idx:
            pk, nk = Pl[i], Nl[i]
            m = np.isfinite(nk[:, 0]) & (np.hypot(*pk.T) < 7.0)
            n_ = nk[m] * np.where(np.sum(nk[m] * pk[m], 1) > 0, -1.0, 1.0)[:, None]
            P.append(pk[m] @ _rot(X[i, 2]) + X[i, :2]); N.append(n_ @ _rot(X[i, 2]))
        P, N = np.vstack(P), np.vstack(N)
        if len(P) < 300: continue
        # punkty widziane z min. 2 kluczowych skanow (odsiew szumu), siatka 2,5 cm
        kom = np.round(P / 0.025).astype(np.int64); _, odw, ile = np.unique(kom[:, 0] * 1000003 + kom[:, 1], return_inverse=True, return_counts=True)
        m = ile[odw] >= 2
        P, N = P[m], N[m]
        if len(P) < 200: continue
        o = P.min(0)
        try:
            sciany = [pl + o for pl in lica(P - o, N)[0]]
        except Exception:
            sciany = []
        wyn.append({'nr': len(wyn) + 1, 'kf': idx, 'P': P, 'sciany': sciany, 'trasa': X[idx, :2],
                    'od_s': float(kt[idx[0]]), 'do_s': float(kt[idx[-1]])})
    return wyn, t0


def zapisz(kat, K, t0):
    import ezdxf, matplotlib, time
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from dxf_wspolne import podzialka
    wy = os.path.join(kat, '_wynik')
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    msp = doc.modelspace()
    doc.layers.add('SKALA', color=7); doc.layers.add('OPIS', color=3)
    fig, ax = plt.subplots(figsize=(14, 12))
    kolory = plt.cm.tab20(np.linspace(0, 1, 20))
    opis = []
    for k in K:
        n = f"K{k['nr']:02d}"
        g = lambda s_: time.strftime('%H:%M:%S', time.localtime((t0 / 1e9) + s_)) if t0 else f'{s_:.0f} s'
        c = k['P'].mean(0)
        blk = doc.blocks.new(name=n)
        for pl in k['sciany']:
            blk.add_lwpolyline([(float(x - c[0]), float(y - c[1])) for x, y in pl], dxfattribs={'layer': '0', 'color': 0, 'lineweight': 35})
        for x, y in k['P'][::4]:
            blk.add_point((float(x - c[0]), float(y - c[1])), dxfattribs={'layer': '0', 'color': 0})
        if len(k['trasa']) > 1:
            blk.add_lwpolyline([(float(x - c[0]), float(y - c[1])) for x, y in k['trasa']], dxfattribs={'layer': '0', 'color': 0, 'linetype': 'DASHED'})
        blk.add_text(f"{n} {g(k['od_s'])}-{g(k['do_s'])}", height=0.35, dxfattribs={'layer': '0', 'color': 0}).set_placement((0.0, 0.0))
        war = f'KAWALEK_{k["nr"]:02d}'
        doc.layers.add(war, color=[1, 2, 3, 4, 5, 6, 30, 140, 200, 240][k['nr'] % 10])
        msp.add_blockref(n, (float(c[0]), float(c[1])), dxfattribs={'layer': war})
        kol = kolory[k['nr'] % 20]
        ax.scatter(k['P'][::3, 0], k['P'][::3, 1], s=0.3, color=kol)
        ax.text(c[0], c[1], n, fontsize=9, weight='bold', color='k', ha='center',
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=kol, alpha=0.85))
        opis.append({'kawalek': n, 'od': g(k['od_s']), 'do': g(k['do_s']), 'skanow': int(len(k['kf'])), 'punktow': int(len(k['P']))})
    if K:
        P = np.vstack([k['P'] for k in K]); m0 = P.min(0)
        podzialka(msp, float(m0[0]), float(m0[1]) - 1.5)
        msp.add_text(f'{os.path.basename(kat)}: {len(K)} kawalkow - bloki K01..K{len(K):02d}, przesuwaj/obracaj w CAD (MOVE, ROTATE)',
                     height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((float(m0[0]), float(P[:, 1].max()) + 1.0))
    doc.saveas(os.path.join(wy, 'kawalki.dxf'))
    ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title(f'{os.path.basename(kat)}: {len(K)} kawałków (cięcie na przekrojach i przerwach)')
    fig.tight_layout(); fig.savefig(os.path.join(wy, 'kawalki.png'), dpi=90); plt.close(fig)
    json.dump(opis, open(os.path.join(wy, 'kawalki.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return opis


if __name__ == '__main__':
    kat = sys.argv[1].rstrip('/')
    K, t0 = kawalki(kat)
    o = zapisz(kat, K, t0)
    print(f'{len(o)} kawalkow -> {kat}/_wynik/kawalki.dxf')

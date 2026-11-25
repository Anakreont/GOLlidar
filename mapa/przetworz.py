"""Przetwarza jedna sesje skanera na wyniki dla przegladarki:
    <sesja>/_wynik/mapa.json   - kondygnacje (odcinki z barometru): sciany, chmura, trasa, znaczniki w ukladzie mapy
    <sesja>/_wynik/rzut.dxf    - rzut z nazwami kondygnacji (nazwy.json), bez ukrytych odcinkow
    <sesja>/_wynik/podglad.png - podglad rzutu
Uzycie:  python3 przetworz.py <katalog_sesji> [--tylko-dxf]
"""
import json, os, sys, time
import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))
import slam, sprawdz as S
from eksport_dxf import kierunek, kondygnacja, szkielet, KOM, MIN_KF

WERSJA = 1


def _znaczniki(kat, t0):
    zn = []
    p = os.path.join(kat, 'znaczniki.txt')
    if os.path.exists(p):
        for l in open(p, encoding='utf-8'):
            k = l.rstrip('\n').split('\t')
            if len(k) >= 2 and k[0].isdigit():
                zn.append(((int(k[0]) - t0) / 1e9, k[1], k[2] if len(k) > 2 else ''))
    return zn


def _rodzaj(opis):
    for pref, r in (('FOTO ', 'foto'), ('PRZEKROJ_WYNIK', 'przekroj'), ('PUNKT ', 'punkt'), ('POM ', 'oprawy'),
                    ('GRZ ', 'grzejnik'), ('OKN', 'okno'), ('NOTATKA', 'notatka'), ('IMU_PION0', 'system'),
                    ('PAUZA', 'system'), ('AUTO_', 'system'), ('PRZEKROJ_', 'system')):
        if opis.startswith(pref):
            return r
    return 'notatka'


def przetworz(kat):
    wy = os.path.join(kat, '_wynik'); os.makedirs(wy, exist_ok=True)
    t_start = time.time()
    z = slam.wczytaj_lub_dekoduj(kat)
    t0 = int(z['t0'])
    w = S.wysokosci(os.path.join(kat, 'baro.bin')) if os.path.exists(os.path.join(kat, 'baro.bin')) else []
    pozi = list(S.poziomy(w)) if w else []
    if not pozi:
        pozi = [(t0, t0 + int(z['tl'][-1] * 1e9), 0.0)]
    P, kf, info, pion = slam.mapa(kat, znak=-1, segmenty=[(b - t0) / 1e9 for a, b, h in pozi[:-1]], dane=z)
    kt = [P[np.argmin(np.abs(P[:, 1:4] - Tk).sum(1)), 0] for Tk, _, _ in kf]
    zn = _znaczniki(kat, t0)
    kond = []
    for nr, (a, b, h) in enumerate(pozi):
        a, b = (a - t0) / 1e9, (b - t0) / 1e9
        try:
            Q, Nn, ile = kondygnacja(kf, kt, a, b)
        except ValueError:
            continue
        th = kierunek(Q, Nn); c, s = np.cos(-th), np.sin(-th); R = np.array([[c, s], [-s, c]])
        Qr = Q @ R
        kom = np.round(Qr / KOM).astype(np.int64)
        _, odw = np.unique(kom[:, 0] * 1000003 + kom[:, 1], return_inverse=True)
        par = np.unique(np.stack([odw, ile], 1), axis=0)
        widz = np.bincount(par[:, 0], minlength=odw.max() + 1)
        Qd = Qr[widz[odw] >= MIN_KF]
        if len(Qd) < 200:
            continue
        x0, y0 = Qd.min(0)
        Qd = Qd - [x0, y0]
        sciany = szkielet(Qd)
        m = (P[:, 0] >= a) & (P[:, 0] <= b)
        tr = P[m, 1:3] @ R - [x0, y0]

        def poz(t):
            i = min(np.searchsorted(P[:, 0], t), len(P) - 1)
            return P[i, 1:4]

        zm = []
        for t, opis, wys in zn:
            if not a <= t <= b:
                continue
            T_ = poz(t); q = T_[:2] @ R - [x0, y0]
            r = {'t': round(t, 1), 'opis': opis, 'rodzaj': _rodzaj(opis), 'x': round(float(q[0]), 2), 'y': round(float(q[1]), 2),
                 'wys': round(float(wys), 2) if wys not in ('', None) else None}
            if r['rodzaj'] == 'foto':
                r['plik'] = opis.split()[1]
            elif r['rodzaj'] == 'punkt':
                d_ = dict(x.split('=') for x in opis.split()[1:3] if '=' in x)
                try:
                    al = -np.radians(float(d_['kat'])); rr = float(d_['odl']) / 1000
                    c_, s_ = np.cos(T_[2]), np.sin(T_[2])
                    pw = T_[:2] + np.array([[c_, -s_], [s_, c_]]) @ [rr * np.cos(al), rr * np.sin(al)]
                    q = pw @ R - [x0, y0]; r['x'], r['y'] = round(float(q[0]), 2), round(float(q[1]), 2)
                    r['opis'] = 'punkt: ' + ' '.join(opis.split()[3:])
                except (KeyError, ValueError):
                    pass
            elif r['rodzaj'] == 'przekroj':
                d_ = dict(x.split('=') for x in opis.split()[1:] if '=' in x)
                r['opis'] = f"przekrój {d_.get('nr', '')}: " + (
                    f"{float(d_['a']):.2f} × {float(d_['b']):.2f} m" if d_.get('a') else 'brak wymiarów')
            zm.append(r)
        chm = np.round(Qd[::3] * 100).astype(int)
        kond.append({'nr': nr, 'od': round(a, 1), 'do': round(b, 1), 'poziom': round(float(h), 2),
                     'szer': round(float(Qd[:, 0].max()), 2), 'wys': round(float(Qd[:, 1].max()), 2),
                     'sciany': [np.round(pl, 3).tolist() for pl in sciany],
                     'chmura_cm': chm.ravel().tolist(),
                     'trasa': np.round(tr[::3], 2).tolist(), 'znaczniki': zm})
    odrz = {}
    for _, x in info:
        odrz[x] = odrz.get(x, 0) + 1
    wynik = {'wersja': WERSJA, 'sesja': os.path.basename(os.path.normpath(kat)), 'policzono': time.strftime('%Y-%m-%d %H:%M'),
             'czas_liczenia_s': round(time.time() - t_start), 'skanow': int(len(P)), 'odrzucone': odrz,
             'pion': [round(float(v), 2) for v in pion], 'kondygnacje': kond}
    with open(os.path.join(wy, 'mapa.json'), 'w', encoding='utf-8') as f:
        json.dump(wynik, f, ensure_ascii=False, separators=(',', ':'))
    dxf_i_podglad(kat)
    return wynik


def nazwy(kat):
    try:
        with open(os.path.join(kat, '_wynik', 'nazwy.json'), encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def dxf_i_podglad(kat):
    """DXF i PNG z mapa.json - szybkie, bez ponownego liczenia mapy (np. po zmianie nazw kondygnacji)."""
    import ezdxf, matplotlib
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    wy = os.path.join(kat, '_wynik')
    with open(os.path.join(wy, 'mapa.json'), encoding='utf-8') as f:
        M = json.load(f)
    nz = nazwy(kat)
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    for n, kol in (('SCIANY', 7), ('PUNKTY', 8), ('TRASA', 1), ('OPIS', 3), ('ZDJECIA', 5), ('PRZEKROJE', 6), ('ZNACZNIKI', 2)):
        doc.layers.add(n, color=kol)
    msp = doc.modelspace(); dx = 0.0
    kol = {'foto': 'b', 'przekroj': 'm', 'punkt': 'orange', 'notatka': 'g', 'oprawy': 'c', 'grzejnik': 'r', 'okno': 'y'}
    widoczne = [k for k in M['kondygnacje'] if not nz.get(str(k['nr']), {}).get('ukryj')]
    fig, ax = plt.subplots(figsize=(max(8, 6 * len(widoczne)), 7))
    for k in widoczne:
        nazwa = nz.get(str(k['nr']), {}).get('nazwa') or f"Odcinek {k['nr']}"
        for pl in k['sciany']:
            msp.add_lwpolyline([(x + dx, y) for x, y in pl], dxfattribs={'layer': 'SCIANY'})
            ax.plot([x + dx for x, _ in pl], [y for _, y in pl], 'k-', lw=1.2)
        ch = np.array(k['chmura_cm']).reshape(-1, 2) / 100
        for p in ch[::2]:
            msp.add_point((p[0] + dx, p[1]), dxfattribs={'layer': 'PUNKTY'})
        ax.scatter(ch[:, 0] + dx, ch[:, 1], s=0.2, c='0.7', lw=0)
        if k['trasa']:
            msp.add_lwpolyline([(x + dx, y) for x, y in k['trasa']], dxfattribs={'layer': 'TRASA'})
            tr = np.array(k['trasa']); ax.plot(tr[:, 0] + dx, tr[:, 1], 'r-', lw=0.4, alpha=0.5)
        for r in k['znaczniki']:
            if r['rodzaj'] == 'system':
                continue
            war = {'foto': 'ZDJECIA', 'przekroj': 'PRZEKROJE'}.get(r['rodzaj'], 'ZNACZNIKI')
            msp.add_circle((r['x'] + dx, r['y']), 0.1, dxfattribs={'layer': war})
            tx = r.get('plik', r['opis']).replace('.jpg', '')
            msp.add_text(tx[:60], height=0.12, dxfattribs={'layer': war}).set_placement((r['x'] + dx + 0.13, r['y']))
            ax.plot(r['x'] + dx, r['y'], 'o', mfc='none', color=kol.get(r['rodzaj'], 'g'), ms=6)
        opis = f"{nazwa} (poziom baro {k['poziom']:+.1f} m)"
        msp.add_text(opis, height=0.25, dxfattribs={'layer': 'OPIS'}).set_placement((dx, k['wys'] + 0.6))
        ax.text(dx, k['wys'] + 0.4, opis, fontsize=10, color='g')
        dx += k['szer'] + 5.0
    doc.saveas(os.path.join(wy, 'rzut.dxf'))
    ax.set_aspect('equal'); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(os.path.join(wy, 'podglad.png'), dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    if '--tylko-dxf' in sys.argv:
        dxf_i_podglad(k)
    else:
        W = przetworz(k)
        print(f"{W['sesja']}: {len(W['kondygnacje'])} odcinkow, {W['skanow']} skanow, {W['czas_liczenia_s']} s")

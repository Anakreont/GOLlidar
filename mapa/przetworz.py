"""Przetwarza jedna sesje skanera na wyniki dla przegladarki:
    <sesja>/_wynik/mapa.json   - kondygnacje (cala sesja = jedna; podzial wg barometru tylko przy DZIEL_BARO): sciany, chmura, trasa, znaczniki w ukladzie mapy
    <sesja>/_wynik/rzut.dxf    - rzut z nazwami kondygnacji (nazwy.json), bez ukrytych odcinkow
    <sesja>/_wynik/podglad.png - podglad rzutu
Uzycie:  python3 przetworz.py <katalog_sesji> [--tylko-dxf]
"""
import json, os, sys, time
import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))
import slam, sprawdz as S
from eksport_dxf import kierunek, kondygnacja, szkielet, lica, KOM, MIN_KF

WERSJA = 5
DZIEL_BARO = False   # True: dawny podzial sesji na kondygnacje wg barometru
PETLE = False          # zamykanie petli (posegraf.graf) - do dopracowania wag, na razie gorzej niz bez
RZUTUJ = False
ZSZYJ = True         # zszywanie fragmentow po lukach w sledzeniu (posegraf.zszyj)


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
    for pref, r in (('SALA ', 'sala'), ('FOTO ', 'foto'), ('PRZEKROJ_WYNIK', 'przekroj'), ('PUNKT ', 'punkt'), ('POM ', 'oprawy'),
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
    # Barometr NIE dzieli sesji na kondygnacje: Piotr nagrywa kazde pietro osobna sesja, a skoki cisnienia
    # (schody, cieplo w kuchni, wentylacja, drzwi) dzielily jedno pietro na osobne, niepolaczone mapy.
    pozi = list(S.poziomy(w)) if (w and DZIEL_BARO) else []
    if not pozi:
        pozi = [(t0, t0 + int(z['tl'][-1] * 1e9), 0.0)]
    P, kf, info, pion = slam.mapa(kat, znak=-1, segmenty=[(b - t0) / 1e9 for a, b, h in pozi[:-1]], dane=z, rzutuj=RZUTUJ)
    kt = [P[np.argmin(np.abs(P[:, 1:4] - Tk).sum(1)), 0] for Tk, _, _ in kf]
    # katy proste: prostowanie kursu osobno w kazdym odcinku (kondygnacji)
    import posegraf
    kt = np.array(kt); Xorig = np.array([k[0] for k in kf], float)
    if ZSZYJ:                                 # fragmenty po lukach w sledzeniu dopasowane po wspolnych scianach
        Xz = posegraf.zszyj(kf, slam.mapa.frag, slam.mapa.frag_luka)
        kf = [(Xz[i], kf[i][1], kf[i][2]) for i in range(len(kf))]
    X0 = np.array([k[0] for k in kf], float); X = X0.copy()
    granice = [(a - t0) / 1e9 for a, b, h in pozi] + [1e12]
    for s_ in range(len(pozi)):
        idx = np.where((kt >= granice[s_]) & (kt < granice[s_ + 1]))[0]
        if len(idx) > 20:
            sub = [kf[i] for i in idx]
            try:                              # zamykanie petli - wylaczone: na rzucie CAD pogarszalo wynik (21,6 -> 9,3 mm bez petli)
                if not PETLE: raise StopIteration
                Xg, L = posegraf.graf(sub, kt[idx], log=lambda *a: None)
                print(f'  petle: {len(L)}')
                sub = [(Xg[j], sub[j][1], sub[j][2]) for j in range(len(sub))]
            except StopIteration:
                pass
            except Exception as e:
                print('  petle: blad', e)
            X[idx] = posegraf.prostuj(sub)
    P = posegraf.przenies(P, kt, Xorig, X)
    kf = [(X[i], kf[i][1], kf[i][2]) for i in range(len(kf))]
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
        dobre = widz[odw] >= MIN_KF
        Qd = Qr[dobre]; Nd = (Nn @ R)[dobre]
        if len(Qd) < 200:
            continue
        x0, y0 = Qd.min(0)
        Qd = Qd - [x0, y0]
        sciany, typy = lica(Qd, Nd)
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
            v = np.array([np.cos(T_[2]), np.sin(T_[2])]) @ R      # kierunek 0 st. LiDAR-u = przod operatora (cialo jest z tylu)
            r = {'t': round(t, 1), 'opis': opis, 'rodzaj': _rodzaj(opis), 'x': round(float(q[0]), 2), 'y': round(float(q[1]), 2),
                 'kier': round(float(np.degrees(np.arctan2(v[1], v[0]))), 1),
                 'wys': round(float(wys), 2) if wys not in ('', None) else None}
            if r['rodzaj'] == 'sala':
                r['opis'] = opis[5:]
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
                r['nr'] = d_.get('nr', '')
                if d_.get('nr', '').isdigit():
                    r['obraz'] = f"przekroj_{int(d_['nr']):02d}.jpg"
                r['opis'] = f"przekrój {d_.get('nr', '')}: " + (
                    f"{float(d_['a']):.2f} × {float(d_['b']):.2f} m" if d_.get('a') else 'brak wymiarów')
            zm.append(r)
        chm = np.round(Qd[::3] * 100).astype(int)
        kond.append({'nr': nr, 'od': round(a, 1), 'do': round(b, 1), 'poziom': round(float(h), 2),
                     'szer': round(float(Qd[:, 0].max()), 2), 'wys': round(float(Qd[:, 1].max()), 2),
                     'sciany': [np.round(pl, 3).tolist() for pl in sciany], 'typy_scian': typy,
                     'chmura_cm': chm.ravel().tolist(),
                     'trasa': np.round(tr[::3], 2).tolist(), 'znaczniki': zm, '_Qd': Qd})
    odrz = {}
    for _, x in info:
        odrz[x] = odrz.get(x, 0) + 1
    wynik = {'wersja': WERSJA, 'sesja': os.path.basename(os.path.normpath(kat)), 'policzono': time.strftime('%Y-%m-%d %H:%M'),
             'czas_liczenia_s': round(time.time() - t_start), 'skanow': int(len(P)), 'odrzucone': odrz,
             'pion': [round(float(v), 2) for v in pion], 'kondygnacje': kond}
    prz = []
    try:
        import przekroje
        prz = przekroje.policz(kat, z)
        przekroje.zapisz(kat, prz)
    except Exception as e:                 # przekroje sa dodatkiem - blad nie moze zatrzymac mapy
        print('przekroje: blad', e)
    # przekroj na rzucie: linia w poprzek albo wzdluz kierunku operatora - ta, wzdluz ktorej szerokosc z rzutu
    # najlepiej zgadza sie z szerokoscia z przekroju (pojedynczy nieruchomy skan = pomiar bez dryfu)
    for k in wynik['kondygnacje']:
        Qd = k.pop('_Qd')
        for r in k['znaczniki']:
            p = next((p for p in prz if str(p['nr']) == str(r.get('nr')) and r['rodzaj'] == 'przekroj'), None)
            if not p or not p.get('szerokosc'):
                continue
            best = None
            for dk in (90.0,):     # przy polozonej glowicy kierunek 0 st. LD19 patrzy w dol, wiec przekroj lezy wzdluz osi 90/270 st. (w poprzek przodu)
                a_ = np.radians(r['kier'] + dk); u = np.array([np.cos(a_), np.sin(a_)])
                d = (Qd - [r['x'], r['y']]); wz = d @ u; pp = np.abs(d @ [-u[1], u[0]])
                m = pp < 0.06
                plus = wz[m & (wz > 0.15)]; minus = -wz[m & (wz < -0.15)]
                if len(plus) < 3 or len(minus) < 3:
                    continue
                dp, dm = np.percentile(plus, 3), np.percentile(minus, 3)
                szer = dp + dm
                if best is None or abs(szer - p['szerokosc']) < abs(best[0] - p['szerokosc']):
                    best = (szer, dk, u, dp, dm)
            if best and abs(best[0] / p['szerokosc'] - 1) < 0.10:   # wieksza roznica = obrot tyczki przy przekroju albo skosy - linii nie rysujemy
                szer, dk, u, dp, dm = best
                r['linia'] = [[round(r['x'] - u[0] * dm, 2), round(r['y'] - u[1] * dm, 2)], [round(r['x'] + u[0] * dp, 2), round(r['y'] + u[1] * dp, 2)]]
                r['szer_rzut'] = round(float(szer), 3); r['szer_przekroj'] = p['szerokosc']
                r['wysokosc_przekroj'] = p['wysokosc']; r['kier_przekroju'] = 'w poprzek' if dk else 'wzdłuż'
    # wysokosc kondygnacji = mediana wysokosci z przekrojow w jej czasie (do modelu 3D)
    for k in wynik['kondygnacje']:
        h = [p['wysokosc'] for p in prz if k['od'] <= p['t'] <= k['do'] and 1.8 < p['wysokosc'] < 4.5]
        k['wysokosc_m'] = round(float(np.median(h)), 2) if h else None
        k['wysokosc_zrodlo'] = f'{len(h)} przekrojów' if h else 'brak przekrojów - przyjęto 2,70 m'
        for r in k['znaczniki']:
            if r.get('obraz') and not os.path.exists(os.path.join(wy, r['obraz'])):
                r.pop('obraz')
    with open(os.path.join(wy, 'mapa.json'), 'w', encoding='utf-8') as f:
        json.dump(wynik, f, ensure_ascii=False, separators=(',', ':'))
    dxf_i_podglad(kat)
    model3d(kat)
    return wynik


def model3d(kat):
    """Model 3D z rzutu: kazda sciana wyciagnieta na wysokosc kondygnacji (z przekrojow, inaczej 2,70 m),
    kondygnacje jedna nad druga wg barometru. Zapis: model3d.obj i model3d.dxf (3DFACE)."""
    import ezdxf
    wy = os.path.join(kat, '_wynik')
    with open(os.path.join(wy, 'mapa.json'), encoding='utf-8') as f:
        M = json.load(f)
    nz = nazwy(kat)
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    doc.layers.add('SCIANY_3D', color=7); doc.layers.add('PODLOGA_3D', color=8)
    msp = doc.modelspace(); V, F = [], []
    for k in M['kondygnacje']:
        if nz.get(str(k['nr']), {}).get('ukryj'):
            continue
        k = obrocona(k, nz.get(str(k['nr']), {}).get('obrot') or 0)
        h = k.get('wysokosc_m') or 2.7
        z0 = 0.0
        for pl in k['sciany']:
            for (x1, y1), (x2, y2) in zip(pl[:-1], pl[1:]):
                q = [(x1, y1, z0), (x2, y2, z0), (x2, y2, z0 + h), (x1, y1, z0 + h)]
                msp.add_3dface(q, dxfattribs={'layer': 'SCIANY_3D'})
                n0 = len(V); V += q; F.append((n0 + 1, n0 + 2, n0 + 3, n0 + 4))
        msp.add_3dface([(0, 0, z0), (k['szer'], 0, z0), (k['szer'], k['wys'], z0), (0, k['wys'], z0)], dxfattribs={'layer': 'PODLOGA_3D'})
    doc.saveas(os.path.join(wy, 'model3d.dxf'))
    with open(os.path.join(wy, 'model3d.obj'), 'w') as f:
        f.write('# model 3D z pomiaru LiDAR (GOLlidar), metry\n')
        f.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in V)
        f.writelines(f'f {a} {b} {c} {d}\n' for a, b, c, d in F)


def nazwy(kat):
    try:
        with open(os.path.join(kat, '_wynik', 'nazwy.json'), encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def obrocona(k, st):
    """Kopia kondygnacji obrocona o st stopni (przeciwnie do zegara, jak w przegladarce) i przesunieta do (0,0)."""
    if not st:
        return k
    a = np.radians(st); cx, cy = k['szer'] / 2, k['wys'] / 2
    Rm = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    f = lambda xy: (np.asarray(xy, float).reshape(-1, 2) - [cx, cy]) @ Rm.T + [cx, cy]
    ch = f(np.array(k['chmura_cm']).reshape(-1, 2) / 100)
    sc = [f(pl) for pl in k['sciany']]; tr = f(k['trasa']) if k['trasa'] else np.zeros((0, 2))
    zn = []
    for r in k['znaczniki']:
        r2 = dict(r, **dict(zip(('x', 'y'), f([r['x'], r['y']])[0])))
        if r.get('kier') is not None:
            r2['kier'] = r['kier'] + st
        if r.get('linia'):
            r2['linia'] = f(r['linia']).tolist()
        zn.append(r2)
    wsz = np.vstack([ch] + sc) if sc else ch
    m0 = wsz.min(0)
    k2 = dict(k)
    k2['chmura_cm'] = np.round((ch - m0) * 100).astype(int).ravel().tolist()
    k2['sciany'] = [(pl - m0).tolist() for pl in sc]
    k2['trasa'] = (tr - m0).tolist()
    k2['znaczniki'] = [dict(r, x=float(r['x'] - m0[0]), y=float(r['y'] - m0[1]),
                            **({'linia': (np.array(r['linia']) - m0).tolist()} if r.get('linia') else {})) for r in zn]
    mx = wsz.max(0) - m0; k2['szer'], k2['wys'] = float(mx[0]), float(mx[1])
    return k2


def dxf_i_podglad(kat):
    """DXF i PNG z mapa.json - szybkie, bez ponownego liczenia mapy (np. po zmianie nazw kondygnacji)."""
    import ezdxf, matplotlib
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    wy = os.path.join(kat, '_wynik')
    with open(os.path.join(wy, 'mapa.json'), encoding='utf-8') as f:
        M = json.load(f)
    nz = nazwy(kat)
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.M
    for n, kol in (('SCIANY', 7), ('PUNKTY', 8), ('TRASA', 1), ('OPIS', 3), ('ZDJECIA', 5), ('PRZEKROJE', 6), ('ZNACZNIKI', 2), ('POMIESZCZENIA', 4),
                   ('SKALA', 7)):
        doc.layers.add(n, color=kol)
    from dxf_wspolne import podzialka
    msp = doc.modelspace(); dx = 0.0
    kol = {'foto': 'b', 'przekroj': 'm', 'punkt': 'orange', 'notatka': 'g', 'oprawy': 'c', 'grzejnik': 'r', 'okno': 'y'}
    widoczne = [k for k in M['kondygnacje'] if not nz.get(str(k['nr']), {}).get('ukryj')]
    fig, ax = plt.subplots(figsize=(max(8, 6 * len(widoczne)), 7))
    for k in widoczne:
        k = obrocona(k, nz.get(str(k['nr']), {}).get('obrot') or 0)
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
            if r['rodzaj'] == 'sala':                    # nazwa pomieszczenia w miejscu, gdzie operator ja wpisal
                msp.add_text(r['opis'][:40], height=0.22, dxfattribs={'layer': 'POMIESZCZENIA'}).set_placement((r['x'] + dx, r['y']))
                ax.text(r['x'] + dx, r['y'], r['opis'][:40], fontsize=8, color='navy', ha='center')
                continue
            war = {'foto': 'ZDJECIA', 'przekroj': 'PRZEKROJE'}.get(r['rodzaj'], 'ZNACZNIKI')
            msp.add_circle((r['x'] + dx, r['y']), 0.1, dxfattribs={'layer': war})
            if r['rodzaj'] == 'foto' and r.get('kier') is not None:          # kierunek zdjecia (przod operatora)
                a_ = np.radians(r['kier'])
                for da in (-30, 30):
                    b_ = a_ + np.radians(da)
                    msp.add_line((r['x'] + dx, r['y']), (r['x'] + dx + 0.6 * np.cos(b_), r['y'] + 0.6 * np.sin(b_)), dxfattribs={'layer': 'ZDJECIA'})
            if r.get('linia'):
                (xa, ya), (xb, yb) = r['linia']
                msp.add_line((xa + dx, ya), (xb + dx, yb), dxfattribs={'layer': 'PRZEKROJE'})
                ax.plot([xa + dx, xb + dx], [ya, yb], '-', color='m', lw=1)
            tx = r.get('plik', r['opis']).replace('.jpg', '')
            msp.add_text(tx[:60], height=0.12, dxfattribs={'layer': war}).set_placement((r['x'] + dx + 0.13, r['y']))
            ax.plot(r['x'] + dx, r['y'], 'o', mfc='none', color=kol.get(r['rodzaj'], 'g'), ms=6)
        opis = nazwa if not DZIEL_BARO else f"{nazwa} (poziom baro {k['poziom']:+.1f} m)"
        msp.add_text(opis, height=0.25, dxfattribs={'layer': 'OPIS'}).set_placement((dx, k['wys'] + 0.6))
        podzialka(msp, dx, -1.2)
        ax.text(dx, k['wys'] + 0.4, opis, fontsize=10, color='g')
        dx += k['szer'] + 5.0
    doc.saveas(os.path.join(wy, 'rzut.dxf'))
    ax.set_aspect('equal'); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(os.path.join(wy, 'podglad.png'), dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    k = sys.argv[1].rstrip('/')
    if '--tylko-dxf' in sys.argv:
        dxf_i_podglad(k); model3d(k)
    else:
        W = przetworz(k)
        print(f"{W['sesja']}: {len(W['kondygnacje'])} odcinkow, {W['skanow']} skanow, {W['czas_liczenia_s']} s")

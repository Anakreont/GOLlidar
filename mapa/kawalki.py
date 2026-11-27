"""Mapa pocieta na kawalki w miejscach przerw w sledzeniu (przekroj, duzy przechyl, PAUZA) - do recznego
sklejania w CAD. Kazdy kawalek jest lokalnie poprawny (krotki odcinek trasy), a bledy powstaja na przerwach.

    python3 kawalki.py <katalog_sesji>

Wymaga _wynik/kf.npz (przetworz.py od WERSJI 6). Wynik:
    _wynik/kawalki.dxf - kazdy kawalek jako osobny BLOK (K01, K02, ...) wstawiony tam, gdzie go polozyl program;
                         w CAD przesuwasz/obracasz caly blok (MOVE/ROTATE), sciany i trasa ida razem.
                         Warstwa wstawienia KAWALEK_NN (kolory na zmiane), w bloku: sciany, punkty, trasa, opis.
    _wynik/kawalki_osobno.dxf - te same bloki rozlozone obok siebie (bez nakladania) - do mierzenia w obrebie kawalka;
                         warstwa kawalka: KAWALEK_NN godzina_od-godzina_do sale ("Jestem tu").
    W kazdym kawalku sale zmierzone automatycznie (pomieszczenia.py na samym kawalku): obrys kreskowany + nazwa i m2;
    tylko obrysy domkniete scianami co najmniej w 60 % ("(?)" = do sprawdzenia). Lista: kawalki_pomieszczenia.csv.
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


def etykiety_kf(kat):
    """Numer kawalka (jak w kawalki.dxf: K01, K02...; 0 = pominiety) dla kazdego kluczowego skanu z kf.npz,
    oraz {nr: (od_s, do_s)} i t0. Ta sama regula ciecia i pomijania malych kawalkow co w kawalki()."""
    z = np.load(os.path.join(kat, '_wynik', 'kf.npz'))
    kt = z['kt']; gr = np.cumsum(z['dl'])[:-1]
    Pl = np.split(z['p'].astype(float), gr); Nl = np.split(z['n'].astype(float), gr)
    try:
        ciecia, t0 = przekroje_czasy(kat)
    except (OSError, KeyError):
        ciecia, t0 = np.array([]), 0
    if 'fr' in z.files and len(z['fr']) == len(kt):
        fr = z['fr']; nr = np.concatenate([[0], np.cumsum(np.diff(fr) != 0)])
    else:
        nr = podziel(kt, ciecia)
    lab = np.zeros(len(kt), int); czasy = {}; n_ = 0
    for c in range(nr.max() + 1):
        idx = np.where(nr == c)[0]
        if len(idx) < MIN_KF:
            continue
        ile = sum(int((np.isfinite(Nl[i][:, 0]) & (np.hypot(*Pl[i].T) < 7.0)).sum()) for i in idx)
        if ile < 300:
            continue
        n_ += 1; lab[idx] = n_; czasy[n_] = (float(kt[idx[0]]), float(kt[idx[-1]]))
    return lab, czasy, t0


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
    if 'fr' in z.files and len(z['fr']) == len(kt):
        # ciecie tylko tam, gdzie sledzenie sie urwalo i fragment nie zostal doklejony pewnie (nowy fragment
        # w slam.mapa); przekroje i przechyly, przez ktore sledzenie przeszlo, zostaja w jednym kawalku
        fr = z['fr']; nr = np.concatenate([[0], np.cumsum(np.diff(fr) != 0)])
    else:
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
        wyn.append({'nr': len(wyn) + 1, 'N': N, 'kf': idx, 'P': P, 'sciany': sciany, 'trasa': X[idx, :2], 'czasy': kt[idx],
                    'od_s': float(kt[idx[0]]), 'do_s': float(kt[idx[-1]])})
    # obrot calosci do kierunku scian (jak rzut w przegladarce), zeby kawalki lezaly prosto
    if wyn:
        from eksport_dxf import kierunek
        try:
            th = kierunek(np.vstack([k['P'] for k in wyn]), np.vstack([k['N'] for k in wyn]))
            c, s_ = np.cos(-th), np.sin(-th); R = np.array([[c, s_], [-s_, c]])
            for k in wyn:
                k['P'] = k['P'] @ R; k['trasa'] = k['trasa'] @ R; k['sciany'] = [pl @ R for pl in k['sciany']]
        except Exception:
            pass
    return wyn, t0


def sale(kat, t0):
    """Znaczniki "Jestem tu" (SALA <nazwa>): [(czas_s, nazwa)]."""
    wyn = []
    p = os.path.join(kat, 'znaczniki.txt')
    if os.path.exists(p) and t0:
        for l in open(p, encoding='utf-8'):
            k = l.rstrip('\n').split('\t')
            if len(k) >= 2 and k[0].isdigit() and k[1].startswith('SALA'):
                nz = (k[1][4:].strip() or (k[2] if len(k) > 2 else '')).strip()
                wyn.append(((S.popraw_czas(kat, int(k[0])) - t0) / 1e9, nz))
    return wyn


def pomieszczenia_kawalka(k, sl):
    """Podzial jednego kawalka na pomieszczenia (kawalek jest lokalnie poprawny, wiec sale w nim mozna zmierzyc).
    Zwraca pomieszczenia z oceną inna niz 'niepewne' (obrys domkniety scianami co najmniej w 60 %)."""
    import pomieszczenia as PM
    zn = []
    for t, nz in sl:
        if k['od_s'] - 1 <= t <= k['do_s'] + 1 and nz:
            x = np.interp(t, k['czasy'], k['trasa'][:, 0]); y = np.interp(t, k['czasy'], k['trasa'][:, 1])
            zn.append({'rodzaj': 'sala', 'opis': nz, 'x': float(x), 'y': float(y)})
    kk = {'nr': k['nr'], 'chmura_cm': np.round(k['P'] * 100).astype(int).ravel().tolist(),
          'sciany': [np.asarray(pl).tolist() for pl in k['sciany']], 'trasa': np.asarray(k['trasa']).tolist(), 'znaczniki': zn}
    try:
        w = PM.podziel(kk, log=lambda *a: None)
    except Exception as e:
        print(f"K{k['nr']:02d}: pomieszczenia - blad {e}")
        return [], [z['opis'] for z in zn]
    return [p for p in w['pomieszczenia'] if p['ocena'] != 'niepewne'], [z['opis'] for z in zn]


def _nazwa_warstwy(s):
    return ''.join(c if c not in '<>/\\":;?*|=`,' else '_' for c in s)[:120]


def zapisz(kat, K, t0):
    import ezdxf, matplotlib, time
    matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from dxf_wspolne import podzialka
    wy = os.path.join(kat, '_wynik')
    sl = sale(kat, t0)
    g = lambda s_: time.strftime('%H.%M.%S', time.localtime((t0 / 1e9) + s_)) if t0 else f'{s_:.0f}s'
    # dwa rysunki: kawalki tam, gdzie je polozyl program (do sklejania) i rozlozone obok siebie (do mierzenia)
    docs = {}
    for rodz in ('kawalki', 'kawalki_osobno'):
        d_ = ezdxf.new('R2010', setup=True); d_.units = ezdxf.units.M
        d_.layers.add('SKALA', color=7); d_.layers.add('OPIS', color=3)
        docs[rodz] = d_
    fig, ax = plt.subplots(figsize=(14, 12))
    kolory = plt.cm.tab20(np.linspace(0, 1, 20))
    opis = []
    # siatka do rozlozenia: kawalki w wierszach, odstep 3 m
    x_kur, y_kur, wys_wiersza, szer_max = 0.0, 0.0, 0.0, None
    if K:
        P_ = np.vstack([k['P'] for k in K]); szer_max = max(60.0, float(np.ptp(P_[:, 0])))
    for k in K:
        n = f"K{k['nr']:02d}"
        c = k['P'].mean(0)
        pom, nazwy = pomieszczenia_kawalka(k, sl)
        nazwy_u = list(dict.fromkeys(nazwy))
        war = _nazwa_warstwy(f"KAWALEK_{k['nr']:02d} {g(k['od_s'])}-{g(k['do_s'])}" + (' ' + ' '.join(nazwy_u) if nazwy_u else ''))
        for d_ in docs.values():
            blk = d_.blocks.new(name=n)
            for pl in k['sciany']:
                blk.add_lwpolyline([(float(x - c[0]), float(y - c[1])) for x, y in pl], dxfattribs={'layer': '0', 'color': 0, 'lineweight': 35})
            for x, y in k['P'][::4]:
                blk.add_point((float(x - c[0]), float(y - c[1])), dxfattribs={'layer': '0', 'color': 0})
            if len(k['trasa']) > 1:
                blk.add_lwpolyline([(float(x - c[0]), float(y - c[1])) for x, y in k['trasa']], dxfattribs={'layer': '0', 'color': 0, 'linetype': 'DASHED'})
            blk.add_text(f"{n} {g(k['od_s'])}-{g(k['do_s'])}", height=0.35, dxfattribs={'layer': '0', 'color': 0}).set_placement((0.0, 0.0))
            for p in pom:                                    # sale zmierzone w kawalku: obrys + nazwa i powierzchnia
                o = [(float(x - c[0]), float(y - c[1])) for x, y in p['obrys']]
                blk.add_lwpolyline(o, close=True, dxfattribs={'layer': '0', 'color': 0, 'linetype': 'DASHED'})
                tx = f"{p['nazwa'] or 'pom. ' + str(p['nr'])}: " + f"{p['pow_m2']:.2f} m2".replace('.', ',') + ('' if p['ocena'] == 'pewne' else ' (?)')
                blk.add_text(tx, height=0.2, dxfattribs={'layer': '0', 'color': 0}).set_placement(
                    (float(p['srodek'][0] - c[0]), float(p['srodek'][1] - c[1])))
            d_.layers.add(war, color=[1, 2, 3, 4, 5, 6, 30, 140, 200, 240][k['nr'] % 10])
        docs['kawalki'].modelspace().add_blockref(n, (float(c[0]), float(c[1])), dxfattribs={'layer': war})
        # rozlozenie: lewy dolny rog kawalka w (x_kur, y_kur)
        lo, hi = k['P'].min(0), k['P'].max(0); w_, h_ = hi - lo
        if x_kur > 0 and x_kur + w_ > szer_max:
            x_kur, y_kur, wys_wiersza = 0.0, y_kur - wys_wiersza - 3.0, 0.0
        wst = (float(x_kur - lo[0] + c[0]), float(y_kur - h_ - lo[1] + c[1]))
        ms = docs['kawalki_osobno'].modelspace()
        ms.add_blockref(n, wst, dxfattribs={'layer': war})
        ms.add_text(war, height=0.4, dxfattribs={'layer': war}).set_placement((float(x_kur), float(y_kur + 0.4)))
        x_kur += w_ + 3.0; wys_wiersza = max(wys_wiersza, h_ + 1.0)
        kol = kolory[k['nr'] % 20]
        ax.scatter(k['P'][::3, 0], k['P'][::3, 1], s=0.3, color=kol)
        ax.text(c[0], c[1], n, fontsize=9, weight='bold', color='k', ha='center',
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=kol, alpha=0.85))
        opis.append({'kawalek': n, 'warstwa': war, 'od': g(k['od_s']), 'do': g(k['do_s']), 'skanow': int(len(k['kf'])), 'punktow': int(len(k['P'])),
                     'sale': nazwy_u, 'pomieszczenia': [{'nazwa': p['nazwa'], 'pow_m2': p['pow_m2'], 'dl_m': p['dl_m'], 'szer_m': p['szer_m'],
                                                          'ocena': p['ocena']} for p in pom]})
    if K:
        P = np.vstack([k['P'] for k in K]); m0 = P.min(0)
        podzialka(docs['kawalki'].modelspace(), float(m0[0]), float(m0[1]) - 1.5)
        docs['kawalki'].modelspace().add_text(f'{os.path.basename(kat)}: {len(K)} kawalkow - bloki K01..K{len(K):02d}, przesuwaj/obracaj w CAD (MOVE, ROTATE)',
                     height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((float(m0[0]), float(P[:, 1].max()) + 1.0))
        podzialka(docs['kawalki_osobno'].modelspace(), 0.0, 2.0)
        docs['kawalki_osobno'].modelspace().add_text(f'{os.path.basename(kat)}: {len(K)} kawalkow rozlozonych osobno - kazdy na swojej warstwie (nazwa: godziny i sale), '
                     'do mierzenia pomieszczen w obrebie kawalka', height=0.4, dxfattribs={'layer': 'OPIS'}).set_placement((0.0, 3.5))
    for rodz, d_ in docs.items():
        d_.saveas(os.path.join(wy, rodz + '.dxf'))
    ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title(f'{os.path.basename(kat)}: {len(K)} kawałków (cięcie tam, gdzie mapa się urwała)')
    fig.tight_layout(); fig.savefig(os.path.join(wy, 'kawalki.png'), dpi=90); plt.close(fig)
    json.dump(opis, open(os.path.join(wy, 'kawalki.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    with open(os.path.join(wy, 'kawalki_pomieszczenia.csv'), 'w', encoding='utf-8-sig') as f:
        f.write('kawalek;godziny;sala;powierzchnia m2;dlugosc m;szerokosc m;ocena\n')
        for o in opis:
            for p in o['pomieszczenia']:
                lz = lambda v: f'{v:.2f}'.replace('.', ',')
                f.write(f"{o['kawalek']};{o['od']}-{o['do']};{p['nazwa']};{lz(p['pow_m2'])};{lz(p['dl_m'])};{lz(p['szer_m'])};{p['ocena']}\n")
    return opis


if __name__ == '__main__':
    kat = sys.argv[1].rstrip('/')
    K, t0 = kawalki(kat)
    o = zapisz(kat, K, t0)
    print(f'{len(o)} kawalkow, {sum(len(x["pomieszczenia"]) for x in o)} zmierzonych pomieszczen -> {kat}/_wynik/kawalki.dxf, kawalki_osobno.dxf')

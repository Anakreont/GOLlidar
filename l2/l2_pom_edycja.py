#!/usr/bin/env python3
"""Reczne poprawianie pomieszczen L2 (Plan / 3D w przegladarce).

Pomieszczenie = obszar "zalany" od punktu wskazanego przez uzytkownika (ziarno) do scian dochodzacych do sufitu
(te same co dla automatycznego podzialu: pas 1,0-1,9 m nad podloga, bez szaf). Przejscia wezsze niz 2*r_drzwi
(domyslnie 1,1 m) sa domykane automatycznie; wnek drzwiowych nie wlicza sie do powierzchni (zasada Piotra).
Uzytkownik moze dorysowac sciane (kreska), podzielic, polaczyc, usunac, nazwac.

    python l2_pom_edycja.py <wynik_l2> start                       # przejmij pomieszczenia z automatu (gdy brak edycji)
    python l2_pom_edycja.py <wynik_l2> dodaj  --k NR --x X --y Y   # nowe pomieszczenie od punktu
    python l2_pom_edycja.py <wynik_l2> sciana --k NR --x X --y Y --x2 X --y2 Y   # dorysowana sciana / domkniecie drzwi
    python l2_pom_edycja.py <wynik_l2> podziel --k NR --x .. --y .. --x2 .. --y2 ..  # sciana + nowe ziarno po drugiej stronie
    python l2_pom_edycja.py <wynik_l2> polacz --k NR --id A --id2 B
    python l2_pom_edycja.py <wynik_l2> usun   --k NR --id A
    python l2_pom_edycja.py <wynik_l2> usun_sciane --k NR --id I
    python l2_pom_edycja.py <wynik_l2> nazwa  --k NR --id A --nazwa "Sala 12"
    python l2_pom_edycja.py <wynik_l2> drzwi  --r 0.55             # polowa najszerszego domykanego przejscia [m]
    python l2_pom_edycja.py <wynik_l2> reset                       # usun reczne poprawki

Pliki w <wynik_l2>: pomieszczenia_edycja.json (poprawki), pom_dane.npz (pamiec podreczna scian i sufitu),
pomieszczenia_poprawione.json / .csv, rzut_poprawiony.dxf (rzut z poprawionymi pomieszczeniami).
"""
import argparse, json, os, sys, time

import numpy as np
from scipy import ndimage

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TU, '..', 'mapa')); sys.path.insert(0, TU)
KOM = 0.03                 # komorka siatki [m]
R_DRZWI = 0.55             # domykane przejscia do 2*R [m]
R_OBRYS = 1.0              # obrys budynku: domkniecie dziur do 2*R (okna, drzwi zewnetrzne) [m]
MIN_POW = 0.5              # mniejsze kawalki pomijane przy podziale [m2]


# ---------------------------------------------------------------- dane (pamiec podreczna)
def wczytaj(wy, nazwa, dom):
    p = os.path.join(wy, nazwa)
    return json.load(open(p, encoding='utf-8')) if os.path.exists(p) else dom


def zapisz_json(wy, nazwa, obj):
    tmp = os.path.join(wy, nazwa + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, separators=(',', ':') if nazwa.endswith('poprawione.json') else (', ', ': '),
                  indent=None if nazwa.endswith('poprawione.json') else 1)
    os.replace(tmp, os.path.join(wy, nazwa))


def dane(wy, PL):
    """Na kazda kondygnacje: raster scian do sufitu (KOM) i mapa wysokosci pod sufitem (0,10 m). Liczone raz z chmury."""
    npz = os.path.join(wy, 'pom_dane.npz')
    zr = [os.path.join(wy, 'chmura.pcd'), os.path.join(wy, 'plan.json')]
    if os.path.exists(npz) and all(os.path.getmtime(npz) >= os.path.getmtime(p) for p in zr if os.path.exists(p)):
        Z = np.load(npz)
        if any(f.startswith('sc_') for f in Z.files):
            return {int(k[4:]): {x: Z[f'{x}_{k[4:]}'] for x in ('prz', 'o', 'H', 'oh', 'sc')} for k in Z.files if k.startswith('kond')}
    from l2_ciecia import chmura
    from l2_rzut import punkty_scian, mapa_sufitu
    P = chmura(wy).astype(float)
    KD = PL['kondygnacje']; wyn = {}; zap = {}
    for i, k in enumerate(KD):
        podl = k['podl']; gora = KD[i + 1]['podl'] - 0.05 if i + 1 < len(KD) else np.inf
        Pk = P[(P[:, 2] > podl - 0.15) & (P[:, 2] < gora)]
        mn = np.array(k['min']) - 1.0; mx = np.array(k['max']) + 1.0
        Pk = Pk[((Pk[:, :2] >= mn) & (Pk[:, :2] <= mx)).all(1)]
        M = mapa_sufitu(Pk, podl, 0.10)
        try:
            A = punkty_scian(Pk, podl, M, pas_z=(1.0, 1.9))
        except Exception:
            A = np.zeros((0, 2))
        if len(A) < 100:
            A = Pk[(Pk[:, 2] > podl + 1.0) & (Pk[:, 2] < podl + 1.9), :2]
        # rdzen sciany (jak w l2_rzut.pomieszczenia_kondygnacji): szum L2 nie pogrubia scian
        q = np.floor((A - mn) / 0.02).astype(np.int64); sh2 = tuple((q.max(0) + 1).tolist()) if len(q) else (1, 1)
        Hc = np.zeros(sh2, np.int32)
        if len(q):
            np.add.at(Hc, (q[:, 0], q[:, 1]), 1)
        rd = (Hc > 0) & (Hc >= 0.35 * ndimage.maximum_filter(Hc, 9))
        A = (np.argwhere(rd) + 0.5) * 0.02 + mn
        sh = tuple(np.ceil((mx - mn) / KOM).astype(int) + 1)
        prz = np.zeros(sh, bool)
        ij = np.floor((A - mn) / KOM).astype(int); ij = ij[((ij >= 0) & (ij < sh)).all(1)]
        prz[ij[:, 0], ij[:, 1]] = True
        # sciany rzutu (pas pod sufitem) przechodza nad oknami i drzwiami (nadproza) - osobno; do przeszkod tylko na obrysie
        # zewnetrznym (zamykaja okna), wewnatrz nie (drzwi miedzy pomieszczeniami musza zostac otwarte)
        sc = np.zeros(sh, bool)
        for pl in k.get('sciany', []):
            rysuj(sc, mn, pl)
        S, oh, _, _ = M
        H = (S - podl).astype(np.float32)
        wyn[k['nr']] = {'prz': prz, 'o': mn, 'H': H, 'oh': np.asarray(oh), 'sc': sc}
        zap.update({f'kond{k["nr"]}': np.zeros(1), f'prz_{k["nr"]}': prz, f'o_{k["nr"]}': mn, f'H_{k["nr"]}': H, f'oh_{k["nr"]}': np.asarray(oh),
                    f'sc_{k["nr"]}': sc})
    np.savez_compressed(npz, **zap)
    return wyn


def rysuj(G, o, pl, grub=1):
    """Polilinia (metry) na raster G (KOM, poczatek o)."""
    pl = np.asarray(pl, float)
    for a, b in zip(pl[:-1], pl[1:]):
        n = int(np.ceil(np.hypot(*(b - a)) / (KOM / 2))) + 1
        t = np.linspace(0, 1, n)[:, None]
        ij = np.floor((a + (b - a) * t - o) / KOM).astype(int)
        ij = ij[((ij >= 0) & (ij < G.shape)).all(1)]
        G[ij[:, 0], ij[:, 1]] = True
    if grub > 1:
        pass


def domkniecie(X, r_kom):
    """Obrys budynku: dylatacja kolem r_kom komorek, wypelnienie dziur, erozja (przez transformate odleglosci - szybko)."""
    pd = int(np.ceil(r_kom)) + 3                            # zapas: brzeg siatki nie moze "zjadac" domkniecia
    Xp = np.pad(X, pd)
    dil = ndimage.distance_transform_edt(~Xp) <= r_kom
    # wypelnienie PRZED erozja: przerwy w cienkich scianach (okna, drzwi zewn. < 2r) sa zamkniete pasem dylatacji,
    # wnetrze wypelnione, erozja cofa juz tylko zewnetrzny brzeg (2 komorki zapasu za licem zewnetrznym)
    dil = ndimage.binary_fill_holes(dil)
    return (ndimage.distance_transform_edt(dil) > r_kom - 2)[pd:-pd, pd:-pd]


def maski(D, sc_uz, r_drzwi):
    """Podzial wolnej przestrzeni na obszary pomieszczen:
    1) rdzenie = wolna przestrzen dalej niz r_drzwi od scian (waskie przejscia, np. drzwi, rozdzielaja rdzenie),
    2) kazda komorka w zasiegu r od rdzenia nalezy do najblizszego rdzenia (odbudowa do lica),
    3) reszta (zaokraglone narozniki, wneki, przejscia): skladowa stykajaca sie z jednym pomieszczeniem nalezy do niego,
       z dwoma lub wiecej - to przejscie/wneka drzwiowa i nie wchodzi do zadnego (zasada: wnek drzwiowych nie liczymy)."""
    prz = D['prz'].copy(); o = D['o']
    uz = np.zeros(prz.shape, bool)
    for s_ in sc_uz:
        rysuj(uz, o, [s_[:2], s_[2:]])
    uz = ndimage.binary_dilation(uz)
    prz_d = ndimage.binary_dilation(prz)                    # zatyka szczeliny 1 komorki miedzy punktami
    budynek = domkniecie(prz_d | uz, R_OBRYS / KOM)
    if 'sc' in D:                                           # linie scian rzutu przy obrysie zewnetrznym = zamkniete okna
        zew = ndimage.distance_transform_edt(budynek) <= 0.5 / KOM
        prz_d = prz_d | ndimage.binary_dilation(D['sc'] & zew)
    # brzeg obrysu (2 komorki) jako przeszkoda: zamyka okna i drzwi zewnetrzne, ktorych nie objely linie scian rzutu
    prz_d = prz_d | (budynek & (ndimage.distance_transform_edt(budynek) <= 2))
    wolne = budynek & ~prz_d & ~uz
    rk = max(1.0, r_drzwi / KOM)
    dist = ndimage.distance_transform_edt(wolne)
    rdzen = dist > rk
    rl, n = ndimage.label(rdzen)
    lab = np.zeros(prz.shape, np.int32)
    if n:
        d2, idx = ndimage.distance_transform_edt(~rdzen, return_indices=True)
        bl = rl[idx[0], idx[1]]
        odb = wolne & (d2 <= rk + 1)
        lab[odb] = bl[odb]
    reszta = wolne & (lab == 0)
    rr, nr = ndimage.label(reszta)
    drzwi = np.zeros(prz.shape, bool)
    if nr:
        for i_, sl in enumerate(ndimage.find_objects(rr), 1):
            sl2 = tuple(slice(max(0, x.start - 2), x.stop + 2) for x in sl)
            m = rr[sl2] == i_
            sas = np.unique(lab[sl2][ndimage.binary_dilation(m, iterations=2) & ~m])
            sas = sas[sas > 0]
            if len(sas) == 1:
                lab[sl2][m] = sas[0]
            elif len(sas) >= 2:
                drzwi[sl2] |= m
    return {'prz': prz, 'prz_d': prz_d, 'uz': uz, 'drzwi': drzwi, 'budynek': budynek, 'lab': lab, 'o': o}


def do_kom(Mk, x, y, szukaj=0.6):
    """Punkt (m) -> etykieta obszaru; gdy punkt w scianie/drzwiach - najblizsza wolna komorka do szukaj m."""
    o = Mk['o']; lab = Mk['lab']
    i, j = int((x - o[0]) / KOM), int((y - o[1]) / KOM)
    if not (0 <= i < lab.shape[0] and 0 <= j < lab.shape[1]):
        return 0
    if lab[i, j]:
        return int(lab[i, j])
    r = int(szukaj / KOM)
    w = lab[max(0, i - r):i + r + 1, max(0, j - r):j + r + 1]
    ii, jj = np.nonzero(w)
    if not len(ii):
        return 0
    d = (ii + max(0, i - r) - i) ** 2 + (jj + max(0, j - r) - j) ** 2
    m = np.argmin(d)
    return int(w[ii[m], jj[m]])


# ---------------------------------------------------------------- pomieszczenia z ziaren
def kierunek(PLk):
    sx = sy = 0.0
    for pl in PLk.get('sciany', []):
        pl = np.asarray(pl, float)
        d = np.diff(pl, axis=0); L = np.hypot(*d.T); a = 4 * np.arctan2(d[:, 1], d[:, 0])
        sx += (L * np.cos(a)).sum(); sy += (L * np.sin(a)).sum()
    return np.arctan2(sy, sx) / 4 if (sx or sy) else 0.0


def opis_pom(mask, Mk, D, PLk, kat):
    from skimage import measure
    o = Mk['o']
    sl = ndimage.find_objects(mask.astype(int))[0]
    m = ndimage.binary_fill_holes(mask[sl])
    pad = np.pad(m, 1)
    cs = measure.find_contours(pad.astype(float), 0.5)
    c = max(cs, key=len)
    c = measure.approximate_polygon(c, tolerance=1.2)
    xy = (c - 1 + np.array([sl[0].start, sl[1].start]) + 0.5) * KOM + o
    x, y = xy[:, 0], xy[:, 1]
    pow_ = float(abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))) / 2)
    dt = ndimage.distance_transform_edt(m)
    ci = np.unravel_index(np.argmax(dt), dt.shape)
    sr = (np.array(ci) + [sl[0].start, sl[1].start] + 0.5) * KOM + o
    R = np.array([[np.cos(-kat), -np.sin(-kat)], [np.sin(-kat), np.cos(-kat)]]); q = xy @ R.T
    dl, sz = sorted(np.ptp(q, 0))[::-1]
    # wysokosc z mapy sufitu (0,10 m)
    ii, jj = np.nonzero(m); C = (np.c_[ii + sl[0].start, jj + sl[1].start] + 0.5) * KOM + o
    hi = np.clip(((C - D['oh']) / 0.10).astype(int), 0, np.array(D['H'].shape) - 1)
    h = D['H'][hi[:, 0], hi[:, 1]]
    wynik = {'obrys': np.round(xy, 3).tolist(), 'srodek': np.round(sr, 3).tolist(), 'pow_m2': round(pow_, 2),
             'dl_m': round(float(dl), 2), 'szer_m': round(float(sz), 2)}
    if len(h):
        a = pow_ / max(1e-9, m.sum() * KOM * KOM)             # raster -> dokladny obrys
        wynik['wys_m'] = round(float(np.median(h)), 2)
        wynik['kub_m3'] = round(float(np.clip(h, 0, None).sum() * KOM * KOM * a), 1)
        if np.mean(h < PLk['wys'] - 0.3) > 0.08:            # pod skosem - PN-ISO 9836
            A100 = float((h >= 2.2).sum()) * KOM * KOM * a; A50 = float(((h >= 1.4) & (h < 2.2)).sum()) * KOM * KOM * a
            wynik['pow_pn_iso_9836'] = round(A100 + 0.5 * A50, 2)
            wynik['pow_ponizej_1_40'] = round(float((h < 1.4).sum()) * KOM * KOM * a, 2)
            wynik['wys_max_m'] = round(float(np.percentile(h, 98)), 2)
    return wynik


def policz_kond(PLk, D, E):
    """E = edycja kondygnacji {'sciany': [[x0,y0,x1,y1]], 'pom': [{'id','seed','nazwa','grupa'}]} -> lista pomieszczen."""
    Mk = maski(D, E['sciany'], E.get('_r', R_DRZWI))
    kat = kierunek(PLk)
    grupy = {}
    for p in E['pom']:
        grupy.setdefault(p.get('grupa') or p['id'], []).append(p)
    wyn = []; zajete = {}
    for g, ps in grupy.items():
        ety = sorted({e for e in (do_kom(Mk, *p['seed']) for p in ps) if e})
        if not ety:
            wyn.append({'id': ps[0]['id'], 'ids': [p['id'] for p in ps], 'nazwa': ps[0].get('nazwa', ''), 'ocena': 'brak – punkt w ścianie',
                        'seed': ps[0]['seed']}); continue
        mask = np.isin(Mk['lab'], ety)
        if len(ps) > 1:                                     # polaczone: domknij przejscie miedzy czesciami
            mask = ndimage.binary_closing(mask, iterations=int(0.6 / KOM)) & ~Mk['prz_d'] & Mk['budynek']
            mask |= np.isin(Mk['lab'], ety)
        # obrys do lica: oddaj komorke poszerzenia scian
        mask = ndimage.binary_dilation(mask) & ~Mk['prz'] & ~Mk['drzwi'] & ~Mk['uz'] | mask
        # otwarte = pomieszczenie styka sie z obrysem budynku tam, gdzie nie ma sciany (przerwa > ok. 2 m), na >= 0,6 m
        bez_sc = ~ndimage.binary_dilation(Mk['prz_d'], iterations=3)
        styk = ndimage.binary_dilation(mask, iterations=2) & ~Mk['budynek'] & bez_sc
        otw = bool(styk.sum() * KOM / 2 >= 0.6)
        dub = [zajete[e] for e in ety if e in zajete]
        for e in ety:
            zajete.setdefault(e, ps[0]['id'])
        p0 = ps[0]
        r = {'id': p0['id'], 'ids': [p['id'] for p in ps], 'nazwa': p0.get('nazwa', ''), 'seed': p0['seed']}
        r.update(opis_pom(mask, Mk, D, PLk, kat))
        r['ocena'] = 'otwarte – domknij ścianę' if otw else ('to samo co nr ' + ','.join(map(str, dub)) + ' – podziel kreską' if dub else 'ok')
        wyn.append(r)
    wyn.sort(key=lambda r: r['id'])
    for i, r in enumerate(wyn, 1):
        r['nr'] = i
    return wyn, Mk


# ---------------------------------------------------------------- operacje
def nowe_id(E):
    return max([p['id'] for p in E['pom']] + [0]) + 1


def start(PL, ED):
    """Ziarna z automatycznych pomieszczen (srodek, nazwa)."""
    for k in PL['kondygnacje']:
        E = ED['k'].setdefault(str(k['nr']), {'sciany': [], 'pom': []})
        if E['pom']:
            continue
        for p in k.get('pomieszczenia', []):
            if p.get('srodek'):
                E['pom'].append({'id': nowe_id(E), 'seed': [round(p['srodek'][0], 3), round(p['srodek'][1], 3)], 'nazwa': p.get('nazwa') or ''})


def zapisz_wyniki(wy, PL, POP):
    zapisz_json(wy, 'pomieszczenia_poprawione.json', POP)
    with open(os.path.join(wy, 'pomieszczenia_poprawione.csv'), 'w', encoding='utf-8-sig') as f:
        f.write('kondygnacja;nr;nazwa;powierzchnia m2;dl. m;szer. m;wysokosc m;kubatura m3;PN-ISO 9836 m2;ocena\n')
        for k in POP['kondygnacje']:
            for p in k['pomieszczenia']:
                if 'pow_m2' not in p:
                    continue
                f.write(f"{k['nr']};{p['nr']};{p.get('nazwa', '')};{p['pow_m2']:.2f};{p['dl_m']:.2f};{p['szer_m']:.2f};"
                        f"{p.get('wys_m', 0):.2f};{p.get('kub_m3', 0):.1f};{p.get('pow_pn_iso_9836', '')};{p['ocena']}\n".replace('.', ','))
    # rzut z poprawionymi pomieszczeniami: rzut (z przekrojami, jesli sa) bez automatycznych konturow i opisow + poprawione
    import ezdxf
    from ezdxf.enums import TextEntityAlignment as TA
    src = os.path.join(wy, 'rzut_przekroje.dxf') if os.path.exists(os.path.join(wy, 'rzut_przekroje.dxf')) else os.path.join(wy, 'rzut.dxf')
    if not os.path.exists(src):
        return
    d = ezdxf.readfile(src); msp = d.modelspace()
    for war in ('Kontur wewnętrzny', 'ARCH_OPIS POMIESZCZEN', 'ARCH_OPIS WYS POM'):
        for e in list(msp.query(f'*[layer=="{war}"]')):
            msp.delete_entity(e)
    for n, c in (('POM_OBRYS', 3), ('POM_OPIS', 7), ('POM_SCIANY_DORYSOWANE', 1)):
        if n not in d.layers:
            d.layers.add(n, color=c)
    dxk = {k['nr']: k['dx'] for k in PL['kondygnacje']}
    for k in POP['kondygnacje']:
        dx = dxk.get(k['nr'], 0.0)
        for s in k.get('sciany', []):
            msp.add_line((s[0] + dx, s[1]), (s[2] + dx, s[3]), dxfattribs={'layer': 'POM_SCIANY_DORYSOWANE'})
        for p in k['pomieszczenia']:
            if 'obrys' not in p:
                continue
            msp.add_lwpolyline([(x + dx, y) for x, y in p['obrys']], close=True, dxfattribs={'layer': 'POM_OBRYS'})
            x, y = p['srodek'][0] + dx, p['srodek'][1]
            t1 = f"{p['nr']} | {p['nazwa']}" if p.get('nazwa') else str(p['nr'])
            msp.add_text(t1, height=0.15, dxfattribs={'layer': 'POM_OPIS'}).set_placement((x, y + 0.05), align=TA.BOTTOM_CENTER)
            msp.add_text(f"{p['pow_m2']:.2f} m²".replace('.', ','), height=0.15, dxfattribs={'layer': 'POM_OPIS'}).set_placement((x, y - 0.05), align=TA.TOP_CENTER)
            if p.get('wys_m'):
                msp.add_text(f"hśw={p['wys_m']:.2f}m".replace('.', ','), height=0.10, dxfattribs={'layer': 'POM_OPIS'}).set_placement((x, y - 0.32), align=TA.TOP_CENTER)
    d.saveas(os.path.join(wy, 'rzut_poprawiony.dxf'))


def przelicz(wy, PL, ED, tylko=None):
    D = dane(wy, PL)
    POP = wczytaj(wy, 'pomieszczenia_poprawione.json', {'kondygnacje': []})
    stare = {k['nr']: k for k in POP['kondygnacje']}
    out = []
    for k in PL['kondygnacje']:
        E = ED['k'].setdefault(str(k['nr']), {'sciany': [], 'pom': []}); E['_r'] = ED.get('r_drzwi', R_DRZWI)
        if tylko is not None and k['nr'] != tylko and k['nr'] in stare:
            out.append(stare[k['nr']]); continue
        pom, _ = policz_kond(k, D[k['nr']], E)
        E.pop('_r', None)
        out.append({'nr': k['nr'], 'pomieszczenia': pom, 'sciany': E['sciany']})
    for E in ED['k'].values():
        E.pop('_r', None)
    POP = {'kondygnacje': out, 'r_drzwi': ED.get('r_drzwi', R_DRZWI), 'czas': time.strftime('%Y-%m-%d %H:%M')}
    zapisz_wyniki(wy, PL, POP)
    return POP


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('wynik'); ap.add_argument('op', choices=('start', 'dodaj', 'sciana', 'podziel', 'polacz', 'usun', 'usun_sciane', 'nazwa', 'drzwi', 'reset'))
    ap.add_argument('--k', type=int); ap.add_argument('--x', type=float); ap.add_argument('--y', type=float)
    ap.add_argument('--x2', type=float); ap.add_argument('--y2', type=float)
    ap.add_argument('--id', type=int); ap.add_argument('--id2', type=int); ap.add_argument('--nazwa', default=''); ap.add_argument('--r', type=float)
    a = ap.parse_args(); wy = a.wynik
    PL = json.load(open(os.path.join(wy, 'plan.json'), encoding='utf-8'))
    if a.op == 'reset':
        for f in ('pomieszczenia_edycja.json', 'pomieszczenia_poprawione.json', 'pomieszczenia_poprawione.csv', 'rzut_poprawiony.dxf'):
            try:
                os.remove(os.path.join(wy, f))
            except OSError:
                pass
        print(json.dumps({'ok': True})); return
    ED = wczytaj(wy, 'pomieszczenia_edycja.json', {'wersja': 1, 'r_drzwi': R_DRZWI, 'k': {}})
    tylko = a.k
    if a.op == 'start' or not ED['k']:
        start(PL, ED); tylko = None
    E = ED['k'].setdefault(str(a.k), {'sciany': [], 'pom': []}) if a.k is not None else None
    if a.op == 'dodaj':
        E['pom'].append({'id': nowe_id(E), 'seed': [round(a.x, 3), round(a.y, 3)], 'nazwa': a.nazwa})
    elif a.op in ('sciana', 'podziel'):
        if np.hypot(a.x2 - a.x, a.y2 - a.y) < 0.1:
            raise SystemExit('za krotka kreska')
        if a.op == 'podziel':                              # ziarno po drugiej stronie kreski w kazdym przecietym pomieszczeniu
            D = dane(wy, PL)[a.k]; PLk = next(k for k in PL['kondygnacje'] if k['nr'] == a.k)
            E['_r'] = ED.get('r_drzwi', R_DRZWI)
            M0 = maski(D, E['sciany'], E['_r'])
            nowe = E['sciany'] + [[round(a.x, 3), round(a.y, 3), round(a.x2, 3), round(a.y2, 3)]]
            M1 = maski(D, nowe, E['_r']); E.pop('_r')
            for p in list(E['pom']):
                e0 = do_kom(M0, *p['seed'])
                if not e0:
                    continue
                e1 = do_kom(M1, *p['seed'])
                czesci = np.unique(M1['lab'][(M0['lab'] == e0) & (M1['lab'] > 0)])
                for c in czesci:
                    if c == e1 or any(do_kom(M1, *q['seed']) == c for q in E['pom']):
                        continue
                    m = M1['lab'] == c
                    if m.sum() * KOM * KOM < MIN_POW:
                        continue
                    dt = ndimage.distance_transform_edt(m); ci = np.unravel_index(np.argmax(dt), dt.shape)
                    s = (np.array(ci) + 0.5) * KOM + M1['o']
                    E['pom'].append({'id': nowe_id(E), 'seed': [round(float(s[0]), 3), round(float(s[1]), 3)], 'nazwa': ''})
        E['sciany'].append([round(a.x, 3), round(a.y, 3), round(a.x2, 3), round(a.y2, 3)])
    elif a.op == 'polacz':
        grp = lambda i: next((p.get('grupa') or p['id'] for p in E['pom'] if p['id'] == i), None)
        ga, gb = grp(a.id), grp(a.id2)
        if ga is None or gb is None:
            raise SystemExit('nie ma takiego pomieszczenia')
        for p in E['pom']:
            if (p.get('grupa') or p['id']) in (ga, gb):
                p['grupa'] = ga
    elif a.op == 'usun':
        gr = next((p.get('grupa') for p in E['pom'] if p['id'] == a.id), None)
        E['pom'] = [p for p in E['pom'] if p['id'] != a.id and not (gr and p.get('grupa') == gr)]
    elif a.op == 'usun_sciane':
        if 0 <= a.id < len(E['sciany']):
            E['sciany'].pop(a.id)
    elif a.op == 'nazwa':
        for p in E['pom']:
            if p['id'] == a.id:
                p['nazwa'] = a.nazwa[:60]
    elif a.op == 'drzwi':
        ED['r_drzwi'] = float(min(1.0, max(0.0, a.r))); tylko = None
    zapisz_json(wy, 'pomieszczenia_edycja.json', ED)
    POP = przelicz(wy, PL, ED, tylko)
    k_ = next((k for k in POP['kondygnacje'] if k['nr'] == a.k), None) if a.k is not None else None
    print(json.dumps({'ok': True, 'pomieszczen': sum(len(k['pomieszczenia']) for k in POP['kondygnacje']),
                      'kond': k_['pomieszczenia'] if k_ else None}, ensure_ascii=False)[-3000:])


if __name__ == '__main__':
    main()

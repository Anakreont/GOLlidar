"""Kontrola jakosci mapy L2 (Point-LIO) PRZED strojeniem parametrow: czy czas, osie i montaz sa w porzadku.

    python l2_kontrola.py <chmura.pcd> <trasa.csv> <katalog_wyniku> [--log pointlio.log]

Liczy (wynik: kontrola_l2.txt i kontrola_l2.json):
 1. Zamkniecie trasy: odleglosc konca od poczatku (XY, Z) - sens ma, gdy wracasz na miejsce startu.
 2. Postoje (predkosc < 5 cm/s przez >= 5 s): drganie pozycji (std) i dryf w trakcie postoju - powinny byc ~0.
 3. Plaskosc podlogi pod trasa: grubosc warstwy podlogi (p10-p90 odchylek od plaszczyzny) i "falowanie" -
    rozrzut wysokosci podlogi w polach 2 x 2 m wzdluz trasy. Rozwarstwiona/falujaca podloga = zly czas IMU,
    ekstrinsyka albo miekki montaz.
 4. Ostrosc scian: lokalna grubosc scian (PCA w promieniu 30 cm, pas 0,5-2,0 m nad podloga) - mediana i p90.
    Podwojne sciany ("duchy") po obrocie lub powrocie podbijaja p90.
 5. Log Point-LIO: liczba komunikatow o cofnieciu czasu ("loop back", "clear buffer").
Progi sa orientacyjne (punkt startowy z konsultacji 28.11.2026, do potwierdzenia na prawdziwych nagraniach).

Zalecana sekwencja testowa (60-90 s, w jednym pomieszczeniu/korytarzu): 15 s postoju z plecakiem na ziemi,
powolny obrot o 360 st. przed wyrazna sciana, 10-15 m prosto i powrot dokladnie na miejsce startu, 15 s postoju.
"""
import argparse, json, os, sys
import numpy as np
from scipy.spatial import cKDTree

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU)
from l2_rzut import czytaj_pcd, czytaj_trase, kondygnacje

# progi wstepne: szum pojedynczego punktu L2 na licu ~2,5-3,5 cm (przyklad Unitree), wiec "grubosc" (1,4826*MAD)
# ponizej ~4 cm to szum czujnika; podwojna sciana / rozwarstwiona podloga daje wiecej. Do potwierdzenia na nagraniach.
PROGI = {'zamkniecie_xy_cm': 15, 'zamkniecie_z_cm': 5, 'postoj_std_cm': 2, 'podloga_grubosc_cm': 4,
         'podloga_falowanie_cm': 3, 'sciana_p50_cm': 4, 'sciana_p90_cm': 6}


def postoje(T, v_max=0.05, min_s=5.0):
    t, P = T[:, 0], T[:, 1:4]
    v = np.r_[0, np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
    stoi = v < v_max
    wyn, i = [], 0
    while i < len(t):
        if stoi[i]:
            j = i
            while j + 1 < len(t) and stoi[j + 1]:
                j += 1
            if t[j] - t[i] >= min_s:
                Q = P[i:j + 1]
                wyn.append({'od_s': round(float(t[i] - t[0]), 1), 'czas_s': round(float(t[j] - t[i]), 1),
                            'std_cm': round(float(np.linalg.norm(Q.std(0)) * 100), 2),
                            'dryf_cm': round(float(np.linalg.norm(Q[-1] - Q[0]) * 100), 2)})
            i = j + 1
        else:
            i += 1
    return wyn


def podloga(P, T, podl, kom=0.25):
    """Podloga do 3 m od trasy. Pola 25 cm: mediana wysokosci (szum pojedynczych punktow sie usrednia).
    falowanie = rozrzut median pol po odjeciu plaszczyzny (p5-p95); grubosc = odporne odchylenie punktow od mediany
    swojego pola (1,4826*MAD) - dwie warstwy podlogi (rozjechane przejscia) je podbijaja."""
    Q = P[np.abs(P[:, 2] - podl) < 0.12]
    if len(Q) < 500:
        return None
    d, _ = cKDTree(T[:, 1:3]).query(Q[:, :2])
    Q = Q[d < 3.0]
    if len(Q) < 500:
        return None
    # pola przy pionowych przeszkodach (sciany, meble: punkty 0,3-1,0 m nad podloga) i ich sasiedzi - pomijane,
    # bo dolne punkty scian wpadaja w pas podlogi
    W = P[(P[:, 2] > podl + 0.3) & (P[:, 2] < podl + 1.0), :2]
    wij = np.floor(W / kom).astype(np.int64)
    przy = {(a + da, b + db) for a, b in set(map(tuple, wij)) for da in (-1, 0, 1) for db in (-1, 0, 1)}
    ij = np.floor(Q[:, :2] / kom).astype(np.int64)
    mk = np.array([(a, b) not in przy for a, b in ij])
    Q, ij = Q[mk], ij[mk]
    if len(Q) < 500:
        return None
    klucz = ij[:, 0] * 1000003 + ij[:, 1]
    u, inv, n = np.unique(klucz, return_inverse=True, return_counts=True)
    med = np.array([np.median(Q[inv == i, 2]) if n[i] >= 15 else np.nan for i in range(len(u))])
    dobre = np.isfinite(med)
    if dobre.sum() < 10:
        return None
    srodki = np.c_[(u // 1000003) * kom, (u % 1000003) * kom][dobre]
    A = np.c_[srodki, np.ones(len(srodki))]
    k, *_ = np.linalg.lstsq(A, med[dobre], rcond=None)
    r_pol = med[dobre] - A @ k
    r_pkt = Q[:, 2] - med[inv]
    r_pkt = r_pkt[np.isfinite(r_pkt)]
    mad = 1.4826 * np.median(np.abs(r_pkt - np.median(r_pkt)))
    return {'punktow': int(len(Q)), 'pol': int(dobre.sum()), 'grubosc_mad_cm': round(float(mad * 100), 2),
            'falowanie_cm': round(float((np.percentile(r_pol, 95) - np.percentile(r_pol, 5)) * 100), 2),
            'nachylenie_st': round(float(np.degrees(np.arctan(np.hypot(k[0], k[1])))), 3)}


def sciany(P, podl, ile=4000, r=0.30, seed=0):
    """Pionowe powierzchnie (normalna 3D prawie pozioma) 0,5-2,0 m nad podloga: grubosc warstwy punktow
    (1,4826*MAD odleglosci od plaszczyzny w kuli 30 cm). Skosy dachu, podloga, meble o niepionowych scianach - pomijane."""
    Q = P[(P[:, 2] > podl + 0.5) & (P[:, 2] < podl + 2.0)]
    if len(Q) < 1000:
        return None
    drzewo = cKDTree(Q)
    rng = np.random.default_rng(seed)
    gr = []
    for i in rng.choice(len(Q), min(ile, len(Q)), replace=False):
        nb = drzewo.query_ball_point(Q[i], r)
        if len(nb) < 40:
            continue
        X = Q[nb] - Q[nb].mean(0)
        w, V = np.linalg.eigh(X.T @ X / len(nb))
        nrm = V[:, 0]
        if abs(nrm[2]) > 0.15 or w[1] < 4 * w[0]:           # nie pionowa albo nie plaska (naroznik, mebel)
            continue
        dd = X @ nrm
        gr.append(1.4826 * np.median(np.abs(dd - np.median(dd))) * 100)
    if len(gr) < 50:
        return None
    gr = np.array(gr)
    return {'probek': int(len(gr)), 'grubosc_p50_cm': round(float(np.median(gr)), 2), 'grubosc_p90_cm': round(float(np.percentile(gr, 90)), 2)}


def log_pointlio(plik):
    if not plik or not os.path.exists(plik):
        return None
    t = open(plik, errors='replace').read().lower()
    return {'loop_back': t.count('loop back'), 'clear_buffer': t.count('clear buffer')}


def ocena(w):
    uw = []
    z = w.get('zamkniecie')
    if z and z['xy_cm'] > PROGI['zamkniecie_xy_cm']:
        uw.append(f"koniec trasy {z['xy_cm']} cm od startu w XY (cel < {PROGI['zamkniecie_xy_cm']} cm, jesli wracales na start)")
    if z and z['z_cm'] > PROGI['zamkniecie_z_cm']:
        uw.append(f"roznica wysokosci start-koniec {z['z_cm']} cm (cel < {PROGI['zamkniecie_z_cm']} cm)")
    for p in w.get('postoje', []):
        if p['std_cm'] > PROGI['postoj_std_cm'] or p['dryf_cm'] > PROGI['postoj_std_cm']:
            uw.append(f"postoj od {p['od_s']} s: drganie {p['std_cm']} cm, dryf {p['dryf_cm']} cm (cel < {PROGI['postoj_std_cm']} cm)")
    for k in w.get('kondygnacje', []):
        f = k.get('podloga')
        if f and f['grubosc_mad_cm'] > PROGI['podloga_grubosc_cm']:
            uw.append(f"kondygnacja {k['nr']}: podloga rozwarstwiona {f['grubosc_mad_cm']} cm (cel < {PROGI['podloga_grubosc_cm']})")
        if f and f['falowanie_cm'] and f['falowanie_cm'] > PROGI['podloga_falowanie_cm']:
            uw.append(f"kondygnacja {k['nr']}: podloga faluje {f['falowanie_cm']} cm (cel < {PROGI['podloga_falowanie_cm']})")
        s = k.get('sciany')
        if s and s['grubosc_p50_cm'] > PROGI['sciana_p50_cm']:
            uw.append(f"kondygnacja {k['nr']}: sciany rozmyte, mediana {s['grubosc_p50_cm']} cm (cel < {PROGI['sciana_p50_cm']})")
        if s and s['grubosc_p90_cm'] > PROGI['sciana_p90_cm']:
            uw.append(f"kondygnacja {k['nr']}: czesc scian podwojna/rozmyta, p90 {s['grubosc_p90_cm']} cm (cel < {PROGI['sciana_p90_cm']})")
    l = w.get('log')
    if l and (l['loop_back'] or l['clear_buffer']):
        uw.append(f"log Point-LIO: 'loop back' {l['loop_back']}x, 'clear buffer' {l['clear_buffer']}x - sprawdz czasy w l2_bag.py")
    return uw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('trasa'); ap.add_argument('wyjscie'); ap.add_argument('--log')
    a = ap.parse_args()
    os.makedirs(a.wyjscie, exist_ok=True)
    P = czytaj_pcd(a.pcd); P = P[np.isfinite(P).all(1)]
    T = czytaj_trase(a.trasa)
    if T is None:
        raise SystemExit('brak trasy')
    w = {'trasa_s': round(float(T[-1, 0] - T[0, 0]), 1),
         'dlugosc_trasy_m': round(float(np.linalg.norm(np.diff(T[:, 1:4], axis=0), axis=1).sum()), 1),
         'zamkniecie': {'xy_cm': round(float(np.linalg.norm(T[-1, 1:3] - T[0, 1:3]) * 100), 1),
                        'z_cm': round(float(abs(T[-1, 3] - T[0, 3]) * 100), 1)},
         'postoje': postoje(T), 'kondygnacje': [], 'log': log_pointlio(a.log)}
    KD = kondygnacje(P, T)
    for nr, (podl, suf, zt) in enumerate(KD):
        gora = KD[nr + 1][0] - 0.05 if nr + 1 < len(KD) else np.inf
        Pk = P[(P[:, 2] > podl - 0.15) & (P[:, 2] < gora)]
        Tk = T if zt is None else T[np.abs(T[:, 3] - zt) < 0.75]
        w['kondygnacje'].append({'nr': nr, 'podloga_m': round(podl, 3), 'podloga': podloga(Pk, Tk if len(Tk) > 10 else T, podl),
                                 'sciany': sciany(Pk, podl)})
    w['uwagi'] = ocena(w)
    w['progi'] = PROGI
    json.dump(w, open(os.path.join(a.wyjscie, 'kontrola_l2.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    L = [f"KONTROLA L2: trasa {w['dlugosc_trasy_m']} m, {w['trasa_s']} s",
         f"zamkniecie trasy (koniec-start): XY {w['zamkniecie']['xy_cm']} cm, Z {w['zamkniecie']['z_cm']} cm",
         f"postoje >= 5 s: {len(w['postoje'])}" + ''.join(f"\n  od {p['od_s']} s ({p['czas_s']} s): drganie {p['std_cm']} cm, dryf {p['dryf_cm']} cm" for p in w['postoje'])]
    for k in w['kondygnacje']:
        f, s = k['podloga'], k['sciany']
        L.append(f"kondygnacja {k['nr']} (podloga {k['podloga_m']:+.2f} m): "
                 + (f"podloga grubosc {f['grubosc_mad_cm']} cm, falowanie {f['falowanie_cm']} cm, nachylenie {f['nachylenie_st']} st.; " if f else 'podloga: za malo punktow; ')
                 + (f"sciany grubosc mediana {s['grubosc_p50_cm']} cm, p90 {s['grubosc_p90_cm']} cm" if s else 'sciany: za malo punktow'))
    if w['log']:
        L.append(f"log Point-LIO: loop back {w['log']['loop_back']}, clear buffer {w['log']['clear_buffer']}")
    L.append('WYNIK: ' + ('OK - mozna przejsc do strojenia / pomiarow' if not w['uwagi'] else 'UWAGI:\n  - ' + '\n  - '.join(w['uwagi'])))
    txt = '\n'.join(L)
    open(os.path.join(a.wyjscie, 'kontrola_l2.txt'), 'w', encoding='utf-8').write(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()

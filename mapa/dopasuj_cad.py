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
    roz = float(np.ptp(Q, 0).max()) if Q is not None and len(Q) else 0.0
    for j in kolejnosc:
        K = klastry(S * SKALA[j])
        if not K: continue
        m = max(np.ptp(k.reshape(-1, 2), 0).max() for k in K)
        if 5 <= m <= 400 and m >= 0.7 * roz:
            return j
    return kolejnosc[0]


def klastry(S, przerwa=4.0, min_dl=40.0):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja'); ap.add_argument('cad')
    ap.add_argument('--klaster', type=int)
    ap.add_argument('--zrodlo', default='nasz')
    a = ap.parse_args()
    kat = a.sesja.rstrip('/'); wy = os.path.join(kat, '_wynik'); os.makedirs(wy, exist_ok=True)
    ust = json.load(open(os.path.join(a.cad, 'cad.json'), encoding='utf-8'))
    S0, jedn_plik = odcinki(os.path.join(a.cad, 'rzut.dxf'), ust['warstwa'])
    if not len(S0):
        raise SystemExit(f"brak linii na warstwie {ust['warstwa']}")
    Q, sciany = skan_sesji(kat, a.zrodlo)
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
    # wynik
    wynik = {'klaster': i, 'jednostka_cad': jedn, 'warstwa': ust['warstwa'], 'zrodlo': a.zrodlo,
             'obrot_st': round(float(np.degrees(np.arctan2(Rc[1, 0], Rc[0, 0]))), 2),
             'przesuniecie_m': [round(float(x), 3) for x in t], 'statystyki': st, 'niepewne': bool(niepewne),
             'klastry': [{'nr': w[0], 'zgodnych_proc': w[3]['zgodnych_20cm_proc'], 'mediana_mm': w[3]['mediana_mm']} for w in wyniki],
             'odchylki_pkt_10cm_proc': round(100 * float(np.mean(d > 0.10)), 1)}
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

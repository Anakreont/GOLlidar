#!/usr/bin/env python3
"""Wyniki L2 na zadanie (przycisk w przegladarce): ciecie poziome, przekroje pionowe, model 3D.

    python l2_ciecia.py poziomy <wynik_l2> [--h 1.00] [--grub 0.05]
        -> ciecie_100.dxf (h=1,00 m nad podloga kazdej kondygnacji, uklad jak rzut.dxf: kondygnacje obok siebie)
    python l2_ciecia.py pionowy <wynik_l2> --ax X --ay Y --bx X --by Y [--kond N] [--nazwa A] [--grub 0.10]
        -> przekroj_A.dxf, przekroj_A.png; wpis w przekroje.json; rzut_przekroje.dxf (rzut.dxf + linie przekrojow)
    python l2_ciecia.py usun <wynik_l2> --nazwa A
    python l2_ciecia.py model3d <wynik_l2>
        -> model3d.obj, model3d.stl, model3d.dxf (sciany wyciagniete od podlogi do sufitu, 3DFACE),
           chmura3d.dxf (przerzedzona chmura jako POINT 3D), chmura.ply

<wynik_l2> = katalog z chmura.pcd i plan.json (l2_rzut.py). Wspolrzedne w metrach, skala 1:1.
"""
import argparse, json, os, string, struct, sys

import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TU, '..', 'mapa')); sys.path.insert(0, TU)


def chmura(wy):
    """chmura.pcd -> pamiec podreczna chmura.npy (float32), kolejne ciecia czytaja w ulamku sekundy."""
    pcd, npy = os.path.join(wy, 'chmura.pcd'), os.path.join(wy, 'chmura_xyz.npy')
    if os.path.exists(npy) and os.path.getmtime(npy) >= os.path.getmtime(pcd):
        return np.load(npy)
    from l2_podglad3d import czytaj_pcd
    P = czytaj_pcd(pcd); P = P[np.isfinite(P).all(1)].astype(np.float32)
    np.save(npy, P)
    return P


def plan(wy):
    p = os.path.join(wy, 'plan.json')
    if not os.path.exists(p):
        raise SystemExit('brak plan.json - przelicz rzut L2 (przycisk "Przelicz rzut" albo l2_rzut.py)')
    return json.load(open(p, encoding='utf-8'))


def nowy_dxf(warstwy_):
    import ezdxf
    from dxf_wspolne import warstwy
    d = ezdxf.new('R2010', setup=True); d.units = ezdxf.units.M
    warstwy(d, warstwy_)
    return d


def linie(Q, kom=0.02):
    """Punkty 2D -> polilinie po srodku smugi (szkielet jak dla scian rzutu)."""
    if len(Q) < 30:
        return []
    from eksport_dxf import szkielet
    try:
        return [np.asarray(pl) for pl in szkielet(np.asarray(Q, float), kom=kom)]
    except Exception as e:
        print('szkielet pominiety:', e); return []


# ---------------------------------------------------------------- ciecie poziome
def poziomy(wy, h=1.0, grub=0.05):
    P = chmura(wy); PL = plan(wy)
    from dxf_wspolne import podzialka
    war = f'CIECIE_{int(round(h * 100)):03d}'
    d = nowy_dxf(((war, 1), (war + '_PUNKTY', 8), ('OPIS', 3), ('SKALA', 7)))
    msp = d.modelspace(); ile = 0
    for k in PL['kondygnacje']:
        z = k['podl'] + h
        Q = P[(P[:, 2] > z - grub / 2) & (P[:, 2] < z + grub / 2), :2].astype(float)
        mn, mx = np.array(k['min']), np.array(k['max'])
        Q = Q[((Q >= mn - 0.2) & (Q <= mx + 0.2)).all(1)]
        dx = k['dx']
        for pl in linie(Q):
            msp.add_lwpolyline([(float(x + dx), float(y)) for x, y in pl], dxfattribs={'layer': war}); ile += 1
        if len(Q):
            ij = np.unique(np.round(Q / 0.02).astype(np.int64), axis=0) * 0.02
            for x, y in ij:
                msp.add_point((float(x + dx), float(y)), dxfattribs={'layer': war + '_PUNKTY'})
        msp.add_text(f"kondygnacja {k['nr']}: ciecie {h:.2f} m nad podloga (z = {z:.2f})".replace('.', ','), height=0.3,
                     dxfattribs={'layer': 'OPIS'}).set_placement((mn[0] + dx, mx[1] + 0.8))
        podzialka(msp, float(mn[0] + dx), float(mn[1]) - 1.5)
    plik = os.path.join(wy, f'ciecie_{int(round(h * 100)):03d}.dxf'); d.saveas(plik)
    print(f'zapisano {plik}: {ile} polilinii')
    L = przekroje(wy)
    if L:                                     # te same oznaczenia przekrojow co na rzucie
        rzut_z_przekrojami(wy, PL, L)
        plik = plik[:-4] + '_przekroje.dxf'
    print('PLIK', os.path.basename(plik))
    return plik


# ---------------------------------------------------------------- przekroje pionowe
def przekroje(wy):
    p = os.path.join(wy, 'przekroje.json')
    return json.load(open(p, encoding='utf-8')) if os.path.exists(p) else []


def zapisz_przekroje(wy, L):
    with open(os.path.join(wy, 'przekroje.json'), 'w', encoding='utf-8') as f:
        json.dump(L, f, ensure_ascii=False, indent=1)


def nastepna_nazwa(L):
    uz = {p['nazwa'] for p in L}
    for n in list(string.ascii_uppercase) + [a + b for a in string.ascii_uppercase for b in string.ascii_uppercase]:
        if n not in uz:
            return n


def pionowy(wy, a, b, kond=None, nazwa=None, grub=0.10):
    P = chmura(wy); PL = plan(wy); L = [p for p in przekroje(wy) if p['nazwa'] != nazwa]
    nazwa = nazwa or nastepna_nazwa(L)
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.hypot(*(b - a)) < 0.3:
        raise SystemExit('za krotka linia przekroju (< 0,3 m)')
    KD = PL['kondygnacje']
    if kond is None:                          # kondygnacja, w ktorej obrysie lezy srodek linii (gdy kilka - najnizsza)
        c = (a + b) / 2
        kond = next((k['nr'] for k in KD if all(k['min'][i] - 0.5 <= c[i] <= k['max'][i] + 0.5 for i in (0, 1))), KD[0]['nr'])
    k = next(k for k in KD if k['nr'] == kond)
    t = (b - a) / np.hypot(*(b - a)); n = np.array([-t[1], t[0]]); dl = float(np.hypot(*(b - a)))
    # przekroj przez caly budynek w pionie (wszystkie kondygnacje) - widac stropy, schody, poddasze
    Q = P.astype(float); r = Q[:, :2] - a
    u = r @ t; v = r @ n
    m = (np.abs(v) < grub / 2) & (u > -0.05) & (u < dl + 0.05)
    S = np.stack([u[m], Q[m, 2]], 1)
    z0 = k['podl']                            # +/-0,00 = podloga kondygnacji, na ktorej wskazano przekroj
    S[:, 1] -= z0
    from dxf_wspolne import podzialka
    d = nowy_dxf((('PRZEKROJ', 7), ('PRZEKROJ_PUNKTY', 8), ('POZIOMY', 3), ('OPIS', 2), ('SKALA', 7)))
    msp = d.modelspace(); pl_ = linie(S, 0.02)
    for pl in pl_:
        msp.add_lwpolyline([(float(x), float(y)) for x, y in pl], dxfattribs={'layer': 'PRZEKROJ'})
    for x, y in np.unique(np.round(S / 0.02).astype(np.int64), axis=0) * 0.02:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'PRZEKROJ_PUNKTY'})
    for kk in KD:                             # poziomy podlog i sufitow wszystkich kondygnacji
        for zz, op in ((kk['podl'] - z0, f"podloga k{kk['nr']}"), (kk['suf'] - z0, f"sufit k{kk['nr']}")):
            msp.add_line((-0.6, zz), (dl + 0.6, zz), dxfattribs={'layer': 'POZIOMY', 'linetype': 'DASHED'})
            msp.add_text(f"{'+' if zz >= 0 else ''}{zz:.2f} {op}".replace('.', ','), height=0.12, dxfattribs={'layer': 'POZIOMY'}).set_placement((dl + 0.7, zz))
    zmax = float(S[:, 1].max()) if len(S) else 3.0; zmin = float(S[:, 1].min()) if len(S) else 0.0
    msp.add_text(f'PRZEKROJ {nazwa}-{nazwa}', height=0.35, dxfattribs={'layer': 'OPIS'}).set_placement((0, zmax + 0.6))
    msp.add_text(f'{nazwa}', height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((-0.5, zmax + 0.1))
    msp.add_text(f'{nazwa}', height=0.3, dxfattribs={'layer': 'OPIS'}).set_placement((dl + 0.2, zmax + 0.1))
    msp.add_text(f"od ({a[0]:.2f}; {a[1]:.2f}) do ({b[0]:.2f}; {b[1]:.2f}), dlugosc {dl:.2f} m, grubosc plastra {grub * 100:.0f} cm, "
                 f"+/-0,00 = podloga kondygnacji {kond}".replace('.', ','), height=0.12, dxfattribs={'layer': 'OPIS'}).set_placement((0, zmax + 0.35))
    podzialka(msp, 0.0, zmin - 1.0)
    plik = os.path.join(wy, f'przekroj_{nazwa}.dxf'); d.saveas(plik)
    # podglad PNG
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(min(24, 3 + dl / 1.5), min(14, 2 + (zmax - zmin) / 1.5)))
    ax.scatter(S[::2, 0], S[::2, 1], s=0.4, c='0.55', lw=0)
    for pl in pl_:
        ax.plot(pl[:, 0], pl[:, 1], '-', color='k', lw=0.9)
    for kk in KD:
        for zz in (kk['podl'] - z0, kk['suf'] - z0):
            ax.axhline(zz, color='tab:green', ls='--', lw=0.6)
            ax.text(dl + 0.1, zz, f'{zz:+.2f}'.replace('.', ','), color='tab:green', fontsize=8, va='bottom')
    ax.set_aspect('equal'); ax.grid(alpha=.25); ax.set_title(f'Przekroj {nazwa}-{nazwa}  (dlugosc {dl:.2f} m)'.replace('.', ','))
    fig.tight_layout(); fig.savefig(os.path.join(wy, f'przekroj_{nazwa}.png'), dpi=110); plt.close(fig)
    wys = round(k['suf'] - k['podl'], 3)
    L.append({'nazwa': nazwa, 'a': [round(float(x), 3) for x in a], 'b': [round(float(x), 3) for x in b], 'kond': kond,
              'dl': round(dl, 3), 'grub': grub, 'punktow': int(len(S)), 'wys_srodek': wys})
    L.sort(key=lambda p: (len(p['nazwa']), p['nazwa'])); zapisz_przekroje(wy, L)
    rzut_z_przekrojami(wy, PL, L)
    print(f'zapisano {plik}: {len(S)} pkt, {len(pl_)} polilinii, dl. {dl:.2f} m')
    print('NAZWA', nazwa); print('PLIK', os.path.basename(plik))
    return nazwa


def rzut_z_przekrojami(wy, PL, L):
    """rzut.dxf + linie przekrojow z oznaczeniami (A-A, strzalki kierunku patrzenia) -> rzut_przekroje.dxf;
    to samo dla ciecie_*.dxf, jesli sa."""
    import ezdxf, glob
    for src in [os.path.join(wy, 'rzut.dxf')] + sorted(glob.glob(os.path.join(wy, 'ciecie_[0-9][0-9][0-9].dxf'))):
        if not os.path.exists(src):
            continue
        d = ezdxf.readfile(src)
        if 'PRZEKROJE' not in d.layers:
            d.layers.add('PRZEKROJE', color=6)
        msp = d.modelspace()
        for e in list(msp.query('*[layer=="PRZEKROJE"]')):
            msp.delete_entity(e)
        for p in L:
            k = next((k for k in PL['kondygnacje'] if k['nr'] == p['kond']), PL['kondygnacje'][0]); dx = k['dx']
            a = np.array(p['a']) + [dx, 0]; b = np.array(p['b']) + [dx, 0]
            t = (b - a) / max(1e-9, np.hypot(*(b - a))); n = np.array([-t[1], t[0]])
            msp.add_line(tuple(a - t * 0.5), tuple(b + t * 0.5), dxfattribs={'layer': 'PRZEKROJE', 'linetype': 'DASHDOT'})
            for q, s_ in ((a - t * 0.5, -1), (b + t * 0.5, 1)):
                msp.add_lwpolyline([tuple(q), tuple(q + n * 0.6), tuple(q + n * 0.45 + t * 0.08), tuple(q + n * 0.6), tuple(q + n * 0.45 - t * 0.08)],
                                   dxfattribs={'layer': 'PRZEKROJE'})
                msp.add_text(p['nazwa'], height=0.3, dxfattribs={'layer': 'PRZEKROJE'}).set_placement(tuple(q + n * 0.75 + t * s_ * 0.1))
        d.saveas(src[:-4] + '_przekroje.dxf')


def usun(wy, nazwa):
    L = [p for p in przekroje(wy) if p['nazwa'] != nazwa]; zapisz_przekroje(wy, L)
    for e in ('dxf', 'png'):
        try:
            os.remove(os.path.join(wy, f'przekroj_{nazwa}.{e}'))
        except OSError:
            pass
    if os.path.exists(os.path.join(wy, 'plan.json')):
        rzut_z_przekrojami(wy, plan(wy), L)
    print('usunieto', nazwa)


# ---------------------------------------------------------------- model 3D
def model3d(wy, grub_sc=0.10, voxel=0.06, maks=150000):
    PL = plan(wy)
    V, F = [], []                                          # wierzcholki i trojkaty

    def prost(p0, p1, z0, z1):
        """Sciana jako prostopadloscian grubosci grub_sc wzdluz odcinka p0-p1."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        L_ = np.hypot(*(p1 - p0))
        if L_ < 0.03:
            return
        t = (p1 - p0) / L_; n = np.array([-t[1], t[0]]) * grub_sc / 2
        rog = [p0 - n, p1 - n, p1 + n, p0 + n]
        i0 = len(V)
        for z in (z0, z1):
            for c in rog:
                V.append((c[0], c[1], z))
        q = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
        for a, b, c, d in q:
            F.append((i0 + a, i0 + b, i0 + c)); F.append((i0 + a, i0 + c, i0 + d))

    for k in PL['kondygnacje']:
        for pl in k['sciany']:
            for p0, p1 in zip(pl[:-1], pl[1:]):
                prost(p0, p1, k['podl'], k['suf'])
        # posadzka: prostokat obrysu kondygnacji (cienka plyta 5 cm) - punkt odniesienia w CAD
        mn, mx = np.array(k['min']), np.array(k['max']); z = k['podl']; i0 = len(V)
        for zz in (z - 0.05, z):
            for c in ((mn[0], mn[1]), (mx[0], mn[1]), (mx[0], mx[1]), (mn[0], mx[1])):
                V.append((c[0], c[1], zz))
        for a, b, c, d in [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]:
            F.append((i0 + a, i0 + b, i0 + c)); F.append((i0 + a, i0 + c, i0 + d))
    V = np.asarray(V, float); F = np.asarray(F, int)
    # OBJ
    with open(os.path.join(wy, 'model3d.obj'), 'w') as f:
        f.write('# GOLlidar L2: sciany wyciagniete od podlogi do sufitu, metry\n')
        f.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in V)
        f.writelines(f'f {a + 1} {b + 1} {c + 1}\n' for a, b, c in F)
    # STL (binarny)
    with open(os.path.join(wy, 'model3d.stl'), 'wb') as f:
        f.write(b'GOLlidar L2 model 3D'.ljust(80, b' ')); f.write(struct.pack('<I', len(F)))
        T = V[F]; N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
        rek = np.zeros(len(F), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]); rek['n'] = N; rek['v'] = T
        f.write(rek.tobytes())
    # DXF 3D (3DFACE)
    d = nowy_dxf((('SCIANY_3D', 7), ('POSADZKA_3D', 8)))
    msp = d.modelspace()
    for a, b, c in F:
        msp.add_3dface([tuple(V[a]), tuple(V[b]), tuple(V[c]), tuple(V[c])], dxfattribs={'layer': 'SCIANY_3D'})
    d.saveas(os.path.join(wy, 'model3d.dxf'))
    # chmura: DXF z punktami 3D (przerzedzona) i PLY (pelna)
    P = chmura(wy)
    from l2_podglad3d import podglad
    Q, _ = podglad(P.astype(float), voxel, maks)
    d = nowy_dxf((('CHMURA', 8),)); msp = d.modelspace()
    for x, y, z in Q:
        msp.add_point((float(x), float(y), float(z)), dxfattribs={'layer': 'CHMURA'})
    d.saveas(os.path.join(wy, 'chmura3d.dxf'))
    with open(os.path.join(wy, 'chmura.ply'), 'wb') as f:
        f.write(f'ply\nformat binary_little_endian 1.0\ncomment GOLlidar L2\nelement vertex {len(P)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode())
        f.write(P.astype('<f4').tobytes())
    print(f'model 3D: {len(F)} trojkatow; chmura3d.dxf {len(Q)} pkt; chmura.ply {len(P)} pkt')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('co', choices=('poziomy', 'pionowy', 'usun', 'model3d')); ap.add_argument('wynik')
    ap.add_argument('--h', type=float, default=1.0); ap.add_argument('--grub', type=float)
    for x in ('ax', 'ay', 'bx', 'by'):
        ap.add_argument('--' + x, type=float)
    ap.add_argument('--kond', type=int); ap.add_argument('--nazwa')
    a = ap.parse_args()
    if a.nazwa and not (a.nazwa.isalnum() and len(a.nazwa) <= 4):
        raise SystemExit('nazwa przekroju: litery/cyfry, do 4 znakow')
    if a.co == 'poziomy':
        poziomy(a.wynik, a.h, a.grub or 0.05)
    elif a.co == 'pionowy':
        pionowy(a.wynik, (a.ax, a.ay), (a.bx, a.by), a.kond, a.nazwa, a.grub or 0.10)
    elif a.co == 'usun':
        usun(a.wynik, a.nazwa)
    else:
        model3d(a.wynik)


if __name__ == '__main__':
    main()

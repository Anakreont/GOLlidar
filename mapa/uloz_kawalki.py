"""Ukladanie pocietych kawalkow mapy (D500) na rzucie CAD - kazdy kawalek jako bryla sztywna.

Kawalki (kawalki.py) sa lokalnie poprawne, bledne jest ich polozenie i obrot wzgledem siebie. Algorytm
(konsultacje z innymi AI 28.11.2026, oba niezaleznie zaproponowaly to samo):
 1. Kierunek scian kawalka (1-2 szczyty histogramu modulo 90 st.) i rysunku -> 4-8 obrotow bazowych na kawalek.
 2. Hipotezy polozenia: mapa odleglosci od scian CAD obcieta na 0,5 m (sciana przerobiona w budynku nie psuje
    wyniku reszty), korelacja (FFT) punktow kawalka z ta mapa po calej kondygnacji, top-K minimow (odstep >= 1,5 m).
 3. Znaczniki "Jestem tu" (SALA <nazwa>): punkt trasy z chwili znacznika powinien wypasc w sali o tej nazwie
    na rysunku (obrys pomieszczenia albo kolo 3 m wokol etykiety, z zapasem 1,5 m). Poza sala - kara, nie zakaz
    (pomylka operatora / nieaktualny rysunek).
 4. Kazda hipoteza dociagnieta ICP punkt-prosta do scian CAD (Cauchy 5 cm, obrot +-5 st.); ocena: udzial punktow
    < 8 cm od scian CAD.
 5. Wybor zgodnego zestawu hipotez algorytmem Viterbiego na lancuchu czasowym kawalkow: poczatek K(n+1) najwyzej
    0,5 m + 1,2 m/s * przerwa od konca K(n) (ponad tym kara), kierunek marszu przez przerwe miekko ciagly.
    Stan "nieznany" dla kawalka bez sensownej hipotezy.
 6. Klasa kawalka: pewny / niepewny / nieznany (dopasowanie do CAD + margines nad druga najlepsza mozliwoscia).
Wynik: polozenie kazdego kawalka (obrot wokol jego srodka + przesuniecie) w ukladzie rysunku (metry).
"""
import os
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
from scipy.optimize import least_squares

RES = 0.10          # siatka szukania [m]
OBCIECIE = 0.5      # obciecie mapy odleglosci [m]
INL = 0.12          # punkt "na scianie" CAD [m] (smuga D500 ~10 cm)
K_HIP = 6           # hipotez na kawalek
ODSTEP = 1.5        # minimalny odstep hipotez [m]
KARA_SALA = 0.25    # kara za znacznik poza swoja sala (w jednostkach "udzialu punktow")
U_NIEZNANY = 0.88   # koszt stanu "nieznany" (= udzial 0,12); prawdziwe kawalki D500 maja udzial 0,2-0,6
V_MAX, ZAPAS = 1.2, 0.5


def _R(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def kier90(P=None, N=None, ile=2):
    """Kandydaci na kierunek scian modulo 90 st. [rad]: szczyty histogramu (1 st., wygladzony) kierunkow normalnych.
    Meble i ludzie dodaja kierunki przypadkowe i potrafia przesunac srednia o kilkanascie stopni, stad szczyty
    zamiast sredniej i do 'ile' kandydatow (drugi, jesli ma >= 50 % pierwszego)."""
    if N is None or len(N) != len(P):
        t = cKDTree(P); idx = np.arange(0, len(P), max(1, len(P) // 4000))
        _, j = t.query(P[idx], k=20)
        X = P[j] - P[j].mean(1, keepdims=True)
        C = np.einsum('nki,nkj->nij', X, X); w, v = np.linalg.eigh(C)
        m = w[:, 0] < 0.02 * w[:, 1]
        N = v[m, :, 0]
    if not len(N):
        return [0.0]
    a = np.degrees(np.arctan2(N[:, 1], N[:, 0])) % 90
    h = np.bincount(np.floor(a).astype(int) % 90, minlength=90).astype(float)
    k = np.exp(-0.5 * (np.arange(-6, 7) / 2.0) ** 2)
    h = np.convolve(np.r_[h[-6:], h, h[:6]], k, 'same')[6:-6]
    wyn = []
    for _ in range(ile):
        i = int(np.argmax(h))
        if wyn and h[i] < 0.5 * hmax:
            break
        if not wyn:
            hmax = h[i]
        # dokladniej: srednia kolowa wokol szczytu +-3 st.
        d = (a - i - 0.5 + 45) % 90 - 45; m = np.abs(d) < 3
        wyn.append(np.radians(i + 0.5 + (d[m].mean() if m.any() else 0)))
        for o in range(-10, 11):
            h[(i + o) % 90] = 0
    return wyn


def kier90_cad(S):
    d = S[:, 1] - S[:, 0]; L = np.linalg.norm(d, axis=1)
    a = np.arctan2(d[:, 1], d[:, 0])
    return float(np.angle((L * np.exp(4j * a)).sum()) / 4)


def probki(S, krok=0.02):
    P, N = [], []
    for a, b in S:
        L = np.linalg.norm(b - a)
        if L < 1e-6:
            continue
        n = max(2, int(L / krok)); t = np.linspace(0, 1, n)[:, None]
        P.append(a + (b - a) * t); u = (b - a) / L; N.append(np.repeat([[-u[1], u[0]]], n, 0))
    return np.vstack(P), np.vstack(N)


class Rzut:
    """Rzut CAD jednej kondygnacji: sciany (odcinki w m), mapa kosztu, probki do ICP, sale (nazwa -> maska)."""

    def __init__(self, S, sale=None, margines=12.0):
        self.S = np.asarray(S, float)
        self.C, self.CN = probki(self.S)
        self.drzewo = cKDTree(self.C)
        self.o = self.C.min(0) - margines
        sh = tuple(((self.C.max(0) + margines - self.o) / RES).astype(int) + 1)
        G = np.zeros(sh, bool); ij = ((self.C - self.o) / RES).astype(int); G[ij[:, 0], ij[:, 1]] = True
        self.koszt = np.minimum(ndimage.distance_transform_edt(~G) * RES, OBCIECIE)
        self.phi = kier90_cad(self.S)
        self.sale = {}
        for nazwa, ksz in (sale or {}).items():           # ksz: wielokat (n,2) albo punkt etykiety (2,)
            M = np.zeros(sh, bool)
            ksz = np.asarray(ksz, float)
            if ksz.ndim == 1:
                yy, xx = np.mgrid[0:sh[0], 0:sh[1]]
                c = (ksz - self.o) / RES
                M = (yy - c[0]) ** 2 + (xx - c[1]) ** 2 <= ((3.0 + 1.5) / RES) ** 2
            else:
                from matplotlib.path import Path
                ii, jj = np.mgrid[0:sh[0], 0:sh[1]]
                pts = np.c_[ii.ravel(), jj.ravel()] * RES + self.o
                M = Path(ksz).contains_points(pts).reshape(sh)
                M = ndimage.binary_dilation(M, iterations=int(1.5 / RES))
            self.sale[normuj(nazwa)] = M

    def odl(self, Q):
        d, _ = self.drzewo.query(Q, distance_upper_bound=1.0)
        return np.minimum(d, 1.0)


def normuj(n):
    """'01/12' == '1/12' == ' 1 / 12 ' (porownanie nazw sal ze znacznika i z rysunku)."""
    import re
    n = str(n).lower().replace(' ', '')
    return '/'.join(p.lstrip('0') or '0' for p in re.split(r'[/\\.]', n))


def przeksztalc(P, c, a, t):
    return (np.asarray(P) - c) @ _R(a).T + t


def hipotezy(kaw, rz, log=print):
    """Top-K hipotez (a, t, udzial, kara_sali) dla jednego kawalka."""
    P = kaw['P']
    c = P.mean(0)
    rng = np.random.default_rng(kaw['nr'])
    v = np.unique(np.round(P / 0.05).astype(np.int64), axis=0, return_index=True)[1]
    Q = P[v]
    if len(Q) > 2500:
        Q = Q[rng.choice(len(Q), 2500, replace=False)]
    phis = kier90(P)        # wlasne normalne z 20 sasiadow (normalne z SLAM bywaja zbyt lokalne)
    znaczniki = []                                          # (punkt w ukladzie kawalka, maska sali)
    for t_, nz in kaw.get('sale', []):
        M = rz.sale.get(normuj(nz))
        if M is not None:
            m = np.array([np.interp(t_, kaw['czasy'], kaw['trasa'][:, 0]), np.interp(t_, kaw['czasy'], kaw['trasa'][:, 1])])
            znaczniki.append((m, M))
    kand = []
    for k in range(4 * len(phis)):
        a = rz.phi - phis[k // 4] + (k % 4) * np.pi / 2
        Qr = (Q - c) @ _R(a).T
        lo = Qr.min(0)
        ij = np.round((Qr - lo) / RES).astype(int)
        B = np.zeros(ij.max(0) + 1); np.add.at(B, (ij[:, 0], ij[:, 1]), 1.0)
        if B.shape[0] >= rz.koszt.shape[0] or B.shape[1] >= rz.koszt.shape[1]:
            continue
        Sc = fftconvolve(rz.koszt, B[::-1, ::-1], mode='valid') / len(Q)     # sredni koszt dla polozenia (i, j)
        U = 1.0 - Sc / OBCIECIE                                               # ~ udzial punktow na scianach
        kara = np.zeros_like(U)
        for m, M in znaczniki:                                                # znacznik poza swoja sala: kara
            off = np.round(((m - c) @ _R(a).T - lo) / RES).astype(int)
            Ms = np.zeros_like(U, bool)
            i0, j0 = off
            sub = M[max(i0, 0):i0 + U.shape[0], max(j0, 0):j0 + U.shape[1]]
            Ms[max(-i0, 0):max(-i0, 0) + sub.shape[0], max(-j0, 0):max(-j0, 0) + sub.shape[1]] = sub
            kara += np.where(Ms, 0.0, KARA_SALA)
        if znaczniki:
            kara /= len(znaczniki)
        W = U - kara
        # top kandydaci z tlumieniem otoczenia
        Wc = W.copy(); r = int(ODSTEP / RES)
        for _ in range(K_HIP):
            i, j = np.unravel_index(np.argmax(Wc), Wc.shape)
            t = rz.o + np.array([i, j]) * RES - lo
            kand.append((float(Wc[i, j]), a, t, float(kara[i, j])))
            Wc[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = -9
    kand.sort(key=lambda x: -x[0])
    # tlumienie miedzy obrotami (ta sama pozycja, inny obrot zostaje) i dociagniecie ICP
    wyn = []
    for w0, a, t, kara in kand[:4 * K_HIP]:
        a2, t2 = icp(Q, c, a, t, rz)
        d = rz.odl(przeksztalc(Q, c, a2, t2))
        u = float(np.mean(d < INL))
        if any(np.linalg.norm(t2 - h['t']) < ODSTEP and abs(np.angle(np.exp(1j * (a2 - h['a'])))) < np.radians(20) for h in wyn):
            continue
        kara2 = 0.0
        for m, M in znaczniki:
            q = np.round((przeksztalc(m[None], c, a2, t2)[0] - rz.o) / RES).astype(int)
            ok = 0 <= q[0] < M.shape[0] and 0 <= q[1] < M.shape[1] and M[q[0], q[1]]
            kara2 += 0.0 if ok else KARA_SALA
        if znaczniki:
            kara2 /= len(znaczniki)
        wyn.append({'a': a2, 't': t2, 'udzial': u, 'kara': kara2, 'c': c})
    wyn.sort(key=lambda h: -(h['udzial'] - h['kara']))
    return wyn[:K_HIP]


def icp(Q, c, a, t, rz, iters=12):
    """Dociagniecie bryly do scian CAD: punkt-prosta, Cauchy 5 cm, zasieg 0,5 -> 0,15 m."""
    x = np.array([a, *t])
    for it, md in enumerate(np.linspace(0.5, 0.15, iters)):
        X = przeksztalc(Q, c, x[0], x[1:])
        d, j = rz.drzewo.query(X, distance_upper_bound=md)
        m = np.isfinite(d)
        if m.sum() < 20:
            break
        Qm, Pm, Nm = Q[m], rz.C[j[m]], rz.CN[j[m]]

        def res(y):
            return np.sum((przeksztalc(Qm, c, y[0], y[1:]) - Pm) * Nm, 1)
        y = least_squares(res, x, loss='cauchy', f_scale=0.05, max_nfev=20).x
        if abs(y[0] - a) > np.radians(5):                     # obrot tylko +-5 st. od obrotu bazowego
            y[0] = a + np.sign(y[0] - a) * np.radians(5)
        x = y
    return x[0], x[1:]


def _kier(T):
    d = T[-1] - T[0]
    return float(np.arctan2(d[1], d[0])) if np.hypot(*d) > 0.5 else None


def uloz(kawalki, rz, log=print):
    """Hipotezy dla kazdego kawalka + wybor zgodnego zestawu (Viterbi na lancuchu czasowym)."""
    H = []
    for k in kawalki:
        h = hipotezy(k, rz, log)
        H.append(h)
        log(f"  K{k['nr']:02d}: {len(h)} hipotez, najlepsza udzial {h[0]['udzial']:.2f}" if h else f"  K{k['nr']:02d}: brak hipotez")
    n = len(kawalki)
    u = [np.array([2 * (1 - h['udzial'] + h['kara']) for h in H[i]] + [2 * U_NIEZNANY]) for i in range(n)]

    def przejscie(i, a_, b_):
        """Koszt miedzy stanem a_ kawalka i a stanem b_ kawalka i+1 (ostatni stan = nieznany)."""
        if a_ == len(H[i]) or b_ == len(H[i + 1]):
            return 0.3
        ha, hb = H[i][a_], H[i + 1][b_]
        ka, kb = kawalki[i], kawalki[i + 1]
        e = przeksztalc(ka['trasa'][-1:], ha['c'], ha['a'], ha['t'])[0]
        s = przeksztalc(kb['trasa'][:1], hb['c'], hb['a'], hb['t'])[0]
        dt = max(0.0, kb['od_s'] - ka['do_s'])
        nad = max(0.0, np.linalg.norm(s - e) - (ZAPAS + V_MAX * dt))
        # kierunek marszu na koncu K(n) i na poczatku K(n+1) w ukladzie rysunku (uklady kawalkow sa dowolne,
        # wiec porownujemy ruch, nie obrot); w przerwie mozna skrecic, stad kara slaba i obcieta
        de = _kier(przeksztalc(ka['trasa'][-6:], ha['c'], ha['a'], ha['t']))
        ds = _kier(przeksztalc(kb['trasa'][:6], hb['c'], hb['a'], hb['t']))
        da = abs(np.angle(np.exp(1j * (ds - de)))) if de is not None and ds is not None else 0.0
        return min((nad / 0.5) ** 2, 4.0) + 0.3 * min((da / np.radians(60)) ** 2, 1.0)

    T = [np.array([[przejscie(i, a_, b_) for b_ in range(len(u[i + 1]))] for a_ in range(len(u[i]))]) for i in range(n - 1)]
    # Viterbi (przod) + wstecz -> min-marginesy (do oceny pewnosci)
    F = [u[0]]
    for i in range(n - 1):
        F.append(u[i + 1] + np.min(F[-1][:, None] + T[i], axis=0))
    Bk = [np.zeros(len(u[-1]))]
    for i in range(n - 2, -1, -1):
        Bk.insert(0, np.min(T[i] + (u[i + 1] + Bk[0])[None, :], axis=1))
    mm = [F[i] + Bk[i] for i in range(n)]
    wyb = [int(np.argmin(m_)) for m_ in mm]
    wynik = []
    for i, k in enumerate(kawalki):
        s = wyb[i]
        marg = float(np.min(np.delete(mm[i], s)) - mm[i][s]) if len(mm[i]) > 1 else 9.0
        if s == len(H[i]):
            if not H[i]:
                wynik.append({'nr': k['nr'], 'klasa': 'nieznany', 'udzial': 0.0, 'margines': round(marg, 2)})
                continue
            h = H[i][0]; klasa = 'nieznany'          # rysowany w najlepszym miejscu, na czerwono
        else:
            h = H[i][s]
            klasa = 'pewny' if (h['udzial'] >= 0.30 and marg >= 0.3) else 'niepewny' if h['udzial'] >= 0.15 else 'nieznany'
        wynik.append({'nr': k['nr'], 'klasa': klasa, 'udzial': round(h['udzial'], 3), 'margines': round(marg, 2),
                      'a': float(h['a']), 't': h['t'].tolist(), 'c': h['c'].tolist(), 'kara_sali': round(h['kara'], 2),
                      'hipotez': len(H[i])})
    return wynik


def metryki(kawalki, wynik, rz):
    klasy = {c: sum(w['klasa'] == c for w in wynik) for c in ('pewny', 'niepewny', 'nieznany')}
    d = []
    for k, w in zip(kawalki, wynik):
        if 'a' in w and w['klasa'] != 'nieznany':
            d.append(rz.odl(przeksztalc(k['P'][::3], w['c'], w['a'], np.array(w['t']))))
    if not d:
        return {'klasy': klasy}
    d = np.concatenate(d)
    return {'mediana_cm': round(float(np.median(d) * 100), 1), 'p90_cm': round(float(np.percentile(d, 90) * 100), 1),
            **{f'pod_{x}cm_proc': round(float(np.mean(d < x / 100) * 100), 1) for x in (5, 10, 20)},
            'klasy': klasy}


# ---------------------------------------------------------------- sesja + rysunek z dysku -------------------------

def etykiety_cad(plik, skala):
    """Teksty z rysunku (TEXT/MTEXT/ATTRIB na wszystkich warstwach): [(tekst, xy w m)] - do znacznikow SALA."""
    import ezdxf
    wyn = []
    try:
        d = ezdxf.readfile(plik)
    except Exception:
        return wyn
    for e in d.modelspace().query('TEXT MTEXT INSERT'):
        try:
            if e.dxftype() == 'INSERT':
                for at in e.attribs:
                    wyn.append((at.dxf.text, np.array([at.dxf.insert.x, at.dxf.insert.y]) * skala))
                continue
            t = e.plain_text() if e.dxftype() == 'MTEXT' else e.dxf.text
            wyn.append((t, np.array([e.dxf.insert.x, e.dxf.insert.y]) * skala))
        except Exception:
            pass
    return wyn


def sale_z_etykiet(etyk, nazwy):
    """Nazwa znacznika -> polozenie etykiety na rysunku (porownanie po normuj, takze pierwsze slowo tekstu)."""
    wyn = {}
    for nz in set(nazwy):
        n = normuj(nz)
        for t, xy in etyk:
            slowa = [normuj(x) for x in str(t).replace('\n', ' ').split()]
            if normuj(t) == n or (slowa and slowa[0] == n):
                wyn[nz] = xy
                break
    return wyn


def zapisz(kat, kawalki, wynik, S, met, nazwa='kawalki_cad'):
    import ezdxf, json
    wy = os.path.join(kat, '_wynik')
    d = ezdxf.new('R2010'); d.header['$INSUNITS'] = 6; msp = d.modelspace()
    d.layers.add('CAD_RZUT', color=8)
    for a, b in S:
        msp.add_line(tuple(a), tuple(b), dxfattribs={'layer': 'CAD_RZUT'})
    kol = {'pewny': 3, 'niepewny': 30, 'nieznany': 1}
    for k, w in zip(kawalki, wynik):
        if 'a' not in w:
            continue
        war = f"KAWALEK_{k['nr']:02d}_{w['klasa']}"
        d.layers.add(war, color=kol[w['klasa']])
        f = lambda X: przeksztalc(X, np.array(w['c']), w['a'], np.array(w['t']))
        P = f(k['P'])
        P = P[np.unique(np.round(P / 0.03).astype(np.int64), axis=0, return_index=True)[1]]
        for p in P:
            msp.add_point(tuple(p), dxfattribs={'layer': war})
        for pl in k.get('sciany', []):
            msp.add_lwpolyline(f(np.asarray(pl)).tolist(), dxfattribs={'layer': war, 'lineweight': 35})
        msp.add_lwpolyline(f(k['trasa']).tolist(), dxfattribs={'layer': war, 'linetype': 'DASHED'} if 'DASHED' in d.linetypes else {'layer': war})
        c = f(k['trasa']).mean(0)
        msp.add_text(f"K{k['nr']:02d} {w['klasa']} {w['udzial']:.2f}", height=0.25,
                     dxfattribs={'layer': war}).set_placement(tuple(c))
    p = os.path.join(wy, nazwa + '.dxf'); d.saveas(p)
    json.dump({'metryki': met, 'kawalki': wynik}, open(os.path.join(wy, nazwa + '.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=1)
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(16, 10))
        for a, b in S:
            ax.plot([a[0], b[0]], [a[1], b[1]], color='0.55', lw=0.5)
        kk = {'pewny': 'tab:green', 'niepewny': 'tab:orange', 'nieznany': 'tab:red'}
        for k, w in zip(kawalki, wynik):
            if 'a' in w:
                X = przeksztalc(k['P'][::3], np.array(w['c']), w['a'], np.array(w['t']))
                ax.scatter(*X.T, s=0.2, color=kk[w['klasa']]); ax.text(*X.mean(0), f"K{k['nr']}", fontsize=7)
        ax.set_aspect('equal'); ax.set_title('kawalki na rysunku: zielone pewne, pomaranczowe niepewne, czerwone bez pewnego miejsca')
        fig.savefig(os.path.join(wy, nazwa + '.png'), dpi=110, bbox_inches='tight'); plt.close(fig)
    except Exception:
        pass
    return p


def main():
    import argparse, json, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import kawalki as KW
    from dopasuj_cad import odcinki, zgadnij_jednostke, klastry, SKALA
    ap = argparse.ArgumentParser(description='Ukladanie kawalkow sesji D500 na rzucie CAD')
    ap.add_argument('sesja'); ap.add_argument('cad', help='katalog z rzut.dxf + cad.json (jak dopasuj_cad.py)')
    ap.add_argument('--klaster', type=int, help='numer rzutu w rysunku (domyslnie: najlepiej pasujacy)')
    a = ap.parse_args()
    kat = a.sesja.rstrip('/')
    K, t0 = KW.kawalki(kat)
    sl = KW.sale(kat, t0)
    for k in K:
        k['sale'] = [(t, nz) for t, nz in sl if k['od_s'] - 1 <= t <= k['do_s'] + 1]
    print(f'{len(K)} kawalkow, znacznikow SALA: {len(sl)}', flush=True)
    ust = json.load(open(os.path.join(a.cad, 'cad.json'), encoding='utf-8'))
    plik = os.path.join(a.cad, 'rzut.dxf')
    S0, jp = odcinki(plik, ust['warstwa'])
    jedn = ust.get('jedn') or 'auto'
    if jedn == 'auto':
        jedn = zgadnij_jednostke(S0, np.vstack([k['P'] for k in K]), tuple(dict.fromkeys(([jp] if jp else []) + ['cm', 'm', 'mm'])))
    S = S0 * SKALA[jedn]; S = S[np.linalg.norm(S[:, 1] - S[:, 0], axis=1) > 0.05]
    KL = klastry(S)
    et = etykiety_cad(plik, SKALA[jedn])
    sale = sale_z_etykiet(et, [nz for _, nz in sl])
    print(f'CAD: jednostka {jedn}, rzutow {len(KL)}, znaczniki odnalezione na rysunku: {len(sale)}/{len(set(nz for _, nz in sl))}', flush=True)
    if a.klaster is not None:
        wyb = [a.klaster]
    elif len(KL) == 1:
        wyb = [0]
    else:                                   # wybor rzutu: najwieksze kawalki probnie na kazdym
        duze = sorted(K, key=lambda k: -len(k['P']))[:4]; oc = []
        for i, Sk in enumerate(KL):
            rz = Rzut(Sk.reshape(-1, 2, 2), sale)
            oc.append(np.mean([max([h['udzial'] - h['kara'] for h in hipotezy(k, rz)] or [0]) for k in duze]))
            print(f'  rzut {i}: ocena {oc[-1]:.2f}', flush=True)
        wyb = [int(np.argmax(oc))]
    Sk = KL[wyb[0]].reshape(-1, 2, 2)
    rz = Rzut(Sk, sale)
    W = uloz(K, rz)
    met = metryki(K, W, rz); met['rzut'] = wyb[0]; met['jedn'] = jedn
    p = zapisz(kat, K, W, Sk, met)
    print(json.dumps(met, ensure_ascii=False))
    print('zapisano', p)


if __name__ == '__main__':
    main()

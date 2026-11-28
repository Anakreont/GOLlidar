"""Pierwszy potok mapy 2D: obroty LD19 -> deskew z yaw IMU -> ICP point-to-line scan-to-submap."""
import sys, numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))
import sprawdz as S_

# parametry dopasowania (strojone na rzucie CAD domu Piotra, sesja 25.11 18:57)
PAR = dict(imu_w=200.0, promien=8.0, kf_dl=0.25, kf_kat=8.0, co_ktory=2, przechyl_max=12.0, huber=0.03, max_d=0.3,
           intens=0.3, sub_max=150, iters=15, luka_s=2.0, luka_frac=0.6, luka_rms=0.05,
           # 26.11 (opinie innych AI): degeneracja ICP w korytarzach, pas wysokosci zamiast odrzucania przechylu
           deg=True, deg_prog=40.0,        # wartosc wlasna bloku przesuniecia ponizej progu = slaby kierunek -> ruch z przewidywania
           pas=True, pas_gora=0.8, pas_dol=0.9,   # punkty wyzej/nizej od glowicy (sufit, meble, podloga) odrzucane przy przechyle
           przechyl_zm=99.0,               # skan odrzucany, gdy przechyl zmienia sie w trakcie obrotu o wiecej (wkladanie w przekroj)
           yaw_zyro=False,
           luka_kurs=9.0,                  # po luce: szukanie kursu +-9 st. (BNO po przekroju), 0 = wylaczone
           postoj_mg=25.0,
           # 28.11 (Test dom 4, 2 pietro99): po wyjsciu z ciasnego pomieszczenia trasa byla przesunieta o ~11 cm / 2 st.
           # i hol rysowal sie drugi raz. Kotwica: skan dopasowany dodatkowo TYLKO do starych kluczowych skanow
           # (starszych niz kotw_wiek s); pewna i powtarzalna roznica = dryf -> pozycja wraca na stara mape,
           # a blad jest rozkladany wstecz na odcinek, na ktorym powstal.
           deskew_v=1.0,                   # 28.11: deskew takze przesuniecia (ruch glowicy w czasie obrotu, ~5 cm przy 0,5 m/s); 0 = tylko obrot.
                                           # A/B na 6 sesjach: grubosc scian (mediana) 22->18, 21->14, 28->22, 22->18 mm; znak -1 pogarsza (29-33 mm)
           przechyl_sledz=0.0,             # 25 = skany przechylone 12-25 st. po rzutowaniu na poziom sluza do samego sledzenia (nie trafiaja do mapy).
                                           # WYLACZONE: SP162 2 pietro lepiej (3 -> 2 fragmenty, mapa ciagla), ale Test dom 4 'zewnatrz' duzo gorzej
                                           # (fragment z zewnatrz nalozony na wnetrze). Z bramka jakosci (skos_frac=0.7) 'zewnatrz' dobrze, 2 pietro gorzej (5 fragm.)
           kotwica=True, kotw_wiek=20.0, kotw_inl=0.45, kotw_max=0.6, kotw_max_kat=10.0, kotw_ile=3)                 # odch. std |przysp.| w 0,5 s ponizej progu = operator stoi -> przewidywany ruch zerowy                  # IMU SHTP: kurs z calki zyroskopu; wylaczone - przy zanikach zasilania (USB) gubi skoki, ktore kwaternion zachowuje
STAT = {}
import os as _os, json as _json
if _os.environ.get('LIDAR_PAR'):            # proby porownawcze: LIDAR_PAR='{"deg": false}' python3 przetworz.py ...
    PAR.update(_json.loads(_os.environ['LIDAR_PAR']))
LOG_LUK = []
TEST_LUKI = []   # testy: [(t_od, czas_s, dx, dy, dth_st)] - wyciecie skanow i sztuczny blad pozycji po luce


def wczytaj(kat):
    z = np.load(kat + '_dek.npz')
    return {k: z[k] for k in z.files}

WERSJA_DEK = 4   # zmiana dekodowania -> stara pamiec podreczna dek.npz liczona od nowa

def wczytaj_lub_dekoduj(kat):
    """Zdekodowane dane z pamieci podrecznej <sesja>/_wynik/dek.npz (odswiezanej, gdy lidar.bin jest nowszy)."""
    import os
    from dekoduj import dekoduj
    p = os.path.join(kat, '_wynik', 'dek.npz')
    if os.path.exists(p) and os.path.getmtime(p) >= os.path.getmtime(os.path.join(kat, 'lidar.bin')):
        z = np.load(p)
        if (int(z['wersja_dek']) if 'wersja_dek' in z.files else 0) == WERSJA_DEK:
            return {k: z[k] for k in z.files}
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return dekoduj(kat, p)

def pion_z_info(kat):
    """Pion zapisany przyciskiem 'Zeruj pion' (info.txt), jesli byl ustawiony."""
    import os
    try:
        d = dict(l.strip().split('=', 1) for l in open(os.path.join(kat, 'info.txt'), encoding='utf-8') if '=' in l)
        p, r = float(d['imu_pitch0']), float(d['imu_roll0'])
        return (p, r) if (p or r) else None
    except (OSError, KeyError, ValueError):
        return None


def przerwy(kat, t0):
    """Przedzialy PAUZA_START..PAUZA_KONIEC w sekundach od poczatku nagrania (z znaczniki.txt)."""
    import os
    p = os.path.join(kat, 'znaczniki.txt'); out = []; a = None
    if not os.path.exists(p): return out
    for l in open(p, encoding='utf-8'):
        k = l.split('\t')
        if len(k) < 2 or not k[0].isdigit(): continue
        t = (S_.popraw_czas(kat, int(k[0])) - t0) / 1e9
        if k[1].startswith('PAUZA_START'): a = t
        elif k[1].startswith('PAUZA_KONIEC') and a is not None: out.append((a - 0.5, t + 1.0)); a = None
    if a is not None: out.append((a - 0.5, 1e12))
    return out


def obroty(z, rmin=0.3, rmax=9.0, pion=None, pion_pas=None):
    """Dzieli pakiety na obroty; kazdy punkt z czasem z interpolacji wewnatrz pakietu."""
    a0, a1, d, tl = z['a0'], z['a1'], z['d'], z['tl']
    inten = z['i'].astype(float)
    # slabe odbicia (przez szybe, po skosie, ciemne powierzchnie): intensywnosc ponizej 30 % mediany dla tej odleglosci
    przedz = np.arange(0, 16.5, 0.5)
    dd, ii = d.ravel(), inten.ravel()
    med = np.array([np.median(ii[(dd >= a) & (dd < a + 0.5)]) if np.sum((dd >= a) & (dd < a + 0.5)) > 50 else np.nan for a in przedz])
    med = np.where(np.isfinite(med), med, np.nanmin(med) if np.isfinite(med).any() else 0)
    krok = ((a1 - a0) % 360) / 11
    ang = (a0[:, None] + krok[:, None] * np.arange(12)) % 360
    dtp = np.median(np.diff(tl))
    tp = tl[:, None] + (np.arange(12) - 11) * dtp / 12      # znacznik pakietu ~ ostatni punkt
    nowy = np.where(np.diff(a0) < -180)[0] + 1
    gr = np.split(np.arange(len(a0)), nowy)
    gr = gr[1:-1]
    # operator (glowa, plecak, tyczka, rece przy niesieniu w dloni) porusza sie razem z czujnikiem i zatruwa
    # dopasowanie. Wycinki katowe (10 st.), w ktorych w ok. 10 s wiekszosc obrotow ma punkt blizej niz 0,7 m,
    # uznajemy za operatora i odrzucamy w tym oknie; dodatkowo zawsze tyl glowicy (130-230 st.).
    blisko = np.zeros((len(gr), 36), bool)
    odl_bl = np.full((len(gr), 36), np.nan)      # mediana bliskiej odleglosci w wycinku
    for j, g in enumerate(gr):
        A, D = ang[g].ravel(), d[g].ravel()
        k = (D > 0.1) & (D < 0.7)
        sek = (A[k] // 10).astype(int) % 36
        blisko[j, sek] = True
        if PAR.get('operator_ruch'):
            for s_ in np.unique(sek):
                odl_bl[j, s_] = np.median(D[k][sek == s_])
    # operator_ruch: operator obraca sie razem z glowica - w ukladzie czujnika stoi w tym samym wycinku w STALEJ
    # odleglosci. Sciana ciasnego pomieszczenia przy obrocie glowicy zmienia odleglosc w wycinku. Gdy w oknie byl
    # obrot >= 25 st., wycinki o niestalej odleglosci (rozrzut 10-90 % > 8 cm) nie sa uznawane za operatora.
    yaw_gr = None
    if PAR.get('operator_ruch') and len(gr) and not bool(z.get('bez_imu', False)):
        yaw_gr = np.degrees(np.interp(np.array([tl[g].mean() for g in gr]), z['ti'], np.unwrap(np.radians(np.asarray(z['yaw'], float)))))
    okno = 100
    maska = np.zeros_like(blisko)
    for j0 in range(0, len(gr), okno // 2):
        sl = slice(max(0, j0 - okno // 2), j0 + okno)
        b = blisko[sl]
        if not len(b): continue
        mk = b.mean(0) > 0.6
        if yaw_gr is not None and mk.any() and np.ptp(yaw_gr[sl]) >= 25.0:
            od = odl_bl[sl]
            for s_ in np.where(mk)[0]:
                o = od[:, s_][np.isfinite(od[:, s_])]
                if len(o) >= 10 and np.subtract(*np.percentile(o, [90, 10])) > 0.08:
                    mk[s_] = False
        maska[j0: j0 + okno // 2] = mk
    # rzutowanie na poziom: przy przechyle wiazka biegnie skosnie i odleglosc do pionowej sciany jest za duza o 1/cos(elewacji).
    # Os BNO wzgledem LiDAR-u ustalona z przekrojow: roll +85 st. = kierunek 0 st. LD19 w dol, wiec elewacja wiazki o kacie A
    # wynosi ok. -roll*cos(A) + pitch*sin(A) (przechyl liczony od pionu z postoju / Zeruj pion).
    rzut = pion is not None and not bool(z.get('bez_imu', False))
    imu = pion_pas is not None and not bool(z.get('bez_imu', False))
    if imu:
        ti = z['ti']; pp = pion if pion is not None else pion_pas; pr = z['pitch'] - pp[0]; rr = z['roll'] - pp[1]
    n_pas = [0, 0]
    out = []
    for j, g in enumerate(gr):
        A, D, Tt = ang[g].ravel(), d[g].ravel(), tp[g].ravel()
        I_ = inten[g].ravel()
        slabe = I_ < PAR['intens'] * np.interp(D, przedz + 0.25, med)
        pasm = np.ones(len(D), bool)
        if imu:
            p_ = np.radians(np.interp(Tt, ti, pr)); r_ = np.radians(np.interp(Tt, ti, rr)); Ar = np.radians(A)
            elew = -r_ * np.cos(Ar) + p_ * np.sin(Ar)
            if PAR['pas']:
                # wysokosc punktu wzgledem glowicy (ok. 1,9 m nad podloga): przy przechyle daleka wiazka trafia
                # w sufit, lampy, lawki lub podloge - takie punkty nie sa sciana na wysokosci skanu
                h = D * np.sin(np.clip(elew, -1.2, 1.2))
                pasm = (h < PAR['pas_gora']) & (h > -PAR['pas_dol'])
                n_pas[0] += int((~pasm & (D > rmin)).sum()); n_pas[1] += len(D)
            # przechyl_sledz: skan przechylony ponad przechyl_max nie jest odrzucany, tylko rzutowany na poziom
            # (odleglosc do pionowej sciany * cos elewacji) i uzywany do samego sledzenia (nie trafia do mapy)
            if rzut or (PAR.get('przechyl_sledz', 0) and np.degrees(np.sqrt(p_ ** 2 + r_ ** 2).max()) > PAR['przechyl_max']):
                D = D * np.cos(np.clip(elew, -0.6, 0.6))
        m = (D > rmin) & (D < rmax) & ~slabe & pasm
        m &= ~((D < 0.7) & (A > 130) & (A < 230))
        m &= ~((D < 0.9) & maska[j][(A // 10).astype(int) % 36])
        if m.sum() < 150: continue
        out.append((Tt[m], np.radians(A[m]), D[m]))
    STAT['pas_odrzucone'] = n_pas[0] / max(1, n_pas[1])
    return out

def normalne(p):
    """Normalne z sasiadow w kolejnosci kata (punkty i-2..i+2); NaN gdy brak liniowosci."""
    n = np.full_like(p, np.nan)
    for i in range(2, len(p) - 2):
        q = p[i - 2:i + 3]
        if np.max(np.linalg.norm(q - p[i], axis=1)) > 0.25: continue
        c = np.cov((q - q.mean(0)).T)
        w, v = np.linalg.eigh(c)
        if w[0] > 0.1 * w[1]: continue
        n[i] = v[:, 0]
    return n

def icp(src, tgt, ntg, tree, T0, iters=15, huber=0.05, max_d=0.5, prior=None, deg=None):
    """deg = (x0, y0, prog): w kierunkach przesuniecia slabo okreslonych przez sciany (wartosc wlasna H ponizej
    progu - typowo wzdluz korytarza) pozycja trzymana przy przewidywaniu (x0, y0) zamiast swobodnie dryfowac."""
    x, y, th = T0
    for _ in range(iters):
        c, s = np.cos(th), np.sin(th)
        q = src @ np.array([[c, s], [-s, c]]) + [x, y]
        dist, j = tree.query(q, distance_upper_bound=max_d)
        m = np.isfinite(dist)
        if m.sum() < 50: return None
        qm, nm, tm, sm = q[m], ntg[j[m]], tgt[j[m]], src[m]
        r = np.sum((qm - tm) * nm, 1)
        w = np.where(np.abs(r) < huber, 1.0, huber / np.abs(r))
        # jakobian dla (dx, dy, dth)
        dq = np.stack([-s * sm[:, 0] - c * sm[:, 1], c * sm[:, 0] - s * sm[:, 1]], 1)
        J = np.stack([nm[:, 0], nm[:, 1], np.sum(nm * dq, 1)], 1)
        H = (J * w[:, None]).T @ J; g = (J * w[:, None]).T @ r
        if prior is not None:
            H[2, 2] += prior[1]; g[2] += prior[1] * (th - prior[0])
        if deg is not None:
            ew, ev_ = np.linalg.eigh(H[:2, :2])
            for k in range(2):
                if ew[k] < deg[2]:
                    vv = ev_[:, k]; lam = deg[2] - ew[k]
                    H[:2, :2] += lam * np.outer(vv, vv); g[:2] += lam * vv * (vv @ (np.array([x, y]) - deg[:2]))
        try: dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
        except np.linalg.LinAlgError: return None
        x, y, th = x + dx[0], y + dx[1], th + dx[2]
        if np.abs(dx).max() < 1e-4: break
    ev = np.linalg.eigvalsh(((J * w[:, None]).T @ J)[:2, :2])     # bez sztucznego wiezu - do oceny degeneracji
    return np.array([x, y, th]), np.sqrt(np.mean(r ** 2 * w)), ev, m.mean()

def icp_widok(src, tgt, ntg, tree, T0, iters=25, huber=0.03, max_d=0.5, prior=None, deg=None):
    """ICP do starej mapy z kontrola widocznosci: normalne celu sa skierowane do czujnika, ktory je widzial;
    punkt celu liczy sie tylko, gdy to lico jest zwrocone takze do obecnej pozycji (inaczej skan z pokoju A
    dociagalby sie do drugiej strony tej samej sciany, widzianej kiedys z pokoju B).
    Zwraca (T, rms, ev, udzial punktow skanu zgodnych z mapa do 5 cm) albo None."""
    x, y, th = T0
    for _ in range(iters):
        c, s = np.cos(th), np.sin(th)
        q = src @ np.array([[c, s], [-s, c]]) + [x, y]
        dist, j = tree.query(q, distance_upper_bound=max_d)
        m = np.isfinite(dist)
        if m.sum() < 50: return None
        jj = j[m]; qm = q[m]
        prom = np.array([x, y]) - qm; prom /= np.maximum(np.linalg.norm(prom, axis=1), 1e-6)[:, None]
        wid = np.sum(ntg[jj] * prom, 1) > 0.15
        if wid.sum() < 50: return None
        idx = np.where(m)[0][wid]
        qm, nm, tm_, sm = q[idx], ntg[j[idx]], tgt[j[idx]], src[idx]
        r = np.sum((qm - tm_) * nm, 1)
        w = np.where(np.abs(r) < huber, 1.0, huber / np.maximum(np.abs(r), 1e-9))
        dq = np.stack([-s * sm[:, 0] - c * sm[:, 1], c * sm[:, 0] - s * sm[:, 1]], 1)
        J = np.stack([nm[:, 0], nm[:, 1], np.sum(nm * dq, 1)], 1)
        H = (J * w[:, None]).T @ J; g = (J * w[:, None]).T @ r
        ev = np.linalg.eigvalsh(H[:2, :2])
        if deg is not None:                       # slaby kierunek: pozycja trzymana przy (x0, y0)
            ew, ev_ = np.linalg.eigh(H[:2, :2])
            for k in range(2):
                if ew[k] < deg[2]:
                    vv = ev_[:, k]; lam = deg[2] - ew[k]
                    H[:2, :2] += lam * np.outer(vv, vv); g[:2] += lam * vv * (vv @ (np.array([x, y]) - np.asarray(deg[:2])))
        if prior is not None:
            H[2, 2] += prior[1]; g[2] += prior[1] * (th - prior[0])
        try: dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
        except np.linalg.LinAlgError: return None
        x, y, th = x + dx[0], y + dx[1], th + dx[2]
        if np.abs(dx).max() < 1e-4: break
    return np.array([x, y, th]), np.sqrt(np.mean(r ** 2 * w)), ev, float(np.sum(np.abs(r) < 0.05)) / len(src)

def _do_czujnika(Pw, Nw, T):
    """Normalne skierowane do pozycji czujnika (znak nie zmienia dopasowania punkt-prosta)."""
    return Nw * np.where(np.sum(Nw * (T[:2] - Pw), 1) < 0, -1.0, 1.0)[:, None]

def mapa(kat, znak=+1, t_od=0, t_do=1e9, imu_w=None, pion=None, log=False, segmenty=None, promien=None, dane=None, rzutuj=True):
    imu_w = PAR['imu_w'] if imu_w is None else imu_w; promien = PAR['promien'] if promien is None else promien
    z = dane if dane is not None else wczytaj(kat)
    if bool(z.get('bez_imu', False)): imu_w = 0.0
    ti, yaw = z['ti'], np.radians(z['yaw'])
    if PAR['yaw_zyro'] and 'yaw_zyro' in z:
        yaw = np.radians(z['yaw_zyro'])
    pit, rol = z['pitch'], z['roll']
    if pion is None:
        pion = pion_z_info(kat)
    if pion is None:
        m10 = ti < ti[0] + 10; pion = (np.median(pit[m10]), np.median(rol[m10]))
    sk = obroty(z, pion=pion if rzutuj else None, pion_pas=pion)
    am = np.linalg.norm(np.asarray(z['acc'], float), axis=1)
    c1 = np.concatenate([[0], np.cumsum(am)]); c2 = np.concatenate([[0], np.cumsum(am ** 2)])
    def stoi(t):
        a_, b_ = np.searchsorted(ti, [t - 0.5, t]); n_ = b_ - a_
        if n_ < 10 or not am.any(): return False
        m_ = (c1[b_] - c1[a_]) / n_
        return (c2[b_] - c2[a_]) / n_ - m_ * m_ < PAR['postoj_mg'] ** 2
    STAT.update(skany=len(sk), slabe=0, przechyl=0, zmiana=0, ev=[], kotwice=[], skos_sledzone=0)
    kf_t = []; t_kotw = None; kand = []        # kotwica: czasy kluczowych, ostatnie pewne zakotwiczenie, kandydaci poprawki
    prz = np.degrees(np.arccos(np.clip(np.cos(np.radians(pit - pion[0])) * np.cos(np.radians(rol - pion[1])), -1, 1)))
    poses, kf, info = [], [], []
    T = np.zeros(3); v = np.zeros(2); ostT = None; yaw_ost = None
    seg_ost = None; kfw = []   # kluczowe w ukladzie swiata (punkty, normalne, pozycja, segment)
    pauzy = przerwy(kat, int(z['t0'])); po_pauzie = False
    frag = []; frag_luka = {}; fr_id = 0; t_ok = None; zaburz = None   # fragmenty: nowy po luce w sledzeniu (nie po PAUZIE)
    n_sk = max(1, len(sk)); krok_p = max(200, n_sk // 50)
    for i_sk, (tt, ang, D) in enumerate(sk):
        if i_sk % krok_p == 0:
            print(f'POSTEP mapa {i_sk}/{n_sk}', flush=True)    # przegladarka liczy z tego czas do konca
        tm = tt.mean()
        if tm < t_od or tm > t_do: continue
        if any(pa <= tm <= pb for pa, pb in pauzy):     # PAUZA: zdjety plecak, przekroj z reki - nie do rzutu
            info.append((tm, 'pauza')); po_pauzie = True; continue
        seg = 0 if segmenty is None else int(np.searchsorted(segmenty, tm))
        if seg != seg_ost:
            kf_seg = []; seg_ost = seg
        pz = np.interp(tt, ti, prz)
        skos = pz.max() > PAR['przechyl_max']
        if skos and not (PAR.get('przechyl_sledz', 0) and pz.max() <= PAR['przechyl_sledz'] and kf_seg):
            info.append((tm, 'przechyl')); STAT['przechyl'] += 1; continue
        if skos: STAT['skos_sledzone'] = STAT.get('skos_sledzone', 0) + 1
        if pz.max() - pz.min() > PAR['przechyl_zm']: info.append((tm, 'przechyl')); STAT['zmiana'] += 1; continue
        tl_ = [l for l in TEST_LUKI if l[0] <= tm <= l[0] + l[1]]
        if tl_: info.append((tm, 'test')); zaburz = tl_[0]; continue
        yp = np.interp(tt, ti, yaw); ym = np.interp(tm, ti, yaw)
        a = -ang + znak * (yp - ym)              # LD19 zgodnie z zegarem -> matematycznie przeciwnie; deskew obrotu
        p = np.stack([D * np.cos(a), D * np.sin(a)], 1)
        if PAR.get('deskew_v') and len(poses) and t_ok is not None and tm - t_ok < 0.3:
            # deskew przesuniecia: w czasie obrotu (0,1 s) glowica przechodzi ~5 cm; v = ruch na skan w ukladzie swiata
            c_, s__ = np.cos(T[2]), np.sin(T[2]); vl = np.array([c_ * v[0] + s__ * v[1], -s__ * v[0] + c_ * v[1]])
            p = p + float(PAR['deskew_v']) * np.outer((tt - tm) / max(tm - t_ok, 0.05), vl)
        w = np.exp(-(pz.mean() ** 2) / (2 * 8 ** 2))
        def dodaj_kf():
            n = normalne(p); mk = np.isfinite(n[:, 0]); c, s_ = np.cos(T[2]), np.sin(T[2]); Rm = np.array([[c, s_], [-s_, c]])
            Pw_ = p[mk] @ Rm + T[:2]
            kf.append((T.copy(), p, n)); frag.append((seg, fr_id)); kf_t.append(tm)
            kf_seg.append([T.copy(), Pw_, _do_czujnika(Pw_, n[mk] @ Rm, T), tm, len(kf) - 1, p[mk], n[mk]])
        if not kf_seg:
            dodaj_kf(); poses.append((tm, *T, w)); yaw_ost = ym; t_ok = tm; continue
        dth = znak * (ym - yaw_ost)
        if zaburz is not None:
            T = T + np.array([zaburz[2], zaburz[3], np.radians(zaburz[4])]); zaburz = None
        luka = (not po_pauzie and t_ok is not None and tm - t_ok > PAR['luka_s'])
        if luka or stoi(tm): v = np.zeros(2)
        T0 = np.array([T[0] + v[0], T[1] + v[1], T[2] + dth])
        poz = np.array([x[0][:2] for x in kf_seg])
        bl = np.where(np.hypot(*(poz - T0[:2]).T) < promien)[0]
        wyb = sorted(set(bl[-PAR['sub_max']:].tolist()) | set(range(max(0, len(kf_seg) - 10), len(kf_seg))))
        P = np.vstack([kf_seg[i][1] for i in wyb]); N = np.vstack([kf_seg[i][2] for i in wyb])
        ps = p[::PAR['co_ktory']]
        tree = cKDTree(P)
        if po_pauzie:
            # po pauzie operator stoi mniej wiecej w tym samym miejscu: szukamy pozycji w promieniu ok. 0,6 m
            v = np.zeros(2); wyn = None
            for ddx in (0, -0.4, 0.4):
                for ddy in (0, -0.4, 0.4):
                    r_ = icp(ps, P, N, tree, T0 + [ddx, ddy, 0], prior=(T0[2], imu_w), max_d=0.8, iters=25)
                    if r_ is not None and (wyn is None or r_[3] - 2 * r_[1] > wyn[3] - 2 * wyn[1]): wyn = r_
            po_pauzie = False
            if wyn is None: info.append((tm, 'icp')); continue
            Tn, rms, ev, frac = wyn
        else:
            dg = np.array([T0[0], T0[1], PAR['deg_prog']]) if (PAR['deg'] and not luka) else None
            wyn = icp(ps, P, N, tree, T0, prior=(T0[2], imu_w), huber=PAR['huber'], max_d=PAR['max_d'], iters=PAR['iters'], deg=dg)
            if luka and PAR['luka_kurs'] > 0:
                # po przekroju (glowica na boku) kurs z BNO bywa przesuniety o kilka stopni (zszywanie poprawialo
                # fragmenty o 5-6 st.): szukamy tez obroconych startow ze slabym wiezem kursu
                for dk in np.radians(np.arange(-PAR['luka_kurs'], PAR['luka_kurs'] + 0.1, 3.0)):
                    if abs(dk) < 1e-6: continue
                    r_ = icp(ps, P, N, tree, T0 + [0, 0, dk], prior=(T0[2] + dk, 5.0), huber=PAR['huber'], max_d=0.5, iters=25)
                    if r_ is not None and (wyn is None or r_[3] - 2 * r_[1] > wyn[3] - 2 * wyn[1] + 0.02):
                        wyn = r_
            if wyn is not None:
                STAT.setdefault('ev', []).append(wyn[2][0])
                if wyn[2][0] < PAR['deg_prog']: STAT['slabe'] += 1
            if luka:
                # po luce w sledzeniu (przechyl, brak IMU): sprawdzamy, czy skan pewnie pasuje do starej mapy.
                # Jesli nie - nie doklejamy na sile (to przesuwalo kuchnie/korytarz), tylko zaczynamy nowy
                # fragment od zera; fragmenty zszywa potem posegraf.zszyj po wspolnych scianach.
                q = None if wyn is None else (wyn[3], wyn[1], float(np.linalg.norm(wyn[0][:2] - T0[:2])))
                if q is not None:
                    # jednoznacznosc: start z +-0,5 m nie moze dawac innej, rownie dobrej pozycji (rownolegle sciany)
                    for ddx, ddy in ((0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5)):
                        Ts = np.array([T0[0] + ddx, T0[1] + ddy, wyn[0][2]])
                        r_ = icp(ps, P, N, tree, Ts, prior=(wyn[0][2], imu_w), huber=PAR['huber'], max_d=0.6, iters=25)
                        if (r_ is not None and np.linalg.norm(r_[0][:2] - wyn[0][:2]) > 0.15 and r_[3] > wyn[3] - 0.05
                                and r_[1] < 1.5 * wyn[1] + 0.005):
                            q = (q[0], q[1], q[2], 'niejednoznaczne'); break
                LOG_LUK.append((tm, tm - t_ok, q))
                if q is None or len(q) > 3 or q[0] < PAR['luka_frac'] or q[1] > PAR['luka_rms'] or q[2] > 0.5 + 0.6 * (tm - t_ok):
                    fr_id += 1; info.append((tm, 'fragment')); frag_luka[fr_id] = tm - t_ok
                    # start fragmentu w najlepszej znanej pozycji (wynik ICP, jesli byl) - zszywanie zmienia ja
                    # tylko wtedy, gdy znajdzie wyraznie lepsze i jednoznaczne dopasowanie
                    T = wyn[0].copy() if wyn is not None else np.array([T[0], T[1], T0[2]]); kf_seg = []
                    dodaj_kf(); poses.append((tm, *T, w)); yaw_ost = ym; t_ok = tm; continue
            if wyn is None: info.append((tm, 'icp')); continue
            Tn, rms, ev, frac = wyn
            if skos and PAR.get('skos_frac', 0) > 0 and (frac < PAR['skos_frac'] or rms > PAR.get('skos_rms', 0.03) or ev[0] < PAR['deg_prog']):
                # przechylony skan sluzy do sledzenia tylko, gdy pewnie pasuje do mapy; inaczej jak dawniej - odrzucony
                STAT['skos_sledzone'] -= 1; info.append((tm, 'przechyl')); STAT['przechyl'] += 1; continue
            if frac < 0.3 or np.linalg.norm(Tn[:2] - T0[:2]) > 0.4: info.append((tm, 'skok')); Tn = T0
        v = 0.5 * v + 0.5 * (Tn[:2] - T[:2]); T = Tn; yaw_ost = ym; t_ok = tm
        poses.append((tm, *T, w))
        if PAR['kotwica']:
            stare = [i for i in bl if kf_seg[i][3] < tm - PAR['kotw_wiek']]
            rk = None
            if len(stare) >= 3:
                if len(stare) > 60:
                    stare = [stare[i] for i in np.argsort(np.hypot(*(poz[stare] - T[:2]).T))[:60]]
                Ps = np.vstack([kf_seg[i][1] for i in stare]); Ns = np.vstack([kf_seg[i][2] for i in stare])
                rk = icp_widok(ps, Ps, Ns, cKDTree(Ps), T.copy(), prior=(T[2], 20.0))
            dk = None
            if (rk is not None and rk[3] >= PAR['kotw_inl'] and rk[1] <= 0.03 and rk[2][0] >= 1.5 * PAR['deg_prog']):
                dk = rk[0] - T
                if np.hypot(dk[0], dk[1]) > PAR['kotw_max'] or abs(dk[2]) > np.radians(PAR['kotw_max_kat']): dk = None
            if dk is None:
                kand = []
            elif np.hypot(dk[0], dk[1]) < 0.03 and abs(dk[2]) < np.radians(0.7):
                t_kotw = tm; kand = []                      # zgodne ze stara mapa - stad liczymy ewentualny dryf
            else:
                kand.append((dk, max(kf_seg[i][3] for i in stare)))
                K = np.array([k_[0] for k_ in kand[-PAR['kotw_ile']:]])
                if len(kand) >= PAR['kotw_ile'] and np.ptp(K[:, 0]) < 0.05 and np.ptp(K[:, 1]) < 0.05 and np.ptp(K[:, 2]) < np.radians(1.5):
                    # pewny, powtarzalny dryf wzgledem starej mapy: rozkladamy go liniowo w czasie od ostatniego
                    # zakotwiczenia (albo od chwili, gdy to miejsce bylo ostatnio widziane) do teraz
                    t_a = max(kand[-1][1], t_kotw if t_kotw is not None else -1e9)
                    Tc = rk[0]; dth_ = Tc[2] - T[2]
                    pa = np.array(poses); ia = max(0, np.searchsorted(pa[:, 0], t_a, side='right') - 1); A = pa[ia, 1:3].copy()
                    def _R(a_): return np.array([[np.cos(a_), -np.sin(a_)], [np.sin(a_), np.cos(a_)]])
                    tres = Tc[:2] - (A + _R(dth_) @ (T[:2] - A))
                    def _popr(Tp, t_):
                        w_ = float(np.clip((t_ - t_a) / max(tm - t_a, 1e-6), 0, 1))
                        xy = A + _R(w_ * dth_) @ (np.asarray(Tp[:2]) - A) + w_ * tres
                        return np.array([xy[0], xy[1], Tp[2] + w_ * dth_])
                    for i_ in range(ia + 1, len(poses)):
                        t_, x_, y_, th_, ww_ = poses[i_]; n_ = _popr((x_, y_, th_), t_); poses[i_] = (t_, n_[0], n_[1], n_[2], ww_)
                    for e in kf_seg:
                        if e[3] > t_a:
                            e[0] = _popr(e[0], e[3]); c, s_ = np.cos(e[0][2]), np.sin(e[0][2]); Rm = np.array([[c, s_], [-s_, c]])
                            e[1] = e[5] @ Rm + e[0][:2]; e[2] = _do_czujnika(e[1], e[6] @ Rm, e[0])
                            kf[e[4]] = (e[0].copy(), kf[e[4]][1], kf[e[4]][2])
                    STAT['kotwice'].append((float(tm), float(tm - t_a), float(np.hypot(*(Tc[:2] - T[:2]))), float(np.degrees(dth_))))
                    T = np.array(poses[-1][1:4]); t_kotw = tm; kand = []
        Tk = kf_seg[-1][0]
        if not skos and (np.hypot(*(T[:2] - Tk[:2])) > PAR['kf_dl'] or abs(T[2] - Tk[2]) > np.radians(PAR['kf_kat'])):
            dodaj_kf()
    mapa.frag = np.array(frag, int).reshape(-1, 2); mapa.frag_luka = frag_luka
    return np.array(poses), kf, info, pion

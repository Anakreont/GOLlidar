"""Porownanie mapy bez kotwicy i z kotwica (slam.PAR['kotwica']) na podanych sesjach - bez zapisu wynikow sesji.
Miara spojnosci: dla punktow scian kazdego kluczowego skanu szukamy tego samego lica (zgodna normalna) w skanach
odleglych w czasie o > 25 s; 'podwojne' = odleglosc od tamtego lica 5-30 cm (sciana narysowana dwa razy).
    python3 test_kotwica.py <sesja> [<sesja> ...]
"""
import os, sys, time
import numpy as np
from scipy.spatial import cKDTree
TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..'))
import slam


def spojnosc(kf, kt):
    W = []
    for T, p, n in kf:
        m = np.isfinite(n[:, 0]); c, s = np.cos(T[2]), np.sin(T[2]); R = np.array([[c, s], [-s, c]])
        Pw = p[m] @ R + T[:2]; W.append((Pw, slam._do_czujnika(Pw, n[m] @ R, T)))
    kt = np.asarray(kt); r_all = []
    for i in range(len(kf)):
        o = np.where(np.abs(kt - kt[i]) > 25)[0]
        if not len(o) or not len(W[i][0]): continue
        o = o[np.hypot(*(np.array([kf[j][0][:2] for j in o]) - kf[i][0][:2]).T) < 10]
        if not len(o): continue
        Q = np.vstack([W[j][0] for j in o]); N = np.vstack([W[j][1] for j in o])
        d, j = cKDTree(Q).query(W[i][0], distance_upper_bound=0.3)
        m = np.isfinite(d)
        if not m.any(): continue
        zg = np.sum(N[j[m]] * W[i][1][m], 1) > 0.7
        r_all.append(np.abs(np.sum((W[i][0][m][zg] - Q[j[m]][zg]) * N[j[m]][zg], 1)))
    if not r_all: return None
    r = np.concatenate(r_all)
    return dict(pkt=len(r), mediana_mm=float(np.median(r) * 1000), p90_mm=float(np.percentile(r, 90) * 1000), podwojne_proc=float(np.mean(r > 0.05) * 100))


for kat in sys.argv[1:]:
    kat = kat.rstrip('/'); print('==', os.path.basename(kat), flush=True)
    try:
        z = slam.wczytaj_lub_dekoduj(kat)
        for kot in (False, True):
            slam.PAR['kotwica'] = kot; t0 = time.time()
            so = sys.stdout; sys.stdout = open(os.devnull, 'w')
            try: P, kf, info, pion = slam.mapa(kat, znak=-1, dane=z, rzutuj=False)
            finally: sys.stdout = so
            kt = [P[np.argmin(np.abs(P[:, 1:4] - Tk).sum(1)), 0] for Tk, _, _ in kf]
            s = spojnosc(kf, kt); K = slam.STAT.get('kotwice') or []
            if kot:
                import posegraf
                Xd = posegraf.dopracuj(kf, kt, log=lambda *a: None); kfd = [(Xd[i], kf[i][1], kf[i][2]) for i in range(len(kf))]
                sd = spojnosc(kfd, kt); dd = np.hypot(*(Xd[:, :2] - np.array([k[0][:2] for k in kf])).T)
                print(f"  + dopracowanie: mediana {sd['mediana_mm']:.0f} mm, 90% {sd['p90_mm']:.0f} mm, podwojne {sd['podwojne_proc']:.1f}%, ruszono {int((dd > 0.005).sum())} skanow, najw. {dd.max()*100:.0f} cm", flush=True)
            print(f"  kotwica={'tak' if kot else 'nie'}: skanow {len(P)}, kluczowych {len(kf)}, fragmentow {len(set(map(tuple, slam.mapa.frag.tolist())))}, "
                  + (f"te same lica po >25 s: mediana {s['mediana_mm']:.0f} mm, 90% {s['p90_mm']:.0f} mm, podwojne {s['podwojne_proc']:.1f}% ({s['pkt']} pkt)" if s else 'brak powrotow')
                  + f", poprawek {len(K)}" + (f" (najw. {max(k[2] for k in K)*100:.0f} cm, {max(abs(k[3]) for k in K):.1f} st)" if K else '')
                  + f", {time.time()-t0:.0f} s", flush=True)
    except Exception as e:
        print('  blad:', repr(e), flush=True)

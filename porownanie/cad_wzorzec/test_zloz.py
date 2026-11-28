"""Test mapa/zloz_kawalki.py (bez CAD): symulowane przejscie po rysunku, pociete na kawalki, kazdy kawalek z bledem
narastajacym jak w SLAM (przy kazdej przerwie losowe przesuniecie i obrot dodawane do poprzednich).
Ocena: po zlozeniu, bledy kawalkow wzgledem prawdy (po dopasowaniu calosci do prawdy na kawalku startowym).
Uzycie: python3 test_zloz.py rys.pkl [--kondygnacja N] [--dl 8] [--sk 1.0] [--sa 4] [--smieci 0.3] [--png]
"""
import os, sys, pickle, argparse, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'mapa'))
import test_uloz as T
import zloz_kawalki as Z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pkl'); ap.add_argument('--kondygnacja', type=int); ap.add_argument('--dl', type=float, default=8)
    ap.add_argument('--sk', type=float, default=1.0, help='odch. std. przesuniecia na przerwe [m]')
    ap.add_argument('--sa', type=float, default=4.0, help='odch. std. obrotu na przerwe [st.]')
    ap.add_argument('--smieci', type=float, default=0.0); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--png')
    a = ap.parse_args()
    D = pickle.load(open(a.pkl, 'rb'))
    for ki in ([a.kondygnacja] if a.kondygnacja is not None else range(len(D))):
        Kd = D[ki]
        if len(Kd['E']) < 4:
            continue
        t0 = time.time()
        poz, skany, etyk, E = T.przejscie(Kd, a.seed)
        rng = np.random.default_rng(a.seed + 1)
        n = len(poz); i = 0; nr = 1; K = []; PR = []
        A, Tt = 0.0, np.zeros(2); c_ref = poz[0]
        while i < n - 8:
            j = min(n, i + max(8, int(rng.uniform(0.5, 1.5) * a.dl / 0.4)))
            if nr > 1:
                A += np.radians(rng.normal(0, a.sa)); Tt = Tt + rng.normal(0, a.sk, 2)
            P = np.vstack(skany[i:j]); tr = poz[i:j]
            f = lambda X: (X - c_ref) @ Z._R(A).T + c_ref + Tt
            Pz = f(P)
            if a.smieci:
                m = int(a.smieci * len(P) / (1 - a.smieci))
                sr = f(tr)[rng.integers(0, len(tr), m // 40 + 1)] + rng.normal(0, 1, (m // 40 + 1, 2))
                kk = rng.uniform(0, np.pi, len(sr)); w = rng.integers(0, len(sr), m); u = rng.uniform(-.6, .6, m)
                Pz = np.vstack([Pz, sr[w] + u[:, None] * np.c_[np.cos(kk[w]), np.sin(kk[w])]])
            K.append({'nr': nr, 'P': Pz, 'trasa': f(tr), 'A': A}); PR.append(P)
            i = j + int(rng.integers(2, 6)); nr += 1
        S = Z.zloz(K, log=print if os.environ.get("LOG") else (lambda s: None))
        st = [s for s in S if s['stan'] == 'start'][0]; i0 = S.index(st)
        def poloz(i, s):
            return Z.przeksztalc(K[i]['P'][:len(PR[i])], np.array(s['c']), s['a'], np.array(s['t']))
        # dopasowanie calosci do prawdy na kawalku startowym (Procrustes)
        X0 = poloz(i0, st); Y0 = PR[i0]
        mx, my = X0.mean(0), Y0.mean(0); u_, _, vt = np.linalg.svd((X0 - mx).T @ (Y0 - my)); R = u_ @ vt
        bl = []
        bl_przed = []
        for i, s in enumerate(S):
            X = (poloz(i, s) - mx) @ R + my
            bl.append(float(np.median(np.linalg.norm(X - PR[i], axis=1))))
            Xp = (Z.przeksztalc(K[i]['P'][:len(PR[i])], np.array(s['c']), 0.0, np.zeros(2)) - mx) @ R + my
            bl_przed.append(float(np.median(np.linalg.norm(Xp - PR[i], axis=1))))
        if os.environ.get('LOG'):
            for i, s in enumerate(S):
                print(f"   K{K[i]['nr']:02d} {s['stan']:9s} obrot {np.degrees(s['a']):+6.1f}  potrzebny {np.degrees(K[i0]['A'] - K[i]['A']):+6.1f}  blad {bl[i]:.2f}")
        bl = np.array(bl); zl = np.array([s['stan'] != 'niepewny' for s in S])
        print(f"== {Kd['nazwa']}: {len(K)} kawalkow; przed: mediana bledu {np.median(bl_przed):.2f} m; "
              f"po: zlozonych {zl.sum()}/{len(S)}, z nich <20 cm: {(bl[zl] < 0.2).sum()}, BLEDNIE zlozonych (>=50 cm): "
              f"{(bl[zl] >= 0.5).sum()}; mediana bledu po {np.median(bl):.2f} m; {time.time() - t0:.0f} s")
        if a.png:
            import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(20, 9)); cm = plt.get_cmap('tab20')
            for i, s in enumerate(S):
                for x, (AA, TT) in enumerate([(0.0, np.zeros(2)), (s['a'], np.array(s['t']))]):
                    X = Z.przeksztalc(K[i]['P'][::3], np.array(s['c']), AA, TT)
                    ax[x].scatter(*X.T, s=0.2, color=cm(i % 20)); ax[x].text(*X.mean(0), f"K{K[i]['nr']}", fontsize=7)
            ax[0].set_title('przed (bledy jak ze SLAM)'); ax[1].set_title('po zlozeniu kawalkow miedzy soba')
            for x in ax: x.set_aspect('equal')
            fig.savefig(a.png, dpi=100, bbox_inches='tight')


if __name__ == '__main__':
    main()

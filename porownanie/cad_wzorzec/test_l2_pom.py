"""Test pomieszczen z chmury L2 (l2/l2_rzut.py) na rysunkach CAD: z rzutu budowany jest sztuczny budynek 3D
(sciany od podlogi do sufitu, nadproza nad drzwiami 2,05 m, okna z parapetem 0,9 m i nadprozem 2,0 m, podloga, sufit, szafy przy scianach 2,0-2,4 m),
trasa operatora przez wszystkie pomieszczenia (jak test_uloz.przejscie), szum 2 cm. Wynik porownywany z
powierzchniami z rysunku (jak symulacja.py: pomieszczenia rozdzielone w rysunku tylko linia - razem).
Uzycie: python3 test_l2_pom.py rys.pkl [--kondygnacja N] [--szafy 0.3] [--png]
"""
import os, sys, pickle, argparse, subprocess, json, tempfile, time, numpy as np
from scipy import ndimage
TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TU); sys.path.insert(0, os.path.join(TU, '..', '..', 'mapa'))
import pomieszczenia as PM
import symulacja as SY
import test_uloz as T

K_ = 0.03
H = 2.70


def zapisz_pcd(plik, P):
    P = P.astype(np.float32)
    with open(plik, 'wb') as f:
        f.write(f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(P)}\nHEIGHT 1\n"
                f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(P)}\nDATA binary\n".encode())
        f.write(P.tobytes())


def budynek(Kd, szafy=0.3, seed=0):
    rng = np.random.default_rng(seed)
    S = Kd['S']
    Pp = np.array([p for s in S for p in s]); m0 = Pp.min(0) - 2
    shape = tuple(np.ceil((Pp.max(0) + 2 - m0) / K_).astype(int))
    G0 = np.zeros(shape, bool)
    for a, b in S:
        PM.rysuj_odcinki(G0, np.array([(a - m0) / K_, (b - m0) / K_]))
    G = ndimage.binary_dilation(G0)
    obw = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(G, 60), np.ones((45, 45)))[60:-60, 60:-60])
    drzwi = np.zeros(shape, bool)
    for a, b in Kd['DR']:
        if (b - a).max() > 3.5:
            continue
        a = np.floor((a - m0) / K_).astype(int); b = np.ceil((b - m0) / K_).astype(int)
        drzwi[a[0]:b[0] + 1, a[1]:b[1] + 1] = True
    sciana = G0 & ~drzwi; nadproze = G0 & drzwi
    OK_ = np.zeros(shape, bool)                       # okna: parapet do 0,9 m i nadproze od 2,0 m (szyby L2 nie widzi)
    for a, b in Kd['okna']:
        PM.rysuj_odcinki(OK_, np.array([(a - m0) / K_, (b - m0) / K_]))
    OK_ &= ~G0
    wolne = ~ndimage.binary_dilation(G0) & obw
    xy = lambda M, co: (np.argwhere(M)[::co] + rng.random((len(np.argwhere(M)[::co]), 2))) * K_ + m0
    C = []
    W = xy(sciana, 1); nz = 10
    C.append(np.c_[np.repeat(W, nz, 0), rng.uniform(0, H, len(W) * nz)])
    C.append(np.c_[np.repeat(W, 16, 0), rng.uniform(H - 0.8, H, len(W) * 16)])   # L2 z plecaka widzi gorna czesc scian gesto
    # (prawdziwy L2 z 2-3 m: setki punktow na dm2 sciany; tu tyle, by pas pod sufitem byl ciagly w siatce 4 cm)
    # nadproza drzwi (od 2,05 m) i okien (od 2,0 m) oraz parapety (do 0,9 m) - gestosc jak sciana (to ta sama sciana)
    N_ = xy(nadproze, 1); C.append(np.c_[np.repeat(N_, 16, 0), rng.uniform(2.05, H, len(N_) * 16)])
    O_ = xy(OK_, 1); C.append(np.c_[np.repeat(O_, 16, 0), rng.uniform(2.0, H, len(O_) * 16)])
    C.append(np.c_[np.repeat(O_, 10, 0), rng.uniform(0, 0.9, len(O_) * 10)])
    F = xy(wolne, 3)
    C.append(np.c_[F, np.zeros(len(F))]); C.append(np.c_[F, np.full(len(F), H)])
    # szafy: przy scianach, w losowych miejscach wolnej przestrzeni tuz przy scianie, wysokosc 2,0-2,4 m (pod sufitem szczelina)
    przy = wolne & ndimage.binary_dilation(G0, iterations=int(0.12 / K_)) & ~ndimage.binary_dilation(drzwi, iterations=int(1.0 / K_))
    kand = np.argwhere(przy)
    n_sz = int(szafy * len(Kd['E']))
    for _ in range(n_sz):
        if not len(kand):
            break
        c = kand[rng.integers(len(kand))]
        dl, gl, h = rng.uniform(0.8, 2.0), 0.6, rng.uniform(2.0, 2.4)
        # kierunek sciany: gradient odleglosci od scian
        d = ndimage.distance_transform_edt(~G0[max(0, c[0] - 40):c[0] + 40, max(0, c[1] - 40):c[1] + 40])
        gy, gx = np.gradient(d); cc = (min(c[0], 40), min(c[1], 40)); n = np.array([gy[cc], gx[cc]]); n = n / (np.linalg.norm(n) or 1)
        t = np.array([-n[1], n[0]]); o = c * K_ + m0
        u, v = np.meshgrid(np.arange(-dl / 2, dl / 2, 0.04), np.arange(0, gl, 0.04))
        front = o + np.outer(u.ravel(), t)[:0].sum(0) if False else None
        uu = np.arange(-dl / 2, dl / 2, 0.04); zz = np.arange(0, h, 0.05)
        A = (o + n * gl)[None] + uu[:, None] * t                               # czolo szafy
        C.append(np.c_[np.repeat(A, len(zz), 0), np.tile(zz, len(A))])
        vv = np.arange(0, gl, 0.04); B_ = (o + vv[:, None] * n)
        for s_ in (-dl / 2, dl / 2):                                           # boki
            Bs = B_ + s_ * t; C.append(np.c_[np.repeat(Bs, len(zz), 0), np.tile(zz, len(Bs))])
        Top = (o + vv[:, None, None] * n + uu[None, :, None] * t).reshape(-1, 2); C.append(np.c_[Top, np.full(len(Top), h)])
    P = np.vstack(C)
    P[:, :2] += rng.normal(0, 0.012, (len(P), 2)); P[:, 2] += rng.normal(0, 0.01, len(P))   # L2: szum lica ~1-2 cm
    return P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pkl'); ap.add_argument('--kondygnacja', type=int); ap.add_argument('--szafy', type=float, default=0.3)
    ap.add_argument('--png', action='store_true')
    a = ap.parse_args()
    D = pickle.load(open(a.pkl, 'rb'))
    for ki in ([a.kondygnacja] if a.kondygnacja is not None else range(len(D))):
        Kd = D[ki]
        if len(Kd['E']) < 4:
            continue
        t0 = time.time()
        k_, E, pomin = SY.symuluj(Kd)                 # E z numerami "fizycznych" pomieszczen (wzorzec)
        if k_ is None:
            continue
        poz, _, _, _ = T.przejscie(Kd)
        P = budynek(Kd, a.szafy)
        with tempfile.TemporaryDirectory() as tmp:
            zapisz_pcd(os.path.join(tmp, 'c.pcd'), P)
            tr = np.c_[np.arange(len(poz)) * 0.5, poz, np.full(len(poz), 1.75)]
            np.savetxt(os.path.join(tmp, 'trasa.csv'), tr, delimiter=';', header='t;x;y;z', comments='', fmt='%.3f')
            r = subprocess.run([sys.executable, os.path.join(TU, '..', '..', 'l2', 'l2_rzut.py'), os.path.join(tmp, 'c.pcd'), os.path.join(tmp, 'wy'),
                                '--trasa', os.path.join(tmp, 'trasa.csv'), '--bez-otworow'], capture_output=True, text=True)
            if r.returncode:
                print(r.stdout[-1500:], r.stderr[-1500:]); continue
            Wj = json.load(open(os.path.join(tmp, 'wy', 'pomieszczenia.json')))['kondygnacje'][0]
            if a.png:
                import shutil; shutil.copy(os.path.join(tmp, 'wy', 'rzut.png'), f"l2pom_{os.path.basename(a.pkl).replace('.pkl', '')}_{ki}.png")
        nazwa = f"{os.path.basename(a.pkl).replace('.pkl', '')}_{Kd['nazwa']} (L2, {len(P) / 1e6:.1f} mln pkt, {time.time() - t0:.0f} s)"
        SY.porownaj(nazwa, None, E, w=Wj)


if __name__ == '__main__':
    main()

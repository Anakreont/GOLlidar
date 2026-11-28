#!/usr/bin/env python3
"""Lekki podglad chmury L2 do przegladarki (three.js): chmura.pcd -> podglad3d.bin.

    python l2_podglad3d.py <chmura.pcd> [podglad3d.bin] [--voxel 0.03] [--max 700000]

Format (little endian):
    4 B  'L2P1'
    u32  liczba punktow N
    3 x f32  poczatek ukladu (min x, y, z) [m]
    N x 3 x u16  x, y, z w cm od poczatku
Chmura jest przerzedzana siatka (voxel, srodek komorki = pierwszy punkt) i ewentualnie losowo do --max punktow,
zeby strona dzialala tez na telefonie (700 tys. pkt = ok. 4 MB).
"""
import argparse, os, struct, sys

import numpy as np


def czytaj_pcd(plik):
    """Minimalny czytnik PCD (ascii / binary) -> tablica N x 3 (x, y, z)."""
    with open(plik, 'rb') as f:
        nag = {}
        while True:
            l = f.readline()
            if not l:
                raise SystemExit('PCD bez linii DATA')
            l = l.decode('ascii', 'replace').strip()
            if not l or l.startswith('#'):
                continue
            k, *v = l.split(); nag[k.upper()] = v
            if k.upper() == 'DATA':
                break
        pola = nag['FIELDS']; roz = list(map(int, nag['SIZE'])); typ = nag['TYPE']
        ile = list(map(int, nag.get('COUNT', ['1'] * len(pola)))); n = int(nag['POINTS'][0])
        if nag['DATA'][0] == 'ascii':
            A = np.loadtxt(f, dtype=float, ndmin=2)
            ix = [sum(ile[:pola.index(c)]) for c in 'xyz']
            return A[:, ix]
        if nag['DATA'][0] != 'binary':
            raise SystemExit('PCD binary_compressed - nieobslugiwany')
        mapa = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = np.dtype([(p if p != '_' else f'_{i}', mapa[(t, s)], (c,) if c > 1 else ()) for i, (p, t, s, c) in enumerate(zip(pola, typ, roz, ile))])
        A = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        return np.stack([A['x'], A['y'], A['z']], 1).astype(np.float64)


def podglad(P, voxel=0.03, maks=700000, ziarno=1):
    P = P[np.isfinite(P).all(1)]
    if not len(P):
        return P, np.zeros(3)
    # odrzucenie pojedynczych dalekich odczytow (0,1 % z kazdej strony), zeby skala nie rozjechala sie przez jeden punkt
    lo = np.percentile(P, 0.05, axis=0) - 1.0; hi = np.percentile(P, 99.95, axis=0) + 1.0
    P = P[((P >= lo) & (P <= hi)).all(1)]
    if voxel > 0:
        K = np.floor((P - P.min(0)) / voxel).astype(np.int64)
        klucz = (K[:, 0] * 73856093) ^ (K[:, 1] * 19349663) ^ (K[:, 2] * 83492791)
        _, i = np.unique(klucz, return_index=True)
        P = P[np.sort(i)]
    if len(P) > maks:
        P = P[np.sort(np.random.default_rng(ziarno).choice(len(P), maks, replace=False))]
    o = P.min(0)
    return P, o


def zapisz(P, o, plik):
    Q = np.clip(np.round((P - o) * 100), 0, 65535).astype('<u2')
    tmp = plik + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(b'L2P1' + struct.pack('<I3f', len(Q), *o)); f.write(Q.tobytes())
    os.replace(tmp, plik)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('wyjscie', nargs='?')
    ap.add_argument('--voxel', type=float, default=0.03); ap.add_argument('--max', type=int, default=700000)
    a = ap.parse_args()
    wy = a.wyjscie or os.path.join(os.path.dirname(os.path.abspath(a.pcd)), 'podglad3d.bin')
    P = czytaj_pcd(a.pcd); n0 = len(P)
    P, o = podglad(P, a.voxel, a.max)
    zapisz(P, o, wy)
    print(f'podglad 3D: {n0} -> {len(P)} pkt, {os.path.getsize(wy) / 1e6:.1f} MB -> {wy}')


if __name__ == '__main__':
    sys.exit(main())

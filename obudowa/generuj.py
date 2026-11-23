#!/usr/bin/env python3
"""
Generator obudowy glowicy skanera (LiDAR D500 / STL-19P + IMU BNO085 + adaptery USB) na gwint foto 1/4".
Wymaga: pip install trimesh manifold3d.   Uzycie: python3 generuj.py  ->  podstawa.stl, pokrywa.stl

Uklad wspolrzednych [mm]: X w prawo, Y do przodu (strona zlacza LiDAR-u = +Y), Z do gory.
Punkt (0,0) = srodek miedzy dwoma bocznymi otworami mocujacymi LiDAR-u.
Wymiary LiDAR-u wg noty STL-19P: otwory 2x w rozstawie 46,8 mm + 1x 31,92 mm dalej, korpus 38,59 x 38,59 mm.
"""
import numpy as np, trimesh
from trimesh.creation import box, cylinder

# ---------------- parametry (zmien i wygeneruj ponownie)
X0, X1, Y0, Y1 = -34.0, 34.0, -38.0, 22.0      # obrys zewnetrzny
SCIANA, DNO = 2.4, 2.4
WYS = 32.0                                      # wysokosc podstawy
OS = (0.0, -8.5)                                # os obrotu LiDAR-u = os gwintu 1/4"
NAKR_AF, NAKR_H = 11.5, 6.0                     # nakretka 1/4"-20: 11,11 mm pod klucz + luz; glebokosc gniazda
SRUBA_14 = 6.8                                  # otwor na srube 1/4"
POKRYWA_GR = 3.6
LID_OTW = [(-23.4, 0.0), (23.4, 0.0), (0.0, -31.92)]   # otwory LiDAR-u (sruby M2)
SLUPKI = [(X0 + 5.9, Y0 + 5.9), (X1 - 5.9, Y0 + 5.9), (X0 + 5.9, Y1 - 5.9), (X1 - 5.9, Y1 - 5.9)]
BNO_SR = (-21.0, -10.0)                         # srodek plytki BNO085 (Adafruit, 25,4 x 17,8 mm)
BNO_DX, BNO_DY = 12.7, 20.32                    # rozstaw otworow plytki BNO085 - SPRAWDZ suwmiarka przed drukiem


def b(x0, x1, y0, y1, z0, z1):
    m = box((x1 - x0, y1 - y0, z1 - z0)); m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m

def c(x, y, r, z0, z1, n=48):
    m = cylinder(radius=r, height=z1 - z0, sections=n); m.apply_translation((x, y, (z0 + z1) / 2)); return m

def U(*m): return trimesh.boolean.union(list(m), engine="manifold")
def D(a, *m): return trimesh.boolean.difference([a] + list(m), engine="manifold")


def podstawa():
    korpus = D(b(X0, X1, Y0, Y1, 0, WYS), b(X0 + SCIANA, X1 - SCIANA, Y0 + SCIANA, Y1 - SCIANA, DNO, WYS + 1))
    dod = [c(OS[0], OS[1], 10.5, 0, 2.0 + NAKR_H)]                                   # nadlew pod nakretke 1/4"
    dod += [b(x - 3.5, x + 3.5, y - 3.5, y + 3.5, 0, WYS) for x, y in SLUPKI]         # slupki pod wkrety pokrywy
    bno = [(BNO_SR[0] + sx * BNO_DX / 2, BNO_SR[1] + sy * BNO_DY / 2) for sx in (-1, 1) for sy in (-1, 1)]
    dod += [c(x, y, 2.9, 0, DNO + 5.0, 24) for x, y in bno]                            # slupki pod BNO085
    m = U(korpus, *dod)
    wyc = [c(OS[0], OS[1], SRUBA_14 / 2, -1, 3),
           c(OS[0], OS[1], NAKR_AF / 2 / np.cos(np.pi / 6), 2.0, 2.0 + NAKR_H + 1, 6)]  # gniazdo szesciokatne
    wyc += [c(x, y, 1.35, WYS - 14, WYS + 1, 20) for x, y in SLUPKI]                   # wkrety M3 (otwor 2,7)
    wyc += [c(x, y, 1.05, DNO + 0.6, DNO + 6, 16) for x, y in bno]                     # wkrety M2,5 (otwor 2,1)
    wyc += [b(X1 - SCIANA - 1, X1 + 1, yc - 6.5, yc + 6.5, WYS - 12, WYS + 1) for yc in (-22.0, 6.0)]  # wyjscia kabli USB
    wyc += [b(x - 1.1, x + 1.1, yc - 3, yc + 3, -1, DNO + 1) for x in (15.0, 28.5) for yc in (-22.0, 6.0)]  # szczeliny na opaski
    return D(m, *wyc)


def pokrywa():
    m = b(X0, X1, Y0, Y1, 0, POKRYWA_GR)
    wyc = [c(x, y, 1.2, -1, POKRYWA_GR + 1, 20) for x, y in LID_OTW]                    # sruby M2 LiDAR-u (otwor 2,4)
    wyc += [c(x, y, 1.7, -1, POKRYWA_GR + 1, 24) for x, y in SLUPKI]                    # wkrety M3 pokrywy (otwor 3,4)
    wyc += [b(-7, 7, 12.5, 18.5, -1, POKRYWA_GR + 1)]                                   # przepust przewodu 4-pin LiDAR-u
    return D(m, *wyc)


if __name__ == "__main__":
    for nazwa, f in (("podstawa", podstawa), ("pokrywa", pokrywa)):
        m = f(); m.export(nazwa + ".stl")
        print(nazwa, "szczelna:", m.is_watertight, "objetosc cm3: %.1f" % (m.volume / 1000), "wymiary:", np.round(m.extents, 1))

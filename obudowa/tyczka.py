#!/usr/bin/env python3
"""
Mocowanie tyczki (rurka Ø25 mm) w twardym plecaku (np. Yato YT-089090) + korek z gwintem 1/4" na glowice skanera.
Wymaga: pip install trimesh manifold3d.   Uzycie: python3 tyczka.py  ->  korek.stl, obejma.stl, gniazdo.stl

Wszystkie czesci drukuja sie bez podpor (PETG, 4 obrysy, wypelnienie 40 %).
Os rurki = os Z. Sciana skrzyni (tylna, od plecow) lezy w plaszczyznie Y = 0.
"""
import numpy as np, trimesh
from trimesh.creation import box, cylinder

# ---------------- parametry (zmierz rurke suwmiarka i popraw)
RURA_D = 40.0           # srednica zewnetrzna rurki
RURA_DW = 36.0          # srednica wewnetrzna (40 x 2 mm -> 36)
LUZ = 0.4               # luz otworu na rurke (druk zwykle zweza otwory)
OD_SCIANY = RURA_D / 2 + 7   # odleglosc osi rurki od wewnetrznej sciany skrzyni
SRUBA_SCIANA = 5.5      # otwory na sruby M5 przez sciane skrzyni
SRUBA_ZACISK = 4.4      # sruba zaciskowa M4
NAKR_M4 = 7.4           # nakretka M4 pod klucz 7 mm + luz
LEB_14 = 11.6           # leb sruby 1/4"-20 (klucz 7/16" = 11,11 mm) + luz
SR_14 = 6.7             # trzpien 1/4"


def b(x0, x1, y0, y1, z0, z1):
    m = box((x1 - x0, y1 - y0, z1 - z0)); m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m

def c(x, y, r, z0, z1, n=72):
    m = cylinder(radius=r, height=z1 - z0, sections=n); m.apply_translation((x, y, (z0 + z1) / 2)); return m

def cx(y, z, r, x0, x1, n=36):   # walec wzdluz X
    m = cylinder(radius=r, height=x1 - x0, sections=n)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    m.apply_translation(((x0 + x1) / 2, y, z)); return m

def cy(x, z, r, y0, y1, n=36):   # walec wzdluz Y
    m = cylinder(radius=r, height=y1 - y0, sections=n)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (1, 0, 0)))
    m.apply_translation((x, (y0 + y1) / 2, z)); return m

def hexx(y, z, af, x0, x1):      # gniazdo szesciokatne wzdluz X
    m = cylinder(radius=af / np.sqrt(3), height=x1 - x0, sections=6)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    m.apply_translation(((x0 + x1) / 2, y, z)); return m

def hexz(af, z0, z1):
    m = cylinder(radius=af / np.sqrt(3), height=z1 - z0, sections=6); m.apply_translation((0, 0, (z0 + z1) / 2)); return m

def U(*m): return trimesh.boolean.union(list(m), engine="manifold")
def D(a, *m): return trimesh.boolean.difference([a] + list(m), engine="manifold")


def korek():
    """Wciskany w gorny koniec rurki; od dolu wklada sie srube 1/4"-20 x 20-25 mm lbem w gniazdo szesciokatne,
    trzpien wystaje do gory - na niego nakreca sie glowice skanera (albo glowice kulowa). Poprzeczny otwor M4
    na kolek przez rurke - korek nie obroci sie ani nie wysunie."""
    trzon = c(0, 0, (RURA_DW - 0.3) / 2, 0, 32)                     # czesc w rurce (lekko wcisk)
    kol = c(0, 0, RURA_D / 2 + 3.0, 32, 39)                         # kolnierz oparty na koncu rurki
    m = U(trzon, kol)
    m = D(m, hexz(LEB_14, 22, 28.5), c(0, 0, SR_14 / 2, 28.4, 40),   # leb 1/4" (od dolu) + trzpien
          c(0, 0, LEB_14 / 2 + 0.3, -1, 22.1),                       # kanal do wsuniecia sruby od dolu
          cx(0, 12, SRUBA_ZACISK / 2, -20, 20))                      # kolek M4 przez rurke
    return m


def obejma():
    """Obejma na sciane skrzyni: plyta przykrecana 4 srubami M5 przez sciane (od zewnatrz duze podkladki),
    zacisk rurki 2 srubami M4. Druk: plyta pionowo, otwor na rurke pionowo (tak jak lezy w modelu)."""
    H = 60.0; y0 = OD_SCIANY; R = (RURA_D + LUZ) / 2
    P = R + 5 + 16                                                   # sruby M5 7 mm od korpusu zacisku
    plyta = b(-P, P, 0, 5, 0, H)
    blok = b(-R - 5, R + 5, 0, y0, 0, H)
    piers = c(0, y0, R + 5, 0, H)
    uszy = b(-10, 10, y0, y0 + R + 13, 0, H)
    m = U(plyta, blok, piers, uszy)
    otw = [c(0, y0, R, -1, H + 1),                                   # rurka
           b(-1.2, 1.2, y0, y0 + R + 14, -1, H + 1)]                 # szczelina zacisku
    for z in (13, H - 13):
        otw += [cx(y0 + R + 6.5, z, SRUBA_ZACISK / 2, -11, 11),     # sruba zaciskowa M4
                hexx(y0 + R + 6.5, z, NAKR_M4, 6.5, 11),            # nakretka M4
                cx(y0 + R + 6.5, z, 4.2, -11, -6.5)]                # leb sruby
    for x in (-(P - 8), P - 8):
        for z in (9, H - 9):
            otw += [cy(x, z, SRUBA_SCIANA / 2, -1, 6)]               # M5 przez sciane skrzyni
    return D(m, *otw)


def gniazdo():
    """Kubek na dno skrzyni: rurka stoi w nim i opiera sie o dno kubka. Plyta 4 x M5 przez dno skrzyni.
    Uklad: sciana skrzyni w Y = 0, os rurki w Y = OD_SCIANY (tak samo jak w obejmie)."""
    y0 = OD_SCIANY; R = (RURA_D + LUZ) / 2
    P = R + 4 + 16
    plyta = b(-P, P, 1.0, y0 + R + 18, 0, 5)
    kubek = c(0, y0, R + 4, 0, 33)
    m = U(plyta, kubek)
    otw = [c(0, y0, R, 4, 34)]                                       # rurka (dno kubka 4 mm)
    for x in (-(P - 8), P - 8):
        for y in (8, y0 + R + 10):
            otw += [c(x, y, SRUBA_SCIANA / 2, -1, 6)]
    return D(m, *otw)


if __name__ == "__main__":
    for nazwa, f in (("korek", korek), ("obejma", obejma), ("gniazdo", gniazdo)):
        m = f()
        assert m.is_watertight, nazwa
        m.export(f"{nazwa}.stl")
        print(f"{nazwa}.stl  {m.bounds[1] - m.bounds[0]} mm, {m.volume / 1000:.1f} cm3")

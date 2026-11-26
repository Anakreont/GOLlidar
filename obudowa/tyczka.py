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


def kolano(kat=35.0):
    """Kolano zamiast korka: nasuwane NA rurke (kolek M4 przez rurke), gorna plyta pochylona o 'kat' stopni
    (kopula L2 do przodu = +Y). W plycie srubka 1/4"-20 x 16 mm lbem od srodka - na nia glowica / plytka LiDAR-u.
    Kable ida rurka, przez kanal w kolanie i wychodza oknem z tylu (-Y). kat = 0 -> plyta pozioma (obecny LiDAR 2D)."""
    Rz = RURA_D / 2 + LUZ / 2 + 4.0                 # promien zewnetrzny tulei
    tul = c(0, 0, Rz, 0, 42)
    a = np.radians(kat)
    os_ = np.array([0, np.sin(a), np.cos(a)])        # pochylona os gornej czesci
    p0 = np.array([0, 0, 42.0])                      # punkt obrotu
    def wzd(r, l0, l1, n=72):                        # walec wzdluz pochylonej osi
        m = cylinder(radius=r, height=l1 - l0, sections=n)
        m.apply_transform(trimesh.transformations.rotation_matrix(-a, (1, 0, 0)))
        m.apply_translation(p0 + os_ * (l0 + l1) / 2); return m
    def hex_wzd(af, l0, l1):
        m = cylinder(radius=af / np.sqrt(3), height=l1 - l0, sections=6)
        m.apply_transform(trimesh.transformations.rotation_matrix(-a, (1, 0, 0)))
        m.apply_translation(p0 + os_ * (l0 + l1) / 2); return m
    kula = trimesh.creation.icosphere(subdivisions=4, radius=Rz); kula.apply_translation(p0)
    gora = wzd(Rz, 0, 30)
    m = U(tul, kula, gora)
    otw = [c(0, 0, RURA_D / 2 + LUZ / 2, -1, 36),                       # rurka wchodzi na 36 mm
           c(0, 0, RURA_DW / 2 - 2, 30, 43),                            # kanal kabli w gore
           wzd(RURA_DW / 2 - 2, -2, 22),                                # kanal w pochylonej czesci
           hex_wzd(LEB_14, 18, 24.5), wzd(SR_14 / 2, 24, 31),           # leb sruby 1/4" + trzpien
           cy(0, 46, 9.0, -Rz - 5, 0),                                  # okno na kable z tylu (-Y)
           cx(0, 14, SRUBA_ZACISK / 2, -Rz - 1, Rz + 1)]                # kolek M4 przez rurke
    return D(m, *otw)


# L2 (instrukcja L2, rys. „Installation Dimensions”): 4 x M3 gl. 6 mm na okregu O51, co 90 st., obrocone o 22,5 st.
# wzgledem osi wyjscia kabla (kabel do tylu, -Y). Kwadrat ok. 36,1 x 36,1 mm (L1 mial 45 x 45 - to byl blad).
L2_R_OTW = 25.5
L2_OTWORY = [(L2_R_OTW * np.sin(np.radians(f)), -L2_R_OTW * np.cos(np.radians(f))) for f in (22.5, 112.5, 202.5, 292.5)]
L2_PLYTA = 64.0    # srednica plyty pod L2 (podstawa L2 ma 75 x 75; otwory na O51 + leb M3)


def kolano_l2(kat=35.0):
    """Kolano z plyta bezposrednio pod Unitree L2: 4 x M3 (sruby M3 x 8 od spodu, gwint w L2 ma 6 mm glebokosci).
    Kopula do przodu (+Y), kable L2 w dol przez okno z tylu do rurki."""
    Rz = RURA_D / 2 + LUZ / 2 + 4.0
    a = np.radians(kat)
    os_ = np.array([0, np.sin(a), np.cos(a)]); p0 = np.array([0, 0, 42.0])
    def wzd(r, l0, l1, x=0.0, y=0.0, n=72):
        m = cylinder(radius=r, height=l1 - l0, sections=n); m.apply_translation((x, y, (l0 + l1) / 2))
        m.apply_transform(trimesh.transformations.rotation_matrix(-a, (1, 0, 0)))
        m.apply_translation(p0); return m
    kula = trimesh.creation.icosphere(subdivisions=4, radius=Rz); kula.apply_translation(p0)
    L = 24.0                                                             # dlugosc pochylonej czesci do plyty
    m = U(c(0, 0, Rz, 0, 42), kula, wzd(Rz, 0, L), wzd(L2_PLYTA / 2, L, L + 5))
    otw = [c(0, 0, RURA_D / 2 + LUZ / 2, -1, 36), c(0, 0, RURA_DW / 2 - 2, 30, 43),
           wzd(RURA_DW / 2 - 2, -2, L - 1),                              # kanal (sufit 6 mm pod plyta)
           cy(0, 46, 9.0, -Rz - 5, 0),                                   # okno kabli z tylu
           cx(0, 14, SRUBA_ZACISK / 2, -Rz - 1, Rz + 1)]                 # kolek M4
    for ox, oy in L2_OTWORY:
        otw += [wzd(1.7, L - 1, L + 6, ox, oy, 24),          # M3
                wzd(3.2, L - 30, L + 1.0, ox, oy, 24)]        # leb M3 od spodu (plyta 4 mm)
    return D(m, *otw)



def uchwyt_l2_wieko(kat=20.0, H=120.0):
    """Uchwyt L2 przykrecany do GORNEJ scianki twardego plecaka (Yato YT-089090: zewn. 465 x 355 x 205 mm),
    bez tyczki - jak Hovermap. Plyta 110 x 70 x 6 mm, 4 x M5 przez scianke (od srodka duze podkladki), kolumna
    O50 wysokosci H, na niej plytka O64 pod L2 (4 x M3 na O51) pochylona o 'kat' stopni DO TYLU (-Y, nad plecami):
    kopula patrzy w gore i do tylu, glowa operatora zaslania mniej. Kabel L2 w dol kolumna i otworem O26 w plycie
    do skrzyni (w sciance wywiercic O30). Druk plyta na stole, bez podpor (sufit kanalu pod plytka ~O30 - most)."""
    a = np.radians(kat)
    p0 = np.array([0, 0, H])
    def wzd(r, l0, l1, x=0.0, y=0.0, n=72):          # walec wzdluz osi pochylonej plytki
        m = cylinder(radius=r, height=l1 - l0, sections=n); m.apply_translation((x, y, (l0 + l1) / 2))
        m.apply_transform(trimesh.transformations.rotation_matrix(a, (1, 0, 0)))   # +kat wokol X -> os do -Y
        m.apply_translation(p0); return m
    plyta = b(-55, 55, -35, 35, 0, 6)
    kol = c(0, 0, 25, 0, H)
    kula = trimesh.creation.icosphere(subdivisions=4, radius=25); kula.apply_translation(p0)
    gora = U(wzd(25, 0, 10), wzd(L2_PLYTA / 2, 10, 15))
    m = U(plyta, kol, kula, gora)
    otw = [c(0, 0, 13, -1, H), trimesh.creation.icosphere(subdivisions=3, radius=13).apply_translation(p0),
           wzd(13, -1, 9)]                                                  # kanal kabla
    for x in (-45, 45):
        for y in (-25, 25):
            otw.append(c(x, y, 2.75, -1, 7))                                 # M5
    for ox, oy in L2_OTWORY:
        otw += [wzd(1.7, 9, 16, ox, oy, 24), wzd(3.2, -20, 11, ox, oy, 24)]  # M3 + leb od spodu (plytka 4 mm)
    return D(m, *otw)


def kolano90():
    """Dolne kolano 90 st.: rurka stoi NA ZEWNATRZ skrzyni (miedzy skrzynia a szelkami), wchodzi w kolano od gory,
    kable skrecaja w kolanie i ida otworem przez tylna sciane do srodka - nic nie widac. Kolnierz 4 x M5 przez sciane
    (od srodka duze podkladki), w scianie otwor ~Ø30 na kable. Gorna obejma (obejma.stl) trzyma rurke przy scianie.
    Druk: tak jak w modelu (dno plaskie na stole, kolnierz pionowo), bez podpor."""
    y0 = OD_SCIANY; R = (RURA_D + LUZ) / 2; Rz = R + 4.0; P = Rz + 16
    blok = b(-Rz, Rz, 0, y0, 0, 2 * Rz)                                  # kolano (dol plaski)
    tul = c(0, y0, Rz, 0, 2 * Rz + 40)                                   # tuleja na rurke
    kol = b(-P, P, 0, 5, 0, 2 * Rz + 16)                                 # kolnierz na scianie (Y=0)
    m = U(blok, tul, kol)
    zc = Rz                                                              # wysokosc osi kanalu
    otw = [c(0, y0, R, 2 * Rz - 2, 2 * Rz + 41),                         # rurka wchodzi na ~42 mm, opiera sie na stopniu
           c(0, y0, RURA_DW / 2 - 2, zc, 2 * Rz),                         # kanal pionowy
           cy(0, zc, RURA_DW / 2 - 2, -1, y0),                            # kanal poziomy przez kolnierz
           trimesh.creation.icosphere(subdivisions=3, radius=RURA_DW / 2 - 2).apply_translation((0, y0, zc)),
           cx(2 * Rz + 22, y0 * 0 + 2 * Rz + 22, 0.1, -0.05, 0.05)]       # (pusty element)
    otw = otw[:-1]
    otw += [cx(y0, 2 * Rz + 22, SRUBA_ZACISK / 2, -Rz - 1, Rz + 1)]     # kolek M4 przez tuleje i rurke
    for x in (-(P - 8), P - 8):
        for z in (8, 2 * Rz + 8):
            otw += [cy(x, z, SRUBA_SCIANA / 2, -1, 6)]
    return D(m, *otw)


if __name__ == "__main__":
    for nazwa, f in (("korek", korek), ("obejma", obejma), ("gniazdo", gniazdo),
                     ("kolano_0", lambda: kolano(0)), ("kolano_35", lambda: kolano(35)),
                     ("kolano_l2_35", lambda: kolano_l2(35)), ("kolano90", kolano90),
                     ("uchwyt_l2_wieko_0", lambda: uchwyt_l2_wieko(0)), ("uchwyt_l2_wieko_20", lambda: uchwyt_l2_wieko(20)),
                     ("uchwyt_l2_wieko_35", lambda: uchwyt_l2_wieko(35))):
        m = f()
        assert m.is_watertight, nazwa
        m.export(f"{nazwa}.stl")
        print(f"{nazwa}.stl  {m.bounds[1] - m.bounds[0]} mm, {m.volume / 1000:.1f} cm3")

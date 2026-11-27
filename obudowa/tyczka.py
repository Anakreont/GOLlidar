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


def napis(tekst, wys_lit, z0, x_srodek, y_tyl, y_przod):
    """Wypukly napis na pionowej scianie zwroconej do +Y, czytany od dolu do gory (litery 'gora' w strone +X).
    Zwraca (siatka, dlugosc_mm)."""
    from matplotlib.textpath import TextPath
    from matplotlib.font_manager import FontProperties
    from shapely.geometry import Polygon
    from functools import reduce
    tp = TextPath((0, 0), tekst, size=wys_lit, prop=FontProperties(family="DejaVu Sans", weight="bold"))
    pol = [Polygon(r) for r in tp.to_polygons() if len(r) > 2]
    ksz = reduce(lambda q, w: q.symmetric_difference(w), [x.buffer(0) for x in pol])   # parzysto-nieparzyscie: dziury w literach
    (u0, v0, u1, v1) = ksz.bounds
    geom = list(ksz.geoms) if hasattr(ksz, "geoms") else [ksz]
    m = trimesh.util.concatenate([trimesh.creation.extrude_polygon(g, y_przod - y_tyl) for g in geom])   # (u, v, w)
    M = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)   # x=v, y=w, z=u (bez odbicia)
    m.apply_translation((-u0, -(v0 + v1) / 2, 0)); m.apply_transform(M)
    m.apply_translation((x_srodek, y_tyl, z0))
    return m, u1 - u0


R_KOL = 24.0       # promien kolumny uchwytu L2 (O48)
R_KAN = 16.0       # promien kanalu (O32): kable L2, GPS, czujnika, tasma kamery
SKORA = 1.6        # najcienszy plaszcz rury nad obiektywem / antena
# Elementy w srodku rury, wsuwane OD GORY przez otwor w plytce L2 (przed przykreceniem L2), kazdy we wlasnej prowadnicy
# do ogranicznika; z zewnatrz widac tylko otwor obiektywu i szczeliny przewiewu.
GPS_PL = (13.1, 15.7, 1.6)    # ATGM336H (goldpiny 5 x 2,54 na krotszym boku - w dol)
ANTENA = (13.5, 22.0, 8.0)    # max szer. x dl. x grubosc anteny ceramicznej (zmierzyc!)
CZUJNIK = (16.0, 3.0)         # max szerokosc plytki czujnika temp./wilg. (SHT31, BME280...) i grubosc szczeliny
KAM_PL = (25.0, 24.0, 1.0)    # Camera Module 3 Wide: plytka 25 x 24 x 1 mm
KAM_WYS = 12.4                # plytka + obiektyw
KAM_SOCZ = 9.5                # srodek obiektywu od gornej krawedzi plytki (sprawdzic na kamerze)
KAM_OBIEKTYW = 9.6            # szerokosc oprawy obiektywu (kwadrat ok. 8,5 mm + luz)


def uchwyt_l2_sciana(kat=20.0, PLYTA_H=100.0, NAD=120.0, tekst="GOLlidar"):
    """Uchwyt L2 na PLASKA, PIONOWA scianke plecaka (zewnetrzna, od strony za operatorem) - JEDEN WYDRUK.
    Uklad: scianka w plaszczyznie Y = 0, na zewnatrz +Y (= do tylu za operatorem), gora skrzyni na Z = PLYTA_H + 10.
    Plyta 70 x PLYTA_H x 6 mm, 4 x M5 przez scianke (od srodka duze podkladki), mostek do kolumny. Kolumna O48 (kanal O32)
    przechodzi plynnie w plytke O64 pod L2 (4 x M3 na O51) pochylona o 'kat' do tylu; srodek plytki nad osia kolumny.
    Przez plytke pionowy otwor (kanal + prowadnice) - przez niego, ZANIM przykrecisz L2, wsuwa sie od gory do
    ogranicznikow: kamere (prowadnice po bokach plytki, obiektyw do tylu przez otwor w plaszczu), antene GPS (wneka
    w plaszczu, 1,2 mm pod powierzchnia), plytke GPS i czujnik temp./wilg. (szczeliny przy sciance kanalu). Przewody
    i tasma kamery w dol kanalem; kabel L2 oknem 20 x 30 mm od strony scianki i tym samym kanalem do otworu
    w sciance skrzyni (kanal O32 -> otwor O33). Napis 'tekst' na boku -X. Druk tak jak w modelu (stoi na dolnej krawedzi), bez podpor."""
    a = np.radians(kat)
    Yk = 6 + R_KOL                                     # os kolumny od scianki
    RP = L2_PLYTA / 2
    Zt = PLYTA_H - 10 + NAD - 22.0                     # gora czesci walcowej kolumny
    Zp = Zt + 20.0                                     # srodek plytki L2 (nad osia kolumny)
    R = trimesh.transformations.rotation_matrix(-a, (1, 0, 0))          # os plytki pochylona do +Y
    pp = np.array([0, Yk, Zp])
    def pl(r, l0, l1, x=0.0, y=0.0, n=72):             # walec wzdluz osi plytki (l = 0 na spodzie plytki)
        m = cylinder(radius=r, height=l1 - l0, sections=n); m.apply_translation((x, y, (l0 + l1) / 2))
        m.apply_transform(R); m.apply_translation(pp); return m
    def kolo_pl(r, l, n=96):
        f = np.linspace(0, 2 * np.pi, n, endpoint=False)
        P = np.c_[r * np.cos(f), r * np.sin(f), np.full(n, l), np.ones(n)] @ R.T
        return P[:, :3] + pp
    f = np.linspace(0, 2 * np.pi, 96, endpoint=False)
    gora_kol = np.c_[R_KOL * np.cos(f), Yk + R_KOL * np.sin(f), np.full(96, Zt)]
    przejscie = trimesh.convex.convex_hull(np.vstack([gora_kol, kolo_pl(RP, 0)]))
    plyta = b(-35, 35, 0, 6, 0, PLYTA_H)
    kol = c(0, Yk, R_KOL, 0, Zt)
    lacz = b(-R_KOL, R_KOL, 0, Yk, 0, PLYTA_H)
    czesci = [plyta, kol, lacz, przejscie, pl(RP, 0, 5)]
    if tekst:
        dol_txt, gora_txt = PLYTA_H + 16, Zt - 4
        h = min(11.0, (gora_txt - dol_txt) / 5.6)
        nap, dl = napis(tekst, h, 0, 0, R_KOL - 2.0, R_KOL + 1.0)
        nap.apply_translation((0, 0, dol_txt + max(0.0, (gora_txt - dol_txt - dl) / 2)))
        nap.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 0, 1)))   # +Y -> -X
        nap.apply_translation((0, Yk, 0))
        czesci.append(nap)
    m = U(*czesci)

    GORA = Zp + 30                                     # prowadnice wychodza przez plytke
    def kier(fi, r0, r1, sz, z0, z1):                  # prostopadloscian w kierunku fi (st.): promieniowo r0..r1, szer. sz
        mm = b(r0, r1, -sz / 2, sz / 2, z0, z1)
        mm.apply_transform(trimesh.transformations.rotation_matrix(np.radians(fi), (0, 0, 1)))
        mm.apply_translation((0, Yk, 0)); return mm
    zk = 30.0
    # wysokosci (od gory): kamera, antena, GPS, czujnik
    z_kam_g = Zt - 8.0                                 # gorna krawedz plytki kamery
    z_kam_d = z_kam_g - KAM_PL[1]                      # ogranicznik kamery
    z_socz = z_kam_g - KAM_SOCZ
    z_ant = Zt - 4.0 - ANTENA[1]                       # ogranicznik anteny
    z_gps = z_kam_d - 4.0 - GPS_PL[1]                  # ogranicznik GPS
    z_cz = PLYTA_H + 14.0                              # ogranicznik czujnika (nad krawedzia skrzyni)
    yb = R_KOL - SKORA - KAM_WYS                       # tyl plytki kamery (od osi kolumny, w strone +Y)
    otw = [c(0, Yk, R_KAN, zk, GORA), cy(0, zk, R_KAN, -1, Yk), trimesh.creation.icosphere(subdivisions=3, radius=R_KAN).apply_translation((0, Yk, zk)),
           b(-10, 10, Yk - R_KOL - 2, Yk, Zt - 30, Zt - 2)]                                 # okno kabla L2 (-Y)
    # kamera: rowki na krawedzie plytki + kanal obiektywu do plaszcza + okno obiektywu z fazka (120 st.)
    otw += [b(-KAM_PL[0] / 2 - 0.4, KAM_PL[0] / 2 + 0.4, Yk + yb - 0.3, Yk + yb + KAM_PL[2] + 0.4, z_kam_d, GORA),
            b(-KAM_OBIEKTYW / 2, KAM_OBIEKTYW / 2, Yk + yb, Yk + R_KOL - SKORA, z_kam_d, GORA)]
    rs = 5.2; rz = rs + SKORA * np.tan(np.radians(62)) + 0.6
    f = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    otw += [trimesh.convex.convex_hull(np.vstack([np.c_[rs * np.cos(f), np.full(40, Yk + R_KOL - SKORA - 0.05), z_socz + rs * np.sin(f)],
                                                  np.c_[rz * np.cos(f), np.full(40, Yk + R_KOL + 0.3), z_socz + rz * np.sin(f)]]))]
    # antena GPS: wneka w plaszczu (kierunek -22,5 st., miedzy srubami M3), 1,2 mm pod powierzchnia
    sz_a = ANTENA[0] + 0.6
    r_a = np.sqrt((R_KOL - SKORA) ** 2 - (sz_a / 2) ** 2)                      # naroza wneki tez pod plaszczem
    otw += [kier(-22.5, r_a - ANTENA[2] - 0.5, r_a, sz_a, z_ant, GORA)]
    # plytka GPS: szczelina przy sciance kanalu (kierunek 180 st., -X)
    otw += [kier(180, R_KAN - 2.5, R_KAN + 1.5, GPS_PL[0] + 0.7, z_gps, GORA)]
    # czujnik: szczelina od strony scianki (-Y), przewiew szczelinami -Y i +Y na jego wysokosci
    otw += [kier(-90, R_KAN - 2.5, R_KAN + CZUJNIK[1] - 2.5, CZUJNIK[0] + 0.6, z_cz, GORA)]
    for i in range(4):
        z = z_cz + 2 + i * 4.5
        otw += [b(-6, 6, Yk - R_KOL - 1, Yk - R_KAN + 2, z, z + 2.2), b(-6, 6, Yk + R_KAN - 2, Yk + R_KOL + 1, z, z + 2.2)]
    for x in (-24, 24):
        for z in (12, PLYTA_H - 12):
            otw += [cy(x, z, 2.75, -1, Yk + 30), cy(x, z, 5.5, 6, Yk + 30)]   # M5 + dostep do lba od zewnatrz
    # tunele na leb M3 i klucz od spodu - omijaja z zapasem 1,5 mm kanal, wneke anteny i obiektyw (nie otwieraja ich na zewnatrz)
    chron = U(c(0, Yk, R_KAN + 1.5, 0, Zp), kier(-22.5, r_a - ANTENA[2] - 2, r_a + 1.5, sz_a + 3, z_ant - 1.5, GORA),
              b(-KAM_OBIEKTYW / 2 - 1.5, KAM_OBIEKTYW / 2 + 1.5, Yk + yb - 2, Yk + R_KOL - SKORA + 1.5, z_kam_d - 1.5, GORA),
              b(-KAM_PL[0] / 2 - 1.9, KAM_PL[0] / 2 + 1.9, Yk + yb - 2, Yk + yb + KAM_PL[2] + 2, z_kam_d - 1.5, GORA))
    for ox, oy in L2_OTWORY:
        tun = pl(3.2, -35, 1.0, ox, oy, 24)
        kier_r = (R[:3, :3] @ np.array([ox, oy, 0.0])) / np.hypot(ox, oy)      # promieniowo na zewnatrz (w ukladzie plytki)
        tun2 = tun.copy(); tun2.apply_translation(kier_r * 12)
        tun = trimesh.convex.convex_hull(np.vstack([tun.vertices, tun2.vertices]))   # rowek otwarty na zewnatrz: wkret i klucz z boku
        otw += [pl(1.7, 1, 6, ox, oy, 24),                                  # M3 (plytka 4 mm)
                D(tun, chron)]
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
                     ("uchwyt_l2_wieko_35", lambda: uchwyt_l2_wieko(35)),
                     ("uchwyt_l2_sciana_0", lambda: uchwyt_l2_sciana(0)), ("uchwyt_l2_sciana_20", lambda: uchwyt_l2_sciana(20)),
                     ("uchwyt_l2_sciana_35", lambda: uchwyt_l2_sciana(35))):
        m = f()
        assert m.is_watertight, nazwa
        m.export(f"{nazwa}.stl")
        print(f"{nazwa}.stl  {m.bounds[1] - m.bounds[0]} mm, {m.volume / 1000:.1f} cm3")

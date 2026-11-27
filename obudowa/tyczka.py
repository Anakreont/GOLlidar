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


R_KOL = 20.0       # promien kolumny uchwytu L2 (O40; wczesniej O50)
R_KAN = 12.0       # promien kanalu kabla (O24)


GPS_POLKA = 56.0  # polka pod modul GPS (np. NEO-M8N / BN-880 do 38 x 38 mm) - modul na ZEWNETRZNYM koncu, poza obrysem L2 (x > 37,5)
CZUJ_KLATKA = (24.0, 18.0, 34.0)   # wnetrze klatki czujnika temp./wilg. (np. SHT31, BME280): X, Y, Z


def uchwyt_l2_sciana(kat=20.0, PLYTA_H=100.0, NAD=120.0, tekst="GOLlidar", gps=True, czujnik=True):
    """Uchwyt L2 na PLASKA, PIONOWA scianke plecaka (zewnetrzna, od strony za operatorem) przy gornej krawedzi.
    Uklad: scianka w plaszczyznie Y = 0, na zewnatrz +Y (= do tylu za operatorem), gora skrzyni na Z = PLYTA_H + 10.
    Plyta 70 x PLYTA_H x 6 mm, 4 x M5 przez scianke (od srodka duze podkladki), z mostkiem do kolumny. Kolumna O40
    (kanal O24) odsunieta od scianki; plytka O64 pod L2 (4 x M3 na O51) ok. NAD mm nad krawedzia skrzyni, pochylona
    o 'kat' do tylu (+Y), pod nia stozek 30 st. (druk bez podpor).
    Kabel L2 wychodzi z boku podstawy L2 (strona -Y, do scianki): opada i wchodzi OKNEM 20 x 30 mm tuz pod stozkiem,
    dalej kolumna w dol i poziomym kanalem przez plyte do otworu O30 w sciance. Dodatkowo otwor O24 przez srodek
    plytki (gdyby kabel wychodzil od spodu L2). Na kolumnie od strony +Y wypukly napis 'tekst'.
    GPS: polka 56 x 42 mm z boku (+X; modul na zewnetrznym koncu, poza cieniem L2), z rantem i szczelinami na opaske, pod nia wspornik 45 st.; przewod otworem O8
    do kanalu. Czujnik temp./wilg.: klatka z boku (-X), otwarta od dolu, szczeliny wentylacyjne, przewod otworem O8
    do kanalu - z dala od plecow operatora i od elektroniki w skrzyni.
    Druk tak jak w modelu (stoi na dolnej krawedzi, kolumna pionowo), bez podpor."""
    a = np.radians(kat)
    Yk = 6 + R_KOL                                     # os kolumny od scianki
    RP = L2_PLYTA / 2
    Lc = (RP - R_KOL) / np.tan(np.radians(30))         # dlugosc stozka (30 st. od osi)
    Zg = PLYTA_H - 10 + NAD - Lc + 10                  # gora kolumny (punkt obrotu)
    p0 = np.array([0, Yk, Zg])
    R = trimesh.transformations.rotation_matrix(-a, (1, 0, 0))          # os pochylona do +Y
    def wzd(r, l0, l1, x=0.0, y=0.0, n=72):
        m = cylinder(radius=r, height=l1 - l0, sections=n); m.apply_translation((x, y, (l0 + l1) / 2))
        m.apply_transform(R); m.apply_translation(p0); return m
    def stozek(r0, r1, l0, l1, n=96):
        f = np.linspace(0, 2 * np.pi, n, endpoint=False)
        pk = np.vstack([np.c_[r0 * np.cos(f), r0 * np.sin(f), np.full(n, l0)], np.c_[r1 * np.cos(f), r1 * np.sin(f), np.full(n, l1)]])
        m = trimesh.convex.convex_hull(pk); m.apply_transform(R); m.apply_translation(p0); return m
    def kula(r, pkt, sub=4):
        k = trimesh.creation.icosphere(subdivisions=sub, radius=r); k.apply_translation(pkt); return k
    plyta = b(-35, 35, 0, 6, 0, PLYTA_H)
    kol = c(0, Yk, R_KOL, 0, Zg)
    lacz = b(-R_KOL, R_KOL, 0, Yk, 0, PLYTA_H)        # pelny mostek scianka-kolumna (sztywnosc)
    gora = U(kula(R_KOL, p0), stozek(R_KOL, RP, 0, Lc), wzd(RP, Lc, Lc + 5))
    czesci = [plyta, kol, lacz, gora]
    if tekst:
        h = 11.0
        nap, dl = napis(tekst, h, 0, 0, Yk + R_KOL - 2.0, Yk + R_KOL + 1.0)
        z0 = PLYTA_H + 10 + 8 + max(0.0, (Zg - 14 - (PLYTA_H + 18) - dl) / 2)   # nad krawedzia skrzyni, pod oknem
        nap.apply_translation((0, 0, z0))
        czesci.append(nap)
    zs = Zg - 32.0                                     # gora polki GPS
    if gps:
        x0, x1, h2 = R_KOL - 3, R_KOL - 3 + GPS_POLKA, 21.0
        czesci.append(b(x0, x1, Yk - h2, Yk + h2, zs - 4, zs))                       # polka 4 mm
        czesci.append(b(x1 - 2, x1, Yk - h2, Yk + h2, zs, zs + 3))                   # rant zewnetrzny
        wsp = trimesh.convex.convex_hull(np.array([[x0, y, z] for y in (Yk - 4, Yk + 4) for z in (zs - 4, zs - 4 - (x1 - x0))] +
                                                  [[x1, y, zs - 4] for y in (Yk - 4, Yk + 4)]))      # wspornik 45 st.
        czesci.append(wsp)
    zt = PLYTA_H + 22.0                                # dol klatki czujnika
    if czujnik:
        kx, ky, kz = CZUJ_KLATKA
        czesci.append(b(-R_KOL - kx - 2, -R_KOL + 4, Yk - ky / 2 - 2, Yk + ky / 2 + 2, zt, zt + kz + 2))
    m = U(*czesci)
    zk = 30.0                                          # wysokosc kanalu kabla przez plyte (otwor O30 w sciance)
    otw = [c(0, Yk, R_KAN, zk, Zg), kula(R_KAN, p0, 3),
           wzd(R_KAN, -1, Lc + 6),                     # kanal przez stozek i plytke (otwor O24 w srodku plytki)
           cy(0, zk, R_KAN, -1, Yk), kula(R_KAN, (0, Yk, zk), 3),
           b(-10, 10, Yk - R_KOL - 2, Yk, Zg - 36, Zg - 6)]   # okno na kabel L2 od strony scianki (-Y)
    if gps:
        otw += [cx(Yk, zs + 6, 4.0, 0, R_KOL + 6)]                                    # przewod GPS do kanalu
        for y in (Yk - 12, Yk + 12):
            otw += [b(R_KOL + 14, R_KOL + GPS_POLKA - 10, y - 1.5, y + 1.5, zs - 5, zs + 1)]   # szczeliny na opaske
    if czujnik:
        kx, ky, kz = CZUJ_KLATKA
        otw += [b(-R_KOL - kx, -R_KOL - 1, Yk - ky / 2, Yk + ky / 2, zt - 1, zt + kz),     # wnetrze, otwarte od dolu
                cx(Yk, zt + kz - 6, 4.0, -R_KOL - 2, 0)]                                    # przewod do kanalu
        for i in range(4):                                                                 # szczeliny wentylacyjne
            z = zt + 6 + i * 6.5
            otw += [b(-R_KOL - kx - 3, -R_KOL - kx + 1, Yk - ky / 2 + 3, Yk + ky / 2 - 3, z, z + 2.5),
                    b(-R_KOL - kx + 3, -R_KOL - 4, Yk - ky / 2 - 3, Yk + ky / 2 + 3, z, z + 2.5)]
    for x in (-24, 24):
        for z in (12, PLYTA_H - 12):
            otw += [cy(x, z, 2.75, -1, Yk + 30), cy(x, z, 5.5, 6, Yk + 30)]   # M5 + dostep do lba od zewnatrz
    for ox, oy in L2_OTWORY:
        otw += [wzd(1.7, Lc + 1, Lc + 6, ox, oy, 24), wzd(3.2, -30, Lc + 1.0, ox, oy, 24)]  # M3 (plytka 4 mm) + leb od spodu
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

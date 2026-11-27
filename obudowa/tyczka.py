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


R_KOL = 21.0       # promien kolumny uchwytu L2 (O42; v1 O50)
R_KAN = 15.0       # promien kanalu w srodku (O30) - kable L2, GPS i czujnik; scianka 6 mm (sztywnosc jak O40/O24)
# Wneka serwisowa z klapka (+X): w srodku rury GPS ATGM336H (plytka 13,1 x 15,7 mm + antena ceramiczna) i czujnik
# temp./wilg. (np. SHT31 / BME280). GPS i antena przyklejone od srodka do klapki (antena patrzy na zewnatrz przez 2 mm
# PETG); czujnik nizej, za szczelinami w klapce i w przeciwleglej sciance (przewiew przez rure).
WNEKA_SZ = 18.0    # szerokosc otworu w rurze
WNEKA_H = 62.0     # wysokosc otworu
PAD_X = 24.0       # plaska powierzchnia pod klapke (x od osi kolumny)
KLAPKA_SZ = 30.0   # szerokosc klapki (= szerokosc plaskiego pola)
KLAPKA_GR = 2.0
# GPS ATGM336H na klapce: plytka 13,1 x 15,7 x 1,6 mm wsuwana od dolu w prowadnice; antena ceramiczna (zmierzyc!)
# w kolysce nad plytka, opaska 2,5 mm; czujnik temp./wilg. na polce na dole klapki, opaska, przy szczelinach.
GPS_PL = (13.1, 15.7, 1.6)
ANTENA = (13.5, 22.0, 9.0)    # max szerokosc (y), dlugosc (z), grubosc (x) anteny w kolysce
# Kamera Raspberry Pi Camera Module 3 Wide: plytka 25 x 24 mm, 4 otwory M2 (rozstaw 21 x 12,5, 2 mm od krawedzi),
# wys. z obiektywem 12,4 mm, obiektyw ok. 9,5 mm od gornej krawedzi (sprawdzic na plytce). Tasma w dol do kanalu.
KAM_PL = (25.0, 24.0)
KAM_OTW = [(-10.5, 10.0), (10.5, 10.0), (-10.5, -2.5), (10.5, -2.5)]   # otwory wzgl. srodka plytki (x, z), gora +z
KAM_SOCZ_Z = 2.5             # srodek obiektywu wzgl. srodka plytki
KAM_WYS = 12.4               # plytka + obiektyw
KAM_PAD = (38.0, 50.0, 26.0) # pole pod oslone kamery: szer. (x), wys. (z), odleglosc lica od osi kolumny (y)


def uchwyt_l2_sciana(kat=20.0, PLYTA_H=100.0, NAD=120.0, tekst="GOLlidar", wneka=True, kamera=True):
    """Uchwyt L2 na PLASKA, PIONOWA scianke plecaka (zewnetrzna, od strony za operatorem) przy gornej krawedzi.
    Uklad: scianka w plaszczyznie Y = 0, na zewnatrz +Y (= do tylu za operatorem), gora skrzyni na Z = PLYTA_H + 10.
    Plyta 70 x PLYTA_H x 6 mm, 4 x M5 przez scianke (od srodka duze podkladki), z mostkiem do kolumny. Kolumna O42
    (kanal O30) odsunieta od scianki; plytka O64 pod L2 (4 x M3 na O51) ok. NAD mm nad krawedzia skrzyni, pochylona
    o 'kat' do tylu (+Y), pod nia stozek 30 st. (druk bez podpor).
    Kabel L2 wychodzi z boku podstawy L2 (strona -Y, do scianki): opada i wchodzi OKNEM 20 x 30 mm tuz pod stozkiem,
    dalej kolumna w dol i poziomym kanalem przez plyte do otworu O30 w sciance. Dodatkowo otwor O24 przez srodek plytki.
    W srodku rury, za klapka z boku (+X, klapka_tuby.stl, 2 x wkret M3): GPS (gora) i czujnik temp./wilg. (dol,
    przewiew szczelinami w klapce i w sciance -X). Przewody w dol tym samym kanalem do skrzyni.
    Od tylu (+Y) kieszen kamery Camera Module 3 Wide z oslona (oslona_kamery.stl, 2 x M3), tasma w dol kanalem.
    Na boku -X wypukly napis 'tekst'. Druk tak jak w modelu (stoi na dolnej krawedzi), bez podpor."""
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
    zh1 = Zg - 12.0; zh0 = zh1 - WNEKA_H               # otwor wneki
    if wneka:                                          # plaskie pole pod klapke, dol i gora sciete 45 st.
        hy = KLAPKA_SZ / 2; zp0, zp1 = zh0 - 10, zh1 + 10; xw = 8.0
        pk = [[x, y, z] for y in (Yk - hy, Yk + hy) for (x, z) in ((xw, zp0 - (PAD_X - xw)), (PAD_X, zp0), (PAD_X, zp1), (xw, zp1 + (PAD_X - xw)))]
        czesci.append(trimesh.convex.convex_hull(np.array(pk)))
    kx, kz, ky = KAM_PAD
    zc = Zg - 8 - kz / 2                               # srodek pola kamery (od tylu, +Y)
    if kamera:
        hx = kx / 2; zp0, zp1 = zc - kz / 2, zc + kz / 2; yw = 6.0
        pk = [[x, Yk + y, z] for x in (-hx, hx) for (y, z) in ((yw, zp0 - (ky - yw)), (ky, zp0), (ky, zp1), (yw, zp1))]
        czesci.append(trimesh.convex.convex_hull(np.array(pk)))
    if tekst:                                          # napis na boku -X, nad szczelinami przewiewu (od tylu jest kamera)
        dol_txt = (Zg - 12 - WNEKA_H + 24) if wneka else PLYTA_H + 14
        gora_txt = Zg - 6
        h = min(11.0, (gora_txt - dol_txt) / 5.4)
        nap, dl = napis(tekst, h, 0, 0, R_KOL - 2.0, R_KOL + 1.0)      # wzgledem osi kolumny, potem obrot na -X
        nap.apply_translation((0, 0, dol_txt + max(0.0, (gora_txt - dol_txt - dl) / 2)))
        nap.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 0, 1)))   # +Y -> -X
        nap.apply_translation((0, Yk, 0))
        czesci.append(nap)
    m = U(*czesci)
    zk = 30.0                                          # wysokosc kanalu kabla przez plyte (otwor O30 w sciance)
    otw = [c(0, Yk, R_KAN, zk, Zg), kula(R_KAN, p0, 3),
           wzd(12, -1, Lc + 6),                        # kanal przez stozek i plytke (otwor O24 w srodku plytki)
           cy(0, zk, R_KAN, -1, Yk), kula(R_KAN, (0, Yk, zk), 3),
           b(-10, 10, Yk - R_KOL - 2, Yk, Zg - 36, Zg - 6)]   # okno na kabel L2 od strony scianki (-Y)
    if wneka:
        otw += [b(0, PAD_X + 1, Yk - WNEKA_SZ / 2, Yk + WNEKA_SZ / 2, zh0, zh1)]           # otwor wneki
        for z in (zh0 - 5, zh1 + 5):
            otw += [cx(Yk, z, 1.25, PAD_X - 12, PAD_X + 1, 24)]                             # wkrety M3 (otwor 2,5 w plastik)
        for i in range(4):                                                                  # przewiew: szczeliny w sciance -X
            z = zh0 + 3 + i * 5.0
            otw += [b(-R_KOL - 2, -R_KAN + 1, Yk - 6, Yk + 6, z, z + 2.0)]
    if kamera:                                         # kieszen kamery: plytka lezy na polkach Yk+11, tasma w dol do kanalu
        otw += [b(-KAM_PL[0] / 2 - 0.5, KAM_PL[0] / 2 + 0.5, Yk + 11, Yk + ky + 1, zc - KAM_PL[1] / 2 - 6.5, zc + KAM_PL[1] / 2 + 0.5),
                b(-9, 9, Yk + 4, Yk + 12, zc - KAM_PL[1] / 2 - 6.5, zc - KAM_PL[1] / 2 + 2)]     # przejscie tasmy do kanalu
        for x in (-(kx / 2 - 2.5), kx / 2 - 2.5):
            otw += [cy(x, zc, 1.25, Yk + ky - 10, Yk + ky + 1, 24)]                       # wkrety M3 oslony
    for x in (-24, 24):
        for z in (12, PLYTA_H - 12):
            otw += [cy(x, z, 2.75, -1, Yk + 30), cy(x, z, 5.5, 6, Yk + 30)]   # M5 + dostep do lba od zewnatrz
    for ox, oy in L2_OTWORY:
        otw += [wzd(1.7, Lc + 1, Lc + 6, ox, oy, 24), wzd(3.2, 2, Lc + 1.0, ox, oy, 24)]  # M3 (plytka 4 mm) + leb od spodu
    return D(m, *otw)


def klapka_tuby():
    """Klapka wneki w rurze uchwytu L2 (2 x wkret M3 x 10), od srodka stelaz (bez tasmy):
    - GPS ATGM336H: prowadnice z rowkiem 1,9 mm, plytka wsuwana od dolu (goldpiny w dol) do zderzaka;
    - antena ceramiczna: kolyska nad GPS (polka od dolu) + opaska 2,5 mm przez szczeliny w klapce;
    - czujnik temp./wilg.: polka na dole + dwa zeberka odsuwajace od klapki (powietrze z tylu) + opaska; szczeliny
      przewiewu w klapce za czujnikiem.
    Uklad lokalny: x = grubosc (x > 0 lico zewn., x < 0 do srodka rury), y w poprzek, z wzdluz (otwor wneki z 10..72).
    Druk plaska strona do stolu - stelaz stoi pionowo, bez podpor. Nie z filamentu z weglem (tlumi GPS)."""
    hz = WNEKA_H + 20; zo0, zo1 = 10.0, 10.0 + WNEKA_H; hy = WNEKA_SZ / 2 - 0.35   # wnetrze otworu (z luzem)
    cz = [b(0, KLAPKA_GR, -KLAPKA_SZ / 2, KLAPKA_SZ / 2, 0, hz)]
    cz += [b(-1.5, 0.01, -hy, -hy + 1.2, zo0 + 0.35, zo1 - 0.35), b(-1.5, 0.01, hy - 1.2, hy, zo0 + 0.35, zo1 - 0.35),
           b(-1.5, 0.01, -hy, hy, zo0 + 0.35, zo0 + 1.5), b(-1.5, 0.01, -hy, hy, zo1 - 1.5, zo1 - 0.35)]   # ramka centrujaca
    z_cz = zo0 + 1.5                                                                       # czujnik: polka + zeberka
    cz += [b(-9, 0.01, -hy, hy, z_cz, z_cz + 1.5)] + [b(-3.0, 0.01, y - 0.6, y + 0.6, z_cz, z_cz + 16) for y in (-5, 5)]
    gw, gh, gt = GPS_PL                                                                    # GPS: prowadnice
    z_g0 = z_cz + 19.0; z_g1 = z_g0 + gh + 0.6
    yr = gw / 2 + 0.25
    for sg in (-1, 1):
        cz += [b(-6.5, 0.01, *sorted((sg * (yr - 1.2), sg * hy)), z_g0, z_g1)]
    cz += [b(-6.5, 0.01, -hy, hy, z_g1, z_g1 + 1.5)]                                     # zderzak u gory
    z_a = z_g1 + 3.0                                                                       # antena: polka kolyski
    cz += [b(-ANTENA[2] - 1.5, 0.01, -hy, hy, z_a, z_a + 1.5)]
    m = U(*cz)
    otw = [b(-4.2, -2.3, -yr, yr, z_g0 - 1, z_g1 + 0.01),                                  # rowek na PCB (1,9 mm)
           b(-7, -1.6, -(yr - 1.2) + 0.01, (yr - 1.2) - 0.01, z_g0 - 1, z_g1)]             # miedzy prowadnicami wolno
    for z in (5, hz - 5):
        otw += [cx(0, z, 1.7, -2, 3, 24)]                                                 # M3
    for zz in (z_cz + 8, z_a + 1.5 + ANTENA[1] / 2):                                     # szczeliny na opaske 2,5 mm
        for y in (-(hy - 0.8), hy - 0.8):
            otw += [b(-2, 3, y - 0.8, y + 0.8, zz - 1.6, zz + 1.6)]
    for i in range(4):                                                                    # przewiew za czujnikiem
        z = z_cz + 2.5 + i * 3.5
        otw += [b(-2, 3, -4, 4, z, z + 2.0)]
    m = D(m, *otw)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))     # plasko na stole
    m.apply_translation(-m.bounds[0])
    return m


def oslona_kamery():
    """Oslona kieszeni kamery (Camera Module 3 Wide) od tylu rury: plyta 2 mm na 2 x wkret M3 x 10, okno obiektywu
    O15 z fazka (pod kat 120 st.), 4 tulejki dociskajace plytke kamery (kolki O1,7 w otwory M2).
    Druk lico do stolu. Uklad lokalny: z = grubosc (0 = lico zewn., rosnie do srodka), x, y = szer./wys. (y do gory)."""
    kx, kz, ky = KAM_PAD
    dl = ky - 11 - 1.0                                   # od lica oslony do lica plytki kamery (plytka 1 mm na polkach Yk+11)
    assert dl - KAM_WYS + 1.0 > 0.3, dl
    m = U(b(-kx / 2, kx / 2, -kz / 2, kz / 2, 0, 2),
          *[c(x, z, 2.2, 1.9, dl, 24) for x, z in KAM_OTW],                                 # tulejki do lica plytki
          *[c(x, z, 0.85, dl - 0.1, dl + 1.2, 16) for x, z in KAM_OTW])                     # kolki w otwory M2
    f = np.linspace(0, 2 * np.pi, 48, endpoint=False); r1 = 7.5 + 2.2 * np.tan(np.radians(60))
    stozek = trimesh.convex.convex_hull(np.vstack([np.c_[7.5 * np.cos(f), 7.5 * np.sin(f) + KAM_SOCZ_Z, np.full(48, 2.1)],
                                                   np.c_[r1 * np.cos(f), r1 * np.sin(f) + KAM_SOCZ_Z, np.full(48, -0.1)]]))
    return D(m, stozek, *[c(x, 0, 1.7, -1, 3, 24) for x in (-(kx / 2 - 2.5), kx / 2 - 2.5)])


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
                     ("uchwyt_l2_sciana_35", lambda: uchwyt_l2_sciana(35)), ("klapka_tuby", klapka_tuby), ("oslona_kamery", oslona_kamery)):
        m = f()
        assert m.is_watertight, nazwa
        m.export(f"{nazwa}.stl")
        print(f"{nazwa}.stl  {m.bounds[1] - m.bounds[0]} mm, {m.volume / 1000:.1f} cm3")

"""Pomiar pomieszczenia z postoju (przycisk „Pomiar” na stronie skanera).

Operator stoi z tyczka pionowo ok. 3 s; z ~30 obrotow LiDAR-u liczymy:
- wymiary prostokata dopasowanego do scian (kierunek scian z histogramu kierunkow odcinkow),
- powierzchnie prostokata i powierzchnie obrysu (wielokat z punktow, przyciety do prostokata,
  zeby otwarte drzwi i okna nie powiekszaly pola).
Czysty Python (bez numpy) - liczy sie na Raspberry Pi w ulamek sekundy.
"""
import math

ZASIEG = 11.0          # m; dalej D500 juz niepewny


def licz(obroty, pitch=0.0, roll=0.0):
    """obroty: lista obrotow [(kat_st, odl_mm), ...]; pitch/roll: przechyl wzgledem pionu (st.).
    Zwraca slownik: a, b (m, a >= b), pole (m2, prostokat), pole_obrys (m2), kat (st.), pokrycie (0-1), uwagi."""
    n_obr = len(obroty)
    if n_obr < 5:
        return {"blad": "za malo obrotow LiDAR-u"}
    kosze = [[] for _ in range(360)]
    for ob in obroty:
        for kat, odl in ob:
            if 150 < odl < ZASIEG * 1000:
                kosze[int(kat) % 360].append(odl / 1000.0)
    p, r = math.radians(pitch), math.radians(roll)
    pkt = [None] * 360
    for k in range(360):
        d = sorted(kosze[k])
        if len(d) < max(3, 0.3 * n_obr):
            continue
        dm = d[len(d) // 2]
        A = math.radians(k + 0.5)
        if 130 <= k < 230 and dm < 0.9:           # operator (glowa, plecak) za glowica
            continue
        if dm < 0.35:
            continue
        dm *= math.cos(max(-0.35, min(0.35, -r * math.cos(A) + p * math.sin(A))))   # rzut na poziom
        pkt[k] = (dm * math.sin(A), dm * math.cos(A))
    wazne = [k for k in range(360) if pkt[k]]
    pokrycie = len(wazne) / 360
    if len(wazne) < 90:
        return {"blad": "za malo widocznych scian (stoisz za blisko przeszkody?)", "pokrycie": round(pokrycie, 2)}

    # odcinki miedzy sasiednimi katami -> kierunek scian (modulo 90 st.)
    odc = []
    for k in range(360):
        a_, b_ = pkt[k], pkt[(k + 1) % 360]
        if a_ and b_:
            dx, dy = b_[0] - a_[0], b_[1] - a_[1]
            dl = math.hypot(dx, dy)
            if 0.005 < dl < 0.3:
                odc.append((a_, b_, dl, math.degrees(math.atan2(dy, dx)) % 90))
    h = [0.0] * 90
    for _, _, dl, kt in odc:
        h[int(kt) % 90] += dl
    hs = [sum(h[(i + j) % 90] for j in (-2, -1, 0, 1, 2)) for i in range(90)]
    th0 = max(range(90), key=lambda i: hs[i])
    # dokladniej: srednia kolowa (okres 90 st.) z odcinkow blisko szczytu
    sx = sy = 0.0
    for _, _, dl, kt in odc:
        if min(abs(kt - th0), 90 - abs(kt - th0)) < 4:
            sx += dl * math.cos(math.radians(kt * 4)); sy += dl * math.sin(math.radians(kt * 4))
    th = (math.degrees(math.atan2(sy, sx)) / 4) % 90
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))

    def obr(q):
        return (q[0] * c + q[1] * s, -q[0] * s + q[1] * c)

    # sciany: dla odcinkow rownoleglych do osi zbieramy wspolrzedna poprzeczna
    wu, wv = {}, {}
    BIN = 0.02
    for a_, b_, dl, _ in odc:
        (ua, va), (ub, vb) = obr(a_), obr(b_)
        du, dv = ub - ua, vb - va
        if abs(du) < 0.2 * abs(dv):        # sciana "pionowa" w ukladzie pokoju -> stale u
            u = (ua + ub) / 2; wu[round(u / BIN)] = wu.get(round(u / BIN), 0) + dl
        elif abs(dv) < 0.2 * abs(du):
            v = (va + vb) / 2; wv[round(v / BIN)] = wv.get(round(v / BIN), 0) + dl

    def sciana(hist, znak):
        strona = {k: w for k, w in hist.items() if k * BIN * znak > 0.3}
        if not strona:
            return None
        # wygladzenie +-1 kosz
        g = {k: sum(strona.get(k + j, 0) for j in (-1, 0, 1)) for k in strona}
        mx = max(g.values())
        if mx < 0.4:                       # mniej niz 40 cm widocznej sciany
            return None
        # najdalsza sciana z co najmniej 1 m widocznej sciany (wymiar „w swietle” miedzy najdalszymi scianami,
        # takze za filarem czy szafa); krotsze kawalki dalej to zwykle sciana sali obok widziana przez drzwi
        kand = [k for k, w in g.items() if w >= min(1.0, 0.8 * mx)]
        k = max(kand, key=lambda k: k * znak)
        sw = sum(strona.get(k + j, 0) for j in (-1, 0, 1))
        return sum((k + j) * BIN * strona.get(k + j, 0) for j in (-1, 0, 1)) / sw

    uP, uL, vG, vD = sciana(wu, 1), sciana(wu, -1), sciana(wv, 1), sciana(wv, -1)
    uwagi = []
    if None in (uP, uL, vG, vD):
        uwagi.append("nie widać wszystkich 4 ścian – pokój większy niż zasięg albo zasłonięty")
    a = (uP - uL) if (uP is not None and uL is not None) else None
    b = (vG - vD) if (vG is not None and vD is not None) else None

    # obrys: wielokat z punktow w kolejnosci kata, przyciety do prostokata (drzwi, okna)
    pole_obr = None
    if a and b:
        pts = []
        for k in wazne:
            u, v = obr(pkt[k])
            pts.append((min(max(u, uL), uP), min(max(v, vD), vG)))
        pole_obr = abs(sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))) / 2
        if pokrycie < 0.7:
            uwagi.append("obrys niepełny – część kątów zasłonięta")
        elif pole_obr < 0.8 * a * b:
            uwagi.append("nieprostokątne lub coś zasłania – zmierz też z drugiego miejsca")
    wyn = {"a": None, "b": None, "pole": None, "pole_obrys": round(pole_obr, 2) if pole_obr else None,
           "kat": round(th, 1), "pokrycie": round(pokrycie, 2), "uwagi": uwagi}
    if a and b:
        a, b = max(a, b), min(a, b)
        wyn.update(a=round(a, 3), b=round(b, 3), pole=round(a * b, 2))
    else:
        wyn.update(a=round(a, 3) if a else None, b=round(b, 3) if b else None)
    return wyn


def opis(w):
    """Tekst na ekran telefonu."""
    if w.get("blad"):
        return w["blad"]
    f = lambda x: f"{x:.2f}".replace(".", ",")
    if w["a"] and w["b"]:
        t = f"{f(w['a'])} × {f(w['b'])} m"
        if w["pole_obrys"] and abs(w["pole_obrys"] - w["pole"]) > 0.03 * w["pole"]:
            # uskoki, wneki, filary: powierzchnia z obrysu, prostokat tylko dla porownania
            t += f" · {f(w['pole_obrys'])} m² (prostokąt {f(w['pole'])} m²)"
        else:
            t += f" · {f(w['pole'])} m²"
    else:
        t = "widać tylko " + (f(w["a"] or w["b"]) + " m" if (w["a"] or w["b"]) else "fragmenty ścian")
    return t + ("" if not w["uwagi"] else " – " + w["uwagi"][0])

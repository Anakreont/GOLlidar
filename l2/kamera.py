"""Klatki z kamery pod L2 (kamera.bin z rejestratora skan_web.py).
Rekord: [czas_ns u64][dlugosc u32][ekspozycja_us u32][ostrosc_dpt*1000 u16][JPEG]; czas = poczatek ekspozycji,
ten sam zegar co l2.bin (czas maliny).
Uzycie: python3 l2/kamera.py <sesja> [--wyciagnij katalog] [--co N]   (bez opcji: podsumowanie)
Kolorowanie chmury (po kalibracji kamera-L2) - nastepny etap.
"""
import os, sys, struct, argparse

NAGL = struct.Struct("<QIIH")


def klatki(plik):
    """Generator (czas_ns, ekspozycja_us, ostrosc_dpt, jpeg_bytes)."""
    with open(plik, "rb") as f:
        while True:
            h = f.read(NAGL.size)
            if len(h) < NAGL.size:
                return
            t, n, e, o = NAGL.unpack(h)
            d = f.read(n)
            if len(d) < n or d[:2] != b"\xff\xd8":
                return                                   # urwany koniec pliku (wylaczenie w trakcie zapisu)
            yield t, e, o / 1000.0, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sesja"); ap.add_argument("--wyciagnij"); ap.add_argument("--co", type=int, default=1)
    a = ap.parse_args()
    p = os.path.join(a.sesja, "kamera.bin")
    n, t0, t1, ek = 0, None, None, []
    for i, (t, e, o, d) in enumerate(klatki(p)):
        t0 = t0 or t; t1 = t; n += 1; ek.append(e)
        if a.wyciagnij and i % a.co == 0:
            os.makedirs(a.wyciagnij, exist_ok=True)
            open(os.path.join(a.wyciagnij, f"{t}.jpg"), "wb").write(d)
    if not n:
        sys.exit("brak klatek")
    ek.sort()
    print(f"{n} klatek, {(t1 - t0) / 1e9:.0f} s, {n / max(1e-9, (t1 - t0) / 1e9):.2f} kl./s, "
          f"ekspozycja mediana {ek[len(ek) // 2] / 1000:.1f} ms, max {ek[-1] / 1000:.1f} ms")


if __name__ == "__main__":
    main()

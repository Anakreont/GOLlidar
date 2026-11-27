#!/usr/bin/env python3
"""
sync_http.py - wysylka nagran z ~/skany do BMS API przez HTTPS (dopisywanie z kontrola pozycji).
Konfiguracja w sync.conf:  URL="https://adres-api"   TOKEN="..."   (opcjonalnie DOM_SSID="SiecDomowa,Siec5G")

Tryb wysylki (ustawiany na stronie skanera, zakladka Dane; zapisany w ~/skany/.ustawienia):
  auto  - co minute wszystko, czego brakuje na serwerze (najnowsze sesje najpierw)
  dom   - automatycznie tylko w sieci domowej (Wi-Fi z listy DOM_SSID / pola na stronie); poza domem tylko kolejka
  reczny- tylko sesje dodane do kolejki przyciskiem „Wyslij zaznaczone”
Kolejka (~/skany/.sync_kolejka) przezywa restart; sesja znika z niej dopiero, gdy caly katalog jest na serwerze.
Stan kazdej sesji (ile bajtow jest na serwerze) -> ~/skany/.sync_stan.json - strona pokazuje go przy nagraniach.
Malina NIC nie kasuje sama: pliki zostaja na karcie, dopoki nie usuniesz ich na stronie.

Wywolywane przez sync.sh (timer co minute). Recznie:
  python3 sync_http.py                 # jak timer (wg trybu)
  python3 sync_http.py --sesje A,B     # dopisz do kolejki i wyslij teraz
  python3 sync_http.py --stan          # tylko sprawdz na serwerze, co tam jest (wszystkie sesje, bez wysylania)
Kod wyjscia: 0 = OK, 1 = blad, 3 = nic do roboty z powodu trybu (recznie / poza domem).
"""
import argparse, json, os, re, subprocess, sys, time, urllib.error, urllib.parse, urllib.request

PIERWSZE = ("info.txt", "znaczniki.txt", "pomieszczenia.csv", "grzejniki.csv", "okna.csv")      # male pliki ida przed duzymi
WZOR = re.compile(r"^[a-z0-9_]{1,40}\.(csv|txt|bin|jpg)$")
KAWALEK = 2 * 1024 * 1024          # serwer przyjmuje do 8 MB; 2 MB = mniej zapytan przy L2 (ok. 15 MB/min)
LIMIT_S = 50                       # jedno uruchomienie pracuje najwyzej ok. 50 s (timer co minute)
TRYBY = ("auto", "dom", "reczny")
PL = str.maketrans("ąćęłńóśźżĄĆĘŁŃÓŚŹŻ", "acelnoszzACELNOSZZ")


def wczytaj_conf(sciezka):
    conf = {}
    try:
        with open(sciezka) as f:
            for linia in f:
                linia = linia.strip()
                if linia and not linia.startswith("#") and "=" in linia:
                    k, v = linia.split("=", 1)
                    conf[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    return conf


def czytaj_json(p, dom):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return dom


def zapisz_json(p, obj):
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp, p)


def biezaca_siec():
    """Nazwa (SSID) Wi-Fi, do ktorego malina jest podlaczona jako klient; '' gdy brak (albo malina sama nadaje)."""
    try:
        out = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID,MODE", "dev", "wifi"], capture_output=True, text=True, timeout=5).stdout
        for l in out.splitlines():
            czesci = re.split(r"(?<!\\):", l)
            if czesci and czesci[0] == "yes":
                return czesci[1].replace("\\:", ":") if len(czesci) > 1 else ""
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        return subprocess.run(["iwgetid", "-r"], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def ustawienia_wysylki(zrodlo, conf):
    u = czytaj_json(os.path.join(zrodlo, ".ustawienia"), {})
    tryb = u.get("wysylka_tryb") or conf.get("TRYB") or "auto"
    dom = u.get("wysylka_dom") if u.get("wysylka_dom") is not None else conf.get("DOM_SSID", "")
    return (tryb if tryb in TRYBY else "auto"), [s.strip() for s in dom.split(",") if s.strip()]


def zapytanie(url, token, metoda="GET", dane=None):
    req = urllib.request.Request(url, data=dane, method=metoda,
                                 headers={"X-Lidar-Token": token, "Content-Type": "application/octet-stream",
                                          "User-Agent": "GOLlidar-sync/2"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b"{}")
        except ValueError:
            return e.code, {}


def bezpieczna(nazwa):
    """Nazwa sesji dla serwera: tylko litery ASCII, cyfry i _ + - (serwer inne odrzuca)."""
    n = nazwa.translate(PL)
    return "".join(ch if (ch.isascii() and ch.isalnum()) or ch in "_+-" else "_" for ch in n)[:100]


def pliki_sesji(kat):
    return [p for p in os.listdir(kat) if WZOR.match(p) and os.path.isfile(os.path.join(kat, p))]


def lokalnie(kat):
    pl = pliki_sesji(kat)
    return sum(os.path.getsize(os.path.join(kat, p)) for p in pl), len(pl)


def nagrywana(kat):
    """Sesja chyba jeszcze nagrywana: w info.txt brak 'koniec=' i pliki zmienialy sie w ostatnich 10 min
    (sesja przerwana np. odcieciem zasilania nie blokuje kolejki w nieskonczonosc)."""
    try:
        with open(os.path.join(kat, "info.txt"), encoding="utf-8") as f:
            if "koniec=" in f.read():
                return False
        return time.time() - max(os.path.getmtime(os.path.join(kat, p)) for p in os.listdir(kat)) < 600
    except (OSError, ValueError):
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sesje", default="", help="sesje (nazwy katalogow, po przecinku) do kolejki i wysylki teraz")
    ap.add_argument("--stan", action="store_true", help="tylko sprawdz rozmiary na serwerze, bez wysylania")
    a = ap.parse_args()
    tu = os.path.dirname(os.path.abspath(__file__))
    conf = wczytaj_conf(os.path.join(tu, "sync.conf"))
    url, token = conf.get("URL", "").rstrip("/"), conf.get("TOKEN", "")
    zrodlo = os.path.expanduser(conf.get("ZRODLO", "~/skany"))
    if not url or not token:
        sys.exit("W sync.conf brakuje URL lub TOKEN")
    p_kol, p_stan = os.path.join(zrodlo, ".sync_kolejka"), os.path.join(zrodlo, ".sync_stan.json")
    kolejka = [n for n in czytaj_json(p_kol, []) if isinstance(n, str)]
    for n in (x.strip() for x in a.sesje.split(",")):
        if n and n not in kolejka and os.path.isdir(os.path.join(zrodlo, n)):
            kolejka.append(n)
    kolejka = [n for n in kolejka if os.path.isdir(os.path.join(zrodlo, n))]
    usuniete = set()

    def zapisz_kolejke():
        """Strona moze dopisac sesje w trakcie wysylki - laczymy z aktualnym plikiem, zamiast go nadpisac."""
        teraz = [n for n in czytaj_json(p_kol, []) if isinstance(n, str)]
        wynik = []
        for n in kolejka + teraz:
            if n not in wynik and n not in usuniete and os.path.isdir(os.path.join(zrodlo, n)):
                wynik.append(n)
        zapisz_json(p_kol, wynik)

    zapisz_kolejke()
    stan = czytaj_json(p_stan, {})
    wszystkie = sorted((n for n in os.listdir(zrodlo) if not n.startswith(".") and os.path.isdir(os.path.join(zrodlo, n))), reverse=True)

    tryb, dom = ustawienia_wysylki(zrodlo, conf)
    siec = biezaca_siec() if tryb == "dom" else ""
    wolno_wszystko = a.stan or tryb == "auto" or (tryb == "dom" and siec in dom)
    do_zrobienia = kolejka + [n for n in wszystkie if n not in kolejka] if wolno_wszystko else list(kolejka)
    if not do_zrobienia:
        print(f"tryb {tryb}{' (siec: ' + (siec or 'brak') + ')' if tryb == 'dom' else ''}: kolejka pusta, nic nie wysylam")
        sys.exit(3 if not wolno_wszystko else 0)

    limit = time.time() + (600 if a.stan else LIMIT_S)
    wyslano, bledy = 0, []
    for sesja in do_zrobienia:
        if time.time() >= limit:
            break
        kat = os.path.join(zrodlo, sesja)
        lok, ile = lokalnie(kat)
        st = stan.get(sesja, {})
        if not a.stan and st.get("pelna") and st.get("lokalnie") == lok and st.get("pliki") == ile and not nagrywana(kat):
            if sesja in kolejka:
                kolejka.remove(sesja); usuniete.add(sesja)
            continue                                 # juz cala na serwerze i nic sie nie zmienilo - bez zapytan
        zdalnie_suma, pelna, przerwano = 0, True, False
        for plik in sorted(pliki_sesji(kat), key=lambda p: (p not in PIERWSZE, p.endswith(".bin"), p)):
            sciezka = os.path.join(kat, plik)
            q = urllib.parse.urlencode({"sesja": bezpieczna(sesja), "plik": plik})
            kod, odp = zapytanie(f"{url}/lidar/rozmiar?{q}", token)
            if kod in (401, 503):
                zapisz_json(p_stan, stan); zapisz_kolejke()
                sys.exit(f"Serwer odpowiedzial {kod}: {odp}")
            if kod != 200:                          # klopot z jedna sesja nie blokuje pozostalych
                bledy.append(f"{sesja}/{plik}: {kod} {odp}"); pelna = False
                continue
            zdalny = int(odp.get("rozmiar", 0))
            if not a.stan:
                with open(sciezka, "rb") as f:
                    while zdalny < os.path.getsize(sciezka):
                        if time.time() >= limit:
                            przerwano = True
                            break
                        f.seek(zdalny)
                        dane = f.read(KAWALEK)
                        if not dane:
                            break
                        kod, odp = zapytanie(f"{url}/lidar/dopisz?{q}&offset={zdalny}", token, "PUT", dane)
                        if kod == 409:
                            zdalny = int(odp.get("rozmiar", 0))
                            continue
                        if kod != 200:
                            bledy.append(f"{sesja}/{plik}: {kod} {odp}")
                            break
                        zdalny = int(odp["rozmiar"])
                        wyslano += len(dane)
            zdalnie_suma += min(zdalny, os.path.getsize(sciezka))
            if zdalny < os.path.getsize(sciezka):
                pelna = False
            if przerwano:
                break
        lok, ile = lokalnie(kat)
        if przerwano:                                 # pliki nieobejrzane w tym przebiegu: zachowaj stara wartosc, jesli wieksza
            zdalnie_suma = max(zdalnie_suma, min(st.get("zdalnie", 0), lok))
        pelna = pelna and not przerwano and zdalnie_suma >= lok and not nagrywana(kat)
        stan[sesja] = {"zdalnie": zdalnie_suma, "lokalnie": lok, "pliki": ile, "pelna": pelna, "czas": int(time.time())}
        if pelna and sesja in kolejka:
            kolejka.remove(sesja); usuniete.add(sesja)
        zapisz_json(p_stan, stan)
    for n in list(stan):                              # sesje usuniete z karty
        if n not in wszystkie:
            stan.pop(n)
    zapisz_json(p_stan, stan); zapisz_kolejke()
    print(f"tryb {tryb}: wyslano {wyslano} bajtow, w kolejce {len(kolejka)}")
    if bledy:
        sys.exit("Bledy:\n  " + "\n  ".join(bledy[:10]))


if __name__ == "__main__":
    try:
        main()
    except (urllib.error.URLError, OSError) as e:
        sys.exit(f"Brak polaczenia: {e}")

#!/usr/bin/env python3
"""Przegladarka pomiarow LiDAR na serwerze (Dell) - tylko siec lokalna.

Pokazuje nagrania z LIDAR_DIR pogrupowane budynek -> dzien (albo kondygnacja) -> sesja: mape z warstwami (sciany, chmura punktow,
trasa, zdjecia, przekroje, punkty, notatki), zdjecia, znaczniki, inwentaryzacje (oprawy, grzejniki, okna) i pliki (DXF).
Mapy licza sie same w tle po zakonczeniu sesji (mapa/przetworz.py), mozna tez wymusic przyciskiem.

Uruchomienie:  python3 przegladarka.py [--port 8150] [--katalog ~/skany-lidar]
Odpowiada tylko klientom z adresow prywatnych (siec domowa, localhost).
"""
import argparse, hashlib, hmac, ipaddress, json, os, queue, re, shutil, subprocess, sys, threading, time, types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

TU = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TU)
sys.modules.setdefault('serial', types.SimpleNamespace(Serial=None, SerialException=Exception))
sys.path.insert(0, REPO)
import skan_web as SW          # tylko funkcje czytajace CSV inwentaryzacji

ARGS = None
SESJA_RE = re.compile(r'^[A-Za-z0-9_+\-]{1,100}$')
PLIK_RE = re.compile(r'^((_wynik|_porownanie|_wynik/l2)/)?[A-Za-z0-9_\-]{1,60}\.(jpg|csv|txt|bin|dxf|png|json|obj|log|pcd)$')
KOLEJKA = queue.PriorityQueue()   # (0 = recznie z przycisku, 1 = automat, licznik, sesja)
NR_ZLEC = __import__('itertools').count()
W_TOKU = {}                    # sesja -> 'czeka' | 'liczy'
BLOK = threading.Lock()


def ram_wolny_gb():
    try:
        for l in open('/proc/meminfo'):
            if l.startswith('MemAvailable:'):
                return int(l.split()[1]) / 1024 / 1024
    except OSError:
        pass
    return 99.0


class Limit:
    """Ile ciezkich liczen naraz (mapy, porownania, CAD): do LIDAR_ROWNOLEGLE (domyslnie 1), a kolejne rusza
    tylko, gdy zostaje >= LIDAR_RAM_ZAPAS_GB wolnej pamieci (domyslnie 4 GB). Pierwsze rusza zawsze."""
    def __init__(self):
        self.ile = 0; self.c = threading.Condition()
        self.max = max(1, int(os.environ.get('LIDAR_ROWNOLEGLE', '1')))   # 3 naraz = OOM na Dellu (26.11)
        self.zapas = float(os.environ.get('LIDAR_RAM_ZAPAS_GB', '4'))

    def __enter__(self):
        with self.c:
            while not (self.ile == 0 or (self.ile < self.max and ram_wolny_gb() >= self.zapas)):
                self.c.wait(10)
            self.ile += 1

    def __exit__(self, *a):
        with self.c:
            self.ile -= 1; self.c.notify_all()


LICZ = Limit()                 # mapy: rownolegle, z kontrola pamieci
LICZ_RECZNE = Limit()          # porownanie SLAM i dopasowanie CAD (z przycisku) - osobne miejsce, nie czekaja na mapy
LICZ_RECZNE.max = 1
ENV_DZIECI = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='2', MKL_NUM_THREADS='2')
KOLEJKA_POR = queue.Queue()
W_POR = {}                     # sesja -> 'czeka' | 'liczy' (porownanie z gotowymi SLAM)
METODY_POR = ('carto_imu', 'carto', 'slam_toolbox')


def skrypt_por():
    """Polecenie porownaj z porownanie/instaluj.sh (srodowisko ROS 2 w ~/ros2_slam)."""
    for p in (os.environ.get('LIDAR_POROWNAJ', ''), os.path.expanduser('~/bin/porownaj')):
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def katalog(n):
    if not SESJA_RE.match(n or '') or n.startswith('_'):
        return None
    k = os.path.join(ARGS.katalog, n)
    return k if os.path.isdir(k) else None


def info(k):
    d = {}
    try:
        with open(os.path.join(k, 'info.txt'), encoding='utf-8') as f:
            for l in f:
                if '=' in l:
                    a, b = l.rstrip('\n').split('=', 1); d[a] = b
    except OSError:
        pass
    return d


def rozmiar(p):
    try:
        return os.path.getsize(p)
    except OSError:
        return 0


MIN_DANE = 2000                # ponizej tylu bajtow czujnik uznajemy za niepodlaczony (D500 / L2)


def usun_wynik_2d(k):
    """Usuwa wyniki mapy 2D (D500); mapa L2 (_wynik/l2, liczona dlugo) zostaje."""
    wy = os.path.join(k, '_wynik')
    for p in os.listdir(wy) if os.path.isdir(wy) else []:
        if p == 'l2':
            continue
        q = os.path.join(wy, p)
        shutil.rmtree(q, ignore_errors=True) if os.path.isdir(q) else os.remove(q)


def czujniki(k):
    """Ktore LiDAR-y nagraly dane w tej sesji: D500 (lidar.bin) i/lub Unitree L2 (l2.bin)."""
    return {'d500': rozmiar(os.path.join(k, 'lidar.bin')) >= MIN_DANE, 'l2': rozmiar(os.path.join(k, 'l2.bin')) >= MIN_DANE}


def stan_wyniku(n, k):
    """Stan mapy 2D (D500). Sesja tylko z L2 nie jest pusta - ma stan 'tylko L2' (mapa w zakladce L2)."""
    with BLOK:
        if n in W_TOKU:
            return W_TOKU[n]
    lid = os.path.join(k, 'lidar.bin'); mj = os.path.join(k, '_wynik', 'mapa.json')
    cz = czujniki(k)
    if not cz['d500']:
        pliki = [p for p in (lid, os.path.join(k, 'l2.bin')) if os.path.exists(p)]
        if pliki and 'koniec' not in info(k) and time.time() - max(os.path.getmtime(p) for p in pliki) < 600:
            return 'nagrywanie'
        return 'tylko L2' if cz['l2'] else 'brak danych'
    if 'koniec' not in info(k) and time.time() - os.path.getmtime(lid) < 600:
        return 'nagrywanie'
    if os.path.exists(os.path.join(k, '_wynik', 'blad.txt')) and not os.path.exists(mj):
        return 'błąd'
    if os.path.exists(mj):
        if os.path.getmtime(mj) >= os.path.getmtime(lid) and wersja_mapy(mj) >= WERSJA_MAPY:
            return 'gotowe'
        return 'nieaktualna'          # starsza wersja programu albo dane doszly po policzeniu - przelicza tylko recznie
    if os.path.exists(os.path.join(k, '_wynik', 'zatrzymane.txt')):
        return 'zatrzymane'
    return 'do policzenia'


def _wersja_kodu():
    try:
        m = re.search(r'^WERSJA = (\d+)', open(os.path.join(REPO, 'mapa', 'przetworz.py'), encoding='utf-8').read(), re.M)
        return int(m.group(1)) if m else 0
    except OSError:
        return 0


WERSJA_MAPY = _wersja_kodu()     # mapy policzone starsza wersja przetworz.py licza sie od nowa (automat)


def wersja_mapy(mj):
    """Wersja z poczatku mapa.json ({"wersja":N,...}) - bez wczytywania calego pliku."""
    try:
        with open(mj, 'rb') as f:
            m = re.match(rb'\{"wersja":(\d+)', f.read(40))
        return int(m.group(1)) if m else 0
    except OSError:
        return 0


def sesje():
    out = []
    pp = postepy()
    for n in sorted(os.listdir(ARGS.katalog), reverse=True):
        k = katalog(n)
        if not k:
            continue
        i = info(k)
        pliki = os.listdir(k)
        rozm = sum(os.path.getsize(os.path.join(k, p)) for p in pliki if os.path.isfile(os.path.join(k, p)))
        czas = None
        try:
            from datetime import datetime
            if 'start' in i and 'koniec' in i:
                czas = (datetime.fromisoformat(i['koniec']) - datetime.fromisoformat(i['start'])).total_seconds()
        except ValueError:
            pass
        start = i.get('start', '')[:16].replace('T', ' ')
        if not start and n[:8].isdigit():
            start = f'{n[:4]}-{n[4:6]}-{n[6:8]} {n[9:11]}:{n[11:13]}'
        cz = czujniki(k)
        out.append({'n': n, 'budynek': i.get('budynek') or '(bez nazwy)', 'kondygnacja': i.get('kondygnacja') or '–',
                    'start': start, 'dzien': start[:10], 'godz': start[11:16], 'czas_s': czas, 'mb': round(rozm / 1e6, 1),
                    'foto': sum(p.startswith('foto_') for p in pliki), 'stan': stan_wyniku(n, k), 'postep': pp.get(n),
                    'd500': cz['d500'], 'l2': cz['l2'], 'l2_stan': stan_l2(n, k)['stan'] if cz['l2'] else None,
                    'odebrano': max((os.path.getmtime(os.path.join(k, p)) for p in pliki if p.endswith('.bin')), default=0)})
    return out


def znaczniki(k):
    out = []
    t0 = None
    try:
        with open(os.path.join(k, 'lidar.bin'), 'rb') as f:
            import struct
            t0 = struct.unpack('<Q', f.read(8))[0]
    except (OSError, Exception):
        pass
    p = os.path.join(k, 'znaczniki.txt')
    if os.path.exists(p):
        for l in open(p, encoding='utf-8'):
            z = l.rstrip('\n').split('\t')
            if len(z) >= 2 and z[0].isdigit():
                out.append({'t': round((int(z[0]) - t0) / 1e9, 1) if t0 else None,
                            'godz': time.strftime('%H:%M:%S', time.localtime(int(z[0]) / 1e9)),
                            'opis': z[1], 'wys': z[2] if len(z) > 2 else ''})
    return out


W_START = {}                   # sesja -> czas rozpoczecia liczenia mapy
PROC = {}                      # (rodzaj, sesja) -> Popen - do zatrzymywania z przegladarki
ZATRZ = set()                  # (rodzaj, sesja) zatrzymane recznie


def uruchom(klucz, cmd, **kw):
    """Jak subprocess.run, ale proces (z dziecmi - wlasna grupa) da sie zatrzymac przyciskiem."""
    if kw.pop('capture_output', False):
        kw['stdout'] = subprocess.PIPE; kw['stderr'] = subprocess.PIPE
    p = subprocess.Popen(cmd, start_new_session=True, **kw)
    with BLOK:
        PROC[klucz] = p
    try:
        out, err = p.communicate()
    finally:
        with BLOK:
            PROC.pop(klucz, None)
    return subprocess.CompletedProcess(cmd, p.returncode, out, err)


def zatrzymaj(n):
    """Zatrzymuje wszystko dla sesji: liczenie mapy, porownanie, dopasowanie CAD, kawalki; usuwa z kolejki."""
    import signal
    ile = 0
    with BLOK:
        for rodz in ('mapa', 'por', 'cad', 'kaw', 'l2'):
            p = PROC.get((rodz, n))
            if p:
                ZATRZ.add((rodz, n))
                try:
                    os.killpg(p.pid, signal.SIGKILL); ile += 1
                except OSError:
                    pass
        if W_TOKU.get(n) == 'czeka':
            W_TOKU.pop(n); ile += 1
        if W_POR.get(n) == 'czeka':
            W_POR.pop(n); ile += 1
        if W_CAD.get(n) == 'czeka':
            W_CAD.pop(n); ile += 1
        if W_L2.get(n) == 'czeka':
            W_L2.pop(n); ile += 1
    return ile

def zatrzymaj_i_czekaj(n, limit=20.0):
    """Zatrzymuje liczenie sesji i czeka, az watki skoncza sprzatanie (zapis znacznika zatrzymania), zeby
    po przeniesieniu katalogu do kosza nic juz do niego nie pisalo. True gdy sesja jest wolna."""
    koniec = time.time() + limit
    while time.time() < koniec:
        zatrzymaj(n)
        with BLOK:
            zajeta = (n in W_TOKU or n in W_POR or n in W_CAD or n in W_KAW or n in W_L2
                      or any(kl[1] == n for kl in PROC))
        if not zajeta:
            return True
        time.sleep(0.3)
    return False


def do_kosza(n):
    """Przenosi sesje do kosza, najpierw zatrzymujac jej liczenie. Zwraca komunikat bledu albo None."""
    if not zatrzymaj_i_czekaj(n):
        return 'nie udalo sie zatrzymac liczenia tej sesji - sprobuj za chwile'
    k = os.path.join(ARGS.katalog, n)
    if not os.path.isdir(k):
        return None
    kosz = os.path.join(ARGS.katalog, '_kosz'); os.makedirs(kosz, exist_ok=True)
    cel = os.path.join(kosz, n)
    if os.path.exists(cel):
        cel += time.strftime('_%Y%m%d%H%M%S')
    shutil.move(k, cel)
    return None


TEMPO_DOM = 6.0                # s liczenia na 1 MB lidar.bin, zanim zmierzymy na Dellu


def mb_lidar(k):
    try:
        return os.path.getsize(os.path.join(k, 'lidar.bin')) / 1e6
    except OSError:
        return 0.0


def _plik_tempa():
    return os.path.join(ARGS.katalog, '_tempo.json')


def zapisz_tempo(dt, mb):
    if mb < 1 or dt < 5: return
    try:
        T = json.load(open(_plik_tempa()))
    except (OSError, ValueError):
        T = []
    T = (T + [round(dt / mb, 2)])[-20:]
    try:
        json.dump(T, open(_plik_tempa(), 'w'))
    except OSError:
        pass


def tempo():
    try:
        T = sorted(json.load(open(_plik_tempa())))
        return T[len(T) // 2] if T else TEMPO_DOM
    except (OSError, ValueError):
        return TEMPO_DOM


ETAPY = {'dekodowanie': 0.02, 'zszywanie': 0.72, 'przekroje': 0.85, 'eksport': 0.92}


def ulamek(k):
    """Postep liczenia mapy z ostatniej linii 'POSTEP ...' w _wynik/log.txt (0..1)."""
    try:
        with open(os.path.join(k, '_wynik', 'log.txt'), 'rb') as f:
            f.seek(0, 2); f.seek(max(0, f.tell() - 6000))
            ost = [l for l in f.read().decode('utf-8', 'replace').splitlines() if l.startswith('POSTEP ')]
    except OSError:
        return 0.0
    if not ost: return 0.0
    p = ost[-1].split()
    if p[1] == 'mapa' and '/' in p[2]:
        i, n = p[2].split('/'); return 0.05 + 0.65 * int(i) / max(1, int(n))
    return ETAPY.get(p[2], 0.0) if p[1] == 'etap' and len(p) > 2 else 0.0


def postepy():
    """{sesja: {'proc', 'eta_s', 'poz'}} - liczone mapy i kolejka (szacunek z tempa na Dellu)."""
    teraz = time.time(); tp = tempo(); out = {}
    with BLOK:
        bieg = dict(W_START); czek = {n for n, s_ in W_TOKU.items() if s_ == 'czeka'}
    wolne = 0.0
    for n, t0 in bieg.items():
        k = katalog(n)
        if not k: continue
        f = ulamek(k); el = teraz - t0; szac = mb_lidar(k) * tp
        calosc = el / f if f >= 0.1 else max(szac, el * 1.1)
        eta = max(0.0, calosc - el)
        out[n] = {'proc': round(100 * f), 'eta_s': round(eta)}
        wolne = max(wolne, eta)
    kol = []
    for pr, nr, n in sorted(list(KOLEJKA.queue)):
        if n in czek and n not in kol: kol.append(n)
    acc = wolne
    for i, n in enumerate(kol):
        k = katalog(n)
        d = mb_lidar(k) * tp if k else 0
        acc += d
        out[n] = {'poz': i + 1, 'eta_s': round(acc)}
    return out


def pracownik():
    while True:
        _, _, n = KOLEJKA.get()
        k = katalog(n)
        with BLOK:
            if W_TOKU.get(n) != 'czeka':     # duplikat (przesuniete recznie na poczatek) albo juz policzone
                continue
            if not k:
                W_TOKU.pop(n, None); continue
        os.makedirs(os.path.join(k, '_wynik'), exist_ok=True)
        log = os.path.join(k, '_wynik', 'log.txt')
        with LICZ:
            with BLOK:
                W_TOKU[n] = 'liczy'; W_START[n] = time.time()
            with open(log, 'w') as f:
                r = uruchom(('mapa', n), [sys.executable, os.path.join(REPO, 'mapa', 'przetworz.py'), k], stdout=f, stderr=subprocess.STDOUT, env=ENV_DZIECI)
        with BLOK:
            dt = time.time() - W_START.pop(n, time.time())
        if r.returncode == 0:
            zapisz_tempo(dt, mb_lidar(k))
        blad = os.path.join(k, '_wynik', 'blad.txt')
        with BLOK:
            stop = ('mapa', n) in ZATRZ; ZATRZ.discard(('mapa', n))
        if stop:                                  # zatrzymane recznie - automat tego nie ponawia
            open(os.path.join(k, '_wynik', 'zatrzymane.txt'), 'w').write(time.strftime('%Y-%m-%d %H:%M'))
        elif r.returncode != 0:
            with open(log, errors='replace') as f, open(blad, 'w') as g:
                if r.returncode in (-9, 137):
                    g.write('Proces zabity przez system - zabraklo pamieci (OOM). Przelicz, gdy nic innego sie nie liczy.\n\n')
                g.write(f.read()[-3000:])
        elif os.path.exists(blad):
            os.remove(blad)
        with BLOK:
            W_TOKU.pop(n, None)


def pracownik_por():
    while True:
        n, metody = KOLEJKA_POR.get()
        k = katalog(n)
        sk = skrypt_por()
        if not k or not sk:
            with BLOK:
                W_POR.pop(n, None)
            continue
        wy = os.path.join(k, '_porownanie'); os.makedirs(wy, exist_ok=True)
        blad = os.path.join(wy, 'blad.txt')
        if os.path.exists(blad):
            os.remove(blad)
        with LICZ_RECZNE:
            with BLOK:
                W_POR[n] = 'liczy'
            with open(os.path.join(wy, 'przebieg.log'), 'w') as f:
                r = uruchom(('por', n), [sk, k, '--metody', ','.join(metody)], stdout=f, stderr=subprocess.STDOUT)
        with BLOK:
            stop = ('por', n) in ZATRZ; ZATRZ.discard(('por', n))
        if r.returncode != 0 and not stop:
            with open(os.path.join(wy, 'przebieg.log'), errors='replace') as f, open(blad, 'w') as g:
                g.write(f.read()[-3000:])
        with BLOK:
            W_POR.pop(n, None)


KOLEJKA_CAD = queue.Queue()
W_CAD = {}                     # sesja -> 'czeka' | 'liczy' (dopasowanie do rzutu CAD)


def katalog_cad(b):
    return os.path.join(ARGS.katalog, '_cad', re.sub(r'[^A-Za-z0-9_+\-]', '_', b or '') or '_')


def ustawienia_cad(b):
    p = os.path.join(katalog_cad(b), 'cad.json')
    try:
        return json.load(open(p, encoding='utf-8'))
    except (OSError, ValueError):
        return None


def numer_pietra(nazwa):
    """'-2 segment' -> -2, 'Parter 1' -> 0, '1 pietro segment 2' -> 1, '2 piętro' -> 2; inaczej None."""
    t = (nazwa or '').strip().lower()
    if t.startswith('parter'):
        return 0
    m = re.match(r'^\s*([+-]?\d+)', t)
    return int(m.group(1)) if m else None


def pracownik_cad():
    while True:
        n, klaster, zrodlo = KOLEJKA_CAD.get()
        k = katalog(n)
        if k:
            wy = os.path.join(k, '_wynik'); os.makedirs(wy, exist_ok=True)
            blad = os.path.join(wy, 'cad_blad.txt')
            if os.path.exists(blad): os.remove(blad)
            b_ = info(k).get('budynek', '')
            cmd = [sys.executable, os.path.join(REPO, 'mapa', 'dopasuj_cad.py'), k, katalog_cad(b_), '--zrodlo', zrodlo]
            if klaster is not None: cmd += ['--klaster', str(klaster)]
            else:                                     # podpowiedz kondygnacji z nazw sesji budynku ("-2 segment", "Parter", "1 pietro")
                f = numer_pietra(info(k).get('kondygnacja', ''))
                wsz = sorted({x for x in (numer_pietra(s_['kondygnacja']) for s_ in sesje() if s_['budynek'] == b_) if x is not None}, reverse=True)
                if f is not None and wsz:
                    cmd += ['--pietro', str(f), '--pietra', ','.join(map(str, range(wsz[0], wsz[-1] - 1, -1)))]
            with LICZ_RECZNE:
                with BLOK:
                    W_CAD[n] = 'liczy'
                r = uruchom(('cad', n), cmd, capture_output=True, text=True)
            with open(os.path.join(wy, 'cad_log.txt'), 'w') as f:
                f.write((r.stdout or '') + (r.stderr or ''))
            with BLOK:
                stop = ('cad', n) in ZATRZ; ZATRZ.discard(('cad', n))
            if r.returncode != 0 and not stop:
                with open(blad, 'w') as f:
                    f.write((r.stdout + r.stderr)[-3000:])
        with BLOK:
            W_CAD.pop(n, None)


W_KAW = {}                     # sesja -> 'liczy' (ciecie mapy na kawalki do recznego sklejania w CAD)


def licz_kawalki(n, k):
    wy = os.path.join(k, '_wynik')
    try:
        with LICZ_RECZNE:
            with BLOK:
                W_KAW[n] = 'liczy'
            r = uruchom(('kaw', n), [sys.executable, os.path.join(REPO, 'mapa', 'kawalki.py'), k], capture_output=True, text=True, env=ENV_DZIECI)
        with open(os.path.join(wy, 'kawalki_log.txt'), 'w') as f:
            f.write((r.stdout or '') + (r.stderr or ''))
        with BLOK:
            ZATRZ.discard(('kaw', n))
    finally:
        with BLOK:
            W_KAW.pop(n, None)


def stan_cad(n, k):
    b = info(k).get('budynek', '')
    wy = os.path.join(k, '_wynik')
    with BLOK:
        st = W_CAD.get(n)
    wyn = None
    if os.path.exists(os.path.join(wy, 'cad.json')):
        try:
            wyn = json.load(open(os.path.join(wy, 'cad.json'), encoding='utf-8'))
        except ValueError:
            pass
    if not st:
        st = 'błąd' if os.path.exists(os.path.join(wy, 'cad_blad.txt')) else ('gotowe' if wyn else 'brak')
    zrodla = ['nasz'] + [m for m in METODY_POR if os.path.exists(os.path.join(k, '_porownanie', f'k0_{m}.pgm'))]
    return {'stan': st, 'wynik': wyn, 'budynek': b, 'rzut': ustawienia_cad(b), 'zrodla': zrodla,
            'png': os.path.exists(os.path.join(wy, 'cad_porownanie.png')), 'dxf': os.path.exists(os.path.join(wy, 'rzut_cad.dxf')),
            'blad': os.path.exists(os.path.join(wy, 'cad_blad.txt'))}


def stan_por(n, k):
    wy = os.path.join(k, '_porownanie')
    with BLOK:
        st = W_POR.get(n)
    pliki = sorted('_porownanie/' + p for p in os.listdir(wy)
                   if re.match(r'^k\d+_[a-z_]+\.(png|dxf)$', p) or p in ('przebieg.log', 'blad.txt')) if os.path.isdir(wy) else []
    wyn = {}
    if os.path.exists(os.path.join(wy, 'wynik.json')):
        try:
            wyn = json.load(open(os.path.join(wy, 'wynik.json'), encoding='utf-8'))
        except ValueError:
            pass
    if not st:
        st = 'błąd' if os.path.exists(os.path.join(wy, 'blad.txt')) else ('gotowe' if wyn else 'brak')
    return {'stan': st, 'pliki': pliki, 'wynik': wyn, 'dostepne': bool(skrypt_por()), 'metody': list(METODY_POR)}


KOLEJKA_L2 = queue.Queue()
W_L2 = {}                      # sesja -> 'czeka' | 'liczy' (mapa 3D z Unitree L2: l2.bin -> rosbag -> Point-LIO -> rzut)
W_START_L2 = {}


def skrypt_l2():
    """Polecenie l2_mapa z l2/instaluj_l2.sh (ROS 1 + Point-LIO w ~/ros1_l2)."""
    for p in (os.environ.get('LIDAR_L2', ''), os.path.expanduser('~/bin/l2_mapa')):
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def pracownik_l2():
    """Point-LIO odtwarza nagranie w czasie rzeczywistym, wiec liczenie trwa mniej wiecej tyle, co nagranie.
    Jedno naraz (roscore na stalym porcie 11411), w tym samym limicie co mapy 2D (pamiec Della)."""
    while True:
        n = KOLEJKA_L2.get()
        k = katalog(n); sk = skrypt_l2()
        with BLOK:
            if W_L2.get(n) != 'czeka':
                continue
            if not k or not sk:
                W_L2.pop(n, None); continue
        wy = os.path.join(k, '_wynik', 'l2'); os.makedirs(wy, exist_ok=True)
        for p in ('blad.txt', 'zatrzymane.txt'):
            if os.path.exists(os.path.join(wy, p)):
                os.remove(os.path.join(wy, p))
        log = os.path.join(wy, 'przebieg.log')
        with LICZ:
            with BLOK:
                W_L2[n] = 'liczy'; W_START_L2[n] = time.time()
            with open(log, 'w') as f:
                r = uruchom(('l2', n), [sk, k], stdout=f, stderr=subprocess.STDOUT, env=ENV_DZIECI)
        with BLOK:
            W_START_L2.pop(n, None)
            stop = ('l2', n) in ZATRZ; ZATRZ.discard(('l2', n))
        if stop:
            open(os.path.join(wy, 'zatrzymane.txt'), 'w').write(time.strftime('%Y-%m-%d %H:%M'))
        elif r.returncode != 0:
            with open(log, errors='replace') as f, open(os.path.join(wy, 'blad.txt'), 'w') as g:
                g.write(f.read()[-3000:])
        with BLOK:
            W_L2.pop(n, None)


def stan_l2(n, k):
    wy = os.path.join(k, '_wynik', 'l2')
    with BLOK:
        st = W_L2.get(n); t0 = W_START_L2.get(n)
    jest = (lambda p: os.path.exists(os.path.join(wy, p)))
    pliki = sorted('_wynik/l2/' + p for p in os.listdir(wy)
                   if p in ('rzut.png', 'rzut.dxf', 'wysokosci.txt', 'chmura.pcd', 'przebieg.log', 'blad.txt', 'pointlio.log', 'otwory.csv')) if os.path.isdir(wy) else []
    if not st:
        st = 'błąd' if jest('blad.txt') else 'gotowe' if (jest('rzut.png') or jest('rzut.dxf')) else 'zatrzymane' if jest('zatrzymane.txt') else 'brak'
    wys = {}
    if jest('wysokosci.txt'):
        for l in open(os.path.join(wy, 'wysokosci.txt'), encoding='utf-8', errors='replace'):
            if '=' in l:
                a, b = l.strip().split('=', 1); wys[a] = b
    czas = None
    i = info(k)
    try:
        from datetime import datetime
        czas = (datetime.fromisoformat(i['koniec']) - datetime.fromisoformat(i['start'])).total_seconds()
    except (KeyError, ValueError):
        pass
    otw = []
    if jest('otwory.json'):
        try:
            otw = json.load(open(os.path.join(wy, 'otwory.json'), encoding='utf-8'))
        except ValueError:
            pass
    return {'stan': st, 'pliki': pliki, 'wysokosci': wys, 'otwory': otw, 'dostepne': bool(skrypt_l2()),
            'liczy_s': round(time.time() - t0) if t0 else None, 'nagranie_s': czas, 'mb': round(rozmiar(os.path.join(k, 'l2.bin')) / 1e6, 1)}


def zlec(n, pilne=False):
    with BLOK:
        if n in W_TOKU:
            return
        W_TOKU[n] = 'czeka'
    KOLEJKA.put((0 if pilne else 1, next(NR_ZLEC), n))


def automat():
    """Co minute: zakonczone sesje BEZ mapy trafiaja do kolejki (bledne nie sa ponawiane w kolko).
    Mapy nieaktualne (stara wersja, dane doszly pozniej) przelicza sie tylko recznie - przyciskiem albo zaznaczajac."""
    while True:
        try:
            for s in sesje():
                k = katalog(s['n'])
                if s['stan'] == 'do policzenia' and not os.path.exists(os.path.join(k, '_wynik', 'blad.txt')) \
                        and not os.path.exists(os.path.join(k, '_wynik', 'zatrzymane.txt')):
                    zlec(s['n'])
        except OSError:
            pass
        time.sleep(60)


LOGOWANIE = """<!doctype html><html lang="pl"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pomiary LiDAR - logowanie</title><style>body{margin:0;height:100vh;display:flex;align-items:center;justify-content:center;background:#111827;font:15px system-ui,sans-serif}
form{background:#fff;padding:28px;border-radius:14px;width:min(340px,90vw);box-shadow:0 20px 50px rgba(0,0,0,.4)}h1{font-size:18px;margin:0 0 16px}
input{width:100%;padding:10px;border:1px solid #d1d5db;border-radius:8px;font:inherit;box-sizing:border-box}button{margin-top:12px;width:100%;padding:10px;border:0;border-radius:8px;background:#2563eb;color:#fff;font:inherit;cursor:pointer}
.b{color:#dc2626;margin:8px 0 0}</style></head><body><form method="post" action="/zaloguj"><h1>📐 Pomiary LiDAR</h1>
<input type="password" name="haslo" placeholder="hasło" autofocus><!--blad--><button>Zaloguj</button></form></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _zewnetrzny(self):
        """Ruch z internetu: przez tunel Cloudflare (naglowki CF-*) albo z adresu publicznego."""
        if self.headers.get('CF-Connecting-IP') or self.headers.get('CF-Ray'):
            return True
        try:
            ip = ipaddress.ip_address(self.client_address[0])
            return not (ip.is_private or ip.is_loopback)
        except ValueError:
            return True

    def _zalogowany(self):
        sekret = os.environ.get('LIDAR_SEKRET', '')
        for c in (self.headers.get('Cookie') or '').split(';'):
            a, _, b = c.strip().partition('=')
            if a == 'lidar' and sekret and '.' in b:
                wazne, podpis = b.split('.', 1)
                ok = hmac.compare_digest(podpis, hmac.new(sekret.encode(), wazne.encode(), 'sha256').hexdigest())
                if ok and wazne.isdigit() and int(wazne) > time.time():
                    return True
        return False

    def _lan(self):
        """Siec domowa bez hasla; z zewnatrz (Cloudflare) tylko po zalogowaniu."""
        if not self._zewnetrzny() or self._zalogowany():
            return True
        u = urlparse(self.path)
        if not os.environ.get('LIDAR_HASLO_HASH'):
            self.send_error(403, 'Dostep z zewnatrz wylaczony (brak hasla na serwerze)')
        elif self.command == 'GET' and u.path == '/':
            self._wyslij(200, 'text/html; charset=utf-8', LOGOWANIE.encode())
        else:
            self._json({'blad': 'zaloguj sie'}, 401)
        return False

    def _logowanie(self):
        dl = int(self.headers.get('Content-Length') or 0)
        q = parse_qs(self.rfile.read(min(dl, 2000)).decode('utf-8', 'replace'))
        haslo = (q.get('haslo') or [''])[0]
        wzor = os.environ.get('LIDAR_HASLO_HASH', '')
        if wzor and hmac.compare_digest(hashlib.sha256(haslo.encode()).hexdigest(), wzor):
            wazne = str(int(time.time()) + 30 * 86400)
            podpis = hmac.new(os.environ.get('LIDAR_SEKRET', '').encode(), wazne.encode(), 'sha256').hexdigest()
            self.send_response(303)
            self.send_header('Location', '/')
            self.send_header('Set-Cookie', f'lidar={wazne}.{podpis}; Max-Age={30 * 86400}; Path=/; HttpOnly; Secure; SameSite=Lax')
            self.end_headers()
        else:
            time.sleep(1.5)
            self._wyslij(200, 'text/html; charset=utf-8', LOGOWANIE.replace('<!--blad-->', '<p class="b">Złe hasło</p>').encode())

    def _wyslij(self, kod, typ, dane, naglowki=()):
        self.send_response(kod)
        self.send_header('Content-Type', typ); self.send_header('Content-Length', str(len(dane)))
        self.send_header('Cache-Control', 'no-store')
        for a, b in naglowki:
            self.send_header(a, b)
        self.end_headers(); self.wfile.write(dane)

    def _json(self, obj, kod=200):
        self._wyslij(kod, 'application/json; charset=utf-8', json.dumps(obj, ensure_ascii=False).encode())

    def do_GET(self):
        if not self._lan():
            return
        u = urlparse(self.path); q = {a: b[0] for a, b in parse_qs(u.query).items()}
        if u.path == '/':
            with open(os.path.join(TU, 'przegladarka.html'), 'rb') as f:
                return self._wyslij(200, 'text/html; charset=utf-8', f.read())
        if u.path in ('/static/three.module.js', '/static/OrbitControls.js'):
            with open(os.path.join(TU, 'static', os.path.basename(u.path)), 'rb') as f:
                return self._wyslij(200, 'text/javascript; charset=utf-8', f.read(), [('Cache-Control', 'max-age=86400')])
        if u.path == '/api/sesje':
            return self._json(sesje())
        k = katalog(q.get('n'))
        if u.path.startswith('/api/') and not k:
            return self._json({'blad': 'nie ma takiej sesji'}, 404)
        if u.path == '/api/sesja':
            pliki = sorted(p for p in os.listdir(k) if os.path.isfile(os.path.join(k, p)))
            wyn = sorted('_wynik/' + p for p in os.listdir(os.path.join(k, '_wynik'))
                         if p in ('rzut.dxf', 'podglad.png', 'przekroje.dxf', 'model3d.dxf', 'model3d.obj', 'log.txt', 'blad.txt')
                         or (p.startswith('przekroj_') and p.endswith('.jpg'))
                         or (p.startswith('porownanie_') and p.endswith('.png'))) if os.path.isdir(os.path.join(k, '_wynik')) else []
            pj = os.path.join(k, '_wynik', 'przekroje.json')
            prz = json.load(open(pj, encoding='utf-8')) if os.path.exists(pj) else []
            return self._json({'info': info(k), 'stan': stan_wyniku(q['n'], k), 'postep': postepy().get(q['n']), 'pliki': pliki, 'wyniki': wyn, 'przekroje': prz,
                               'porownanie': stan_por(q['n'], k), 'cad': stan_cad(q['n'], k), 'czujniki': czujniki(k), 'l2': stan_l2(q['n'], k),
                               'kawalki': {'stan': W_KAW.get(q['n']) or ('gotowe' if os.path.exists(os.path.join(k, '_wynik', 'kawalki.dxf')) else
                                           ('brak danych' if not os.path.exists(os.path.join(k, '_wynik', 'kf.npz')) else 'brak'))},
                               'znaczniki': znaczniki(k), 'oprawy': SW.wiersze_pom(k), 'grzejniki': SW.wiersze_grz(k),
                               'okna': SW.wiersze_okn(k)})
        if u.path == '/api/mapa':
            p = os.path.join(k, '_wynik', 'mapa.json')
            if not os.path.exists(p):
                return self._json({'blad': 'mapa jeszcze nie policzona'}, 404)
            with open(p, 'rb') as f:
                dane = f.read()
            nz = json.dumps(__import__('json').load(open(os.path.join(k, '_wynik', 'nazwy.json'), encoding='utf-8'))
                            if os.path.exists(os.path.join(k, '_wynik', 'nazwy.json')) else {}, ensure_ascii=False)
            pk = os.path.join(k, 'kontrola.json')
            ko = open(pk, 'rb').read() if os.path.exists(pk) else b'[]'
            return self._wyslij(200, 'application/json; charset=utf-8', b'{"mapa":' + dane + b',"nazwy":' + nz.encode() + b',"kontrola":' + ko + b'}')
        if u.path == '/plik':
            f = q.get('f', '')
            if not k or not PLIK_RE.match(f):
                return self.send_error(404)
            p = os.path.join(k, f)
            if not os.path.isfile(p):
                return self.send_error(404)
            typ = {'jpg': 'image/jpeg', 'png': 'image/png', 'json': 'application/json', 'txt': 'text/plain; charset=utf-8',
                   'csv': 'text/csv; charset=utf-8', 'log': 'text/plain; charset=utf-8'}.get(f.rsplit('.', 1)[1], 'application/octet-stream')
            jedn = q.get('jedn', 'm')
            nazwa = os.path.basename(f)
            if f.endswith('.dxf') and jedn in ('cm', 'mm') and not nazwa.startswith('rzut_cad'):
                # rysunek zapisany w metrach -> przeliczenie na cm/mm (skala 1:1 w CAD), warstwy bez zmian
                sys.path.insert(0, os.path.join(REPO, 'mapa'))
                from dxf_wspolne import w_jednostkach
                dane = w_jednostkach(p, jedn)
                nazwa = nazwa[:-4] + '_' + jedn + '.dxf'
            else:
                with open(p, 'rb') as fh:
                    dane = fh.read()
            nag = [('Content-Disposition', f'attachment; filename="{q["n"]}_{nazwa}"')] \
                if f.endswith(('.dxf', '.bin', '.csv', '.pcd')) else []
            return self._wyslij(200, typ, dane, nag)
        self.send_error(404)

    def do_POST(self):
        if urlparse(self.path).path == '/zaloguj':
            return self._logowanie()
        if not self._lan():
            return
        u = urlparse(self.path); q = {a: b[0] for a, b in parse_qs(u.query).items()}
        if u.path == '/api/usun_wszystkie_mapy':
            ile = 0
            for n in os.listdir(ARGS.katalog):
                kk = katalog(n)
                if kk and os.path.isdir(os.path.join(kk, '_wynik')):
                    with BLOK:
                        if W_TOKU.get(n) == 'liczy':
                            continue
                    usun_wynik_2d(kk); ile += 1
            return self._json({'ok': True, 'ile': ile})
        if u.path in ('/api/usun_budynek', '/api/usun_puste'):
            # takze sesje w trakcie liczenia: liczenie jest najpierw zatrzymywane
            ile, nie = 0, 0
            for s_ in sesje():
                if u.path == '/api/usun_budynek' and s_['budynek'] != q.get('b'):
                    continue
                if u.path == '/api/usun_puste' and s_['stan'] != 'brak danych':
                    continue
                if do_kosza(s_['n']):
                    nie += 1
                else:
                    ile += 1
            return self._json({'ok': True, 'ile': ile, 'nie': nie})
        if u.path == '/api/przywroc_kosz':
            # przywrocenie wszystkich sesji z kosza (np. po pomylkowym skasowaniu); nie nadpisuje istniejacych
            kosz = os.path.join(ARGS.katalog, '_kosz'); ile, pom = 0, 0
            for n in sorted(os.listdir(kosz)) if os.path.isdir(kosz) else []:
                nazwa = re.sub(r'_\d{14}$', '', n)
                cel = os.path.join(ARGS.katalog, nazwa)
                if os.path.exists(cel):
                    pom += 1; continue
                shutil.move(os.path.join(kosz, n), cel); ile += 1
            return self._json({'ok': True, 'ile': ile, 'pominieto': pom})
        if u.path == '/api/cad_wgraj':
            b = q.get('b', '')
            if not b:
                return self._json({'blad': 'brak nazwy budynku'}, 400)
            dl = int(self.headers.get('Content-Length') or 0)
            if dl <= 0 or dl > 200 * 1024 * 1024:
                return self._json({'blad': 'plik pusty albo za duzy (max 200 MB)'}, 400)
            dane = self.rfile.read(dl)
            kc = katalog_cad(b); os.makedirs(kc, exist_ok=True)
            tmp = os.path.join(kc, 'nowy.dxf')
            with open(tmp, 'wb') as f:
                f.write(dane)
            try:
                sys.path.insert(0, os.path.join(REPO, 'mapa'))
                from dopasuj_cad import warstwy_dxf, domyslna_warstwa
                lista, jedn_plik = warstwy_dxf(tmp)
            except Exception as e:
                os.remove(tmp)
                return self._json({'blad': f'nie umiem odczytac DXF: {e}'}, 400)
            if not lista:
                os.remove(tmp)
                return self._json({'blad': 'w pliku nie ma linii ani polilinii'}, 400)
            os.replace(tmp, os.path.join(kc, 'rzut.dxf'))
            ust = {'nazwa_pliku': q.get('nazwa', 'rzut.dxf')[:120], 'warstwy': lista[:200], 'warstwa': domyslna_warstwa(lista),
                   'jedn': 'auto', 'jedn_plik': jedn_plik, 'wgrano': time.strftime('%Y-%m-%d %H:%M')}
            with open(os.path.join(kc, 'cad.json'), 'w', encoding='utf-8') as f:
                json.dump(ust, f, ensure_ascii=False)
            return self._json({'ok': True, 'rzut': ust})
        if u.path == '/api/cad_ustaw':
            b = q.get('b', ''); ust = ustawienia_cad(b)
            if not ust:
                return self._json({'blad': 'najpierw wgraj rzut DXF'}, 404)
            if q.get('warstwa') in [w for w, _ in ust['warstwy']]:
                ust['warstwa'] = q['warstwa']
            if q.get('jedn') in ('auto', 'm', 'cm', 'mm'):
                ust['jedn'] = q['jedn']
            with open(os.path.join(katalog_cad(b), 'cad.json'), 'w', encoding='utf-8') as f:
                json.dump(ust, f, ensure_ascii=False)
            return self._json({'ok': True, 'rzut': ust})
        if u.path == '/api/zatrzymaj_wszystko':
            with BLOK:
                nazwy = set(W_TOKU) | set(W_POR) | set(W_CAD) | set(W_KAW) | set(W_L2) | {n_ for _, n_ in PROC}
            ile = sum(zatrzymaj(n_) for n_ in nazwy)
            return self._json({'ok': True, 'ile': ile})
        if u.path == '/api/oproznij_kosz':
            shutil.rmtree(os.path.join(ARGS.katalog, '_kosz'), ignore_errors=True)
            return self._json({'ok': True})
        k = katalog(q.get('n'))
        if not k:
            return self._json({'blad': 'nie ma takiej sesji'}, 404)
        if u.path == '/api/przelicz':
            if not czujniki(k)['d500']:
                return self._json({'blad': 'w tej sesji nie nagrywal D500 - mape liczy sie w zakladce „Unitree L2”'}, 409)
            for b in ('blad.txt', 'zatrzymane.txt'):
                b = os.path.join(k, '_wynik', b)
                if os.path.exists(b):
                    os.remove(b)
            with BLOK:                       # juz czeka w kolejce automatu -> przesun na poczatek
                if W_TOKU.get(q['n']) == 'czeka':
                    W_TOKU.pop(q['n'])
            zlec(q['n'], pilne=True)
            return self._json({'ok': True})
        if u.path == '/api/l2_licz':
            if not czujniki(k)['l2']:
                return self._json({'blad': 'w tej sesji nie ma danych z L2 (l2.bin)'}, 409)
            if not skrypt_l2():
                return self._json({'blad': 'Brak programu l2_mapa - uruchom na Dellu: bash ~/GOLlidar/l2/instaluj_l2.sh'}, 409)
            with BLOK:
                if q['n'] in W_L2:
                    return self._json({'blad': 'mapa L2 tej sesji juz jest w kolejce'}, 409)
                W_L2[q['n']] = 'czeka'
            KOLEJKA_L2.put(q['n'])
            return self._json({'ok': True})
        if u.path == '/api/porownaj':
            if not skrypt_por():
                return self._json({'blad': 'Brak programu porownaj - uruchom na Dellu: bash ~/GOLlidar/porownanie/instaluj.sh'}, 409)
            metody = [m for m in (q.get('metody') or 'carto_imu').split(',') if m in METODY_POR]
            if not metody:
                return self._json({'blad': 'nieznana metoda'}, 400)
            with BLOK:
                if q['n'] in W_POR:
                    return self._json({'blad': 'porownanie tej sesji juz jest w kolejce'}, 409)
                W_POR[q['n']] = 'czeka'
            KOLEJKA_POR.put((q['n'], metody))
            return self._json({'ok': True})
        if u.path == '/api/zatrzymaj':
            return self._json({'ok': True, 'ile': zatrzymaj(q['n'])})
        if u.path == '/api/kawalki':
            if not os.path.exists(os.path.join(k, '_wynik', 'kf.npz')):
                return self._json({'blad': 'najpierw przelicz mape ta wersja programu (brak kf.npz)'}, 409)
            with BLOK:
                if q['n'] in W_KAW:
                    return self._json({'blad': 'juz sie liczy'}, 409)
                W_KAW[q['n']] = 'czeka'
            threading.Thread(target=licz_kawalki, args=(q['n'], k), daemon=True).start()
            return self._json({'ok': True})
        if u.path == '/api/dopasuj':
            if not ustawienia_cad(info(k).get('budynek', '')):
                return self._json({'blad': 'najpierw wgraj rzut DXF tego budynku'}, 409)
            zr = q.get('zrodlo', 'nasz')
            if zr != 'nasz' and zr not in METODY_POR:
                return self._json({'blad': 'nieznane zrodlo'}, 400)
            if zr == 'nasz' and not os.path.exists(os.path.join(k, '_wynik', 'mapa.json')):
                return self._json({'blad': 'mapa tej sesji nie jest jeszcze policzona'}, 409)
            kl = q.get('klaster')
            kl = int(kl) if kl not in (None, '') and kl.isdigit() else None
            with BLOK:
                if q['n'] in W_CAD:
                    return self._json({'blad': 'dopasowanie juz jest w kolejce'}, 409)
                W_CAD[q['n']] = 'czeka'
            KOLEJKA_CAD.put((q['n'], kl, zr))
            return self._json({'ok': True})
        if u.path == '/api/usun_mape':
            if not zatrzymaj_i_czekaj(q['n']):
                return self._json({'blad': 'nie udalo sie zatrzymac liczenia - sprobuj za chwile'}, 409)
            usun_wynik_2d(k)
            return self._json({'ok': True})
        if u.path == '/api/usun_sesje':
            b_ = do_kosza(q['n'])
            return self._json({'blad': b_}, 409) if b_ else self._json({'ok': True})
        if u.path == '/api/kontrola':
            dl = int(self.headers.get('Content-Length') or 0)
            try:
                L = json.loads(self.rfile.read(min(dl, 200000)) or b'[]')
                L = [{'seg': int(x['seg']), 'a': [float(v) for v in x['a'][:2]], 'b': [float(v) for v in x['b'][:2]],
                      'laser_m': float(x['laser_m']), 'opis': str(x.get('opis', ''))[:80], 'kiedy': str(x.get('kiedy', ''))[:30]} for x in L]
            except (ValueError, KeyError, TypeError):
                return self._json({'blad': 'zle dane'}, 400)
            with open(os.path.join(k, 'kontrola.json'), 'w', encoding='utf-8') as f:
                json.dump(L, f, ensure_ascii=False)
            return self._json({'ok': True})
        if u.path == '/api/nazwy':
            dl = int(self.headers.get('Content-Length') or 0)
            try:
                nz = json.loads(self.rfile.read(min(dl, 100000)) or b'{}')
                nz = {str(int(a)): {'nazwa': str(b.get('nazwa', ''))[:80], 'ukryj': bool(b.get('ukryj')),
                                    'obrot': round(float(b.get('obrot') or 0) % 360, 2)} for a, b in nz.items()}
            except (ValueError, AttributeError):
                return self._json({'blad': 'zle dane'}, 400)
            os.makedirs(os.path.join(k, '_wynik'), exist_ok=True)
            with open(os.path.join(k, '_wynik', 'nazwy.json'), 'w', encoding='utf-8') as f:
                json.dump(nz, f, ensure_ascii=False)
            if os.path.exists(os.path.join(k, '_wynik', 'mapa.json')):
                subprocess.run([sys.executable, os.path.join(REPO, 'mapa', 'przetworz.py'), k, '--tylko-dxf'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return self._json({'ok': True})
        self.send_error(404)




def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8150)
    ap.add_argument('--katalog', default=os.environ.get('LIDAR_DIR', os.path.expanduser('~/skany-lidar')))
    ARGS = ap.parse_args()
    for _ in range(LICZ.max):          # kilka map naraz (Limit pilnuje pamieci)
        threading.Thread(target=pracownik, daemon=True).start()
    threading.Thread(target=automat, daemon=True).start()
    threading.Thread(target=pracownik_por, daemon=True).start()
    threading.Thread(target=pracownik_cad, daemon=True).start()
    threading.Thread(target=pracownik_l2, daemon=True).start()
    print(f'Przegladarka: http://<adres-serwera>:{ARGS.port}  katalog: {ARGS.katalog}')
    ThreadingHTTPServer(('0.0.0.0', ARGS.port), H).serve_forever()


if __name__ == '__main__':
    main()

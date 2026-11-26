#!/usr/bin/env python3
"""Przegladarka pomiarow LiDAR na serwerze (Dell) - tylko siec lokalna.

Pokazuje nagrania z LIDAR_DIR pogrupowane budynek -> kondygnacja -> sesja: mape z warstwami (sciany, chmura punktow,
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
PLIK_RE = re.compile(r'^((_wynik|_porownanie)/)?[A-Za-z0-9_\-]{1,60}\.(jpg|csv|txt|bin|dxf|png|json|obj|log)$')
KOLEJKA = queue.Queue()
W_TOKU = {}                    # sesja -> 'czeka' | 'liczy'
BLOK = threading.Lock()
LICZ = threading.Lock()        # jedno ciezkie liczenie naraz (mapa albo porownanie) - oszczednosc RAM
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


def stan_wyniku(n, k):
    with BLOK:
        if n in W_TOKU:
            return W_TOKU[n]
    lid = os.path.join(k, 'lidar.bin'); mj = os.path.join(k, '_wynik', 'mapa.json')
    if not os.path.exists(lid):
        return 'brak danych'
    if 'koniec' not in info(k) and time.time() - os.path.getmtime(lid) < 600:
        return 'nagrywanie'
    if os.path.exists(os.path.join(k, '_wynik', 'blad.txt')) and not os.path.exists(mj):
        return 'błąd'
    if os.path.exists(mj) and os.path.getmtime(mj) >= os.path.getmtime(lid) and wersja_mapy(mj) >= WERSJA_MAPY:
        return 'gotowe'
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
        out.append({'n': n, 'budynek': i.get('budynek') or '(bez nazwy)', 'kondygnacja': i.get('kondygnacja') or '–',
                    'start': i.get('start', '')[:16].replace('T', ' '), 'czas_s': czas, 'mb': round(rozm / 1e6, 1),
                    'foto': sum(p.startswith('foto_') for p in pliki), 'stan': stan_wyniku(n, k)})
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


def pracownik():
    while True:
        n = KOLEJKA.get()
        k = katalog(n)
        if not k:
            continue
        with BLOK:
            W_TOKU[n] = 'liczy'
        os.makedirs(os.path.join(k, '_wynik'), exist_ok=True)
        log = os.path.join(k, '_wynik', 'log.txt')
        with LICZ, open(log, 'w') as f:
            r = subprocess.run([sys.executable, os.path.join(REPO, 'mapa', 'przetworz.py'), k], stdout=f, stderr=subprocess.STDOUT)
        blad = os.path.join(k, '_wynik', 'blad.txt')
        if r.returncode != 0:
            with open(log) as f, open(blad, 'w') as g:
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
        with LICZ:
            with BLOK:
                W_POR[n] = 'liczy'
            with open(os.path.join(wy, 'przebieg.log'), 'w') as f:
                r = subprocess.run([sk, k, '--metody', ','.join(metody)], stdout=f, stderr=subprocess.STDOUT)
        if r.returncode != 0:
            with open(os.path.join(wy, 'przebieg.log'), errors='replace') as f, open(blad, 'w') as g:
                g.write(f.read()[-3000:])
        with BLOK:
            W_POR.pop(n, None)


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


def zlec(n):
    with BLOK:
        if n in W_TOKU:
            return
        W_TOKU[n] = 'czeka'
    KOLEJKA.put(n)


def automat():
    """Co minute: zakonczone sesje bez aktualnej mapy trafiaja do kolejki (bledne nie sa ponawiane w kolko)."""
    while True:
        try:
            for s in sesje():
                k = katalog(s['n'])
                if s['stan'] == 'do policzenia' and not os.path.exists(os.path.join(k, '_wynik', 'blad.txt')):
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
            return self._json({'info': info(k), 'stan': stan_wyniku(q['n'], k), 'pliki': pliki, 'wyniki': wyn, 'przekroje': prz,
                               'porownanie': stan_por(q['n'], k),
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
            with open(p, 'rb') as fh:
                dane = fh.read()
            nag = [('Content-Disposition', f'attachment; filename="{q["n"]}_{os.path.basename(f)}"')] \
                if f.endswith(('.dxf', '.bin', '.csv')) else []
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
                    shutil.rmtree(os.path.join(kk, '_wynik'), ignore_errors=True); ile += 1
            return self._json({'ok': True, 'ile': ile})
        if u.path in ('/api/usun_budynek', '/api/usun_puste'):
            kosz = os.path.join(ARGS.katalog, '_kosz'); os.makedirs(kosz, exist_ok=True)
            ile = 0
            for s_ in sesje():
                if u.path == '/api/usun_budynek' and s_['budynek'] != q.get('b'):
                    continue
                if u.path == '/api/usun_puste' and s_['stan'] != 'brak danych':
                    continue
                with BLOK:
                    if s_['n'] in W_TOKU:
                        continue
                cel = os.path.join(kosz, s_['n'])
                if os.path.exists(cel):
                    cel += time.strftime('_%Y%m%d%H%M%S')
                shutil.move(os.path.join(ARGS.katalog, s_['n']), cel); ile += 1
            return self._json({'ok': True, 'ile': ile})
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
        if u.path == '/api/oproznij_kosz':
            shutil.rmtree(os.path.join(ARGS.katalog, '_kosz'), ignore_errors=True)
            return self._json({'ok': True})
        k = katalog(q.get('n'))
        if not k:
            return self._json({'blad': 'nie ma takiej sesji'}, 404)
        if u.path == '/api/przelicz':
            b = os.path.join(k, '_wynik', 'blad.txt')
            if os.path.exists(b):
                os.remove(b)
            zlec(q['n'])
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
        if u.path == '/api/usun_mape':
            with BLOK:
                if W_TOKU.get(q['n']) == 'liczy':
                    return self._json({'blad': 'mapa wlasnie sie liczy - sprobuj za chwile'}, 409)
            shutil.rmtree(os.path.join(k, '_wynik'), ignore_errors=True)
            return self._json({'ok': True})
        if u.path == '/api/usun_sesje':
            with BLOK:
                if q['n'] in W_TOKU:
                    return self._json({'blad': 'mapa tej sesji wlasnie sie liczy - sprobuj za chwile'}, 409)
            kosz = os.path.join(ARGS.katalog, '_kosz'); os.makedirs(kosz, exist_ok=True)
            cel = os.path.join(kosz, q['n'])
            if os.path.exists(cel):
                cel += time.strftime('_%Y%m%d%H%M%S')
            shutil.move(k, cel)
            return self._json({'ok': True})
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
    threading.Thread(target=pracownik, daemon=True).start()
    threading.Thread(target=automat, daemon=True).start()
    threading.Thread(target=pracownik_por, daemon=True).start()
    print(f'Przegladarka: http://<adres-serwera>:{ARGS.port}  katalog: {ARGS.katalog}')
    ThreadingHTTPServer(('0.0.0.0', ARGS.port), H).serve_forever()


if __name__ == '__main__':
    main()

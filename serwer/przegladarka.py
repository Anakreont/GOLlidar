#!/usr/bin/env python3
"""Przegladarka pomiarow LiDAR na serwerze (Dell) - tylko siec lokalna.

Pokazuje nagrania z LIDAR_DIR pogrupowane budynek -> kondygnacja -> sesja: mape z warstwami (sciany, chmura punktow,
trasa, zdjecia, przekroje, punkty, notatki), zdjecia, znaczniki, inwentaryzacje (oprawy, grzejniki, okna) i pliki (DXF).
Mapy licza sie same w tle po zakonczeniu sesji (mapa/przetworz.py), mozna tez wymusic przyciskiem.

Uruchomienie:  python3 przegladarka.py [--port 8150] [--katalog ~/skany-lidar]
Odpowiada tylko klientom z adresow prywatnych (siec domowa, localhost).
"""
import argparse, ipaddress, json, os, queue, re, shutil, subprocess, sys, threading, time, types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

TU = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TU)
sys.modules.setdefault('serial', types.SimpleNamespace(Serial=None, SerialException=Exception))
sys.path.insert(0, REPO)
import skan_web as SW          # tylko funkcje czytajace CSV inwentaryzacji

ARGS = None
SESJA_RE = re.compile(r'^[A-Za-z0-9_+\-]{1,100}$')
PLIK_RE = re.compile(r'^(_wynik/)?[A-Za-z0-9_\-]{1,60}\.(jpg|csv|txt|bin|dxf|png|json)$')
KOLEJKA = queue.Queue()
W_TOKU = {}                    # sesja -> 'czeka' | 'liczy'
BLOK = threading.Lock()


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
    if os.path.exists(mj) and os.path.getmtime(mj) >= os.path.getmtime(lid):
        return 'gotowe'
    return 'do policzenia'


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
        with open(log, 'w') as f:
            r = subprocess.run([sys.executable, os.path.join(REPO, 'mapa', 'przetworz.py'), k], stdout=f, stderr=subprocess.STDOUT)
        blad = os.path.join(k, '_wynik', 'blad.txt')
        if r.returncode != 0:
            with open(log) as f, open(blad, 'w') as g:
                g.write(f.read()[-3000:])
        elif os.path.exists(blad):
            os.remove(blad)
        with BLOK:
            W_TOKU.pop(n, None)


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


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _lan(self):
        try:
            ip = ipaddress.ip_address(self.client_address[0])
            if ip.is_private or ip.is_loopback:
                return True
        except ValueError:
            pass
        self.send_error(403, 'Tylko siec lokalna')
        return False

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
            return self._wyslij(200, 'text/html; charset=utf-8', STRONA.encode())
        if u.path == '/api/sesje':
            return self._json(sesje())
        k = katalog(q.get('n'))
        if u.path.startswith('/api/') and not k:
            return self._json({'blad': 'nie ma takiej sesji'}, 404)
        if u.path == '/api/sesja':
            pliki = sorted(p for p in os.listdir(k) if os.path.isfile(os.path.join(k, p)))
            wyn = sorted('_wynik/' + p for p in os.listdir(os.path.join(k, '_wynik'))
                         if p in ('rzut.dxf', 'podglad.png', 'log.txt', 'blad.txt')) if os.path.isdir(os.path.join(k, '_wynik')) else []
            return self._json({'info': info(k), 'stan': stan_wyniku(q['n'], k), 'pliki': pliki, 'wyniki': wyn,
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
            return self._wyslij(200, 'application/json; charset=utf-8', b'{"mapa":' + dane + b',"nazwy":' + nz.encode() + b'}')
        if u.path == '/plik':
            f = q.get('f', '')
            if not k or not PLIK_RE.match(f):
                return self.send_error(404)
            p = os.path.join(k, f)
            if not os.path.isfile(p):
                return self.send_error(404)
            typ = {'jpg': 'image/jpeg', 'png': 'image/png', 'json': 'application/json', 'txt': 'text/plain; charset=utf-8',
                   'csv': 'text/csv; charset=utf-8'}.get(f.rsplit('.', 1)[1], 'application/octet-stream')
            with open(p, 'rb') as fh:
                dane = fh.read()
            nag = [('Content-Disposition', f'attachment; filename="{q["n"]}_{os.path.basename(f)}"')] \
                if f.endswith(('.dxf', '.bin', '.csv')) else []
            return self._wyslij(200, typ, dane, nag)
        self.send_error(404)

    def do_POST(self):
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
        if u.path == '/api/nazwy':
            dl = int(self.headers.get('Content-Length') or 0)
            try:
                nz = json.loads(self.rfile.read(min(dl, 100000)) or b'{}')
                nz = {str(int(a)): {'nazwa': str(b.get('nazwa', ''))[:80], 'ukryj': bool(b.get('ukryj'))} for a, b in nz.items()}
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


STRONA = r'''<!doctype html><html lang="pl"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pomiary LiDAR</title><style>
*{box-sizing:border-box}body{margin:0;font:14px system-ui,sans-serif;background:#f4f5f7;color:#222;display:flex;height:100vh}
#lewy{width:300px;overflow:auto;background:#fff;border-right:1px solid #ddd;padding:10px;flex-shrink:0}
#prawy{flex:1;display:flex;flex-direction:column;min-width:0}
h1{font-size:17px;margin:4px 0 10px}h2{font-size:15px;margin:12px 0 4px}h3{font-size:13px;margin:8px 0 2px;color:#555}
.ses{padding:6px 8px;border-radius:6px;cursor:pointer;margin:2px 0;font-size:13px}.ses:hover{background:#eef}.ses.akt{background:#dde6ff}
.st{font-size:11px;padding:1px 6px;border-radius:8px;background:#eee;margin-left:4px}.st.gotowe{background:#c8f0d0}.st.liczy,.st.czeka{background:#ffe9a8}.st.błąd{background:#f8c4c4}
#tb{display:flex;gap:4px;padding:8px;background:#fff;border-bottom:1px solid #ddd;flex-wrap:wrap;align-items:center}
#tb button,.btn{border:1px solid #ccc;background:#fff;padding:5px 12px;border-radius:6px;cursor:pointer}#tb button.akt{background:#2b59c3;color:#fff;border-color:#2b59c3}
#tresc{flex:1;overflow:auto;position:relative}section{display:none;padding:12px}section.akt{display:block}
#smapa{padding:0;height:100%}#smapa.akt{display:flex}#cw{flex:1;position:relative}canvas{width:100%;height:100%;display:block;background:#fff;touch-action:none}
#panel{width:320px;overflow:auto;border-left:1px solid #ddd;background:#fff;padding:10px}
#panel img{width:100%;border-radius:6px}label{display:block;margin:3px 0;cursor:pointer}
table{border-collapse:collapse;background:#fff;font-size:13px}td,th{border:1px solid #ddd;padding:4px 8px;text-align:left}th{background:#f0f2f5}
.gal{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px}.gal div{background:#fff;padding:6px;border-radius:8px}.gal img{width:100%;border-radius:6px;cursor:pointer}
.kol{display:inline-block;width:10px;height:10px;border-radius:5px;margin-right:4px}.mala{color:#666;font-size:12px}
#info{font-size:12px;color:#555;margin-left:auto}input[type=text]{padding:4px;width:100%}
@media(max-width:800px){body{flex-direction:column;height:auto}#lewy{width:100%;max-height:35vh}#smapa.akt{flex-direction:column}#cw{height:60vh}#panel{width:100%}}
</style></head><body>
<div id="lewy"><h1>Pomiary LiDAR</h1><div id="lista">wczytuję…</div>
<h2>Porządki</h2><button class="btn" onclick="usunWszystkieMapy()">Usuń wszystkie mapy</button>
<p class="mala">Mapy policzą się od nowa same (np. po poprawce programu). Nagrania zostają.</p>
<button class="btn" onclick="oproznijKosz()">Opróżnij kosz</button>
<p class="mala">Usunięte sesje trafiają do kosza (katalog _kosz). Usuń je też na skanerze – inaczej kopia z maliny wyśle je ponownie.</p></div>
<div id="prawy"><div id="tb"><button data-t="mapa" class="akt">Mapa</button><button data-t="foto">Zdjęcia</button><button data-t="zn">Znaczniki i notatki</button>
<button data-t="inw">Inwentaryzacja</button><button data-t="pliki">Pliki</button><span id="info">wybierz sesję z listy</span></div>
<div id="tresc">
<section id="smapa" class="akt"><div id="cw"><canvas id="c"></canvas></div><div id="panel">
<h2>Kondygnacja</h2><select id="seg" style="width:100%"></select>
<div id="nazwa" style="margin-top:6px"></div>
<h2>Warstwy</h2><div id="warstwy"></div>
<h2>Wybrany punkt</h2><div id="wyb" class="mala">kliknij znacznik na mapie</div>
<p class="mala">Przeciąganie – przesuwanie, kółko/szczypanie – powiększenie, dwuklik – cała kondygnacja.</p></div></section>
<section id="sfoto"><div class="gal" id="gal"></div></section>
<section id="szn"><table id="tzn"></table></section>
<section id="sinw"><div id="inw"></div></section>
<section id="spliki"><div id="pliki"></div></section>
</div></div>
<script>
const $=id=>document.getElementById(id),esc=s=>String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
let S=null,M=null,NZ={},seg=0,wid={x:0,y:0,s:50},wyb=null;const W={sciany:1,chmura:1,trasa:1,foto:1,przekroj:1,punkt:1,notatka:1,oprawy:1,grzejnik:1,okno:1};
const KOL={foto:'#2b59c3',przekroj:'#b03ab0',punkt:'#e08a00',notatka:'#1b9e5a',oprawy:'#0aa',grzejnik:'#d33',okno:'#a80'};
const NAZ={sciany:'Ściany',chmura:'Chmura punktów',trasa:'Trasa',foto:'Zdjęcia',przekroj:'Przekroje',punkt:'Punkty z podglądu',notatka:'Notatki',oprawy:'Oprawy',grzejnik:'Grzejniki',okno:'Okna'};
document.querySelectorAll('#tb button').forEach(b=>b.onclick=()=>{document.querySelectorAll('#tb button,section').forEach(x=>x.classList.remove('akt'));b.classList.add('akt');$('s'+b.dataset.t).classList.add('akt');if(b.dataset.t=='mapa')rys()});
function lista(){fetch('api/sesje').then(r=>r.json()).then(L=>{const B={};L.forEach(s=>((B[s.budynek]??={})[s.kondygnacja]??=[]).push(s));
 $('lista').innerHTML=Object.keys(B).sort().map(b=>'<h2>'+esc(b)+'</h2>'+Object.keys(B[b]).sort().map(k=>'<h3>'+esc(k)+'</h3>'+B[b][k].map(s=>
 '<div class="ses'+(S&&S.n==s.n?' akt':'')+'" onclick="otworz(\''+s.n+'\')">'+esc(s.start)+' · '+(s.czas_s?Math.round(s.czas_s/60)+' min':'')+' · '+s.foto+' zdj.<span class="st '+esc(s.stan)+'">'+esc(s.stan)+'</span></div>').join('')).join('')).join('')||'brak nagrań'})}
function otworz(n){fetch('api/sesja?n='+n).then(r=>r.json()).then(d=>{S=d;S.n=n;lista();$('info').textContent=(d.info.budynek||'')+' · '+(d.info.kondygnacja||'')+' · '+n+' · '+d.stan;
 galeria();tabZn();inwent();pliki();M=null;
 fetch('api/mapa?n='+n).then(r=>r.ok?r.json():null).then(j=>{if(j){M=j.mapa;NZ=j.nazwy||{};seg=0;segmenty();dopasuj()}else{M=null;$('seg').innerHTML='';$('nazwa').innerHTML='<p class="mala">Mapa: '+esc(S.stan)+'</p><button class="btn" onclick="przelicz()">Policz teraz</button>'}rys()})})}
function przelicz(){fetch('api/przelicz?n='+S.n,{method:'POST'}).then(()=>{alert('Zlecone – mapa liczy się około minuty na każde 8 minut nagrania.');lista()})}
function segmenty(){$('seg').innerHTML=M.kondygnacje.map((k,i)=>'<option value="'+i+'">'+esc((NZ[k.nr]||{}).nazwa||('Odcinek '+k.nr))+' ('+k.od+'–'+k.do+' s, '+(k.poziom>=0?'+':'')+k.poziom+' m)'+((NZ[k.nr]||{}).ukryj?' – ukryty':'')+'</option>').join('');
 $('seg').value=seg;$('seg').onchange=()=>{seg=+$('seg').value;wyb=null;nazwaForm();dopasuj();rys()};nazwaForm();
 $('warstwy').innerHTML=Object.keys(W).map(w=>'<label><input type="checkbox" '+(W[w]?'checked':'')+' onchange="W[\''+w+'\']=this.checked;rys()"> '+(KOL[w]?'<span class="kol" style="background:'+KOL[w]+'"></span>':'')+NAZ[w]+'</label>').join('')}
function nazwaForm(){const k=M.kondygnacje[seg],z=NZ[k.nr]||{};$('nazwa').innerHTML='<input type="text" id="nz" placeholder="nazwa, np. PARTER" value="'+esc(z.nazwa||'')+'"><label><input type="checkbox" id="uk" '+(z.ukryj?'checked':'')+'> pomiń w DXF (np. powtórne przejście)</label><button class="btn" onclick="zapiszNazwe()">Zapisz</button> <button class="btn" onclick="przelicz()">Przelicz mapę</button><p class="mala">Policzono '+esc(M.policzono)+', '+M.skanow+' skanów.</p>'}
function zapiszNazwe(){const k=M.kondygnacje[seg];NZ[k.nr]={nazwa:$('nz').value,ukryj:$('uk').checked};fetch('api/nazwy?n='+S.n,{method:'POST',body:JSON.stringify(NZ)}).then(r=>r.json()).then(()=>{segmenty();pliki()})}
const c=$('c'),g=c.getContext('2d');
function dopasuj(){if(!M||!M.kondygnacje[seg])return;const k=M.kondygnacje[seg],r=c.getBoundingClientRect();wid.s=Math.min(r.width/(k.szer+1),r.height/(k.wys+1));wid.x=r.width/2-k.szer/2*wid.s;wid.y=r.height/2+k.wys/2*wid.s}
const P=(x,y)=>[wid.x+x*wid.s,wid.y-y*wid.s];
function rys(){const r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);
 if(!M||!M.kondygnacje[seg]){g.fillStyle='#888';g.font='15px system-ui';g.fillText(S?'Mapa: '+S.stan:'Wybierz sesję',20,30);return}
 const k=M.kondygnacje[seg];
 const kr=wid.s>=1?Math.max(.2,Math.min(1,10/wid.s)):1;const sk=wid.s*0.5>=1?1:1;
 g.strokeStyle='#eee';g.lineWidth=1;const m=wid.s>=60?.5:1;for(let x=0;x<=k.szer+1;x+=m){const[a]=P(x,0);g.beginPath();g.moveTo(a,0);g.lineTo(a,r.height);g.stroke()}for(let y=0;y<=k.wys+1;y+=m){const[,b]=P(0,y);g.beginPath();g.moveTo(0,b);g.lineTo(r.width,b);g.stroke()}
 if(W.chmura){g.fillStyle='rgba(120,120,120,.45)';const ch=k.chmura_cm;for(let i=0;i<ch.length;i+=2){const[a,b]=P(ch[i]/100,ch[i+1]/100);g.fillRect(a,b,1.3,1.3)}}
 if(W.trasa&&k.trasa.length){g.strokeStyle='rgba(220,40,40,.6)';g.lineWidth=1;g.beginPath();k.trasa.forEach((p,i)=>{const[a,b]=P(p[0],p[1]);i?g.lineTo(a,b):g.moveTo(a,b)});g.stroke()}
 if(W.sciany){g.strokeStyle='#111';g.lineWidth=2;k.sciany.forEach(pl=>{g.beginPath();pl.forEach((p,i)=>{const[a,b]=P(p[0],p[1]);i?g.lineTo(a,b):g.moveTo(a,b)});g.stroke()})}
 k.znaczniki.forEach(z=>{if(z.rodzaj=='system'||!W[z.rodzaj])return;const[a,b]=P(z.x,z.y);g.fillStyle=KOL[z.rodzaj]||'#1b9e5a';g.beginPath();g.arc(a,b,z===wyb?8:5,0,7);g.fill();if(wid.s>40){g.font='11px system-ui';g.fillText((z.plik||z.opis).slice(0,40),a+8,b+4)}});
 const L=(wid.s>=100?.5:wid.s>=40?1:wid.s>=15?2:5);g.fillStyle='#333';g.fillRect(14,r.height-22,L*wid.s,4);g.font='12px system-ui';g.fillText(L+' m',14,r.height-28)}
let drag=null,pinch=null;
c.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,ox:wid.x,oy:wid.y,mv:0};c.setPointerCapture(e.pointerId)});
c.addEventListener('pointermove',e=>{if(!drag||pinch)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.mv=Math.max(drag.mv,Math.abs(dx)+Math.abs(dy));wid.x=drag.ox+dx;wid.y=drag.oy+dy;rys()});
c.addEventListener('pointerup',e=>{if(drag&&drag.mv<5)klik(e);drag=null});
c.addEventListener('wheel',e=>{e.preventDefault();const r=c.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,f=e.deltaY<0?1.2:1/1.2;wid.x=mx-(mx-wid.x)*f;wid.y=my-(my-wid.y)*f;wid.s*=f;rys()},{passive:false});
c.addEventListener('touchstart',e=>{if(e.touches.length==2){const[a,b]=e.touches;pinch={d:Math.hypot(a.clientX-b.clientX,a.clientY-b.clientY),s:wid.s,x:wid.x,y:wid.y,cx:(a.clientX+b.clientX)/2,cy:(a.clientY+b.clientY)/2}}});
c.addEventListener('touchmove',e=>{if(pinch&&e.touches.length==2){e.preventDefault();const[a,b]=e.touches,r=c.getBoundingClientRect(),f=Math.hypot(a.clientX-b.clientX,a.clientY-b.clientY)/pinch.d,mx=pinch.cx-r.left,my=pinch.cy-r.top;wid.s=pinch.s*f;wid.x=mx-(mx-pinch.x)*f;wid.y=my-(my-pinch.y)*f;rys()}},{passive:false});
c.addEventListener('touchend',e=>{if(e.touches.length<2)pinch=null});
c.addEventListener('dblclick',()=>{dopasuj();rys()});addEventListener('resize',rys);
function klik(e){if(!M)return;const r=c.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;let best=null,bd=14;
 M.kondygnacje[seg].znaczniki.forEach(z=>{if(z.rodzaj=='system'||!W[z.rodzaj])return;const[a,b]=P(z.x,z.y),d=Math.hypot(a-mx,b-my);if(d<bd){bd=d;best=z}});wyb=best;pokaz(best);rys()}
function pokaz(z){if(!z){$('wyb').innerHTML='<span class="mala">kliknij znacznik na mapie</span>';return}
 $('wyb').innerHTML='<b>'+esc(z.opis)+'</b><br><span class="mala">'+z.t+' s od startu'+(z.wys!=null?', wysokość '+z.wys.toFixed(2)+' m':'')+'</span>'+(z.plik?'<a href="plik?n='+S.n+'&f='+z.plik+'" target="_blank"><img src="plik?n='+S.n+'&f='+z.plik+'"></a>':'')}
function galeria(){const F=S.pliki.filter(p=>p.startsWith('foto_')),zf={};S.znaczniki.forEach(z=>{if(z.opis.startsWith('FOTO '))zf[z.opis.slice(5)]=z});
 $('gal').innerHTML=F.map(f=>'<div><a href="plik?n='+S.n+'&f='+f+'" target="_blank"><img loading="lazy" src="plik?n='+S.n+'&f='+f+'"></a><div class="mala">'+esc(f)+(zf[f]?' · '+zf[f].godz+' · '+zf[f].t+' s':'')+'</div></div>').join('')||'brak zdjęć'}
function tabZn(){const Z=S.znaczniki.filter(z=>!/^(AUTO_|PRZEKROJ_(START|KONIEC|OK|OD_NOWA))/.test(z.opis));
 $('tzn').innerHTML='<tr><th>godz.</th><th>s</th><th>opis</th><th>wysokość wzgl. m</th></tr>'+Z.map(z=>'<tr><td>'+z.godz+'</td><td>'+(z.t??'')+'</td><td>'+esc(z.opis)+'</td><td>'+esc((+z.wys||0).toFixed(2))+'</td></tr>').join('')}
function tab(L,kol,naz){if(!L.length)return'<p class="mala">brak wpisów</p>';return'<table><tr>'+naz.map(n=>'<th>'+n+'</th>').join('')+'</tr>'+L.map(r=>'<tr>'+kol.map(k=>'<td>'+esc(r[k])+'</td>').join('')+'</tr>').join('')+'</table>'}
function inwent(){$('inw').innerHTML='<h2>Oświetlenie</h2>'+tab(S.oprawy,['nr','typ','opraw','punktow','moc','moc_razem','uwagi','godz'],['pomieszczenie','rodzaj','opraw','pkt/oprawę','W/pkt','W razem','uwagi','godz.'])+
 '<h2>Grzejniki</h2>'+tab(S.grzejniki,['nr','rodzaj','sztuk','zeberek','szer','dl','uwagi','godz'],['pomieszczenie','rodzaj','szt.','żeberek','szer. cm','dł. cm','uwagi','godz.'])+
 '<h2>Okna i drzwi zewnętrzne</h2>'+tab(S.okna,['nr','typ','rodzaj','szer','wys_cm','rama','szyby','sztuk','uwagi'],['pomieszczenie','typ','rodzaj','szer. cm','wys. cm','rama','szyby','szt.','uwagi'])}
function post(u,ok){fetch(u,{method:'POST'}).then(r=>r.json()).then(j=>{if(j.blad)alert(j.blad);else ok&&ok(j)})}
function usunWszystkieMapy(){if(confirm('Usunąć mapy (DXF, PNG, wyniki) wszystkich sesji? Nagrania zostają, mapy policzą się od nowa.'))post('api/usun_wszystkie_mapy',j=>{alert('Usunięto mapy '+j.ile+' sesji');M=null;lista();if(S)otworz(S.n)})}
function oproznijKosz(){if(confirm('Opróżnić kosz? Sesji z kosza nie da się już odzyskać.'))post('api/oproznij_kosz',()=>alert('Kosz opróżniony'))}
function usunMape(){if(confirm('Usunąć mapę tej sesji? Policzy się od nowa.'))post('api/usun_mape?n='+S.n,()=>otworz(S.n))}
function usunSesje(){if(confirm('Przenieść całą sesję '+S.n+' (nagrania, zdjęcia, wpisy, mapy) do kosza?'))post('api/usun_sesje?n='+S.n,()=>{S=null;M=null;$('info').textContent='sesja usunięta';rys();lista()})}
function pliki(){fetch('api/sesja?n='+S.n).then(r=>r.json()).then(d=>{$('pliki').innerHTML='<h2>Usuwanie</h2><button class="btn" onclick="usunMape()">Usuń mapę tej sesji</button> <button class="btn" style="color:#b00" onclick="usunSesje()">Usuń całą sesję</button><h2>Wyniki</h2>'+(d.wyniki.map(f=>'<p><a href="plik?n='+S.n+'&f='+f+'" target="_blank">'+esc(f.replace('_wynik/',''))+'</a></p>').join('')||'<p class="mala">brak – mapa: '+esc(d.stan)+'</p>')+
 '<h2>Dane z nagrania</h2>'+d.pliki.filter(f=>!f.startsWith('foto_')).map(f=>'<p><a href="plik?n='+S.n+'&f='+f+'">'+esc(f)+'</a></p>').join('')+'<h2>Informacje</h2><pre>'+esc(Object.entries(d.info).map(([a,b])=>a+'='+b).join('\n'))+'</pre>'})}
lista();setInterval(lista,30000);
</script></body></html>'''


def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8150)
    ap.add_argument('--katalog', default=os.environ.get('LIDAR_DIR', os.path.expanduser('~/skany-lidar')))
    ARGS = ap.parse_args()
    threading.Thread(target=pracownik, daemon=True).start()
    threading.Thread(target=automat, daemon=True).start()
    print(f'Przegladarka: http://<adres-serwera>:{ARGS.port}  katalog: {ARGS.katalog}')
    ThreadingHTTPServer(('0.0.0.0', ARGS.port), H).serve_forever()


if __name__ == '__main__':
    main()

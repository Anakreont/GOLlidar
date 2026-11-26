"""Liczy mape jednej sesji skanera gotowymi programami SLAM i zestawia wyniki z naszym potokiem.

    porownaj <katalog_sesji> [--metody carto_imu,carto,slam_toolbox] [--kondygnacja N] [--szybkosc 2]

(`porownaj` = skrypt porownanie/porownaj.sh, ktory wlacza srodowisko ROS 2 z ~/ros2_slam.)
Wyniki w <sesja>/_porownanie/:
    k<N>_<metoda>.pgm/.yaml/.png  - mapa zajetosci (2,5 cm), czarne = sciany
    k<N>_<metoda>.dxf             - punkty scian (do nalozenia na rzut w CAD)
    k<N>_zestawienie.png          - wszystkie metody obok siebie (+ nasz podglad.png)
    wynik.json                    - czasy, liczba skanow, ewentualne bledy
oraz kopia zestawienia w <sesja>/_wynik/porownanie_k<N>.png (widoczna w przegladarce, zakladka Pliki).
"""
import argparse, json, os, shutil, subprocess, sys, time
import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
METODY = {'slam_toolbox': 'slam_toolbox (Karto, kurs z IMU)', 'carto': 'Cartographer (sam LiDAR)',
          'carto_imu': 'Cartographer + IMU'}
ROZDZ = 0.025


def env_ros():
    e = dict(os.environ)
    e.setdefault('ROS_DOMAIN_ID', '77'); e['ROS_LOCALHOST_ONLY'] = '1'
    e['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
    return e


def zapisz_mape(siatka, rozdz, origin, stem):
    """siatka: int8 (-1 nieznane, 0..100), wiersz 0 = dol mapy (jak OccupancyGrid). Zapis jak map_saver + PNG."""
    import yaml
    img = np.full(siatka.shape, 205, np.uint8)
    img[(siatka >= 0) & (siatka < 25)] = 254
    img[siatka > 65] = 0
    img = img[::-1]
    with open(stem + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0])); f.write(img.tobytes())
    yaml.safe_dump({'image': os.path.basename(stem) + '.pgm', 'mode': 'trinary', 'resolution': float(rozdz),
                    'origin': [float(origin[0]), float(origin[1]), 0.0], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.25}, open(stem + '.yaml', 'w'))


def wczytaj_mape(stem):
    import yaml
    y = yaml.safe_load(open(stem + '.yaml'))
    with open(os.path.join(os.path.dirname(stem), y['image']), 'rb') as f:
        dane = f.read()
    # naglowek PGM (P5) - moze miec komentarze
    pola, poz = [], 0
    while len(pola) < 4:
        while dane[poz:poz + 1].isspace(): poz += 1
        if dane[poz:poz + 1] == b'#':
            poz = dane.index(b'\n', poz) + 1; continue
        k = poz
        while not dane[poz:poz + 1].isspace(): poz += 1
        pola.append(dane[k:poz])
    poz += 1
    w, h = int(pola[1]), int(pola[2])
    img = np.frombuffer(dane[poz:poz + w * h], np.uint8).reshape(h, w)
    return img, float(y['resolution']), y['origin']


def punkty_scian(img, rozdz, origin):
    """Srodki komorek zajetych (czarnych) w metrach, uklad mapy."""
    h = img.shape[0]
    r, c = np.where(img < 50)
    x = origin[0] + (c + 0.5) * rozdz
    y = origin[1] + (h - 1 - r + 0.5) * rozdz
    return np.stack([x, y], 1)


def zapisz_dxf(P, plik):
    import ezdxf
    d = ezdxf.new('R2010'); d.units = ezdxf.units.M
    d.layers.add('SCIANY_PUNKTY', color=7)
    msp = d.modelspace()
    for x, y in P:
        msp.add_point((float(x), float(y)), dxfattribs={'layer': 'SCIANY_PUNKTY'})
    d.saveas(plik)


def png(img, rozdz, plik, tytul=''):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    r, c = np.where(img < 205)
    if len(r) == 0: return
    m = 20
    img = img[max(0, r.min() - m): r.max() + m, max(0, c.min() - m): c.max() + m]
    h, w = img.shape
    s = max(w, h) * rozdz
    fig = plt.figure(figsize=(min(20, max(6, w / 100)), min(20, max(6, h / 100))))
    plt.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    plt.title(f'{tytul}  ({w * rozdz:.1f} x {h * rozdz:.1f} m)'); plt.axis('off')
    plt.tight_layout(); plt.savefig(plik, dpi=100); plt.close(fig)


def czekaj_az_bezczynny(pid, prog=30.0, okres=4.0, max_s=3600):
    """Czeka, az proces przestanie liczyc (liczy na jednym rdzeniu ~100 %, bezczynny ma tylko krotkie skoki)."""
    import psutil
    p = psutil.Process(pid); t0 = time.time(); spokoj = 0
    p.cpu_percent(None)
    while time.time() - t0 < max_s:
        time.sleep(okres)
        if not p.is_running(): return
        if p.cpu_percent(None) < prog: spokoj += 1
        else: spokoj = 0
        if spokoj >= 2: return


def mapa_ros(nazwa_srv, stem, env):
    """Pobiera mape z serwisu nav_msgs/GetMap (slam_toolbox/dynamic_map) i zapisuje."""
    kod = f'''
import rclpy, numpy as np, sys
from nav_msgs.srv import GetMap
rclpy.init(); n = rclpy.create_node("pobierz_mape")
c = n.create_client(GetMap, "{nazwa_srv}")
if not c.wait_for_service(timeout_sec=30): print("brak serwisu"); sys.exit(2)
f = c.call_async(GetMap.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=300)
m = f.result().map
np.savez("{stem}_tmp.npz", d=np.array(m.data, np.int8).reshape(m.info.height, m.info.width),
         r=m.info.resolution, o=[m.info.origin.position.x, m.info.origin.position.y])
'''
    subprocess.run([sys.executable, '-c', kod], env=env, check=True, timeout=400)
    z = np.load(stem + '_tmp.npz'); os.remove(stem + '_tmp.npz')
    zapisz_mape(z['d'], float(z['r']), z['o'], stem)


def slam_toolbox(bag, stem, szybkosc, log):
    env = env_ros()
    st = subprocess.Popen(['ros2', 'run', 'slam_toolbox', 'sync_slam_toolbox_node', '--ros-args',
                           '--params-file', os.path.join(TU, 'slam_toolbox.yaml'), '-r', '__node:=slam_toolbox',
                           '-p', 'use_sim_time:=true'], env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(4)
        subprocess.run(['ros2', 'bag', 'play', bag, '--clock', '100', '-r', str(szybkosc),
                        '--topics', '/scan', '/tf'], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        # sync: kolejka skanow liczy sie dalej po koncu odtwarzania - czekamy, az proces przestanie liczyc
        import psutil
        dzieci = psutil.Process(st.pid).children(recursive=True)
        czekaj_az_bezczynny(dzieci[0].pid if dzieci else st.pid)
        mapa_ros('/slam_toolbox/dynamic_map', stem, env)
    finally:
        st.terminate()
        try: st.wait(10)
        except subprocess.TimeoutExpired: st.kill()


def splaszcz_lua(nazwa, katalogi, byly=None):
    """Wkleja pliki z 'include' (Cartographer z robostack nie rozwiazuje include - czyta katalog zamiast pliku)."""
    import re
    byly = set() if byly is None else byly
    for k in katalogi:
        p = os.path.join(k, nazwa)
        if os.path.isfile(p): break
    else:
        raise FileNotFoundError(nazwa)
    out = []
    for l in open(p, encoding='utf-8'):
        m = re.match(r'\s*include\s+"([^"]+)"', l)
        if m:
            if m.group(1) not in byly:
                byly.add(m.group(1)); out.append(splaszcz_lua(m.group(1), katalogi, byly))
        elif re.match(r'\s*return\s+\w+\s*$', l) and nazwa != byly_glowny[0]:
            continue                        # 'return X' z plikow dolaczanych usuwamy, zostaje tylko w glownym
        else:
            out.append(l)
    return ''.join(out)

byly_glowny = ['']


def cartographer(bag, stem, lua, log):
    env = env_ros()
    pb = stem + '.pbstream'
    katalogi = [TU, os.path.join(sys.prefix, 'share', 'cartographer', 'configuration_files')]
    byly_glowny[0] = lua
    plaski = os.path.join(os.path.dirname(stem), '_lua'); os.makedirs(plaski, exist_ok=True)
    open(os.path.join(plaski, lua), 'w', encoding='utf-8').write(splaszcz_lua(lua, katalogi))
    subprocess.run(['ros2', 'run', 'cartographer_ros', 'cartographer_offline_node',
                    '-configuration_directory', plaski, '-configuration_basenames', lua,
                    '-bag_filenames', bag, '-save_state_filename', pb, '--ros-args', '-p', 'use_sim_time:=true'],
                   env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=4 * 3600)
    subprocess.run(['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                    '-pbstream_filename', pb, '-map_filestem', stem, '-resolution', str(ROZDZ)],
                   env=env, stdout=log, stderr=subprocess.STDOUT, check=True)


def zestawienie(panele, plik, tytul):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    n = len(panele)
    if not n: return
    kol = 2 if n > 1 else 1; wier = (n + 1) // 2
    fig, ax = plt.subplots(wier, kol, figsize=(10 * kol, 9 * wier), squeeze=False)
    for a in ax.ravel(): a.axis('off')
    for a, (nazwa, obraz) in zip(ax.ravel(), panele):
        a.imshow(obraz, cmap='gray' if obraz.ndim == 2 else None, interpolation='nearest'); a.set_title(nazwa, fontsize=14)
    fig.suptitle(tytul, fontsize=16); plt.tight_layout(); plt.savefig(plik, dpi=80); plt.close(fig)


def kadr(img, m=20):
    r, c = np.where(img < 205)
    if not len(r): return img
    return img[max(0, r.min() - m): r.max() + m, max(0, c.min() - m): c.max() + m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja')
    ap.add_argument('--metody', default='carto_imu,carto', help='carto_imu, carto, slam_toolbox (eksperymentalnie)')
    ap.add_argument('--kondygnacja', type=int)
    ap.add_argument('--szybkosc', type=float, default=2.0, help='predkosc odtwarzania dla slam_toolbox')
    ap.add_argument('--bez-imu', action='store_true')
    a = ap.parse_args()
    kat = os.path.abspath(a.sesja.rstrip('/'))
    wy = os.path.join(kat, '_porownanie'); os.makedirs(wy, exist_ok=True)
    bagi = os.path.join(wy, 'bagi')
    cmd = [sys.executable, os.path.join(TU, 'eksport_bag.py'), kat, bagi]
    if a.kondygnacja is not None: cmd += ['--kondygnacja', str(a.kondygnacja)]
    if a.bez_imu: cmd += ['--bez-imu']
    print('== eksport do ROS 2:', ' '.join(cmd[1:]), flush=True)
    subprocess.run(cmd, check=True)
    opis = json.load(open(os.path.join(bagi, 'bagi.json')))
    wynik = {'sesja': os.path.basename(kat), 'kondygnacje': []}
    nasz = os.path.join(kat, '_wynik', 'podglad.png')
    for k in opis['kondygnacje']:
        nr = k['nr']; panele = []; wk = dict(k, metody={})
        if os.path.exists(nasz):
            import matplotlib.image as mpimg
            panele.append(('Nasz potok (podglad.png, cala sesja)', mpimg.imread(nasz)))
        for m in [x.strip() for x in a.metody.split(',') if x.strip()]:
            stem = os.path.join(wy, f'k{nr}_{m}')
            print(f'== kondygnacja {nr}: {METODY.get(m, m)} ...', flush=True)
            t = time.time()
            with open(stem + '.log', 'w') as log:
                try:
                    if m == 'slam_toolbox':
                        slam_toolbox(k['bag'], stem, a.szybkosc, log)
                    elif m == 'carto':
                        cartographer(k['bag'], stem, 'carto.lua', log)
                    elif m == 'carto_imu':
                        if not opis.get('imu', True):
                            raise RuntimeError('sesja bez IMU')
                        cartographer(k['bag'], stem, 'carto_imu.lua', log)
                    else:
                        raise RuntimeError('nieznana metoda')
                    img, rz, org = wczytaj_mape(stem)
                    P = punkty_scian(img, rz, org)
                    zapisz_dxf(P, stem + '.dxf')
                    png(img, rz, stem + '.png', f'{METODY.get(m, m)} - kondygnacja {nr}')
                    panele.append((METODY.get(m, m), kadr(img)))
                    wk['metody'][m] = {'czas_s': round(time.time() - t, 1), 'punktow_scian': int(len(P))}
                    print(f'   gotowe w {time.time() - t:.0f} s', flush=True)
                except Exception as e:
                    wk['metody'][m] = {'blad': str(e), 'log': os.path.basename(stem) + '.log'}
                    print(f'   BLAD: {e} (szczegoly: {stem}.log)', flush=True)
        zp = os.path.join(wy, f'k{nr}_zestawienie.png')
        zestawienie(panele, zp, f"{os.path.basename(kat)} - kondygnacja {nr} ({k['czas_s']:.0f} s, {k['skanow']} skanow)")
        if os.path.exists(zp):
            os.makedirs(os.path.join(kat, '_wynik'), exist_ok=True)
            shutil.copy(zp, os.path.join(kat, '_wynik', f'porownanie_k{nr}.png'))
        wynik['kondygnacje'].append(wk)
    json.dump(wynik, open(os.path.join(wy, 'wynik.json'), 'w'), indent=1, ensure_ascii=False)
    print('== wyniki:', wy)


if __name__ == '__main__':
    main()

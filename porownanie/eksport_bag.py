"""Zamienia sesje skanera (lidar.bin, imu.bin, baro.bin, znaczniki.txt) na nagrania ROS 2 (rosbag2),
cala sesja jako jedna kondygnacja (opcja --dziel-baro: podzial wg barometru), do liczenia gotowymi programami SLAM (slam_toolbox, Cartographer).

    python3 eksport_bag.py <katalog_sesji> <katalog_wyjsciowy> [--kondygnacja N] [--bez-imu]

Tematy w nagraniu:
    /scan  sensor_msgs/LaserScan  - jeden pelny obrot D500, po korekcji ruchu (deskew z kursu IMU),
                                    bez operatora, slabych odbic, skanow z przechylem > 12 st. i z PAUZY
    /tf    odom -> base_link       - sam kurs z BNO085 (bez przesuniecia), 50 Hz
    /imu   sensor_msgs/Imu         - predkosc katowa z kursu BNO085 + sztuczna grawitacja (dla Cartographera 2D)
Wszystkie ramki: base_link (LiDAR = srodek ukladu).
"""
import argparse, os, sys, json
import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TU, '..', 'mapa')); sys.path.insert(0, os.path.join(TU, '..'))
import slam, sprawdz as S

ZNAK = -1          # jak w przetworz.py: LD19 zgodnie z zegarem, kurs = ZNAK * yaw BNO
N_WIAZEK = 720     # 0,5 st.
BAZA = 1000.0      # s - przesuniecie czasu nagrania (ROS nie lubi czasu 0)
RMIN, RMAX = 0.3, 9.0


def skany(kat, bez_imu=False, dziel_baro=False):
    """Lista (t_srodka, kat_matematyczny[rad], odleglosc[m]) + funkcja kursu + opis kondygnacji."""
    z = slam.wczytaj_lub_dekoduj(kat)
    t0 = int(z['t0'])
    bez = bez_imu or bool(z.get('bez_imu', False))
    ti, yaw = z['ti'], np.radians(z['yaw'])
    if bez:
        yaw = np.zeros_like(yaw)
    pit, rol = z['pitch'], z['roll']
    pion = slam.pion_z_info(kat)
    if pion is None:
        m10 = ti < ti[0] + 10
        pion = (np.median(pit[m10]), np.median(rol[m10]))
    prz = np.degrees(np.arccos(np.clip(np.cos(np.radians(pit - pion[0])) * np.cos(np.radians(rol - pion[1])), -1, 1)))
    pauzy = slam.przerwy(kat, t0)
    out, odrz = [], {'pauza': 0, 'przechyl': 0}
    for tt, ang, D in slam.obroty(z, rmin=RMIN, rmax=RMAX, pion=None):
        tm = float(tt.mean())
        if any(a <= tm <= b for a, b in pauzy):
            odrz['pauza'] += 1; continue
        if not bez and np.interp(tt, ti, prz).max() > slam.PAR['przechyl_max']:
            odrz['przechyl'] += 1; continue
        yp = np.interp(tt, ti, yaw); ym = np.interp(tm, ti, yaw)
        a = -ang + ZNAK * (yp - ym)
        out.append((tm, a, D))
    pozi = []
    pb = os.path.join(kat, 'baro.bin')
    if dziel_baro and os.path.exists(pb):
        pozi = [((a - t0) / 1e9, (b - t0) / 1e9, h) for a, b, h in S.poziomy(S.wysokosci(pb))]
    if not pozi:
        pozi = [(-1e12, 1e12, 0.0)]
    kurs = lambda t: ZNAK * np.interp(t, ti, yaw)
    return out, kurs, pozi, odrz, bez


def do_wiazek(a, D):
    a = (a + np.pi) % (2 * np.pi) - np.pi
    k = np.floor((a + np.pi) / (2 * np.pi) * N_WIAZEK).astype(int) % N_WIAZEK
    r = np.full(N_WIAZEK, np.inf)
    np.minimum.at(r, k, D)
    r[~np.isfinite(r)] = 0.0          # brak odczytu: ponizej range_min -> programy pomijaja wiazke
    return r


def zapisz_bag(sciezka, sk, kurs, z_imu=True):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan, Imu
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from builtin_interfaces.msg import Time

    def czas(t):
        t = BAZA + t; s = int(t)
        return Time(sec=s, nanosec=int(round((t - s) * 1e9)) % 1000000000)

    def ns(t):
        return int(round((BAZA + t) * 1e9))

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=sciezka, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for nazwa, typ in (('/scan', 'sensor_msgs/msg/LaserScan'), ('/tf', 'tf2_msgs/msg/TFMessage'), ('/imu', 'sensor_msgs/msg/Imu')):
        w.create_topic(rosbag2_py.TopicMetadata(name=nazwa, type=typ, serialization_format='cdr'))
    ta, tb = sk[0][0] - 1.0, sk[-1][0] + 0.5
    zd = []
    # TF odom->base_link (tylko kurs) i IMU, 50 Hz; zaczynaja 1 s przed pierwszym skanem
    th0 = kurs(ta)
    tt = np.arange(ta, tb, 0.02)
    th = kurs(tt) - th0
    om = np.gradient(np.unwrap(th), tt)
    for t, h, o in zip(tt, th, om):
        m = TransformStamped()
        m.header.stamp = czas(t); m.header.frame_id = 'odom'; m.child_frame_id = 'base_link'
        m.transform.rotation.z = float(np.sin(h / 2)); m.transform.rotation.w = float(np.cos(h / 2))
        zd.append((ns(t), '/tf', TFMessage(transforms=[m])))
        if z_imu:
            i = Imu()
            i.header.stamp = czas(t); i.header.frame_id = 'base_link'
            i.orientation_covariance[0] = -1.0          # orientacji nie podajemy
            i.angular_velocity.z = float(o)
            i.linear_acceleration.z = 9.81
            zd.append((ns(t), '/imu', i))
    for t, a, D in sk:
        m = LaserScan()
        m.header.stamp = czas(t); m.header.frame_id = 'base_link'
        m.angle_min = float(-np.pi); m.angle_increment = float(2 * np.pi / N_WIAZEK)
        m.angle_max = float(-np.pi + (N_WIAZEK - 1) * m.angle_increment)
        m.time_increment = 0.0; m.scan_time = 0.1
        m.range_min = RMIN; m.range_max = RMAX
        m.ranges = do_wiazek(a, D).astype(np.float32).tolist()
        zd.append((ns(t), '/scan', m))
    zd.sort(key=lambda x: x[0])
    for t, top, m in zd:
        w.write(top, serialize_message(m), t)
    del w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sesja'); ap.add_argument('wyjscie')
    ap.add_argument('--kondygnacja', type=int, help='numer odcinka z barometru (od 0); domyslnie wszystkie')
    ap.add_argument('--bez-imu', action='store_true')
    ap.add_argument('--dziel-baro', action='store_true', help='dziel sesje na kondygnacje wg barometru (domyslnie cala sesja = jedna)')
    ap.add_argument('--min-s', type=float, default=30.0, help='pomijaj odcinki krotsze niz tyle sekund')
    a = ap.parse_args()
    kat = a.sesja.rstrip('/')
    sk, kurs, pozi, odrz, bez = skany(kat, a.bez_imu, a.dziel_baro)
    print(f'skanow: {len(sk)}, odrzucone: {odrz}, kondygnacji (barometr): {len(pozi)}, IMU: {"nie" if bez else "tak"}')
    os.makedirs(a.wyjscie, exist_ok=True)
    opis = []
    for nr, (ta, tb, h) in enumerate(pozi):
        if a.kondygnacja is not None and nr != a.kondygnacja:
            continue
        s = [x for x in sk if ta <= x[0] <= tb]
        if len(s) < 50 or s[-1][0] - s[0][0] < a.min_s:
            print(f'  kondygnacja {nr}: za krotka ({len(s)} skanow) - pomijam'); continue
        p = os.path.join(a.wyjscie, f'k{nr}.bag')
        if os.path.exists(p):
            import shutil; shutil.rmtree(p)
        zapisz_bag(p, s, kurs)
        d = s[-1][0] - s[0][0]
        print(f'  kondygnacja {nr}: {len(s)} skanow, {d:.0f} s, wys. {h:+.1f} m -> {p}')
        opis.append({'nr': nr, 'bag': p, 'skanow': len(s), 'czas_s': round(d, 1), 'wys_m': round(h, 2)})
    json.dump({'kondygnacje': opis, 'odrzucone': odrz, 'imu': not bez}, open(os.path.join(a.wyjscie, 'bagi.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()

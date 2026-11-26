"""Sesja skanera z Unitree L2 (l2.bin z rejestratora) -> rosbag (ROS 1) dla Point-LIO.

    python3 l2_bag.py <katalog_sesji> [wyjscie.bag]

Tematy jak w oficjalnym sterowniku unitree_lidar_ros (to, czego oczekuje point_lio_unilidar, lidar_type 5):
    /unilidar/cloud  sensor_msgs/PointCloud2, pola x y z intensity (float32), ring (uint16), time (float32, s
                     od poczatku chmury), 18 pakietow na chmure, ramka unilidar_lidar
    /unilidar/imu    sensor_msgs/Imu (kwaternion, rad/s, m/s2), ramka unilidar_imu
Czas: znaczniki sprzetowe L2 (wspolny zegar chmury i IMU) przeliczone na czas maliny odporna regresja po dolnej
obwiedni opoznien odbioru (jak dla D500); skoki zegara maliny usuwane (sprawdz.popraw_czas).
Wymaga: numpy, rosbags (pip install rosbags).
"""
import os, sys, struct
import numpy as np

TU = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TU, '..'))
import l2_protokol as P
import sprawdz as S

PAKIETOW_NA_CHMURE = 18


def wczytaj(kat):
    """Ramki z l2.bin: [(czas_maliny_s, typ, dane)]."""
    st = P.Strumien(); out = []
    for t, d in S.rekordy(os.path.join(kat, 'l2.bin')):
        t = S.popraw_czas(kat, t) / 1e9
        for typ, dane in st.feed(d):
            out.append((t, typ, dane))
    if st.zle:
        print(f'L2: odrzucone uszkodzone ramki: {st.zle}')
    return out


def zegar(t_pi, t_l2):
    """Czas maliny = a * czas L2 + b: regresja po dolnej obwiedni (opoznienia odbioru sa tylko dodatnie)."""
    x = np.asarray(t_l2) - t_l2[0]; y = np.asarray(t_pi)
    a, b = np.polyfit(x, y, 1)
    for _ in range(5):
        r = y - (a * x + b); m = r < np.percentile(r, 20)
        a, b = np.polyfit(x[m], y[m], 1)
    return lambda t: a * (np.asarray(t) - t_l2[0]) + b, a


def main():
    kat = sys.argv[1].rstrip('/')
    wy = sys.argv[2] if len(sys.argv) > 2 else os.path.join(kat, '_wynik', 'l2.bag')
    os.makedirs(os.path.dirname(os.path.abspath(wy)), exist_ok=True)
    R = wczytaj(kat)
    ch = [(t, P.chmura(d, np)) for t, typ, d in R if typ == P.CHMURA]
    im = [(t, P.imu(d)) for t, typ, d in R if typ == P.IMU]
    if len(ch) < 50:
        raise SystemExit(f'za malo pakietow chmury w l2.bin ({len(ch)}) - czy L2 wysylal dane?')
    print(f'L2: {len(ch)} pakietow chmury, {len(im)} IMU')
    # wspolny zegar sprzetowy L2 -> czas maliny
    tp = [t for t, _ in ch] + [t for t, _ in im]
    tl = [c['t'] for _, c in ch] + [v[1] for _, v in im]
    o = np.argsort(tl); tp, tl = np.array(tp)[o], np.array(tl)[o]
    na_pi, a = zegar(tp, tl)
    print(f'L2: skala zegara {(a - 1) * 1e6:+.0f} ppm, czas nagrania {tl[-1] - tl[0]:.0f} s')
    t0 = float(na_pi(tl[0]))

    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    T = ts.types
    Time = T['builtin_interfaces/msg/Time']; Hdr = T['std_msgs/msg/Header']
    PF = T['sensor_msgs/msg/PointField']; PC2 = T['sensor_msgs/msg/PointCloud2']
    Imu = T['sensor_msgs/msg/Imu']; Q = T['geometry_msgs/msg/Quaternion']; V3 = T['geometry_msgs/msg/Vector3']
    pola = [PF(name='x', offset=0, datatype=7, count=1), PF(name='y', offset=4, datatype=7, count=1),
            PF(name='z', offset=8, datatype=7, count=1), PF(name='intensity', offset=12, datatype=7, count=1),
            PF(name='ring', offset=16, datatype=4, count=1), PF(name='time', offset=20, datatype=7, count=1)]
    typ_p = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('_', '<u2'), ('time', '<f4')])
    zero_cov = np.zeros(9)

    def czas(t):
        s = int(t); return Time(sec=s, nanosec=int(round((t - s) * 1e9)) % 1000000000)

    if os.path.exists(wy):
        os.remove(wy)
    n_ch = 0
    with Writer(wy) as w:
        c_ch = w.add_connection('/unilidar/cloud', PC2.__msgtype__, typestore=ts)
        c_im = w.add_connection('/unilidar/imu', Imu.__msgtype__, typestore=ts)
        zap = []                                   # (czas_ns, polaczenie, dane) - zapis w kolejnosci czasu
        for _, (seq, t, q, g, acc) in im:
            tt = float(na_pi(t))
            msg = Imu(header=Hdr(seq=seq, stamp=czas(tt), frame_id='unilidar_imu'),
                      orientation=Q(x=q[1], y=q[2], z=q[3], w=q[0]), orientation_covariance=zero_cov,
                      angular_velocity=V3(x=g[0], y=g[1], z=g[2]), angular_velocity_covariance=zero_cov,
                      linear_acceleration=V3(x=acc[0], y=acc[1], z=acc[2]), linear_acceleration_covariance=zero_cov)
            zap.append((int(tt * 1e9), c_im, ts.serialize_ros1(msg, Imu.__msgtype__)))
        for k in range(0, len(ch) - PAKIETOW_NA_CHMURE + 1, PAKIETOW_NA_CHMURE):
            grupa = [c for _, c in ch[k:k + PAKIETOW_NA_CHMURE]]
            tc = grupa[0]['t']
            n = sum(len(c['x']) for c in grupa)
            if n < 100:
                continue
            a_ = np.zeros(n, typ_p); i = 0
            for c in grupa:
                m = len(c['x'])
                a_['x'][i:i + m], a_['y'][i:i + m], a_['z'][i:i + m] = c['x'], c['y'], c['z']
                a_['intensity'][i:i + m] = c['i']; a_['ring'][i:i + m] = 1
                a_['time'][i:i + m] = (c['t'] - tc) + c['dt']
                i += m
            tt = float(na_pi(tc))
            msg = PC2(header=Hdr(seq=n_ch, stamp=czas(tt), frame_id='unilidar_lidar'), height=1, width=n, fields=pola,
                      is_bigendian=False, point_step=typ_p.itemsize, row_step=typ_p.itemsize * n,
                      data=np.frombuffer(a_.tobytes(), np.uint8), is_dense=True)
            zap.append((int(tt * 1e9), c_ch, ts.serialize_ros1(msg, PC2.__msgtype__)))
            n_ch += 1
        for tns, c, d in sorted(zap, key=lambda x: x[0]):
            w.write(c, tns, d)
    print(f'L2: zapisano {n_ch} chmur i {len(im)} IMU -> {wy}')


if __name__ == '__main__':
    main()

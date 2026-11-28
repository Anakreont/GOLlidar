#!/usr/bin/env python3
"""Import cudzego nagrania ROS 1 z gotowa (zlozona) chmura + odometria jako sesji przegladarki - bez Point-LIO.

    python import_bag_chmura.py <nagranie.bag> <katalog_sesji> [--chmura /lidar/point_cloud] [--odom /lidar/odom]
                                [--budynek "Przyklad RealRecon"] [--kondygnacja Bunker] [--voxel 0.02]

Uruchamiac w srodowisku z rosbag (~/ros1_l2). Chmura w ukladzie mapy (frame 'world'/'map'/'odom') jest sumowana wprost;
w ukladzie czujnika - przeksztalcana pozycja z odometrii (najblizsza w czasie). Wynik: <sesja>/_wynik/l2/chmura.pcd,
trasa.csv (t;x;y;z), info.txt; dalej l2_rzut.py liczy rzut, plan, pomieszczenia (jak dla L2).
"""
import argparse, os, sys
from datetime import datetime

import numpy as np


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def xyz_z_pc2(m):
    """sensor_msgs/PointCloud2 -> N x 3 (pola x, y, z float32)."""
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3, 'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    A = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    P = np.stack([A['x'], A['y'], A['z']], 1).astype(np.float64)
    return P[np.isfinite(P).all(1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('sesja')
    ap.add_argument('--chmura'); ap.add_argument('--odom')
    ap.add_argument('--budynek', default='Przykład'); ap.add_argument('--kondygnacja', default='')
    ap.add_argument('--voxel', type=float, default=0.02)
    a = ap.parse_args()
    import rosbag
    b = rosbag.Bag(a.bag)
    typy = b.get_type_and_topic_info()[1]
    tc = a.chmura or next(t for t, i in typy.items() if i.msg_type == 'sensor_msgs/PointCloud2')
    to = a.odom or next((t for t, i in typy.items() if i.msg_type == 'nav_msgs/Odometry'), None)
    T, poz = [], []
    if to:
        for _, m, ts in b.read_messages(topics=[to]):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T.append(m.header.stamp.to_sec() or ts.to_sec()); poz.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    T = np.array(T); poz = np.array(poz)
    chm = []; n_sk = 0; uklad_mapy = None
    for _, m, ts in b.read_messages(topics=[tc]):
        P = xyz_z_pc2(m); n_sk += 1
        if uklad_mapy is None:
            uklad_mapy = m.header.frame_id.lstrip('/') in ('world', 'map', 'odom', 'camera_init')
        if not uklad_mapy:
            if not len(T):
                raise SystemExit('chmura w ukladzie czujnika, a brak odometrii')
            i = int(np.clip(np.searchsorted(T, m.header.stamp.to_sec()), 0, len(T) - 1))
            P = P @ quat_R(poz[i, 3:]).T + poz[i, :3]
        # przerzedzenie na biezaco (pamiec)
        k = np.unique(np.floor(P / a.voxel).astype(np.int64), axis=0, return_index=True)[1]
        chm.append(P[k].astype(np.float32))
    P = np.vstack(chm)
    k = np.unique(np.floor(P / a.voxel).astype(np.int64), axis=0, return_index=True)[1]; P = P[k]
    wy = os.path.join(a.sesja, '_wynik', 'l2'); os.makedirs(wy, exist_ok=True)
    n = len(P)
    with open(os.path.join(wy, 'chmura.pcd'), 'wb') as f:
        f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'.encode())
        f.write(P.astype('<f4').tobytes())
    if len(T):
        with open(os.path.join(wy, 'trasa.csv'), 'w') as f:
            f.write('t;x;y;z\n')
            for t, p in zip(T, poz):
                f.write(f'{t:.3f};{p[0]:.3f};{p[1]:.3f};{p[2]:.3f}\n')
    t0 = datetime.fromtimestamp(b.get_start_time()); t1 = datetime.fromtimestamp(b.get_end_time())
    with open(os.path.join(a.sesja, 'info.txt'), 'w', encoding='utf-8') as f:
        f.write(f'nazwa={os.path.basename(a.sesja)}\nbudynek={a.budynek}\nkondygnacja={a.kondygnacja}\nstart={t0.isoformat(timespec="seconds")}\n'
                f'koniec={t1.isoformat(timespec="seconds")}\nl2=import: {os.path.basename(a.bag)} ({tc}, {to}), chmura gotowa - bez Point-LIO\n')
    print(f'{n_sk} skanow ({tc}, uklad mapy: {uklad_mapy}), {len(T)} pozycji ({to}) -> {n} pkt (siatka {a.voxel} m), '
          f'zakres {np.ptp(P, 0).round(1).tolist()} m')


if __name__ == '__main__':
    main()

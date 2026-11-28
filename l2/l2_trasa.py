"""Trasa Point-LIO (czas, x, y, z glowicy) z rosbaga zapisanego w trakcie liczenia mapy -> trasa.csv.
Potrzebna do podzialu nagrania na kondygnacje (l2_rzut.py --trasa): z trasy wiadomo, na jakich wysokosciach
chodzil operator, wiec jedno ciagle nagranie parter + pietro + poddasze daje osobne rzuty.

    python l2_trasa.py <pointlio_wynik.bag> <trasa.csv>
"""
import sys
from pathlib import Path


def czytaj(sciezka):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    wyn = []
    with AnyReader([Path(sciezka)], default_typestore=get_typestore(Stores.ROS1_NOETIC)) as r:
        tematy = {c.topic for c in r.connections}
        t_odo = next((t for t in ('/pointlio/odom', '/aft_mapped_to_init', '/Odometry') if t in tematy), None)
        if not t_odo:
            raise SystemExit(f'w nagraniu brak tematu pozycji: {sorted(tematy)}')
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == t_odo]):
            m = r.deserialize(raw, c.msgtype)
            p = m.pose.pose.position
            wyn.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
    return sorted(wyn)


if __name__ == '__main__':
    T = czytaj(sys.argv[1])
    with open(sys.argv[2], 'w') as f:
        f.write('t;x;y;z\n')
        for w in T[::5]:                       # ~10 Hz wystarczy
            f.write(';'.join(f'{v:.3f}' for v in w) + '\n')
    print(f'trasa: {len(T)} pozycji -> {sys.argv[2]}')

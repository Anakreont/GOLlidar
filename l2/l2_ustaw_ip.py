#!/usr/bin/env python3
"""Jednorazowa zmiana adresu Unitree L2 (zamiast programu Unilidar 2 na Windowsie).

Fabrycznie L2 ma 192.168.1.62 i wysyla dane do 192.168.1.2 - to koliduje z siecia domowa (192.168.1.x)
i z siecia GOLlidar maliny. Docelowo: L2 192.168.2.62 -> malina 192.168.2.2 (osobna podsiec na kablu).

Na malinie, z L2 podpietym kablem do gniazda Ethernet:
    sudo ip addr add 192.168.1.2/24 dev eth0          # chwilowo adres, ktory L2 zna z fabryki
    python3 ~/skaner/l2/l2_ustaw_ip.py               # wysyla nowa konfiguracje i sprawdza odpowiedz
    sudo ip addr del 192.168.1.2/24 dev eth0
Potem wylacz i wlacz zasilanie L2 i ustaw na stale adres maliny na kablu:
    sudo nmcli con add type ethernet ifname eth0 con-name l2 ipv4.method manual ipv4.addresses 192.168.2.2/24
(bez bramy - kabel sluzy tylko do L2).
"""
import argparse, os, socket, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import l2_protokol as P


def main():
    ap = argparse.ArgumentParser(description='Zmiana adresu IP Unitree L2')
    ap.add_argument('--stary-l2', default='192.168.1.62')
    ap.add_argument('--nowy-l2', default='192.168.2.62')
    ap.add_argument('--nowy-host', default='192.168.2.2', help='adres maliny na kablu, na ktory L2 ma wysylac dane')
    ap.add_argument('--sprawdz', action='store_true', help='tylko nasluchuj, czy L2 przysyla dane (port 6201)')
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', P.PORT_HOST)); s.settimeout(3)
    try:
        d, skad = s.recvfrom(65535)
        print(f'L2 odpowiada z {skad[0]} ({len(d)} B)')
    except socket.timeout:
        print('Brak danych z L2 na porcie 6201 - sprawdz kabel, zasilanie 12 V i adres maliny na eth0.')
        if a.sprawdz:
            return 1
    if a.sprawdz:
        return 0
    print(f'Wysylam: L2 {a.stary_l2} -> nowy adres {a.nowy_l2}, dane do {a.nowy_host}')
    for _ in range(3):
        s.sendto(P.ustaw_ip(a.nowy_l2, a.nowy_host), (a.stary_l2, P.PORT_L2)); time.sleep(0.3)
    print('Gotowe. Wylacz i wlacz zasilanie L2, potem ustaw na malinie eth0 na', a.nowy_host + '/24 (opis w pliku).')
    return 0


if __name__ == '__main__':
    sys.exit(main())

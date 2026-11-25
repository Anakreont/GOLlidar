# Test odpornosci na luki w sledzeniu: python3 test_luki.py "[[t_od,czas_s,dx,dy,dth_st],...]" nowy|stary SESJA CAD.dxf
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam, porownaj_cad as pc
S_ = sys.argv[3]
cad = pc.odcinki_cad(sys.argv[4], ('Z_POWIERZCHNIE',), (), (55.5, 53, 75.5, 62.5))
scen = json.loads(sys.argv[1]); tryb = sys.argv[2]
slam.TEST_LUKI[:] = [tuple(x) for x in scen]
if tryb == 'stary': slam.PAR['luka_s'] = 1e9
Q = pc.chmura_sesji(S_, zszyj=(tryb == 'nowy'))
R, t, st, Y = pc.dopasuj(Q, cad, log=lambda *a: None)
print(json.dumps({'tryb': tryb, 'scen': len(scen), 'fragmentow': int(slam.mapa.frag[:, 1].max()) + 1, 'luki': [(round(a), round(b, 1), None if q is None else [round(float(x), 3) if not isinstance(x, str) else x for x in q]) for a, b, q in slam.LOG_LUK],
                  'blad_lic_mm': st['blad_lic_mm'], 'p90': st['blad_lic_p90_mm'], 'pokrycie': st['pokrycie_cad_proc']}))

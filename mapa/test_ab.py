"""A/B wariantow slam.PAR na jednej sesji: python3 test_ab.py <sesja> '{"klucz": wartosc}' <nazwa>
Miary po sledzeniu i po dopracowaniu: podwojne % (to samo lico po > 25 s, 5-30 cm obok), p90, grubosc sciany
(odch. std punktow wzdluz normalnej wsrod sasiadow 15 cm; mediana / 90 %). Liczba fragmentow."""
import sys,os,json,numpy as np; sys.path.insert(0,'.'); sys.path.insert(0,'..')
import slam, posegraf
from scipy.spatial import cKDTree
def swiat(kf):
    W=[]
    for T,p,n in kf:
        m=np.isfinite(n[:,0]); c,s=np.cos(T[2]),np.sin(T[2]); R=np.array([[c,s],[-s,c]]); Pw=p[m]@R+T[:2]; W.append((Pw,slam._do_czujnika(Pw,n[m]@R,T)))
    return W
def miary(kf,kt):
    W=swiat(kf); kt=np.asarray(kt); r_all=[]
    for i in range(len(kf)):
        o=np.where(np.abs(kt-kt[i])>25)[0]
        if not len(o) or not len(W[i][0]): continue
        o=o[np.hypot(*(np.array([kf[j][0][:2] for j in o])-kf[i][0][:2]).T)<10]
        if not len(o): continue
        Q=np.vstack([W[j][0] for j in o]); N=np.vstack([W[j][1] for j in o])
        d,j=cKDTree(Q).query(W[i][0],distance_upper_bound=0.3); m=np.isfinite(d)
        if not m.any(): continue
        zg=np.sum(N[j[m]]*W[i][1][m],1)>0.7
        r_all.append(np.abs(np.sum((W[i][0][m][zg]-Q[j[m]][zg])*N[j[m]][zg],1)))
    r=np.concatenate(r_all) if r_all else np.array([0.])
    # grubosc sciany: odch. std wzdluz normalnej wsrod sasiadow (15 cm, zgodna normalna) - wszystkie skany razem
    P=np.vstack([w[0] for w in W]); N=np.vstack([w[1] for w in W]); tr=cKDTree(P)
    idx=np.random.default_rng(0).choice(len(P),min(4000,len(P)),replace=False); g=[]
    for i in idx:
        nb=tr.query_ball_point(P[i],0.15); nb=np.array(nb); nb=nb[N[nb]@N[i]>0.8]
        if len(nb)>=15: g.append(np.std((P[nb]-P[i])@N[i]))
    g=np.array(g) if g else np.array([0.])
    return np.mean(r>0.05)*100, np.percentile(r,90)*1000, np.median(g)*1000, np.percentile(g,90)*1000
kat=sys.argv[1]; war=json.loads(sys.argv[2]); naz=sys.argv[3]
z=slam.wczytaj_lub_dekoduj(kat)
if war.get('yaw_zyro') and 'yaw_zyro' not in z: print(f"{os.path.basename(kat)[:38]:38s} {naz:14s} brak zyroskopu"); sys.exit()
slam.PAR.update(war)
so=sys.stdout; sys.stdout=open(os.devnull,'w')
P,kf,info,pion=slam.mapa(kat,znak=-1,dane=z,rzutuj=False)
kt=[P[np.argmin(np.abs(P[:,1:4]-Tk).sum(1)),0] for Tk,_,_ in kf]
m1=miary(kf,kt)
Xd=posegraf.dopracuj(kf,kt,log=lambda *a:None); kfd=[(Xd[i],kf[i][1],kf[i][2]) for i in range(len(kf))]
m2=miary(kfd,kt); sys.stdout=so
print(f"{os.path.basename(kat)[:38]:38s} {naz:14s} sledzenie: podw {m1[0]:5.1f}% p90 {m1[1]:4.0f} grub {m1[2]:4.1f}/{m1[3]:4.1f} mm | +dopr: podw {m2[0]:5.1f}% p90 {m2[1]:4.0f} grub {m2[2]:4.1f}/{m2[3]:4.1f} mm | frag {len(set(map(tuple,slam.mapa.frag.tolist())))}",flush=True)

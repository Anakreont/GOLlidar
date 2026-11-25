import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))
import slam, sprawdz as S
k=sys.argv[1]; out=sys.argv[2]
z=slam.wczytaj(k); t0=z['t0']
w=S.wysokosci(k+'/baro.bin'); pozi=list(S.poziomy(w))
P,kf,info,pion=slam.mapa(k,znak=-1,segmenty=[(b-t0)/1e9 for a,b,h in pozi[:-1]])
print('pozycji',len(P),'kluczowych',len(kf),'odrzuconych',len(info), {r:sum(1 for _,x in info if x==r) for r in ('przechyl','icp','skok')})
print('poziomy', [(round((a-t0)/1e9),round((b-t0)/1e9),round(h,1)) for a,b,h in pozi])
# punkty kluczowe z czasem -> przypisanie do poziomu
kt=[]
for Tk,pk,nk in kf:
    i=np.argmin(np.abs(P[:,1:4]-Tk).sum(1)); kt.append(P[i,0])
fig,axs=plt.subplots(1,len(pozi),figsize=(7*len(pozi),7))
axs=np.atleast_1d(axs)
for ax,(a,b,h) in zip(axs,pozi):
    a=(a-t0)/1e9; b=(b-t0)/1e9
    pts=[]
    for (Tk,pk,nk),t in zip(kf,kt):
        if a<=t<=b:
            c,s=np.cos(Tk[2]),np.sin(Tk[2]); pts.append(pk@np.array([[c,s],[-s,c]])+Tk[:2])
    m=(P[:,0]>=a)&(P[:,0]<=b)
    if pts:
        q=np.vstack(pts); ax.scatter(q[:,0],q[:,1],s=0.3,c='k',alpha=0.25,lw=0)
    ax.plot(P[m,1],P[m,2],'r-',lw=0.8); ax.plot(P[m,1][:1],P[m,2][:1],'go')
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_title(f'{a:.0f}-{b:.0f} s, poziom {h:+.1f} m')
plt.tight_layout(); plt.savefig(out,dpi=130)
np.save(out+'.poses.npy',P)

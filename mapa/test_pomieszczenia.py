import json, numpy as np, sys
rng = np.random.default_rng(1)
# sciany: odcinki (x1,y1,x2,y2) w m; drzwi/okna jako przerwy
W = []
def sc(x1,y1,x2,y2,przerwy=()):
    # przerwy: lista (od, do) wzdluz odcinka w m od poczatku
    L = np.hypot(x2-x1,y2-y1); u = np.array([x2-x1,y2-y1])/L; a=0
    for p,q in sorted(przerwy)+[(L,L)]:
        if p>a: W.append((x1+u[0]*a,y1+u[1]*a,x1+u[0]*p,y1+u[1]*p))
        a=q
okna=lambda L:[(L*0.3,L*0.3+1.5)]
sc(0,0,15,0,[(1.2,2.7),(5,6.5),(12,13.5)])
sc(15,0,15,8.5,[(3,5)]); sc(15,8.5,0,8.5,[(2,3.5),(9,10.5)]); sc(0,8.5,0,0,[(3,4.2)])
sc(0,3.5,11,3.5,[(1.5,2.4),(5.5,6.4),(9,9.9)])        # korytarz dol + drzwi do A,B,C
sc(0,5.0,11,5.0,[(2,2.9),(7,7.9)])                     # korytarz gora + drzwi D,E
sc(4,0,4,3.5); sc(8,0,8,3.5); sc(5,5,5,8.5)
sc(11,0,11,8.5,[(3.7,4.7)])                             # wejscie do salonu F z korytarza 1.0 m
th=np.radians(7); R=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
pts=[]
for x1,y1,x2,y2 in W:
    n=int(np.hypot(x2-x1,y2-y1)/0.02)
    t=rng.random(n); p=np.c_[x1+(x2-x1)*t,y1+(y2-y1)*t]+rng.normal(0,0.02,(n,2)); pts.append(p)
for cx,cy,r in [(2,1.5,0.4),(13,6,0.6),(7,7,0.5)]:   # meble
    a=rng.random(300)*2*np.pi; pts.append(np.c_[cx+r*np.cos(a),cy+r*np.sin(a)])
pts.append(np.c_[rng.uniform(-4,19,200), rng.uniform(-4,12,200)])  # odosobnione smieci
P=np.vstack(pts)@R.T
sciany=[(np.array([[x1,y1],[x2,y2]])@R.T).tolist() for x1,y1,x2,y2 in W]
tr=np.array([[0.6,4.25],[1.95,4.25],[1.95,2],[1.95,4.25],[5.9,4.25],[5.9,2],[5.9,4.25],[9.45,4.25],[9.45,2],[9.45,4.25],
 [7.45,4.25],[7.45,5.6],[8.5,6.5],[7.45,5.6],[7.45,4.25],[2.45,4.25],[2.45,7],[2.45,4.25],[10.5,4.25],[13,4.2],[13,2],[13,4.2],[14,7.5],[13,4.2]])
d=[];
for a,b in zip(tr[:-1],tr[1:]):
    n=max(2,int(np.linalg.norm(b-a)/0.1)); d.append(np.linspace(a,b,n))
tr=np.vstack(d)@R.T
k={'nr':0,'sciany':sciany,'chmura_cm':np.round(P*100).astype(int).ravel().tolist(),'trasa':tr.round(2).tolist(),
   'znaczniki':[{'rodzaj':'sala','opis':'Salon','x':float((np.array([13,6.5])@R.T)[0]),'y':float((np.array([13,6.5])@R.T)[1])}],
   'wysokosc_m':2.6,'szer':16,'wys':10}

# --- test: python3 test_pomieszczenia.py  (syntetyczny dom 15 x 8,5 m, 7 pomieszczen, obrot 7 st., szum 2 cm, okna bez punktow, meble)
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import pomieszczenia
w = pomieszczenia.podziel(k)
wzor = sorted([14.0, 14.0, 10.5, 17.5, 21.0, 34.0, 16.5])
wyn = sorted(p['pow_m2'] for p in w['pomieszczenia'])
print('wykryte:', wyn); print('wzorzec:', wzor)
assert len(wyn) == len(wzor), 'zla liczba pomieszczen'
bl = [abs(a / b - 1) for a, b in zip(wyn, wzor)]
print('maks. blad powierzchni: %.1f %%' % (100 * max(bl)))
assert max(bl) < 0.03
print('OK')

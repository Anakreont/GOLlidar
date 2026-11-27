"""Odczyt rzutu z ArCADii (DXF): sciany, drzwi, okna, pomieszczenia (etykiety z numerem, nazwa, powierzchnia) -> dane.pkl.
Uzycie: python3 arcadia_czytaj.py rysunek.dxf   (rysunkow klientow NIE dodawac do repo - jest publiczne)"""
import ezdxf, re, pickle, sys
from collections import Counter, defaultdict
from ezdxf.proxygraphic import ProxyGraphic
d=ezdxf.readfile(sys.argv[1])  # DXF z ArCADii (obiekty ARCADIA_RXENTITY z grafika zastepcza)
msp=d.modelspace()
def warstwa(e):
    for sub in e.xtags.subclasses:
        for t in sub:
            if t.code==8: return t.value
    return '?'
def klasa_txt(e):
    out=[]
    for sub in e.xtags.subclasses[2:]:
        for t in sub:
            if t.code in (1,2,3,1000,300,301,302,303,304) and isinstance(t.value,str): out.append(t.value[:40])
    return out[:6]
dane=[]
for e in msp:
    if e.dxftype()!='ARCADIA_RXENTITY': continue
    w=warstwa(e)
    try: V=list(ProxyGraphic(e.proxy_graphic,d).virtual_entities())
    except Exception as ex: V=[]; 
    m=re.match(r"isa_VCD0F10432144119_(?:B18C0F104321561F9([LR]\w{16})|(T))(\w+)",w)
    poz,typ=((m.group(1) or "T"),m.group(3)) if m else ('?',w)
    geo=[]
    for v in V:
        t=v.dxftype()
        try:
            if t=='LINE': geo.append(('L',[tuple(v.dxf.start),tuple(v.dxf.end)]))
            elif t in('LWPOLYLINE',): geo.append(('P',[tuple(p)[:3] for p in v.get_points('xy')]))
            elif t=='POLYLINE': geo.append(('P',[tuple(p.dxf.location) for p in v.vertices]))
            elif t=='TEXT': geo.append(('T',(tuple(v.dxf.insert),v.dxf.text)))
            elif t=='MTEXT': geo.append(('T',(tuple(v.dxf.insert),v.text)))
            else: geo.append((t,None))
        except Exception as ex: geo.append((t,'err'))
    dane.append({'w':w,'poz':poz,'typ':typ,'geo':geo,'txt':klasa_txt(e)})
pickle.dump(dane,open('dane.pkl','wb'))
c=Counter((x['poz'],x['typ']) for x in dane)
for k,v in sorted(c.items()): print(v,k)

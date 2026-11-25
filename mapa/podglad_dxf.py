import sys, ezdxf, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
d=ezdxf.readfile(sys.argv[1]); fig,ax=plt.subplots(figsize=(18,8)); kol={'ZDJECIA':'b','PRZEKROJE':'m','ZNACZNIKI':'orange','OPIS':'g'}
for e in d.modelspace():
    L=e.dxf.layer
    if e.dxftype()=='LWPOLYLINE' and L=='SCIANY': ax.plot(*zip(*[(p[0],p[1]) for p in e.get_points()]),'k-',lw=1.3)
    elif e.dxftype()=='LWPOLYLINE': ax.plot(*zip(*[(p[0],p[1]) for p in e.get_points()]),'r-',lw=.3,alpha=.5)
    elif e.dxftype()=='POINT': ax.plot(e.dxf.location.x,e.dxf.location.y,',',color='0.75')
    elif e.dxftype()=='CIRCLE': ax.add_patch(plt.Circle((e.dxf.center.x,e.dxf.center.y),e.dxf.radius,fill=False,color=kol[L]))
    elif e.dxftype()=='TEXT': ax.text(e.dxf.insert.x,e.dxf.insert.y,e.dxf.text,fontsize=6 if L!='OPIS' else 10,color=kol[L])
ax.set_aspect('equal'); ax.grid(alpha=.3); plt.tight_layout(); plt.savefig(sys.argv[2],dpi=120)

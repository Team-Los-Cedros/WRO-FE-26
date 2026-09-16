import csv,sys
LIDAR_X=128.0; SIGMA_OK=60.0
def F(x,d=0.0):
    try: return float(x)
    except: return d
print("corrida   pasos REALES  por lado MALO  1er fallo   dist    vel")
for f in sys.argv[1:]:
    r=list(csv.DictReader(open(f)))
    t0=F(r[0]['t']); n=len(r)
    dt=(F(r[-1]['t'])-t0)/max(1,n-1)
    dist=sum(abs(F(x['velocidad']))*4.0*dt for x in r)/1000.0
    objs={}
    for x in r:
        o=x['obj_id']
        if o and o!='0' and x['obj_lado'] in ('1','-1'): objs.setdefault(o,[]).append(x)
    ok=mal=0; primero=None
    for o,s in objs.items():
        for i in range(1,len(s)):
            if F(s[i-1]['trk_y'])+LIDAR_X >= 0 > F(s[i]['trk_y'])+LIDAR_X:
                c=s[i]; sep=F(c['separacion_lat'])
                if F(c['obj_sigma'],999) > SIGMA_OK: continue
                if sep>0: ok+=1
                else:
                    mal+=1
                    if primero is None: primero=(F(c['t'])-t0,o,c['trk_color'])
                break
    p="#%s %s a %.1fs"%(primero[1],primero[2],primero[0]) if primero else "ninguno"
    print("%-9s %8d %13d   %-16s %5.2fm %4.0fmm/s"%(f.split('_')[-1][:6],ok,mal,p,dist,dist*1000/(F(r[-1]['t'])-t0)))

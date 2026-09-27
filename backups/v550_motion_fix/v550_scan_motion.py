import gzip, json, math, sys
import numpy as np
from scipy.spatial import cKDTree

def rotation(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])

def pose(x,y,a):
    out=np.eye(3); out[:2,:2]=rotation(a); out[:2,2]=[x,y]; return out

def points(scan):
    r=np.array([np.nan if v is None else v for v in scan['ranges']])
    a=scan['angle_min']+np.arange(len(r))*scan['increment']
    ok=np.isfinite(r)&(r>max(.22,scan['range_min']))&(r<min(8.,scan['range_max']))
    return np.column_stack((r[ok]*np.cos(a[ok]),r[ok]*np.sin(a[ok])))[::2]

def icp(previous,current):
    tree=cKDTree(previous)
    total=np.eye(3); p=current.copy()
    for _ in range(45):
        distances,nearest=tree.query(p)
        mask=distances<min(.35,np.quantile(distances,.85)+1e-5)
        if mask.sum()<30: return None
        src=p[mask]; dst=previous[nearest[mask]]
        cs,cd=src.mean(0),dst.mean(0)
        u,_,vt=np.linalg.svd((src-cs).T@(dst-cd))
        fix=np.eye(2); fix[-1,-1]=np.linalg.det(vt.T@u.T)
        r=vt.T@fix@u.T; t=cd-r@cs
        step=np.eye(3); step[:2,:2]=r; step[:2,2]=t
        total=step@total; p=p@r.T+t
        if np.linalg.norm(t)<1e-5 and abs(math.atan2(r[1,0],r[0,0]))<1e-5:break
    d,_=tree.query(p)
    good=d<.12
    return total,float(np.median(d)),float(good.mean())

data=json.load(gzip.open(sys.argv[1],'rt')); s=data['series']
scans=s['/scan']; start=scans[0]['stamp']
od=s.get('/odom',[])
ots=np.array([v['stamp'] for v in od]); vals=np.array([[v['x'],v['y'],v['yaw']] for v in od]); vals[:,2]=np.unwrap(vals[:,2])
def odom(t):
    return pose(*[np.interp(t,ots,vals[:,j]) for j in range(3)])
rows=[]; previous=scans[0]; pp=points(previous)
for scan in scans[1:]:
    if scan['stamp']-previous['stamp']<.5:continue
    cp=points(scan)
    result=icp(pp,cp) if min(len(cp),len(pp))>30 else None
    if result:
        tf,error,overlap=result
        d=np.linalg.inv(odom(previous['stamp']))@odom(scan['stamp'])
        yaw=math.atan2(tf[1,0],tf[0,0]); oyaw=math.atan2(d[1,0],d[0,0])
        # Compare actual scan-coordinate motion with each candidate laser yaw.
        candidates={}
        for name,angle in [('yaw_0',0.),('yaw_pi',math.pi)]:
            extr=pose(.0834767944381037,.0000132296970929568,angle)
            pred=np.linalg.inv(extr)@d@extr
            candidates[name]=float(np.linalg.norm(tf[:2,2]-pred[:2,2]))
        row=dict(time=round(scan['stamp']-start,2),scan_dx=round(float(tf[0,2]),4),scan_dy=round(float(tf[1,2]),4),scan_yaw_deg=round(math.degrees(yaw),3),odom_dx=round(float(d[0,2]),4),odom_dy=round(float(d[1,2]),4),odom_yaw_deg=round(math.degrees(oyaw),3),median_error=error,overlap=overlap,**candidates)
        rows.append(row)
    previous=scan; pp=cp
moving=[r for r in rows if math.hypot(r['odom_dx'],r['odom_dy'])>.01 or abs(r['odom_yaw_deg'])>1 or math.hypot(r['scan_dx'],r['scan_dy'])>.015 or abs(r['scan_yaw_deg'])>1]
print('pairs:',len(rows),'motion pairs:',len(moving))
for row in moving:print(json.dumps(row))
translations=[r for r in moving if math.hypot(r['odom_dx'],r['odom_dy'])>.025 and r['overlap']>.8 and r['median_error']<.04]
print('Reliable translation pairs:',len(translations))
if translations:
    for key in ['yaw_0','yaw_pi']:print(key,'mean prediction error m:',float(np.mean([r[key] for r in translations])))
with open(sys.argv[1]+'.motion.json','w') as f:json.dump(rows,f,indent=2)

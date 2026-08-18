"""Bir bandın HAREKET + GPS + TAŞIMA profilini çıkarır (17.08 ile kıyaslanabilir)."""
import sys, glob, math
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BANT=sys.argv[1]
so=rosbag2_py.StorageOptions(uri=BANT, storage_id='mcap' if glob.glob(BANT+'/*.mcap') else 'sqlite3')
r=rosbag2_py.SequentialReader(); r.open(so, rosbag2_py.ConverterOptions('',''))
tip={t.name:t.type for t in r.get_all_topics_and_types()}
t0=None; G=[];O=[];RC=[];MD=[]
for _ in iter(int,1):
    if not r.has_next(): break
    topic,data,varis=r.read_next()
    if t0 is None: t0=varis
    ts=(varis-t0)/1e9
    if topic=='/mavros/global_position/global':
        m=deserialize_message(data,get_message(tip[topic]))
        G.append((ts,m.latitude,m.longitude,m.altitude,m.position_covariance[0],m.status.status))
    elif topic=='/girdap/fusion/odom':
        m=deserialize_message(data,get_message(tip[topic]))
        O.append((ts,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif topic=='/mavros/rc/in':
        m=deserialize_message(data,get_message(tip[topic])); RC.append((ts,float(m.channels[2]) if len(m.channels)>2 else np.nan))
    elif topic=='/mavros/state':
        m=deserialize_message(data,get_message(tip[topic])); MD.append((ts,m.mode,m.armed))
print(f"### {BANT.split('/')[-1]}")
print(f"süre {ts:.0f} s ({ts/60:.1f} dk) · odom {len(O)} · NavSatFix {len(G)}")
U_MAX=1.173
if not G: print("  GPS yok"); sys.exit()
G=np.array(G,dtype=object)
gt=np.array([float(a) for a in G[:,0]]); lat=np.array([float(a) for a in G[:,1]])
lon=np.array([float(a) for a in G[:,2]]); alt=np.array([float(a) for a in G[:,3]])
cov=np.array([float(a) for a in G[:,4]]); st=np.array([int(a) for a in G[:,5]])
lat0=np.median(lat); mlat=111320.0; mlon=111320.0*math.cos(math.radians(lat0))
gx=(lat-lat0)*mlat; gy=(lon-np.median(lon))*mlon

print(f"\n═══ GPS KALİTESİ ═══")
u,c=np.unique(st,return_counts=True)
print("  fix status: "+" · ".join(f"{int(a)}→{int(b)} mesaj (%{100*b/len(st):.1f})" for a,b in zip(u,c)))
gec=cov[cov>0]
if len(gec): print(f"  kovaryans[0]: ortanca {np.median(gec):.4f} m² ⇒ σ ≈ {math.sqrt(np.median(gec)):.2f} m · p95 {np.percentile(gec,95):.4f}")
print(f"  RTK/SBAS var mı: {'EVET' if (st>0).any() else 'HAYIR — ham GNSS'}")

print(f"\n═══ HAREKET PROFİLİ ═══")
w=max(int(round(1/np.median(np.diff(gt)))),1)
d=np.hypot(gx[w:]-gx[:-w], gy[w:]-gy[:-w]); v=d/(gt[w:]-gt[:-w]); tt=gt[w:]
yol=float(np.sum(np.hypot(np.diff(gx),np.diff(gy))))
print(f"  1 s yer değiştirme: ortanca {np.median(d)*100:.1f} cm · p99 {np.percentile(d,99)*100:.1f} cm · maks {d.max()*100:.1f} cm")
print(f"  toplam yol {yol:.1f} m · konum yayılımı std x {np.std(gx):.2f} y {np.std(gy):.2f} m")
print(f"  ⇒ {'HAREKETSİZ (yerinde durmuş)' if np.median(d)<0.05 else 'GERÇEK KOŞUM'}")

print(f"\n═══ u_max ({U_MAX} m/s) AŞIMLARI — TAŞIMA ADAYI ═══")
i=np.where(v>U_MAX)[0]
print(f"  aşan örnek: {len(i)}/{len(v)} (%{100*len(i)/len(v):.3f})  maks {v.max():.2f} m/s")
if len(i):
    bl=[];b=i[0];p=i[0]
    for k in i[1:]:
        if k-p>int(3*w): bl.append((b,p)); b=k
        p=k
    bl.append((b,p))
    print(f"  ayrı olay: {len(bl)}")
    RCa=np.array(RC) if RC else None
    for b,e in bl:
        t1,t2=tt[b],tt[e]
        m=(gt>=t1-2)&(gt<=t2+2)
        yol_o=float(np.sum(np.hypot(np.diff(gx[m]),np.diff(gy[m]))))
        net=math.hypot(gx[m][-1]-gx[m][0], gy[m][-1]-gy[m][0])
        dalt=alt[m].max()-alt[m].min()
        gaz="—"
        if RCa is not None and len(RCa):
            mm=(RCa[:,0]>=t1)&(RCa[:,0]<=t2)
            if mm.any(): gaz=f"{np.nanmin(RCa[mm,1]):.0f}-{np.nanmax(RCa[mm,1]):.0f}"
        print(f"   · t={t1:7.1f}→{t2:7.1f}s ({t2-t1:5.1f}s) maks {v[b:e+1].max():4.2f} m/s · yol {yol_o:5.1f} m · net {net:5.1f} m · net/yol {net/max(yol_o,1e-9):.2f} · Δirtifa {dalt:.2f} m · RC gaz {gaz}")
if MD:
    print(f"\n═══ KİP ═══")
    onc=None
    for t_,md,ar in MD:
        if (md,ar)!=onc: print(f"   t={t_:7.1f}s  {md} armed={ar}"); onc=(md,ar)

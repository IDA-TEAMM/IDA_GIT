"""🔴 HEDEF DUBASI / BEYAZ SOSİS DUBA YANLIŞ POZİTİF TESTİ (11.08.2026)

Soru: parkurda kesin bulunacak ama verimizde HİÇ OLMAYAN renkler modeli
kandırıyor mu? Özellikle **RAL 3026 (kırmızı hedef dubası)** — turuncuya çok
yakın; kenar dubası sanılırsa SAHTE KAPI üretir ⇒ P1/P2 gider.

Yöntem: gerçek karelerdeki GT duba piksellerini hedef renge boya (biçim, boyut,
su, ışık AYNEN kalır — yalnız RENK değişir), modeli koştur, o kutuda hâlâ
tespit var mı say. Böylece "renk mi biçim mi" ayrışır.
"""
import os, glob, cv2, numpy as np, collections
from ultralytics import YOLO

MODEL = os.path.expanduser("~/girdap_MODEL_512/girdap_512_ep87.pt")
H = os.path.expanduser("~/girdap_HAM_KARELER"); L = os.path.expanduser("~/girdap_ON_ETIKET/labels")
m = YOLO(MODEL); ADLAR = m.names
print(f"model: {os.path.basename(MODEL)}  sınıflar: {ADLAR}\n")

# Şartname renkleri (HSV ton derecesi, OpenCV 0-179 -> derece x2)
HEDEFLER = {
    "RAL 3026 KIRMIZI (hedef duba)": dict(ton=4,   doy=None, par=None),
    "RAL 6037 YEŞİL  (hedef duba)":  dict(ton=144, doy=None, par=None),
    "RAL 9005 SİYAH  (hedef duba)":  dict(ton=None,doy=20,   par=45),
    "BEYAZ (sosis sınır dubası)":    dict(ton=None,doy=15,   par=225),
}

def boya(im, kutu, ton, doy, par):
    x1,y1,x2,y2 = kutu
    kes = im[y1:y2, x1:x2]
    if kes.size == 0: return False
    hsv = cv2.cvtColor(kes, cv2.COLOR_BGR2HSV).astype(np.int16)
    m_ = hsv[:,:,1] > 60                      # duba gövdesi (doygun pikseller)
    if m_.sum() < 10: return False
    if ton is not None: hsv[:,:,0][m_] = ton//2
    if doy is not None: hsv[:,:,1][m_] = doy
    if par is not None:
        v = hsv[:,:,2].astype(float)
        hsv[:,:,2][m_] = np.clip(v[m_]*(par/max(1.0, v[m_].mean())), 0, 255)
    im[y1:y2, x1:x2] = cv2.cvtColor(np.clip(hsv,0,255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    return True

def ortusme(a, b):
    ix = max(0, min(a[2],b[2])-max(a[0],b[0])); iy = max(0, min(a[3],b[3])-max(a[1],b[1]))
    birlesim = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - ix*iy
    return ix*iy/birlesim if birlesim > 0 else 0

kareler = sorted(glob.glob(f"{H}/valid/*.jpg"))[::3]
sonuc = collections.defaultdict(lambda: {"kutu":0, "tespit":0, "sinif":collections.Counter(), "guven":[]})
temel = {"kutu":0, "tespit":0}

for p in kareler:
    k = os.path.splitext(os.path.basename(p))[0]; lp = f"{L}/{k}.txt"
    if not os.path.exists(lp): continue
    im0 = cv2.imread(p)
    if im0 is None: continue
    Hh, W = im0.shape[:2]
    kutular = []
    for s in open(lp):
        q = s.split()
        if len(q) < 5: continue
        cx,cy,w,h = map(float, q[1:5])
        if w*1352 < 20: continue                       # çok küçükleri atla (boyama anlamsız)
        kutular.append((int((cx-w/2)*W), int((cy-h/2)*Hh), int((cx+w/2)*W), int((cy+h/2)*Hh)))
    if not kutular: continue
    # TEMEL: dokunulmamış kare
    r = m.predict(im0, imgsz=512, conf=0.5, verbose=False)[0]
    tb = [tuple(map(int,b)) for b in r.boxes.xyxy.cpu().numpy()] if r.boxes is not None else []
    for kb in kutular:
        temel["kutu"] += 1
        if any(ortusme(kb,t) > 0.3 for t in tb): temel["tespit"] += 1
    # HER HEDEF RENK
    for ad, cfg in HEDEFLER.items():
        im = im0.copy(); boyandi = [kb for kb in kutular if boya(im, kb, **cfg)]
        if not boyandi: continue
        r = m.predict(im, imgsz=512, conf=0.5, verbose=False)[0]
        tb, tc, tg = [], [], []
        if r.boxes is not None:
            tb = [tuple(map(int,b)) for b in r.boxes.xyxy.cpu().numpy()]
            tc = r.boxes.cls.cpu().numpy().astype(int).tolist()
            tg = r.boxes.conf.cpu().numpy().tolist()
        S = sonuc[ad]
        for kb in boyandi:
            S["kutu"] += 1
            for i,t in enumerate(tb):
                if ortusme(kb,t) > 0.3:
                    S["tespit"] += 1; S["sinif"][ADLAR[tc[i]]] += 1; S["guven"].append(tg[i]); break

print(f"örneklem: {temel['kutu']} duba kutusu ({len(kareler)} kare)")
print(f"TEMEL (gerçek turuncu/sarı): {temel['tespit']}/{temel['kutu']} tespit = %{100*temel['tespit']/max(1,temel['kutu']):.1f}\n")
print(f"{'renk (parkurda VAR, verimizde YOK)':34} {'yanlış pozitif':>15} {'hangi sınıf':>28}")
print("-"*84)
for ad in HEDEFLER:
    S = sonuc[ad]
    if not S["kutu"]: continue
    o = 100*S["tespit"]/S["kutu"]
    bayrak = "🔴🔴" if o > 40 else ("🔴" if o > 15 else ("🟡" if o > 5 else "🟢"))
    sf = ", ".join(f"{k}:{v}" for k,v in S["sinif"].most_common()) or "-"
    g = f" (güven ort {np.mean(S['guven']):.2f})" if S["guven"] else ""
    print(f"{bayrak} {ad:31} {S['tespit']:5d}/{S['kutu']:<5d} %{o:5.1f}  {sf}{g}")

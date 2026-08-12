#!/usr/bin/env python3
"""FINAL EGITIM — ADIM 1: veri setini hazirla.

Iki is yapar:
  (a) Kareleri --boyut x --boyut'a EZER (stretch, varsayilan 512) <- DEPLOY GEOMETRISI
      Deploy: setPreviewKeepAspectRatio(False)  (duba_gecis_navigator.py:412)
      Ultralytics ise varsayilan olarak LETTERBOX yapar (base.py:240, r=min(...)).
      Ikisi ayrisirsa model egitimde yuvarlak, sahada 1,33x dikey uzamis duba gorur.
      09.08'de OLCULDU: uzak (<12 px) dubada 3,4 puan recall farki.
      Kare girdide letterbox etkisiz kalir (r=1.000, dolgu yok).
      YOLO etiketleri normalize (0-1) -> etiket dosyalari DEGISMEZ.
  (b) SIZINTISIZ bolme: kare numarasina gore bitisik bloklar + guard band.
      Rastgele bolme yasak: kareler saniyede birden cekildi, komsular ikiz;
      val skoru siser, sahada kor cikar, BELIRTI VERMEZ.
"""
import os, sys, shutil, argparse
import cv2, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--kaynak-img", default=os.path.expanduser("~/girdap_ON_ETIKET/images"))
ap.add_argument("--kaynak-lbl", default=os.path.expanduser("~/girdap_ON_ETIKET/labels"))
ap.add_argument("--hedef",      default=os.path.expanduser("~/girdap_EGITIM_HATTI/veri"))
ap.add_argument("--blok", type=int, default=8,  help="kac bitisik zaman blogu")
ap.add_argument("--val-orani", type=float, default=0.20)
ap.add_argument("--guard", type=int, default=15, help="blok sinirinda atilacak kare")
ap.add_argument("--boyut", type=int, default=512)   # deploy NN_GIRIS ile AYNI (12.08: 416 -> 512)
ap.add_argument("--negatif-oran", type=float, default=0.08,
                help="egitim setinde bos (dubasiz) kare orani; Ultralytics onerisi %0-10")
a = ap.parse_args()

lbls = sorted([x for x in os.listdir(a.kaynak_lbl)
               if x.startswith("kare_") and x.endswith(".txt")],
              key=lambda s: int(s[5:-4]))
if not lbls: sys.exit("etiket bulunamadi")
print(f"etiket dosyasi: {len(lbls)}")

# --- sizintisiz bolme: her blogun ilk %80 train / son %20 valid, sinirda guard
n = len(lbls); blok = max(1, n // a.blok)
train, valid, atilan = [], [], 0
for b0 in range(0, n, blok):
    g = lbls[b0:b0+blok]
    kes = int(len(g) * (1 - a.val_orani))
    train += g[:max(0, kes - a.guard)]
    valid += g[kes + a.guard:]
    atilan += min(a.guard, kes) + min(a.guard, len(g) - kes)
print(f"train {len(train)} / valid {len(valid)}  (guard band'de atilan ~{atilan})")

def kutu_sayisi(p):
    return sum(1 for l in open(p) if l.strip())

def yaz(liste, alt):
    di, dl = f"{a.hedef}/{alt}/images", f"{a.hedef}/{alt}/labels"
    os.makedirs(di, exist_ok=True); os.makedirs(dl, exist_ok=True)
    n = k = bos = 0
    for t in liste:
        ip = f"{a.kaynak_img}/{t[:-4]}.jpg"
        if not os.path.exists(ip): continue
        im = cv2.imread(ip)
        if im is None: continue
        # 🔴 EZ (stretch) — en-boy KORUNMAZ, deploy ile birebir
        im = cv2.resize(im, (a.boyut, a.boyut), interpolation=cv2.INTER_AREA)
        cv2.imwrite(f"{di}/{t[:-4]}.jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 95])
        shutil.copy(f"{a.kaynak_lbl}/{t}", f"{dl}/{t}")
        kk = kutu_sayisi(f"{dl}/{t}"); k += kk; bos += (kk == 0); n += 1
    print(f"  {alt:6}: {n} kare · {k} kutu · {bos} bos (%{100*bos/max(n,1):.1f})")
    return n, bos

os.makedirs(a.hedef, exist_ok=True)
_, tb = yaz(train, "train")
_, vb = yaz(valid, "valid")


# --- negatif (bos) kare oranini hedefe indir  [Eyup karari 09.08: %8]
# Bos kare "burada duba yok" ogretir; fazlasi pozitif sinyali seyreltir.
# Silme ESIT ARALIKLI yapilir -> farkli arka plan/isiktan negatif korunur,
# tek bir zaman blogu topluca gitmez.
def negatif_ayarla(alt, hedef):
    import re as _re
    d = f"{a.hedef}/{alt}"
    lab = sorted([f for f in os.listdir(f"{d}/labels") if f.endswith(".txt")],
                 key=lambda s: int(_re.search(r"(\d+)", s).group(1)))
    bos = [f for f in lab if os.path.getsize(f"{d}/labels/{f}") == 0]
    dolu = len(lab) - len(bos)
    tut = round(hedef * dolu / (1 - hedef))
    if tut >= len(bos):
        print(f"  {alt}: bos oran %{100*len(bos)/max(len(lab),1):.1f} — hedefin altinda, dokunulmadi")
        return
    adim = len(bos) / tut
    tutulan = {bos[min(int(i*adim), len(bos)-1)] for i in range(tut)}
    n = 0
    for f in bos:
        if f in tutulan: continue
        os.remove(f"{d}/labels/{f}")
        j = f"{d}/images/{f[:-4]}.jpg"
        if os.path.exists(j): os.remove(j)
        n += 1
    kalan = len(lab) - n
    print(f"  {alt}: {n} bos kare atildi -> {kalan} kare, bos oran %{100*tut/max(kalan,1):.1f}")

if a.negatif_oran > 0:
    print("\nnegatif oran ayari:")
    negatif_ayarla("train", a.negatif_oran)      # 🔴 yalniz TRAIN — valid gercek dagilimi korur

# 🔴 sinif isimleri KANONIK sirada — Roboflow export'u alfabetik yaziyordu
open(f"{a.hedef}/data.yaml", "w").write(
    f"path: {a.hedef}\ntrain: train/images\nval: valid/images\n"
    f"nc: 2\nnames: ['kenar_dubasi', 'engel_dubasi']\n")
print(f"\ndata.yaml yazildi -> {a.hedef}/data.yaml")
print("names: ['kenar_dubasi','engel_dubasi']   <- 0=kenar=TURUNCU, 1=engel=SARI")
if tb and tb/max(len(train),1) > 0.12:
    print(f"\n⚠️  negatif (bos) kare orani %{100*tb/len(train):.1f} — Ultralytics onerisi %0-10")

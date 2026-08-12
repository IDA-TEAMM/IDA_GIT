#!/usr/bin/env python3
"""FINAL EGITIM — ADIM 3: BLOB KABUL TESTI (3 pazarliksiz kontrol).

Bu betik blob'u URETMEZ, DENETLER. Uretim: luxonis/tools --use-rvc2 + blobconverter.
Neden var: memory'de "3 pazarliksiz kontrol" yaziyordu ama betigi yoktu; export
gunu telasinda atlanmasi cok kolay ve UCU DE SESSIZ HATA uretiyor.

Kullanim:
    python3 3_kabul_testi.py --blob models/yolo11n_duba_rvc2.blob \
                             --config models/config.json \
                             [--pt runs/.../best.pt --kare bir_kare.jpg]
"""
import argparse, json, os, sys

KANONIK = ["kenar_dubasi", "engel_dubasi"]      # 0 = TURUNCU, 1 = SARI
IMGSZ = 512      # deploy NN_GIRIS + blob girisi ile AYNI (12.08: 416 -> 512).
                 # 🔴 Kiyas modelin EGITILDIGI boyutta yapilmali: 512 modelini
                 #    416 ile kosturmak olcek uyumsuzlugu uretir (11.08'de 608
                 #    testinde yasandi, tespit sayisi okunamaz cikti).
hata = []
uyari = []

ap = argparse.ArgumentParser()
ap.add_argument("--blob", required=True)
ap.add_argument("--config", required=True)
ap.add_argument("--pt", help="ayni modelin .pt hali — 1:1 kiyas icin")
ap.add_argument("--kare", help="cihazin passthrough'undan alinmis kare")
a = ap.parse_args()

print("=" * 68)
print("KONTROL 1 — numShaves == 4")
print("=" * 68)
# 🔴 Fazla shave = model HIC YUKLENMEZ (hiz kaybi degil, sert ret).
#    depthai 2.30'da superblob YOK -> shave export aninda donuyor, sahada telafi yok.
try:
    import depthai as dai
    b = dai.OpenVINO.Blob(a.blob)
    print(f"  numShaves = {b.numShaves}")
    if b.numShaves != 4:
        hata.append(f"numShaves {b.numShaves} (4 OLMALI) — cihazda model yuklenmez")
    else:
        print("  ✅ 4 shave")
except ImportError:
    uyari.append("depthai kurulu degil — KONTROL 1 Jetson'da tekrarlanmali")
    print("  ⚠️  depthai yok, atlandi")
except Exception as e:
    hata.append(f"blob okunamadi: {e}")

print()
print("=" * 68)
print("KONTROL 2 — config.json'da sinif ISIMLERI var mi ve KANONIK mi")
print("=" * 68)
# 🔴 Node siniflari ISIMDEN cozuyor (duba_gecis_navigator.py:493-513 _sinif_indeksleri_coz):
#    "kenar" in ad -> KENAR, "engel" in ad -> ENGEL.
#    Isim yoksa/eslesmezse KENAR_CLASS=0, ENGEL_CLASS=1 SABITINE duser.
#    Roboflow export'u isim listesini ALFABETIK yaziyordu (engel<kenar) =>
#    turuncu<->sari SESSIZCE TAKAS -> gecitler yanlis dubalardan kurulur -> C2.
try:
    c = json.load(open(a.config))
    md = c["model"]["heads"][0]["metadata"]
    isim = md.get("classes")
    print(f"  classes  = {isim}")
    print(f"  n_classes= {md.get('n_classes')}")
    if not isim:
        hata.append("config.json'da sinif ISIMLERI YOK — node sabite duser, turuncu<->sari takas riski")
    elif list(isim) != KANONIK:
        hata.append(f"sinif sirasi {list(isim)} — KANONIK {KANONIK} olmali (0=kenar=TURUNCU)")
    else:
        print("  ✅ isimler var ve kanonik sirada")
    for ad in (isim or []):
        adl = ad.lower()
        if ("kenar" not in adl) and ("engel" not in adl):
            hata.append(f"'{ad}' ismi kodun aradigi 'kenar'/'engel' anahtarini icermiyor")
    if md.get("n_classes") != 2:
        hata.append(f"n_classes {md.get('n_classes')} (2 olmali)")
except Exception as e:
    hata.append(f"config.json okunamadi/beklenen yapida degil: {e}")

print()
print("=" * 68)
print("KONTROL 3 — KANAL SIRASI + PASSTHROUGH (cihazda yapilir)")
print("=" * 68)
# 🔴 Blob SIKISTIRMASI siniflari cokertebilir (mimariden bagimsiz risk).
#    ONNX'te cokme yoksa bile blob'da olabilir => cihazda kare alinip
#    ayni kare PC'de .pt ile kosturulur, 1:1 kiyaslanir.
print("""
  KANAL SIRASI — model RGB bekler (ultralytics img[::-1]), kamera BGR gonderir
     (duba_gecis_navigator.py:406). Ceviren olmazsa recall %96,8 -> %43,0 (olculdu).
  ✅ 10.08 UYGULANDI: blob '--reverse_input_channels' ILE derlendi
     (sha256 5726819a...31871b9c). Kamera BGR kalir => sartname Dosya-1 mp4'u
     dogru renkte. 🔴 (A) ve (B) BIRLIKTE UYGULANMAZ - cift cevirme geri alir.
  🚨 Saha kurtarma (internetsiz): blob yanlissa deploy'da setColorOrder(RGB).
  🔬 CIHAZDA DOGRULAMA: passthrough'dan kare al, ayni kareyi PC'de .pt ile kostur.
     Tespit sayisi yari yariya dusuyorsa KANAL SIRASI TERS demektir.
""")
if a.pt and a.kare:
    try:
        from ultralytics import YOLO
        r = YOLO(a.pt).predict(a.kare, imgsz=IMGSZ, conf=0.25, iou=0.7, verbose=False)[0]
        sayim = {}
        for cid in r.boxes.cls.tolist():
            sayim[int(cid)] = sayim.get(int(cid), 0) + 1
        print(f"  .pt sonucu ({os.path.basename(a.kare)}): {sayim}")
        print("  👉 AYNI kareyi cihazda blob ile kosturup bu sayimla KIYASLA.")
        print("     Turuncu duba 'engel' cikiyorsa blob sikistirmasi siniflari cokertmis.")
    except Exception as e:
        uyari.append(f".pt kosturulamadi: {e}")
else:
    uyari.append("KONTROL 3 yapilmadi — --pt ve --kare verilmedi (cihaz kaydi gerekiyor)")
    print("  ⚠️  atlandi: cihazdan passthrough karesi + .pt lazim")

print()
print("=" * 68)
for u in uyari: print(f"⚠️  {u}")
if hata:
    print(f"\n🔴 {len(hata)} HATA — BU BLOB SAHAYA CIKMAZ:")
    for h in hata: print(f"   • {h}")
    sys.exit(1)
print("✅ Otomatik kontroller GECTI." + (" (uyarilardaki adimlar elle tamamlanmali)" if uyari else ""))

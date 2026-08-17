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
print("KONTROL 0 — PROVENANS (blob'un NASIL derlendigi)")
print("=" * 68)
# 🔴 17.08.2026'da eklendi. O gune kadar bu betik "blob --reverse_input_channels
# ILE derlendi" cumlesini SABIT METIN olarak basiyordu; dagitilan blob'da
# --scale_values EKSIK oldugu halde test GECTI. Derleme parametreleri blob'un
# ciktisindan OKUNAMIYOR (shave/giris metadata'si ve config.json bilinen IYI
# blob'da da ayni gorunuyor) ⇒ tek yol uretim anindaki kaydi denetlemek.
# Kayit `model_uret.sh` tarafindan otomatik yaziliyor: elle kopyalanmis,
# kaynagi belirsiz bir blob bu kontrolu GECEMEZ.
ZORUNLU_BAYRAK = ["--scale_values", "--mean_values", "--reverse_input_channels"]
prov_yolu = os.path.join(os.path.dirname(os.path.abspath(a.blob)), "PROVENANS.json")
prov = None
try:
    import hashlib
    prov = json.load(open(prov_yolu))
    h = hashlib.sha256()
    with open(a.blob, "rb") as f:
        for parca in iter(lambda: f.read(1 << 20), b""):
            h.update(parca)
    blob_sha = h.hexdigest()
    print(f"  blob sha256 = {blob_sha[:16]}…")
    print(f"  kayit       = {str(prov.get('blob_sha256'))[:16]}…")
    if blob_sha != prov.get("blob_sha256"):
        hata.append(
            "blob sha256'si PROVENANS.json ile TUTMUYOR — bu blob'un nasil "
            "derlendigi BILINMIYOR (kayitsiz blob sahaya cikmaz)")
    else:
        print("  ✅ blob, kayitli derlemenin ta kendisi")
    eksik = [b for b in ZORUNLU_BAYRAK
             if not any(b in p for p in prov.get("optimizer_params", []))]
    print(f"  optimizer_params = {prov.get('optimizer_params')}")
    if eksik:
        hata.append(f"derlemede ZORUNLU bayrak(lar) yok: {eksik} — "
                    "olcek dusesse aga 0..255 girer (karede 300 uydurma tespit), "
                    "kanal takasi dusesse recall %96,8 -> %43")
    else:
        print("  ✅ ucu de derlemede var (olcek + mean + kanal takasi)")
except FileNotFoundError:
    hata.append(f"PROVENANS.json YOK ({prov_yolu}) — blob'un derleme kaydi "
                "olmadan sahaya cikilmaz; `scripts/model_uret.sh` onu uretir")
except Exception as e:
    hata.append(f"PROVENANS.json okunamadi: {e}")

print()
print("=" * 68)
print("KONTROL 1 — numShaves == 4 + giris boyutu deploy ile AYNI")
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
    # 🔴 Giris boyutu blob'un KENDI metadata'sindan okunuyor (metin degil).
    # preview 512 ↔ blob 416 olursa node'un kendi uyarisiyla "cop tespit"
    # cikar ve hicbir hata basilmaz. IMGSZ deploy NN_GIRIS ile ayni tutulur.
    for ad, t in b.networkInputs.items():
        d = list(t.dims)
        print(f"  giris '{ad}' = {d}")
        if IMGSZ not in d:
            hata.append(f"blob girisi {d}, deploy NN_GIRIS={IMGSZ} — "
                        "olcek uyumsuzlugu, tespitler cop olur")
        if prov and prov.get("nn_giris") not in (None, IMGSZ):
            hata.append(f"PROVENANS nn_giris={prov.get('nn_giris')} ↔ "
                        f"deploy NN_GIRIS={IMGSZ} — biri bayat")
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
  ⚠️ BU BASLIK ARTIK IDDIA ICERMIYOR: bayragin uygulanip uygulanmadigi
     KONTROL 0'da (PROVENANS.json) DENETLENIYOR. Eskiden burada "10.08
     UYGULANDI" diye SABIT METIN vardi ve 17.08'de dagitilan bozuk blob bu
     testten gecti — cumle blob'a degil, gecmise bakiyordu.
  🔬 Burada kalan is BASKA: blob SIKISTIRMASI siniflari cokertiyor mu
     (turuncu <-> sari). Bunu yalniz cihaz karesi gosterir.
  🚨 Saha kurtarmasi (internetsiz) `setColorOrder(RGB)` YALNIZ provenans
     'reverse yok' diyorsa uygulanir — bayrak zaten varken uygulanirsa CIFT
     CEVIRME olur ve recall %43'e duser.
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

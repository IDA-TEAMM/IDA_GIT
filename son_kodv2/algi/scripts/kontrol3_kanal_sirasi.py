#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KONTROL 3 — blob'un KANAL SIRASI cihazda doğru mu? (Jetson'da koşar)

🔴 NEDEN PAZARLIKSIZ: model **RGB** bekler, kamera **BGR** gönderir, blob'un
içinde çeviren yoktur — bu yüzden export `--reverse_input_channels` ile yapılır
(ilk konvolüsyon ağırlığı derleme anında takas edilir, çalışma maliyeti 0 ms).
Bu takas eksik ya da FAZLA yapılırsa **ölçüldü: recall %96,8 → %43,0** ve
**hiçbir hata basılmaz** — node açılır, FPS normaldir, tespitler gelir, yalnız
çoğu duba görünmez olur. Sahada teşhis edilemez.

## Yöntem — neden cihazda `.pt` GEREKMİYOR
Aynı kareyi cihazdaki NN'e **iki kez** veririz: (a) kameranın verdiği gibi,
(b) R ve B kanalları **takas edilmiş** hâlde. Doğru derlenmiş bir blob için
(a) tespit üretir, (b) neredeyse hiç üretmez. Ters çıkarsa kanal sırası
yanlıştır. 10.08'de 416 modelinde bu desen ölçülmüştü: kanallar ters
çevrilince **8/8 karede hiç tespit yok**.

Bu, Jetson'da torch/ultralytics İSTEMEZ — yalnız depthai.

## 🪤 KURU ORTAM TUZAĞI
*"Modeli kuru ortamda test etmek anlamsız"* (10.08 kuralı: suda %95,98, masada
kararsız). Bu yüzden **tespit çıkmaması tek başına BAŞARISIZLIK sayılmaz** —
kadrajda duba yoksa iki taraf da sıfır çıkar ve betik **KARARSIZ** der, "KALDI"
demez. Karara varmak için kadrajda gerçek duba (ya da turuncu/sarı bir cisim)
bulunmalı.

## Koşum
    # Jetson, kamera takılı, algı servisi DURDURULMUŞ (tek OAK):
    sudo systemctl stop girdap-algi
    python3 scripts/kontrol3_kanal_sirasi.py --kare 8 --kaydet /tmp/kontrol3

    # (istege bagli) PC'de 1:1 .pt kiyasi — kaydedilen kareler tasinir:
    python3 scripts/kontrol3_kanal_sirasi.py --pt ~/girdap_MODEL_512/girdap_512_ep87.pt \
                                             --kareler /tmp/kontrol3

Çıkış kodu 0 = GEÇTİ · 1 = KALDI (kanallar ters) · 2 = KARARSIZ (yeniden koş).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_KOK, "girdap_ida_algi"))

GECTI, KALDI, KARARSIZ = 0, 1, 2

# Takas tarafı bu orandan fazla tespit üretirse kanal sırası şüpheli.
# 10.08 ölçümünde takas tarafı 0/8 kare üretmişti; %25 bol pay bırakır.
TAKAS_TOLERANS = 0.25
# Karar verebilmek için normal tarafta gereken en az tespit (kuru ortam tuzağı).
MIN_TESPIT = 3


def _duz(kare: np.ndarray) -> np.ndarray:
    """HWC (interleaved) -> CHW (planar) — depthai NN girişi planar bekler."""
    return kare.transpose(2, 0, 1).flatten()


def _kareleri_topla(sayi: int, zaman_asimi: float):
    """GERÇEK dağıtım boru hattını koştur; NN giriş karelerini ve cihazın
    kendi tespitlerini topla.

    `pipeline_kur()` DOĞRUDAN kullanılıyor — ikinci bir boru hattı kopyası
    yazmak, tam da ölçmek istediğimiz şeyi (deploy'un gördüğü kare) sahteleştirir.
    """
    import time

    from girdap_ida_algi import duba_gecis_navigator as dgn

    if not dgn.KAYIT_AKTIF:
        # passthrough çıkışı yalnız KAYIT_AKTIF iken bağlanıyor; NN giriş
        # karesini oradan alıyoruz.
        dgn.KAYIT_AKTIF = True

    # 🔴 `pipeline_kur()` boru hattını KURAR **ve cihazı AÇAR** — dönüşü
    # `(dev, det_q, rgb_q, siniflar)`, `dai.Pipeline` DEĞİL (bkz.
    # duba_gecis_navigator:551-555; USB2 zorlaması + `dayanikli_ac` orada).
    # 🪤 12.08: burada dönüş bir pipeline sanılıp `dai.Device(...)` ile İKİNCİ
    # kez açılmaya çalışılıyordu ⇒ betik `TypeError` ile daha ilk adımda
    # düşüyordu, yani KONTROL 3 cihazda HİÇ koşmamıştı. Karar mantığının
    # testleri yeşildi (saf fonksiyon), donanım yolunu tutan yoktu —
    # `test_kontrol3_kareleri_topla.py` bu sözleşmeyi artık kilitliyor.
    dev, q_det, q_rgb, _siniflar = dgn.pipeline_kur()
    kareler, cihaz_tespit = [], []

    with dev:
        # 🪤 `q.get()` bloke eder: kamera hiç kare vermezse betik SONSUZA KADAR
        # asılırdı ve sahada "çalışıyor mu, dondu mu" ayırt edilemezdi.
        bitis = time.monotonic() + zaman_asimi
        while len(kareler) < sayi:
            if time.monotonic() > bitis:
                raise TimeoutError(
                    f"{zaman_asimi:.0f} sn içinde yalnız {len(kareler)}/{sayi} kare "
                    "geldi — kamera akışı yok. `girdap-algi` servisi hâlâ açık olabilir "
                    "(tek OAK'ı iki süreç açamaz): systemctl stop girdap-algi"
                )
            paket = q_rgb.tryGet()
            if paket is None:
                time.sleep(0.01)
                continue
            det = q_det.tryGet()
            kare = paket.getCvFrame()
            if kare is None or kare.size == 0:
                continue
            kareler.append(kare)
            cihaz_tespit.append(0 if det is None else len(det.detections))
    return kareler, cihaz_tespit


def _tekrar_oynat(kareler, takas: bool) -> int:
    """Aynı kareleri cihazın NN'ine XLinkIn ile geri ver; toplam tespit döndür.

    `takas=True` iken R↔B çevrilir = blob'un beklediğinin TERSİ kanal sırası.
    """
    import depthai as dai

    from girdap_ida_algi import duba_gecis_navigator as dgn

    siniflar = dgn._model_siniflarini_oku(dgn.MODEL_BLOB)

    pipeline = dai.Pipeline()
    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("giris")
    xin.setMaxDataSize(dgn.NN_GIRIS * dgn.NN_GIRIS * 3)

    # Dağıtımdaki YoloSpatialDetectionNetwork'ün derinliksiz eşleniği: aynı
    # blob, aynı eşikler. Derinlik burada gereksiz — kanal sırası bir GÖRÜNTÜ
    # sorusu, stereo ile ilgisi yok.
    nn = pipeline.create(dai.node.YoloDetectionNetwork)
    nn.setBlobPath(dgn.MODEL_BLOB)
    nn.setConfidenceThreshold(dgn.CONF_ESIK)
    nn.setNumClasses(len(siniflar) if siniflar else 2)
    nn.setCoordinateSize(4)
    nn.setIouThreshold(0.5)
    # YOLOv11 anchor-FREE — setAnchors ÇAĞRILMAZ (deploy ile aynı).
    xin.out.link(nn.input)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("tespit")
    nn.out.link(xout.input)

    from girdap_ida_algi import oak_baglanti as ob

    toplam = 0
    # Burada da USB2 zorlaması ŞART — replay ayrı bir cihaz açılışıdır ve
    # SuperSpeed kilidi bu açılışta da yaşanır.
    with ob.dayanikli_ac(lambda: dai.Device(pipeline, dai.UsbSpeed.HIGH)) as dev:
        q_in = dev.getInputQueue("giris")
        q_det = dev.getOutputQueue("tespit", maxSize=8, blocking=True)
        for kare in kareler:
            veri = kare[:, :, ::-1].copy() if takas else kare
            img = dai.ImgFrame()
            img.setData(_duz(veri))
            img.setType(dai.ImgFrame.Type.BGR888p)
            img.setWidth(dgn.NN_GIRIS)
            img.setHeight(dgn.NN_GIRIS)
            q_in.send(img)
            toplam += len(q_det.get().detections)
    return toplam


def _karar(normal: int, ters: int) -> tuple[int, str]:
    # 🔴 SIRA ÖNEMLİ — "kanallar ters" kontrolü, "yeterli tespit yok"tan ÖNCE.
    # Kanal sırası gerçekten ters olsaydı NORMAL taraf az, TAKAS tarafı çok
    # tespit üretirdi; önce `normal < MIN_TESPIT` bakılsaydı bu tablo
    # "KARARSIZ — dubayı göster, tekrar koş" diye okunurdu ve operatör aynı
    # sonucu alıp durur, gerçek arıza MASKELENİRDİ.
    if ters > max(normal, MIN_TESPIT):
        return KALDI, (
            f"TAKAS edilmiş kareler {ters} tespit, normal {normal} — model "
            "yalnız TERS kanal sırasında görüyor. Blob yanlış derlenmiş: "
            "`--reverse_input_channels` ters uygulanmış. 🔴 DAĞITMA."
        )
    if normal < MIN_TESPIT:
        return KARARSIZ, (
            f"normal tarafta yalnız {normal} tespit (en az {MIN_TESPIT} gerek) — "
            "kadrajda duba YOK ya da çok uzak. Bu KALDI DEĞİL: kuru ortamda model "
            "kararsızdır (10.08 kuralı). Kameraya duba göster ve TEKRAR KOŞ."
        )
    if ters > normal * TAKAS_TOLERANS:
        return KALDI, (
            f"kanalları TAKAS edilmiş kareler {ters} tespit üretti (normal {normal}) — "
            f"beklenen ≈0. Blob'un kanal sırası YANLIŞ: export'ta "
            "`--reverse_input_channels` ya eksik ya fazla. "
            "🔴 DAĞITMA — recall %96,8 → %43 düşer ve hiçbir hata basılmaz."
        )
    return GECTI, (
        f"normal {normal} tespit ↔ takas {ters} — model yalnız doğru kanal "
        "sırasında görüyor. Blob cihazda DOĞRU decode ediliyor."
    )


def _pc_kiyas(pt_yolu: str, klasor: str) -> int:
    """PC tarafı: kaydedilmiş kareleri `.pt` ile koştur, cihazla 1:1 kıyasla."""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("🔴 ultralytics yok — bu kip yalnız PC içindir (Jetson'da --kaydet kullan)")
        return KARARSIZ

    import cv2

    with open(os.path.join(klasor, "cihaz.json"), encoding="utf-8") as f:
        cihaz = json.load(f)

    model = YOLO(pt_yolu)
    satirlar = []
    for ad, cihaz_n in sorted(cihaz["kare_tespit"].items()):
        kare = cv2.imread(os.path.join(klasor, ad))
        r = model.predict(kare, imgsz=cihaz["nn_giris"], conf=cihaz["conf_esik"],
                          verbose=False)[0]
        satirlar.append((ad, cihaz_n, len(r.boxes)))

    print("\nkare                  cihaz   .pt")
    for ad, c, p in satirlar:
        isaret = "✅" if abs(c - p) <= max(1, 0.34 * max(c, p)) else "🔴"
        print(f"  {ad:<18} {c:>5} {p:>5}  {isaret}")

    c_top = sum(c for _, c, _ in satirlar)
    p_top = sum(p for _, _, p in satirlar)
    print(f"\ntoplam: cihaz {c_top} ↔ .pt {p_top}")
    if p_top and c_top < p_top * 0.6:
        print("🔴 cihaz .pt'nin çok altında — blob decode'u ya da kanal sırası şüpheli")
        return KALDI
    print("✅ cihaz ↔ .pt tutarlı")
    return GECTI


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--kare", type=int, default=8, help="toplanacak kare sayısı")
    ap.add_argument("--kaydet", help="kareleri + cihaz tespitlerini bu klasöre yaz")
    ap.add_argument("--zaman-asimi", type=float, default=30.0)
    ap.add_argument("--pt", help="PC kipi: .pt ile 1:1 kıyas")
    ap.add_argument("--kareler", help="PC kipi: --kaydet ile üretilmiş klasör")
    a = ap.parse_args()

    if a.pt:
        if not a.kareler:
            print("🔴 --pt ile birlikte --kareler gerekir")
            return KARARSIZ
        return _pc_kiyas(a.pt, a.kareler)

    print(f"KONTROL 3 — kanal sırası · {a.kare} kare toplanıyor…")
    kareler, cihaz_tespit = _kareleri_topla(a.kare, a.zaman_asimi)
    print(f"  toplandı: {len(kareler)} kare · cihazın canlı tespiti: "
          f"{sum(cihaz_tespit)} kutu")

    print("  tekrar oynatılıyor: NORMAL kanal sırası…")
    normal = _tekrar_oynat(kareler, takas=False)
    print(f"    -> {normal} tespit")
    print("  tekrar oynatılıyor: TAKAS EDİLMİŞ (R↔B) kanal sırası…")
    ters = _tekrar_oynat(kareler, takas=True)
    print(f"    -> {ters} tespit")

    if a.kaydet:
        import cv2

        os.makedirs(a.kaydet, exist_ok=True)
        from girdap_ida_algi import duba_gecis_navigator as dgn

        kayit = {}
        for i, (kare, n) in enumerate(zip(kareler, cihaz_tespit)):
            ad = f"kare_{i:02d}.png"          # 🔴 PNG: jpeg pikselleri DEĞİŞTİRİR
            cv2.imwrite(os.path.join(a.kaydet, ad), kare)
            kayit[ad] = n
        with open(os.path.join(a.kaydet, "cihaz.json"), "w", encoding="utf-8") as f:
            json.dump({"kare_tespit": kayit, "nn_giris": dgn.NN_GIRIS,
                       "conf_esik": dgn.CONF_ESIK, "blob": dgn.MODEL_BLOB,
                       "normal": normal, "takas": ters}, f, indent=2)
        print(f"  kaydedildi -> {a.kaydet} (PC'de --pt ile kıyaslanabilir)")

    kod, mesaj = _karar(normal, ters)
    print()
    print({GECTI: "✅ KONTROL 3 GEÇTİ", KALDI: "🔴 KONTROL 3 KALDI",
           KARARSIZ: "⚠️  KONTROL 3 KARARSIZ"}[kod])
    print(f"   {mesaj}")
    return kod


if __name__ == "__main__":
    sys.exit(main())

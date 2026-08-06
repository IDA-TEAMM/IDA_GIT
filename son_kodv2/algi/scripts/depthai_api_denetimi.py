#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARAÇ 6 — DEPTHAI API DENETİMİ (v2/v3 taşımalarında "kaçak çağrı" avcısı).

Neden bu araç var
-----------------
2026-08-05'te veri seti toplayıcısı v3 → v2'ye taşındı. Taşıma grep ile
yapıldı ve **bir çağrı kaçtı**: `pipeline.isRunning()` (v3'te var, v2'de yok).
Sonuç: servis açıldı, cihazı buldu, USB linkini kurdu, VPU sıcaklığını okudu —
ve tam kare döngüsüne girerken `AttributeError` ile çöktü. `Restart=on-failure`
yüzünden döngüye girdi; her denemede kamerayı açıp kapattı (cihaz teardown'da
zaten çöküyor). Denizde bu, oturumun sessizce kaybı demekti.

Ders: **grep "hangi çağrıyı ARADIĞINI" bilir, hangisini KAÇIRDIĞINI bilmez.**
Bu araç tersini yapar: kaynaktaki depthai çağrılarının HEPSİNİ AST ile çıkarır
ve KURULU paketin gerçek imzalarına karşı doğrular ([[arastirma-oncelikli]]:
birinci kaynak = kurulu paketin kendisi, ezber değil).

Kullanım
--------
    python3 scripts/depthai_api_denetimi.py scripts/oak_veriseti_topla.py
    python3 scripts/depthai_api_denetimi.py girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py

Çıkış kodu: 0 = temiz, 1 = eksik çağrı var (CI'da kullanılabilir).

⚠️ SINIRI: yalnız `degisken.metot` biçimindeki çağrıları görür ve değişken →
depthai tipi eşlemesi aşağıdaki tablodan gelir. Tabloda olmayan değişken
ATLANIR (rapor eder). Yani bu araç "yeşil" derse hata olmadığını değil,
BİLİNEN nesnelerdeki tüm çağrıların var olduğunu söyler. Gerçek kamerayla
smoke test yerine GEÇMEZ.
"""
import ast
import sys
from pathlib import Path

import depthai as dai

# Değişken adı → depthai tipi. Taşıma yaparken buraya EKLE; bilinmeyen değişken
# sessizce atlanmasın diye rapor ediliyor.
TIP_TABLOSU = {
    "dev": "Device",
    "device": "Device",
    "pipeline": "Pipeline",
    "cam": "node.ColorCamera",
    "cam_rgb": "node.ColorCamera",
    "mono_sol": "node.MonoCamera",
    "mono_sag": "node.MonoCamera",
    "stereo": "node.StereoDepth",
    "sdn": "node.YoloSpatialDetectionNetwork",
    "nn": "node.YoloSpatialDetectionNetwork",
    "xout": "node.XLinkOut",
    "rgb_q": "DataOutputQueue",
    "det_q": "DataOutputQueue",
    # ⚠️ "msg" BİLEREK YOK: ROS node'larında msg aynı zamanda Odometry/String/
    # Twist... — jenerik ada tip atamak 8 sahte alarm üretti (05.08, navigatör
    # denetimi). Tespit mesajı alanları (detections/xmin/spatialCoordinates)
    # ayrıca elle doğrulandı: SpatialImgDetections/SpatialImgDetection v2'de tam.
}


def _tip_coz(yol: str):
    """'node.ColorCamera' → dai.node.ColorCamera (yoksa None)."""
    hedef = dai
    for parca in yol.split("."):
        hedef = getattr(hedef, parca, None)
        if hedef is None:
            return None
    return hedef


def denetle(kaynak: Path) -> int:
    agac = ast.parse(kaynak.read_text(encoding="utf-8"))
    bulunan, bilinmeyen = {}, set()

    for dugum in ast.walk(agac):
        if isinstance(dugum, ast.Attribute) and isinstance(dugum.value, ast.Name):
            ad = dugum.value.id
            if ad in TIP_TABLOSU:
                bulunan.setdefault(ad, set()).add((dugum.attr, dugum.lineno))
            elif ad in ("dai", "depthai"):
                bulunan.setdefault("__modul__", set()).add((dugum.attr, dugum.lineno))
            else:
                bilinmeyen.add(ad)

    print(f"=== {kaynak.name} — depthai {dai.__version__} denetimi ===")
    eksik = 0
    for degisken in sorted(bulunan):
        tip = dai if degisken == "__modul__" else _tip_coz(TIP_TABLOSU[degisken])
        etiket = "dai" if degisken == "__modul__" else TIP_TABLOSU[degisken]
        if tip is None:
            print(f"  🔴 {etiket}: TİPİN KENDİSİ bu sürümde YOK "
                  f"(değişken '{degisken}')")
            eksik += 1
            continue
        for attr, satir in sorted(bulunan[degisken], key=lambda x: x[1]):
            if hasattr(tip, attr):
                print(f"  ✅ satır {satir:5d}: {degisken}.{attr}")
            else:
                print(f"  🔴 satır {satir:5d}: {degisken}.{attr}  →  {etiket}'te YOK")
                eksik += 1

    if bilinmeyen:
        print(f"\n  ⓘ tabloda olmayan (ATLANDI): {', '.join(sorted(bilinmeyen)[:15])}")
        print("    depthai nesnesiyse TIP_TABLOSU'na ekle, yoksa görmezden gel.")

    print(f"\nSONUÇ: {eksik} eksik çağrı" if eksik else "\nSONUÇ: TEMİZ")
    return 1 if eksik else 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(denetle(Path(sys.argv[1])))

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Üretilen RVC2 blob'unu Jetson'a taşımadan ÖNCE doğrula (PC'de koşar).

Neden pazarlıksız: yanlış blob teknede DÜZELTİLEMEZ. Jetson'daki depthai
2.30.0.0'da SuperBlob API'si yok (shave'i sonradan seçemezsin) ve yeniden
dönüşüm internet ister — yarışma alanında ikisi de yok (md 4.1).
Rehber: `docs/hubai_model_rehberi.md` §4.

Koşum:
    python3 scripts/model_dogrula.py /yol/yolo11n_duba_rvc2.blob

Çıkış kodu 0 = TESLİME UYGUN, 1 = DUR (mesajda hangi kısıt düştüğü yazar).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

# docs/hubai_model_rehberi.md §0 — ölçülmüş/doğrulanmış kısıtlar. Ayar değil.
BEKLENEN_SHAVE = 4          # deploy boru hattında NN'e kalan bütçe (05.08 ölçümü)
BEKLENEN_GIRIS = 416        # duba_gecis_navigator.NN_GIRIS ile BİRLİKTE değişir
BEKLENEN_SINIF_SAYISI = 2   # kenar_dubasi + engel_dubasi


def _sha256(yol: str) -> str:
    h = hashlib.sha256()
    with open(yol, "rb") as f:
        for blok in iter(lambda: f.read(1 << 20), b""):
            h.update(blok)
    return h.hexdigest()


def _config_yolu(blob_yolu: str) -> str | None:
    """Algı node'unun aradığı iki yer (`_model_siniflarini_oku` ile AYNI sıra)."""
    for aday in (
        os.path.join(os.path.dirname(blob_yolu), "config.json"),
        os.path.splitext(blob_yolu)[0] + ".json",
    ):
        if os.path.exists(aday):
            return aday
    return None


def blob_denetle(blob_yolu: str) -> list[str]:
    """numShaves + giriş boyutu. Cihaz GEREKMEZ (blob başlığından okunur)."""
    hatalar: list[str] = []
    try:
        import depthai as dai
    except ImportError:
        return ["depthai kurulu değil → blob başlığı OKUNAMADI "
                "(pip install depthai; bu denetim atlanamaz)"]

    blob = dai.OpenVINO.Blob(blob_yolu)
    print(f"  shave     : {blob.numShaves} (slice {blob.numSlices})")
    if blob.numShaves != BEKLENEN_SHAVE:
        hatalar.append(
            f"shave {blob.numShaves} ≠ {BEKLENEN_SHAVE} → cihazda "
            f"'compiled for {blob.numShaves} shaves, only 4 available' ile "
            "YÜKLENMEZ (05.08 ölçümü). Dönüşümü shaves=4 ile TEKRARLA."
        )
    for ad, girdi in blob.networkInputs.items():
        boyut = list(girdi.dims)
        print(f"  giriş     : {ad} {boyut}")
        if BEKLENEN_GIRIS not in boyut:
            hatalar.append(
                f"giriş {boyut} içinde {BEKLENEN_GIRIS} yok → "
                "duba_gecis_navigator.NN_GIRIS ile uyuşmuyor "
                "(ikisi BİRLİKTE değişir)"
            )
    return hatalar


def config_denetle(cfg_yolu: str) -> list[str]:
    """Sınıf isimleri — algı node'u sınıfı İSİMDEN çözer, indeksten DEĞİL."""
    hatalar: list[str] = []
    with open(cfg_yolu, encoding="utf-8") as f:
        cfg = json.load(f)
    heads = cfg.get("model", {}).get("heads", [])
    meta = heads[0].get("metadata", {}) if heads else {}
    siniflar = [str(s) for s in meta.get("classes", [])]
    print(f"  sınıflar  : {siniflar}")
    if not siniflar:
        return ["config.json'da sınıf ismi YOK → node yedek sabitlere düşer, "
                "turuncu/sarı yer değiştirebilir (sessiz puan kaybı)"]
    if len(siniflar) != BEKLENEN_SINIF_SAYISI:
        hatalar.append(
            f"{len(siniflar)} sınıf var, {BEKLENEN_SINIF_SAYISI} bekleniyordu"
            + (" — 80 sınıf = stok COCO modeli çevrilmiş, YANLIŞ DOSYA"
               if len(siniflar) == 80 else "")
        )
    # Node'un sözleşmesi (`_sinif_indeksleri_coz`): isimde 'kenar'/'engel'
    # alt dizgisi aranır. Burada ALGORİTMA KOPYALANMIYOR, o fonksiyonun
    # ÇALIŞABİLMESİ İÇİN GEREKEN KOŞUL doğrulanıyor.
    kenar = [i for i, ad in enumerate(siniflar) if "kenar" in ad.lower()]
    engel = [i for i, ad in enumerate(siniflar) if "engel" in ad.lower()]
    if len(kenar) != 1 or len(engel) != 1 or kenar == engel:
        hatalar.append(
            f"isimlerden sınıf çözülemez (kenar eşleşmesi {kenar}, engel "
            f"{engel}) → node yedek sabitlere düşer. İsimleri data.yaml'daki "
            "gibi bırak: 'kenar_dubasi', 'engel_dubasi' "
            "(docs/hubai_model_rehberi.md §1)"
        )
    else:
        print(f"  çözüm     : kenar={kenar[0]} ('{siniflar[kenar[0]]}'), "
              f"engel={engel[0]} ('{siniflar[engel[0]]}') — İSİMDEN")
    # Node'un GERÇEK fonksiyonu import edilebiliyorsa onunla da doğrula
    # (ikinci kopya üretmemek için: sonuç yalnız RAPORLANIR).
    try:
        sys.path.insert(
            0, os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "girdap_ida_algi")
        )
        from girdap_ida_algi.duba_gecis_navigator import (  # noqa: E402
            _sinif_indeksleri_coz,
        )
        k, e, isimle = _sinif_indeksleri_coz(siniflar)
        print(f"  node teyit: kenar={k}, engel={e}, isimle={isimle}")
        if not isimle:
            hatalar.append("node'un kendi çözümü YEDEK SABİTLERE düştü")
    except Exception as exc:                     # depthai/rclpy yoksa normal
        print(f"  node teyit: atlandı ({type(exc).__name__}) — yukarıdaki "
              "isim denetimi geçerli")
    return hatalar


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("blob", help="yolo11n_duba_rvc2.blob yolu")
    a = ap.parse_args()

    if not os.path.exists(a.blob):
        print(f"HATA: blob yok: {a.blob}")
        return 1

    print(f"blob        : {a.blob}")
    print(f"  boyut     : {os.path.getsize(a.blob)} B")
    print(f"  sha256    : {_sha256(a.blob)}")
    hatalar = blob_denetle(a.blob)

    cfg = _config_yolu(a.blob)
    if cfg is None:
        hatalar.append(
            "config.json blob'un YANINDA yok → sınıf isimleri okunamaz "
            "(NNArchive tar.xz'den çıkar: tar -xJf <ad>.tar.xz config.json)"
        )
    else:
        print(f"config.json : {cfg}")
        hatalar += config_denetle(cfg)

    print()
    if hatalar:
        print("🔴 DUR — teslime uygun DEĞİL:")
        for h in hatalar:
            print(f"  · {h}")
        return 1
    print("✅ TESLİME UYGUN — 4 shave · 416×416 · sınıflar isimden çözülüyor.")
    print("   Sıra: blob + config.json → /home/girdap/models/ (USB ile), "
          "sonra scripts/duba_kamera_test.py ile masa teyidi.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

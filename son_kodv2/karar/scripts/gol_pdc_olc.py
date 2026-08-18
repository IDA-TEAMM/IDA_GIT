#!/usr/bin/env python3
"""Sanal göl izini `ParkurSiniri` ile ölç → tek satır özet (seri koşum için).

Kullanım: pdc_olc.py <iz.csv> [etiket] [kapi_sayisi] [acik_m] [aralik_m]
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "IDA_GIT" / "son_kodv2" / "karar"))
from prototype.mission.parkur_siniri import (          # noqa: E402
    DISARIDA, ICERIDE, KAPSAM_DISI, ParkurDisiSayaci, ParkurSiniri,
)

yol = sys.argv[1]
etiket = sys.argv[2] if len(sys.argv) > 2 else "-"
N = int(sys.argv[3]) if len(sys.argv) > 3 else 8
ACIK = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
ARALIK = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0

# 🔑 GEOMETRİ TEK KAYNAKTAN: `sanal_gol.py` koşumda gerçek parkuru
# `~/girdap_logs/gol/parkur.json` künyesine yazar. Burada YENİDEN KURMAK
# ayrışma demektir — kapı genişliği artık şartname bandından (8-12 m)
# kapı başına çekiliyor, sabit bir sayıyla yeniden üretilemez.
import json                                                     # noqa: E402
import os                                                       # noqa: E402
_KUNYE = os.path.expanduser("~/girdap_logs/gol/parkur.json")
kapilar = []
if os.path.exists(_KUNYE):
    with open(_KUNYE, encoding="utf-8") as _f:
        for k in json.load(_f)["kapilar"]:
            h = k["genislik_m"] / 2.0
            kapilar.append(((k["x"] - h, k["y"]), (k["x"] + h, k["y"])))
else:
    # Künye yoksa eski yeniden-kurulum (yalnız SABİT genişlikli koşumlar için
    # doğru). Sessizce yanlış ölçmemek için uyarır.
    print("⚠ parkur.json YOK — geometri yeniden kuruluyor; kapı genişliği "
          "değişkense bu ölçüm YANLIŞ olur.", file=sys.stderr)
    DESEN = [0.0, 5.0, 0.0, -5.0]
    for i in range(N):
        gx, gy, h = DESEN[i % 4], 6.0 + i * ARALIK, ACIK / 2.0
        kapilar.append(((gx - h, gy), (gx + h, gy)))

sinir = ParkurSiniri.kapilardan(kapilar)
sayac = ParkurDisiSayaci(sinir)

iz = list(csv.DictReader(open(yol)))
durumlar = {ICERIDE: 0, DISARIDA: 0, KAPSAM_DISI: 0}
for k in iz:
    d = sayac.adim((float(k["x"]), float(k["y"])), float(k["t"]))
    durumlar[d] = durumlar.get(d, 0) + 1
sure = float(iz[-1]["t"]) if iz else 0.0
sayac.bitir(sure)
son = (float(iz[-1]["x"]), float(iz[-1]["y"])) if iz else (0.0, 0.0)

print(f"{etiket};{len(iz)};{sure:.1f};{son[0]:.2f};{son[1]:.2f};"
      f"{durumlar[ICERIDE]};{durumlar[DISARIDA]};{durumlar[KAPSAM_DISI]};"
      f"{sayac.cikis_sayisi};{sayac.etkin_cikis};{sayac.toplam_sure:.1f};"
      f"{sayac.en_uzun_sure:.1f};{sayac.en_derin:.2f};"
      f"{sayac.puan(1):.1f};{sayac.puan(2):.1f}")

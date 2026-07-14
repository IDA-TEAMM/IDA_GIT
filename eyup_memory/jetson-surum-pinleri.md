---
name: jetson-surum-pinleri
description: "Jetson'daki KESİN sürüm yığını — numpy 1.26.4 + opencv 4.11.0.86 birlikte sabitlenir (4.12+ numpy2 dayatır); PC ile Jetson'un numpy'ı BİLEREK farklı"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 23462142-febe-4512-a8b9-bdaa8d90d7f8
---

Jetson gerçek kurulumunda (2026-07-11) sökülerek öğrenildi; her sürüm PyPI metadata'sından/canlı kurulumdan teyitli. İlgili: [[girdap-ida-proje-durumu]], [[girdap-decision-entegrasyon]], [[jetson-yuk-kod-sadeligi]].

## Jetson yığını (girdap@ubuntu — Orin Nano Super, JetPack r36.5, Ubuntu 22.04, Python 3.10, aarch64)

| Paket | Sürüm | Neden / kaynak |
|---|---|---|
| ROS 2 | **Humble** (ros-base) | tüm yığının tabanı; Jazzy KURMA |
| numpy | **== 1.26.4** | 🔴 <2 ZORUNLU — 2.x ROS apt paketlerinin/scipy'nin ABI'sini kırar (`_ARRAY_API`); 1.26.4 = 2 öncesi son 1.26 |
| opencv-python-headless | **== 4.11.0.86** | 🔴 numpy<2'ye izin veren SON sürüm; **4.12+, 4.13, 5.x HEPSİ numpy≥2 dayatır ve numpy'ı sessizce yükseltir** (Jetson'da iki kez yaşandı — `<5` pini YETMEZ) |
| depthai | 3.7.1 (şart `>=3.6`) | v3 API; masum — yalnız `numpy<3` ister |
| scipy | `>=1.11,<1.14` (1.13.x) | karar yığını pini |
| gtsam | 4.3a0 (`--pre`) | aarch64 cp310 wheel PyPI'da VAR (kaynak derleme YOK — eski F2.4 "wheel yok" endişesi bayat); `numpy>=1.11` ister, 1.26.4 uyumlu |
| matplotlib / pillow | `>=3.8` / güncel | Ekran-2 aracı |
| mavros + extras + msgs | apt `ros-humble-*` | + GeographicLib dataset scripti |
| ffmpeg | apt | Ekran-2 MP4 |
| cupy-cuda12x | **13.6.0 KURULDU** (2026-07-11, numpy pini aynı komutta) | MPPI CUDA Faz A+B bitti — step 9 ms |

## 🔑 ALTIN KURAL — pip'in numpy'ı geri yükseltme döngüsü

Jetson'da HERHANGİ bir pip kurulumunda numpy sabitini AYNI komuta ekle, yoksa çözücü bağımlılık olarak numpy 2.x'i geri çeker (iki kez yaşandı):

```bash
python3 -m pip install --user --force-reinstall opencv-python-headless==4.11.0.86 numpy==1.26.4
```

Teyit: `python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"` → `4.11.0 1.26.4`.
Bekçi: `jetson_kontrol.sh` numpy satırı. Rehber: girdap-ida-algi `docs/BURADAN_BASLA.md` §5 hata 5/8 (`810878b`).

## PC ≠ Jetson (bilerek farklı)

- **Dev PC (x86, /home/eyup):** numpy **2.2.6** + scipy 1.15.3 + matplotlib 3.10.9 (pip --user) + gtsam 4.3a0 — suite (246/1) burada numpy2 ile YEŞİL, çünkü scipy de pip'ten numpy2-uyumlu kuruldu.
- **Jetson:** numpy **1.26.4** ŞART — ROS Humble'ın apt'ten gelen derlenmiş Python paketleri numpy1 ABI'sinde.
- Yani "PC'de çalışıyor" ≠ "Jetson'da çalışır"; sürüm tartışmasında önce HANGİ makine olduğuna bak.

## ✅ Kapandı (2026-07-11 gece teyidi)

Suite Jetson'da numpy 1.26.4 + opencv 4.11.0.86 ile `246 passed / 1 skipped` verdi; sonraki düzeltmelerle (F-A.1, Faz B, F-L.1) güncel taban **250 passed / 2 gerekçeli skip** (skip olan 2 test GPU'lu makinede cupy'siz-fallback, GPU'suz makinede cupy-parite — her iki makine tipinde de 250 geçer). Kaynak: girdap-decision `docs/donanim_gunlugu_2026-07-12.md`.

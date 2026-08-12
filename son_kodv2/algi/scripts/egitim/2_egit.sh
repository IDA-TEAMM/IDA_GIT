#!/bin/bash
# FINAL EGITIM — ADIM 2
#
# Once 1_veri_hazirla.py kosulmus olmali (kareler IMGSZ x IMGSZ'ye EZILMIS olacak).
# 🔴 2026-08-12: IMGSZ 416 -> 512 (deploy NN_GIRIS ile BIRLIKTE degisir).
# Gerekcelerin tamami: ANA-EGITIM-NASIL.md
#
# 🔴 workers=2 PAZARLIKSIZ — 09.08'de workers=8 iki kosuda 18 isciye cikti,
#    ~22 GB istedi, 15 GB RAM + 0 swap => OOM => PC dondu, etiketleme kesildi.
# 🔴 Etiketleme ile egitim AYNI ANDA kosturulmaz.
set -euo pipefail

# 🔴 2026-08-12: varsayilan `veri/` -> `veri512_ton_full/`. `veri/` kareleri
#    416x416; IMGSZ 512 olunca yukaridaki koruma haklı olarak DUR derdi.
#    veri512_ton_full = 7.546 orijinal + 6.942 dikissiz ton kopyasi = 14.488
#    (ep87 modelini ureten set). Ton kopyasiz istenirse: VERI=...veri512/data.yaml
VERI="${VERI:-$HOME/girdap_EGITIM_HATTI/veri512_ton_full/data.yaml}"
EPOCHS="${EPOCHS:-300}"        # ust sinir; erken durdurma karar verir
PATIENCE="${PATIENCE:-50}"     # Ultralytics tavsiyesi
HSV_S="${HSV_S:-0.7}"
HSV_H="${HSV_H:-0.015}"       # 09.08 hue ablasyonu ile kesinlesecek          # 09.08 ablasyonu: 0.35 ile fark TOHUM GURULTUSU icinde
                               # => varsayilanda kalindi. Gerekce: ANA-EGITIM-NASIL.md
IMGSZ="${IMGSZ:-512}"           # blob ve kamera da AYNI olmali (NN_GIRIS,
                               # config.json). 12.08: 416 -> 512, gerekce
                               # menzil (20 m'de duba 4,5 px -> 5,6 px =
                               # recall %80 bandindan %90 bandina).
AD="${AD:-girdap_final_$(date +%Y%m%d_%H%M)}"

[ -f "$VERI" ] || { echo "🔴 data.yaml yok: $VERI  — once 1_veri_hazirla.py"; exit 1; }

# --- kareler gercekten IMGSZ x IMGSZ mi? (stretch adimi atlanmis olabilir)
# 🔴 Beklenen boyut IMGSZ'den TURETILIR — eskiden '416x416' diye SABIT yazilmisti
#    ve IMGSZ degisince bu koruma yeni egitimi haksiz yere DURDURUYORDU.
KOK="$(dirname "$VERI")"
# NOT: 'find ... | head -1' KULLANMA — head boruyu kapatinca find SIGPIPE alir,
# 'set -o pipefail' bunu hata sayar ve betik exit 141 ile SESSIZCE olur (09.08'de yasandi).
ORNEK="$(find "$KOK/train/images" -name '*.jpg' -print -quit)"
BOYUT="$(python3 -c "import cv2,sys;i=cv2.imread(sys.argv[1]);print(f'{i.shape[1]}x{i.shape[0]}')" "$ORNEK")"
if [ "$BOYUT" != "${IMGSZ}x${IMGSZ}" ]; then
  echo "🔴 DUR: egitim kareleri $BOYUT — ${IMGSZ}x${IMGSZ} OLMALI."
  echo "   Deploy kareyi EZIYOR (setPreviewKeepAspectRatio(False)), Ultralytics ise"
  echo "   LETTERBOX yapar. Ayrisirsa uzak dubada ~3,4 puan recall kaybi (09.08 olcumu)."
  echo "   Cozum: python3 1_veri_hazirla.py --boyut $IMGSZ"
  exit 1
fi
echo "✅ kare boyutu ${IMGSZ}x${IMGSZ} (deploy geometrisiyle ayni)"

# --- RAM bekcisi
BOS=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
[ "$BOS" -ge 2500 ] || { echo "🔴 bos RAM ${BOS} MB < 2500 MB — baslatilmiyor"; exit 1; }
echo "✅ bos RAM ${BOS} MB"

echo
echo "epochs=$EPOCHS  patience=$PATIENCE  imgsz=$IMGSZ  hsv_h=$HSV_H  hsv_s=$HSV_S  workers=2"
echo "cikti: runs/detect/$AD"
echo

yolo detect train \
  model=yolo11n.pt \
  data="$VERI" \
  imgsz=$IMGSZ \
  epochs=$EPOCHS \
  patience=$PATIENCE \
  batch=16 \
  device=0 \
  workers=2 \
  cache=False \
  seed=0 \
  hsv_s=$HSV_S \
  hsv_h=$HSV_H \
  name="$AD"

echo
echo "SONRAKI: blob uret (luxonis/tools --use-rvc2 + blobconverter shaves=4), sonra"
echo "  python3 3_kabul_testi.py --blob <blob> --config <config.json>"

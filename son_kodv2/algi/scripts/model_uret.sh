#!/bin/bash
# -*- coding: utf-8 -*-
#
# Eğitilmiş .pt → OAK-D Lite'ta koşan 4-shave RVC2 .blob + config.json.
# TEK KOMUT — çünkü bu zincirin üç adımında da varsayılanlar bizim için
# YANLIŞ (shave 8, superblob açık, giriş 640) ve elle yapıldığında en az bir
# tanesi unutuluyor. Rehber ve ölçümler: docs/hubai_model_rehberi.md.
#
#   ./scripts/model_uret.sh /yol/best.pt [cikti_dizini]
#
# Sonunda scripts/model_dogrula.py koşar: shave/giriş/sınıf isimleri
# doğrulanmadan hiçbir şey "hazır" sayılmaz (yanlış blob TEKNEDE düzeltilemez).
#
# ⚠ İNTERNET GEREKİR (blobconverter bulut derleyicisi). Yarışma alanında
#   internet YOK (md 4.1) → bu adım sahada DEĞİL, evde/atölyede yapılır.
set -euo pipefail

KIRMIZI=$'\e[31m'; YESIL=$'\e[32m'; SARI=$'\e[33m'; NORM=$'\e[0m'
hata() { echo "${KIRMIZI}HATA:${NORM} $*" >&2; exit 1; }
adim() { echo; echo "${YESIL}==>${NORM} $*"; }

[ $# -ge 1 ] || hata "kullanım: $0 <best.pt> [cikti_dizini]"
PT="$(readlink -f "$1")"
[ -f "$PT" ] || hata "model dosyası yok: $PT"
CIKTI="${2:-$HOME/girdap_models/$(date +%Y%m%d_%H%M)}"
ARACLAR="${GIRDAP_ARACLAR:-$HOME/girdap_model_araclari}"
BETIK_DIZINI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# docs/hubai_model_rehberi.md §0 — pazarlıksız kısıtlar (ölçülmüş).
SHAVE=4          # deploy boru hattında NN'e kalan bütçe (05.08 kamerada ölçüldü)
GIRIS=416        # duba_gecis_navigator.NN_GIRIS ile BİRLİKTE değişir
BLOB_ADI="yolo11n_duba_rvc2.blob"   # MODEL_BLOB ile aynı isim olmalı

mkdir -p "$CIKTI"
echo "model  : $PT ($(stat -c%s "$PT") B)"
echo "sha256 : $(sha256sum "$PT" | cut -d' ' -f1)"
echo "çıktı  : $CIKTI"

# ---------------------------------------------------------------- 1) araçlar
adim "1/5 luxonis/tools (YOLO kafa ameliyatı + ONNX) hazır mı"
VENV="$ARACLAR/tools/venv"
if [ ! -x "$VENV/bin/python" ]; then
  echo "  kurulu değil → kuruluyor (torch indirir, ~1 GB, birkaç dakika)"
  mkdir -p "$ARACLAR"
  [ -d "$ARACLAR/tools/.git" ] || \
    git clone --depth 1 --recursive https://github.com/luxonis/tools.git "$ARACLAR/tools"
  cd "$ARACLAR/tools"
  # ⚠ python3-venv (ensurepip) kurulu olmayabilir → --without-pip ile kur ve
  # sistem pip'inin --python bayrağıyla doldur. --system-site-packages sistemde
  # zaten kurulu torch'u yeniden indirmemek için (bu makinede 06.08'de
  # denendi: ensurepip yok, bu yol çalıştı).
  python3 -m venv --without-pip --system-site-packages venv
  python3 -m pip --python venv/bin/python install -q "numpy<2.1" onnx onnxruntime onnxsim
  PIP_CONSTRAINT=constraints.txt PIP_BUILD_CONSTRAINT=constraints.txt \
    python3 -m pip --python venv/bin/python install -q .
fi
"$VENV/bin/python" -c "import tools" 2>/dev/null || \
  hata "tools paketi import edilemiyor: $VENV"
echo "  ✓ $VENV"

# ------------------------------------------------------------------ 2) ONNX
adim "2/5 .pt → ONNX (+ NNArchive tar.xz, sınıf isimleri onun içinde)"
cd "$ARACLAR/tools"
"$VENV/bin/tools" "$PT" --imgsz "$GIRIS" --use-rvc2
ONNX="$(find "$ARACLAR/tools/shared_with_container/outputs" -name '*.onnx' \
        -newer "$PT" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
[ -n "${ONNX:-}" ] || ONNX="$(find "$ARACLAR/tools/shared_with_container/outputs" \
        -name '*.onnx' -printf '%T@ %p\n' | sort -rn | head -1 | cut -d' ' -f2-)"
[ -n "${ONNX:-}" ] || hata "ONNX üretilmedi (outputs/ boş)"
TARXZ="$(dirname "$ONNX")/$(basename "${ONNX%.onnx}").tar.xz"
echo "  ✓ $ONNX"

# --------------------------------------------------------------- 3) blob (4 shave)
adim "3/5 ONNX → $SHAVE shave .blob (blobconverter bulut derleyicisi)"
"$VENV/bin/python" - "$ONNX" "$CIKTI/$BLOB_ADI" "$SHAVE" <<'PY'
import shutil, sys
import blobconverter
onnx, hedef, shave = sys.argv[1], sys.argv[2], int(sys.argv[3])
yol = blobconverter.from_onnx(
    model=onnx,
    data_type="FP16",
    shaves=shave,                       # 🔴 varsayılan 8 — sahada YÜKLENMEZ
    version="2022.1",                   # RVC2 / depthai 2.30 uyumlu
    optimizer_params=["--scale_values=[255,255,255]", "--mean_values=[0,0,0]",
                      # 🔴 PAZARLIKSIZ — 17.08.2026'da EKSİK olduğu bulundu.
                      # Model RGB bekler (ultralytics `img[::-1]`), ColorCamera
                      # BGR gönderir (`setColorOrder(BGR)`), blob'un içinde
                      # çeviren YOKTUR. Bu bayrak olmadan derlenen blob sahada
                      # ÖLÇÜLDÜ: recall %96,8 → %43,0 ve **hiçbir hata basılmaz**
                      # — node açılır, FPS normaldir, tespitler gelir, yalnız
                      # dubaların çoğu görünmez olur.
                      # Bayrak `models/README.md:66` ve `scripts/egitim/OKU.md`'de
                      # "ZORUNLU" diye yazılıydı ama bu betiğe HİÇ girmemişti
                      # (`git log -S` boş döndü) ⇒ bu betikle üretilen HER blob
                      # kanal takassız çıkıyordu. `model_dogrula.py` de kanal
                      # sırasını denetlemiyor (shave/giriş/sınıf adına bakıyor)
                      # ⇒ zincirde yakalayan hiçbir kapı yoktu.
                      # ⚠️ Kamera tarafındaki (B) çözümü `setColorOrder(RGB)` ile
                      # BİRLİKTE UYGULANMAZ — çift çevirme takası geri alır.
                      "--reverse_input_channels"],
)
shutil.copy(yol, hedef)
print("blob:", hedef)
PY

# ------------------------------------------------------- 4) config.json (sınıflar)
adim "4/5 config.json (sınıf isimleri) blob'un YANINA"
if [ -f "$TARXZ" ]; then
  tar -xJf "$TARXZ" -C "$CIKTI" config.json 2>/dev/null || \
    tar -xJf "$TARXZ" -C "$CIKTI" --wildcards '*config.json' --strip-components=1 2>/dev/null || true
fi
[ -f "$CIKTI/config.json" ] || echo "${SARI}UYARI:${NORM} config.json çıkarılamadı — \
sınıf isimleri olmadan node YEDEK SABİTLERE düşer (turuncu/sarı yer değiştirebilir)"

# ------------------------------------------------------------- 5) doğrulama
adim "5/5 DOĞRULAMA (shave · giriş · sınıf isimleri)"
python3 "$BETIK_DIZINI/model_dogrula.py" "$CIKTI/$BLOB_ADI" || \
  hata "doğrulama DÜŞTÜ — bu blob'u tekneye TAŞIMA"

cat <<SON

${YESIL}HAZIR${NORM} → $CIKTI
Sıradaki adımlar (docs/hubai_model_rehberi.md §5):
  1) USB ile Jetson'a:  scp $CIKTI/{$BLOB_ADI,config.json} girdap@<jetson>:/home/girdap/models/
     (yarışma alanında WiFi YOK → USB)
  2) Masa teyidi:       python3 scripts/duba_kamera_test.py
     → "Model sınıf sırası: [...]" logunu OKU, turuncu dubaya 'kenar' demeli
  3) Sınıf çökmesi testi (docs §4b): passthrough karesini PC'de .pt ile karşılaştır
  4) ros2 launch girdap_ida_algi algi.launch.py → /perception/buoys akıyor mu
SON

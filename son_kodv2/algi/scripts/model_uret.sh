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
BLOB_ADI="yolo11n_duba_rvc2.blob"   # MODEL_BLOB ile aynı isim olmalı

# 🔴 GİRİŞ BOYUTU SABİT YAZILMAZ — deploy'un kendi değerinden TÜRETİLİR.
# Gerekçe (12.08 dersi, ölçülmüş): `2_egit.sh`'de "416x416" sabit yazılıydı ve
# 512'ye geçişte yeni eğitimi durduracaktı ⇒ **koruma, koruduğu değerden
# türetilmeli**. Aynı tuzak burada da vardı: 12.08'de `NN_GIRIS` 512 oldu, bu
# betikte `GIRIS=416` KALDI (17.08'de bulundu). Bu betikle üretilen blob 416
# olur, preview 512 gönderir ⇒ node'un kendi uyarısıyla "çöp tespit" —
# ve **hiçbir hata basılmaz**.
NAV_PY="$BETIK_DIZINI/../girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py"
[ -f "$NAV_PY" ] || hata "deploy kaynağı bulunamadı: $NAV_PY (GİRİŞ boyutu ondan okunuyor)"
GIRIS="$(grep -oP '^NN_GIRIS\s*=\s*\K[0-9]+' "$NAV_PY" | head -1)"
[ -n "$GIRIS" ] || hata "duba_gecis_navigator.py içinde NN_GIRIS okunamadı"
echo "giriş  : $GIRIS  (deploy NN_GIRIS'ten türetildi, sabit yazılmadı)"

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
"$VENV/bin/python" - "$ONNX" "$CIKTI/$BLOB_ADI" "$SHAVE" "$PT" "$GIRIS" <<'PY'
import hashlib, json, os, shutil, sys
import blobconverter
onnx, hedef, shave = sys.argv[1], sys.argv[2], int(sys.argv[3])
pt, giris = sys.argv[4], int(sys.argv[5])


def _sha(y):
    h = hashlib.sha256()
    with open(y, "rb") as f:
        for p in iter(lambda: f.read(1 << 20), b""):
            h.update(p)
    return h.hexdigest()


# 🔴 TEK KAYNAK: asagidaki liste hem derlemeye hem PROVENANS.json'a gider.
# Ayri yazilsalardi biri guncellenir, digeri unutulurdu — ve provenans "dogru
# derlendi" derken blob bozuk olurdu. 17.08.2026'da tam bu yasandi: belgeler
# bayragi ZORUNLU yaziyordu, derleme onu hic gecmiyordu.
params = [
    # 🔴 OLCEK — 0..255 -> 0..1. ONNX'te normalizasyon YOK (grafigin ilk
    # dugumu dogrudan Conv, olculdu) ve depthai 2.30 config.json'daki `scale`
    # alanini OKUMAZ (o NNArchive metadata'si). Bu bayrak duserse aga 0..255
    # girer; OLCULDU (17.08, ep135, 120 kare): 226 tespit -> 36.000 tespit =
    # max_det doyumu, medyan guven 0,906. Belirti "duba kacirma" DEGIL, her
    # karede yuzlerce uydurma kutu ⇒ /perception/buoys cop ⇒ P1+P2 = 0.
    "--scale_values=[255,255,255]",
    "--mean_values=[0,0,0]",
    # 🔴 KANAL TAKASI — model RGB bekler (ultralytics `img[::-1]`),
    # ColorCamera BGR gonderir (`setColorOrder(BGR)`), blob'un icinde ceviren
    # YOKTUR. Takas derleme aninda ilk konvolusyon agirliklarina gomulur (0 ms).
    # Duserse OLCULDU: recall %96,8 -> %43,0 ve hicbir hata basilmaz.
    # ⚠ Kamera tarafindaki (B) cozumu `setColorOrder(RGB)` ile BIRLIKTE
    # UYGULANMAZ — cift cevirme takasi geri alir.
    "--reverse_input_channels",
]
yol = blobconverter.from_onnx(
    model=onnx,
    data_type="FP16",
    shaves=shave,                       # 🔴 varsayilan 8 — sahada YUKLENMEZ
    version="2022.1",                   # RVC2 / depthai 2.30 uyumlu
    optimizer_params=params,
)
shutil.copy(yol, hedef)
print("blob:", hedef)

# -- PROVENANS.json — blob'un NASIL derlendiginin makine-okunur kaydi -------
# Neden: derleme parametreleri blob'un CIKTISINDAN OKUNAMIYOR (ne shave/giris
# metadata'si, ne config.json gosteriyor — bilinen IYI blob'da da ayni
# gorunuyorlar). 17.08'de dagitilmis bir blob'un parametreleri ancak
# blobconverter cache hash'i kaba kuvvetle yeniden uretilerek anlasildi.
# Bir daha arkeoloji yapilmasin diye uretim aninda yaziliyor; 3_kabul_testi.py
# KONTROL 0 blob'un sha256'sini buradaki kayitla karsilastirir ⇒ kaynagi
# belirsiz / elle kopyalanmis blob testi GECEMEZ.
prov = {
    "blob": os.path.basename(hedef),
    "blob_sha256": _sha(hedef),
    "blob_boyut": os.path.getsize(hedef),
    "nn_giris": giris,
    "shaves": shave,
    "openvino": "2022.1",
    "data_type": "FP16",
    "optimizer_params": params,
    "onnx": os.path.basename(onnx),
    "onnx_sha256": _sha(onnx),
    "pt": os.path.basename(pt),
    "pt_sha256": _sha(pt),
    "uretim_yolu": "scripts/model_uret.sh",
}
with open(os.path.join(os.path.dirname(hedef), "PROVENANS.json"), "w") as f:
    json.dump(prov, f, indent=2, ensure_ascii=False)
print("provenans:", prov["blob_sha256"])
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

#!/usr/bin/env bash
# GİRDAP İDA — saat kurulum zincirini yükle (§0.61g).
#
# Kullanım (TEK SATIR — terminalde satır kayması olmasın diye script):
#     sudo bash ~/IDA_GIT/son_kodv2/karar/scripts/saat_servisleri_kur.sh
#
# 🔴 NEDEN SCRIPT: 13.08'de elle yapıştırılan çok satırlı kurulum komutu
# terminalde kırıldı (`cp: missing destination file operand`), yarım kurulum
# sessizce "tamam" gibi göründü. Kurulum adımları buraya alındı ki tek satır
# olsun ve HER ADIM DOĞRULANSIN.
#
# NE KURAR
#   /usr/local/bin/girdap_saat_kur.py        (--kaynak seri|ros)
#   /etc/systemd/system/girdap-saat.service      açılış, seri, 12 s pencere
#   /etc/systemd/system/girdap-saat-gec.service  YENİ: fix gelince, MAVROS'tan
#
# ⚠️ Yığını YENİDEN BAŞLATMAZ — o ayrı ve bilinçli bir karardır:
#     sudo systemctl restart girdap-karar

set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "HATA: root gerekiyor → sudo bash $0" >&2
    exit 1
fi

KAYNAK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${KAYNAK}"

for dosya in girdap_saat_kur.py girdap-saat.service girdap-saat-gec.service; do
    [[ -f "${dosya}" ]] || { echo "HATA: ${dosya} yok (${KAYNAK})" >&2; exit 1; }
done

echo "[kur] kaynak: ${KAYNAK}"

install -m 0755 girdap_saat_kur.py /usr/local/bin/girdap_saat_kur.py
echo "[kur] /usr/local/bin/girdap_saat_kur.py yazildi"

install -m 0644 girdap-saat.service     /etc/systemd/system/girdap-saat.service
install -m 0644 girdap-saat-gec.service /etc/systemd/system/girdap-saat-gec.service
echo "[kur] iki servis dosyasi /etc/systemd/system/ altina yazildi"

systemctl daemon-reload
systemctl enable girdap-saat.service     >/dev/null
systemctl enable girdap-saat-gec.service >/dev/null
echo "[kur] daemon-reload + enable tamam"

# --- DOĞRULAMA (yarım kurulum sessiz kalmasın) ---
echo
echo "=== DOGRULAMA ==="
/usr/local/bin/girdap_saat_kur.py --help >/dev/null \
    && echo "  ✓ script calisiyor"
grep -q -- "--kaynak ros" /etc/systemd/system/girdap-saat-gec.service \
    && echo "  ✓ gec servis ROS yolunu kullaniyor"
grep -q -- "--zaman-asimi 12" /etc/systemd/system/girdap-saat.service \
    && echo "  ✓ acilis penceresi 12 s (eski: 45 s)"
systemctl is-enabled girdap-saat-gec.service | sed 's/^/  gec servis: /'

echo
echo "SIRADAKI:"
echo "  sudo systemctl start girdap-saat-gec     # simdi dene (fix varsa kurar)"
echo "  journalctl -u girdap-saat-gec -f         # 'SAAT KURULDU' bekle"
echo "  sudo systemctl restart girdap-karar      # yigini YENI kodla baslat"

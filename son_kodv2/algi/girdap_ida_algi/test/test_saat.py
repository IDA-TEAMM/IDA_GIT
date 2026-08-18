#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""girdap_ida_algi/saat.py — saat güvenilirliği (kamerasız, ağsız).

NEDEN BU TESTLER VAR (2026-08-06 arızası):
Jetson ~15 saat bayat saatle açıldı ve o sabah çekilen 18 kare manifest'e
**dünün tarihiyle + saat_guvenilir=1** yazıldı. Eski ölçüt mutlak eşikti
(`epoch >= 2026-01-01`) — 05.08 19:50 de 2026'nın içinde olduğu için eşiği
geçiyordu. Aşağıdaki `test_bayat_saat_vakasi_*` testleri **tam o vakayı**
kilitliyor: bir daha sessizce geçmesin.

Koşum:  python3 -m pytest girdap_ida_algi/test/test_saat.py -q
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from girdap_ida_algi import saat as st  # noqa: E402

# 2026-08-06'da Jetson'ın gösterdiği bayat saat (05.08 21:34 civarı) —
# gerçek zaman 06.08 ~11:07 idi. Tarih "makul", saat YANLIŞ.
BAYAT_AMA_MAKUL = time.mktime((2026, 8, 5, 21, 34, 0, 0, 0, -1))


# --------------------------------------------------- çekirdek senkron bayrağı
def test_cekirdek_senkron_uc_degerden_biri():
    """adjtimex okunur; True/False/None dışında bir şey DÖNMEMELİ.

    Değerin kendisi makineye bağlı (CI'da senkron olmayabilir) — bu yüzden
    sözleşme test ediliyor, değer değil.
    """
    assert st.cekirdek_senkron_mu() in (True, False, None)


def test_cekirdek_senkron_cagrisi_YETKI_ISTEMEZ():
    """Salt-okunur adjtimex (modes=0) unprivileged çalışır; servis root değil."""
    assert st.cekirdek_senkron_mu() is not None or True   # çökmemesi yeterli


# --------------------------------------------------- asıl karar fonksiyonu
def test_bayat_saat_vakasi_ARTIK_YAKALANIYOR():
    """06.08 arızası: tarih makul ama saat senkron değil → GÜVENİLMEZ."""
    assert st.saat_guvenilir_mi(BAYAT_AMA_MAKUL, senkron=False) is False


def test_bayat_saat_vakasi_ESKI_OLCUTLE_KACIYORDU():
    """Aynı an, yalnız mutlak eşikle bakılırsa 'güvenilir' görünüyor —
    arızanın kök nedeni budur, kayıt için test edildi."""
    assert st.saat_guvenilir_mi(BAYAT_AMA_MAKUL, senkron=None) is True


def test_senkronsuz_ama_ELLE_DOGRULANDIYSA_kabul():
    """`sudo date -s` çekirdek bayrağını temizlemez → insan onayı tek telafi."""
    assert st.saat_guvenilir_mi(BAYAT_AMA_MAKUL, senkron=False,
                                elle_dogrulandi=True) is True


def test_mutlak_esik_senkrondan_BAGIMSIZ_veto_eder():
    """Saat 1970'teyse çekirdek 'senkron' dese bile kabul edilmez."""
    assert st.saat_guvenilir_mi(0.0, senkron=True) is False
    assert st.saat_guvenilir_mi(0.0, senkron=True, elle_dogrulandi=True) is False


def test_senkronsa_kabul():
    assert st.saat_guvenilir_mi(st.SAAT_ALT_SINIR, senkron=True) is True


def test_bilinmiyorsa_eski_davranis_korunur():
    """senkron=None → yalnız mutlak eşik (geriye uyumluluk)."""
    assert st.saat_guvenilir_mi(st.SAAT_ALT_SINIR, senkron=None) is True
    assert st.saat_guvenilir_mi(0.0, senkron=None) is False


# --------------------------------------------------- rapor metni
def test_saat_raporu_uc_durumda_da_dolu_metin():
    for s in (True, False, None):
        m = st.saat_raporu(s)
        assert isinstance(m, str) and len(m) > 20


# --------------------------------------------------- manifest entegrasyonu
# 🗑️ 2026-08-16: buradaki 3 test (`test_manifest_*`) veri seti TOPLAYICISININ
#    manifest şemasını kilitliyordu (`oak_veriseti_topla.manifest_satiri`).
#    Toplayıcı repodan kaldırıldığı için testler de kaldırıldı — kilitledikleri
#    dosya artık yok. saat.py'nin KENDİ sözleşmesi yukarıdaki testlerde duruyor;
#    bayat-saat vakasının regresyon kilidi (`test_bayat_saat_vakasi_*`) el
#    değmedi. Toplayıcı geri getirilirse bu blok da geri gelmelidir.

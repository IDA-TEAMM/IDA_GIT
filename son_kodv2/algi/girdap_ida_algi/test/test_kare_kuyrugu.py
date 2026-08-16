"""Dosya-1 kare kuyruğu — TEK BOŞALTICI sözleşmesi (depthai/rclpy GEREKMEZ).

Neden bu dosya var: P3 işi 13.08'de main'den **geri alındı** ve gerekçesi
aynen şuydu —

    *"FAZ 2 P1/P2'nin çalıştırdığı dosyayı (duba_gecis_navigator.py)
    değiştiriyordu: Dosya-1 kayıt akışı (kuyruk tek yerden boşaltılıyor,
    kayıt kopyaya çiziyor) GERÇEK KAMERAYLA hiç denenmedi."*

Yani riskin kaynağı belliydi ama **testi yoktu**. Kamerayı simüle edemeyiz;
edebileceğimiz şey kuyruk mantığının eski davranışla EŞDEĞER olduğunu
donduracak testlerdir. Şartname md 4.2: Dosya-1 ≥1 Hz, eksik dosya **5 ceza**.

Değişikliğin özü:
    ESKİ: `kayit_adimi` kuyruğu KENDİ boşaltıyordu (KAYIT_HZ'te, ~2 Hz).
          Kuyruk boşsa → atla.
    YENİ: `_kare_tazele` kuyruğu TEK YERDEN boşaltıyor (her `dongu()` turunda),
          `kayit_adimi` `_kare_no == _kayit_kare_no` ise atlıyor.

İkisi eşdeğer — ama YALNIZCA `_kare_tazele` en az `kayit_adimi` kadar sık
çağrılırsa. Buradaki testler o önkoşulu ve eşdeğerliği bağlıyor.
"""
from __future__ import annotations

import os
import sys
import types

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _KOK)


class _SahteKare:
    """depthai ImgFrame vekili."""

    def __init__(self, veri):
        self._veri = veri

    def getCvFrame(self):
        return self._veri


class _SahteKuyruk:
    """depthai OutputQueue vekili — `tryGet` kareyi ALIR (tüketir)."""

    def __init__(self, kareler=()):
        self._kareler = list(kareler)

    def ekle(self, *kareler):
        self._kareler.extend(kareler)

    def tryGet(self):
        return self._kareler.pop(0) if self._kareler else None


class _Log:
    def warn(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def _dugum():
    """`_kare_tazele` + `kayit_adimi` kapısını taşıyan en küçük sahte nesne."""
    from girdap_ida_algi import duba_gecis_navigator as dgn

    n = types.SimpleNamespace()
    n.rgb_q = _SahteKuyruk()
    n._son_kare = None
    n._kare_no = 0
    n._kayit_kare_no = -1
    log = _Log()
    n.get_logger = lambda: log
    n._kare_tazele = types.MethodType(dgn.DubaNavigator._kare_tazele, n)
    return n


def _yazilir_mi(n) -> bool:
    """`kayit_adimi`'nin ERKEN DÖNÜŞ kapısı — kare yazılacak mı?

    Gövdenin tamamı cv2/VideoWriter istiyor; test edilen şey kapının kendisi
    (yaz / atla kararı), çizim değil.
    """
    if n._son_kare is None or n._kare_no == n._kayit_kare_no:
        return False
    n._kayit_kare_no = n._kare_no
    return True


# ── tek boşaltıcı sözleşmesi ────────────────────────────────────────────────
def test_kuyrukta_tek_boşaltici_vardir():
    """🔴 Sözleşmenin kendisi: `tryGet()` yalnız `_kare_tazele`'de geçmeli.

    İki ayrı boşaltıcı olsaydı hangisi önce koşarsa kareyi o kapardı; diğeri
    None görürdü → Dosya-1'de boşluk YA DA hedefin hiç görülmemesi.
    """
    kaynak = os.path.join(_KOK, "girdap_ida_algi", "duba_gecis_navigator.py")
    satirlar = [s for s in open(kaynak, encoding="utf-8")
                if "rgb_q.tryGet()" in s and not s.strip().startswith("#")]
    assert len(satirlar) == 1, (
        f"rgb_q.tryGet() {len(satirlar)} yerde — TEK olmali (tek bosaltici)")


def test_kare_tazele_donguınun_basinda_kosulsuz_cagrilir():
    """Eşdeğerliğin ÖNKOŞULU: tazeleme kayıttan seyrek olamaz.

    `kayit_adimi` KAYIT_HZ ile sınırlı; `_kare_tazele` her turda koşmalı ve
    hiçbir `if` altında olmamalı — yoksa kuyruk birikir, kare bayatlar.
    """
    kaynak = os.path.join(_KOK, "girdap_ida_algi", "duba_gecis_navigator.py")
    metin = open(kaynak, encoding="utf-8").read()
    govde = metin.split("def dongu(self):", 1)[1]
    once = govde.split("self._kare_tazele()", 1)[0]
    assert "self._kare_tazele()" in govde, "dongu() kareyi tazelemiyor"
    assert "kayit_adimi" not in once, "kayit_adimi tazelemeden ONCE cagriliyor"
    assert "hedef_adimi" not in once, "hedef_adimi tazelemeden ONCE cagriliyor"
    # koşulsuz: çağrının girintisi gövde seviyesinde (8 boşluk) olmalı
    for satir in govde.splitlines():
        if "self._kare_tazele()" in satir:
            assert satir.startswith(" " * 8) and not satir.startswith(" " * 9), \
                f"_kare_tazele KOSULLU cagriliyor: {satir!r}"
            break


# ── eski davranışla eşdeğerlik ──────────────────────────────────────────────
def test_yeni_kare_gelince_yazilir():
    n = _dugum()
    n.rgb_q.ekle(_SahteKare("A"))
    n._kare_tazele()
    assert n._son_kare == "A"
    assert _yazilir_mi(n) is True


def test_YENI_KARE_YOKSA_yazilmaz():
    """Eski davranış: 'kuyruk boşsa atla'. Yeni: kare numarası artmadı → atla.

    Bu test ikisinin AYNI olduğunu donduruyor — aynı kareyi iki kez yazmak
    mp4 zaman çizgisini bozar ve tespit çerçevesi bayat görünür.
    """
    n = _dugum()
    n.rgb_q.ekle(_SahteKare("A"))
    n._kare_tazele()
    assert _yazilir_mi(n) is True
    n._kare_tazele()                 # kuyruk BOŞ — yeni kare yok
    assert _yazilir_mi(n) is False, "ayni kare ikinci kez yazildi"


def test_kuyruk_birikirse_EN_TAZE_kare_alinir():
    """Gecikmiş kare yazmak Dosya-1'i hakem gözünde 'gerçek zamanlı değil'
    yapar; kuyruk her zaman sonuna kadar boşaltılmalı."""
    n = _dugum()
    n.rgb_q.ekle(_SahteKare("eski"), _SahteKare("orta"), _SahteKare("TAZE"))
    n._kare_tazele()
    assert n._son_kare == "TAZE"
    assert n._kare_no == 1, "birikmis kuyruk TEK kare sayilmali"


def test_ilk_karede_yazim_ATLANMAZ():
    """`_kayit_kare_no` başlangıcı yanlış olsaydı ilk kare düşerdi ve kayıt
    bir periyot geç başlardı."""
    n = _dugum()
    assert n._kayit_kare_no == -1 != n._kare_no
    n.rgb_q.ekle(_SahteKare("ilk"))
    n._kare_tazele()
    assert _yazilir_mi(n) is True


def test_kamera_durursa_COKMEZ():
    """Kamera susarsa (kuyruk sürekli boş) node ölmemeli, sadece yazmamalı —
    kayıt hatası görevi ASLA durdurmaz (md 4.2 notu)."""
    n = _dugum()
    for _ in range(50):
        n._kare_tazele()
        assert _yazilir_mi(n) is False
    assert n._son_kare is None
    assert n._kare_no == 0


def test_bozuk_kare_kuyrugu_KILITLEMEZ():
    """`getCvFrame()` patlarsa: log + devam. Sayaç artmamalı ki bozuk kare
    'yeni kare' sanılıp yazılmaya çalışılmasın."""
    class _Bozuk:
        def getCvFrame(self):
            raise RuntimeError("bozuk kare")

    n = _dugum()
    n.rgb_q.ekle(_Bozuk())
    n._kare_tazele()                 # istisna DIŞARI çıkmamalı
    assert n._son_kare is None
    assert n._kare_no == 0
    assert _yazilir_mi(n) is False


def test_kayit_HAM_kareyi_bozmaz():
    """🔴 Kayıt görüntünün ÜSTÜNE çiziyor. Kopyalanmazsa `_son_kare` boyanır
    ve Parkur-3 renk analizi KENDİ çizdiğimiz turuncu/sarı çerçeveyi okur —
    hedef rengi sistematik yanlış çıkar. Kaynakta `.copy()` şart."""
    kaynak = os.path.join(_KOK, "girdap_ida_algi", "duba_gecis_navigator.py")
    govde = open(kaynak, encoding="utf-8").read().split("def kayit_adimi", 1)[1]
    ilk = govde.split("def ", 1)[0]
    assert "self._son_kare.copy()" in ilk, \
        "kayit ham kareye ciziyor — P3 renk analizi bozulur"


def test_hedef_yayini_kayittan_BAGIMSIZ():
    """Kayıt kalıcı bozulsa (`_kayit_bozuk`) bile Parkur-3 hedef yayını
    sürmeli; aksi hâlde P3, Dosya-1 ile birlikte sessizce ölürdü."""
    kaynak = os.path.join(_KOK, "girdap_ida_algi", "duba_gecis_navigator.py")
    govde = open(kaynak, encoding="utf-8").read().split("def dongu(self):", 1)[1]
    hedef_satiri = [s for s in govde.splitlines() if "self.hedef_adimi(" in s]
    assert hedef_satiri, "dongu() hedef_adimi cagirmiyor"
    assert "_kayit_bozuk" not in hedef_satiri[0], \
        "hedef yayini _kayit_bozuk'a bagli — kayit olunce P3 de olur"

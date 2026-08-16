"""PARKUR-3 hedef yayını VARSAYILAN KAPALI (depthai/rclpy GEREKMEZ).

Eyüp kararı (16.08.2026): *"Parkur 3'ü şimdilik aktif etme, kodlardan
deaktif kalsın, testte bozar — Parkur 1 ve Parkur 2'yi ölçüyoruz."*

Kapalıyken hiçbir P3 işi koşmamalı: `/perception/targets` yayını yok, bbox
kırpma yok, renk analizi yok ⇒ P1/P2 ölçümü P3 kodundan **hiç etkilenmez**.

🔴 AMA ŞALTER *BÜYÜK CİSİM SÜZGECİNİ* KAPATMAMALI. O süzgeç P3 için değil,
**P2'yi korumak** için var: P3 hedefi Ø64 cm, 25 m'den 7,7 px eder ve tekne
daha P2'nin İÇİNDEYKEN görür. Kenar/engel diye yayınlanırsa füzyon
`EdgeBuoyMemory`'ye KALICI kenar yazar → hayalet kapı → P2'de rota bozulur.
Yani şalteri "P3'ü kapat" diye genişletmek P2'yi BOZARDI.
"""
from __future__ import annotations

import os

_KAYNAK = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "girdap_ida_algi", "duba_gecis_navigator.py",
)


def _metin() -> str:
    return open(_KAYNAK, encoding="utf-8").read()


def test_varsayilan_KAPALI():
    """Ortam değişkeni verilmezse yayın kapalı olmalı.

    Varsayılanı `"1"` yapan bir düzenleme, P1/P2 ölçüm gününde P3'ü sessizce
    açardı — testin yakaladığı şey tam olarak bu.
    """
    s = _metin()
    assert 'os.environ.get("GIRDAP_P3_HEDEF", "0")' in s, \
        "varsayilan '0' degil — P3 kendiliginden ACIK olabilir"


def test_kapali_iken_hedef_adimi_HEMEN_doner():
    """Şalter kontrolü, hız kapısından ve her türlü işten ÖNCE olmalı.

    Sonra gelseydi bbox kırpma/renk analizi yine koşar, P1/P2 ölçümüne CPU
    ve log gürültüsü karışırdı.
    """
    govde = _metin().split("def hedef_adimi", 1)[1].split("\n    def ", 1)[0]
    satirlar = [x.strip() for x in govde.splitlines()
                if x.strip() and not x.strip().startswith(("#", '"', "'"))]
    # docstring'i atla: ilk gerçek kod satırı şalter olmalı
    ilk_kod = next(x for x in satirlar
                   if x.startswith(("if", "self.", "return", "try")))
    assert "P3_HEDEF_YAYINI" in ilk_kod, \
        f"ilk kod satiri salter degil: {ilk_kod!r}"


def test_salter_BUYUK_CISIM_SUZGECINI_kapatmaz():
    """🔴 Süzgeç P2'yi koruyor — şalterle birlikte kapanırsa P2'de hayalet
    kapı oluşur ve rota bozulur. Süzgeç `P3_HEDEF_YAYINI`'ye BAĞLI OLMAMALI."""
    s = _metin()
    assert "buyuk_cisim_mi" in s, "buyuk cisim suzgeci kaybolmus"
    # süzgecin geçtiği satırın bulunduğu blokta şalter geçmemeli
    idx = s.index("buyuk_cisim_mi")
    pencere = s[max(0, idx - 1500):idx]
    assert "if not P3_HEDEF_YAYINI" not in pencere, \
        "buyuk cisim suzgeci saltere baglanmis — P2 bozulur"


def test_dosya1_kaydi_salterden_BAGIMSIZ():
    """Dosya-1 (md 4.2, ≥1 Hz, eksik dosya 5 ceza) P3'ten bağımsız sürmeli."""
    govde = _metin().split("def dongu(self):", 1)[1]
    kayit_satiri = [x for x in govde.splitlines() if "kayit_adimi()" in x]
    assert kayit_satiri, "dongu() kayit_adimi cagirmiyor"
    onceki = govde.split("kayit_adimi()", 1)[0]
    assert "P3_HEDEF_YAYINI" not in onceki, \
        "Dosya-1 kaydi P3 salterine baglanmis — sarta aykiri"


def test_kare_tazeleme_salterden_BAGIMSIZ():
    """Kare tazeleme Dosya-1'i besliyor; şaltere bağlanırsa kayıt ölür."""
    govde = _metin().split("def dongu(self):", 1)[1]
    onceki = govde.split("self._kare_tazele()", 1)[0]
    assert "P3_HEDEF_YAYINI" not in onceki, \
        "kare tazeleme P3 salterine baglanmis — Dosya-1 olur"


def test_acma_yolu_BELGELENMIS():
    """Yarışma günü açacak kişi nasıl açacağını koddan görebilmeli."""
    s = _metin()
    assert "GIRDAP_P3_HEDEF=1" in s, "acma yolu belgelenmemis"

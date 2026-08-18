"""`gol_pusula_ayar.py` — GÖLDE OTOMATİK PUSULA AYARININ KAPILARI.

🔑 NEDEN (17.08.2026): `PreArm: Check mag field (xy diff:118-207>100)` bantlarda
13·14·16.08'de **gölde** kaydedildi — kapalı alan artefaktı değil, gerçek.
Standart mag cal aracı üç eksende döndürmeyi ister; katamaran yalpa/yunuslama
yapamaz. ArduPilot'un "büyük araç" yöntemi (`MAV_CMD_FIXED_MAG_CAL_YAW`, 42006)
bilinen bir yön ister — bizde o yön bedava: GPS'in yer rotası.

Bu dosya iki şeyi dondurur:
  ① **GUIDED'da ASLA kalibre etme.** Kalibrasyon ofsetleri değiştirir; seyrin
    dayandığı sinyali seyir sırasında oynatmak olur. Ayrıca GUIDED'da dümen
    kapalı çevrimdir — "düz gidiyor" hâli denetleyicinin eseridir, ölçüm değil.
  ② **"İşe yaradı" iddia değil ÖLÇÜM.** Öncesi/sonrası yön hatası
    karşılaştırılmadan başarı ilan edilmez.

⚠️ Testler FC'ye komut GÖNDERMEZ, düğüm açmaz — saf kapı mantığı sınanır.
"""
import importlib.util
import os

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "gol_pusula_ayar.py")

pytest.importorskip("rclpy", reason="ROS ortamı yok")
pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("gol_pusula_ayar", _BETIK)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ── açı matematiği (dairesel — 359° ile 1° komşudur) ───────────────────────

def test_aci_farki_sarmali_dogru(mod):
    assert mod.aci_farki(1.0, 359.0) == pytest.approx(2.0)
    assert mod.aci_farki(359.0, 1.0) == pytest.approx(-2.0)
    assert abs(mod.aci_farki(180.0, 0.0)) == pytest.approx(180.0)


def test_aci_ortalama_SIFIR_etrafinda_dogru(mod):
    """Düz aritmetik ortalama burada 180° verir — klasik tuzak."""
    assert mod.aci_ortalama([359.0, 1.0]) == pytest.approx(0.0, abs=1e-6)
    assert mod.aci_ortalama([10.0, 350.0]) == pytest.approx(0.0, abs=1e-6)


def test_aci_sapma_kararli_seyirde_kucuk(mod):
    assert mod.aci_sapma([100.0, 101.0, 99.0, 100.5]) < 1.5


def test_aci_sapma_oynak_seyirde_buyuk(mod):
    assert mod.aci_sapma([0.0, 60.0, 300.0]) > 30.0


def test_aci_sapma_TANIMSIZ_ortalamada_None_doner(mod):
    """Eşit dağılmış açıların dairesel ortalaması YOKTUR — uydurulmamalı.

    `_duz_seyir_mi` bunu `sap is None` ile reddeder: ortalaması tanımsız bir
    rota kümesi kesinlikle "düz seyir" değildir.
    """
    assert mod.aci_ortalama([0.0, 90.0, 180.0, 270.0]) is None
    assert mod.aci_sapma([0.0, 90.0, 180.0, 270.0]) is None


def test_ortalama_TANIMSIZSA_duz_seyir_REDDEDILIR(mod):
    s = _SahteDurum(pencere=[(i * 0.2, r, 1.0, 95.0)
                             for i, r in enumerate([0.0, 90.0, 180.0, 270.0] * 15)])
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "sapma" in sebep


# ── ① mod kapısı: GUIDED'da ASLA ───────────────────────────────────────────

class _SahteDurum:
    """`_duz_seyir_mi` yüzeyi — fazlası değil.

    `_mod_uygun_mu` GERÇEK metottur (fixture'da bağlanır): sınanan kapının
    kendisi taklit edilirse test hiçbir şey doğrulamaz.
    """

    def __init__(self, **kw):
        self._mod = kw.get("mod", "MANUAL")
        self._armed = kw.get("armed", True)
        self._fix = kw.get("fix", 0)
        self._donus = kw.get("donus", 0.0)
        self._pencere = kw.get("pencere", [])


@pytest.fixture(autouse=True)
def _gercek_mod_kapisi(mod):
    """Sahte duruma GERÇEK mod kapısını bağla — taklit etme."""
    _SahteDurum._mod_uygun_mu = mod.PusulaAyar._mod_uygun_mu


def _pencere(rota=90.0, hiz=1.0, pusula=95.0, n=60, sap=0.0, sure=10.0):
    return [(i * sure / n, rota + (sap if i % 2 else -sap), hiz, pusula)
            for i in range(n)]


@pytest.mark.parametrize("mod_adi", ["GUIDED", "AUTO", "HOLD", "ACRO",
                                     "SMART_RTL", "RTL"])
def test_MANUAL_DISINDA_hicbir_modda_kalibre_ETMEZ(mod, mod_adi):
    """🔴 En kritik kapı. GUIDED'da kalibrasyon = seyrin sinyalini seyirde oynatmak."""
    s = _SahteDurum(mod=mod_adi, pencere=_pencere())
    assert mod.PusulaAyar._mod_uygun_mu(s) is False
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False
    assert "MANUAL" in sebep


def test_MANUAL_duz_seyirde_kapi_ACILIR(mod):
    s = _SahteDurum(mod="MANUAL", pencere=_pencere())
    uygun, sebep, veri = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is True, f"düz seyir reddedildi: {sebep}"
    rota, pus, sapma, n = veri
    assert rota == pytest.approx(90.0, abs=1.0)
    assert pus == pytest.approx(95.0, abs=1.0)


# ── ② düz seyir kapısının bileşenleri ──────────────────────────────────────

def test_DISARM_iken_kalibre_ETMEZ(mod):
    s = _SahteDurum(armed=False, pencere=_pencere())
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "DISARM" in sebep


def test_GPS_fix_yoksa_kalibre_ETMEZ(mod):
    """Bu yöntem dünya manyetik modeli için KONUM ister — fix'siz anlamsız."""
    s = _SahteDurum(fix=-1, pencere=_pencere())
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "GPS" in sebep


def test_YAVASKEN_kalibre_ETMEZ(mod):
    """Yavaşta GPS rotası gürültülü ve yan kayma açısı büyür."""
    s = _SahteDurum(pencere=_pencere(hiz=mod.ASGARI_HIZ - 0.1))
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "hız" in sebep


def test_DONERKEN_kalibre_ETMEZ(mod):
    s = _SahteDurum(donus=mod.AZAMI_DONUS + 5.0, pencere=_pencere())
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "dönüş" in sebep


def test_ROTA_OYNAKKEN_kalibre_ETMEZ(mod):
    s = _SahteDurum(pencere=_pencere(sap=mod.ROTA_KARARLILIK + 8.0))
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "sapma" in sebep


def test_ORNEK_AZKEN_kalibre_ETMEZ(mod):
    s = _SahteDurum(pencere=_pencere(n=mod.ASGARI_ORNEK - 5))
    uygun, sebep, _ = mod.PusulaAyar._duz_seyir_mi(s)
    assert uygun is False and "örnek" in sebep


# ── ③ komut sözleşmesi ─────────────────────────────────────────────────────

def test_komut_ID_ve_imzasi_DONDURULDU(mod):
    """42006 + MAVProxy'nin `magcal yaw` imzası. Değişirse bilinçli olsun."""
    assert mod.KOMUT_SABIT_PUSULA == 42006
    assert mod.PUSULA_MASKESI == 0.0, "0 = bütün pusulalar"
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "param1=float(yon)" in kaynak, "yön param1'de olmalı"
    assert "param3=0.0, param4=0.0" in kaynak, (
        "lat/lon 0 olmalı — FC mevcut GPS konumunu kullanır")


def test_KURU_kipte_komut_GONDERILMEZ(mod):
    """Sözleşme 6: --kuru hiçbir şey göndermez."""
    class _S:
        kuru = True
        _komut = None            # servise dokunulursa AttributeError patlar
        satirlar = []
        def _bas(self, t, m): self.satirlar.append((t, m))
    s = _S()
    assert mod.PusulaAyar._kalibre_et(s, 123.4) is True
    assert any("KURU" in m for _t, m in s.satirlar)


def test_kalibrasyon_TEK_SEFER_gonderilir(mod):
    """Sözleşme 3: komut bir kez gider, döngüde tekrar etmez."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert kaynak.count("self._kalibre_et(") == 1, (
        "kalibrasyon birden fazla yerden çağrılıyor — tekrar tekrar "
        "ofset yazma riski")


# ── ④ "işe yaradı" ölçüm olmalı, iddia değil ───────────────────────────────

def test_IYILESME_olculmeden_BASARI_ilan_EDILMEZ(mod):
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "IYILESME_ORANI" in kaynak
    i_olcum = kaynak.index("self._hata_sonra = self._hata_biriktir")
    i_basari = kaynak.index("✅ yön hatası")
    assert i_olcum < i_basari, (
        "başarı, sonrası ölçülmeden ilan ediliyor")


def test_ofset_DEGISMEDIYSE_basari_SAYILMAZ(mod):
    """FC 'kabul ettim' deyip hiçbir şey yazmamış olabilir (GPS fix yoksa)."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "OFSETLER DEĞİŞMEDİ" in kaynak, (
        "komut başarılı döndü diye ofsetlerin yazıldığı varsayılıyor")


def test_YAN_KAYMA_sinirini_belgeliyor(mod):
    """Bu yöntemin gerçek sınırı gizlenmemeli — sessiz varsayım yasak."""
    assert "yan kayma" in (mod.__doc__ or "").lower() or \
           "crab" in (mod.__doc__ or "").lower(), (
        "GPS rotası = gittiği yön, burnunun baktığı yön DEĞİL — "
        "bu sınır başlıkta yazılı olmalı")


# ── ⑤ AKINTI — gölde gerçek koşul (kaptan: "akıntı var ama orda") ──────────

def test_akinti_YOKKEN_gercek_yon_GPS_rotasidir(mod):
    """Akıntı sıfırsa iki bacak da GPS rotasını verir, ayrıştırma zarar vermez."""
    # kuzeye 1 m/s, sonra güneye 1 m/s
    ak, su, yon = mod.akinti_coz((0.0, 1.0), (0.0, -1.0))
    assert ak == pytest.approx((0.0, 0.0), abs=1e-9)
    assert yon == pytest.approx(0.0, abs=1e-6), "kuzey 0° olmalı"
    assert su[1] == pytest.approx(1.0)


def test_akinti_VARKEN_yan_kaymayi_TEMIZLER(mod):
    """🔑 Asıl senaryo. Tekne KUZEYE bakıyor ama akıntı DOĞUYA itiyor.

    Ham GPS rotası kuzeydoğuyu gösterir (yanlış); ayrıştırma kuzeyi bulmalı.
    """
    su_hizi, akinti = 1.0, 0.5          # doğuya 0,5 m/s akıntı
    yer1 = (akinti, +su_hizi)           # burun kuzey
    yer2 = (akinti, -su_hizi)           # burun güney
    ak, su, yon = mod.akinti_coz(yer1, yer2)
    assert ak[0] == pytest.approx(0.5), "akıntı doğu bileşeni bulunamadı"
    assert ak[1] == pytest.approx(0.0, abs=1e-9)
    assert yon == pytest.approx(0.0, abs=1e-6), (
        "gerçek yön kuzey olmalı — yan kayma temizlenmemiş")

    # Ham GPS rotası ne diyordu? Kalibrasyona yazılacak olan HATA buydu:
    ham = mod.sarmala(__import__("math").degrees(
        __import__("math").atan2(yer1[0], yer1[1])))
    assert ham == pytest.approx(26.57, abs=0.1)
    assert abs(mod.aci_farki(ham, yon)) > 25.0, (
        "ham rota ile gerçek yön arasında 26°'lik hata var — "
        "ayrıştırma olmasa bu doğrudan pusula ofsetine yazılırdı")


def test_akinti_TERS_yonde_de_dogru(mod):
    """Akıntı batıya olursa işaret de ters çıkmalı."""
    ak, _su, yon = mod.akinti_coz((-0.5, 1.0), (-0.5, -1.0))
    assert ak[0] == pytest.approx(-0.5)
    assert yon == pytest.approx(0.0, abs=1e-6)


def test_akinti_egik_yonde_cozulur(mod):
    """Tekne doğuya bakıyor, akıntı kuzeye itiyor."""
    ak, _su, yon = mod.akinti_coz((1.0, 0.4), (-1.0, 0.4))
    assert ak == pytest.approx((0.0, 0.4), abs=1e-9)
    assert yon == pytest.approx(90.0, abs=1e-6), "doğu 90° olmalı"


def test_bacaklar_AYNI_yonse_yon_ILAN_EDILMEZ(mod):
    """İki bacak ters değilse V_su sıfıra yakın çıkar — sayı uydurulmamalı."""
    _ak, _su, yon = mod.akinti_coz((0.0, 1.0), (0.0, 1.0))
    assert yon is None, "ters olmayan bacaklardan yön ilan edildi"


# ── ⑥ ÇAKIŞMA: COMPASS_LEARN açıkken bu araç ÖLÇEMEZ ──────────────────────

class _SahteOgrenme:
    def __init__(self, deger):
        self._deger = deger
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))

    def _oku(self, ad, ts=15.0):
        return self._deger


@pytest.mark.parametrize("learn", [1.0, 2.0, 3.0])
def test_COMPASS_LEARN_ACIKKEN_olcmeyi_REDDEDER(mod, learn):
    """İki özne aynı ofsetleri değiştirirse kontrol grubu bozulur.

    17.08: takımdan biri COMPASS_LEARN=3'ü param dosyasına işledi. Bu araç
    o hâlde "ölçtüm" diyemez — iyileşme kime ait, ayrılamaz.
    """
    s = _SahteOgrenme(learn)
    assert mod.PusulaAyar._ogrenme_kapisi(s) is False
    assert any(mod.P_LEARN in m for _t, m in s.satirlar)
    assert any("İNSAN kararı" in m for _t, m in s.satirlar), (
        "araç insanın yerine karar veriyor")


def test_COMPASS_LEARN_KAPALIYKEN_gecer(mod):
    s = _SahteOgrenme(0.0)
    assert mod.PusulaAyar._ogrenme_kapisi(s) is True


def test_COMPASS_LEARN_okunamazsa_SESSIZ_gecmez(mod):
    """Kapı uygulanamadıysa raporlanmalı — 'kapı yok' ile 'geçti' aynı değil."""
    s = _SahteOgrenme(None)
    assert mod.PusulaAyar._ogrenme_kapisi(s) is True
    assert any("uygulanamadı" in m for _t, m in s.satirlar)


def test_ogrenme_kapisi_KALIBRASYONDAN_ONCE_kosar(mod):
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert kaynak.index("_ogrenme_kapisi()") < kaynak.index("self._kalibre_et("), (
        "çakışma kapısı kalibrasyondan SONRA kontrol ediliyor")

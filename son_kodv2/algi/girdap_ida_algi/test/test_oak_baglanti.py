#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""oak_baglanti.py — kamerasız testler.

USB ioctl'i ve gerçek cihaz burada test EDİLMEZ (donanım gerekir); test edilen
şey ETRAFINDAKİ MANTIK: kaç kez denenir, ne zaman reset atılır, hangi hata
yukarı fırlatılır, sıcaklık nasıl sınıflandırılır. Bunlar sahada yanlış
davranırsa kamera hiç açılmaz ya da aşırı ısınma sessizce geçer.

Koşum:  python3 -m pytest girdap_ida_algi/test/test_oak_baglanti.py -q
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from girdap_ida_algi import oak_baglanti as ob  # noqa: E402

GERCEK_USB_DUGUM_YOLU = ob.usb_dugum_yolu
SAHTE_YOL = "/dev/bus/usb/001/007"


@pytest.fixture(autouse=True)
def cihaz_takili_varsay(monkeypatch):
    """Süit GERÇEK USB'ye BAKMASIN — F-A.4 sonrası zorunlu oldu.

    ⚠️ 2026-08-16 DÜZELTMESİ: bu metin *"`dayanikli_ac` süresiz bekliyor"*
    diyordu — YANLIŞTI. Bekleyen `cihazi_bekle`; `dayanikli_ac` eskisi gibi
    4 deneme sonunda `raise` eder. Üstelik `cihazi_bekle` 16.08'e kadar
    hiçbir yerden ÇAĞRILMIYORDU (bkz. `test_FA4_BAGLI_MI_*`). Docstring,
    gerçekleşmemiş bir niyeti olmuş gibi anlatıyordu.

    Kanca yine de gerekli: cihaz yokluğunu sınayan testler `usb_dugum_yolu`u
    kendileri ezmezse süit gerçek USB'ye bakar. Kamera takılı
    Jetson'da süit yeşil, kamera olmayan laptopta **sonsuza dek asılır**
    (ölçüldü: 15.08, 8 sn'lik zaman aşımıyla kanıtlandı). Bu kanca ile testler
    "cihaz takılı" varsayar; cihazın YOKLUĞUNU sınayan testler bunu kendi
    içinde ezer.
    """
    monkeypatch.setattr(ob, "usb_dugum_yolu", lambda: SAHTE_YOL)


@pytest.fixture
def gercek_usb_dugum_yolu(monkeypatch):
    """`usb_dugum_yolu`'nun KENDİSİNİ sınayan testler için kancayı geri al."""
    monkeypatch.setattr(ob, "usb_dugum_yolu", GERCEK_USB_DUGUM_YOLU)


# ───────────────────────────────── dayanikli_ac ───────────────────────────
def test_ilk_denemede_acilirsa_reset_atilmaz(monkeypatch):
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: resetler.append(1) or True)
    dev = ob.dayanikli_ac(lambda: "CIHAZ", deneme=4)
    assert dev == "CIHAZ"
    assert resetler == []          # boş yere USB resetlemek cihazı kilitleyebilir


def test_kilitlenirse_reset_atip_yeniden_dener(monkeypatch):
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: (resetler.append(1), True)[1])
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)

    denemeler = {"n": 0}

    def acici():
        denemeler["n"] += 1
        if denemeler["n"] < 3:
            raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")
        return "CIHAZ"

    assert ob.dayanikli_ac(acici, deneme=4) == "CIHAZ"
    assert denemeler["n"] == 3
    assert len(resetler) == 2       # her başarısız denemeden sonra bir reset


def test_deneme_tukenirse_SON_hatayi_firlatir(monkeypatch):
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)

    def acici():
        raise RuntimeError("son hata")

    with pytest.raises(RuntimeError, match="son hata"):
        ob.dayanikli_ac(acici, deneme=3)


def test_son_denemeden_sonra_bosuna_reset_atmaz(monkeypatch):
    """Deneme bittiyse reset atmanın faydası yok; cihazı boşuna sarsma."""
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: (resetler.append(1), True)[1])
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    with pytest.raises(RuntimeError):
        ob.dayanikli_ac(lambda: (_ for _ in ()).throw(RuntimeError("x")), deneme=3)
    assert len(resetler) == 2       # 3 deneme → aralarda 2 reset, sonda YOK


def test_reset_basarisizsa_yine_de_dener(monkeypatch):
    """USB reset başarısız olsa bile (kural yok/cihaz yok) açılış denenmeye devam
    etmeli — kilit bazen kendiliğinden açılıyor."""
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: False)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    n = {"i": 0}

    def acici():
        n["i"] += 1
        if n["i"] < 2:
            raise RuntimeError("kilit")
        return "CIHAZ"

    assert ob.dayanikli_ac(acici, deneme=3) == "CIHAZ"


def test_log_fonksiyonu_cagrilir(monkeypatch):
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    satirlar = []
    n = {"i": 0}

    def acici():
        n["i"] += 1
        if n["i"] < 2:
            raise RuntimeError("kilit")
        return "CIHAZ"

    ob.dayanikli_ac(acici, deneme=3, kaydet=satirlar.append)
    assert any("açılamadı" in s for s in satirlar)
    assert any("reset" in s.lower() for s in satirlar)


# ─────────────── F-A.4: CİHAZ YOKKEN BEKLE, ÇÖKME DÖNGÜSÜ KURMA ───────────
# Göl günü ölçümü (§1.09f): kamera USB'den düşünce `dayanikli_ac` 4 denemeyi
# tüketip fırlatıyordu → düğüm ölüyor → ROS `respawn` ediyor → ~30 saniyede bir
# aynı çökme. Bir saat boyunca onlarca tur. Ayrım: "cihaz USB'de YOK" beklenecek
# bir DURUM, "cihaz VAR ama açılmıyor" gerçek ARIZA.

def test_FA4_cihaz_yokken_COKMEZ_gelince_acar(monkeypatch):
    """Cihaz yokken fırlatmaz: gelene kadar bekler, gelince açar.

    ⚠️ 16.08'de bu test VAKUMDU: `acici` hiç fırlatmadığı için `dayanikli_ac`
    ilk denemede dönüyordu ve `sahte_yol` HİÇ yoklanmıyordu — yani
    `cihazi_bekle` bağlanmamışken de yeşildi (ölçüldü). Artık yoklamanın
    GERÇEKTEN yapıldığı da doğrulanıyor; bağlantı koparsa test kırmızıya döner.
    """
    yoklama = {"n": 0}

    def sahte_yol():
        yoklama["n"] += 1
        return None if yoklama["n"] < 4 else SAHTE_YOL

    monkeypatch.setattr(ob, "usb_dugum_yolu", sahte_yol)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    assert ob.dayanikli_ac(lambda: "CIHAZ", deneme=4) == "CIHAZ"
    # 🔴 VAKUM KIRICI: bekleme gerçekten koştu mu?
    assert yoklama["n"] >= 4, (
        "usb_dugum_yolu yoklanmadı — dayanikli_ac cihazı BEKLEMİYOR, "
        "F-A.4 yine bağlanmamış."
    )


def test_FA4_bekleme_DENEME_HAKKI_YEMEZ(monkeypatch):
    """Cihazın yokluğunda geçen süre açılış denemesi saymaz.

    Yerse: kamera 10 saniye geç takıldığında hak tükenir ve düğüm yine çöker —
    düzeltmenin tamamı boşa gider.
    """
    yoklama = {"n": 0}

    def sahte_yol():
        yoklama["n"] += 1
        return None if yoklama["n"] <= 10 else SAHTE_YOL

    monkeypatch.setattr(ob, "usb_dugum_yolu", sahte_yol)
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)

    denemeler = {"n": 0}

    def acici():
        denemeler["n"] += 1
        raise RuntimeError("X_LINK kilidi")

    with pytest.raises(RuntimeError, match="X_LINK"):
        ob.dayanikli_ac(acici, deneme=3)
    assert denemeler["n"] == 3      # 10 yoklama beklendi, hak yine 3
    # 🔴 VAKUM KIRICI: bekleme hiç koşmasaydı bu da geçerdi (16.08 ölçümü).
    assert yoklama["n"] > 10, (
        "usb_dugum_yolu 10 kez None dönmesine rağmen beklenmedi — "
        "dayanikli_ac cihazı BEKLEMİYOR."
    )


def test_FA4_cihaz_VARKEN_arizada_ESKI_davranis_KORUNUR(monkeypatch):
    """Cihaz takılı ama açılmıyorsa bu gerçek arızadır → fırlat.

    Sessizce beklemek arızayı GİZLERDİ; F-A.4 yalnız 'cihaz yok' hâlini
    değiştirir.
    """
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    with pytest.raises(RuntimeError, match="kilit"):
        ob.dayanikli_ac(lambda: (_ for _ in ()).throw(RuntimeError("kilit")),
                        deneme=2)


def test_FA4_beklerken_SESSIZ_KALMAZ(monkeypatch):
    """Sessizlik başarı değildir: beklerken de, cihaz gelince de haber verir."""
    yoklama = {"n": 0}

    def sahte_yol():
        yoklama["n"] += 1
        return None if yoklama["n"] < 3 else SAHTE_YOL

    monkeypatch.setattr(ob, "usb_dugum_yolu", sahte_yol)
    saat = {"t": 0.0}
    monkeypatch.setattr(ob.time, "monotonic", lambda: saat["t"])

    def sahte_uyku(s):
        saat["t"] += 60.0           # bildirim aralığını (30 sn) mutlaka aş

    monkeypatch.setattr(ob.time, "sleep", sahte_uyku)

    satirlar = []
    ob.cihazi_bekle(kaydet=satirlar.append)
    assert any("bekleniyor" in s for s in satirlar)
    assert any("göründü" in s for s in satirlar)


def test_FA4_cihaz_ZATEN_takiliysa_BEKLEMEZ(monkeypatch):
    """Takılı cihazda tek saniye bile kaybetme — açılış zaten 6,5 dakika (§0.59)."""
    uykular = []
    monkeypatch.setattr(ob.time, "sleep", uykular.append)
    satirlar = []
    ob.cihazi_bekle(kaydet=satirlar.append)
    assert uykular == []
    assert satirlar == []


# ─────────────────────────────── sicaklik_durumu ──────────────────────────
@pytest.mark.parametrize("c,beklenen", [
    (25.0, "normal"),
    (68.9, "normal"),      # 416@11 FPS'te ölçülen iç mekân platosu (12.08'de
                           # 512@8'e geçildi; yeni plato Jetson'da ölçülecek)
    (84.9, "normal"),
    (85.0, "uyari"),       # eşik DÂHİL
    (94.9, "uyari"),
    (95.0, "kritik"),      # eşik DÂHİL
    (125.0, "kritik"),     # gözlenen çökme sıcaklığı
])
def test_sicaklik_esikleri(c, beklenen):
    assert ob.sicaklik_durumu(c) == beklenen


def test_sicaklik_okunamazsa_alarm_uretmez():
    """Ölçemediğimiz şey için 'kritik' demek yanlış alarmdır; None → normal."""
    assert ob.sicaklik_durumu(None) == "normal"


def test_esikler_disaridan_verilebilir():
    assert ob.sicaklik_durumu(70.0, uyari=60.0, kritik=80.0) == "uyari"
    assert ob.sicaklik_durumu(90.0, uyari=60.0, kritik=80.0) == "kritik"


def test_kritik_esik_cip_anma_sinirinin_ALTINDA():
    """Çip anma sınırı 105 °C, gözlenen çökme 125 °C. Kritik eşiğimiz bunların
    altında olmalı ki müdahale için zaman kalsın."""
    assert ob.SICAKLIK_KRITIK < 105.0
    assert ob.SICAKLIK_UYARI < ob.SICAKLIK_KRITIK


# ───────────────────────────────── vpu_sicakligi ──────────────────────────
def test_vpu_sicakligi_okur():
    class SahteDev:
        def getChipTemperature(self):
            return type("T", (), {"average": 66.5})()

    assert ob.vpu_sicakligi(SahteDev()) == pytest.approx(66.5)


def test_vpu_sicakligi_hatada_None_doner():
    class BozukDev:
        def getChipTemperature(self):
            raise RuntimeError("cihaz kapandı")

    assert ob.vpu_sicakligi(BozukDev()) is None


# ───────────────────────────────── usb_dugum_yolu ─────────────────────────
def test_usb_dugum_yolu_cihaz_yoksa_None(monkeypatch, gercek_usb_dugum_yolu):
    monkeypatch.setattr(ob.glob, "glob", lambda p: [])
    assert ob.usb_dugum_yolu() is None


def test_usb_reset_cihaz_yoksa_False(monkeypatch):
    monkeypatch.setattr(ob, "usb_dugum_yolu", lambda: None)
    assert ob.usb_reset() is False

# ─────────────────── F-A.4 BAĞLANTI DENETİMİ (AST, donanımsız) ─────────────
# 🔴 NEDEN BU TESTLER VAR (2026-08-16 arızası):
# F-A.4 `cihazi_bekle()`yi yazdı, 2 testle sözleşmesini kilitledi, commit
# mesajına *"çökme döngüsü kaldırıldı"* yazdı — ama fonksiyonu HİÇBİR YERDEN
# ÇAĞIRMADI. `grep -rn cihazi_bekle` → yalnız tanım + kendi testleri.
# Sonuç: düzeltme "var" göründü, davranış HİÇ DEĞİŞMEDİ. 16.08 22:44-22:45
# journal'ında eski çökme döngüsü aynen dönüyor (üç tur, `respawn_delay=3.0`).
#
# Mevcut testlerin hiçbiri bunu yakalayamazdı: hepsi `cihazi_bekle`yi DOĞRUDAN
# çağırıp davranışını ölçüyordu. Bir fonksiyonun doğru çalışması, onun
# çağrıldığı anlamına gelmez. Aynı desen §4/5'teki `hedef_bul_p3`te de var.
#
# Denetim AST ile: ROS/depthai kurulu olmayan makinede de koşar
# (`duba_gecis_navigator` import edilemez — `import depthai` ister).
import ast

_PKT = Path(__file__).resolve().parents[1] / "girdap_ida_algi"
_NAV = _PKT / "duba_gecis_navigator.py"
_OB = _PKT / "oak_baglanti.py"


def _govde(dosya, ad):
    for d in ast.walk(ast.parse(dosya.read_text(encoding="utf-8"))):
        if isinstance(d, ast.FunctionDef) and d.name == ad:
            return d
    raise AssertionError(f"{ad} bulunamadı — yeniden mi adlandırıldı?")


def _cagri_satirlari(dugum, ad):
    """`dugum` altında `ad` ile biten çağrıların satır numaraları."""
    n = []
    for c in ast.walk(dugum):
        if isinstance(c, ast.Call):
            f = c.func
            isim = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)
            if isim == ad:
                n.append(c.lineno)
    return n


def test_FA4_BAGLI_MI_cihazi_bekle_gercekten_cagriliyor():
    """🔴 Regresyon kilidi: `cihazi_bekle` çağrısı `pipeline_kur`dan silinirse KIRMIZI."""
    satirlar = _cagri_satirlari(_govde(_OB, "dayanikli_ac"), "cihazi_bekle")
    assert satirlar, (
        "pipeline_kur() `cihazi_bekle()` ÇAĞIRMIYOR — F-A.4 yine yarım. "
        "Cihaz USB'de yokken düğüm çöker ve ~3 sn'de bir respawn döngüsüne girer."
    )


def test_FA4_BAGLI_MI_beklemek_acmadan_ONCE_gelir():
    """Sıra ters olursa bekleme işe yaramaz: önce açmayı dener, yine çöker."""
    govde = _govde(_OB, "dayanikli_ac")
    bekle = _cagri_satirlari(govde, "cihazi_bekle")
    dene = _cagri_satirlari(govde, "acici")
    assert bekle and dene, "iki çağrı da dayanikli_ac içinde olmalı"
    assert min(bekle) < min(dene), (
        f"cihazi_bekle (satır {min(bekle)}) açılış denemesinden "
        f"(satır {min(dene)}) SONRA geliyor — cihaz yokken yine `raise` edilir."
    )


def test_FA4_BAGLI_MI_bekleyis_SESSIZ_degil():
    """`kaydet=` geçilmezse bekleyiş görünmez olur: "sessizlik başarı değildir"."""
    govde = _govde(_OB, "dayanikli_ac")
    for c in ast.walk(govde):
        if not isinstance(c, ast.Call):
            continue
        f = c.func
        # düz çağrı (`cihazi_bekle(...)`) da nitelikli çağrı (`ob.cihazi_bekle(...)`)
        # da geçerli — modül içinden çağrılınca ilki oluyor.
        isim = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)
        if isim == "cihazi_bekle":
            assert any(k.arg == "kaydet" for k in c.keywords), (
                "cihazi_bekle(kaydet=...) verilmemiş — düğüm sessizce bekler, "
                "kimse neyi beklediğini göremez."
            )
            return
    raise AssertionError("cihazi_bekle çağrısı bulunamadı")


def test_FA4_BAGLI_MI_node_loggeri_besliyor():
    """Çağıran taraf: `pipeline_kur(kaydet=...)` gerçekten doldurulmuş mu."""
    agac = ast.parse(_NAV.read_text(encoding="utf-8"))
    cagrilar = [c for c in ast.walk(agac)
                if isinstance(c, ast.Call)
                and getattr(c.func, "id", None) == "pipeline_kur"]
    assert cagrilar, "pipeline_kur hiç çağrılmıyor?"
    assert all(any(k.arg == "kaydet" for k in c.keywords) for c in cagrilar), (
        "pipeline_kur() `kaydet=` olmadan çağrılıyor — bekleyiş journal'a düşmez."
    )

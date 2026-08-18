#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SÜRE ölçümü duvar saatiyle yapılmamalı — kamerasız, ROS'suz (AST).

NEDEN BU TEST VAR (2026-08-09):
Jetson'da **RTC pili yok**; boot'ta saat geride açılıyor — iki kez ölçüldü
(bir kez ~15 saat, bir kez tam bir gün: 08.08'de çekilen kareler 07.08
damgası aldı). Kıyı yordamımız (`docs/veriseti_deniz_oturumu.md` — toplayıcıyla
birlikte 16.08'de kaldırıldı, git geçmişinde) *"`date` ile
saati gözle doğrula, yanlışsa `sudo date -s`"* diyor ve algı servisi boot'ta
otomatik kalkıyor ⇒ **saat, node çalışırken düzeltiliyor.** Tethering takılırsa
NTP de adım atar.

`time.time()` ayarlanabilir bir saattir — kurulu Python'un kendi bildirimi:
    time.get_clock_info("time").adjustable      -> True
    time.get_clock_info("monotonic").adjustable -> False

Duvar saati sıçrarsa (ölçülmüş sonuçlar, düzeltmeden önceki kod):
  • İLERİ  : `pass_bitis_t` deadline'ı anında dolar → geçiş yarıda kesilir (G puanı)
             `son_tespit_t` bayat görünür → o kare geçit kurulmaz
  • GERİ   : `_son_log` gelecekte kalır → `durum_log` TAMAMEN susar (sahada SSH
             yok, tek görünürlük kanalı bu)
             `_son_kayit_t` gelecekte → Dosya-1 kaydı durur (md 4.2, geçersiz
             dosya = 5 ceza puanı, md 5.5.4.3.5)

🔑 Bu test aynı zamanda TERS hatayı da yakalar: Dosya-1'in **görünen zaman
etiketi** (md 4.2 "her frame zaman etiketine sahip olacak") DUVAR saati olmak
ZORUNDA. Kayıt bloğundaki `t` eskiden hem etiketi hem segment süresini
besliyordu; ikiye ayrıldı (`t_duvar` ↔ `time.monotonic()`).

Koşum:  python3 -m pytest girdap_ida_algi/test/test_saat_kaynagi.py -q
"""
import ast
import time
from pathlib import Path

KOK = Path(__file__).resolve().parents[2]

# `time.time()` YALNIZ mutlak-an gerektiren bu bağlamlarda serbest:
#   1) saat güvenilirliği kontrolü (mutlak tarih karşılaştırması)
#   2) adı "duvar" geçen değişkene atama (Dosya-1 görünen etiketi)
IZINLI_CAGRI = {"saat_guvenilir_mi", "oturum_kimligi", "manifest_satiri",
                "localtime", "strftime"}
IZINLI_AD_PARCASI = "duvar"

TARANAN = [
    "girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py",
    "girdap_ida_algi/girdap_ida_algi/oak_baglanti.py",
]


def _time_time_cagrilari(agac):
    """`time.time()` çağrısı olan Call düğümleri."""
    for d in ast.walk(agac):
        if (isinstance(d, ast.Call) and isinstance(d.func, ast.Attribute)
                and d.func.attr == "time"
                and isinstance(d.func.value, ast.Name)
                and d.func.value.id == "time"):
            yield d


def _izinli_mi(agac, cagri):
    """Çağrı, mutlak-an beyaz listesindeki bir bağlamda mı?"""
    for d in ast.walk(agac):
        # (1) izinli bir fonksiyonun argümanı olarak
        if isinstance(d, ast.Call) and isinstance(d.func, ast.Attribute):
            if d.func.attr in IZINLI_CAGRI and any(a is cagri for a in d.args):
                return True
        if isinstance(d, ast.Call) and isinstance(d.func, ast.Name):
            if d.func.id in IZINLI_CAGRI and any(a is cagri for a in d.args):
                return True
        # (2) adında "duvar" geçen bir değişkene atanıyor
        if isinstance(d, ast.Assign) and d.value is cagri:
            for h in d.targets:
                if isinstance(h, ast.Name) and IZINLI_AD_PARCASI in h.id.lower():
                    return True
    return False


def test_kurulu_python_duvar_saati_ayarlanabilir():
    """Testin dayandığı önerme — birinci kaynaktan doğrula, ezberleme."""
    assert time.get_clock_info("time").adjustable is True
    assert time.get_clock_info("monotonic").adjustable is False
    assert time.get_clock_info("monotonic").monotonic is True


def test_sure_olcumu_duvar_saatiyle_yapilmiyor():
    hatalar = []
    for bagil in TARANAN:
        yol = KOK / bagil
        agac = ast.parse(yol.read_text(encoding="utf-8"))
        for cagri in _time_time_cagrilari(agac):
            if not _izinli_mi(agac, cagri):
                hatalar.append(
                    f"{bagil}:{cagri.lineno} time.time() — süre ölçümü ise "
                    f"time.monotonic() olmalı; mutlak an ise adı 'duvar' geçen "
                    f"bir değişkene atayın ya da beyaz listeye ekleyin")
    assert not hatalar, (
        "duvar saati sıçraması sahada sessizce puan kaybettirir:\n  "
        + "\n  ".join(hatalar))


def test_sentinel_degerleri_monotonic_ile_uyumlu():
    """'Hiç olmadı' anlamı 0.0 ile taşınamaz.

    `time.monotonic()` boot'tan itibaren sayar ve açılışta KÜÇÜKTÜR; sentinel
    0.0 kalırsa `simdi - 0.0 < eşik` doğru çıkar ve node açılışta tespit
    olmadan **'taze tespit var'** sanır (`HEDEF_KAYIP_SN`). Duvar saatinde
    (epoch ~1,7e9) bu hata görünmüyordu — saat kaynağı değişince ortaya çıkar.
    """
    kaynak = (KOK / TARANAN[0]).read_text(encoding="utf-8")
    for alan in ("son_tespit_t", "_son_log", "_son_kayit_t", "son_goal_t"):
        assert f"self.{alan} = -math.inf" in kaynak, (
            f"{alan} sentinel'i -math.inf olmalı (0.0 monotonic'te yanlış "
            f"'yeni yapıldı' üretir)")


def test_dosya1_gorunen_etiketi_duvar_saatinde_kaldi():
    """md 4.2: 'her frame zaman etiketine sahip olacak' — monotonic ANLAMSIZ."""
    kaynak = (KOK / TARANAN[0]).read_text(encoding="utf-8")
    assert "t_duvar = time.time()" in kaynak
    assert "time.localtime(t_duvar)" in kaynak, (
        "Dosya-1 etiketi duvar saatinden üretilmeli; monotonic saniyeleri "
        "insan-okunur tarih vermez")

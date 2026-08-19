"""SIFIR VERİM nöbetçisi (19.08.2026) — algı hattı sessizce ölmesin.

🔴 KUSUR: mevcut "SESSİZ RET" alarmı yalnız tespit VARKEN ve reddedilirken
yanar. Tespit HİÇ üretilmediğinde reddedilecek bir şey olmadığı için hiçbir
alarm yanmaz; düğüm `kenar=0 engel=0 | NN 8.0 FPS` diyerek SAĞLIKLI görünür.

ÖLÇÜLDÜ (bant taraması, `/perception/buoys` tespit/kare):
    13.08 sabah   0,9 – 3,4   ✅
    13.08 09:46+  0,00        ❌
    14.08         0,7 – 2,7   ✅
    15-16.08      0,00 – 0,07 ❌
    17.08 04:20   89,26       🔴 kaçak (yanlış pozitif seli)
    17.08 19:33   2,33        ✅
Model çalışıyor ama verim İKİ UÇLU ve **günlerce fark edilmedi**. Sahada SSH
yok, journal tek görünürlük kanalı.

🔑 Bu nöbetçi kusuru DÜZELTMEZ, GÖRÜNÜR kılar: NN nominal hızda dönerken verim
`SIFIR_VERIM_UYARI_S` boyunca sıfır kalırsa gürültülü uyarı basılır.
⚠ Sıfır tek başına kusur kanıtı değildir (kadrajda duba yoksa normaldir) —
uyarı "bak buraya" der, "bozuk" demez.
"""
from __future__ import annotations

import re
from pathlib import Path

KAYNAK = (Path(__file__).resolve().parents[1]
          / "girdap_ida_algi" / "duba_gecis_navigator.py")


def _metin() -> str:
    return KAYNAK.read_text(encoding="utf-8")


def test_SIFIR_VERIM_sabiti_TANIMLI() -> None:
    assert "SIFIR_VERIM_UYARI_S" in _metin(), (
        "sıfır verim uyarı süresi sabiti kayıp"
    )


def test_KOSUL_hem_NN_SAGLIKLI_hem_TESPIT_YOK_ister() -> None:
    """NN zaten yavaşsa ayrı alarm var; bu nöbetçi SAĞLIKLI-ama-verimsiz hâli için."""
    assert re.search(
        r"if self\.olculen_fps >= FPS_UYARI_ESIK and not self\.dubalar", _metin()
    ), "sıfır verim koşulu kayıp ya da değişmiş"


def test_GURULTULU_uyari_basiliyor() -> None:
    m = _metin()
    assert "SIFIR VERİM" in m and "warn(" in m, (
        "uyarı `warn` seviyesinde basılmıyor — sahada journal tek kanal"
    )


def test_TESPIT_GELINCE_sayac_SIFIRLANIR() -> None:
    """'Her zaman yanan alarm alarm değildir' (09.08, `mono_menzil` dersi)."""
    assert re.search(r"else:\s*\n\s*self\._sifir_verim_bas = None", _metin()), (
        "tespit gelince sayaç sıfırlanmıyor — uyarı bir kez yanınca sönmez"
    )

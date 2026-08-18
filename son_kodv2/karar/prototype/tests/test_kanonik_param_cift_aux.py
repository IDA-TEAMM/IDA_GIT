"""Kanonik FC referansı — ÇİFT AUX ANAHTARI nöbetçisi (17.08.2026).

🔴 NEDEN VAR: `Arm: Duplicate Aux Switch Options` pre-arm engelleyicisi
**bir referans dosyası hatasından doğdu**, sahadan değil.

Kanıt zinciri (üçü birbirini doğruluyor):
  1. `docs/fc_REFERANS_TAM_15agustos.param` (15.08 tam döküm) → `RC9_OPTION,0`
  2. `docs/fc_REFERANS_DUZELTME_16agustos_RAPOR.md` "kanonik dosyadan geri
     konanlar" tablosu → `| RC9_OPTION | 0 | 16 |`, yani FC'de **0** olan
     değer kanonik dosyadaki **16** ile EZİLDİ
  3. 17.08 canlı FC okuması → `RC5_OPTION = 16` **ve** `RC9_OPTION = 16`

`fc_referans_uret.py` kanonik dosyayı (`KANONIK_DOSYA = docs/fc_REFERANS.param`)
"saha-doğrulanmış" kabul edip FC'deki değeri onunla değiştiriyor. Kanonikteki
çift kayıt bu yüzden **her referans üretiminde yeniden basılıyordu** — FC'yi
elle düzeltmek yetmez, kaynak düzeltilmeliydi.

ArduPilot aynı sıfır-olmayan `RCn_OPTION` değerini iki kanalda görürse
pre-arm'ı reddeder (`RC_Channels::duplicate_options_exist`). Belirtisi tek
satır bir mesajdır ve `ARMING_CHECK=0` iken HİÇ basılmaz — yani yarışma günü
denetimler geri açılınca tekne sebebi görünmeden arm olmaz.

⛔ GERİ ALINIRSA: kanonik dosyaya ikinci bir `RCn_OPTION=16` (ya da herhangi
bir sıfır-olmayan değerin ikizi) girerse bu test kırmızı yanar. Kural bir
eşik değil ArduPilot'un kendi kısıtı — ayarlanabilir bir sayı DEĞİL.
"""

from __future__ import annotations

import collections
import re
from pathlib import Path

import pytest

_KOK = Path(__file__).resolve().parents[2]

#: `fc_referans_uret.KANONIK_DOSYA` ile AYNI dosya olmalı.
_KANONIK = _KOK / "docs" / "fc_REFERANS.param"

_AUX_DESEN = re.compile(r"^RC(\d+)_OPTION$")


def _aux_secenekleri(yol: Path) -> dict[str, float]:
    """Dosyadaki `RCn_OPTION` adı → değer eşlemesi (sıfırlar dahil)."""
    cikti: dict[str, float] = {}
    for satir in yol.read_text(encoding="utf-8").splitlines():
        satir = satir.strip()
        if not satir or satir.startswith("#"):
            continue
        parca = satir.split(",")
        if len(parca) < 2:
            continue
        ad = parca[0].strip()
        if _AUX_DESEN.match(ad):
            cikti[ad] = float(parca[1])
    return cikti


def test_kanonik_referansta_CIFT_AUX_SECENEGI_YOK() -> None:
    """Aynı sıfır-olmayan RCn_OPTION iki kanalda olamaz (ArduPilot kısıtı)."""
    if not _KANONIK.exists():
        pytest.skip(f"kanonik dosya yok: {_KANONIK}")

    secenekler = _aux_secenekleri(_KANONIK)
    assert secenekler, f"{_KANONIK.name} içinde hiç RCn_OPTION yok — desen bozulmuş"

    kanallar = collections.defaultdict(list)
    for ad, deger in secenekler.items():
        if deger != 0.0:
            kanallar[deger].append(ad)

    cift = {d: sorted(a) for d, a in kanallar.items() if len(a) > 1}
    assert not cift, (
        "kanonik referansta ÇİFT aux anahtarı var → FC'ye yüklenince "
        f"'Arm: Duplicate Aux Switch Options' pre-arm'ı doğar: {cift}"
    )


def test_RC9_OPTION_kanonikte_SIFIR_kaldi() -> None:
    """17.08'de düzeltilen tam değer donduruluyor (16 → 0).

    Ayrı test, çünkü yukarıdaki genel kural `RC5`'i sıfırlayıp `RC9`'u 16
    bırakmakla da sağlanır — ama saha kanıtı (15.08 dökümü + 16.08 raporunun
    "FC'de" kolonu) gerçek anahtarın **RC5** olduğunu söylüyor.
    """
    if not _KANONIK.exists():
        pytest.skip(f"kanonik dosya yok: {_KANONIK}")

    secenekler = _aux_secenekleri(_KANONIK)
    assert secenekler.get("RC9_OPTION") == 0.0, (
        "RC9_OPTION kanonikte 0 olmalı — 16.08 referans üretimi burayı "
        "16 yapıp çift anahtar engelleyicisini doğurmuştu"
    )
    assert secenekler.get("RC5_OPTION") == 16.0, (
        "RC5_OPTION = 16 (AUTO) saha-doğrulanmış gerçek anahtar; "
        "değiştiyse bu düzeltmenin dayanağı yeniden okunmalı"
    )

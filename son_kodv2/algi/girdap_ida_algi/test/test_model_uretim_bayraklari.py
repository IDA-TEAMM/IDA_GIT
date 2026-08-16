"""Blob üretim betiği ZORUNLU bayrakları taşıyor mu? (donanım/ağ GEREKMEZ)

🔴 NEDEN VAR — 17.08.2026'da bulundu, canlı bir açıktı:

`scripts/model_uret.sh` OpenVINO'ya şunu geçiyordu:
    optimizer_params=["--scale_values=...", "--mean_values=..."]
**`--reverse_input_channels` YOKTU.** Oysa aynı bayrak
  · `models/README.md:66`      → "🔴 ZORUNLU"
  · `scripts/egitim/OKU.md:30` → "Export'ta --reverse_input_channels"
  · `scripts/egitim/3_kabul_testi.py` → "10.08 UYGULANDI"
diye üç ayrı yerde şart koşulmuştu. `git log -S "reverse_input_channels" --
scripts/model_uret.sh` **boş döndü** ⇒ bayrak bu betiğe hiç girmemiş ⇒ bu
betikle üretilen HER blob kanal takassız çıkıyordu.

MEKANİZMA: model **RGB** bekler (ultralytics `augment.py` → `img[::-1]`),
`ColorCamera` **BGR** gönderir (`setColorOrder(BGR)`), blob'un içinde çeviren
yoktur. Takas derleme anında ilk konvolüsyon ağırlıklarına gömülür (çalışma
maliyeti 0 ms). Bayrak düşerse: **recall %96,8 → %43,0 (ölçüldü, 600 kare)**
ve **hiçbir hata basılmaz** — node açılır, FPS normaldir, tespitler gelir.

NEDEN BU TEST, `model_dogrula.py` DEĞİL: doğrulayıcı blob'u açıp shave/giriş/
sınıf adlarını okuyor ama **kanal sırasını göremiyor** — takas ağırlıkların
içinde. `config.json`'daki `reverse_channels` alanı da işe yaramıyor: bilinen
İYİ blob'da (ep87, `31fb0348…`, bayrakla derlendi) da **`None`** çıkıyor
(17.08'de iki config yan yana koyulup doğrulandı). Yani çıktıdan anlaşılmıyor
⇒ tek koruma **girdiyi**, yani üretim komutunu dondurmaktır. Bu testin işi bu.

⛔ GERİ ALINIRSA: bir sonraki model üretiminde bayrak yine düşer ve sahada
yalnız "dubaların yarısını göremiyoruz" diye fark edilir — o da yarışma günü.
"""
from __future__ import annotations

import os
import re

#: `.../algi/girdap_ida_algi/test/bu_dosya.py` → üç seviye yukarısı `algi/`.
#: (İlk yazımda iki seviye kalmıştı; `test_betik_var` kontrol grubu anında
#: yakaladı — yol kayarsa testin sessizce "yeşil" vermesini o engelliyor.)
_BETIK = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "scripts", "model_uret.sh",
)

#: Blob'u sahada kullanılamaz ya da sessizce kötü yapan üretim bayrakları.
ZORUNLU = {
    "--reverse_input_channels": "kanal takası — yoksa recall %96,8 → %43,0",
    "--scale_values": "0-255 → 0-1 ölçekleme",
    "--mean_values": "ortalama çıkarma (0)",
}


def _metin() -> str:
    with open(_BETIK, encoding="utf-8") as f:
        return f.read()


def test_betik_var():
    """Kontrol grubu: yol kayarsa test boş kümede 'yeşil' vermesin."""
    assert os.path.exists(_BETIK), f"üretim betiği bulunamadı: {_BETIK}"
    assert "blobconverter" in _metin(), "betik blobconverter çağırmıyor — yol yanlış?"


def test_ZORUNLU_optimizer_bayraklari_yerinde():
    """Üçü de `optimizer_params` içinde geçmeli."""
    s = _metin()
    eksik = [f"{b} ({n})" for b, n in ZORUNLU.items() if b not in s]
    assert not eksik, (
        "model_uret.sh ZORUNLU üretim bayrağı taşımıyor — bu betikle üretilen "
        "blob sahada SESSİZCE bozuk olur:\n  " + "\n  ".join(eksik)
    )


def test_bayrak_optimizer_params_ICINDE_yorumda_degil():
    """🪤 Bayrak yalnız yorum satırında geçiyorsa OpenVINO'ya GİTMEZ.

    `--reverse_input_channels` kelimesi dosyada 'ZORUNLU' diye bir açıklamada
    da geçebilir; asıl soru listeye girip girmediği. `optimizer_params=[...]`
    bloğunu ayıklayıp orada arıyoruz — yorum satırları elenerek.
    """
    s = _metin()
    i = s.find("optimizer_params")
    assert i >= 0, "optimizer_params listesi bulunamadı"
    # ⚠ Basit `\[(.*?)\]` regex'i BURADA ÇALIŞMAZ: değerlerin kendisi köşeli
    # parantez içeriyor (`--scale_values=[255,255,255]`) ⇒ tembel eşleşme ilk
    # iç `]`'de kesiyor ve bayrağı göremiyordu (17.08'de bu test onu yakaladı).
    # Parantez DENGELEYEREK listenin tamamını al.
    bas = s.index("[", i)
    derinlik, son = 0, None
    for k in range(bas, len(s)):
        if s[k] == "[":
            derinlik += 1
        elif s[k] == "]":
            derinlik -= 1
            if derinlik == 0:
                son = k
                break
    assert son is not None, "optimizer_params listesi kapanmıyor"
    blok = "\n".join(
        satir for satir in s[bas:son].splitlines()
        if not satir.lstrip().startswith("#")
    )
    assert "--reverse_input_channels" in blok, (
        "--reverse_input_channels yalnız yorumda/belgede geçiyor, "
        "optimizer_params LİSTESİNE girmiyor ⇒ OpenVINO'ya hiç gitmez"
    )


def test_shave_sabiti_4_kaliyor():
    """Fazla shave = blob HİÇ yüklenmez (sert ret). Ölçüm: 4→19,9 · 8→14,3 FPS."""
    s = _metin()
    assert "shaves=shave" in s, "shave parametresi elden geçmiş"
    assert re.search(r"SHAVE\s*=\s*\{?\$?\{?[^}]*?4", s) or "SHAVE=4" in s, \
        "SHAVE varsayılanı 4 değil — blobconverter varsayılanı 8'dir ve YÜKLENMEZ"

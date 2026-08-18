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


def _liste_blogu(s: str) -> str:
    """OpenVINO'ya GERÇEKTEN giden listenin gövdesi (yorum satırları elenmiş).

    İki yazım da desteklenir:
      · `optimizer_params=[...]`            (doğrudan)
      · `params = [...]` + `optimizer_params=params`  (dolaylı — 17.08'den beri;
        aynı liste PROVENANS.json'a da gittiği için tek kaynak olmalı)
    🪤 Dolaylı yazıma geçilince bu test önce KIRILDI ve doğru yaptı: değişkenin
    içeriğine bakmasaydı, bayrak listeden çıkarılsa bile yeşil verecekti.
    """
    i = s.find("optimizer_params")
    assert i >= 0, "optimizer_params bulunamadı"
    kuyruk = s[i + len("optimizer_params"):].lstrip(" =")
    if not kuyruk.startswith("["):                      # dolaylı: değişken adı
        ad = re.match(r"[A-Za-z_][A-Za-z0-9_]*", kuyruk)
        assert ad, "optimizer_params'a ne verildiği çözülemedi"
        i = s.find(f"{ad.group(0)} = [")
        assert i >= 0, (
            f"optimizer_params={ad.group(0)} deniyor ama '{ad.group(0)}' "
            "listesi betikte tanımlı değil"
        )
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
    return "\n".join(
        satir for satir in s[bas:son].splitlines()
        if not satir.lstrip().startswith("#")
    )


def test_bayraklar_LISTENIN_ICINDE_yorumda_degil():
    """🪤 Bayrak yalnız yorum satırında geçiyorsa OpenVINO'ya GİTMEZ.

    🔴 17.08.2026 — bu test ÜÇ bayrağı birden denetliyor, çünkü o gün
    dağıtılmış blob'da **`--scale_values` düşmüştü** (reverse vardı). Yani
    "listede bir bayrak var" görmek yetmiyor; biri düşünce diğerinin varlığı
    yanlış güven veriyor. Ölçüm: ölçek düşerse ağa 0..255 girer ve karede 300
    uydurma tespit çıkar (max_det doyumu) ⇒ /perception/buoys çöp.
    """
    blok = _liste_blogu(_metin())
    eksik = [f"{b} ({n})" for b, n in ZORUNLU.items() if b not in blok]
    assert not eksik, (
        "Bayrak yalnız yorumda/belgede geçiyor, OpenVINO'ya giden LİSTEDE yok:\n  "
        + "\n  ".join(eksik)
    )


def test_GIRIS_SABIT_YAZILMAZ_deploydan_turetilir():
    """🔴 Giriş boyutu `NN_GIRIS`'ten okunmalı — sabit yazılırsa bayatlar.

    Yaşandı: 12.08'de deploy 416 → 512 oldu, bu betikte `GIRIS=416` KALDI
    (17.08'de bulundu). Bu betikle üretilen blob 416 olur, preview 512
    gönderir ⇒ node'un kendi uyarısıyla "çöp tespit", hata basılmadan.
    Kural (12.08 dersi): **koruma, koruduğu değerden türetilmeli.**
    """
    s = _metin()
    assert re.search(r"GIRIS=.*NN_GIRIS|NN_GIRIS.*\n.*GIRIS=", s) or \
        ("NN_GIRIS" in s and re.search(r'GIRIS="\$\(', s)), \
        "GİRİŞ boyutu duba_gecis_navigator.NN_GIRIS'ten türetilmiyor"
    assert not re.search(r"^\s*GIRIS=[0-9]+", s, re.M), \
        "GİRİŞ boyutu SABİT yazılmış — deploy değişince sessizce bayatlar"


def test_PROVENANS_json_uretiliyor():
    """Üretim, blob'un sha256'sını PROVENANS.json'a yazmalı.

    🔴 NEDEN: derleme parametreleri blob çıktısından okunamıyor. 17.08'de
    dağıtılmış bir blob'un parametreleri ancak blobconverter cache hash'i
    kaba kuvvetle yeniden üretilerek anlaşıldı. Kayıt üretim anında düşmezse
    "bu blob nasıl derlendi" sorusunun cevabı yine kalmaz.
    """
    s = _metin()
    assert "PROVENANS.json" in s, "üretim betiği PROVENANS.json yazmıyor"
    assert "blob_sha256" in s, "PROVENANS.json'a blob sha256'sı yazılmıyor"
    assert '"optimizer_params": params' in s, (
        "provenans, derlemede kullanılan listeyi DEĞİL kendi kopyasını yazıyor "
        "— ikisi ayrışırsa provenans yanlış güven verir"
    )


def test_shave_sabiti_4_kaliyor():
    """Fazla shave = blob HİÇ yüklenmez (sert ret). Ölçüm: 4→19,9 · 8→14,3 FPS."""
    s = _metin()
    assert "shaves=shave" in s, "shave parametresi elden geçmiş"
    assert re.search(r"SHAVE\s*=\s*\{?\$?\{?[^}]*?4", s) or "SHAVE=4" in s, \
        "SHAVE varsayılanı 4 değil — blobconverter varsayılanı 8'dir ve YÜKLENMEZ"

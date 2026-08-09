"""
Girdap İDA — H4/H5: algı düşerse ne oluyor (ROS'SUZ, kaynak denetimi).

`planning_node` rclpy gerektirir → burada import EDİLEMEZ; kaynak metni
okunur. Bu bir "davranış" testi değil, **sözleşme** testi: iki emniyet
mekanizmasının kaldırılmasını/gerilemesini imkânsız kılar.

---

🔴 **H4 — TEK YÖNLÜ MANDAL (2026-08-09 taraması).**

Eski kod:
```python
if self._use_classified and self._classified_seen:
    return          # ham LiDAR yolu KALICI olarak susar
```
Sınıflı akış **bir kez** aktıysa ham `/perception/obstacle_map` yolu bir
daha hiç işlenmiyordu. Koşu ortasında kamera/OAK/füzyon düşerse (F-P.22'de
gerçek donanımda yaşandı):

    classified susar → ham yol mandal yüzünden kapalı → engel torbası DONAR
    → `_last_obstacle_t` güncellenmez → 2 sn sonra F-P.2 thrust'ı sıfırlar
    → **tekne kalıcı durur**

…oysa LiDAR sapasağlam: sınıfsız kol gerçek parkurda P1'i **53,75 puanla**
bitiriyor (§0.20c, 3/3 tohum). Çalışan bir yedek varken kapatılmıştı.

---

🔴 **H5 — "AKIYOR AMA HEP BOŞ".**

F-P.2 bekçisi mesajın **varış zamanına** bakar, **içeriğine** bakmaz. Algı
her karede boş dizi yayınlarsa her şey sağlıklı görünür ve araç **sıfır
engelle** sürer. Varsayımsal değil: B0/F5.1'de LiDAR z filtresi yanlış
çerçevede uygulanınca `obstacle_map` tam da böyle sürekli boş geliyordu
(§0.2b) ve tek satır uyarı basılmıyordu.
"""

from __future__ import annotations

import ast
from pathlib import Path

_NODE = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
    / "girdap_decision" / "planning_node.py"
)
_KAYNAK = _NODE.read_text(encoding="utf-8")


def _govde(ad: str) -> str:
    """Adı verilen metodun kaynak metni."""
    agac = ast.parse(_KAYNAK)
    for dugum in ast.walk(agac):
        if isinstance(dugum, ast.FunctionDef) and dugum.name == ad:
            return ast.get_source_segment(_KAYNAK, dugum) or ""
    raise AssertionError(f"{ad} bulunamadı")


# ----------------------------------------------------------------- H4

def test_H4_mandal_TEK_YONLU_degil() -> None:
    """Ham LiDAR yolu `_classified_seen` ile KALICI kapatılmamalı."""
    govde = _govde("_on_obstacles")
    assert "self._classified_seen" not in govde.split('"""')[-1], (
        "ham LiDAR yolu hâlâ `_classified_seen` mandalıyla kapatılıyor — "
        "füzyon düşerse tekne kalıcı durur (H4)"
    )
    assert "_classified_taze()" in govde, (
        "mandal ölçütü TAZELİK olmalı: sınıflı akış susunca ham yol devralsın"
    )


def test_H4_tazelik_esigi_FP2_butcesinden_TURER() -> None:
    """Eşik ayrı bir ayar DEĞİL — F-P.2 durdurma bütçesinden türemeli.

    Yedeğin devreye girmesi için bütçenin yarısı elde kalmalı; elle yazılmış
    bir sayı, `obstacle_timeout_s` değişince sessizce tutarsızlaşırdı.
    """
    assert "self._obstacle_timeout / 2.0" in _KAYNAK, (
        "H4 eşiği F-P.2 bütçesinden türetilmiyor"
    )


def test_H4_tazelik_saati_SINIFLI_akista_besleniyor() -> None:
    """`_last_classified_t` güncellenmezse yedek hemen devreye girer."""
    govde = _govde("_on_classified")
    assert "self._last_classified_t = self._now()" in govde


def test_H4_yedege_dusus_LOGLANIYOR() -> None:
    """Sahadaki tek görünürlük kanalı: kapı takibi sessizce kaybolmasın."""
    govde = _govde("_sinifsiz_yola_dusuldu")
    assert "KAPI TAKİBİ" in govde or "KAPI TAKIBI" in govde
    assert "_sinifsiz_uyarildi" in govde, "log seli koruması yok"


def test_H4_akis_donunce_uyari_SIFIRLANIYOR() -> None:
    """Kamera geri gelir sonra tekrar düşerse ikinci kez uyarılmalı."""
    assert "self._sinifsiz_uyarildi = False" in _govde("_on_classified")


# ----------------------------------------------------------------- H5

def test_H5_bos_akis_kapani_VAR_ve_IKI_YOLDA_da_cagriliyor() -> None:
    """Kapan yalnız bir yolda olursa diğer yolda arıza yine sessiz kalır."""
    assert "def _bos_akis_denetle" in _KAYNAK, "H5 kapanı yok"
    assert "_bos_akis_denetle(" in _govde("_on_obstacles"), "sınıfsız yolda yok"
    assert "_bos_akis_denetle(" in _govde("_on_classified"), "sınıflı yolda yok"


def test_H5_esigi_FP2den_TURER_ve_bekciden_UZUN() -> None:
    """H5 "hep boş"u ölçer, F-P.2 "sustu"yu — boşluk geçici olabilir.

    Eşik bekçininkinden kısa olsaydı, dubaların arasından çıktığımız her
    anda yanlış alarm basardı.
    """
    assert "self._obstacle_timeout * 5.0" in _KAYNAK, (
        "H5 eşiği F-P.2 bütçesinden türetilmiyor ya da bekçiden kısa"
    )


def test_H5_DOLU_akista_sayac_sifirlaniyor() -> None:
    """Tek dolu kare bile geldiyse alarm sıfırlanmalı (yanlış pozitif yok)."""
    govde = _govde("_bos_akis_denetle")
    assert "if n_engel > 0:" in govde
    assert "self._son_dolu_akis_t = now" in govde


def test_H5_uyari_B0_arizasini_ISARET_ediyor() -> None:
    """Uyarı metni sahada NEREYE bakılacağını söylemeli (B0/F5.1 kökü)."""
    govde = _govde("_bos_akis_denetle")
    assert "B0" in govde or "mount_z" in govde, (
        "uyarı yalnız 'boş' diyor, sebebe yönlendirmiyor"
    )


def test_H5_bootta_yanlis_alarm_YOK() -> None:
    """Açılışta henüz hiç dolu kare gelmemişken uyarı basılmamalı."""
    govde = _govde("_bos_akis_denetle")
    assert "if self._son_dolu_akis_t is None:" in govde

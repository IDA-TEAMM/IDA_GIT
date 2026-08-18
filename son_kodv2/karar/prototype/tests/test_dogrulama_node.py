# -*- coding: utf-8 -*-
"""DOĞRULAMA İZLEYİCİSİ — düğüm seviyesi sözleşmeleri.

Düğümün kendisi rclpy istiyor; kural mantığı ZATEN ayrı test ediliyor.
Buradaki testler **düğümün sistemi bozmadığını** ve tablonun tutarlı
olduğunu donduruyor.
"""
from __future__ import annotations

import ast
import io
import pathlib

import pytest

_KOK = pathlib.Path(__file__).resolve().parents[2]
_NODE = _KOK / "ros2_ws/src/girdap_decision/girdap_decision/dogrulama_node.py"
_KAYNAK = io.open(_NODE, encoding="utf-8").read()
_AGAC = ast.parse(_KAYNAK)


def test_izleyici_HICBIR_SISTEM_TOPICINE_yazmaz():
    """🔑 Gözlemci sınanan sistemi bozamaz — yalnız kendi kanalına yayın.

    Literatürün uyarısı: 'RV alt sistemindeki hatalar görevin tamamını
    tehdit eder.' En basit koruma: sistem topic'ine hiç yayıncı açmamak.
    """
    yayinlar = []
    for n in ast.walk(_AGAC):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "create_publisher" and len(n.args) >= 2):
            t = n.args[1]
            if isinstance(t, ast.Constant):
                yayinlar.append(t.value)
            else:                       # f-string: /girdap/dogrulama/{ad}
                yayinlar.append(ast.unparse(t))
    assert yayinlar, "hiç yayıncı bulunamadı — test kendini doğrulayamıyor"
    for y in yayinlar:
        assert "dogrulama" in y, f"sistem topic'ine yayın YASAK: {y}"


def test_izleyici_ABONE_oldugu_topicler_KARAR_ve_ALGI_iceriyor():
    """Göl tam sistemi kapsamalı — tek taraf değil."""
    assert "/girdap/fusion/odom" in _KAYNAK       # karar
    assert "/girdap/control/thrust" in _KAYNAK    # karar
    assert "/girdap/mission/state" in _KAYNAK     # karar
    assert '"/perception/buoys"' in _KAYNAK       # algı (bizim düğüm)
    assert '"/perception/obstacle_map"' in _KAYNAK  # algı/LiDAR
    # Tabloda GERÇEKTEN abone olunuyor mu (yalnız metinde geçmesi yetmez).
    # 🪤 İlk yazımda `split("_abonelikler")[1]` kullandım; o ilk eşleşmeyi
    # (ÇAĞRI yerini) alıyor, metot gövdesini değil ⇒ test kendi tezgâhından
    # dolayı yanlış sonuç veriyordu. `ast` ile tanım bulunuyor.
    # 🪤 İKİNCİ TUZAK: alt-dizgi araması "/perception/buoys_3d" yüzünden
    # "/perception/buoys" varmış gibi gösteriyordu. Sözlük ANAHTARLARI
    # tam eşleşmeyle çıkarılıyor.
    tanim = next(n for n in ast.walk(_AGAC)
                 if isinstance(n, ast.FunctionDef) and n.name == "_abonelikler")
    anahtarlar = {k.value for n in ast.walk(tanim) if isinstance(n, ast.Dict)
                  for k in n.keys
                  if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    assert anahtarlar, "abonelik sözlüğü çıkarılamadı — test kendini doğrulayamıyor"
    for t in ("/girdap/fusion/odom", "/perception/buoys",
              "/girdap/control/thrust", "/girdap/mission/state",
              "/perception/obstacle_map"):
        assert t in anahtarlar, f"{t} abonelik tablosunda YOK"


def test_izleyici_TEK_YONLU_SAAT_kullanir():
    """§0.61: geçen süre duvar saatinden okunmaz.

    Sim zamanında `time.monotonic` `/clock`'u izlemez ⇒ izleyici SAHTE
    bayatlık üretir. Bu, poz tamponunda bulunan saat tabanı hatasının aynı
    sınıfı — ekibin sözleşme testi bu düğümü de yakaladı ve düzeltildi.
    """
    assert "bayatlik_saati" in _KAYNAK
    # ⚠ Yorumda bile geçmemeli: ekibin sözleşme testi kaynağı düz metin
    # olarak tarıyor (yorum/kod ayrımı yapmıyor) ⇒ yorumdaki bir anma bile
    # yanlış pozitif üretir. Küçük bir açık, ama kurala uymak bedava.
    assert "time.monotonic" not in _KAYNAK


def test_damga_yasi_ROS_Time_aritmetigiyle_olculur():
    """Damga yaşı = duvar saati − duvar saatli damga (DOĞRU).
    Geçen süre = tek yönlü saat (AYRI iş). İkisi karıştırılırsa poz
    tamponundaki 57 yıllık taban hatası tekrarlanır."""
    assert "RclTime.from_msg" in _KAYNAK
    assert "get_clock().now().nanoseconds" not in _KAYNAK


def test_veri_YOKKEN_ihlal_yok_DEMEZ_STALE_der():
    """'Ölçemedim' ile 'sorun yok' karıştırılırsa göl sessizce yeşil yanar."""
    assert "STALE" in _KAYNAK
    assert _KAYNAK.count("DiagnosticStatus.STALE") >= 2


def test_ABORT_kurallari_ERROR_digerleri_WARN():
    """Guard/değişmez/abort ayrımı tepkiye yansımalı."""
    assert "DiagnosticStatus.ERROR if kural.tur is Tur.ABORT" in _KAYNAK


def test_her_kural_MARJ_topicine_de_yayinlaniyor():
    """rqt_plot ile 'ne kadar payımız kaldı' çizilebilsin."""
    assert "/girdap/dogrulama/{k.ad}" in _KAYNAK


def test_C1_yalniz_GOREV_AKTIFKEN_degerlendirilir():
    """🔴 18.08 canlı koşumda bulundu: görev TAMAMLANDI'ya geçince sıfır itki
    DOĞRU davranıştır, ama C1 −25 s ile İHLAL bastı.

    Yanlış pozitif gerçek ihlal kadar zararlıdır — her koşuda yanan alarm
    alarm değildir (09.08 `mono_menzil` dersi)."""
    assert "PARKUR1" in _KAYNAK and "PARKUR2" in _KAYNAK and "PARKUR3" in _KAYNAK
    i = _KAYNAK.index("def _itki_sifir")
    blok = _KAYNAK[i:i + 900]
    assert "mission/state" in blok, "C1 görev durumuna bakmıyor"
    assert "return None" in blok, "görev aktif değilken muaf tutulmuyor"


def test_B1_dongu_periyodu_SAHADAKI_metrikle_AYNI():
    """🔑 B1 `/girdap/control/thrust` yayın periyodunu ölçmeli.

    KAR-11 sahada tam bu metriği kullandı (117 ms → 1.062 ms, 9,1× bozulma).
    Farklı bir ölçüt seçilseydi gölün bulduğu ile sahanın bulduğu
    KIYASLANAMAZDI — ve "sistem kaldırmıyor" sorusu gölde hiç görünmezdi.
    """
    assert "butce.B1" in _KAYNAK, "B1 izleyiciye bağlı değil"
    i = _KAYNAK.index("def _dongu_periyodu")
    blok = _KAYNAK[i:i + 900]
    assert "/girdap/control/thrust" in blok, "B1 yanlış topic'i ölçüyor"
    assert "1.0 / 10.0" in blok, "nominal 10 Hz bütçesi verilmemiş"


def test_setup_py_entry_pointu_KAYITLI():
    sp = io.open(_KOK / "ros2_ws/src/girdap_decision/setup.py", encoding="utf-8").read()
    assert "dogrulama_node = girdap_decision.dogrulama_node:main" in sp


def test_baglanti_tablosu_KURAL_MOTORUNDAN_besleniyor():
    """Kurallar burada yeniden TANIMLANMAMALI — tek kaynak `prototype/dogrulama`."""
    assert "from prototype.dogrulama import butce, canlilik, fizik, sozlesme" in _KAYNAK
    for mod in ("fizik.F1", "sozlesme.S1", "canlilik.C1"):
        assert mod in _KAYNAK, f"{mod} bağlanmamış"


def test_dugum_DERLENIYOR():
    compile(_KAYNAK, str(_NODE), "exec")

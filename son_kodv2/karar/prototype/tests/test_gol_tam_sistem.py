# -*- coding: utf-8 -*-
"""GÖL KAPSAMI — dağıtımda koşan her düğüm gölde de koşabilmeli.

🔴 Bu dosyanın varlık sebebi: 18.08'de ölçüldü, göl dağıtımdaki **17**
düğümün yalnız **5**'ini koşturuyordu. Gölde hiç koşmayan yedi gerçek düğüm
vardı ve ikisi doğrudan **teslim dosyası** üretiyor (md 4.2 — eksik dosya
başına 5 ceza puanı). Yani teslim zinciri uçtan uca hiç sınanmamıştı.

Test, kapsamı **kod ile** bağlar: dağıtıma yeni bir düğüm eklenip göle
eklenmezse burası kırmızı yanar.
"""
from __future__ import annotations

import io
import pathlib
import re

import pytest

_KOK = pathlib.Path(__file__).resolve().parents[2]
_LAUNCH = _KOK / "ros2_ws/src/girdap_decision/launch/hardware.launch.py"
_GOL = _KOK / "scripts/gol_kos.sh"
_GOL_METIN = io.open(_GOL, encoding="utf-8").read()

#: Gölde koşMAması GEREKEN düğümler — sanal göl bunların yerine geçer.
#: (Donanım sürücüleri + MAVROS + mock; gerekçe: gerçek donanım yok.)
_SAHTELENEN = frozenset({
    "livox_driver_node", "oakd_driver_node", "mavros_node",
    "mock_sensors", "static_transform_publisher",
    "kamera_kayit_node",   # gerçek kamera karesi ister (OAK-D); FAZ 7
})


def _dagitim_dugumleri() -> set:
    m = io.open(_LAUNCH, encoding="utf-8").read()
    return set(re.findall(r'executable="([a-z_0-9]+)"', m))


def _gol_dugumleri() -> set:
    return set(re.findall(r"girdap_decision (\w+)", _GOL_METIN))


def test_dagitimdaki_HER_dugum_golde_de_kosuyor():
    """🔑 Kapsam kapısı — göl 'tam sistem' iddiasını taşımalı."""
    eksik = _dagitim_dugumleri() - _gol_dugumleri() - _SAHTELENEN
    assert not eksik, (
        f"Bu düğümler DAĞITIMDA var ama GÖLDE yok: {sorted(eksik)} — "
        "göl tam sistemi koşturmuyor demektir"
    )


def test_teslim_ureticileri_GOLDE_kosuyor():
    """Dosya-1/2/3 zinciri sınanmazsa 5 ceza puanı sahada öğrenilir."""
    g = _gol_dugumleri()
    assert "telemetry_node" in g, "Dosya-2 (telemetri CSV) üreticisi gölde yok"
    assert "local_map_node" in g, "Dosya-3 (yerel harita) üreticisi gölde yok"


def test_gercek_ALGI_zinciri_golde_kosabiliyor():
    """`sanal_gol` algıyı baypas ediyordu; `sahte_ham_sensor` zinciri kapatır."""
    assert "sahte_ham_sensor" in _GOL_METIN
    g = _gol_dugumleri()
    assert "perception_lidar_node" in g
    assert "perception_fusion_node" in g


def test_ALGI_acikken_sanal_gol_ciktisi_REMAP_ediliyor():
    """🔑 İKİ ÜRETİCİ TUZAĞI.

    `sanal_gol` ideal algıyı doğrudan `/perception/*`'a basıyor. Gerçek algı
    zinciri açılınca o topic'leri `perception_lidar/fusion` üretecek. Remap
    olmazsa iki üretici aynı topic'e basar ⇒ füzyon hangisini aldığını
    bilemez ve ölçüm anlamsızlaşır.

    ⚠ Remap `sanal_gol`e konmalı — `sahte_ham_sensor`e DEĞİL (o zaten
    `/gercek/*` dinliyor; ters kurulum sessizce hiçbir şey değiştirmezdi).
    """
    assert "-r /perception/obstacle_map:=/gercek/obstacle_map" in _GOL_METIN
    assert "-r /perception/classified_obstacles:=/gercek/classified_obstacles" in _GOL_METIN
    # Remap sanal_gol çağrısında olmalı
    i = _GOL_METIN.index("basla sanal_gol")
    assert "$SG_REMAP" in _GOL_METIN[i:i + 200], "remap sanal_gol'e bağlanmamış"
    # sahte_ham_sensor'de /gercek→/gercek gibi ölü remap OLMAMALI
    j = _GOL_METIN.index("basla ham_sensor")
    assert "-r /gercek" not in _GOL_METIN[j:j + 200], "ölü remap kalmış"


def test_P3_renk_kapisi_GOLDE_acilabiliyor():
    """`kamikaze_param_node` olmadan hedef rengi HİÇ yayınlanmaz ⇒
    `p3_bekleniyor` hep False ⇒ FSM PARKUR3'e hiç geçmez."""
    assert "kamikaze_param_node" in _gol_dugumleri()


def test_dogrulama_izleyicisi_GOLDE_acilabiliyor():
    assert "dogrulama_node" in _gol_dugumleri()


# ────────────────────── varsayılan davranış korunuyor mu ──────────────────
@pytest.mark.parametrize("salter", [
    "GIRDAP_GOL_ALGI", "GIRDAP_GOL_TESLIM",
    "GIRDAP_GOL_P3", "GIRDAP_GOL_IZLEYICI",
])
def test_yeni_katmanlar_VARSAYILAN_KAPALI(salter):
    """§0.8a: yeni yetenek ölçülmeden varsayılan olmaz.

    `${X:-0}` deseni ⇒ şalter verilmezse 0 ⇒ eski davranış BİT BİREBİR.
    """
    assert f'"${{{salter}:-0}}" = "1"' in _GOL_METIN, (
        f"{salter} varsayılan kapalı değil — eski koşumlar sessizce değişir")


def test_katmanlar_AYRI_salterlerde():
    """Tek 'hepsi açık' şalteri arıza ayrıştırmayı imkânsız kılardı."""
    for s in ("GIRDAP_GOL_ALGI", "GIRDAP_GOL_TESLIM",
              "GIRDAP_GOL_P3", "GIRDAP_GOL_IZLEYICI"):
        assert _GOL_METIN.count(s) >= 2, f"{s} bağımsız kullanılmıyor"


def test_TAM_salteri_hepsini_aciyor():
    # 🪤 `split(...)[1]` ilk eşleşmeyi (YORUMU) alıyordu — bu oturumda
    # üçüncü kez aynı tuzak. Şalterin GERÇEK kullanıldığı satırdan başla.
    # TAM bloğu, şalterleri ATAYAN yerdir (koşulda okuyan yer değil).
    # `GIRDAP_GOL_TAM` artık remap koşulunda da geçiyor ⇒ ilk eşleşmeye
    # güvenmek yine yanlış blok verirdi.
    # 🪤 ÜÇÜNCÜ TUZAK aynı testte: `index("}")` bloğun sonunu değil
    # `${GIRDAP_GOL_TAM:-0}` içindeki süslü parantezi buluyordu. Blok sonu
    # SATIR BAŞINDAKİ `}` ile aranır.
    i = _GOL_METIN.index('[ "${GIRDAP_GOL_TAM:-0}" = "1" ] && {')
    blok = _GOL_METIN[i:_GOL_METIN.index("\n}", i)]
    for s in ("GIRDAP_GOL_ALGI", "GIRDAP_GOL_TESLIM",
              "GIRDAP_GOL_P3", "GIRDAP_GOL_IZLEYICI"):
        assert s in blok, f"GIRDAP_GOL_TAM {s}'i açmıyor"


def test_sanal_gol_PARAMETRELER_okunmadan_ONCE_tanimli():
    """🪤 rclpy parametreyi tanımlamadan okursan `ParameterNotDeclaredException`
    fırlatır ve düğüm AÇILIŞTA ölür. Sanal göl ölünce tüm zincir "poz hiç
    gelmedi" der ve **sebep gizlenir** — 18.08'de tam bu oldu.

    Test her `self.X = ...get_parameter("ad")` okumasının, o adın
    `declare_parameter` satırından SONRA geldiğini doğrular.
    """
    import re
    kaynak = io.open(_KOK / "scripts/sanal_gol.py", encoding="utf-8").read()
    tanim = {m.group(1): m.start()
             for m in re.finditer(r'declare_parameter\(\s*"([^"]+)"', kaynak)}
    for m in re.finditer(r'get_parameter\(\s*"([^"]+)"\s*\)', kaynak):
        ad = m.group(1)
        assert ad in tanim, f"{ad} hiç declare edilmemiş"
        assert tanim[ad] < m.start(), (
            f"{ad} TANIMLANMADAN okunuyor (satır sırası ters) — "
            "düğüm açılışta ParameterNotDeclaredException ile ölür")


def test_ariza_enjeksiyon_salterleri_VAR_ve_KAPALI():
    """Kural motorunun DUYARLILIĞINI sınayan tetikler."""
    kaynak = io.open(_KOK / "scripts/sanal_gol.py", encoding="utf-8").read()
    for p in ("ariza_poz_sicramasi_m", "ariza_poz_nan_orani",
              "ariza_damga_kaydirma_s", "ariza_kadans_bolen",
              "ariza_kesinti_t_s", "ariza_govde_yansimasi_m"):
        assert f'declare_parameter("{p}"' in kaynak, f"{p} yok"
    # Hepsi varsayılan KAPALI (0 / 1)
    import re
    for p, v in re.findall(r'declare_parameter\("(ariza_\w+)",\s*([\d.]+)\)', kaynak):
        assert float(v) in (0.0, 1.0), f"{p} varsayılanı {v} — kapalı değil"


def test_betik_SOZDIZIMI_temiz():
    import subprocess
    r = subprocess.run(["bash", "-n", str(_GOL)], capture_output=True)
    assert r.returncode == 0, r.stderr.decode()


def test_sahtelenen_dugumler_GEREKCELI():
    """Muafiyet listesi keyfi büyümemeli — her biri donanım/kapsam gerekçeli."""
    assert len(_SAHTELENEN) <= 8, "muafiyet listesi şişiyor, gerekçeleri gözden geçir"

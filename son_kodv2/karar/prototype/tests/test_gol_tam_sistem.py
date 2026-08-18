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


def test_gol_temizleyici_TUM_gol_dugumlerini_taniyor():
    """🔴 HAYALET DÜĞÜM KAPISI (18.08 ölçümü).

    `gol_temizle.py` yalnız `girdap_decision` + `sanal_gol` arıyordu;
    `sahte_ham_sensor.py` listede YOKTU ⇒ hiç öldürülmüyordu. Ardışık
    koşumlardan **6 kopya** birikti (en eskisi 26 dakikalık) ve aynı topic'e
    altı üretici basıyordu — hangi verinin kimden geldiği belirsizleşir,
    ölçüm SESSİZCE anlamsızlaşır.

    Bu test göl betiğinin başlattığı her `scripts/*.py` düğümünün
    temizleyicinin desen listesinde olduğunu doğrular.
    """
    import re

    temizle = io.open(_KOK / "scripts/gol_temizle.py", encoding="utf-8").read()
    desenler = re.search(r"_GOL_DESENLERI\s*=\s*\(([^)]*)\)", temizle)
    assert desenler, "_GOL_DESENLERI bulunamadı"
    liste = re.findall(r'"([^"]+)"', desenler.group(1))
    assert "girdap_decision" in liste and "sanal_gol" in liste

    # Göl betiğinde `python3 "$S/<ad>.py"` ile başlatılan her düğüm
    for ad in re.findall(r'python3 "\$S/(\w+)\.py"', _GOL_METIN):
        assert any(d in ad or ad in d for d in liste), (
            f"{ad}.py göl betiğinde başlatılıyor ama gol_temizle.py onu "
            "TANIMIYOR — öldürülemez, hayalet düğüm olarak kalır")


def test_betik_SOZDIZIMI_temiz():
    import subprocess
    r = subprocess.run(["bash", "-n", str(_GOL)], capture_output=True)
    assert r.returncode == 0, r.stderr.decode()


def test_sahtelenen_dugumler_GEREKCELI():
    """Muafiyet listesi keyfi büyümemeli — her biri donanım/kapsam gerekçeli."""
    assert len(_SAHTELENEN) <= 8, "muafiyet listesi şişiyor, gerekçeleri gözden geçir"


# ══════════════════════════════════════════════════════════════════════════
# BİZİM KATMANIMIZ GÖLDE (18.08.2026) — `/perception/buoys` üreticisi
#
# 🔴 ÖLÇÜLDÜ: bu bağlantı yokken göl koşumunda `/perception/buoys` YAYINCI 0
# idi; kural motoru S1 · S2 · S5 ve C3'ün bir kolunu "veri yok" diye STALE
# bırakıyordu. STALE = "ihlal yok" DEĞİL, "hiç ölçülmedi". Yani P1/P2 puanını
# üreten kendi katmanımız gölde sınanmıyordu.
#
# Bağlandıktan sonra ölçüldü: yayıncı 1 · abone 2 · 4,999 Hz; S1 +0,2077 s,
# S2 +1, S5 +1, C3 iki kolu da (+0,1765 / +0,2704) — hepsi YEŞİL ve ÖLÇÜLÜR.
# ══════════════════════════════════════════════════════════════════════════
_KOS = _GOL                                   # scripts/gol_kos.sh
_DERLE = _KOK / "scripts" / "gol_derle.sh"
#: algı paketi KARAR ağacının DIŞINDA (son_kodv2/algi) — `_KOK` karar/
_NAV = (_KOK.parent / "algi" / "girdap_ida_algi" / "girdap_ida_algi"
        / "duba_gecis_navigator.py")


def _kos_komutlari() -> str:
    """🔑 YALNIZ ÇALIŞTIRILABİLİR satırlar — yorumlar ATILIR.

    ⚠ Bunu doğuran kusur: ilk hâli ham metinde arıyordu ve yukarıdaki
    açıklama bloğunda `duba_gecis_navigator` geçtiği için, `basla` satırı
    SİLİNDİĞİNDE bile test YEŞİL kalıyordu (mutasyonla ölçüldü: 22/22
    geçti). Yorumu ölçen nöbetçi, nöbetçi değildir.
    """
    satirlar, biriken = [], ""
    for ham in _KOS.read_text(encoding="utf-8").splitlines():
        kod = ham.split("#", 1)[0]
        if not kod.strip() and not biriken:
            continue
        if kod.rstrip().endswith("\\"):        # ters bölü = satır devam ediyor
            biriken += kod.rstrip()[:-1]
            continue
        satirlar.append((biriken + kod) if biriken else kod)
        biriken = ""
    if biriken:
        satirlar.append(biriken)
    return "\n".join(satirlar)


def test_algi_katmani_BIZIM_dugumu_baslatiyor():
    """`/perception/buoys` üreticisi göl algı katmanında olmalı."""
    m = _kos_komutlari()
    assert "duba_gecis_navigator" in m, (
        "gol_kos.sh bizim algı düğümümüzü başlatmıyor → /perception/buoys "
        "yayıncı 0 → S1/S2/S5/C3 STALE (ölçülmemiş)")
    # Kip seçilebilir: 1 = geometrik · 2 = görüntü (varsayılan). Aranan şey
    # bayrağın VERİLMİŞ ve SIFIR OLMAMASI; 0 olursa düğüm OAK-D arar ve
    # gölde asla açılmaz.
    import re
    esl = re.search(r"GIRDAP_SIM_KAYNAK=\$\{GIRDAP_GOL_ALGI_KIP:-(\d)\}", m)
    assert esl or "GIRDAP_SIM_KAYNAK=1" in m or "GIRDAP_SIM_KAYNAK=2" in m, (
        "sim kaynak kipi verilmemiş → düğüm OAK-D arar, gölde asla açılmaz")
    if esl:
        assert esl.group(1) in ("1", "2"), (
            f"geçersiz varsayılan kip {esl.group(1)!r}")


def test_navigator_ALGI_katmanina_bagli():
    """Düğüm ALGI bloğunun İÇİNDE olmalı — koşulsuz başlarsa katman
    bayrakları anlamını yitirir (algısız koşum artık mümkün olmaz)."""
    ic, bulundu = False, False
    for kod in _kos_komutlari().splitlines():
        if kod.startswith('if [ "${GIRDAP_GOL_ALGI:-0}" = "1" ]; then'):
            ic = True
        elif ic and kod.rstrip() == "fi":     # bloğu KAPATAN fi (girintisiz)
            ic = False
        elif ic and kod.strip().startswith("basla ") \
                and "duba_gecis_navigator" in kod:
            # ⚠ "satırda adı geçiyor" YETMEZ: aynı blokta düğümü ADIYLA anan
            # bir `echo` var ve mutasyonla ölçüldü — `basla` bloğu dışarı
            # taşındığı hâlde test YEŞİL kalıyordu. Aranan şey KOMUTUN
            # KENDİSİ; bu yüzden mantıksal satırlar birleştirilip (ters bölü
            # devamı) `basla` ile başlaması şart koşulur.
            bulundu = True
    assert bulundu, (
        "navigator ALGI bloğunun dışında başlatılıyor — katman bayrakları "
        "anlamını yitirir (algısız koşum imkânsızlaşır)")


def test_gol_derleme_ALGI_paketini_de_kuruyor():
    """`ros2 run girdap_ida_algi ...` paket kurulu değilse ÇALIŞMAZ.

    18.08'de tam bu yüzden sessizdi: ~/ros2_ws/install altında yalnız
    girdap_decision vardı.
    """
    d = _DERLE.read_text(encoding="utf-8")
    assert "girdap_ida_algi" in d, "gol_derle.sh algı paketini kurmuyor"
    assert "duba_gecis_navigator" in d, (
        "kurulum doğrulanmıyor — colcon sessizce atlarsa fark edilmez")


def test_navigator_KONTROL_yoluna_dokunmuyor():
    """🔑 Güvenlik: göle eklenen düğüm tekneyi SÜRMEMELİ.

    `MOD="dogrudan_surus"` cmd_vel basar, `"mppi_hedef"` /goal_pose basar —
    ikisi de karar hattıyla çakışır ve gölü sessizce anlamsızlaştırır.
    Dağıtım varsayılanı `algi_yayin`; bu test onu dondurur.
    """
    # ⚠ Ham metin araması BURADA YETMEZ: aynı dize modül docstring'inde de
    # geçiyor (satır 35, mod tablosu) ve mutasyonla ölçüldü — atama
    # değiştirildiği hâlde test YEŞİL kalıyordu. Bu yüzden AST'ten okunur.
    import ast
    agac = ast.parse(_NAV.read_text(encoding="utf-8"))
    mod = next(
        (n.value.value for n in agac.body
         if isinstance(n, ast.Assign)
         and any(getattr(t, "id", None) == "MOD" for t in n.targets)),
        None)
    assert mod == "algi_yayin", (
        f"MOD={mod!r} — göle eklenen düğüm kontrol yoluna yayın yapar "
        "(dogrudan_surus→cmd_vel, mppi_hedef→/goal_pose)")


def test_gate_passed_TUZAGI_kapali_kalıyor():
    """`/perception/gate_passed` True basılırsa `fsm_node._on_gate_passed`
    İLK geçitte PARKUR2→PARKUR3'e atlar: P2 tamamlanmaz (md 5.5.2.4),
    (G2/KD2)×40 puan ve ödül sıralaması gider. Gölde bu düğüm artık
    koştuğu için bayrağın kapalılığı ARTIK GÖL İÇİN DE kritik.
    """
    nav = _NAV.read_text(encoding="utf-8")
    assert "GATE_PASSED_YAYINLA = False" in nav, (
        "gate_passed açılmış — FSM'i erken PARKUR3'e atlatır")


# ══════════════════════════════════════════════════════════════════════════
# SANAL GÖL: SINIFLI TESPİT ÜRETİMİ (18.08.2026)
#
# 🔴 GİRİNTİ KUSURU: `_algi()` içinde `Detection3D` bloğu duba döngüsünün
# DIŞINDA ve `if self.ar_govde_m > 0.0:` (gövde yansıması ARIZASI, varsayılan
# KAPALI) İÇİNDE duruyordu. Sonuç:
#   ① `/perception/classified_obstacles` **her zaman BOŞ** (ölçüldü: 120
#      mesaj / 0 tespit) ⇒ `sahte_ham_sensor` renk bulamayıp hiç duba
#      çizmiyordu ⇒ `/oak/rgb/image_raw` baştan beri boş su karesiydi.
#   ② Arıza açıkken bile döngüden ARTAKALAN değişkenler kullanılıyordu ⇒
#      yalnız SON duba yazılıyordu.
# Düzeltmeden sonra ölçüldü: 120 mesaj / **1240 tespit**.
# ══════════════════════════════════════════════════════════════════════════
_SANAL_GOL = _KOK / "scripts" / "sanal_gol.py"


def _algi_govdesi() -> str:
    """`_algi` metodunun kaynağı (ast ile kesilir — metin araması değil)."""
    import ast
    kaynak = _SANAL_GOL.read_text(encoding="utf-8")
    agac = ast.parse(kaynak)
    for n in ast.walk(agac):
        if isinstance(n, ast.FunctionDef) and n.name == "_algi":
            return ast.get_source_segment(kaynak, n) or ""
    raise AssertionError("sanal_gol._algi bulunamadı")


def test_siniflandirma_ARIZA_bayragina_bagli_DEGIL():
    """🔑 Asıl kural: sınıflı tespit üretimi arıza enjeksiyonundan bağımsız.

    `da.detections.append` çağrısı `ar_govde_m` dalının içindeyse, arıza
    kapalıyken (varsayılan) sınıflı topic boş kalır ve kamera zinciri
    sessizce kör olur.
    """
    import ast
    agac = ast.parse(_algi_govdesi())
    for n in ast.walk(agac):
        if not (isinstance(n, ast.If) and "ar_govde_m" in ast.unparse(n.test)):
            continue
        icerik = ast.unparse(n)
        assert "da.detections.append" not in icerik, (
            "sınıflı tespit üretimi gövde-yansıması ARIZASININ içinde — "
            "arıza kapalıyken classified_obstacles BOŞ kalır")


def test_siniflandirma_duba_DONGUSUNUN_icinde():
    """Her duba için bir tespit üretilmeli; döngü dışında kalırsa yalnız
    SONUNCUSU (artakalan değişkenlerle) yazılır."""
    import ast
    agac = ast.parse(_algi_govdesi())
    for n in ast.walk(agac):
        if isinstance(n, ast.For) and "cisimler" in ast.unparse(n.iter):
            assert "da.detections.append" in ast.unparse(n), (
                "tespit üretimi duba döngüsünün dışında")
            return
    raise AssertionError("`for ... in cisimler` döngüsü bulunamadı")


def test_gol_KAMERA_cozunurlugu_tasinabilir_boyutta():
    """1280×720 kare 2,7 MB; 10 Hz'te 27 MB/s ⇒ DDS düşürüyor.

    Ölçüldü: varsayılanla `/perception/buoys` 1,71 Hz'e düşüyor ve LiDAR
    karelerinin %64,9'u füzyonda eşleşemiyordu; 512×512'de omap 10,00 Hz,
    buoys 5,12 Hz. Darboğaz tespit DEĞİL (13,7 ms/kare), TAŞIMA.
    """
    m = _kos_komutlari()
    assert "kamera_genislik_px:=512" in m and "kamera_yukseklik_px:=512" in m, (
        "göl kamerası dağıtımın NN girdisine (512×512) sabitlenmemiş")


def test_P3_sinif5_rengi_SIYAH():
    """Şartname s.18 hedef renkleri RAL 3026/6037/9005 — kahverengi YOK.

    `kamikaze_hedef.py` 18.08'de düzeltildi ama gölün sahte kamerası eski
    rengi basmaya devam ediyordu ⇒ hakem 'siyah' dediğinde göl YANLIŞ rengi
    gösteriyordu.
    """
    ham = (_KOK / "scripts" / "sahte_ham_sensor.py").read_text(encoding="utf-8")
    import ast
    for n in ast.walk(ast.parse(ham)):
        if isinstance(n, ast.Assign) and any(
                getattr(t, "id", None) == "_SINIF_BGR" for t in n.targets):
            tablo = ast.literal_eval(n.value)
            b, g, r = tablo["5"]
            assert max(b, g, r) <= 60, (
                f"sınıf 5 rengi {tablo['5']} — siyah (RAL 9005) değil")
            return
    raise AssertionError("_SINIF_BGR bulunamadı")

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
    # 19.08: renk köprüsü FC parametresini (SCR_USER1) MAVROS üzerinden okur;
    # gölde MAVROS zaten sahtelenmiş (yukarıda) ⇒ okuyacak bir FC yok, node
    # yalnız "param/get hazır değil" uyarısı basardı. Kendi uçtan uca testi
    # var: `test_renk_kodu_koprusu.py` (sahte /mavros/param/get + GERÇEK
    # kamikaze_param_node ile 4 test).
    "renk_kodu_koprusu",
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


def test_gol_KAMERA_karesi_SOKET_TAMPONUNA_siginiyor():
    """Ham görüntü DDS'ten geçerse kare soket tamponundan KÜÇÜK olmalı.

    Ölçüldü: `net.core.rmem_max = 212992` B (bu makinede). Tek kare bundan
    büyükse FastDDS parçaları toparlayamıyor ve kare DÜŞÜYOR:
        1280×720 → 2,7 MB  ⇒ abone ~1,0 Hz
         512×512 → 786 KB  ⇒ abone 4,25 Hz (yayıncı 8,11)
         256×256 → 196 KB  ⇒ kayıpsız, abone 8,00 Hz
    🔑 Korunan şey KADANS: gerçek 8 Hz kamera ↔ 10 Hz LiDAR farkı kapı
    zincirini bozan etkendir. Çözünürlük yalnız tespit hassasiyetini
    etkiler; bbox NORMALİZE olduğu için bu kip ölçekten bağımsız.
    ⚠ Gerçek teknede görüntü DDS'ten HİÇ geçmez (depthai USB, aynı süreç);
    bu sınır yalnız gölde vardır. Jetson kurulumunda `net.core.rmem_max`
    büyütülmeli.
    """
    import re
    m = _kos_komutlari()
    esl = re.search(r"kamera_genislik_px:=\"?\$\{GIRDAP_GOL_KAM_PX:-(\d+)\}",
                    m)
    assert esl, "göl kamera çözünürlüğü ayarlamıyor"
    px = int(esl.group(1))
    try:
        tampon = int((pathlib.Path("/proc/sys/net/core/rmem_max")
                      ).read_text().strip())
    except OSError:                      # Linux dışı / erişilemez
        tampon = 212992
    assert px * px * 3 <= tampon, (
        f"{px}×{px} kare {px*px*3} B > rmem_max {tampon} B ⇒ kareler düşer, "
        "kamera kadansı gerçeği yansıtmaz")


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


# ══════════════════════════════════════════════════════════════════════════
# GÖL GERÇEĞİ YANSITIYOR MU — kadans · TF · montaj (18.08.2026)
#
# Eyüp: *"gerçekte nasıl ise kodda öyle olsun"* — gölün amacı kodun burada
# iyi görünmesi DEĞİL, gerçek koşulları yansıtması. Aşağıdaki her sayı
# dağıtım kaynağından gelir, tahminden değil.
# ══════════════════════════════════════════════════════════════════════════
_HAM = _KOK / "scripts" / "sahte_ham_sensor.py"
_HW = _KOK / "ros2_ws/src/girdap_decision/config/hardware.yaml"


def test_LIDAR_ve_KAMERA_ayri_kadansta():
    """Gerçekte Livox 10 Hz, OAK 8 Hz (`duba_gecis_navigator.FPS`).

    Tek timer'da basmak yavaş tarafın hızlı tarafın karelerini düşürmesini
    gizler — füzyon `ApproximateTimeSynchronizer` ile eşleştirdiği için bu
    fark kapı zincirini doğrudan etkiler.
    """
    # ⚠ Alt dize araması YETMEZ: `_tick_kamera_X` de "_tick_kamera" içerir
    # ve mutasyon testten KAÇTI (ölçüldü). Metot adları AST'ten okunur.
    import ast
    agac = ast.parse(_HAM.read_text(encoding="utf-8"))
    metotlar = {n.name for n in ast.walk(agac)
                if isinstance(n, ast.FunctionDef)}
    assert {"_tick_lidar", "_tick_kamera"} <= metotlar, (
        f"LiDAR ve kamera ayrı kadansta değil — bulunan: "
        f"{sorted(x for x in metotlar if x.startswith('_tick'))}")
    # ve ikisi de GERÇEKTEN timer'a bağlı olmalı
    baglilar = {ast.unparse(a).split(".")[-1]
                for n in ast.walk(agac)
                if isinstance(n, ast.Call)
                and getattr(n.func, "attr", None) == "create_timer"
                for a in n.args[1:2]}
    assert {"_tick_lidar", "_tick_kamera"} <= baglilar, (
        f"timer'a bağlı olanlar: {sorted(baglilar)}")
    m = _kos_komutlari()
    assert "kamera_hz:=8.0" in m, "kamera kadansı gerçeğe (8 Hz) sabitlenmemiş"
    assert "lidar_hz:=10.0" in m, "LiDAR kadansı gerçeğe (10 Hz) sabitlenmemiş"


def test_kamera_kadansi_ALGI_kodundaki_FPS_ile_ayni():
    """🔑 Ayrışma kapısı: algı `FPS`'i değişirse göl de değişmeli."""
    import ast
    nav = _NAV.read_text(encoding="utf-8")
    fps = next(
        (n.value.value for n in ast.parse(nav).body
         if isinstance(n, ast.Assign)
         and any(getattr(t, "id", None) == "FPS" for t in n.targets)), None)
    assert fps is not None, "algı FPS sabiti bulunamadı"
    assert f"kamera_hz:={float(fps)}" in _kos_komutlari(), (
        f"algı FPS={fps} ama göl farklı kadansta koşuyor")


def test_MAVROS_yayin_hizi_stream_rate_ile_ayni():
    """`hardware.yaml stream_rate_hz` FC'den istenen akış hızı (STREAM_ALL).

    Göl 50 Hz basıyordu — 5× gerçeküstü. Fizik 50 Hz kalır, YAYIN seyrelir.
    """
    import yaml
    hw = yaml.safe_load(_HW.read_text(encoding="utf-8")) or {}
    hz = None
    for blok in hw.values():
        if isinstance(blok, dict) and "stream_rate_hz" in blok:
            hz = int(blok["stream_rate_hz"])
    assert hz, "stream_rate_hz hardware.yaml'da yok"
    sg = _SANAL_GOL.read_text(encoding="utf-8")
    assert "_MAVROS_SEYRELTME" in sg, "yayın seyreltmesi yok — göl 50 Hz basar"
    import ast
    seyreltme = next(
        (n.value.value for n in ast.walk(ast.parse(sg))
         if isinstance(n, ast.Assign)
         and any(getattr(t, "id", None) == "_MAVROS_SEYRELTME"
                 for t in n.targets)), None)
    assert 50 / seyreltme == hz, (
        f"göl {50/seyreltme:.0f} Hz basıyor, gerçek {hz} Hz")


def test_fizik_entegrasyonu_50_Hz_KALIYOR():
    """Yayın seyreltilir ama entegrasyon seyreltilmez — yoksa dinamik bozulur."""
    sg = _SANAL_GOL.read_text(encoding="utf-8")
    assert "self.create_timer(0.02, self._fizik)" in sg, (
        "fizik adımı 50 Hz'den çıkmış — kadans düzeltmesi dinamiği bozdu")


def test_TF_agaci_golde_YAYINLANIYOR():
    """🔴 Gölde `/tf_static` YAYINCI 0'dı; dağıtımda üç static TF var."""
    m = _kos_komutlari()
    assert "static_transform_publisher" in m, "gölde TF ağacı yok"
    for cerceve in ("livox_frame", "oak_frame", "imu_link"):
        assert cerceve in m, f"{cerceve} TF'i gölde yayınlanmıyor"


def test_TF_degerleri_ELLE_yazilmamis_hardware_yaml_okunuyor():
    """Tek kaynak kuralı: sayı iki yerde yaşarsa göl yanlış montajı yansıtır.

    (ör. oak yaw = +0,0415 rad, 11.08'de şeritle ÖLÇÜLDÜ.)
    """
    m = _kos_komutlari()
    assert "hardware.yaml" in m, "TF değerleri hardware.yaml'dan okunmuyor"
    assert "0.0415" not in m, "ölçülmüş TF değeri betiğe elle kopyalanmış"


# ══════════════════════════════════════════════════════════════════════════
# HAREKET ANALİZİ (18.08.2026) — "İDA sanal gölde nasıl hareket etti?"
#
# Eyüp: *"kendi göle gitmiş gibi İDA'mızın hareketlerini sanalda incelemek"*.
# Araç bant kaydedip `bant_kapi_olcum.py` koşar — GERÇEK göl bantlarını ölçen
# aracın TA KENDİSİ. Paralel bir analizci yazmak metriklerin sessizce
# ayrışması demek olurdu; aynı kod = geçerli "sanalda böyle ↔ gölde şöyle"
# karşılaştırması.
# ══════════════════════════════════════════════════════════════════════════
_HAREKET = _KOK / "scripts" / "gol_hareket.sh"


def _kod_satirlari(yol) -> str:
    """Yorumlar ayıklanmış kaynak — nöbetçi düzyazıyı eşlemesin."""
    return "\n".join(
        k for k in (l.split("#", 1)[0] for l in
                    yol.read_text(encoding="utf-8").splitlines()) if k.strip())


def test_hareket_araci_GERCEK_analizciyi_kosuyor():
    """🔑 Paralel analizci YAZILMAMALI — metrik ayrışması riski."""
    m = _HAREKET.read_text(encoding="utf-8")
    assert "bant_kapi_olcum.py" in m, (
        "hareket aracı gerçek göl analizcisini koşmuyor — sanal/gerçek "
        "karşılaştırması geçersizleşir")


def test_hareket_araci_ANALIZCININ_topiclerini_kaydediyor():
    """Analizcinin okuduğu her topic banda girmeli; eksik olan sessizce
    'veri yok' der ve o bölüm boş çıkar."""
    import re
    analizci = (_KOK / "scripts" / "bant_kapi_olcum.py").read_text(
        encoding="utf-8")
    istenen = set(re.findall(r'"(/[a-z_0-9/]+)"', analizci))
    kaydedilen = set(re.findall(r'(/[a-z_0-9/]+)',
                                _HAREKET.read_text(encoding="utf-8")))
    eksik = {t for t in istenen if t.count("/") >= 2} - kaydedilen
    assert not eksik, f"banda girmeyen topic'ler: {sorted(eksik)}"


def test_hareket_araci_set_u_KULLANMIYOR():
    """🪤 `/opt/ros/humble/setup.bash` tanımsız değişken okuyor
    (`AMENT_TRACE_SETUP_FILES`); `set -u` betiği ilk satırda öldürür.
    Aynı sınıf `gol_kos.sh`'ta `set -e` ile yaşandı."""
    satirlar = [l.split("#", 1)[0].strip()
                for l in _HAREKET.read_text(encoding="utf-8").splitlines()]
    assert "set -u" not in satirlar and "set -eu" not in satirlar


def test_hareket_araci_GOL_AYAKTA_MI_denetliyor():
    """Göl düşükken bant BOŞ çıkar ve analiz sessizce anlamsızlaşır."""
    m = _kod_satirlari(_HAREKET)
    assert "/girdap/fusion/pose" in m and "exit 1" in m, (
        "göl ayakta mı denetimi yok — boş bant sessizce analiz edilir")


# ══════════════════════════════════════════════════════════════════════════
# ÖLÇÜMÜ SESSİZCE GEÇERSİZ KILAN ÜÇ KUSUR (18.08.2026)
# Eyüp: *"doğruluğunu kesinleştirmeden hüküm vermek yok"*. Üçü de ölçüldü;
# üçü de belirti vermeden yanlış sonuç ürettirdi.
# ══════════════════════════════════════════════════════════════════════════


def test_BAYAT_KURULUM_denetimi_var():
    """🔴 `gol_derle.sh` `--symlink-install` KULLANAMIYOR ⇒ `install/` KOPYA.

    Kaynak değişip derlenmezse göl ESKİ kodu koşar ve hiçbir belirti vermez.
    Ölçüldü: hareket analizi bir kez bayat `duba_gecis_navigator.py` ile
    koşturuldu; bulgular güncel kodu yansıtmıyordu.
    """
    m = _kod_satirlari(_GOL)
    assert "cmp -s" in m and "BAYAT KURULUM" in _GOL.read_text(encoding="utf-8"), (
        "göl bayat kurulum denetimi yapmıyor — 'bizim kodumuz koşuyor' "
        "iddiası taşınamaz")


def test_DALGA_argumanlari_ondaliga_zorlaniyor():
    """🔴 rclpy tipi KATIDIR: `0` INTEGER gelir, düğüm AÇILIŞTA ÖLÜR.

    Ölçüldü: `gol_kos.sh 8 6.0 4.0 4 0 0 90 1.0 20 42` ile `sanal_gol`
    `InvalidParameterTypeException` alıp öldü; hiç engel yayınlanmadı,
    sahte kamera boş kare bastı, algı `kenar=0` gösterdi. Belirti ALGIDA,
    sebep BURADA — iki koşumu birden anlamsızlaştırdı.
    Aynı koruma YON0 için zaten vardı; dalga argümanları korunmamıştı.
    """
    m = _kod_satirlari(_GOL)
    for degisken in ("DALGA", "DALGA_YAW", "YON0"):
        assert f'printf' in m and degisken in m, f"{degisken} korunmuyor"
    import re
    assert re.search(r'DALGA="\$\(printf', m), "DALGA ondalığa zorlanmıyor"
    assert re.search(r'DALGA_YAW="\$\(printf', m), "DALGA_YAW zorlanmıyor"


def test_hareket_araci_GOREV_FAZINI_denetliyor():
    """🔴 Görev bitince araç DURUR; o pencerede ölçülen hız aracın davranışı
    DEĞİL, bitmiş görevin sessizliğidir.

    İKİ KEZ yaşandı: pencere kısmen TAMAMLANDI'da → 0,170 m/s; tamamen
    TAMAMLANDI'da → 0,001 m/s · sıfır komut %100. İkisi de yanlış okundu.
    Faza göre ayrıştırılmış doğru ölçüm: PARKUR1'de ortanca 0,211 m/s,
    %95 0,910 m/s.
    """
    m = _kod_satirlari(_HAREKET)
    assert "/girdap/mission/state" in m, "görev fazı okunmuyor"
    assert "TAMAMLANDI" in m and "exit 1" in m, (
        "bitmiş görevde ölçüm reddedilmiyor")


# ══════════════════════════════════════════════════════════════════════════
# ŞARTNAME TOPOLOJİSİ (18.08.2026 — Şekil 3'ten okundu)
#
# Lejant: "Karşılıklı 2 Kenar Dubası Arası Mesafe" = **8-12 m**
#         "Yan Yana 2 Kenar Dubası Arası Mesafe"  = X, DEĞİŞKEN
# Gövde (s.20): mesafeler "yarışma alanına göre değişkenlik gösterecektir".
# Parkur-1 düzeni: UZUN DİYAGONAL KOLLAR + GN köşelerinde keskin dönüş
# (GN1→GN2→GN3→GN4); bir kolun üzerinde ardışık birkaç kapı var.
# Parkur-2: sarı engel dubaları koridorun İÇİNDE.
#
# 🔴 Göl 18.08'e kadar: sabit 6 m genişlik (bandın ALTINDA = kolay),
# her kapıda kırılan zikzak, engeller parkurun SONUNDA (hiç karşılaşılmıyor).
# ══════════════════════════════════════════════════════════════════════════


def test_kapi_genisligi_SARTNAME_bandindan():
    """Tek sabit genişlik gerçeği yansıtmaz — band 8-12 m, kapı başına çekilir."""
    import ast
    sg = _SANAL_GOL.read_text(encoding="utf-8")
    agac = ast.parse(sg)
    varsayilan = {}
    for n in ast.walk(agac):
        if (isinstance(n, ast.Call)
                and getattr(n.func, "attr", None) == "declare_parameter"
                and n.args and isinstance(n.args[0], ast.Constant)):
            ad = n.args[0].value
            if ad in ("kapi_acik_min_m", "kapi_acik_max_m", "kapi_acikligi_m"):
                varsayilan[ad] = ast.literal_eval(n.args[1])
    assert varsayilan.get("kapi_acik_min_m") == 8.0, "alt sınır 8 m değil"
    assert varsayilan.get("kapi_acik_max_m") == 12.0, "üst sınır 12 m değil"
    assert varsayilan.get("kapi_acikligi_m") == 0.0, (
        "sabit genişlik varsayılan olarak AÇIK — band ezilir, göl kolaylaşır")


def test_gol_kos_bandi_EZMIYOR():
    """Başlatıcı varsayılanı 0 olmalı; >0 bandı ezer."""
    m = _kod_satirlari(_GOL)
    assert 'ACIK="${2:-0}"' in m, (
        "gol_kos.sh varsayılanı bandı eziyor (eski hâl: 12.0)")


def test_engeller_KORIDOR_icinde():
    """Şekil 3: sarı engeller Parkur-2 koridorunun İÇİNDE.

    Parkurun sonuna konursa araç onlarla koridorda hiç karşılaşmaz ve
    Parkur-2'nin asıl zorluğu (kapıdan geçerken kaçınmak) sınanmaz.
    """
    import ast
    sg = _SANAL_GOL.read_text(encoding="utf-8")
    for n in ast.walk(ast.parse(sg)):
        if (isinstance(n, ast.Call)
                and getattr(n.func, "attr", None) == "declare_parameter"
                and n.args and getattr(n.args[0], "value", None) == "engel_yerlesimi"):
            assert ast.literal_eval(n.args[1]) == "koridor", (
                "engel yerleşimi varsayılanı 'koridor' değil")
            return
    raise AssertionError("engel_yerlesimi parametresi yok")


def test_parkur_UZUN_KOLLARDAN_olusuyor():
    """Her kapıda kırılan zikzak Şekil 3'e UYMUYOR — kurs uzun diyagonal
    kollardan ve GN köşelerinden oluşur."""
    sg = _SANAL_GOL.read_text(encoding="utf-8")
    assert "parkur_kollari" in sg, "kol tabanlı topoloji yok"
    assert "desen = [0.0, zig, 0.0, -zig]" not in sg, (
        "her kapıda kırılan eski zikzak deseni geri gelmiş")


def test_parkur_KUNYESI_tek_kaynak():
    """Geometri iki yerde kurulursa ölçüm var olmayan bir parkuru ölçer.

    `sanal_gol` gerçek parkuru künyeye yazar; `gol_pdc_olc.py` oradan okur.
    """
    assert "parkur.json" in _SANAL_GOL.read_text(encoding="utf-8"), (
        "sanal_gol parkur künyesi yazmıyor")
    pdc = (_KOK / "scripts" / "gol_pdc_olc.py").read_text(encoding="utf-8")
    assert "parkur.json" in pdc, "pdc künyeden okumuyor — geometri ayrışır"


# ══════════════════════════════════════════════════════════════════════════
# 🔑 KAPIDAN NEDEN GEÇMİYOR — SÖZLEŞME UYUMSUZLUĞU (19.08.2026)
#
# Ölçüm zinciri (sanal göl, şartname geometrisi):
#   SEBEP: DUZLEMI_ASMADI · en ileri −0,72 / −1,08 m · yanal 0,00 m
#   ⇒ nişan MÜKEMMEL hizalı; araç kapı ortasına kadar geliyor ama
#     düzlemi aşmıyor.
#
# Aritmetik:
#   karar tarafı  : hedef = kapının NİŞAN NOKTASI (`_refine_target` →
#                   `GateFollower.aim_point`, engel yoksa = geometrik ORTA).
#                   Nişan kapı KİRİŞİ ÜZERİNDEDİR — ötesinde değil.
#   varış         : `arrival_radius_m = 2,0` ⇒ araç ortaya 2 m kala
#                   "vardı" sayılır ve SONRAKİ noktaya döner.
#   algı şartı    : geçiş sayılması için düzlemi `PASS_EK_YOL = 1,53 m`
#                   aşmak (kıç da temizlesin diye).
#   ⇒ AÇIK = 2,0 + 1,53 = 3,53 m. Araç en iyi ihtimalle orta−2 m'de döner.
#
# 🔑 Algı ekibi bu tuzağı BİLİYOR: kendi `mppi_hedef` modunda
# `HEDEF_OTELEME = 2,03 m` ile hedefi kapının ÖTESİNE koyuyor. Ama
# dağıtımda kullanılan Plan A'da (`MOD = "algi_yayin"`) sürüşü KARAR
# tarafı yapıyor ve o öteleme YOK.
#
# ⚠ Bu dosya davranışı DÜZELTMEZ (düzeltme karar tarafının kararı —
# ortak alan kuralı). Uyumsuzluğu GÖRÜNÜR ve ÖLÇÜLEBİLİR tutar.
# ══════════════════════════════════════════════════════════════════════════


def _algi_sabiti(ad: str) -> float:
    """Algı düğümünden modül düzeyi sabit (AST — import gerektirmez)."""
    import ast
    kaynak = _NAV.read_text(encoding="utf-8")
    for n in ast.parse(kaynak).body:
        if isinstance(n, ast.Assign) and any(
                getattr(t, "id", None) == ad for t in n.targets):
            try:
                return float(ast.literal_eval(n.value))
            except ValueError:
                # `KAMERA_KIC_MESAFE + 0.5` gibi ifadeler
                return float(eval(  # noqa: S307 — sabit aritmetik
                    ast.unparse(n.value),
                    {"KAMERA_KIC_MESAFE": _algi_sabiti("ARAC_BOY"),
                     "ARAC_BOY": 1.03}))
    raise AssertionError(f"algı sabiti bulunamadı: {ad}")


def test_gecis_sarti_ile_VARIS_yaricapi_arasindaki_ACIK():
    """🔑 Kapıdan geçmemenin ölçülmüş sebebi — sayı olarak dondurulur.

    Açık daralırsa (varış yarıçapı küçülür / öteleme eklenir) bu test
    kırılır ve NOT güncellenmesi gerekir; büyürse durum kötüleşmiştir.
    """
    import yaml
    p = (_KOK / "ros2_ws/src/girdap_decision/config/params.yaml"
         ).read_text(encoding="utf-8")
    varis = None
    for blok in (yaml.safe_load(p) or {}).values():
        if isinstance(blok, dict):
            for alt in blok.values():
                if isinstance(alt, dict) and "arrival_radius_m" in alt:
                    varis = float(alt["arrival_radius_m"])
    assert varis is not None, "arrival_radius_m params.yaml'da bulunamadı"
    ek_yol = _algi_sabiti("PASS_EK_YOL")
    acik = varis + ek_yol
    assert acik == pytest.approx(3.53, abs=0.2), (
        f"varış {varis} + geçiş şartı {ek_yol} = {acik:.2f} m açık — "
        "sözleşme değişmiş, teşhis notu güncellenmeli")


def test_karar_hedefi_kapi_KIRISI_uzerinde_OTESINDE_degil():
    """Karar tarafı nişanı kiriş üzerinde seçiyor; öteleme YOK.

    Algı `mppi_hedef` modunda `HEDEF_OTELEME` ile ötesine koyuyor — iki
    yol arasındaki bu fark, geçişin sayılmamasının doğrudan sebebi.
    """
    gf = (_KOK / "prototype/mission/gate_follower.py").read_text(
        encoding="utf-8")
    assert "aim_point" in gf
    # nişan kiriş üzerinde: normal yönünde öteleme yapan bir terim OLMAMALI
    assert "HEDEF_OTELEME" not in gf, (
        "karar tarafına öteleme eklenmiş — teşhis notu ve bu test "
        "birlikte güncellenmeli (uyumsuzluk kapanmış olabilir)")
    oteleme = _algi_sabiti("HEDEF_OTELEME")
    assert oteleme > 0.0, "algı tarafındaki öteleme kaybolmuş"


def test_algi_yayin_modunda_surusu_KARAR_yapiyor():
    """Plan A'da algı yalnız yayın yapar; hedefi karar tarafı sürer.

    Bu yüzden algının kendi `HEDEF_OTELEME` çözümü Plan A'da DEVREDE DEĞİL.
    """
    import ast
    agac = ast.parse(_NAV.read_text(encoding="utf-8"))
    mod = next((n.value.value for n in agac.body
                if isinstance(n, ast.Assign)
                and any(getattr(t, "id", None) == "MOD" for t in n.targets)),
               None)
    assert mod == "algi_yayin", (
        f"MOD={mod!r} — teşhisin dayanağı değişti")

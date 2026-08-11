"""
Girdap İDA — hardware.launch.py config yükleyici testi (F-V.5 / F3.3).

hardware.yaml okunamadığında launch SESSİZCE yarışma-modu varsayılanlarına
(use_isam2/use_rrt=True) düşüyordu — video günü YAML yazım hatası, bypass'ı
fark edilmeden kapatırdı (md 3.3.1.1 istemsiz-hareket riski: kalibrasyonsuz
iSAM2+RRT*). Düzeltme: fallback KALIR ama stderr'e gürültülü uyarı basılır.

launch/launch_ros gerektirir (ROS ortamı); yoksa SKIP (CI ROS'suz job'ı).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros", reason="launch_ros yok — ROS ortamında koş")

_LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision" / "launch" / "hardware.launch.py"
)



def _kaynak_yaml() -> dict:
    """Kaynak ağaçtaki hardware.yaml — share dizini kurulu olmasa da okunur.

    `_load_hardware_config()` share'i bulamazsa varsayılanlara düşer ve
    `tf` bloğu BOŞ gelir; drift denetimi o yolda sessizce anlamsızlaşırdı.
    Denetlenen şey zaten kaynak dosya (install onun kopyası).
    """
    import yaml
    yol = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "config" / "hardware.yaml"
    )
    with open(yol, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_module():
    spec = importlib.util.spec_from_file_location("hw_launch_test", _LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod



def _anahtar_metni(k) -> str:  # noqa: ANN001
    """Launch parametre anahtarını düz metne çevir.

    🔴 12.08 — bu iki testin AYLARDIR sessizce KIRIK olmasının sebebi.
    `Node(parameters=[{...}])` verildiğinde launch, sözlüğün DEĞERLERİNİ değil
    **ANAHTARLARINI da** normalleştirir: `"mount_z"` → `(TextSubstitution(
    text="mount_z"),)`. Testler `"mount_z" in block` / `str(k)` yazdığı için
    hiçbir anahtar eşleşmiyordu; `str(tuple)` nesne repr'i verir.

    Sonuç: iki nöbetçi de **hiçbir zaman** iddia ettiği şeyi doğrulamadı —
    parametre gerçekten geçmemiş olsa da test aynı hatayı verirdi. Kırık
    nöbetçi, olmayan nöbetçiden kötüdür: yeşil sanılır (KAR-03'teki "sahte
    yeşil"in test tarafındaki karşılığı).
    """
    if isinstance(k, str):
        return k
    if isinstance(k, (tuple, list)):
        return "".join(getattr(x, "text", str(x)) for x in k)
    return getattr(k, "text", str(k))


def _param_anahtarlari(blok) -> set:  # noqa: ANN001, ANN201
    """Bir parametre sözlüğünün anahtarlarını düz metin kümesi olarak ver."""
    return {_anahtar_metni(k) for k in blok}


def test_config_okunamazsa_uyari_basar_ve_varsayilana_duser(
    monkeypatch, capsys
) -> None:
    """F-V.5: hardware.yaml yüklenemezse stderr'e GÜRÜLTÜLÜ uyarı + fallback."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    cfg = mod._load_hardware_config()

    # Fallback davranışı korunur (yarışma-modu varsayılanları).
    assert cfg["use_isam2"] is True and cfg["use_rrt"] is True
    assert cfg["mission_source"] == "file"

    # Ama artık SESSİZ değil: stderr'de video-modu riskini söyleyen uyarı var.
    err = capsys.readouterr().err
    assert "hardware.yaml" in err
    assert "use_isam2" in err or "yarışma" in err.lower()


def test_config_normal_yolda_uyari_basmaz(capsys) -> None:
    """Kaynak ağaçtaki gerçek hardware.yaml ile uyarı ÜRETİLMEZ (yanlış alarm yok).

    Not: share dizini kurulu değilse bu test yine uyarı yolunu tetikleyebilir;
    o durumda kaynak yaml'ı share'den okunamıyor demektir — ortam işareti.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    mod._load_hardware_config()
    assert "UYARI" not in capsys.readouterr().err


# ----- B1/B2 + F-M.6: video blokları hardware.yaml'dan node'lara geçmeli -----


def test_yarisma_bloklari_yamldan_okunur(monkeypatch) -> None:
    """🔄 2026-08-11: `hardware.yaml` artık YARIŞMA tabanı (eski adı
    `test_video_bloklari_yamldan_okunur`).

    ⚠️ **Bu test `pytest.skip` ile sessizce atlanıyordu.** Eski hâli gerçek
    `get_package_share_directory()`'ye bakıyordu; paket install edilmemiş
    geliştirme ortamında skip oluyor, yani hardware.yaml'a dokunan bir
    değişikliği YAKALAMIYORDU — ama Jetson gibi install edilmiş bir ortamda
    kırmızı verirdi. Kardeş testlerin kullandığı `_kaynak_share` yardımcısına
    çevrildi: artık her ortamda koşuyor ve gerçek dosyayı okuyor.
    """
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)
    cfg = mod._load_hardware_config()

    # Görev GUIDED mod kenarıyla başlar; ARM tek başına başlatmaz.
    assert cfg["fsm"]["start_on_mode"] == "GUIDED"
    assert cfg["fsm"]["start_on_arm_in_mode"] is False
    assert cfg["bridge"]["auto_guided"] is True
    assert cfg["mode_name"] == "GUIDED"

    # Ekran-2 kuvvet isteği MPPI'nin kendi thrust'ından.
    assert cfg["telemetry"]["setpoint_source"] == "girdap"

    # 🔴 SOL/SAĞ KANAL — ters etiketleme nöbetçisi (11.08).
    # SERVO1_FUNCTION=74=ThrottleRight=SAĞ · SERVO3_FUNCTION=73=ThrottleLeft=SOL.
    # 07.08'de FİZİKSEL olarak ölçüldü (§0.12c açı taraması + `34ade34`
    # diferansiyel dönüş + §0.13 A/B/C testi). Bu satırlar o gün güncellenmemiş,
    # `left=1 / right=3` kalmıştı → Dosya-2 CSV'si ve Ekran-2c sol/sağ itkiyi
    # ters kaydediyordu (md 4.2 / md 3.3.1.1), üstelik SESSİZCE (değerler makul
    # görünür). Geri çevrilirse CI kırmızı.
    assert cfg["telemetry"]["fc_thrust_left_ch"] == 3
    assert cfg["telemetry"]["fc_thrust_right_ch"] == 1

    # F-M.6 — FC 1 Hz sorunu: bağlantıda akış hızı istenir. ALT SINIR 5 Hz:
    # fusion_node pose_timeout_s=1.0 → 1-2 Hz'de odom yayını KESİLİR.
    assert cfg["bridge"]["stream_rate_hz"] >= 5


def test_yaml_okunamazsa_yarisma_varsayilanina_duser(monkeypatch) -> None:
    """Fallback = YARIŞMA modu (GUIDED + MPPI thrust'ı) — video değerleri değil."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    cfg = mod._load_hardware_config()

    assert cfg["fsm"]["start_on_mode"] == "GUIDED"
    assert cfg["fsm"]["start_on_arm_in_mode"] is False   # yarışma güvenliği
    assert cfg["bridge"]["auto_guided"] is True
    assert cfg["telemetry"]["setpoint_source"] == "girdap"


def test_fv7_auto_videoda_dwell_sifir(monkeypatch) -> None:
    """F-V.7: AUTO'da FC waypoint'te durmaz → sahte bekleme yon_setpoint'i yanıltır.

    🔄 2026-08-11: dwell=0 kuralı artık `video.yaml` overlay'inde (AUTO orada).
    Yarışma tabanında tekneyi BİZ sürüyoruz, gerçek bekleme 2 s.
    ⚠️ Bu test de `pytest.skip` ile atlanıyordu — `_kaynak_share`'e çevrildi.
    """
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)

    monkeypatch.setenv("GIRDAP_CONFIG_OVERLAY", "video.yaml")
    cfg = mod._load_hardware_config()
    assert cfg["mission_timing"]["dwell_time_s"] == 0.0
    assert cfg["mission_timing"]["arrival_radius_m"] > 0.0

    # Yarışma tabanı (overlay yok): dwell GERÇEK bekleme.
    monkeypatch.delenv("GIRDAP_CONFIG_OVERLAY", raising=False)
    assert mod._load_hardware_config()["mission_timing"]["dwell_time_s"] == 2.0


def test_with_drivers_node_lari_launch_descriptiona_eklenir() -> None:
    """F-S.2 regresyonu: with_drivers=true olsa da driver_nodes listesi
    (livox_driver_node/oakd_driver_node/kamera_kayit_node) generate_launch_
    description()'ın döndürdüğü LaunchDescription'a hiç EKLENMEMİŞ bulundu
    (gerçek donanım testinde node list'te hiç görünmediler, 2026-07-16).
    Liste inşa ediliyordu ama `return LaunchDescription([...])` içinde
    `*driver_nodes` unutulmuştu — with_drivers flag'i sessizce hiçbir şey
    yapmıyordu.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    executables = {
        getattr(entity, "_Node__node_executable", None)
        for entity in ld.entities
        if type(entity).__name__ == "Node"
    }
    for exe in ("livox_driver_node", "oakd_driver_node", "kamera_kayit_node"):
        assert exe in executables, (
            f"{exe} generate_launch_description() çıktısında yok — "
            "driver_nodes listesi LaunchDescription'a eklenmemiş"
        )


def test_mavros_respawn_true() -> None:
    """F-P.20 (2026-07-16, gerçek donanım testi): FTDI/seri bağlantı bir
    anlığına EOF verdiğinde mavros_node yakalanmamış bir istisnayla çöküyordu
    (SIGABRT) ve hiçbir şey onu geri başlatmıyordu — karar yığını (planning_
    node/mission_manager_node kendi stale-guard'larıyla GÜVENLİ kalıyor ama
    sistem asla toparlanmıyordu).

    apm.launch/node.launch'ın kendi `respawn_mavros` argümanı VAR ama
    node.launch'taki <node> etiketine hiç bağlanmamış (mavros paketinin
    kendi hatası — canlı testte respawn_mavros:=true geçmesine rağmen
    ikinci çökmede mavros_node bir daha hiç dönmedi, doğrulandı). Bu yüzden
    apm.launch include'u bypass edilip mavros_node doğrudan Node() ile
    respawn=True olarak açılıyor (launch_ros'ta bu gerçekten çalışıyor).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    mavros_nodes = [
        e for e in ld.entities
        if type(e).__name__ == "Node"
        and getattr(e, "_Node__node_executable", None) == "mavros_node"
    ]
    assert mavros_nodes, (
        "mavros_node Node() eylemi bulunamadı — apm.launch include'u hâlâ "
        "kullanılıyor olabilir (respawn çalışmaz, F-P.20 rejeksiyonu)"
    )
    assert mavros_nodes[0]._ExecuteLocal__respawn is True, (
        "mavros_node respawn=True ile açılmıyor — çökerse sistem kalıcı "
        "olarak kör kalır (F-P.20)"
    )


# ----- iSAM2: kök `fusion:` bloğu (keyframe throttle + robust GPS) -----


def test_isam2_blogu_yamldan_okunur() -> None:
    """hardware.yaml kök `fusion:` bloğu fusion_node parametrelerine düşmeli.

    ⚠ `perception.fusion` (kamera-LiDAR bearing) ile AYNI ada sahip iki ayrı
    blok var; yükleyici bunları karıştırırsa iSAM2 ayarları sessizce
    varsayılanda kalır (throttle/robust hiç devreye girmez).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    cfg = mod._load_hardware_config()

    assert cfg["isam2"]["keyframe_rate_hz"] > 0.0
    assert cfg["isam2"]["gps_robust_enabled"] is True
    assert cfg["isam2"]["gps_huber_k"] == pytest.approx(1.345)

    # gps_sigma_by_status alt sözlüğü düzleştirilmiş param adlarına açılmalı
    assert cfg["isam2"]["gps_sigma_gbas_fix"] == pytest.approx(0.05)
    assert cfg["isam2"]["gps_sigma_sbas_fix"] == pytest.approx(0.50)
    assert cfg["isam2"]["gps_sigma_fix"] == pytest.approx(2.50)

    # RTK ile tek-nokta arasında ciddi ağırlık farkı olmalı (yoksa status
    # okumanın anlamı kalmaz)
    assert (
        cfg["isam2"]["gps_sigma_fix"] / cfg["isam2"]["gps_sigma_gbas_fix"]
    ) >= 10.0

    # perception.fusion BOZULMAMALI (ayrı blok, ayrı anahtar)
    assert cfg["fusion"]["camera_image_width_px"] == 1280


def test_isam2_blogu_yoksa_guvenli_varsayilana_duser() -> None:
    """yaml okunamazsa robust GPS AÇIK + throttle AÇIK kalmalı (güvenli taraf)."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    try:
        cfg = mod._load_hardware_config()
    finally:
        monkeypatch.undo()

    assert cfg["isam2"]["gps_robust_enabled"] is True
    assert cfg["isam2"]["keyframe_rate_hz"] == pytest.approx(5.0)


def test_isam2_launch_argumanlari_fusion_nodea_gecer() -> None:
    """fusion.* launch-arg'ları DECLARE edilip fusion_node'a bağlanmalı.

    F-S.2 sınıfı regresyon: blok okunur ama Node(parameters=...) listesine
    hiç eklenmezse ayar sessizce params.yaml'da kalır.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    declared = {
        e.name for e in ld.entities
        if type(e).__name__ == "DeclareLaunchArgument"
    }
    for key in mod._ISAM2_DEFAULTS:
        assert f"fusion.{key}" in declared, f"fusion.{key} declare edilmemiş"

    fusion_nodes = [
        e for e in ld.entities
        if type(e).__name__ == "Node"
        and getattr(e, "_Node__node_executable", None) == "fusion_node"
    ]
    assert fusion_nodes, "fusion_node launch açıklamasında yok"
    param_keys: set[str] = set()
    for block in fusion_nodes[0]._Node__parameters or []:
        if isinstance(block, dict):
            param_keys |= _param_anahtarlari(block)
    for key in mod._ISAM2_DEFAULTS:
        assert key in param_keys, f"{key} fusion_node parametrelerine geçmiyor"


def test_gorev_kaynagi_fc_varsayilani() -> None:
    """md 3.3.1(2): görev YKİ'de tanımlanıp İDA'ya YÜKLENİR → kaynak "fc".

    Elle launch edildiğinde sessizce araç-üstü YAML'a düşmek video şartını
    ihlal eder (görev İDA'da hazır beklemiş olur).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    assert mod._load_hardware_config()["mission_source"] == "fc"


# ----- B0/F5.1: LiDAR montaj ofseti `tf:` bloğundan node'a geçmeli -----


def test_B0_lidar_montaj_ofseti_tf_blogundan_okunur() -> None:
    """`tf.livox_frame` → perception_lidar_node mount_* parametreleri.

    B0: node ham bulutu kendisi base_link'e taşıyor; taşıma sayısı static TF
    yayıncısıyla AYNI bloktan gelmeli. İkinci bir kopya (params.yaml'a elle
    yazılmış offset) sessizce ayrışırsa engel haritası kayar.
    """
    mod = _load_module()
    beklenen = _kaynak_yaml()["tf"]["livox_frame"]

    params = mod._mount_params("livox_frame", _kaynak_yaml()["tf"])
    assert params["mount_x"] == float(beklenen["x"])
    assert params["mount_y"] == float(beklenen["y"])
    assert params["mount_z"] == float(beklenen["z"])
    assert params["mount_yaw"] == float(beklenen.get("yaw", 0.0))


def test_B0_lidar_montaj_z_ölçülmüş_değeri_tasiyor() -> None:
    """🔴 SESSİZ ARIZA KAPANI: `tf.livox_frame.z` SIFIR OLAMAZ.

    Sıfır olursa z_min ham LiDAR çerçevesinde uygulanır, 50 cm'lik duba
    (sensörün ALTINDA kaldığı için) tamamen elenir ve `obstacle_map` hiçbir
    hata basmadan boş döner — 04.08 atölye arızasının tam kendisi.
    Ölçülen değer 0.41 m (docs/olcum_formu.md §2).
    """
    z = float(_kaynak_yaml()["tf"]["livox_frame"]["z"])
    assert z > 0.05, (
        f"tf.livox_frame.z = {z} — LiDAR gövde tabanında olamaz; "
        "ölçülmüş montaj yüksekliği girilmeli (B0/F5.1)"
    )


def test_B0_montaj_parametreleri_lidar_nodeun_EN_SONUNDA() -> None:
    """Parametre sırası sözleşmesi: mount_* sözlüğü params_file'dan SONRA.

    ROS aynı adlı parametrede son değeri alır. Sıra bozulursa params.yaml'da
    unutulmuş bir `mount_z: 0.0` `tf:` bloğunu sessizce ezer.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    lidar = [
        e for e in ld.entities
        if type(e).__name__ == "Node"
        and getattr(e, "_Node__node_executable", None) == "perception_lidar_node"
    ]
    assert len(lidar) == 1
    params = getattr(lidar[0], "_Node__parameters")
    assert isinstance(params[-1], dict), "son parametre blogu sozluk degil"
    assert "mount_z" in _param_anahtarlari(params[-1]), (
        f"mount_z son blokta yok — bulunan anahtarlar: "
        f"{sorted(_param_anahtarlari(params[-1]))}"
    )


# --------------------------------------------------------------------------
# Overlay ZİNCİRİ (2026-08-08) — P1 koşusu yarışma ayarını MİRAS ALIR
# --------------------------------------------------------------------------
_PKG_KAYNAK = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
)


def _kaynak_share(monkeypatch, mod) -> None:
    """`get_package_share_directory` → kaynak paket dizini (config/ orada)."""
    monkeypatch.setattr(mod, "get_package_share_directory", lambda _p: str(_PKG_KAYNAK))


def test_overlay_zinciri_soldan_saga_binder(monkeypatch, capsys) -> None:
    """`yarisma.yaml:parkur1.yaml` → yarışma ayarları + P1 görev etiketleri.

    Zincir olmasaydı parkur1.yaml yarisma.yaml'ın KOPYASI olmak zorunda
    kalırdı; yarisma.yaml başlığının bilerek reddettiği drift budur.
    """
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)
    monkeypatch.setenv("GIRDAP_CONFIG_OVERLAY", "yarisma.yaml:parkur1.yaml")
    cfg = mod._load_hardware_config()

    # parkur1.yaml'ın TEK farkı:
    assert cfg["mission_file"] == "parkur1_mission.yaml"
    # yarisma.yaml'dan MİRAS (P1 koşusu = yarışma koşusu):
    assert cfg["use_rrt"] is True
    assert cfg["fsm"]["start_on_mode"] == "GUIDED"
    assert cfg["fsm"]["start_on_arm_in_mode"] is False
    assert cfg["bridge"]["auto_guided"] is True
    assert cfg["telemetry"]["setpoint_source"] == "girdap"
    assert cfg["mission_timing"]["dwell_time_s"] == 2.0
    # İki overlay de operatöre BİLDİRİLİR (sessiz uygulama yok).
    err = capsys.readouterr().err
    assert "yarisma.yaml" in err and "parkur1.yaml" in err


def test_yarisma_overlayi_tek_basina_competition_gorevini_secer(
    monkeypatch,
) -> None:
    """Zincir eklentisi tek-overlay davranışını BOZMAMALI (geriye uyum)."""
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)
    monkeypatch.setenv("GIRDAP_CONFIG_OVERLAY", "yarisma.yaml")
    cfg = mod._load_hardware_config()
    assert cfg["mission_file"] == "competition_mission.yaml"
    assert cfg["use_rrt"] is True


def test_overlaysiz_YARISMA_moduna_duser(monkeypatch) -> None:
    """🔄 2026-08-11 TERSİNE ÇEVRİLDİ — overlay YOKSA artık YARIŞMA tabanı.

    **Eski hâli:** *"Overlay YOKSA hardware.yaml = VİDEO senaryosu"* ve testin
    kendi gerekçesi şuydu: *"servis dosyası overlay'i set etmeyi unutursa yığın
    sessizce bu moda düşer (2026-08-08 bulgusu)."* Yani test, bilinen bir
    **saha arızasını** donduruyordu — arızayı belgeliyor ama önlemiyordu.

    O arıza üç ayrı turda tekrarlandı (§0.16b · §0.30d · §0.32/A3): her
    seferinde systemd drop-in'inin elle kurulması gerekti, kurulmazsa
    `auto_guided=false` + AUTO ⇒ `cmd_vel` HİÇ yayınlanmaz, GUIDED'a almak
    hiçbir şey yapmaz, iSAM2 ve RRT* kapalı kalır. **Belirtisi yoktu.**

    **Kaptan kararı (11.08):** taban yarışma olsun, video `video.yaml`
    overlay'ine taşınsın. Unutulan kurulumun bedeli artık emniyet tarafında —
    en fazla "masa testinde yarışma ayarları vardı".

    Bu test o kararın nöbetçisi: biri hardware.yaml'ı video değerlerine geri
    çevirirse CI kırmızı olur.
    """
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)
    monkeypatch.delenv("GIRDAP_CONFIG_OVERLAY", raising=False)
    cfg = mod._load_hardware_config()
    assert cfg["use_rrt"] is True
    assert cfg["use_isam2"] is True
    assert cfg["fsm"]["start_on_mode"] == "GUIDED"
    assert cfg["fsm"]["start_on_arm_in_mode"] is False
    assert cfg["bridge"]["auto_guided"] is True
    assert cfg["telemetry"]["setpoint_source"] == "girdap"


def test_video_overlayi_eski_hardware_yaml_davranisini_geri_verir(
    monkeypatch,
) -> None:
    """`video.yaml` = 11.08 öncesi `hardware.yaml`'ın senaryo ayarları.

    Video akışı SİLİNMEDİ, tersine çevrildi. Bu test iki şeyi birden koruyor:
    (a) video senaryosu hâlâ TAM erişilebilir — masa testi/Ekran-2 akışı
        `GIRDAP_CONFIG_OVERLAY=video.yaml` ile aynen geri gelir;
    (b) overlay mekanizması iki yönde de çalışıyor (yarışma→video da, video→
        yarışma da), yani taban değişikliği tek yönlü bir kapı değil.
    """
    mod = _load_module()
    _kaynak_share(monkeypatch, mod)
    monkeypatch.setenv("GIRDAP_CONFIG_OVERLAY", "video.yaml")
    cfg = mod._load_hardware_config()
    assert cfg["use_rrt"] is False
    assert cfg["use_isam2"] is False
    assert cfg["fsm"]["start_on_mode"] == "AUTO"
    assert cfg["fsm"]["start_on_arm_in_mode"] is True
    assert cfg["bridge"]["auto_guided"] is False
    assert cfg["telemetry"]["setpoint_source"] == "fc"
    assert cfg["mission_file"] == "video_mission.yaml"
    # F-V.7/8: AUTO'da ilerlemeyi FC yapar → bizim dwell'imiz 0 olmalı.
    assert cfg["mission_timing"]["dwell_time_s"] == 0.0


def test_systemd_dropinleri_overlay_ortam_degiskenini_veriyor() -> None:
    """Yarışma/P1 drop-in'leri overlay'i GERÇEKTEN set ediyor mu?

    `girdap-karar.service` launch'ı çıplak başlatır; overlay yalnız bu
    drop-in'lerden gelir. Satır düşerse yığın video modunda koşar ve bunun
    sahada BELİRTİSİ YOKTUR — bu yüzden test dondurur.
    """
    scripts = Path(__file__).resolve().parents[2] / "scripts"
    yarisma = (scripts / "girdap-karar-yarisma.conf").read_text(encoding="utf-8")
    parkur1 = (scripts / "girdap-karar-parkur1.conf").read_text(encoding="utf-8")
    assert "Environment=GIRDAP_CONFIG_OVERLAY=yarisma.yaml" in yarisma
    assert (
        "Environment=GIRDAP_CONFIG_OVERLAY=yarisma.yaml:parkur1.yaml" in parkur1
    )


def test_parkur1_gorev_dosyasi_TEK_parkur_yani_kamikaze_tetiklenemez() -> None:
    """parkur1_mission.yaml etiketleri hep 1 → hiçbir parkur geçişi olmaz.

    P1'i competition_mission.yaml ile koşmak 4. waypoint'te PARKUR_3
    (kamikaze) tetikler; bu dosya o yolu YAPISAL olarak kapatır.
    """
    from prototype.mission.parkur_fsm import (
        ParkurState,
        ParkurTransitionLogic,
        load_parkur_labels,
    )

    yol = _PKG_KAYNAK / "config" / "parkur1_mission.yaml"
    labels = load_parkur_labels(str(yol))
    assert labels and set(labels) == {1}

    logic = ParkurTransitionLogic(labels)
    for idx in range(len(labels) + 3):        # fazladan waypoint de gelse
        logic.current_waypoint_reached(idx)
    assert logic.state is ParkurState.PARKUR_1

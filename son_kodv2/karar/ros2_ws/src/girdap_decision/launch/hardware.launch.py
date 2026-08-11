"""
Girdap İDA — Gerçek donanım launch (ArduRover + Pixhawk 6C).

Sahada/gerçek suda çalışan tam yığın. mock_sensors YOKTUR; sensör verisi
gerçek MAVROS + sensör sürücülerinden gelir.

Bileşenler:
    - MAVROS (mavros/apm.launch include) — Pixhawk MAVLink köprüsü, fcu_url
      config/hardware.yaml'dan.
    - Static TF: base_link → livox_frame / oak_frame / imu_link. Değerler
      `config/hardware.yaml` `tf:` bloğundan okunur (tek doğruluk kaynağı).
      ✅ **ÖTELEMELER ÖLÇÜLDÜ** (2026-08-04 LiDAR/IMU/GPS · 2026-08-09 kamera;
      `docs/olcum_formu.md` §0-§3, `docs/sensor_konumlari_base_link.md`).
      ⏳ Kalan: **LiDAR `yaw`** (ampirik, §0.5 — merkez hattına 10 m ileriye
      hedef, `/perception/obstacle_map`'te `position.y ≈ 0` olmalı) ve kamera
      `yaw`'ının suda teyidi.
      ⚠️ Bu satır 2026-08-11'e kadar "kalibre EDİLMEMİŞ, 0,0,0" diyordu ve
      **bayattı** — yapılacaklar belgesi (Alt Alan A, TF ağacı) tam bu cümleyi
      kanıt gösteriyordu. Ölçümler girildiği hâlde launch dosyası kendini
      kalibrasyonsuz ilan etmeye devam ediyordu.
    - Karar yığını: fusion, planning, mavros_bridge, fsm, telemetry.
    - Sensör sürücüleri (Livox / OAK-D) BU LAUNCH'TA YOK — sensör bring-up'ı
      başka ekip üyesinde (aşağıdaki işaretli yorum).

Kullanım:
    ros2 launch girdap_decision hardware.launch.py
    ros2 launch girdap_decision hardware.launch.py fcu_url:=serial:///dev/ttyUSB0:921600

Konfig kaynakları:
    config/hardware.yaml  → fcu_url/gcs_url + mavros_bridge güvenlik override'ları
    config/params.yaml    → algoritma parametreleri (MPPI, fusion hızları, ...)
"""

import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PKG = "girdap_decision"


def _default_localizer_model() -> str:
    """Pakete gömülü YOLO duba lokalizatörünün (models/best.pt) kurulu yolu.

    setup.py `models/*.pt`'yi `share/girdap_decision/models/` altına kurar;
    get_package_share_directory makineler-arası mutlak-yol farkını (host
    ~/ros2_ws ↔ container /root/ros2_ws) yener. Model kurulu değilse (colcon
    build yapılmamış / dosya yok) "" döner → BuoyLocalizer güvenli mock'a düşer
    (`mock = mock or not model_path`), davranış bozulmaz.
    """
    try:
        p = os.path.join(get_package_share_directory(_PKG), "models", "best.pt")
        return p if os.path.exists(p) else ""
    except Exception:       # paket bulunamazsa (kurulmamış) mock'a düş
        return ""


# hardware.yaml okunamazsa kullanılacak güvenli varsayılanlar (yarışma modu).
_HW_DEFAULTS = {
    "fcu_url": "serial:///dev/ttyACM0:57600",
    "gcs_url": "",
    "mode_name": "GUIDED",
    "heartbeat_timeout_s": 5.0,
    "arming_retry_max": 3,
}
# algorithm bloğu varsayılanı: dosya yoksa tam stack (yarışma) güvenli seçim.
_ALGO_DEFAULTS = {"use_isam2": True, "use_rrt": True, "use_mppi": True}
# B1/B2 blokları — varsayılanlar YARIŞMA modudur (GUIDED+MPPI). hardware.yaml
# video için "AUTO"/"fc" der; dosya okunamazsa yarışma varsayılanına düşülür
# (F-V.5 uyarısı zaten basılır).
_FSM_DEFAULTS: dict[str, tuple[object, type]] = {
    "start_on_mode": ("GUIDED", str),
    "start_on_arm_in_mode": (False, bool),   # F-V.6: video true, yarışma false
}
_BRIDGE_DEFAULTS: dict[str, tuple[object, type]] = {
    "auto_guided": (True, bool),
    "stream_rate_hz": (10, int),        # F-M.6: bağlantıda istenen akış hızı
    # F-P.14: RC kill/manuel-override kanal/eşikleri artık CLI-override
    # edilebilir (öncesinde yalnız node kaynağında hardcoded'du).
    "rc_kill_channel": (7, int),
    "rc_kill_threshold_pwm": (1500, int),
    "rc_manual_channel": (4, int),
    "rc_manual_threshold_pwm": (1700, int),
}
_TELEMETRY_DEFAULTS: dict[str, tuple[object, type]] = {
    "setpoint_source": ("girdap", str),
    "fc_cruise_setpoint_mps": (1.0, float),
    # 🔴 SERVO1=74=ThrottleRight=SAĞ · SERVO3=73=ThrottleLeft=SOL (07.08
    # fiziksel ölçümü — §0.12c/§0.13/`34ade34`). Eskiden 1/3 idi ve TERSTİ;
    # yalnız `setpoint_source: "fc"` kolunda okunuyor ama orada Dosya-2 CSV'si
    # ile Ekran-2c'yi sessizce ters etiketliyordu. hardware.yaml ile aynı.
    "fc_thrust_left_ch": (3, int),
    "fc_thrust_right_ch": (1, int),
}
# mission bloğu varsayılanı — yaml'da anahtar yoksa bu kazanır.
# 2026-08-11: taban yarışma olduğu için varsayılan da competition.
_MISSION_DEFAULT = "competition_mission.yaml"
# görev kaynağı (T0-f): file (araç üstü YAML) ↔ fc (YKİ→MAVROS WaypointList).
_MISSION_SOURCE_DEFAULT = "file"
_SKIP_HOME_DEFAULT = True
# F-V.7: fc modunda görev-yöneticisi varış/bekleme ayarları (AUTO video: dwell=0,
# çünkü FC waypoint'te durmaz). Yoksa params.yaml değerleri (yarışma) geçerli.
_MISSION_TIMING_DEFAULTS: dict[str, tuple[object, type]] = {
    "dwell_time_s": (2.0, float),
    "arrival_radius_m": (2.0, float),
}
# perception.lidar varsayılanları: (değer, ROS param tipi) — hardware.yaml
# perception.lidar bloğu override eder, launch-arg CLI'dan da override edilir.
_LIDAR_DEFAULTS: dict[str, tuple[float | int, type]] = {
    "z_min": (0.1, float),
    "z_max": (3.0, float),
    "cluster_tolerance": (0.5, float),
    "min_cluster_size": (5, int),
    "max_cluster_size": (500, int),
    "split_cell_m": (1.0, float),       # F5.4: büyük küme bölme ızgarası
    "max_range": (25.0, float),
    "voxel_size": (0.1, float),         # F5.3: clustering öncesi downsample
    "log_period_s": (5.0, float),
}
# perception.camera skaler varsayılanları (HSV dizileri yalnız params.yaml'da).
_CAMERA_DEFAULTS: dict[str, tuple[object, type]] = {
    "clahe_clip_limit": (2.0, float),
    "clahe_tile": (8, int),
    "min_area_px": (150, int),
    "morph_kernel_px": (5, int),
    # F-P.21/gerçek yarış kararı (2026-07-17): VARSAYILAN AÇIK — model_path
    # boş kaldığı sürece güvenli mock'a düşer, davranış değişmez.
    "use_yolo": (True, bool),
    "yolo_model_path": ("", str),
    # F-S.9: turuncu/sarı/kırmızı/yeşil/kahve için alternatif yol (eğitilmiş
    # lokalizatör + HSV). Model artık pakete gömülü (models/best.pt) → varsayılan
    # kurulu .pt yolu; yoksa "" (mock). CLI ile override: perception.camera.
    # yolo_localizer_model_path:=/baska/yol.pt
    "use_yolo_localizer": (True, bool),
    "yolo_localizer_model_path": (_default_localizer_model(), str),
    "yolo_localizer_min_coverage": (0.15, float),
    "log_period_s": (5.0, float),
}
# fusion (KÖK blok) — iSAM2 GPS/IMU smoother. ⚠ _FUSION_DEFAULTS ile
# karıştırma: o `perception.fusion` (kamera-LiDAR bearing) içindir.
# Anahtarlar fusion_node'un ROS parametre adlarıyla BİREBİR; gps_sigma_by_status
# sözlüğü düzleştirilir (ROS parametreleri sözlük taşımaz).
_ISAM2_DEFAULTS: dict[str, tuple[object, type]] = {
    "keyframe_rate_hz": (5.0, float),
    "gps_robust_enabled": (True, bool),
    "gps_huber_k": (1.345, float),
    "gps_sigma_gbas_fix": (0.05, float),
    "gps_sigma_sbas_fix": (0.50, float),
    "gps_sigma_fix": (2.50, float),
    # 11.08.2026: mutlak yön düzeltmesi (FC AHRS'i, /mavros/imu/data
    # orientation'dan) — bkz. prototype/fusion/pipeline.py
    # FusionPipelineConfig docstring'i. false → eski davranış (yalnız gyro,
    # sınırsız kayabilir).
    "heading_correction_enabled": (True, bool),
    "heading_sigma_psi": (0.05, float),
}
# hardware.yaml `fusion.gps_sigma_by_status.<yaml_key>` → ROS param adı.
_GPS_SIGMA_STATUS_KEYS = {
    "gbas_fix": "gps_sigma_gbas_fix",
    "sbas_fix": "gps_sigma_sbas_fix",
    "fix": "gps_sigma_fix",
}
# planning.mppi_* — MPPI saha tuning yüzeyi (2026-08-02). Değerler
# prototype/planning/mppi.py MPPIConfig ile AYNI olmalı; drift'i
# test_hardware_launch_config.py::test_mppi_launch_varsayilanlari_kodla_ayni
# (ROS'suz, ast ile okur) yakalar. λ nöbetçisi 0.0 = "parkur profili kazansın".
_MPPI_DEFAULTS: dict[str, tuple[object, type]] = {
    "mppi_lambda": (0.0, float),
    "mppi_sigma_u": (0.364, float),   # 06.08 ölçümü (bkz. MPPIConfig)
    "mppi_obstacle_margin": (1.0, float),
    "mppi_terminal_mode": ("lookahead", str),
    "mppi_terminal_lookahead_m": (3.0, float),   # 08.08 ölçümü (bkz. MPPIConfig)
    "mppi_ref_window_size": (100, int),
    "mppi_ref_window_enabled": (True, bool),
}
# --show-args çıktısında operatörün göreceği açıklamalar (sınırlar dahil).
_MPPI_ARG_DESC = {
    "mppi_lambda": "MPPI softmax sıcaklığı λ. 0 = parkur profili kazansın "
                   "(PARKUR1/2=10, PARKUR3=50); >0 profili ezer. λ=1 dejenere "
                   "(tek örnek seçimi), λ≥500 araç hedefe varamaz",
    "mppi_sigma_u": "MPPI kontrol gürültüsü σ (N, her thruster)",
    "mppi_obstacle_margin": "MPPI engel emniyet payı (m, SOFT ceza). "
                            "⚠ BÜYÜTME: 1.2'de ham güzergah noktasına sürüş "
                            "kırılıyor (2/4 nokta). Kapı direği payı için "
                            "gate_post_margin_m'i kullan",
    "mppi_terminal_mode": "Terminal hedef: lookahead | global (eski davranış)",
    "mppi_terminal_lookahead_m": "Terminal hedefin çapadan yay uzaklığı (m); "
                                 "≥ seyir_hızı × horizon olmalı",
    "mppi_ref_window_size": "Kayan referans penceresi ileri derinliği (nokta)",
    "mppi_ref_window_enabled": "false → eski tam tarama (16× yavaş, A/B için)",
}
# planning.gate_* — kapı takibi saha yüzeyi (2026-08-03). Sayısal değerler
# prototype/mission/gate_follower.py GateFollowerConfig ile AYNI olmalı;
# drift'i test_hardware_launch_config.py::test_gate_launch_varsayilanlari_
# kodla_ayni yakalar. Üç anahtar (enabled/class_id/use_classified) node'a
# özgüdür, GateFollowerConfig'te karşılığı yoktur.
_GATE_DEFAULTS: dict[str, tuple[object, type]] = {
    "gate_following_enabled": (True, bool),
    "edge_buoy_class_id": (0, int),
    "use_classified_obstacles": (True, bool),
    "hull_width_m": (0.785, float),
    "hull_length_m": (1.04, float),
    # B2 huni (09.08): kapı direğinin ceza payı ÜST sınırı. Payın kendisi
    # ölçülen açıklıktan türer (planning_node._huni_payi) — bu yalnız tavan.
    "gate_post_margin_m": (1.4, float),
}
_GATE_ARG_DESC = {
    "gate_following_enabled": "Kapı (kenar dubası ikilisi) orta noktası takibi. "
                              "false → ham görev noktasına git (md 5.5.2.2 puanı "
                              "kapıdan geçmekten gelir)",
    "edge_buoy_class_id": "Turuncu KENAR dubasının sınıf kimliği; bu sınıf engel "
                          "torbasından çıkarılıp kapı adayı sayılır",
    "use_classified_obstacles": "/perception/classified_obstacles aktığında "
                                "sınıfsız obstacle_map'in yerine geçsin mi",
    "hull_width_m": "Gövde genişliği (m, ÖLÇÜLMÜŞ). Kapı bundan darsa tekne "
                    "sığmaz → kapı sayılmaz. Ayar değil, tekne boyutu.",
    "hull_length_m": "Gövde boyu (m, ÖLÇÜLMÜŞ). Yarısı = burun hattı; duba "
                     "bunun önünde olmalı. Ayar değil, tekne boyutu.",
    "gate_post_margin_m": "Kapı direği ceza payının ÜST sınırı (m). Payın "
                          "kendisi ölçülen açıklıktan türer, dar kapıda "
                          "kendiliğinden küçülür. 1.4 ölçülmüş (gövde payı "
                          "+0,31 m); 1.0'da temas, 1.8'de son güzergah "
                          "noktasına varılamıyor (nokta dubaya 2,0 m yakın)",
}
# perception.fusion varsayılanları — kamera-LiDAR bearing füzyonu (Sprint 3).
_FUSION_DEFAULTS: dict[str, tuple[object, type]] = {
    "bearing_tolerance_rad": (0.15, float),
    "camera_hfov_rad": (1.2, float),
    # 2026-07-17: oakd_driver_node 1280x720'e çıkarıldı (640x480'de sahada
    # 2m'deki duba bile net görülemiyordu) — eşleşmezse bearing yanlış çıkar.
    "camera_image_width_px": (1280, int),
    "camera_image_height_px": (720, int),
    "sync_slop_s": (0.1, float),
    # 2026-07-09 tezgah ölçümü (gerçek Livox + OAK): LiDAR clustering gecikince
    # eşleşme HİÇ olmuyordu; 10 ve 50 yetmedi, 100 tuttu. Damgalar aynı tabanda
    # (~27 ms) — bu bir gecikme sorunu, slop sorunu DEĞİL. Düzeltme 14.07 klasör
    # taşınmasında düşmüş, 09.08'de algı ekibinin raporuyla geri geldi.
    "sync_queue_size": (100, int),
    "log_period_s": (5.0, float),
}


def _deep_merge(base: dict, over: dict) -> dict:
    """`over`u `base`in üstüne özyinelemeli bindirir (base DEĞİŞMEZ).

    Sözlük değerler birleştirilir, diğer her tip üzerine yazılır. Overlay'in
    yazmadığı anahtarlar base'den aynen gelir — yarışma dosyasının yalnız
    farkları içerebilmesinin sebebi bu.
    """
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _load_hardware_config() -> dict:
    """config/hardware.yaml'ı oku; eksik/bulunamazsa varsayılanlara düş."""
    cfg = dict(_HW_DEFAULTS)
    cfg.update(_ALGO_DEFAULTS)
    cfg["lidar"] = {k: v for k, (v, _) in _LIDAR_DEFAULTS.items()}
    cfg["camera"] = {k: v for k, (v, _) in _CAMERA_DEFAULTS.items()}
    cfg["fusion"] = {k: v for k, (v, _) in _FUSION_DEFAULTS.items()}
    # kök `fusion:` bloğu (iSAM2) — perception.fusion'dan AYRI anahtar.
    cfg["isam2"] = {k: v for k, (v, _) in _ISAM2_DEFAULTS.items()}
    cfg["mission_file"] = _MISSION_DEFAULT
    cfg["mission_source"] = _MISSION_SOURCE_DEFAULT
    cfg["skip_home_seq0"] = _SKIP_HOME_DEFAULT
    # madde #4: Parkur-3 hedef rengi — varsayilan BOS = hedef atanmamis.
    cfg["kamikaze_target_color"] = ""
    cfg["mission_timing"] = {
        k: v for k, (v, _) in _MISSION_TIMING_DEFAULTS.items()
    }
    for block, defaults in (
        ("fsm", _FSM_DEFAULTS),
        ("bridge", _BRIDGE_DEFAULTS),
        ("telemetry", _TELEMETRY_DEFAULTS),
    ):
        cfg[block] = {k: v for k, (v, _) in defaults.items()}
    # planning mod geçidi kök mode_name'i miras alır (drift önlemek için).
    cfg["planning_mode"] = cfg["mode_name"]
    cfg["mppi"] = {k: v for k, (v, _) in _MPPI_DEFAULTS.items()}
    cfg["gate"] = {k: v for k, (v, _) in _GATE_DEFAULTS.items()}
    cfg["tf"] = {}                      # ölçüm girilene kadar boş = hepsi 0
    try:
        cfg_dir = os.path.join(get_package_share_directory(_PKG), "config")
        with open(
            os.path.join(cfg_dir, "hardware.yaml"), "r", encoding="utf-8"
        ) as fh:
            data = yaml.safe_load(fh) or {}
        # --- Yarışma/senaryo overlay'i (F-B.1) ---
        # GIRDAP_CONFIG_OVERLAY=yarisma.yaml → o dosya hardware.yaml'ın ÜSTÜNE
        # bindirilir; yalnız FARKLARI içerir, yazmadığı her anahtar
        # hardware.yaml'dan miras kalır. Ayrı bir tam kopya tutmuyoruz çünkü
        # iki tam dosya zamanla birbirinden kayar (drift) ve yarışma sabahı
        # hangisinin güncel olduğu belirsizleşir.
        #
        # 2026-08-08: ZİNCİR desteklendi — `yarisma.yaml:parkur1.yaml` gibi
        # `:` ile ayrılmış liste SOLDAN SAĞA bindirilir. Gerekçe aynı drift
        # kaygısı: Parkur-1 test günü yarışma ayarlarının TAMAMINI ister,
        # tek farkı görev etiketi dosyasıdır (parkur1.yaml). Zincir olmasaydı
        # yarisma.yaml'ın kopyası çıkarılacaktı — bu dosyanın en başında
        # "yapmayacağız" denen şeyin ta kendisi.
        overlay_ham = os.environ.get("GIRDAP_CONFIG_OVERLAY", "").strip()
        for overlay_name in [p.strip() for p in overlay_ham.split(":") if p.strip()]:
            with open(
                os.path.join(cfg_dir, overlay_name), "r", encoding="utf-8"
            ) as fh:
                data = _deep_merge(data, yaml.safe_load(fh) or {})
            print(
                f"*** config overlay UYGULANDI: {overlay_name} "
                "(hardware.yaml üstüne)",
                file=sys.stderr,
            )
        for key in _HW_DEFAULTS:
            if key in data:
                cfg[key] = data[key]
        # tf: bloğu — sensör montaj offset'leri (docs/olcum_formu.md §2/§3)
        tf_block = data.get("tf")
        if isinstance(tf_block, dict):
            cfg["tf"] = tf_block
        # algorithm: bloğu (video ↔ yarışma modu seçimi)
        algo = data.get("algorithm") or {}
        for key in _ALGO_DEFAULTS:
            if key in algo:
                cfg[key] = bool(algo[key])
        # planning: bloğu varsa mode_name'ini kullan, yoksa kökü miras al.
        planning_block = data.get("planning") or {}
        cfg["planning_mode"] = str(
            planning_block.get("mode_name", cfg["mode_name"])
        )
        # planning.mppi_* — saha tuning; verilmeyen anahtarda kod varsayılanı
        # (λ'da parkur profili) kazanır.
        for key, (_, cast) in _MPPI_DEFAULTS.items():
            if key in planning_block:
                cfg["mppi"][key] = cast(planning_block[key])
        # planning.gate_* — kapı takibi saha tuning (aynı öncelik zinciri).
        for key, (_, cast) in _GATE_DEFAULTS.items():
            if key in planning_block:
                cfg["gate"][key] = cast(planning_block[key])
        # mission: görev dosyası + kaynak seçimi (video ↔ competition, file ↔ fc)
        mission_block = data.get("mission") or {}
        cfg["mission_file"] = str(
            mission_block.get("mission_file", _MISSION_DEFAULT)
        )
        cfg["mission_source"] = str(
            mission_block.get("mission_source", _MISSION_SOURCE_DEFAULT)
        )
        cfg["skip_home_seq0"] = bool(
            mission_block.get("skip_home_seq0", _SKIP_HOME_DEFAULT)
        )
        cfg["kamikaze_target_color"] = str(
            mission_block.get("kamikaze_target_color", "")
        )
        for key, (_, cast) in _MISSION_TIMING_DEFAULTS.items():
            if key in mission_block:
                cfg["mission_timing"][key] = cast(mission_block[key])
        # B1/B2: fsm / bridge / telemetry blokları (AUTO video ↔ yarışma)
        for block, defaults in (
            ("fsm", _FSM_DEFAULTS),
            ("bridge", _BRIDGE_DEFAULTS),
            ("telemetry", _TELEMETRY_DEFAULTS),
        ):
            values = data.get(block) or {}
            for key, (_, cast) in defaults.items():
                if key in values:
                    cfg[block][key] = cast(values[key])
        # perception.lidar / perception.camera / perception.fusion blokları
        # (Sprint 1 + 2 + 3)
        perception = data.get("perception") or {}
        for block, defaults in (
            ("lidar", _LIDAR_DEFAULTS),
            ("camera", _CAMERA_DEFAULTS),
            ("fusion", _FUSION_DEFAULTS),
        ):
            values = perception.get(block) or {}
            for key, (_, cast) in defaults.items():
                if key in values:
                    cfg[block][key] = cast(values[key])
        # kök `fusion:` bloğu — iSAM2 smoother (keyframe throttle + robust GPS).
        isam2_block = data.get("fusion") or {}
        for key, (_, cast) in _ISAM2_DEFAULTS.items():
            if key in isam2_block:
                cfg["isam2"][key] = cast(isam2_block[key])
        # gps_sigma_by_status alt sözlüğü → düzleştirilmiş ROS param adları
        sigma_block = isam2_block.get("gps_sigma_by_status") or {}
        for yaml_key, param_name in _GPS_SIGMA_STATUS_KEYS.items():
            if yaml_key in sigma_block:
                cfg["isam2"][param_name] = float(sigma_block[yaml_key])
    except Exception as exc:                # paket kurulmadan --show-args vb.
        # F-V.5 (F3.3): fallback SESSİZ OLMAMALI — hardware.yaml'daki bir yazım
        # hatası video-modu bayraklarını (use_isam2/use_rrt=false) kaybettirip
        # kalibrasyonsuz TAM STACK'i açar (md 3.3.1.1 istemsiz-hareket riski).
        # Fallback davranışı korunur; operatör stderr + LogInfo satırından
        # (launch sonu "algorithm:" özeti) durumu DOĞRULAMALI.
        print(
            "\n*** UYARI: config/hardware.yaml OKUNAMADI — varsayılanlara "
            f"düşüldü ({exc!r}).\n"
            "*** Varsayılan = YARIŞMA modu: use_isam2=True, use_rrt=True, "
            "mission_source=file.\n"
            "*** VİDEO çekiyorsan bu YANLIŞTIR: yaml'ı düzelt ya da "
            "use_isam2:=false use_rrt:=false mission_source:=fc ver.\n",
            file=sys.stderr,
        )
    return cfg


def _mount_params(child: str, tf_cfg: dict | None = None) -> dict:
    """`tf:` bloğundaki montaj offset'ini node parametresi sözlüğüne çevirir.

    B0/F5.1: perception_lidar_node ham bulutu KENDİ dönüştürüyor (tf2 lookup
    yok — başlangıçta TF hazır değilken sessizce 0 dönüşüm uygulamak, tam da
    kapatmaya çalıştığımız sessiz arızanın kendisi olurdu). Değer yine de
    static TF yayıncısıyla AYNI bloktan okunur → iki kopya yok, drift yok
    (guard: test_hardware_launch_config).
    """
    v = (tf_cfg or {}).get(child) or {}
    return {
        "mount_x": float(v.get("x", 0.0)),
        "mount_y": float(v.get("y", 0.0)),
        "mount_z": float(v.get("z", 0.0)),
        "mount_yaw": float(v.get("yaw", 0.0)),
    }


def _static_tf(parent: str, child: str, tf_cfg: dict | None = None) -> Node:
    """Sensör montaj offset'ini hardware.yaml `tf:` bloğundan okuyan static TF.

    Değerler ÖLÇÜLMÜŞ olmalı (`docs/olcum_formu.md` §2/§3). Blok yoksa ya da
    anahtar eksikse 0 kullanılır — yani ölçüm girilene kadar davranış eski
    (kalibre edilmemiş) haliyle BİREBİR aynıdır.

    Eksen kuralı (REP-103): +x pruva, +y iskele, +z yukarı; açılar RADYAN
    (static_transform_publisher radyan bekler, form derece topluyor → çevir).
    """
    v = (tf_cfg or {}).get(child) or {}
    def _g(key: str) -> str:
        return str(float(v.get(key, 0.0)))
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{child}",
        arguments=[
            "--x", _g("x"), "--y", _g("y"), "--z", _g("z"),
            "--yaw", _g("yaw"), "--pitch", _g("pitch"), "--roll", _g("roll"),
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )


def generate_launch_description() -> LaunchDescription:
    hw = _load_hardware_config()
    share = get_package_share_directory(_PKG)
    params_file = os.path.join(share, "config", "params.yaml")

    # --- Launch argümanları ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    use_isam2 = LaunchConfiguration("use_isam2")
    use_rrt = LaunchConfiguration("use_rrt")

    def _bool_default(value: bool) -> str:
        return "true" if value else "false"

    declared = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Gerçek donanım → sim zamanı KAPALI",
        ),
        DeclareLaunchArgument(
            "fcu_url", default_value=str(hw["fcu_url"]),
            description="Pixhawk MAVLink bağlantısı (hardware.yaml varsayılanı)",
        ),
        DeclareLaunchArgument(
            "gcs_url", default_value=str(hw["gcs_url"]),
            description="YKİ köprüsü (boş = kapalı — Şartname 4.1)",
        ),
        # algorithm.* — video ↔ yarışma modu (hardware.yaml varsayılanı,
        # CLI'dan override edilebilir: use_isam2:=true vb.).
        DeclareLaunchArgument(
            "use_isam2", default_value=_bool_default(hw["use_isam2"]),
            description="true: iSAM2 füzyon | false: MAVROS EKF geçişi (video)",
        ),
        DeclareLaunchArgument(
            "use_rrt", default_value=_bool_default(hw["use_rrt"]),
            description="true: RRT* global plan | false: düz hedef → MPPI (video)",
        ),
        # F-S.10: yerel kontrolcü seçimi — mppi (varsayılan) | pid (ida_topics
        # cascade PID + LiDAR kaçınma, MPPI saha kalibrasyonu tamamlanana
        # kadar düşme-güvenli yedek).
        DeclareLaunchArgument(
            "control_mode", default_value="mppi",
            description="planning_node yerel kontrolcüsü: mppi | pid (F-S.10)",
        ),
        DeclareLaunchArgument(
            "use_onboard_camera", default_value="false",
            description="HSV YEDEK kamera node'u (perception_camera_node). "
                        "VARSAYILAN false (2026-08-04, algı ekibi kararı): "
                        "/perception/buoys'un ASIL üreticisi artık repoda — "
                        "son_kodv2/algi (girdap-ida-algi, DepthAI ile OAK-D'yi "
                        "doğrudan açar, YOLO kameranın VPU'sunda). İkisi aynı "
                        "anda açılırsa hem topic'te ÇİFT PUBLISHER olur hem de "
                        "bbox piksel uzayları farklı olduğu için füzyon bearing'i "
                        "karışır. Geçmiş (F-P.22, 2026-07-17): varsayılan geçici "
                        "olarak true yapılmıştı çünkü algı paketi o ortamda hiç "
                        "yoktu ve /perception/buoys sessizce hiç üretilmedi; "
                        "artık paket burada ve fusion sync bekçisi de bu sessiz "
                        "hâli WARN'la yakalıyor. ⚠ true yaparsan algı node'unu "
                        "kapatmak ZORUNDASIN (tek OAK, tek süreç açabilir).",
        ),
        DeclareLaunchArgument(
            "use_mppi", default_value=_bool_default(hw["use_mppi"]),
            description="REZERVE (F3.2): şu an HİÇBİR node okumuyor — MPPI her "
                        "iki modda da aktif; false vermek davranışı DEĞİŞTİRMEZ",
        ),
        # perception.lidar.* / perception.camera.* — hardware.yaml varsayılanı,
        # CLI override: perception.lidar.z_min:=0.2, perception.camera.use_yolo:=true
        *[
            DeclareLaunchArgument(
                f"perception.lidar.{key}", default_value=str(hw["lidar"][key]),
                description=f"LiDAR engel tespiti: {key}",
            )
            for key in _LIDAR_DEFAULTS
        ],
        *[
            DeclareLaunchArgument(
                f"perception.camera.{key}",
                default_value=(
                    _bool_default(hw["camera"][key])
                    if isinstance(hw["camera"][key], bool)
                    else str(hw["camera"][key])
                ),
                description=f"Kamera duba tespiti: {key}",
            )
            for key in _CAMERA_DEFAULTS
        ],
        *[
            DeclareLaunchArgument(
                f"perception.fusion.{key}", default_value=str(hw["fusion"][key]),
                description=f"Kamera-LiDAR bearing füzyonu: {key}",
            )
            for key in _FUSION_DEFAULTS
        ],
        # planning.mppi_* — MPPI saha tuning (hardware.yaml planning: bloğu
        # varsayılanı; CLI: planning.mppi_lambda:=50.0). Ölçümler CLAUDE.md.
        *[
            DeclareLaunchArgument(
                f"planning.{key}",
                default_value=(
                    _bool_default(hw["mppi"][key])
                    if isinstance(hw["mppi"][key], bool)
                    else str(hw["mppi"][key])
                ),
                description=_MPPI_ARG_DESC[key],
            )
            for key in _MPPI_DEFAULTS
        ],
        # planning.gate_* — kapı takibi saha tuning (md 5.5.2.2).
        *[
            DeclareLaunchArgument(
                f"planning.{key}",
                default_value=(
                    _bool_default(hw["gate"][key])
                    if isinstance(hw["gate"][key], bool)
                    else str(hw["gate"][key])
                ),
                description=_GATE_ARG_DESC[key],
            )
            for key in _GATE_DEFAULTS
        ],
        # fusion.* — iSAM2 smoother (keyframe throttle + robust GPS + fix
        # kalitesi sigma'ları). CLI: fusion.keyframe_rate_hz:=10.0
        *[
            DeclareLaunchArgument(
                f"fusion.{key}",
                default_value=(
                    _bool_default(hw["isam2"][key])
                    if isinstance(hw["isam2"][key], bool)
                    else str(hw["isam2"][key])
                ),
                description=f"iSAM2 sensör füzyonu: {key}",
            )
            for key in _ISAM2_DEFAULTS
        ],
        # mission_file — görev dosyası (Sprint 4 parkur katmanı). config/ altında
        # çözülür; CLI override: mission_file:=competition_mission.yaml
        DeclareLaunchArgument(
            "mission_file", default_value=str(hw["mission_file"]),
            description="Görev dosyası (config/ altında): video ↔ competition",
        ),
        # mission_source — görev kaynağı (T0-f). CLI: mission_source:=fc
        DeclareLaunchArgument(
            "mission_source", default_value=str(hw["mission_source"]),
            description="Görev kaynağı: file (araç üstü YAML) | fc (YKİ→MAVROS)",
        ),
        # with_drivers — sensör sürücüleri (ida_topics paketi, F-S.2). false =
        # video günü (AUTO görevine sensör gerekmez, MAVROS yeter); true =
        # final/algı testleri (Livox UDP + OAK-D + Dosya-1 kamera kaydı).
        DeclareLaunchArgument(
            "with_drivers", default_value="false",
            description="Sensör sürücülerini başlat: Livox (+ OAK-D/kamera kaydı "
                        "yalnız with_oak_driver:=true ise — aşağıya bak)",
        ),
        # 🔴 2026-08-04 (algı ekibi): OAK sürücüsü with_drivers'tan AYRILDI.
        # Önceden tek bayrak Livox + OAK sürücüsü + kamera kaydını birlikte
        # açıyordu. Ama LiDAR'a ihtiyaç var ve OAK'ı bizim algı node'umuz
        # (son_kodv2/algi) DOĞRUDAN DepthAI ile açıyor — tek USB cihazını iki
        # süreç açamaz. Yani "with_drivers:=true" demek LiDAR'ı açarken kamerayı
        # TAMAMEN ÖLDÜRMEK demekti; sahada belirti de vermiyordu.
        # kamera_kayit_node da buraya bağlı: /oak/rgb/image_raw'ı yalnız
        # oakd_driver_node üretiyor, o kapalıyken kaydedici BOŞ mp4 yazardı.
        # Dosya-1 (md 4.2) mp4'ünü zaten algı node'u üretiyor (bbox + SINIF
        # etiketi + her karede zaman damgası).
        DeclareLaunchArgument(
            "with_oak_driver", default_value="false",
            description="OAK-D sürücüsü + kamera kayıt node'unu başlat. "
                        "VARSAYILAN false: kamerayı algı node'u açıyor. "
                        "true yaparsan algı node'unu KAPAT (tek OAK).",
        ),
        # with_mavros — masa testi (Pixhawk yok/bağlı değil). false: gerçek
        # mavros yerine mevcut mock_sensors node'u (/mavros/imu/data,
        # /mavros/global_position/global, /mavros/state) karar yığınını besler
        # — fsm/mission_manager/planning/telemetry uçtan uca canlı test edilir.
        DeclareLaunchArgument(
            "with_mavros", default_value="true",
            description="false: gerçek MAVROS yerine mock_sensors (masa testi)",
        ),
        # F-P.19 — mock_sensors'ın dalgalı deniz/kesinti parametreleri artık
        # TEK launch komutundan geçilebilir (öncesinde ayrı `ros2 run` ile
        # ELLE başlatmak gerekiyordu — bu da launch'ın KENDİ mock_sensors
        # kopyasıyla ÇAKIŞIP aynı topic'lere iki yayıncı basmasına, testin
        # sonucunu maskelemesine yol açtı, 2026-07-15'te canlı bulundu).
        DeclareLaunchArgument(
            "mock.wave_roll_amp_deg", default_value="0.0",
            description="Masa testi: dalga roll salınım genliği (derece), 0=kapalı",
        ),
        DeclareLaunchArgument(
            "mock.wave_pitch_amp_deg", default_value="0.0",
            description="Masa testi: dalga pitch salınım genliği (derece), 0=kapalı",
        ),
        DeclareLaunchArgument(
            "mock.wave_period_s", default_value="4.0",
            description="Masa testi: dalga salınım periyodu (s)",
        ),
        DeclareLaunchArgument(
            "mock.wave_accel_noise_mss", default_value="0.0",
            description="Masa testi: dalga jerk gürültüsü (m/s²), 0=kapalı",
        ),
        DeclareLaunchArgument(
            "mock.dropout_period_s", default_value="0.0",
            description="Masa testi: periyodik GPS/IMU kesinti periyodu (s), 0=kapalı",
        ),
        DeclareLaunchArgument(
            "mock.dropout_duration_s", default_value="2.0",
            description="Masa testi: her periyottaki kesinti süresi (s)",
        ),
        # fsm.start_on_mode / fsm.start_on_arm_in_mode — hardware.yaml
        # varsayılanını launch-arg'dan override edebilmek için (masa testinde
        # mock_sensors sabit armed+GUIDED yayınlar, hardware.yaml'a dokunmadan
        # start_on_arm_in_mode:=true ile görevi başlatabilmek için gerekli).
        DeclareLaunchArgument(
            "fsm.start_on_mode", default_value=str(hw["fsm"]["start_on_mode"]),
            description="BEKLEMEDE'de bu moda geçiş görevi başlatır",
        ),
        DeclareLaunchArgument(
            "fsm.start_on_arm_in_mode",
            default_value=_bool_default(hw["fsm"]["start_on_arm_in_mode"]),
            description="true: BEKLEMEDE'ye zaten start_on_mode'dayken ARM "
                        "girildiğinde de (kenar yok) görev başlar (F-V.6)",
        ),
        # bridge.auto_guided — hardware.yaml varsayılanı video modu (false,
        # B1: FC AUTO'da köprü mod savaşı açmasın). YARIŞMA modu (use_rrt=
        # true) testinde/gününde bunu CLI'dan true'ya çevirmeyi UNUTMAK —
        # gerçek parkur SITL testinde bulunan bir gap — FCU geçici bir
        # EKF failsafe'den HOLD'a düşünce köprü GUIDED'ı geri talep etmiyor,
        # araç sonsuza dek hareketsiz kalıyordu.
        DeclareLaunchArgument(
            "bridge.auto_guided",
            default_value=_bool_default(hw["bridge"]["auto_guided"]),
            description="true: mod hedefte değilse köprü otomatik GUIDED "
                        "talep eder (yarışma ŞART; video: false, FC AUTO sürer)",
        ),
        # F-P.14: RC kill/manuel-override — sahada RC alıcısı kanal
        # kablolaması değişirse kod değiştirip yeniden derlemeden CLI'dan
        # ayarlanabilsin diye (öncesinde yalnız node kaynağında hardcoded).
        DeclareLaunchArgument(
            "bridge.rc_kill_channel",
            default_value=str(hw["bridge"]["rc_kill_channel"]),
            description="RC KILL kanalı (0-indeksli; varsayılan 7 = kanal 8)",
        ),
        DeclareLaunchArgument(
            "bridge.rc_kill_threshold_pwm",
            default_value=str(hw["bridge"]["rc_kill_threshold_pwm"]),
            description="Bu PWM'in ALTI → KILL",
        ),
        DeclareLaunchArgument(
            "bridge.rc_manual_channel",
            default_value=str(hw["bridge"]["rc_manual_channel"]),
            description="RC manuel-override kanalı (0-indeksli; varsayılan 4 = kanal 5)",
        ),
        DeclareLaunchArgument(
            "bridge.rc_manual_threshold_pwm",
            default_value=str(hw["bridge"]["rc_manual_threshold_pwm"]),
            description="Bu PWM'in ÜSTÜ → manuel override",
        ),
    ]

    # --- MAVROS: ArduRover köprüsü ---
    # F-P.20 (2026-07-16, gerçek donanım testi): FTDI/seri bağlantı bir
    # anlığına EOF verdiğinde mavros_node yakalanmamış bir istisnayla
    # ÇÖKÜYOR (SIGABRT) — hiçbir şey onu geri başlatmıyordu, tüm karar
    # yığını kalıcı olarak kör kalıyordu (planning/mission_manager kendi
    # stale-guard'larıyla GÜVENLİ davranıyor ama sistem kendini
    # toparlamıyordu). apm.launch/node.launch'ın kendi `respawn_mavros`
    # argümanı VAR ama node.launch'taki <node> etiketine hiç bağlanmamış
    # (mavros paketinin kendi hatası, `respawn_mavros:=true` geçmek hiçbir
    # şey yapmıyor — canlı testte ikinci çökmede doğrulandı). Bu yüzden
    # apm.launch include'unu bypass edip mavros_node'u BURADA doğrudan
    # Node() ile açıyoruz — respawn=True launch_ros'ta gerçekten çalışıyor.
    _mavros_share = get_package_share_directory("mavros")
    mavros = Node(
        package="mavros", executable="mavros_node", namespace="mavros",
        output="screen",
        respawn=True, respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("with_mavros")),
        parameters=[
            {
                "fcu_url": fcu_url, "gcs_url": gcs_url,
                "tgt_system": 1, "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
            os.path.join(_mavros_share, "launch", "apm_pluginlists.yaml"),
            os.path.join(_mavros_share, "launch", "apm_config.yaml"),
            # 🔴 apm_config.yaml'DAN SONRA gelmeli (sonraki dosya kazanır).
            # setpoint_velocity.mav_frame: LOCAL_NED → BODY_NED.
            # Gerekçe ve ölçümler: config/mavros_overrides.yaml.
            # Özeti: planning_node GÖVDE çerçevesinde surge basıyor ama
            # LOCAL_NED'de ROS linear.x (ENU doğu) NED'in vy alanına düşüyor,
            # vx sıfır kalıyor → Rover işaretli hızı vx'ten okuduğu için
            # GERİ komutu İLERİ olarak uygulanıyordu (07.08'de gerçek
            # donanımda ölçüldü ve kaptan tarafından gözlendi).
            os.path.join(share, "config", "mavros_overrides.yaml"),
        ],
    )
    # Masa testi (with_mavros:=false): gerçek Pixhawk/MAVROS yok, mevcut
    # mock_sensors node'u aynı topic'leri (imu/gps/state) sentetik veriyle
    # besler — karar yığını (fsm/mission_manager/planning/telemetry/bridge)
    # donanımsız uçtan uca çalıştırılabilir.
    mock_sensors_node = Node(
        package=_PKG, executable="mock_sensors", name="mock_sensors",
        condition=UnlessCondition(LaunchConfiguration("with_mavros")),
        output="screen",
        parameters=[{
            "wave_roll_amp_deg": ParameterValue(
                LaunchConfiguration("mock.wave_roll_amp_deg"), value_type=float
            ),
            "wave_pitch_amp_deg": ParameterValue(
                LaunchConfiguration("mock.wave_pitch_amp_deg"), value_type=float
            ),
            "wave_period_s": ParameterValue(
                LaunchConfiguration("mock.wave_period_s"), value_type=float
            ),
            "wave_accel_noise_mss": ParameterValue(
                LaunchConfiguration("mock.wave_accel_noise_mss"), value_type=float
            ),
            "dropout_period_s": ParameterValue(
                LaunchConfiguration("mock.dropout_period_s"), value_type=float
            ),
            "dropout_duration_s": ParameterValue(
                LaunchConfiguration("mock.dropout_duration_s"), value_type=float
            ),
        }],
    )

    # --- Static TF (kalibre edilmemiş; mekanik ekip gerçek ölçümle günceller) ---
    static_tfs = [
        _static_tf("base_link", "livox_frame", hw["tf"]),
        _static_tf("base_link", "oak_frame", hw["tf"]),
        _static_tf("base_link", "imu_link", hw["tf"]),
    ]

    # --- Karar yığını node'ları ---
    common = {
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
        "output": "screen",
    }
    # --- Saat güveni (md 4.2) — TEK yerde hesaplanır, ÜÇ teslime birlikte geçer.
    # 🔴 09.08'e kadar bu tesisat YARIM BAĞLIYDI: local_map_node ve
    # lidar_kayit_node `saat_guvenilir`i OKUYOR ama hiçbir yerden
    # BESLENMİYORDU → varsayılan True'da kalıyor, yani sistem saat 3 saat
    # yanlışken de "güvenilir" diyordu (Jetson 06.08'de ~15 sa, 07.08'de ~3 sa
    # geri açıldı — ölçüldü). telemetry_node'da (Dosya-2) parametre HİÇ yoktu.
    # Ölçüt `prototype/telemetry/saat_guveni.py`de (çekirdek STA_UNSYNC bayrağı);
    # saati GPS'ten KURAN taraf `scripts/girdap_saat_kur.py` + girdap-saat.service.
    try:
        from prototype.telemetry.saat_guveni import saat_guvenilir_mi
        _saat_ok, _saat_neden = saat_guvenilir_mi()
    except Exception as _e:  # PYTHONPATH kırıksa yığın zaten çalışmaz (F2.3)
        _saat_ok, _saat_neden = False, f"olcut yuklenemedi ({_e})"
    # Teslim node'larına ayrı sözlük: `common` başka node'larca da kullanılıyor,
    # onları gereksiz parametreyle kirletmiyoruz.
    teslim_common = {
        "parameters": common["parameters"] + [{"saat_guvenilir": _saat_ok}],
        "output": "screen",
    }
    # mavros_bridge: hardware.yaml güvenlik değerleri params.yaml'ı override eder.
    bridge_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "mode_name": str(hw["mode_name"]),
            "heartbeat_timeout_s": float(hw["heartbeat_timeout_s"]),
            "arming_retry_max": int(hw["arming_retry_max"]),
            # B1: AUTO videosunda köprü GUIDED'a zorlamaz (mod savaşı yok).
            "auto_guided": ParameterValue(
                LaunchConfiguration("bridge.auto_guided"), value_type=bool
            ),
            # F-M.6: bağlantı kenarında FC akış hızı isteği (1 Hz sorunu).
            "stream_rate_hz": int(hw["bridge"]["stream_rate_hz"]),
            # F-P.14: RC kill/manuel-override artık CLI-override edilebilir.
            "rc_kill_channel": ParameterValue(
                LaunchConfiguration("bridge.rc_kill_channel"), value_type=int
            ),
            "rc_kill_threshold_pwm": ParameterValue(
                LaunchConfiguration("bridge.rc_kill_threshold_pwm"), value_type=int
            ),
            "rc_manual_channel": ParameterValue(
                LaunchConfiguration("bridge.rc_manual_channel"), value_type=int
            ),
            "rc_manual_threshold_pwm": ParameterValue(
                LaunchConfiguration("bridge.rc_manual_threshold_pwm"), value_type=int
            ),
        },
    ]
    # fusion: algorithm.use_isam2 (video → MAVROS EKF pass-through) +
    # kök `fusion:` bloğu (keyframe throttle, robust GPS, fix sigma'ları).
    fusion_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "use_isam2": ParameterValue(use_isam2, value_type=bool),
            **{
                key: ParameterValue(
                    LaunchConfiguration(f"fusion.{key}"), value_type=cast
                )
                for key, (_, cast) in _ISAM2_DEFAULTS.items()
            },
        },
    ]
    # planning: mode_name (tek kaynak) + algorithm.use_rrt (video → düz hedef).
    # heartbeat_timeout_s: mavros_bridge_node ile AYNI kaynaktan (hw[]) —
    # eskiden yalnız bridge_params'a geçiyordu, planning_node kendi
    # hardcoded 5.0 varsayılanında kalıyordu (config-drift riski taraması,
    # 2026-07-15: hardware.yaml'da tune edilirse iki güvenlik geçidi
    # farklı anda tetiklenirdi).
    planning_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "mode_name": str(hw["planning_mode"]),
            "use_rrt": ParameterValue(use_rrt, value_type=bool),
            "control_mode": LaunchConfiguration("control_mode"),
            "heartbeat_timeout_s": float(hw["heartbeat_timeout_s"]),
            # MPPI saha tuning (planning.mppi_* launch-arg'ları)
            **{
                key: ParameterValue(
                    LaunchConfiguration(f"planning.{key}"), value_type=cast
                )
                for key, (_, cast) in _MPPI_DEFAULTS.items()
            },
            # Kapı takibi saha tuning (planning.gate_* launch-arg'ları)
            **{
                key: ParameterValue(
                    LaunchConfiguration(f"planning.{key}"), value_type=cast
                )
                for key, (_, cast) in _GATE_DEFAULTS.items()
            },
        },
    ]
    # mission_file: config/ altında çözülen tam yol (video ↔ competition).
    # HEM mission_manager HEM fsm_node'a aynı dosya verilir → waypoint parkur
    # index'leri iki node arasında hizalı kalır (Sprint 4 parkur katmanı).
    mission_path = PathJoinSubstitution(
        [share, "config", LaunchConfiguration("mission_file")]
    )
    # mission_source (file↔fc) LaunchConfiguration'dan (CLI override); skip_home
    # hardware.yaml'dan. fc modunda mission_file yerine /mavros/mission/waypoints
    # okunur (T0-f).
    mission_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "mission_file": mission_path,
            "mission_source": LaunchConfiguration("mission_source"),
            "skip_home_seq0": bool(hw["skip_home_seq0"]),
            # F-V.7: AUTO'da FC durmadığı için dwell=0 (hardware.yaml).
            "dwell_time_s": float(hw["mission_timing"]["dwell_time_s"]),
            "arrival_radius_m": float(
                hw["mission_timing"]["arrival_radius_m"]
            ),
        },
    ]
    # fsm: parkur katmanı için aynı görev dosyası (waypoint parkur etiketleri)
    # + B1 başlatma modu (video: AUTO, yarışma: GUIDED).
    fsm_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "mission_file": mission_path,
            "start_on_mode": LaunchConfiguration("fsm.start_on_mode"),
            # F-V.6: AUTO'dayken arm edilirse de görev başlasın (video);
            # masa testinde (with_mavros:=false) mock_sensors sabit
            # armed+GUIDED yayınladığından kenar oluşmaz — true gerekir.
            "start_on_arm_in_mode": ParameterValue(
                LaunchConfiguration("fsm.start_on_arm_in_mode"), value_type=bool
            ),
            # F-P.8: mission_manager_node'un KENDİ mission_source'uyla AYNI
            # kaynak — fc+çoklu-parkur uyumsuzluk uyarısı için (bkz. fsm_node
            # ._build_parkur_logic).
            "mission_source": LaunchConfiguration("mission_source"),
        },
    ]
    # telemetry: B2 Ekran-2 kaynağı (video: fc = FC servo çıkışı, yarışma:
    # girdap = MPPI thrust'ı).
    telemetry_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "setpoint_source": str(hw["telemetry"]["setpoint_source"]),
            "fc_cruise_setpoint_mps": float(
                hw["telemetry"]["fc_cruise_setpoint_mps"]
            ),
            "fc_thrust_left_ch": int(hw["telemetry"]["fc_thrust_left_ch"]),
            "fc_thrust_right_ch": int(hw["telemetry"]["fc_thrust_right_ch"]),
            # Dosya-2: saat doğrulanamadıysa CSV adına yazılır (bkz. node).
            "saat_guvenilir": _saat_ok,
        },
    ]
    # perception: launch-arg'lar tip korunarak node parametresine geçer.
    def _perception_params(block: str, defaults: dict) -> list:
        return [
            params_file,
            {
                "use_sim_time": use_sim_time,
                **{
                    key: ParameterValue(
                        LaunchConfiguration(f"perception.{block}.{key}"),
                        value_type=cast,
                    )
                    for key, (_, cast) in defaults.items()
                },
            },
        ]

    decision_nodes = [
        # Sprint 1: /livox/lidar → /perception/obstacle_map (MPPI'dan önce —
        # planning engel listesini hazır bulsun).
        Node(package=_PKG, executable="perception_lidar_node",
             name="perception_lidar_node",
             # B0/F5.1: montaj offset'i EN SONDA — ROS parametre listesinde
             # sonraki sözlük öncekini ezer, yani `tf:` bloğu params.yaml'daki
             # olası bir kopyayı da bastırır (tek kaynak `tf:`).
             parameters=_perception_params("lidar", _LIDAR_DEFAULTS)
             + [_mount_params("livox_frame", hw["tf"])],
             output="screen"),
        # Sprint 2 — F3.1: VARSAYILAN KAPALI. /perception/buoys'un asıl
        # üreticisi algı ekibinin OAK node'u (girdap-ida-algi, DepthAI
        # doğrudan — VPU'da YOLO). Bu HSV node'u yalnız YEDEK; ikisi aynı
        # anda açılırsa hem topic çakışır hem OAK USB cihazı iki süreçte
        # açılamaz. Açmak için: use_onboard_camera:=true.
        Node(package=_PKG, executable="perception_camera_node",
             name="perception_camera_node",
             # madde #4: hedef rengi `mission:` bloğunda yaşıyor (kamera TUNING
             # ayarı değil, hakemin verdiği GÖREV bilgisi) → perception.camera.*
             # zincirine değil, buraya ayrı ekleniyor.
             parameters=_perception_params("camera", _CAMERA_DEFAULTS) + [
                 {"kamikaze_target_color": str(hw["kamikaze_target_color"])},
             ],
             condition=IfCondition(LaunchConfiguration("use_onboard_camera")),
             output="screen"),
        # Sprint 3: obstacle_map + buoys (sync) → /perception/classified_obstacles.
        # LiDAR+kamera node'larından SONRA gelmeli (mesajları tüketiyor).
        Node(package=_PKG, executable="perception_fusion_node",
             name="perception_fusion_node",
             # madde #4: hedef rengi burada da lazim — ASIL yer bu node, cunku
             # /perception/buoys'u algi ekibinin paketi uretse bile fuzyon
             # onun altinda kosuyor (use_onboard_camera varsayilani false).
             parameters=_perception_params("fusion", _FUSION_DEFAULTS) + [
                 {"kamikaze_target_color": str(hw["kamikaze_target_color"])},
             ],
             output="screen"),
        Node(package=_PKG, executable="fusion_node",
             name="fusion_node", parameters=fusion_params, output="screen"),
        Node(package=_PKG, executable="planning_node",
             name="planning_node", parameters=planning_params, output="screen"),
        Node(package=_PKG, executable="mavros_bridge_node",
             name="mavros_bridge", parameters=bridge_params, output="screen"),
        Node(package=_PKG, executable="fsm_node",
             name="fsm_node", parameters=fsm_params, output="screen"),
        Node(package=_PKG, executable="telemetry_node",
             name="telemetry_node", parameters=telemetry_params,
             output="screen"),
        # Dosya-3 (md 4.2): /girdap/map/local → zaman damgalı mp4 + PNG yedeği.
        Node(package=_PKG, executable="local_map_node",
             name="local_map_node", **teslim_common),
        # Dosya-1'in "Diğer Otonomi Sensörleri" ayağı (md 487-493): LiDAR
        # kümeleme videosu. 🔴 Bu teslim 07.08.2026'ya kadar HİÇ üretilmiyordu
        # (eksik dosya = 5 ceza, md 5.5.4.3.5). Kamera mp4'ünden AYRI dosya
        # olmak zorunda: "her bir sensör tipi için ayrı ayrı".
        Node(package=_PKG, executable="lidar_kayit_node",
             name="lidar_kayit_node", **teslim_common),
        # Video: 4-nokta waypoint görevi → /girdap/mission/current_target.
        Node(package=_PKG, executable="mission_manager_node",
             name="mission_manager_node", parameters=mission_params,
             output="screen"),
    ]

    # --- Sensör sürücüleri (ida_topics paketi — with_drivers:=true, F-S.2) ---
    # Topic hizalaması remap ile: sürücüler kendi isimlerinde yayınlar, girdap
    # perception /livox/lidar + /oak/rgb/image_raw bekler.
    _drv = IfCondition(LaunchConfiguration("with_drivers"))
    # OAK'a dokunan sürücüler AYRI bayrakta (yukarıdaki gerekçe): LiDAR açılırken
    # kamera ölmesin. AND mantığı: with_drivers VE with_oak_driver.
    _drv_oak = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("with_drivers"), "' == 'true' and '",
            LaunchConfiguration("with_oak_driver"), "' == 'true'",
        ])
    )
    driver_nodes = [
        # Livox Mid-360 — saf Python UDP (SDK'sız). IP/port gerçek cihazda
        # doğrulandı: 192.168.117.100, data 56301.
        Node(package="ida_topics", executable="livox_driver_node",
             name="livox_driver_node", condition=_drv, output="screen",
             remappings=[("/lidar/points", "/livox/lidar"),
                         ("/lidar/scan", "/livox/scan")]),
        # OAK-D Lite — depthai pip paketi gerekir (yalnız Jetson'da kurulu).
        # ⚠ use_onboard_camera:=true (F3.1, algı ekibinin OAK node'u) ile
        # AYNI ANDA açma — iki süreç aynı USB cihazını açamaz.
        Node(package="ida_topics", executable="oakd_driver_node",
             name="oakd_driver_node", condition=_drv_oak, output="screen",
             remappings=[("/camera/image_raw", "/oak/rgb/image_raw")]),
        # Dosya-1 (Şartname 4.2): işlenmiş kamera mp4'ü (bbox overlay + zaman
        # etiketi) → ~/girdap_logs/kamera.
        # ⚠ F-S.3 (bilinen kısıt): kamera_kayit_node ida_topics'in kendi
        # perception_node'unu (ayrı /perception/orange_buoys + /yellow_buoys
        # topic'leri) varsayar; girdap_decision'ın perception_camera_node'u
        # ise TEK topic'te (/perception/buoys) class_id (0=turuncu/1=sarı/
        # 2=hedef) taşır. Bu remap yalnız /perception/buoys'u orange_buoys'a
        # bağlar — Dosya-1 mp4 üretilir (≥1Hz, bbox overlay) ama sarı/hedef
        # sınıflar da "TURUNCU DUBA" etiketiyle çizilir (kozmetik, hata_defteri
        # F-S.3). Düzgün çözüm: kamera_kayit_node'u class_id okur hale getirmek
        # (T1 — video için engelleyici değil, Dosya-1 formatı yine sağlanıyor).
        Node(package="ida_topics", executable="kamera_kayit_node",
             name="kamera_kayit_node", condition=_drv_oak, output="screen",
             remappings=[("/camera/image_raw", "/oak/rgb/image_raw"),
                         ("/perception/orange_buoys", "/perception/buoys")]),
    ]

    return LaunchDescription(
        [
            *declared,
            # F3.2: use_mppi LogInfo'dan çıkarıldı — hiçbir node okumadığı
            # halde basmak operatöre "kapatılabilir" yanılgısı veriyordu.
            LogInfo(msg=[
                "[hardware] ArduRover — fcu_url=", fcu_url,
                " | algorithm: isam2=", use_isam2, " rrt=", use_rrt,
                " | onboard_camera=", LaunchConfiguration("use_onboard_camera"),
                " | with_mavros=", LaunchConfiguration("with_mavros"),
                " (false=masa testi, mock_sensors besler)",
            ]),
            # md 4.2: saat güveni ÜÇ teslimi birden etkiliyor → operatör bunu
            # kaçırmasın. Güvenilmezse damga iddiası düşer (veri kaybolmaz).
            LogInfo(msg=(
                f"[hardware] SAAT: {'GUVENILIR' if _saat_ok else '!! GUVENILMEZ !!'}"
                f" — {_saat_neden}"
                + ("" if _saat_ok else
                   " | Dosya-1/2/3 damgalari 'guvenilmez' isaretlenecek."
                   " Duzeltme: sudo systemctl start girdap-saat"
                   " (GPS'ten kurar; fix sart)")
            )),
            mavros,
            mock_sensors_node,
            *static_tfs,
            # =============================================================== #
            # SENSOR DRIVERS: added by hardware teammate
            #   Livox Mid-360 sürücüsü (livox_ros_driver2) → /livox/lidar
            #   ⚠ OAK-D Lite: depthai_ros SÜRÜCÜSÜ EKLEME! (F3.1) Kamerayı
            #   algı ekibinin node'u DOĞRUDAN DepthAI ile açar (VPU'da YOLO);
            #   ikinci bir süreç USB cihazını açamaz, kamera tamamen ölür.
            #   Bu launch KARAR yazılımıdır; sensör bring-up buraya EKLENMEZ.
            # =============================================================== #
            *driver_nodes,
            *decision_nodes,
        ]
    )

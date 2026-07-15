"""
Girdap İDA — Gerçek donanım launch (ArduRover + Pixhawk 6C).

Sahada/gerçek suda çalışan tam yığın. mock_sensors YOKTUR; sensör verisi
gerçek MAVROS + sensör sürücülerinden gelir.

Bileşenler:
    - MAVROS (mavros/apm.launch include) — Pixhawk MAVLink köprüsü, fcu_url
      config/hardware.yaml'dan.
    - Static TF (kalibre EDİLMEMİŞ, 0,0,0): base_link → livox_frame / oak_frame
      / imu_link. Gerçek ölçümler mekanik ekipten gelince güncellenir.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PKG = "girdap_decision"

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
}
_TELEMETRY_DEFAULTS: dict[str, tuple[object, type]] = {
    "setpoint_source": ("girdap", str),
    "fc_cruise_setpoint_mps": (1.0, float),
    "fc_thrust_left_ch": (1, int),
    "fc_thrust_right_ch": (3, int),
}
# mission bloğu varsayılanı: görev dosyası (video ↔ competition). Sprint 4.
_MISSION_DEFAULT = "video_mission.yaml"
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
    "use_yolo": (False, bool),
    "yolo_model_path": ("", str),
    # F-S.9: turuncu/sarı için alternatif yol (eğitilmiş lokalizatör + HSV).
    "use_yolo_localizer": (False, bool),
    "yolo_localizer_model_path": ("", str),
    "yolo_localizer_min_coverage": (0.15, float),
    "log_period_s": (5.0, float),
}
# perception.fusion varsayılanları — kamera-LiDAR bearing füzyonu (Sprint 3).
_FUSION_DEFAULTS: dict[str, tuple[object, type]] = {
    "bearing_tolerance_rad": (0.15, float),
    "camera_hfov_rad": (1.2, float),
    "camera_image_width_px": (640, int),
    "camera_image_height_px": (480, int),
    "sync_slop_s": (0.1, float),
    "log_period_s": (5.0, float),
}


def _load_hardware_config() -> dict:
    """config/hardware.yaml'ı oku; eksik/bulunamazsa varsayılanlara düş."""
    cfg = dict(_HW_DEFAULTS)
    cfg.update(_ALGO_DEFAULTS)
    cfg["lidar"] = {k: v for k, (v, _) in _LIDAR_DEFAULTS.items()}
    cfg["camera"] = {k: v for k, (v, _) in _CAMERA_DEFAULTS.items()}
    cfg["fusion"] = {k: v for k, (v, _) in _FUSION_DEFAULTS.items()}
    cfg["mission_file"] = _MISSION_DEFAULT
    cfg["mission_source"] = _MISSION_SOURCE_DEFAULT
    cfg["skip_home_seq0"] = _SKIP_HOME_DEFAULT
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
    try:
        path = os.path.join(
            get_package_share_directory(_PKG), "config", "hardware.yaml"
        )
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        for key in _HW_DEFAULTS:
            if key in data:
                cfg[key] = data[key]
        # algorithm: bloğu (video ↔ yarışma modu seçimi)
        algo = data.get("algorithm") or {}
        for key in _ALGO_DEFAULTS:
            if key in algo:
                cfg[key] = bool(algo[key])
        # planning: bloğu varsa mode_name'ini kullan, yoksa kökü miras al.
        cfg["planning_mode"] = str(
            (data.get("planning") or {}).get("mode_name", cfg["mode_name"])
        )
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


def _static_tf(parent: str, child: str) -> Node:
    """Kalibre edilmemiş (0,0,0) static transform yayıncısı."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{child}",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
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
            description="F3.1: true → HSV yedek kamera node'u açılır. "
                        "VARSAYILAN false: /perception/buoys'u algı ekibinin "
                        "OAK node'u üretir (DepthAI doğrudan, VPU'da YOLO)",
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
            description="Sensör sürücülerini başlat: Livox + OAK-D + kamera kaydı",
        ),
        # with_mavros — masa testi (Pixhawk yok/bağlı değil). false: gerçek
        # mavros yerine mevcut mock_sensors node'u (/mavros/imu/data,
        # /mavros/global_position/global, /mavros/state) karar yığınını besler
        # — fsm/mission_manager/planning/telemetry uçtan uca canlı test edilir.
        DeclareLaunchArgument(
            "with_mavros", default_value="true",
            description="false: gerçek MAVROS yerine mock_sensors (masa testi)",
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
    ]

    # --- MAVROS: ArduRover apm.launch (XML) include ---
    mavros_apm = os.path.join(
        get_package_share_directory("mavros"), "launch", "apm.launch"
    )
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_apm),
        launch_arguments={"fcu_url": fcu_url, "gcs_url": gcs_url}.items(),
        condition=IfCondition(LaunchConfiguration("with_mavros")),
    )
    # Masa testi (with_mavros:=false): gerçek Pixhawk/MAVROS yok, mevcut
    # mock_sensors node'u aynı topic'leri (imu/gps/state) sentetik veriyle
    # besler — karar yığını (fsm/mission_manager/planning/telemetry/bridge)
    # donanımsız uçtan uca çalıştırılabilir.
    mock_sensors_node = Node(
        package=_PKG, executable="mock_sensors", name="mock_sensors",
        condition=UnlessCondition(LaunchConfiguration("with_mavros")),
        output="screen",
    )

    # --- Static TF (kalibre edilmemiş; mekanik ekip gerçek ölçümle günceller) ---
    static_tfs = [
        _static_tf("base_link", "livox_frame"),
        _static_tf("base_link", "oak_frame"),
        _static_tf("base_link", "imu_link"),
    ]

    # --- Karar yığını node'ları ---
    common = {
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
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
        },
    ]
    # fusion: algorithm.use_isam2 (video → MAVROS EKF pass-through).
    fusion_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "use_isam2": ParameterValue(use_isam2, value_type=bool),
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
             parameters=_perception_params("lidar", _LIDAR_DEFAULTS),
             output="screen"),
        # Sprint 2 — F3.1: VARSAYILAN KAPALI. /perception/buoys'un asıl
        # üreticisi algı ekibinin OAK node'u (girdap-ida-algi, DepthAI
        # doğrudan — VPU'da YOLO). Bu HSV node'u yalnız YEDEK; ikisi aynı
        # anda açılırsa hem topic çakışır hem OAK USB cihazı iki süreçte
        # açılamaz. Açmak için: use_onboard_camera:=true.
        Node(package=_PKG, executable="perception_camera_node",
             name="perception_camera_node",
             parameters=_perception_params("camera", _CAMERA_DEFAULTS),
             condition=IfCondition(LaunchConfiguration("use_onboard_camera")),
             output="screen"),
        # Sprint 3: obstacle_map + buoys (sync) → /perception/classified_obstacles.
        # LiDAR+kamera node'larından SONRA gelmeli (mesajları tüketiyor).
        Node(package=_PKG, executable="perception_fusion_node",
             name="perception_fusion_node",
             parameters=_perception_params("fusion", _FUSION_DEFAULTS),
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
        # Dosya-3: /girdap/map/local → grayscale PNG serisi (~/girdap_logs).
        Node(package=_PKG, executable="local_map_node",
             name="local_map_node", **common),
        # Video: 4-nokta waypoint görevi → /girdap/mission/current_target.
        Node(package=_PKG, executable="mission_manager_node",
             name="mission_manager_node", parameters=mission_params,
             output="screen"),
    ]

    # --- Sensör sürücüleri (ida_topics paketi — with_drivers:=true, F-S.2) ---
    # Topic hizalaması remap ile: sürücüler kendi isimlerinde yayınlar, girdap
    # perception /livox/lidar + /oak/rgb/image_raw bekler.
    _drv = IfCondition(LaunchConfiguration("with_drivers"))
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
             name="oakd_driver_node", condition=_drv, output="screen",
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
             name="kamera_kayit_node", condition=_drv, output="screen",
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
            *decision_nodes,
        ]
    )

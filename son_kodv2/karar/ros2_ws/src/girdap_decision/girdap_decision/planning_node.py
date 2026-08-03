"""
Girdap İDA — Planlama node'u (Layer 2): RRT* global + MPPI lokal.

Akış:
    fusion_node (smooth pose) ─┐
    perception (engel haritası) ┼─→ planning_node ─→ thrust komutu
    mission waypoints ─────────┤                  └─→ /mavros/setpoint_velocity
    fsm (durum) ───────────────┘

Subscribed topics:
    /girdap/fusion/odom              nav_msgs/Odometry      (smooth pose+vel)
    /girdap/mission/state            std_msgs/String        (FSM durumu)
    /perception/obstacle_map         geometry_msgs/PoseArray
        Engel merkezleri; her poz position.{x,y} = merkez, orientation.z =
        yarıçap (PLACEHOLDER şema — perception ekibi topic'i teslim edince
        güncellenecek; OccupancyGrid gelirse costmap→circle çıkarımı eklenir).
        ⚠ frame `base_link` (GÖVDE, x=ileri) — burada dünya ENU'ya çevrilir.
    /perception/classified_obstacles vision_msgs/Detection3DArray
        F-S.9 füzyon çıktısı: aynı engeller + RENK SINIFI. Aktığı anda
        obstacle_map'in yerine geçer (sınıf bilgisi kesinlikle daha iyidir).
        class_id=0 (turuncu KENAR dubası) engel torbasından ÇIKARILIR ve
        kapı takibine (gate_follower) beslenir — arasından GEÇİLECEK nesne
        engel sayılırsa MPPI kapıya girmeyi pahalı bulur (bkz. CLAUDE.md
        "Emniyet Payları": margin 1.0 m, geçit net açıklığı ~1.35 m).
    /girdap/mission/waypoints        nav_msgs/Path          (F-S.6/F-S.11:
        base_link-göreli ENU, TEK aktif waypoint; current_target ile aynı
        referans; burada son bilinen odom xy'sine eklenip mutlak "map"
        konumuna çevrilir)

Published topics:
    /mavros/setpoint_velocity/cmd_vel_unstamped  geometry_msgs/Twist
        Cascade PID dış döngü çıktısı (CLAUDE.md MAVROS bölümü).
    /girdap/planning/global_path     nav_msgs/Path
        RRT* çıkışı; RViz görselleştirmesi ve replan izleme için.
    /girdap/control/thrust           std_msgs/Float32MultiArray
        Diferansiyel thruster komutu [T_left, T_right] (N) — Layer 1 ESC kanalı.
    /girdap/planning/gate            geometry_msgs/PoseStamped
        Kilitlenilen kapının orta noktası (frame "map"). Kapı yokken
        yayınlanmaz — RViz'de "şu an kapı görüyor muyuz" tek bakışta belli
        olsun diye (saha teşhisi; hiçbir kontrol yolu bunu okumaz).

⚠ FRAME SÖZLEŞMESİ (2026-08-03'te netleştirildi — bkz. `_body_to_world`):
    - mission topic'leri (`current_target`, `waypoints`) ENU-hizalı ÖTELEME
      ofsetidir (`latlon_to_enu` doğu/kuzey verir) → yalnız odom xy eklenir.
    - perception topic'leri (`obstacle_map`, `classified_obstacles`) GÖVDE
      çerçevesindedir (x=ileri) → hem ψ ile DÖNDÜRÜLÜR hem ötelenir.
    İkisi aynı sanılırsa engeller yanlış yere düşer (aşağıdaki nota bak).

Notlar:
    - Tüm planlama mantığı prototype.planning.pipeline.PlanningPipeline'da;
      bu node yalnızca ROS 2 mesaj alanlarını okuyup boru hattına yönlendirir.
      Uçtan uca test (test_planning_pipeline.py) aynı sınıfı kullanır.
    - Kontrol döngüsü 20 Hz (MPPI dt=0.05 ile hizalı). CLAUDE.md 50 Hz hedefi
      Jetson CUDA sürümüne aittir; CPU Layer 2'de 20 Hz doğrulama içindir.
    - Parkur bazlı davranış PlanningPipeline'da: FSM durumu değişince MPPI
      ağırlık profili (w_track/w_obstacle/kamikaze) otomatik değişir.
    - FSM durumu PARKUR1/2/3 değilse thrust 0.0 yayınlanır (FSM otoritesi).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from mavros_msgs.msg import State as MavState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, String
from vision_msgs.msg import Detection3DArray

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig
from prototype.mission.gate_follower import GateFollower, GateFollowerConfig
from prototype.planning.mppi import MPPIConfig
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


class PlanningNode(Node):
    """RRT* global + MPPI lokal planlayıcı sarmalayıcısı."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu;
        # diğer node'larla aynı desen).
        super().__init__("planning_node", **node_kwargs)

        # --- Parametreler ---
        # F-P.13 (robustness taraması, 2026-07-15): varsayılan buradaydı
        # 20.0 iken params.yaml AÇIKÇA 10.0'a düşürüyordu ("20 Hz senkron
        # step'i tutamaz, executor birikir, cmd_vel gecikir/titrer →
        # istemsiz hareket" — md 3.3.1.1 tehlikesi). params.yaml
        # uygulanmadan (elle `ros2 run`, yanlış params yolu) standalone
        # koşulursa node SESSİZCE güvensiz 20 Hz'e düşerdi — kod
        # varsayılanı da güvenli değerle hizalandı.
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("bounds_x", [0.0, 200.0])
        self.declare_parameter("bounds_y", [0.0, 200.0])
        self.declare_parameter("replan_proximity", 2.0)     # m
        self.declare_parameter("mppi_K", 1000)
        self.declare_parameter("mppi_T", 50)
        self.declare_parameter("heartbeat_timeout_s", 5.0)  # MAVROS geçidi
        # F-P.1: son odom bu süreden eskiyse MPPI KOŞULMAZ (thrust sıfır).
        # fusion_node F8.2 bekçisi poz kaynağı susunca odom yayınını keser
        # ("bayat pozla plan yapılmasın") — ama planning son durumu saklayıp
        # 10 Hz sürmeye devam ediyordu → GPS/EKF kesilse bile araç KÖR sürer.
        # Eşik fusion'ın pose_timeout_s'iyle aynı mantıkta (1 s); 0 → kapalı.
        self.declare_parameter("odom_timeout_s", 1.0)
        # F-P.2 (robustness taraması): obstacle_map için de F-P.1 ile aynı
        # bekçi — perception_lidar_node kaynağı (Livox sürücüsü/USB) donarsa
        # son bilinen engel listesi SONSUZA DEK kullanılmasın (var olmayan
        # bir engelden kaçınmaya devam edebilir ya da gerçek bir engelin
        # gittiğini sanıp üstüne sürebilir). Topic her taramada (engel olsun
        # olmasın) publish edildiği için tazelik kontrolü güvenli. 0 → kapalı.
        self.declare_parameter("obstacle_timeout_s", 2.0)
        self.declare_parameter("mode_name", "GUIDED")        # otonomi modu
        self.declare_parameter("map_rate_hz", 10.0)          # Dosya-3 yayım hızı
        self.declare_parameter("use_rrt", True)              # false → video bypass
        # F-S.10: yerel kontrolcü seçimi — "mppi" (varsayılan) | "pid"
        # (ida_topics'in donanımda kanıtlanmış cascade PID'i + LiDAR
        # potansiyel-alan kaçınması; MPPI saha kalibrasyonu tamamlanana
        # kadar düşme-güvenli yedek — bkz. PlanningPipelineConfig.control_mode).
        self.declare_parameter("control_mode", "mppi")

        # --- MPPI saha tuning parametreleri (2026-08-02) ---
        # Varsayılanlar MPPIConfig'ten OKUNUR (kopyalanmaz) → kod ile ROS
        # varsayılanı arasında drift imkânsız. Verilmezse davranış birebir aynı.
        # mppi_lambda İSTİSNA: 0.0 = "parkur profili kazansın" nöbetçi değeri
        # (λ fizik olarak > 0; profiller PARKUR1/2=10, PARKUR3=50).
        _mppi = MPPIConfig()
        self.declare_parameter("mppi_lambda", 0.0)
        self.declare_parameter("mppi_sigma_u", _mppi.sigma_u)
        self.declare_parameter("mppi_obstacle_margin", _mppi.obstacle_margin)
        self.declare_parameter("mppi_terminal_mode", _mppi.terminal_mode)
        self.declare_parameter(
            "mppi_terminal_lookahead_m", _mppi.terminal_lookahead_m
        )
        self.declare_parameter("mppi_ref_window_size", _mppi.ref_window_size)
        self.declare_parameter("mppi_ref_window_enabled", _mppi.ref_window_enabled)

        # --- Kapı takibi (gate following, 2026-08-03) ---
        # Şartname md 5.5.2.2: Parkur-1/2 puanı GPS noktasına basmaktan DEĞİL,
        # karşılıklı KENAR dubası ikilisinin ARASINDAN geçmekten gelir; hakemin
        # verdiği nokta "doğrudan iki kenar dubasının arasında OLMAYABİLİR".
        # Çekirdek `prototype/mission/gate_follower.py` (20 test, 27.07'de
        # yazıldı) buraya kadar BAĞLI DEĞİLDİ — ham GN doğrudan MPPI'ye
        # gidiyordu. Bu blok o bağlantıdır.
        # Varsayılanlar GateFollowerConfig'ten OKUNUR (kopyalanmaz) → drift yok.
        _gate = GateFollowerConfig()
        self.declare_parameter("gate_following_enabled", True)
        # Turuncu kenar dubasının sınıf kimliği (camera_buoys.CLASS_PARKUR_KENARI).
        # Parametre olarak duruyor ki sınıf şeması değişirse kod değişmesin
        # (ve planning_node cv2 bağımlısı camera_buoys'u import etmek zorunda
        # kalmasın — bu node algı kütüphanesi çekmemeli).
        self.declare_parameter("edge_buoy_class_id", 0)
        # classified_obstacles aktığında obstacle_map'in yerine geçsin mi?
        # false → eski davranış (sınıfsız harita), kapı dubaları da engel kalır.
        self.declare_parameter("use_classified_obstacles", True)
        self.declare_parameter("gate_width_min", _gate.gate_width_min)
        self.declare_parameter("gate_width_max", _gate.gate_width_max)
        self.declare_parameter("gate_max_lookahead", _gate.max_lookahead)
        self.declare_parameter("gate_min_forward", _gate.min_forward)
        self.declare_parameter("gate_pair_depth_tol", _gate.pair_depth_tol)
        self.declare_parameter("gate_release_distance", _gate.release_distance)
        self.declare_parameter("gate_match_radius", _gate.match_radius)

        bx = self.get_parameter("bounds_x").value
        by = self.get_parameter("bounds_y").value
        bounds = Bounds(bx[0], bx[1], by[0], by[1])

        control_mode = str(self.get_parameter("control_mode").value).lower()
        if control_mode not in ("mppi", "pid"):
            self.get_logger().warn(
                f"control_mode='{control_mode}' geçersiz → 'mppi' varsayılanına "
                "düşüldü (geçerli değerler: mppi, pid)"
            )
            control_mode = "mppi"

        terminal_mode = str(self.get_parameter("mppi_terminal_mode").value)
        if terminal_mode not in ("lookahead", "global"):
            # F10.1 dersi: planlama yolunda istisna node'u öldürür → WARN + düş.
            self.get_logger().warn(
                f"mppi_terminal_mode='{terminal_mode}' geçersiz → "
                f"'{_mppi.terminal_mode}' varsayılanına düşüldü "
                "(geçerli: lookahead, global)"
            )
            terminal_mode = _mppi.terminal_mode

        # λ: 0 (ya da negatif) = nöbetçi → parkur profili kazansın (None geç).
        lam = float(self.get_parameter("mppi_lambda").value)
        mppi_lambda = lam if lam > 0.0 else None

        cfg = PlanningPipelineConfig(
            replan_proximity=float(self.get_parameter("replan_proximity").value),
            mppi_K=int(self.get_parameter("mppi_K").value),
            mppi_T=int(self.get_parameter("mppi_T").value),
            control_mode=control_mode,
            mppi_lambda=mppi_lambda,
            mppi_sigma_u=float(self.get_parameter("mppi_sigma_u").value),
            mppi_obstacle_margin=float(
                self.get_parameter("mppi_obstacle_margin").value
            ),
            mppi_terminal_mode=terminal_mode,
            mppi_terminal_lookahead_m=float(
                self.get_parameter("mppi_terminal_lookahead_m").value
            ),
            mppi_ref_window_size=int(
                self.get_parameter("mppi_ref_window_size").value
            ),
            mppi_ref_window_enabled=bool(
                self.get_parameter("mppi_ref_window_enabled").value
            ),
        )
        self._pipe = PlanningPipeline(bounds, cfg)

        # Video bypass: use_rrt=false → global plan atlanır, current_target
        # doğrudan MPPI referansı. Son poz absolute hedef hesabı için tutulur.
        self._use_rrt = bool(self.get_parameter("use_rrt").value)
        self._last_xy: Optional[tuple] = None
        # Gövde→dünya dönüşümü için heading de gerekir (yalnız xy YETMEZ).
        self._last_psi: float = 0.0

        # --- Kapı takibi durumu ---
        self._gate_enabled = bool(self.get_parameter("gate_following_enabled").value)
        self._edge_class_id = int(self.get_parameter("edge_buoy_class_id").value)
        self._use_classified = bool(
            self.get_parameter("use_classified_obstacles").value
        )
        self._gate = GateFollower(
            GateFollowerConfig(
                gate_width_min=float(self.get_parameter("gate_width_min").value),
                gate_width_max=float(self.get_parameter("gate_width_max").value),
                max_lookahead=float(self.get_parameter("gate_max_lookahead").value),
                min_forward=float(self.get_parameter("gate_min_forward").value),
                pair_depth_tol=float(
                    self.get_parameter("gate_pair_depth_tol").value
                ),
                release_distance=float(
                    self.get_parameter("gate_release_distance").value
                ),
                match_radius=float(self.get_parameter("gate_match_radius").value),
            )
        )
        # Kenar dubaları DÜNYA ENU'da (classified_obstacles'tan her taramada
        # tazelenir). Boş liste = kapı görünmüyor → gate_follower ham GN'ye düşer.
        self._edge_buoys: list[tuple[float, float]] = []
        # classified_obstacles hiç aktı mı? (obstacle_map ile hakemlik için)
        self._classified_seen = False
        self._gate_log_t = 0.0
        self._last_gate_used_fallback = True

        # MAVROS mod/arm geçidi — mavros_bridge ile aynı karar çekirdeği (DRY).
        # Hedef mod (mode_name) değilse cmd_vel yayınlanmaz; armed değilse thrust
        # sıfırlanır. mode_name mavros_bridge ile AYNI olmalı (tek kaynak: köke
        # bakan hardware.yaml → hardware.launch her iki node'a aktarır).
        self._bridge = MavrosBridge(
            MavrosBridgeConfig(
                heartbeat_timeout_s=float(
                    self.get_parameter("heartbeat_timeout_s").value
                ),
                target_mode=str(self.get_parameter("mode_name").value),
            )
        )

        # F-P.1: odom bayatlık takibi
        self._odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        self._last_odom_t: float | None = None
        self._stale_warn_t = 0.0
        # F-P.2: obstacle_map bayatlık takibi
        self._obstacle_timeout = float(
            self.get_parameter("obstacle_timeout_s").value
        )
        self._last_obstacle_t: float | None = None
        self._obstacle_stale_warn_t = 0.0

        # --- Subscribers ---
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )
        self._sub_mav_state = self.create_subscription(
            MavState, "/mavros/state", self._on_mav_state, 10
        )
        self._sub_obs = self.create_subscription(
            PoseArray, "/perception/obstacle_map", self._on_obstacles, 10
        )
        # F-S.9 füzyon çıktısı — sınıflı engeller. Aktığı anda obstacle_map'in
        # yerine geçer; turuncu kenar dubaları buradan kapı takibine gider.
        self._sub_classified = self.create_subscription(
            Detection3DArray,
            "/perception/classified_obstacles",
            self._on_classified,
            10,
        )
        self._sub_wp = self.create_subscription(
            Path, "/girdap/mission/waypoints", self._on_waypoints, 10
        )
        # Video bypass (use_rrt=false): mission_manager'dan doğrudan hedef.
        self._sub_target = self.create_subscription(
            PoseStamped, "/girdap/mission/current_target", self._on_target, 10
        )

        # --- Publishers ---
        self._pub_cmd_vel = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )
        self._pub_path = self.create_publisher(
            Path, "/girdap/planning/global_path", 10
        )
        self._pub_thrust = self.create_publisher(
            Float32MultiArray, "/girdap/control/thrust", 10
        )
        # Dosya-3: yerel maliyet haritası (RViz + local_map_node PNG dumper).
        self._pub_map = self.create_publisher(
            OccupancyGrid, "/girdap/map/local", sensor_data_qos()
        )
        # Saha teşhisi: kilitlenilen kapının orta noktası (kontrol yolu DEĞİL).
        self._pub_gate = self.create_publisher(
            PoseStamped, "/girdap/planning/gate", 10
        )

        # --- Kontrol döngüsü ---
        rate = float(self.get_parameter("control_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_control_step)

        # --- Yerel harita yayım döngüsü (Dosya-3, ~10 Hz) ---
        map_rate = float(self.get_parameter("map_rate_hz").value)
        self._map_timer = self.create_timer(1.0 / map_rate, self._publish_local_map)

        planner = "RRT*+MPPI" if self._use_rrt else "düz hedef+MPPI (video)"
        self.get_logger().info(
            f"planning_node aktif [{planner}] (MPPI K={cfg.mppi_K}, "
            f"T={cfg.mppi_T}, control={rate} Hz, map={map_rate} Hz)"
        )
        # Saha tuning değerleri log'a — hangi ayarla uçtuğumuz kayıt altında.
        self.get_logger().info(
            f"MPPI tuning: λ={'profil' if mppi_lambda is None else mppi_lambda}, "
            f"σ_u={cfg.mppi_sigma_u}, engel_payı={cfg.mppi_obstacle_margin} m, "
            f"terminal={cfg.mppi_terminal_mode}"
            f"({cfg.mppi_terminal_lookahead_m} m), "
            f"ref_pencere={cfg.mppi_ref_window_size}"
            f"{'' if cfg.mppi_ref_window_enabled else ' (KAPALI)'}"
        )
        if self._gate_enabled:
            self.get_logger().info(
                "kapı takibi AÇIK: kenar dubası sınıfı="
                f"{self._edge_class_id}, genişlik "
                f"[{self.get_parameter('gate_width_min').value}, "
                f"{self.get_parameter('gate_width_max').value}] m, "
                f"menzil {self.get_parameter('gate_max_lookahead').value} m "
                "— turuncu dubalar engel torbasından ÇIKARILIR"
            )
        else:
            self.get_logger().warn(
                "kapı takibi KAPALI (gate_following_enabled=false) → ham görev "
                "noktasına gidilir; md 5.5.2.2 puanı kapıdan geçmekten gelir"
            )

    # ----- subscriber callback'leri -----

    def _on_odom(self, msg: Odometry) -> None:
        """ENU pose + velocity → durum vektörü [x, y, ψ, u, v, r]."""
        self._last_odom_t = self._now()          # F-P.1: bayatlık saati
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        psi = 2.0 * math.atan2(q.z, q.w)             # z-eksen quaternion → yaw
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self._pipe.set_state(np.array([p.x, p.y, psi, v.x, v.y, w.z]))
        self._last_xy = (p.x, p.y)               # bypass absolute hedef için
        self._last_psi = psi                     # gövde→dünya dönüşümü için

    # ----- frame dönüşümü -----

    def _body_to_world(self, bx: float, by: float) -> tuple[float, float]:
        """base_link (x=ileri) → dünya ENU. Döndürme + öteleme.

        🔴 2026-08-03 BULGUSU — bu dönüşüm EKSİKTİ. `/perception/obstacle_map`
        `perception_lidar_node`'da açıkça `frame_id="base_link"` ile yayınlanıyor
        (gövde çerçevesi, x=ileri), ama `_on_obstacles` koordinatları OLDUĞU GİBİ
        `PlanningPipeline`'a veriyordu — oysa boru hattı DÜNYA çerçevesinde
        çalışır (`set_state` odom mutlak pozunu alır, RRT* start=mutlak poz,
        MPPI maliyeti rollout dünya konumlarını engel koordinatlarıyla
        karşılaştırır). Sonuç: araç origin'de ve ψ=0 iken tesadüfen doğru,
        BAŞKA HER DURUMDA engeller hem döndürülmemiş hem ötelenmemiş yanlış
        yere düşüyordu → var olmayan engelden kaçınma + gerçek engele sürme.
        Mission topic'lerinde (`current_target`, `waypoints`) bu hata YOK: onlar
        `latlon_to_enu` ile ENU-hizalı öteleme ofseti (doğu/kuzey) taşır, sadece
        xy eklemek doğrudur — iki sözleşme aynı sanıldığı için gözden kaçmış.
        """
        if self._last_xy is None:
            # Poz yok: döndüremeyiz. Gövde koordinatını dünya sanmak
            # (eski davranış) sessiz bir hata olurdu — çağıran atlar.
            raise ValueError("odom yok, gövde→dünya dönüşümü yapılamaz")
        c, s = math.cos(self._last_psi), math.sin(self._last_psi)
        return (
            self._last_xy[0] + bx * c - by * s,
            self._last_xy[1] + bx * s + by * c,
        )

    def _on_obstacles(self, msg: PoseArray) -> None:
        """PLACEHOLDER şema: position.{x,y} merkez, orientation.z yarıçap.

        Sınıflı topic (`classified_obstacles`) bir kez aktıysa BU YOL SUSAR —
        aynı engeller oradan sınıf bilgisiyle birlikte geliyordur ve kapı
        dubalarının ayıklanması yalnız orada mümkündür (F-S.9).
        """
        if self._use_classified and self._classified_seen:
            return
        # F-P.2 bayatlık saati poz kontrolünden ÖNCE: bu bekçi "perception
        # kaynağı sustu mu"yu ölçer, "biz dönüştürebildik mi"yi değil. Poz
        # yokluğunda saati durdurmak bekçiyi sessizce kör ederdi.
        self._last_obstacle_t = self._now()
        if self._last_xy is None:                 # poz yok → dönüştürülemez
            return
        obstacles = [
            CircleObstacle(*self._body_to_world(pp.position.x, pp.position.y),
                           abs(pp.orientation.z))
            for pp in msg.poses
        ]
        self._pipe.set_obstacles(obstacles)

    def _on_classified(self, msg: Detection3DArray) -> None:
        """Sınıflı engeller → kapı dubaları AYRIŞTIRILIR, gerisi engel kalır.

        Turuncu KENAR dubası (class_id=`edge_buoy_class_id`, varsayılan 0)
        arasından GEÇİLECEK bir nesnedir; engel torbasında bırakılırsa MPPI'nin
        `obstacle_margin`=1.0 m'lik ceza halkası geçidin içini kaplar ve araç
        kapıdan geçmek yerine etrafından dolanmayı ucuz bulur (CLAUDE.md
        "Emniyet Payları" ölçümü: 1.5 m'de geçitten HİÇ geçmiyor). Bu yüzden
        kenar dubaları engelden çıkarılıp `gate_follower`'a beslenir.

        Eşleşmeyen LiDAR tespiti (CLASS_UNKNOWN=99) engel olarak KALIR —
        füzyon sözleşmesinin güvenlik kuralı (bilinmeyeni atma).
        """
        self._classified_seen = True
        self._last_obstacle_t = self._now()          # F-P.2 bekçisini besle (poz
                                                     # kontrolünden ÖNCE — üstteki
                                                     # _on_obstacles notuna bak)
        if self._last_xy is None:                 # poz yok → dönüştürülemez
            return

        obstacles: list[CircleObstacle] = []
        edges: list[tuple[float, float]] = []
        for det in msg.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None            # sayısal olmayan sınıf → engel say
            c = det.bbox.center.position
            try:
                wx, wy = self._body_to_world(c.x, c.y)
            except ValueError:
                return
            if self._gate_enabled and cls == self._edge_class_id:
                edges.append((wx, wy))
                continue                  # kapı dubası ENGEL DEĞİL
            # bbox.size.x = çap (perception_fusion_node sözleşmesi)
            obstacles.append(CircleObstacle(wx, wy, abs(det.bbox.size.x) / 2.0))

        self._edge_buoys = edges
        self._pipe.set_obstacles(obstacles)

    def _obstacles_stale(self) -> bool:
        """F-P.2: son obstacle_map `obstacle_timeout_s`'ten eski mi?

        obstacle_map HİÇ gelmediyse False — perception henüz açılmamış
        olabilir (boot), yanlış alarm basmanın anlamı yok. 0 → bekçi kapalı.
        """
        if self._obstacle_timeout <= 0.0 or self._last_obstacle_t is None:
            return False
        return (self._now() - self._last_obstacle_t) > self._obstacle_timeout

    def _warn_stale_obstacles(self) -> None:
        """Bayatlık uyarısını saniyede bir bas (10 Hz döngüde spam olmasın)."""
        now = self._now()
        if now - self._obstacle_stale_warn_t < 1.0:
            return
        self._obstacle_stale_warn_t = now
        age = now - (self._last_obstacle_t or now)
        self.get_logger().error(
            f"engel haritası {age:.1f}s'dir gelmiyor → MPPI DURDURULDU, "
            "thrust sıfır (F-P.2: bayat engel bilgisiyle kör sürme yok)"
        )

    def _refine_target(self, coarse: tuple[float, float]) -> tuple[float, float]:
        """Ham görev noktasını (GN) algılanan kapının ORTA NOKTASIYLA değiştir.

        Şartname md 5.5.2.2: Parkur-1/2 puanı iki KENAR dubasının arasından
        geçmekten gelir ve hakemin verdiği nokta tam kapı ortasında OLMAYABİLİR
        → ham GN'ye yönelmek puan kaybettirir. Kapı görünmüyorsa (menzil dışı,
        oklüzyon, Parkur-3) `GateFollower` ham GN'ye düşer; yani bu çağrı
        kapısız durumda DAVRANIŞI DEĞİŞTİRMEZ (geriye tam uyumlu).

        `gate_following_enabled=false` → tamamen devre dışı, eski davranış.
        """
        if not self._gate_enabled or self._last_xy is None:
            return coarse
        result = self._gate.update(self._last_xy, coarse, self._edge_buoys)
        # Kapı bulundu/kaybedildi geçişini bir kez logla (10 Hz'te spam yok).
        if result.used_fallback != self._last_gate_used_fallback:
            self._last_gate_used_fallback = result.used_fallback
            if result.gate is not None:
                self.get_logger().info(
                    f"kapı KİLİTLENDİ: orta ({result.target[0]:.1f}, "
                    f"{result.target[1]:.1f}), genişlik {result.gate.width:.1f} m "
                    f"— ham GN yerine buraya gidiliyor"
                )
            else:
                self.get_logger().info(
                    "kapı görüş dışı → ham görev noktasına dönüldü (fallback)"
                )
        if result.gate is not None:
            self._publish_gate(result.target)
        return result.target

    def _on_waypoints(self, msg: Path) -> None:
        """F-S.6/F-S.11: mission_manager_node current_target'la AYNI referansta
        (base_link-göreli ENU ÖTELEMESİ) TEK aktif waypoint yayınlar — burada
        son bilinen odom xy'sine eklenerek mutlak "map" konumuna çevrilir
        (_on_target ile aynı desen), sonra kapı ortasıyla rafine edilir.
        """
        if not self._use_rrt:                    # video bypass → RRT* girişi yok
            return
        if self._last_xy is None:                 # henüz odom gelmedi
            return
        waypoints = [
            (self._last_xy[0] + ps.pose.position.x,
             self._last_xy[1] + ps.pose.position.y)
            for ps in msg.poses
        ]
        if waypoints:
            # RRT* hedefi listenin SON elemanı (F-S.11) → rafine edilecek olan o.
            waypoints[-1] = self._refine_target(waypoints[-1])
            self._pipe.set_waypoints(waypoints)
            path = self._pipe.global_path
            if path is not None:
                self._publish_path(path)

    def _on_target(self, msg: PoseStamped) -> None:
        """Video bypass: mission_manager hedefi → düz çizgi MPPI referansı.

        current_target base_link'te araç-göreli ENU ofsetidir; absolute hedef
        için son odom pozuna eklenir (RRT* atlanır), sonra kapı ortasıyla
        rafine edilir.
        """
        if self._use_rrt or self._last_xy is None:
            return
        tx = self._last_xy[0] + msg.pose.position.x
        ty = self._last_xy[1] + msg.pose.position.y
        tx, ty = self._refine_target((tx, ty))
        self._pipe.set_reference_direct(tx, ty)
        path = self._pipe.global_path
        if path is not None:
            self._publish_path(path)

    def _on_mission_state(self, msg: String) -> None:
        # Parkur değişince kilitli kapıyı BIRAK: Parkur-1'in son kapısına
        # kilitliyken Parkur-2'ye geçilirse eski kapı hedefi taşınmamalı
        # (gate_follower.reset() sözleşmesi: "parkur geçişi / yeniden başlama").
        if msg.data != self._pipe.mission_state:
            self._gate.reset()
            self._last_gate_used_fallback = True
        self._pipe.set_mission_state(msg.data)

    def _on_mav_state(self, msg: MavState) -> None:
        """MAVROS mod/arm geçidi için FCU durumunu güncelle."""
        self._bridge.update_state(
            self._now(), msg.connected, msg.armed, msg.guided, msg.mode
        )

    # ----- kontrol döngüsü -----

    def _odom_stale(self) -> bool:
        """F-P.1: son odom `odom_timeout_s`'ten eski mi?

        odom HİÇ gelmediyse False — MPPI zaten kontrol üretmez (durum yok),
        boot'ta yanlış alarm basmanın anlamı yok. 0 → bekçi kapalı.
        """
        if self._odom_timeout <= 0.0 or self._last_odom_t is None:
            return False
        return (self._now() - self._last_odom_t) > self._odom_timeout

    def _warn_stale_odom(self) -> None:
        """Bayatlık uyarısını saniyede bir bas (10 Hz döngüde spam olmasın)."""
        now = self._now()
        if now - self._stale_warn_t < 1.0:
            return
        self._stale_warn_t = now
        age = now - (self._last_odom_t or now)
        self.get_logger().error(
            f"poz {age:.1f}s'dir gelmiyor → MPPI DURDURULDU, thrust sıfır "
            "(F-P.1: bayat pozla kör sürme yok)"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_control_step(self) -> None:
        """20 Hz'te MPPI step → thrust komut + Twist setpoint (MAVROS geçitli).

        Geçit kuralları (prototype.control.mavros_bridge):
            - armed=False / heartbeat kaybı → thrust sıfır
            - mode != GUIDED → cmd_vel yayınlanmaz (mavros zaten yok sayar)
        """
        gate = self._bridge.control_gate(self._now())

        u = self._pipe.compute_control()
        if u is None:                                # FSM parkur dışı → motor stop
            u = np.zeros(2)
        if gate.zero_thrust:                         # disarm / KILL → motor stop
            u = np.zeros(2)
        if self._odom_stale():                       # F-P.1: poz bayat → kör sürme
            u = np.zeros(2)
            self._warn_stale_odom()
        if self._obstacles_stale():                   # F-P.2: engel bayat → kör sürme
            u = np.zeros(2)
            self._warn_stale_obstacles()

        self._publish_thrust(u)
        if gate.allow_cmd_vel:                       # yalnız GUIDED + armed
            self._publish_cmd_vel(u)

    # ----- yayım yardımcıları -----

    def _publish_path(self, path) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)

    def _publish_gate(self, midpoint: tuple[float, float]) -> None:
        """Kilitli kapının orta noktası — RViz/saha teşhisi (kontrol yolu DEĞİL)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(midpoint[0])
        msg.pose.position.y = float(midpoint[1])
        msg.pose.orientation.w = 1.0
        self._pub_gate.publish(msg)

    def _publish_thrust(self, u: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = [float(u[0]), float(u[1])]
        self._pub_thrust.publish(msg)

    def _publish_cmd_vel(self, u: np.ndarray) -> None:
        # Diferansiyel thruster → ileri sürat + yaw rate (kaba yaklaşım;
        # gerçek dönüşüm Cascade PID iç döngüsünde yapılır).
        p = self._pipe._dyn.p
        twist = Twist()
        twist.linear.x = float((u[0] + u[1]) / max(1.0, 2.0 * p.mass))
        twist.angular.z = float((u[1] - u[0]) / max(1e-6, p.inertia_z))
        self._pub_cmd_vel.publish(twist)

    def _publish_local_map(self) -> None:
        """Dosya-3: araç merkezli yerel maliyet haritası (OccupancyGrid).

        Frame base_link, origin (-w·res/2, -h·res/2) → araç pencere merkezinde,
        kuzey yukarı. Veri MPPI engel maliyetinden 0-100 normalize; arena dışı -1.
        """
        cg = self._pipe.local_cost_grid()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.info.resolution = float(cg.resolution)
        msg.info.width = int(cg.width)
        msg.info.height = int(cg.height)
        msg.info.origin.position.x = -cg.width * cg.resolution / 2.0
        msg.info.origin.position.y = -cg.height * cg.resolution / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = cg.data.tolist()
        self._pub_map.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

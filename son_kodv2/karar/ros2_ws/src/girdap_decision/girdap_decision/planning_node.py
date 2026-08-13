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
        Kilitlenilen kapının NİŞAN noktası (frame "map") = MPPI'ye verilen
        hedefin ta kendisi. Geometrik ortadan farkı, kirişteki engellere göre
        yapılan kaymadır (gate_follower.aim_point). Kapı yokken yayınlanmaz —
        RViz'de "şu an kapı görüyor muyuz + nereye nişan alıyoruz" tek bakışta
        belli olsun diye (saha teşhisi; hiçbir kontrol yolu bunu OKUMAZ).

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

import functools
import math
from typing import Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from mavros_msgs.msg import State as MavState
from mavros_msgs.msg import StatusText
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, Int32, String

#: KAR-04: `PlanningPipeline._ACTIVE_STATES` ile AYNI olmak zorunda — sebep
#: etiketi boru hattinin gercek kararini yansitmali, tahmin etmemeli.
_AKTIF_DURUMLAR = ("PARKUR1", "PARKUR2", "PARKUR3")
from vision_msgs.msg import Detection3DArray

from girdap_decision.qos_profiles import sensor_data_qos
from girdap_decision.saat_kaynagi import bayatlik_saati
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig
from girdap_decision.yeniden_baslama import ResetAbonesi
from prototype.mission.edge_memory import EdgeBuoyMemory
from prototype.mission.gate_follower import (
    BUOY_RADIUS_M,
    GateFollower,
    GateFollowerConfig,
)
from prototype.planning.mppi import MPPIConfig
from prototype.telemetry.ariza_bildirici import (
    CMDVEL_KESIK,
    ENGEL_BOS,
    GPU_YOK,
    KAPI_YOK,
    KONTROL_HATA,
    RRT_RED,
    SEBEP_TUREVLI_ARIZALAR,
    SETPOINT_BOSLUK,
    SINIF_YOK,
    ArizaBildirici,
    sebepten_kodla,
)
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


def _guard(fn: Callable[..., None]) -> Callable[..., None]:
    """Subscriber/timer callback'ini çökme-güvenli sarar.

    F-P.3 deseni (perception node'larına uygulanmıştı) kontrol node'una da
    taşınır: TEK bir bozuk mesaj ya da beklenmedik hata, en güvenlik-kritik
    node'u KALICI ÖLDÜRMESİN (hiçbir restart supervisor'ı yok — ölen
    planning_node tekneyi son cmd_vel'le kör/komutsuz bırakır). Hata
    throttle'lı loglanır, o çağrı atlanır. Girdi susarsa F-P.1/F-P.2
    watchdog'ları thrust'ı zaten sıfırlar (fail-safe korunur).
    NOT: `_on_control_step` bu decorator'ı KULLANMAZ — orada atlamak değil,
    aktif olarak motorları durdurmak gerekir (bkz. `_safe_stop`).
    """

    @functools.wraps(fn)
    def _wrapped(self: "PlanningNode", *args: object, **kwargs: object) -> None:
        try:
            fn(self, *args, **kwargs)
        except Exception as exc:                    # kasıtlı geniş yakalama
            self.get_logger().error(
                f"{fn.__name__} beklenmedik hata, atlandı: {exc!r}",
                throttle_duration_sec=5.0,
            )

    return _wrapped


class PlanningNode(Node):
    """RRT* global + MPPI lokal planlayıcı sarmalayıcısı."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu;
        # diğer node'larla aynı desen).
        super().__init__("planning_node", **node_kwargs)
        # §0.61: bayatlık/zaman aşımı ölçümleri tek yönlü saatte (duvar saati
        # adımı sahte "poz bayat / setpoint boşluğu" üretiyordu).
        self._saat = bayatlik_saati(self)

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
        # 🔑 Kapı seçiminde AYARLANABİLİR EŞİK YOK (2026-08-03 kararı: "tahmine
        # dayalı hiçbir şey olmasın"). Geriye yalnız ÖLÇÜLMÜŞ tekne boyutları
        # kalıyor; genişlik bandı / menzil / derinlik toleransı / bırakma
        # mesafesi / eşleşme yarıçapı hepsi geometriden türetildi (bkz.
        # GateFollowerConfig). Bu ikisi tekne değişirse güncellenir, sahada
        # "deneyerek" ayarlanmaz.
        self.declare_parameter("hull_width_m", _gate.hull_width_m)
        self.declare_parameter("hull_length_m", _gate.hull_length_m)
        # B2 huni tavanı — payın kendisi ölçülen açıklıktan türer (`_huni_payi`).
        # AYRI bir sayı, mppi_obstacle_margin DEĞİL: küresel payı büyütmek model
        # gelmeyen kolu kırıyor, huni ise sınıf yoksa hiç devreye girmiyor.
        self.declare_parameter("gate_post_margin_m", 1.4)

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
                hull_width_m=float(self.get_parameter("hull_width_m").value),
                hull_length_m=float(self.get_parameter("hull_length_m").value),
            )
        )
        # KENAR DUBASI HAFIZASI — bir kez turuncu sınıflanan duba, rengi
        # kadrajdan çıksa da kenar kalır (§0.17e; edge_memory.py docstring'i).
        # 12 m'lik gerçek kapıda P1'in ÇALIŞMA ŞARTI: hafızasız 1/4 nokta.
        self._edge_memory = EdgeBuoyMemory()
        # madde #11 (md 5.5.3.1): yeniden baslama hakki. PUAN sifirlamasi
        # BURADA yapiliyor cunku gecis sayaci GateFollower'in icinde yasiyor.
        self._reset = ResetAbonesi(self, self._yeniden_basla)
        self._edge_mem_log_t = 0.0
        # Hatırlanan cisimlerin engel torbasına konacağı yarıçap (m) — yeni bir
        # ayar DEĞİL, yerel maliyet haritası penceresinin yarısı (§0.26c).
        self._harita_yaricapi = cfg.map_width * cfg.map_resolution / 2.0
        self._edge_mem_son_acilan = 0        # log penceresi başına yeni kayıt
        self._son_cmd_vel_t: Optional[float] = None   # çıkış kadansı bekçisi
        self._backend_loglandi = False       # MPPI hesap yolu bir kez yazılır
        self._gate_post_margin = float(
            self.get_parameter("gate_post_margin_m").value
        )
        # Kenar dubaları DÜNYA ENU'da (classified_obstacles'tan her taramada
        # tazelenir). Boş liste = kapı görünmüyor → gate_follower ham GN'ye düşer.
        self._edge_buoys: list[tuple[float, float]] = []
        # Dairesel engeller DÜNYA ENU'da (x, y, r) — MPPI'ye giden torbanın
        # AYNISI, kopya değil aynı taramadan. Kapı NİŞANININ engellere göre
        # kayması için gerekli: kenar dubaları MPPI'de engel olmadığından
        # geçitte iten tek kuvvet budur (gate_follower.aim_point).
        self._obstacles_world: list[tuple[float, float, float]] = []
        # classified_obstacles hiç aktı mı? (yalnız log/teşhis için — hakemlik
        # artık TAZELİKLE yapılıyor, aşağıdaki `_last_classified_t`)
        self._classified_seen = False
        # H4: sınıflı akışın SON varış anı. Mandal artık tek yönlü değil —
        # akış susarsa ham LiDAR yolu devralır (`_on_obstacles` docstring'i).
        self._last_classified_t: Optional[float] = None
        self._sinifsiz_uyarildi = False
        # H5: "akıyor ama hep boş" kapanı (`_bos_akis_denetle`)
        self._son_dolu_akis_t: Optional[float] = None
        self._bos_akis_uyarildi = False
        self._gate_log_t = 0.0
        self._last_gate_used_fallback = True
        # Algı karesi sayacı — her `classified_obstacles` mesajında artar.
        # `GateFollower`'ın B5 onay sayacı buna bakar: hedef 5 Hz tazelenirken
        # algı 1 Hz'e düşebilir (§11.3 kümeleme ölçümü), o zaman AYNI kare iki
        # kez sayılıp onay boşa çıkardı. Kimliği vermek bunu imkânsız kılar.
        self._algi_no = 0

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
        # H4 eşiği AYRI BİR AYAR DEĞİL, F-P.2 bütçesinden TÜRER: durdurma
        # bütçesinin yarısında yedeğe geç → devralmak için bütçenin yarısı
        # hep elde kalır. Bütçe 0 (bekçi kapalı) ise yedek de kapalı sayılır
        # ve eski davranış birebir korunur.
        self._classified_timeout = (
            self._obstacle_timeout / 2.0 if self._obstacle_timeout > 0.0
            else float("inf")
        )
        # H5 eşiği de F-P.2'den türer ama TERS yönde: bekçi "sustu"yu ölçer
        # (2 sn yeter), bu "hep boş"u ölçer. Boşluk geçici olabilir (dubaların
        # arasından çıkmış olabiliriz), o yüzden çok daha uzun: bütçenin 5
        # katı = 10 sn. 1,05 m/s'te ~10 m yol — parkurda bu kadar boşluk yok.
        self._bos_akis_uyari_s = (
            self._obstacle_timeout * 5.0 if self._obstacle_timeout > 0.0
            else 10.0
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
        # 🔴 KAR-04 (12.08): "komut SIFIR" ile "komut YOK" ayirt edilemiyordu.
        # Kaptanin bag analizinde 30.874 thrust mesajinin TAMAMI [0,0] idi ve
        # her oturumda BASKA bir kilit devredeydi (BOOT / KILL / BEKLEMEDE) —
        # ama bag'e bakan kisi bunu ancak dort ayri topic'i capraz okuyarak
        # cikarabildi. Sebep, komutun KENDI yaninda yayinlanmali.
        self._pub_inhibit = self.create_publisher(
            String, "/girdap/control/inhibit_reason", 10
        )
        self._son_inhibit = ""
        # 🔴 KAR-10 (12.08): ArduPilot GUIDED modunda setpoint akisi KESILIRSE
        # failsafe devreye girer. Kaptanin bag'inde bu topic 5 saatte 110
        # mesaj ve EN BUYUK SESSIZLIK 30 DAKIKA — akis hic kurulmamis.
        # Bu bekci, akisin kurulmus olmasi gereken anlarda (geçit açıkken)
        # gercek YAYIN ARALIGINI olcer. Ayni zamanda KAR-09'un 8-12 s'lik
        # donmalarini ICERIDEN gorur: bag'den sonradan cikarmak yerine olay
        # aninda log'a dusuruyor.
        self.declare_parameter("setpoint_bosluk_uyari_s", 0.5)
        self._setpoint_bosluk_s = float(
            self.get_parameter("setpoint_bosluk_uyari_s").value
        )
        self._son_setpoint_t: float | None = None
        self._setpoint_bosluk_sayaci = 0
        # Dosya-3: yerel maliyet haritası (RViz + local_map_node PNG dumper).
        self._pub_map = self.create_publisher(
            OccupancyGrid, "/girdap/map/local", sensor_data_qos()
        )
        # Saha teşhisi: kilitlenilen kapının orta noktası (kontrol yolu DEĞİL).
        self._pub_gate = self.create_publisher(
            PoseStamped, "/girdap/planning/gate", 10
        )
        # GEÇİŞ SAYACI (B3) — SALT TEŞHİS/KANIT kanalı, kontrol yolu DEĞİL.
        # `GateFollower.passed_gate_count` çekirdekte zaten hesaplanıyordu ama
        # hiçbir yere çıkmıyordu: ne operatör görüyordu ne de md 5.5.2.4'ün
        # "en az 2 duba ikilisinden geçiş" şartı için G1/G2 kanıtı üretiliyordu.
        #
        # 🔴 FSM'e BİLEREK BAĞLANMADI. `fsm_node._on_gate_passed` gelen HERHANGİ
        # bir True'yu PARKUR3'e atlama tetiği sayıyor → sayaç oraya bağlanırsa
        # İLK kapıda Parkur-2 yarıda kesilir, md 5.5.2.4 sağlanmaz ve md 657
        # gereği P3'ün 145 puanı hiç açılmaz. O tuzağın çözümü ayrı bir karar
        # (A: tetiğe "sayaç ≥ 2" şartı · B: geçişi waypoint ilerlemesinden sür,
        # sayaç yalnız kanıt olsun — GIRDAP_DURUM §0.6d/§18-4).
        # Bu kanal o kararı BEKLEMEDEN güvenle açılabilir: kimse tüketmiyor,
        # yalnız operatör görüyor ve hakem sorarsa puan kanıtı oluyor.
        self._pub_gate_count = self.create_publisher(
            Int32, "/girdap/planning/gate_count", 10
        )
        self._son_gate_count = -1        # yalnız DEĞİŞİNCE yayınla + logla
        # KENAR DUBALARI — DÜNYA (odom) çerçevesinde, Dosya-3 çizimi için.
        #
        # 🔑 **Neden ayrı topic, neden local_map_node kendisi dönüştürmüyor:**
        # kenar dubaları MPPI'nin engel torbasından bilerek çıkarılır
        # (`_on_classified`), dolayısıyla `local_cost_grid()` occupancy'sinde
        # HİÇ görünmezler → teslim edilen "engel haritası" parkurun ANA
        # nesnesini göstermiyordu (md 4.2 Dosya-3 denetimi, 2026-08-07).
        # Çizim katmanı olarak eklenmeleri gerekiyor; ama gövde→dünya
        # dönüşümünü ikinci bir node'da TEKRAR yazmak, bu projenin iki kez
        # yediği "iki kopya ayrıştı" hatasını davet ederdi (§0.0b). Dönüşüm
        # TEK yerde (`_body_to_world`) kalsın diye sonuç burada yayınlanır.
        # ⚠ Salt TEŞHİS/ÇİZİM kanalı — hiçbir kontrol kararı buradan sürülmez.
        self._pub_edge_buoys = self.create_publisher(
            PoseArray, "/girdap/planning/edge_buoys", 10
        )

        # 🔴 ARIZA KODLARI → YER KONTROL İSTASYONU (kaptan isteği, 13.08.2026).
        #
        # Ölçüldü (13.08 02:09, canlı yığın): LiDAR ağı düşmüştü, bu düğüm
        # saniyede bir "engel haritası HİÇ gelmedi → MPPI DURDURULDU" basıyordu
        # — ama YALNIZ ROS günlüğüne. `/mavros/statustext/send` hattında 12
        # saniyede TEK mesaj yoktu. Kıyıdaki operatörün elinde yalnız Mission
        # Planner var; teknenin niye durduğunu görmesinin hiçbir yolu yoktu.
        #
        # `fsm_node` görev durumunu zaten bu hattan bildiriyor; buraya yalnız
        # ALT SİSTEM arızaları giriyor. Metinler 50 karakter ve tazeleme
        # periyoduyla sınırlı — 868 MHz telsizin hava hızı ~2,1 KB/s (§10.1;
        # 16.07'de hat dolunca uçuş kontrolcüsü komut kabul etmemişti).
        self._pub_statustext = self.create_publisher(
            StatusText, "/mavros/statustext/send", 10
        )
        self.declare_parameter("ariza_statustext_periyot_s", 20.0)
        self._ariza = ArizaBildirici(
            tazeleme_s=float(
                self.get_parameter("ariza_statustext_periyot_s").value
            )
        )
        # RRT* düz çizgiye düştüğünde sayaç artar; arızayı sayacın ARTIŞINDAN
        # türetiyoruz (mutlak değerinden değil) — yoksa bir kez düşen plan
        # görev boyunca "RRT reddetti" diye görünürdü.
        self._son_duz_cizgi_sayaci = 0
        # 🔴 OLAY TABANLI ARIZALARIN DÜŞME SÜRESİ (13.08.2026 düzeltmesi).
        # `SETPOINT` ve `CMDVEL` bir DURUM değil, olmuş bitmiş bir OLAYDIR
        # ("akışta 1,4 sn boşluk oldu"). Olayın kendisi hiçbir zaman
        # "düzelmez", dolayısıyla durum yüklemi de yazılamaz: son olaydan bu
        # kadar saniye sonra arıza düşer. Tazeleme periyoduyla aynı seçildi —
        # böylece geçici bir olay ekranda en az bir kez görünür (değişim anında
        # hemen gönderilir), sonra kendiliğinden temizlenir.
        self.declare_parameter("ariza_olay_tutma_s", 20.0)
        self._ariza_olay_tutma_s = float(
            self.get_parameter("ariza_olay_tutma_s").value
        )
        self._son_setpoint_bosluk_t: Optional[float] = None
        self._son_cmdvel_bosluk_t: Optional[float] = None
        self._ariza_timer = self.create_timer(1.0, self._ariza_gonder)

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
        # 🔴 HESAP YOLU (2026-08-10): `backend="auto"` cupy'yi bulamazsa SESSİZCE
        # numpy'ye iniyor ve N=100 engelde adım 3,7 ms → 144 ms'ye çıkıyor
        # (10 Hz bütçesi 100 ms). Jetson ekransız koştuğu için belirti vermez →
        # hangi yola çözüldüğü YAZILIR (MPPI kurulur kurulmaz, `_on_control_step`
        # içinden). Ayrıntı: `MPPIController.backend_adi`.
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
                f"kapı takibi AÇIK: kenar dubası sınıfı={self._edge_class_id}, "
                f"gövde {self._gate._cfg.hull_width_m}×"
                f"{self._gate._cfg.hull_length_m} m "
                "— turuncu dubalar engel torbasından ÇIKARILIR. "
                "Ayarlanabilir eşik yok: geçilebilirlik gövde genişliğinden, "
                "kapı ayrımı |Δileri|<|Δyanal| geometrisinden gelir."
            )
        else:
            self.get_logger().warn(
                "kapı takibi KAPALI (gate_following_enabled=false) → ham görev "
                "noktasına gidilir; md 5.5.2.2 puanı kapıdan geçmekten gelir"
            )

    def _yeniden_basla(self) -> None:
        """md 5.5.3.1 yeniden baslama — planlama tarafinin sifirlanmasi.

        Dort sey birden:
          1. `reset()`              — kilitli kapi + yarim kalmis aday temiz
          2. `reset_passed_gates()` — 🔴 PUAN SIFIRLAMASI. Sartname: "yeniden
             baslama hakkini kullanan takimin topladigi puanlar SIFIRLANACAKTIR."
             Temizlenmezse ikinci turda ayni gecitler "zaten gecildi" sayilir ve
             HICBIRI puan getirmez. K1 listesi de burada temizleniyor: arac
             fiilen basa dondugu icin o kapilar artik ARKADA degil ONDE.
          3. `EdgeBuoyMemory.temizle()` — eski hatirlanan kenarlar artik
             yanlis yerde; tasinirsa hayalet kapi uretir. (Mevcut API
             kullaniliyor; ayni isi yapan ikinci bir metot EKLENMEDI.)
          4. `PlanningPipeline.yeniden_basla()` — MPPI warm-start + kayan
             pencere capasi + PID integratoru. Capa temizlenmezse arac basa
             dondugunde her adim kenar-fallback'e duser (58 -> 791 ms).
        """
        self._gate.reset()
        self._gate.reset_passed_gates()
        self._edge_memory.temizle()
        self._pipe.yeniden_basla()
        self._edge_buoys = []

    # ----- subscriber callback'leri -----

    @_guard
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

    @_guard
    def _on_obstacles(self, msg: PoseArray) -> None:
        """PLACEHOLDER şema: position.{x,y} merkez, orientation.z yarıçap.

        Sınıflı topic (`classified_obstacles`) **TAZE AKARKEN** bu yol susar —
        aynı engeller oradan sınıf bilgisiyle birlikte geliyordur ve kapı
        dubalarının ayıklanması yalnız orada mümkündür (F-S.9).

        🔴 **H4 (2026-08-09) — MANDAL ÇİFT YÖNLÜ YAPILDI.** Eskiden ölçüt
        `_classified_seen` idi: sınıflı akış **bir kez** geldiyse bu yol
        KALICI olarak susuyordu. Koşu ortasında kamera/OAK/füzyon düşerse
        (F-P.22'de gerçek donanımda yaşandı) sonuç şuydu:

            classified susar → ham LiDAR yolu mandal yüzünden kapalı →
            engel torbası DONAR → `_last_obstacle_t` güncellenmez →
            2 sn sonra F-P.2 thrust'ı sıfırlar → **tekne kalıcı durur**

        …oysa LiDAR sapasağlam ve tek başına kaçınma yapabilir: sınıfsız kol
        gerçek parkurda P1'i **53,75 puanla bitiriyor** (§0.20c ölçümü, model
        yokken 3/3 tohum). Yani çalışan bir yedek varken kapatılmıştı.

        Artık ölçüt **tazelik**: sınıflı akış `_classified_timeout` kadar
        susarsa bu yol kendiliğinden devralır, geri gelince yine susar.
        Eşik F-P.2'nin durdurma bütçesinin YARISI — yedeğin devreye girmesi
        için bütçenin yarısı hep elde kalır, "durmadan hemen önce devral"
        gibi işe yaramaz bir eşik olmaz.
        """
        if self._use_classified and self._classified_taze():
            return
        self._sinifsiz_yola_dusuldu()
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
        self._obstacles_world = [(o.cx, o.cy, o.r) for o in obstacles]
        self._pipe.set_obstacles(obstacles)
        self._bos_akis_denetle(len(obstacles))

    def _classified_taze(self) -> bool:
        """Sınıflı akış hâlâ geliyor mu (H4 mandalının ölçütü)."""
        if self._last_classified_t is None:
            return False
        return (self._now() - self._last_classified_t) <= self._classified_timeout

    def _sinifsiz_yola_dusuldu(self) -> None:
        """Sınıflı → sınıfsız geçişini BİR KEZ logla (10 Hz'te sel olmasın).

        Sahadaki tek görünürlük kanalı: bu satır basılmışsa kapı takibi artık
        çalışmıyor demektir (tüm dubalar engel), yani P1/P2 geçiş puanı
        düşecek — ama araç sürmeye devam ediyor.
        """
        if not self._classified_seen or self._sinifsiz_uyarildi:
            return
        self._sinifsiz_uyarildi = True
        self.get_logger().error(
            f"sınıflı algı {self._classified_timeout:.1f} sn'dir gelmiyor → "
            "HAM LiDAR yoluna düşüldü. Araç sürmeye devam eder ama KAPI TAKİBİ "
            "YOK (tüm dubalar engel; geçiş puanı düşer). Kamera/OAK/füzyon "
            "node'unu kontrol et."
        )
        # ⚠ Arıza kodu BURADAN basılmaz — bu dal mandallı (log seli olmasın
        # diye bir kez çalışır). Telsize giden kod `_ariza_durumlardan_
        # guncelle`'de her turda `_classified_taze()` yüklemiyle kurulur;
        # akış dönünce kendiliğinden düşer.

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

        🔑 **KENAR DUBASI HAFIZASI (2026-08-09, §0.17e).** "Şu an turuncu
        görünen" ile "kenar dubası" aynı şey DEĞİL: kapı 12 m ise iki direk
        ancak 8,8-15 m arasında aynı karede görünür, daha yakında kadrajdan
        çıkarlar ve UNKNOWN olarak **engel torbasına** düşerler → MPPI tam kapı
        ağzında dışarı iter. Kaybolan konum değil RENKTİR (Livox 360°/25 m
        konumu akıtmaya devam eder) ve renk bir kez öğrenildi. Ölçüldü: 12 m'de
        hafızasız **1/4** güzergah noktası, hafızalı **4/4**.
        Ayrıntı + eşleşme ölçüsü: `prototype/mission/edge_memory.py`.
        """
        self._classified_seen = True
        self._last_classified_t = self._now()        # H4: tazelik saati
        self._sinifsiz_uyarildi = False              # akış döndü → uyarı sıfırla
        self._last_obstacle_t = self._now()          # F-P.2 bekçisini besle (poz
                                                     # kontrolünden ÖNCE — üstteki
                                                     # _on_obstacles notuna bak)
        if self._last_xy is None:                 # poz yok → dönüştürülemez
            return

        # Tespitleri önce DÜNYA çerçevesine al (sınıf kararı ondan sonra).
        tespitler: list[tuple[float, float, float, Optional[int]]] = []
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
            # bbox.size.x = çap (perception_fusion_node sözleşmesi)
            tespitler.append((wx, wy, abs(det.bbox.size.x) / 2.0, cls))

        if self._gate_enabled:
            kenar_mi = self._edge_memory.siniflandir(tespitler, self._edge_class_id)
            # 🆕 H1 (§0.21): bu karede GÖRÜLMEYEN cisimler de haritada kalır.
            # Eskiden engel torbası her karede sıfırdan kuruluyordu → o an
            # görünmeyen cisim planlayıcı için YOKTU. Kapıya yaklaşırken
            # direkler önce kameranın 69°'lik kadrajından, sonra LiDAR'ın
            # (30 cm duba için ~8 m) menzilinden çıkıyor ve kapı ortadan
            # kayboluyordu. UNUTMA YOK — kaptan kararı 09.08 (gerekçe:
            # `EdgeBuoyMemory.hatirlananlar` docstring'i).
            # 🔴 YAYIM MENZİLİ (2026-08-10, §0.26b-c): hatırlanan cisimlerin
            # yalnız yerel harita penceresi içindekiler engel torbasına girer.
            # Kayıt SİLİNMİYOR (kaptan kararı korunuyor) — araç yaklaşınca geri
            # gelir. Sebebi ölçüm: konum sıçraması çakışma bandının üçte birini
            # geçince aynı duba ikinci kayıt açıyor ve torba sınırsız büyüyor;
            # bedeli `_huni_payi`'nin O(n²) saf Python taramasında ve MPPI'nin
            # (K,T+1,N) engel tensöründe ödeniyor. Yarıçap uydurulmadı: yerel
            # maliyet haritasının kendi penceresi (planlayıcının akıl yürüttüğü
            # alan) ve LiDAR `max_range`'iyle de örtüşüyor.
            # KAR-11: unutma menzili = yayim menzilinin 2 KATI. Suzmek
            # yetmiyordu; maliyet yayimda degil TARAMADA (her tespit x her
            # kayit). Canli olcum: 2404 kayit, dongu 117 -> 1062 ms (9x).
            # 2x pay birakiliyor ki arac donunce hala isimize yarayabilecek
            # kayitlar silinmesin (09.08'in "unutma yok" gerekcesi korunuyor);
            # yalniz cok geride kalmis, bir daha kullanilmayacak kopyalar duser.
            for tespit, kenar in self._edge_memory.hatirlananlar(
                self._last_xy,
                self._harita_yaricapi,
                unutma_menzili=self._harita_yaricapi * 2.0,
            ):
                tespitler.append(tespit)
                kenar_mi.append(kenar)
        else:
            kenar_mi = [False] * len(tespitler)

        obstacles: list[CircleObstacle] = []
        edges: list[tuple[float, float]] = []
        for (wx, wy, r, _cls), kenar in zip(tespitler, kenar_mi):
            if kenar:
                edges.append((wx, wy))
            else:
                obstacles.append(CircleObstacle(wx, wy, r))

        # B2 HUNİ: kapı direkleri kenar OLARAK KALIR ama çarpışma korumasından
        # çıkarılmaz — payları ölçülen açıklıktan türetilerek engel torbasına da
        # girerler (aşağıdaki `_huni_payi`). Ölçüldü: −0,231 → −0,019 m gövde
        # payı, üstelik geçilen kapı 6/8 → 7/8 (huni geçidi kapatmıyor, aksine
        # aracı ortadan geçmeye zorluyor).
        for i, (ex, ey) in enumerate(edges):
            m = self._huni_payi(i, edges)
            obstacles.append(CircleObstacle(ex, ey, BUOY_RADIUS_M, margin=m))

        self._edge_buoys = edges
        self._log_edge_memory()
        # ⚠ H5 TAZE tespit sayısına bakar, hafızadan eklenenlere DEĞİL: hafıza
        # doluyken algı ölse torba "dolu" görünür ve boş-akış kapanı körleşirdi.
        self._bos_akis_denetle(len(msg.detections))
        self._obstacles_world = [(o.cx, o.cy, o.r) for o in obstacles]
        self._pipe.set_obstacles(obstacles)
        self._algi_no += 1                # B5 onayı: yeni algı karesi geldi
        self._publish_edge_buoys(edges)

    def _huni_payi(
        self, i: int, kenarlar: list[tuple[float, float]]
    ) -> float:
        r"""B2 HUNİ — kapı direğinin engel payı, ÖLÇÜLEN açıklıktan türer.

        🔴 **Çözdüğü arıza (§0.2b B2 + §0.17g/2).** Kenar dubaları engel
        torbasından tamamen çıkarılıyordu; gerekçe doğruydu (küresel 1,0 m'lik
        ceza halkası dar bir geçidin içini kaplar, 1,5 m'de araç geçitten HİÇ
        geçmiyor) ama sonuç fazla keskindi: **dubalardan iten hiçbir kuvvet
        kalmıyordu.** Çarpma cezası (Ç1/Ç2) kenar dubalarını da sayar — P1'de
        16, P2'de 30 puan; md 815-818'e göre aynı dubaya 30 sn temas = 2 çarpma.
        Ölçülen gövde payı **−0,23 m** (temas).

        **Formül:**

            m = clamp( (W − hull_width − 2r) / 2 , 0 , gate_post_margin_m )

        `W` = bu direğin **en yakın diğer kenar dubasına** ölçülen mesafe.
        Formülün girdileri ya ölçülmüş tekne boyutu (`hull_width_m`) ya şartname
        sabiti (duba çapı 30 cm) ya da **o an ölçülen** geometri; tek serbest
        sayı TAVAN'dır ve o da kapı SEÇİMİNE değil kaçınma şiddetine ait
        (`obstacle_margin` ile aynı aile) → §0.0d'nin donmuş kuralı bozulmuyor.

        🔑 **Neden "en yakın diğer kenar", "kapının kendi genişliği" değil:**
        koridoru daraltan şey her zaman kapının kendi direkleri olmayabilir —
        ardışık kapıların direkleri birbirine kapı açıklığından daha yakın
        olabilir (gerçek P1: partner 12 m, komşu kapının direği 6,4 m). Ölçüt
        "geçmem gereken en dar boşluk" olmalı; bu tanım kapıyı da komşuluğu da
        kapsar ve kapı eşleştirmesinin YAPILMASINI beklemez (bu fonksiyon
        `_on_classified`'da, kapı seçiminden ÖNCE çalışır).

        🔑 **Tavan neden AYRI bir parametre, `obstacle_margin` DEĞİL:** küresel
        payı büyütmek gövde payını düzeltir ama **model gelmeyen kolu kırar**
        (ölçüm: 1.4'te ham güzergah noktasına sürüş 3,3/4 → 1/4 nokta; hakemin
        noktası dubaya ~2,2 m yakın olabiliyor ve büyüyen halka aracı 2,0 m'lik
        varış yarıçapına sokmuyor). Huni ise sınıf gelmeden HİÇ devreye girmez
        → iki kol birbirini bozmadan ayrı ayrı ayarlanabiliyor.

        Gerçek P1'de (12 m) formül tavana dayanır, yani direkler normal engel
        gibi davranır — dar kapıda ise pay kendiliğinden küçülür ve geçidi
        asla kapatmaz.
        """
        if len(kenarlar) < 2:
            return self._gate_post_margin
        dx, dy = kenarlar[i]
        # İndeksle dışla, koordinatla DEĞİL: iki tespit aynı noktaya düşerse
        # koordinat karşılaştırması ikisini birden eler ve pay tavana çıkardı.
        en_yakin = min(
            math.hypot(dx - kx, dy - ky)
            for j, (kx, ky) in enumerate(kenarlar) if j != i
        )
        serbest = en_yakin - self._gate._cfg.hull_width_m - 2.0 * BUOY_RADIUS_M
        return max(0.0, min(self._gate_post_margin, serbest / 2.0))

    def _log_edge_memory(self) -> None:
        """Hafızanın sahadaki tek görünürlük kanalı — 5 sn'de bir özet.

        Neden gerekli: hafıza sessiz çalışır. Tutmuyorsa belirtisi yalnız
        "araç kapı ağzında dışarı itiliyor"dur ve bunu logdan ayırt etmek
        imkânsızdır. `kurtarılan` 0 kalıyorsa hafıza iş görmüyor demektir.
        """
        now = self._now()
        if now - self._edge_mem_log_t < 5.0:
            return
        self._edge_mem_log_t = now
        if not self._gate_enabled or self._edge_memory.boyut == 0:
            return
        # 🔑 Sahada anlamlı olan sayı `boyut` değil **hız**: pencere başına açılan
        # yeni kayıt. Araç yeni bir bölgeye girmiyorken bu sıfıra inmiyorsa yeni
        # cisim görülmüyordur, AYNI cisim için ikinci kayıt açılıyordur (§0.26b:
        # konum sıçraması çakışma bandının üçte birini geçince başlıyor).
        yeni = self._edge_memory.acilan_kayit - self._edge_mem_son_acilan
        self._edge_mem_son_acilan = self._edge_memory.acilan_kayit
        self.get_logger().info(
            f"kalıcı harita: {self._edge_memory.boyut} kayıt "
            f"(son 5 sn'de +{yeni} yeni, {self._edge_memory.son_menzil_disi} tanesi "
            f"{self._harita_yaricapi:.0f} m menzil dışı → torbaya konmadı), "
            f"rengi görünmezken kurtarılan tespit "
            f"{self._edge_memory.hatirlanarak_kurtarilan}, "
            f"unutulan {self._edge_memory.unutulan}, "
            f"sınıfı güncellenen {self._edge_memory.celiskiyle_silinen} "
            f"(şu an {len(self._edge_buoys)} kenar / kapı takibi)"
        )

    def _obstacles_stale(self) -> bool:
        """F-P.2: son obstacle_map `obstacle_timeout_s`'ten eski mi? 0 → kapalı.

        🔴 KAR-03 (12.08) — odomdaki ile AYNI kör nokta buradaydı ve burada
        sonucu daha ağır: engel haritası hiç gelmediyse MPPI'nin engel torbası
        BOŞTUR, yani "önüm tamamen açık" demektir. Eski kod bunu bekçiye
        göstermiyordu → algı düğümü hiç açılmamışken araç, engel görme
        yeteneği olmadığından habersiz, tam güvenle ilerlerdi. Algı çöktükten
        SONRAKİ 2 saniye korunuyordu ama HİÇ açılmamış olması korunmuyordu.
        """
        if self._obstacle_timeout <= 0.0:
            return False
        if self._last_obstacle_t is None:
            return True
        return (self._now() - self._last_obstacle_t) > self._obstacle_timeout

    def _warn_stale_obstacles(self) -> None:
        """Bayatlık uyarısı — "hiç gelmedi" ve "eskidi" ayrı (bkz. `_warn_stale_odom`)."""
        now = self._now()
        if self._last_obstacle_t is None:
            if now - self._obstacle_stale_warn_t < 5.0:
                return
            self._obstacle_stale_warn_t = now
            self.get_logger().error(
                "engel haritası HİÇ gelmedi → MPPI DURDURULDU, thrust sıfır. "
                "Algı düğümü ayakta mı, LiDAR veri veriyor mu? "
                "(F-P.2/KAR-03: engel torbası boşken 'önüm açık' sanılmaz)"
            )
            return
        if now - self._obstacle_stale_warn_t < 1.0:
            return
        self._obstacle_stale_warn_t = now
        age = now - self._last_obstacle_t
        self.get_logger().error(
            f"engel haritası {age:.1f}s'dir gelmiyor → MPPI DURDURULDU, "
            "thrust sıfır (F-P.2: bayat engel bilgisiyle kör sürme yok)"
        )

    def _bos_akis_denetle(self, n_engel: int) -> None:
        """🔴 H5 — "AKIYOR AMA HEP BOŞ" kapanı (2026-08-09 taraması).

        F-P.2 bekçisi mesajın **varış zamanına** bakar, **içeriğine** bakmaz:
        her mesajda `_last_obstacle_t` tazelenir. Dolayısıyla algı her karede
        **boş dizi** yayınlarsa her şey sağlıklı görünür ve araç **sıfır
        engelle** sürer — hiçbir yerde tek satır uyarı basılmadan.

        Bu varsayımsal değil, projenin YAŞADIĞI arıza: B0/F5.1'de LiDAR z
        filtresi yanlış çerçevede uygulanınca `obstacle_map` sürekli boş
        geliyordu (§0.2b). O gün belirti yoktu; bugün olacak.

        Ölçüt uydurma değil, fiziksel: parkurda **her zaman** duba vardır
        (şartname md 5.5.2.1 — kenar dubaları parkuru tanımlar). Görevdeyken
        `boş_akış_uyarı_s` boyunca TEK BİR cisim bile görülmediyse algı
        çalışmıyordur; açık suda bile 25 m menzilde sıfır dönüş fiziksel
        olarak beklenmez.
        """
        now = self._now()
        if n_engel > 0:
            self._son_dolu_akis_t = now
            self._bos_akis_uyarildi = False
            return
        if self._son_dolu_akis_t is None:            # boot — henüz hiç dolu gelmedi
            self._son_dolu_akis_t = now
            return
        if self._bos_akis_uyarildi:
            return
        if now - self._son_dolu_akis_t < self._bos_akis_uyari_s:
            return
        self._bos_akis_uyarildi = True
        self.get_logger().error(
            f"algı {now - self._son_dolu_akis_t:.0f} sn'dir AKIYOR ama HEP BOŞ "
            "(0 engel). F-P.2 bekçisi bunu yakalamaz — mesaj geliyor, içi boş. "
            "Araç ENGELSİZ sürüyor. Kontrol: LiDAR z filtresi doğru çerçevede "
            "mi (B0/F5.1, mount_z girili mi) · `ros2 topic echo "
            "/perception/obstacle_map --once` · kümeleme logunda nokta sayısı"
        )
        # ⚠ Arıza kodu BURADAN basılmaz (yukarıdaki `SINIF-YOK` gerekçesi):
        # bu dal mandallı, telsize giden kod durum yükleminden kurulur.

    def _refine_target(self, coarse: tuple[float, float]) -> tuple[float, float]:
        """Ham görev noktasını (GN) algılanan kapının NİŞAN NOKTASIYLA değiştir.

        Şartname md 5.5.2.2: Parkur-1/2 puanı iki KENAR dubasının arasından
        geçmekten gelir ve hakemin verdiği nokta tam kapı ortasında OLMAYABİLİR
        → ham GN'ye yönelmek puan kaybettirir. Kapı görünmüyorsa (menzil dışı,
        oklüzyon, Parkur-3) `GateFollower` ham GN'ye düşer; yani bu çağrı
        kapısız durumda DAVRANIŞI DEĞİŞTİRMEZ (geriye tam uyumlu).

        🔑 **MPPI ile birlikte çalışma noktası burası.** Dönen değer doğrudan
        MPPI'nin referansı olur (`set_reference_direct` / `set_waypoints` →
        `PlanningPipeline`). Nişan kör orta nokta DEĞİL, kapı kirişi üzerinde
        engellerden en açık yerdir; bu yüzden aynı taramadan gelen engel listesi
        de `GateFollower`'a geçirilir. Engel yoksa nişan tam ortadır (eski
        davranış birebir).

        ⚠️ **2026-08-10 düzeltmesi — bu docstring bayattı.** Eskiden burada
        *"kenar dubaları engel torbasından çıkarıldığı için geçitte iten tek
        kuvvet nişandır"* yazıyordu; **B2 HUNİ'den (§0.18d) beri doğru değil**:
        direkler torbada KALIYOR, payları ölçülen açıklıktan türüyor
        (`_huni_payi`). Yani geçitte iki kuvvet var — nişanın çekimi ve huninin
        itmesi. Ölçüldü: direklerin torbada olması nişanı **kaydırmıyor**
        (simetrik kapıda kayma 0,000 m), yani iki mekanizma çakışmıyor.

        `gate_following_enabled=false` → tamamen devre dışı, eski davranış.
        """
        if not self._gate_enabled or self._last_xy is None:
            return coarse
        result = self._gate.update(
            self._last_xy, coarse, self._edge_buoys, self._obstacles_world,
            gozlem_no=self._algi_no,
        )
        # Kapı bulundu/kaybedildi geçişini bir kez logla (10 Hz'te spam yok).
        if result.used_fallback != self._last_gate_used_fallback:
            self._last_gate_used_fallback = result.used_fallback
            if result.gate is not None:
                kayma = result.gate.aim_shift
                self.get_logger().info(
                    f"kapı KİLİTLENDİ: nişan ({result.target[0]:.1f}, "
                    f"{result.target[1]:.1f}), genişlik {result.gate.width:.1f} m, "
                    f"ortadan kayma {kayma:.2f} m "
                    f"({'engel var, nişan kaydı' if kayma > 0.01 else 'temiz, tam orta'})"
                    " — MPPI referansı buraya kuruluyor"
                )
            else:
                self.get_logger().info(
                    "kapı görüş dışı → ham görev noktasına dönüldü (fallback)"
                )
        if result.gate is not None:
            self._publish_gate(result.target)
        else:
            self._warn_sessiz_ret()
        self._publish_gate_count()
        return result.target

    def _publish_edge_buoys(self, edges: list[tuple[float, float]]) -> None:
        """Kenar dubalarını DÜNYA (odom) çerçevesinde yayınla — Dosya-3 katmanı.

        Boş liste de yayınlanır: "kapı görünmüyor" da bir bilgidir ve çizici
        bayat duba göstermemelidir.
        """
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"          # ⚠ DÜNYA — gövde değil
        for wx, wy in edges:
            p = Pose()
            p.position.x = float(wx)
            p.position.y = float(wy)
            msg.poses.append(p)
        self._pub_edge_buoys.publish(msg)

    def _publish_gate_count(self) -> None:
        """Geçilen FARKLI kapı sayısı — md 5.5.2.4 için G1/G2 kanıtı.

        Yalnız DEĞİŞTİĞİNDE yayınlanır (20 Hz'te sabit sayı basmanın anlamı
        yok) ve aynı anda INFO'ya düşer: operatör telemetride "kaç kapıdan
        geçtik"i görür, koşu sonrası da logda kalır.

        ⚠ Bu bir TEŞHİS kanalıdır — hiçbir node tüketmiyor, kontrol/geçiş
        kararı buradan sürülmüyor (yukarıdaki fsm tuzağı notu).
        """
        n = self._gate.passed_gate_count
        if n == self._son_gate_count:
            return
        self._son_gate_count = n
        self._pub_gate_count.publish(Int32(data=int(n)))
        self.get_logger().info(
            f"KAPI GEÇİLDİ — toplam {n} farklı kapı "
            "(md 5.5.2.4 kanıtı; parkur geçişi bu sayıdan SÜRÜLMÜYOR)"
        )

    def _warn_sessiz_ret(self) -> None:
        """Turuncu duba GÖRÜNÜYOR ama kapı oluşmuyorsa ne olduğunu yaz.

        Kapı seçiminde ayarlanabilir eşik kalmadı (hepsi ölçülmüş tekne
        boyutu ya da geometri) — dolayısıyla bu artık "ayarı düzelt" uyarısı
        değil, **algı teşhisi**: kapı oluşmuyorsa sebep neredeyse kesin olarak
        tespit tarafındadır (dubanın biri görünmüyor, renk sınıfı kaçmış,
        LiDAR kümesi bölünmüş). Sahada bakılacak yer orası.

        5 saniyede bir basılır (10 Hz döngüde spam yok).
        """
        d = self._gate.last_diagnostics
        if d.n_edge_buoys < 2:
            return                     # tek duba: kapı beklemek zaten anlamsız
        now = self._now()
        if now - self._gate_log_t < 5.0:
            return
        self._gate_log_t = now
        sebep = []
        if d.reddedilen_genislik:
            dar = ", ".join(f"{s:.2f}" for s in sorted(d.reddedilen_genislik)[:4])
            # ⚠ Karşılaştırma MERKEZ-merkez mesafeyle yapılır; sığma eşiği
            # gövde genişliği DEĞİL `hull + 2r`'dir (duba yüzeyleri arası
            # serbest açıklık) — mesajda o sayı basılmalı, yoksa operatör
            # "0.90 < 0.78" gibi yanlış görünen bir satır okur.
            esik = self._gate._cfg.min_passable_width
            sebep.append(
                f"{len(d.reddedilen_genislik)} çift GEÇİLEMEZ (merkez arası "
                f"{dar} m < {esik:.2f} m = gövde {self._gate._cfg.hull_width_m} "
                "+ 2×duba yarıçapı 0.15 — tekne sığmaz, muhtemelen tek duba "
                "iki tespite bölünmüş)"
            )
        if d.reddedilen_derinlik:
            sebep.append(
                f"{d.reddedilen_derinlik} çift kursa DİK DEĞİL (ardışık "
                "kapıların dubaları — normal)"
            )
        if d.reddedilen_gecilmis:
            # K1: bunlar ARKADA bıraktığımız kapılar. Görülmeleri normaldir;
            # elenmeleri de öyle — eskiden elenmedikleri için araç geri dönüp
            # sonsuz salınıyordu (§0.9b). Sahada "neden o kapıya gitmiyoruz"
            # sorusunun cevabı burada görünsün.
            sebep.append(
                f"{d.reddedilen_gecilmis} çift ZATEN GEÇİLDİ (arkada kaldı — "
                f"toplam {len(self._gate.gecilen_kapilar)} kapı geride)"
            )
        self.get_logger().warn(
            f"KAPI SEÇİLEMEDİ: {d.n_edge_buoys} turuncu duba görülüyor "
            f"({d.n_in_range} burun hattının önünde), {d.n_pairs_checked} çift "
            f"denendi. Sebep: {'; '.join(sebep) if sebep else 'karşılıklı çift yok'}. "
            "Kapı seçiminde ayarlanabilir eşik YOK → sorun algıda: dubanın "
            "biri görünmüyor ya da renk sınıfı kaçıyor olabilir."
        )
        # ⚠ Arıza kodu BURADAN basılmaz — bu dal 5 saniyede bir kısılıyor,
        # üstelik kapı KİLİTLENDİĞİNDE hiç çağrılmıyor. Telsize giden kod
        # `_ariza_durumlardan_guncelle`'de "kapı kilitli değil + en az iki
        # turuncu duba görünüyor" yüklemiyle kurulur; kapı kilitlenince düşer.

    @_guard
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

    @_guard
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

    @_guard
    def _on_mission_state(self, msg: String) -> None:
        # Parkur değişince kilitli kapıyı BIRAK: Parkur-1'in son kapısına
        # kilitliyken Parkur-2'ye geçilirse eski kapı hedefi taşınmamalı
        # (gate_follower.reset() sözleşmesi: "parkur geçişi / yeniden başlama").
        # ⚠ KENAR HAFIZASI BİLEREK SIFIRLANMIYOR (`_edge_memory.temizle()` YOK).
        # Kilitli kapıyı bırakmak doğru, hafızayı atmak DEĞİL: geçiş anında araç
        # Parkur-2'nin ilk kapısına 8,8 m'den yakınsa (12 m açıklıkta iki direğin
        # aynı karede görüldüğü pencerenin içi) rengi bir daha HİÇ öğrenemez ve
        # tam da düzeltilen arıza geri gelir. Yanlış hafızayı süre değil SINIF
        # ÇELİŞKİSİ temizler (edge_memory.py: sarı/hedef görülürse kayıt silinir).
        if msg.data != self._pipe.mission_state:
            self._gate.reset()
            self._last_gate_used_fallback = True
        self._pipe.set_mission_state(msg.data)

    @_guard
    def _on_mav_state(self, msg: MavState) -> None:
        """MAVROS mod/arm geçidi için FCU durumunu güncelle."""
        self._bridge.update_state(
            self._now(), msg.connected, msg.armed, msg.guided, msg.mode
        )

    # ----- kontrol döngüsü -----

    def _odom_stale(self) -> bool:
        """F-P.1: son odom `odom_timeout_s`'ten eski mi? 0 → bekçi kapalı.

        🔴 KAR-03 (12.08) — "HİÇ GELMEDİ" ARTIK BAYAT SAYILIR.
        Eskiden `_last_odom_t is None` → `False` dönüyordu, gerekçesi
        *"MPPI zaten kontrol üretmez (durum yok)"* idi. **Bu gerekçe yanlıştı:**
        `PlanningPipeline.__init__` durumu `np.zeros(6)` ile başlatır, yani
        "durum yok" diye bir hal YOKTUR — poz hiç gelmemişken de tam geçerli
        görünen bir (0,0,0) pozu vardır. FSM aktif duruma geçtiği anda MPPI o
        UYDURMA ORİJİNDEN gerçek thrust üretir, üstelik F-P.1 bekçisi tam da bu
        sırada susar. KAR-01'de `ARM ↔ PARKUR2` salınımı odometri (0,0,0)
        iken gerçekleşti — yani bu yol kuramsal değil, o oturumda yaşandı.
        Bekçinin amacı "bayat pozla kör sürme yok" idi; poz hiç gelmemesi
        bayat pozdan DAHA KÖTÜ bir durumdur, bekçinin kapsadığı ilk hal olmalı.

        (KAR-05'te füzyondaki, F8.2'deki aynı `is not None` kör noktası
        düzeltilmişti — desen tekrar ediyor: *bir bekçi yazarken "hiç olmadı"yı
        "eskidi"den AYRI ele al; sessiz arızaların çoğu birincisidir.*)
        """
        if self._odom_timeout <= 0.0:
            return False
        if self._last_odom_t is None:
            return True
        return (self._now() - self._last_odom_t) > self._odom_timeout

    def _warn_stale_odom(self) -> None:
        """Bayatlık uyarısını bas. İki hal AYRI: "hiç gelmedi" ve "eskidi".

        KAR-03: eski kod `age = now - (self._last_odom_t or now)` yazıyordu;
        poz hiç gelmemişken bu **0.0** verir ve log'a *"poz 0.0s'dir gelmiyor"*
        gibi kendi kendini yalanlayan bir satır düşerdi. "Hiç gelmedi" halinde
        yaş diye bir büyüklük yoktur — operatöre söylenmesi gereken şey yaş
        değil, NE YAPACAĞI.

        Cadans da ayrı: "hiç gelmedi" boot'ta dakikalarca sürebilir (MAVROS
        bağlanana kadar), saniyede bir ERROR log'u boğar → 5 s. "Eskidi" ise
        uçuş ortasında ani bir kayıptır, saniyede bir uyarılmalı.
        """
        now = self._now()
        if self._last_odom_t is None:
            if now - self._stale_warn_t < 5.0:
                return
            self._stale_warn_t = now
            self.get_logger().error(
                "poz HİÇ gelmedi → MPPI DURDURULDU, thrust sıfır. "
                "/girdap/fusion/odom akmıyor: MAVROS bağlı mı, füzyon düğümü "
                "ayakta mı? (F-P.1/KAR-03: uydurma orijinden sürme yok)"
            )
            return
        if now - self._stale_warn_t < 1.0:
            return
        self._stale_warn_t = now
        age = now - self._last_odom_t
        self.get_logger().error(
            f"poz {age:.1f}s'dir gelmiyor → MPPI DURDURULDU, thrust sıfır "
            "(F-P.1: bayat pozla kör sürme yok)"
        )

    def _now(self) -> float:
        """Bayatlık saati — TEK YÖNLÜ (§0.61). Mutlak an olarak kullanılmaz."""
        return self._saat()

    def _on_control_step(self) -> None:
        """20 Hz'te MPPI step → thrust komut + Twist setpoint (MAVROS geçitli).

        Geçit kuralları (prototype.control.mavros_bridge):
            - armed=False / heartbeat kaybı → thrust sıfır
            - mode != GUIDED → cmd_vel yayınlanmaz (mavros zaten yok sayar)

        Fail-safe: bu 20 Hz timer callback'i beklenmedik bir hata (MPPI sayısal
        çökme/NaN, yayım hatası) fırlatırsa korumasız bir timer callback'i tüm
        executor'ı durdurabilir → node ölür → tekne SON cmd_vel'le komutsuz
        sürer (md 3.3.1.1 istemsiz hareket). Bu yüzden gövde try ile sarılır ve
        HATADA motorlar aktif DURDURULUR (`_safe_stop`), yalnızca atlanmaz.
        """
        try:
            gate = self._bridge.control_gate(self._now())
            self._log_backend()      # MPPI kurulur kurulmaz bir kez yazar

            # KAR-04: sebepleri SIRAYLA topla — bir komutu birden fazla kilit
            # sifirlayabilir ve hepsini bilmek gerekir. Yalniz ilkini yazmak,
            # operator birini duzeltince "hala sifir" surprizi uretirdi.
            sebepler: list[str] = []

            u = self._pipe.compute_control()
            if u is None:
                u = np.zeros(2)
                # `compute_control` iki AYRI sebeple None döner ve operatör
                # icin bunlar hic benzemez: (a) FSM parkur disi — beklenen,
                # gorev henuz baslamadi; (b) FSM aktif ama KONTROLCU KURULU
                # DEGIL — referans/waypoint gelmemis, yani gorev basladi ama
                # arac kipirdamiyor. Ikisine ayni etiketi yazmak, KAR-04'te
                # bag'den cikarilmaya calisilan bilginin aynisini kaybederdi.
                # SIRA `compute_control`'un kendi mantigini yansitir: orada
                # once FSM durumu, sonra kontrolcu bakilir. Tersine cevirmek
                # bootta "kontrolcu hazir degil" yazardi — dogru ama YANILTICI,
                # cunku o asamada gorev zaten baslamamis.
                if self._pipe.mission_state not in _AKTIF_DURUMLAR:
                    sebepler.append(f"FSM-DISI({self._pipe.mission_state})")
                else:
                    sebepler.append("KONTROLCU-HAZIR-DEGIL")
            if gate.zero_thrust:                     # disarm / KILL → motor stop
                u = np.zeros(2)
                sebepler.append("DISARM-VEYA-KILL")
            if self._odom_stale():                   # F-P.1: poz bayat → kör sürme
                u = np.zeros(2)
                self._warn_stale_odom()
                sebepler.append(
                    "POZ-YOK" if self._last_odom_t is None else "POZ-BAYAT"
                )
            if self._obstacles_stale():               # F-P.2: engel bayat → kör sürme
                u = np.zeros(2)
                self._warn_stale_obstacles()
                sebepler.append(
                    "ENGEL-YOK" if self._last_obstacle_t is None
                    else "ENGEL-BAYAT"
                )

            self._publish_inhibit(sebepler, gate)
            self._ariza_kilitlerden_guncelle(sebepler)
            self._ariza.temizle(KONTROL_HATA)     # bu tur çökmeden tamamlandı
            self._publish_thrust(u)
            if gate.allow_cmd_vel:                   # yalnız GUIDED + armed
                self._setpoint_akisini_denetle()
                self._publish_cmd_vel(u)
            else:
                # Geçit kapandı → sayaç sıfırlanır, yoksa bir sonraki arm'da
                # kasıtlı sessizlik "kesinti" diye raporlanır.
                self._son_setpoint_t = None
        except Exception as exc:                     # kontrol adımı ASLA çökmemeli
            self.get_logger().error(
                f"kontrol adımı hatası → motorlar DURDURULDU: {exc!r}",
                throttle_duration_sec=2.0,
            )
            self._ariza.bildir(KONTROL_HATA)
            self._safe_stop()

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

    def _publish_gate(self, aim: tuple[float, float]) -> None:
        """Kilitli kapının NİŞAN noktası — RViz/saha teşhisi (kontrol yolu DEĞİL).

        Yayınlanan nokta MPPI'ye verilen hedefin AYNISIDIR; geometrik ortadan
        sapması engellere göre yapılan kaymadır. (Parametre adı eskiden
        `midpoint`'ti — artık ikisi ayrı kavram, bkz. `Gate.midpoint` ↔
        `Gate.aim`.)
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(aim[0])
        msg.pose.position.y = float(aim[1])
        msg.pose.orientation.w = 1.0
        self._pub_gate.publish(msg)

    def _publish_inhibit(self, sebepler: list[str], gate) -> None:  # noqa: ANN001
        """KAR-04: thrust neden sıfır — komutun yanında, aynı anda.

        Boş liste → "YOK" (hiçbir kilit yok, komut gerçek). Bu ayrım kritik:
        `[0,0]` bir kilidin sonucu da olabilir, MPPI'nin gerçekten sıfır
        istemesi de. Kaptanın 30.874 mesajlık analizinde bu ikisi ayırt
        edilemediği için her oturumun sebebi ayrı ayrı, dört topic çapraz
        okunarak çıkarılmak zorunda kalındı.

        `allow_cmd_vel` de metne giriyor (KAR-10): thrust üretiliyor ama
        FCU'ya setpoint gitmiyorsa — mod GUIDED değil ya da disarm — bu,
        "araç neden kıpırdamıyor" sorusunun bambaşka bir cevabıdır ve
        aynı satırda görünmeli.

        Yalnız DEĞİŞİMDE yayınlanır: 20 Hz'te sabit metin basmak bag'i
        şişirir ve asıl geçiş anını gürültüye gömer.
        """
        metin = ",".join(sebepler) if sebepler else "YOK"
        if not gate.allow_cmd_vel:
            metin += "|SETPOINT-KAPALI"
        if metin == self._son_inhibit:
            return
        self._son_inhibit = metin
        self._pub_inhibit.publish(String(data=metin))
        self.get_logger().info(f"kontrol kilidi degisti: {metin}")

    def _ariza_kilitlerden_guncelle(self, sebepler: list[str]) -> None:
        """Kilit sebeplerini arıza koduna çevir (tespit MANTIĞI TEK YERDE).

        `sebepler` KAR-04 için zaten her turda üretiliyor; telsize çıkan kod
        da aynı listeden türetilir, böylece ikisi hiçbir zaman ayrışamaz.
        Listede olmayan sebep-türevli arızalar bu turda DÜŞMÜŞ demektir —
        temizlenir ki operatör düzelen şeyi de görsün.
        """
        aktif = {t.kod for t in sebepten_kodla(sebepler)}
        for tanim in SEBEP_TUREVLI_ARIZALAR:
            self._ariza.ayarla(tanim, aktif=tanim.kod in aktif)

    def _ariza_rrt_denetle(self) -> None:
        """RRT* bu saniyede düz çizgiye düştü mü (kaptanın 'RRT reddetti')."""
        sayac = self._pipe.duz_cizgiye_dusuldu
        self._ariza.ayarla(RRT_RED, aktif=sayac > self._son_duz_cizgi_sayaci)
        self._son_duz_cizgi_sayaci = sayac

    def _ariza_durumlardan_guncelle(self) -> None:
        """Kilit listesi DIŞINDAKİ arızaları her turda DURUMDAN yeniden kur.

        🔴 **13.08.2026 düzeltmesi — mandal kusuru.** İlk sürümde bu kodlar
        yalnız `bildir()` ediliyordu ve hiçbir yerde `temizle()` edilmiyordu;
        yani bir kez ateşleyen kod görev sonuna kadar aktif kalıyordu. Ölçülen
        sonuç: `KAPI-YOK` parkurun OLAĞAN bir anıdır (iki turuncu duba görünüp
        çift kurulamadığı her an basılır) ⇒ ilk kapı yaklaşmasında kesin
        ateşliyor ⇒ o andan sonra **"ariza yok" bir daha hiç basılamıyor** ve
        gerçek arıza (LiDAR vb.) düzeldiğinde ekran temiz görünmek yerine
        dakikalar önce olmuş bir olaya düşüyordu. Operatör ekrandaki kodun
        ŞİMDİ mi GEÇMİŞTE mi olduğunu ayırt edemezdi.

        👉 Kural: **arıza kodu DURUMDUR, olay değil.** Her tur yeniden
        hesaplanır; koşul geçerse kod kendiliğinden düşer. Kilit sebebi
        türevlileri bunu zaten `_ariza_kilitlerden_guncelle`'de yapıyordu —
        bu metot aynı disiplini kalan kodlara uygular.

        ⚠ `GPU-YOK` BİLEREK dışarıda: MPPI hesap yolu açılışta bir kez seçilir,
        tekne koşu ortasında GPU'ya kavuşmaz. Onun mandallı kalması DOĞRU;
        `test_GPU_YOK_bilerek_mandalli_kalir` bu ayrımı donduruyor.
        """
        simdi = self._now()

        # SINIF-YOK: sınıflı akış bir kez görüldü ama artık taze değil.
        # `_sinifsiz_yola_dusuldu`'nun mandalı LOG seli içindir; arıza kodu
        # tazelik yükleminden gelir, böylece akış dönünce kod da düşer
        # (kurtarma dalı `_on_classified`'da zaten vardı, haber verilmiyordu).
        self._ariza.ayarla(
            SINIF_YOK,
            aktif=self._classified_seen and not self._classified_taze(),
        )

        # ENGEL-BOS: algı akıyor ama uyarı süresidir tek cisim bile yok.
        self._ariza.ayarla(
            ENGEL_BOS,
            aktif=(
                self._son_dolu_akis_t is not None
                and (simdi - self._son_dolu_akis_t) >= self._bos_akis_uyari_s
            ),
        )

        # KAPI-YOK: en az iki turuncu duba GÖRÜNÜYOR ama kapı kilitlenemiyor.
        # İki dubadan azken arıza değil (kapı beklemek zaten anlamsız) —
        # `_warn_sessiz_ret`'in kullandığı ölçütün aynısı, ayrışamasınlar.
        self._ariza.ayarla(
            KAPI_YOK,
            aktif=(
                self._gate_enabled
                and self._last_gate_used_fallback
                and self._gate.last_diagnostics.n_edge_buoys >= 2
            ),
        )

        # SETPOINT / CMDVEL: olay tabanlı — son olaydan `ariza_olay_tutma_s`
        # sonra düşer (bir olay "düzelmez", bu yüzden durum yüklemi yazılamaz).
        for tanim, olay_t in (
            (SETPOINT_BOSLUK, self._son_setpoint_bosluk_t),
            (CMDVEL_KESIK, self._son_cmdvel_bosluk_t),
        ):
            self._ariza.ayarla(
                tanim,
                aktif=(
                    olay_t is not None
                    and (simdi - olay_t) < self._ariza_olay_tutma_s
                ),
            )

    def _ariza_gonder(self) -> None:
        """Sırası gelen arıza kodunu STATUSTEXT ile yer istasyonuna yolla.

        🔴 ABONESİZ YAYIN SESSİZCE ÇÖPE GİDER — `fsm_node`'un aynı dersi:
        MAVROS'un `sys_status` eklentisi bu topic'e abone olana kadar
        yollanan her mesaj kaybolur. Abone yokken GÖNDERİLMİŞ SAYMIYORUZ:
        bildiriciye hiç sorulmuyor, böylece abone belirince aynı arıza
        bir sonraki turda yeniden denenir.

        ⚠ **Durum güncellemesi abone kontrolünden ÖNCE yapılır.** Aksi hâlde
        `_ariza_rrt_denetle`'nin sayaç tabanı abone yokken donar ve MAVROS
        abone olduğu anda, dakikalar önce olmuş bir düşüş için sahte bir
        `RRT-RED` çakardı — hem de tam operatörün ekranı açtığı saniyede.
        Güncelleme ucuzdur (yüklem hesabı), gönderim pahalıdır (telsiz).
        """
        self._ariza_rrt_denetle()
        self._ariza_durumlardan_guncelle()
        if self._pub_statustext.get_subscription_count() == 0:
            return
        gonderim = self._ariza.gonderilecek(self._now())
        if gonderim is None:
            return
        metin, seviye = gonderim
        msg = StatusText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.severity = seviye
        msg.text = metin
        self._pub_statustext.publish(msg)

    def _publish_thrust(self, u: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = [float(u[0]), float(u[1])]
        self._pub_thrust.publish(msg)

    def _setpoint_akisini_denetle(self) -> None:
        """KAR-10: iki setpoint yayını arasındaki boşluğu ölç ve bağır.

        ⚠ Bu bekçi yalnız geçit AÇIKKEN anlamlıdır — geçit kapalıyken
        yayın yapmamak doğru davranıştır (disarm/GUIDED değil). Kapalıdan
        açığa geçişte de ölçüm yapılmaz: aradaki "boşluk" gerçek bir kesinti
        değil, kasıtlı sessizliktir. Bunu ayırmazsak her arm'da yanlış alarm
        basar ve bekçi güvenilirliğini kaybeder.
        """
        simdi = self._now()
        onceki = self._son_setpoint_t
        self._son_setpoint_t = simdi
        if onceki is None or self._setpoint_bosluk_s <= 0.0:
            return
        bosluk = simdi - onceki
        if bosluk <= self._setpoint_bosluk_s:
            return
        self._setpoint_bosluk_sayaci += 1
        self.get_logger().error(
            f"🔴 SETPOINT AKISINDA {bosluk:.2f}s BOSLUK (esik "
            f"{self._setpoint_bosluk_s:.2f}s, toplam "
            f"{self._setpoint_bosluk_sayaci}) — ArduPilot GUIDED'da setpoint "
            "kesilirse FAILSAFE'e duser. Kontrol dongusu butcesini asiyor "
            "olabilir (KAR-11) ya da tum yigin donmus olabilir (KAR-09).",
            throttle_duration_sec=2.0,
        )
        # Olay ZAMANI kaydedilir, arıza değil: `_ariza_durumlardan_guncelle`
        # kodu `ariza_olay_tutma_s` boyunca aktif tutar, sonra düşürür.
        self._son_setpoint_bosluk_t = simdi

    def _publish_cmd_vel(self, u: np.ndarray) -> None:
        # Diferansiyel thruster → ileri sürat + yaw rate.
        #
        # 🔴 2026-08-06 (ÖLÇÜLDÜ, GIRDAP_DURUM §0.7d): eski formül
        # `(u₀+u₁)/(2·m)` YANLIŞ boyuttaydı — kuvvet/kütle = İVME, hız değil.
        # Kapalı döngüde teknenin fiilen yaptığı hızın **onda birini** komut
        # ediyordu (σ=5/λ=10'da 0,042 ↔ 0,436 m/s = 0,10×; σ=0,36/λ=1'de
        # 0,09×). MAVROS bunu setpoint olarak FC'ye taşıdığı için araç
        # MPPI'nin istediğinden ~10× yavaş sürülüyordu.
        #
        # Doğrusu DENGE hızı: itki = sürükleme → (u₀+u₁) = |Xu|·v, yani
        # v = (u₀+u₁)/|Xu|. Aynı iki koşuda 0,93× ve 0,88× isabet.
        # Bu, dinamiğin kendi parametresinden türer (dynamics.yaml Xu, log
        # 58'den tanılandı) — ayarlanabilir sabit DEĞİL.
        #
        # 🔴 2026-08-06 (gece) — angular.z DE DÜZELTİLDİ (ÖLÇÜLDÜ, §0.9e).
        # Eski formül `(u₁−u₀)/inertia_z` iki ayrı yerden yanlıştı:
        #   (a) BOYUT: tork/atalet = açısal İVME, hız değil (linear.x'teki
        #       hatanın birebir yaw ikizi);
        #   (b) MOMENT KOLU EKSİK: diferansiyel itkinin torku (u₁−u₀)·B/2'dir,
        #       (u₁−u₀) değil.
        # Doğrusu, linear.x ile AYNI mantık — DENGE yaw hızı: tork = |Nr|·r →
        #   r = (u₁−u₀)·(B/2)/|Nr|
        # Kapalı döngü ölçümü (slalom, ~2100 yaw-aktif adım, 2 tohum):
        #   koddaki eski formül / fiili r : 1.995 · 2.013   (2× fazla komut)
        #   denge formülü      / fiili r : 0.991 · 1.000   ✅
        # Analitik oran da birebir: (1/I_z)/((B/2)/|Nr|) = 0.2/0.0993 = 2.013.
        #
        # ⚠ ÖNCEKİ KARARIN DÜZELTİLMESİ: "Nr doğrulanmadı, dokunma" notu iki
        # ayrı şeyi karıştırıyordu. Formülün BİÇİMİ parametre değerlerinden
        # bağımsız olarak yanlıştı (boyut + moment kolu) ve şimdi düzeltildi;
        # SAYININ doğruluğu hâlâ `Nr`/`inertia_z`'ye bağlı ve açık-çevrim
        # diferansiyel step testiyle teyit edilecek (GIRDAP_DURUM §18 DİĞER-4).
        # angular.z bir SETPOINT'tir — döngüyü FC kapatır; bize düşen, MPPI'nin
        # kendi modelindeki NİYETİNİ doğru çevirmek. Denge formülü tam odur.
        # ⚠ Tesadüf notu: log 58 gerçek teknenin modelden ~1,9× fazla döndüğünü
        # söylüyor; eski 2,0× hata bunu kısmen sönümlüyordu. Yani sahada fark
        # küçük görünebilir — ama iki hatanın birbirini götürmesine güvenilmez.
        p = self._pipe._dyn.p
        twist = Twist()
        twist.linear.x = float((u[0] + u[1]) / max(1e-6, abs(p.Xu)))
        twist.angular.z = float(
            (u[1] - u[0]) * (p.thruster_spacing / 2.0) / max(1e-6, abs(p.Nr))
        )
        self._pub_cmd_vel.publish(twist)
        self._cmd_vel_kadans_denetle()

    def _log_backend(self) -> None:
        """MPPI'nin fiilen çözüldüğü hesap yolunu BİR KEZ bas (bkz. çağıran).

        ⚠ MPPI ilk referans gelmeden kurulmuyor, yani açılışta henüz yok →
        kurulana kadar her kontrol adımında yeniden denenir, kurulunca yazılır
        ve mandal kapanır. Açılışta tek satır basıp geçmek, tam da görülmesi
        gereken bilgiyi kaçırırdı.
        """
        if self._backend_loglandi:
            return
        mppi = getattr(self._pipe, "_mppi", None)
        ad = getattr(mppi, "backend_adi", None) if mppi is not None else None
        if ad is None:
            return
        self._backend_loglandi = True
        if ad.startswith("numpy"):
            self.get_logger().warn(
                f"MPPI hesap yolu: {ad} — GPU YOK/BULUNAMADI. Engel sayısı "
                "arttıkça adım süresi hızla büyür (ölçüm: N=100'de 144 ms, "
                "10 Hz bütçesi 100 ms). Jetson'da bu beklenmiyor: cupy kurulumunu "
                "kontrol et (cupy-cuda12x + numpy sürüm uyumu)."
            )
            self._ariza.bildir(GPU_YOK)
        else:
            self.get_logger().info(f"MPPI hesap yolu: {ad}")

    def _cmd_vel_kadans_denetle(self) -> None:
        """🔴 ArduPilot 3 SANİYE KURALI — çıkış kadansı bekçisi (2026-08-10).

        ArduPilot Rover'ın kendi dokümanı (Rover Commands in Guided Mode):
        *"velocity commands should be re-sent every second — **the vehicle will
        stop after 3 seconds** if no command is received."* Yani `cmd_vel`
        yayınımız 3 sn kesilirse **uçuş kontrolcüsü tekneyi durdurur** ve bunu
        bize söylemez.

        Bizim en uzun GİRİŞ bekçimiz `heartbeat_timeout_s` = 5,0 s → 3-5 sn
        arasında bir tıkanmada araç çoktan durmuşken yığın hâlâ "sağlıklı" der.
        Tıkanma varsayımsal değil: `plan()` hâlâ kontrol timer'ının thread'inde
        (`rclpy.spin`, tek executor) ve sahada ölçüldü — kontrol döngüsü 10 Hz
        yerine ~5 Hz koştu (§0.12b: cmd_vel 49 → 100 mesaj/10 s).

        ⚠ **Bu bir ÖNLEME değil, GÖRÜNÜR KILMA.** Aynı executor bloklandığı
        için ayrı bir timer da ateşlenemezdi; ölçüm ancak tıkanma bittikten
        sonra, bir sonraki yayımda yapılabilir. Kaydı tutmak yeterli: koşum
        sonrası logda "FC bu aralıkta durdurmuş olabilir" penceresi görünür.
        """
        now = self._now()
        onceki = self._son_cmd_vel_t
        self._son_cmd_vel_t = now
        if onceki is None:
            return
        aralik = now - onceki
        if aralik <= 1.0:
            return
        self.get_logger().error(
            f"cmd_vel yayını {aralik:.1f} sn KESİLDİ (hedef "
            f"{1.0 / max(1e-6, float(self.get_parameter('control_rate_hz').value)):.2f} sn). "
            "ArduPilot 3 sn'de hız setpoint'i gelmezse aracı DURDURUR — bu "
            "aralıkta tekne durmuş olabilir. Sebep neredeyse kesin olarak "
            "kontrol thread'inin bloklanmasıdır (RRT* replan, §18/P1)."
        )
        # Olay ZAMANI kaydedilir (yukarıdaki `SETPOINT` gerekçesi).
        self._son_cmdvel_bosluk_t = now

    def _safe_stop(self) -> None:
        """Fail-safe motor durdurma: kontrol adımı çökerse sıfır thrust + sıfır
        cmd_vel yayınla (son komut kalıcı olmasın). Yayım da çökerse yapacak
        bir şey kalmaz — MAVROS kendi setpoint-timeout failsafe'ine düşer."""
        try:
            self._publish_thrust(np.zeros(2))
            self._publish_cmd_vel(np.zeros(2))
        except Exception:                            # yayım da çöktü — son çare
            pass

    @_guard
    def _publish_local_map(self) -> None:
        """Dosya-3: araç merkezli yerel maliyet haritası (OccupancyGrid).

        🔴 **FRAME ETİKETİ DÜZELTİLDİ (2026-08-07).** Eskiden
        `frame_id="base_link"` yazıyordu ama `local_cost_grid()` hücreleri
        **dünya ENU** ekseninde kuruyor ve araç yaw'ını HİÇ kullanmıyor —
        yani veri gövde çerçevesinde DEĞİL. Ölçümle doğrulandı: araç 0°/90°/
        180°'ye döndürüldüğünde engel haritada **aynı** hücrede kalıyor
        (base_link olsaydı sola/öne/sağa kaymalıydı). TF/RViz tüketen herkes
        engelleri ψ kadar yanlış yere koyardı.

        Doğrusu: eksenler dünya (ENU) ile hizalı, köken araçta → **`odom`**.
        ⚠️ Bu, `perception_lidar_node`'da bir kez yaşanmış hatanın aynısıydı
        (GIRDAP_DURUM §0.0b): "etiketi base_link yaz, dönüşümü yapma".
        Frame kuralı: perception topic'leri GÖVDE, mission/map topic'leri
        ENU-hizalı ÖTELEME.

        Origin (-w·res/2, -h·res/2) → araç pencere merkezinde, kuzey yukarı.
        """
        cg = self._pipe.local_cost_grid()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
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

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
from collections import deque
import threading
from typing import Callable, Optional, Tuple

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from mavros_msgs.msg import State as MavState
from mavros_msgs.msg import StatusText
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, Int32, String

#: KAR-04: `PlanningPipeline._ACTIVE_STATES` ile AYNI olmak zorunda — sebep
#: etiketi boru hattinin gercek kararini yansitmali, tahmin etmemeli.
_AKTIF_DURUMLAR = ("PARKUR1", "PARKUR2", "PARKUR3")
from vision_msgs.msg import Detection3DArray

from girdap_decision.qos_profiles import latched_qos, sensor_data_qos
from girdap_decision.saat_kaynagi import bayatlik_saati
from prototype.control.cmd_slew import EgimSinirlayici, EgimSinirlayiciConfig
from prototype.control.pivot_kapisi import (
    PivotKapisi,
    PivotKapisiConfig,
    pivot_itkisi,
)
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig
from girdap_decision.yeniden_baslama import ResetAbonesi
from prototype.mission.edge_memory import EdgeBuoyMemory
from prototype.perception.fusion import CLASS_UNKNOWN
from prototype.mission.gate_follower import (
    BUOY_RADIUS_M,
    GateFollower,
    GateFollowerConfig,
)
from prototype.planning.mppi import MPPIConfig
from prototype.mission.hedef_secim import Hedef, HedefKilidi, nisan_hedefi
from prototype.telemetry.ariza_bildirici import (
    CMDVEL_KESIK,
    ENGEL_BOS,
    GPU_YOK,
    KAPI_YOK,
    KONTROL_HATA,
    RRT_RED,
    SAAT_YOK,
    SEBEP_TUREVLI_ARIZALAR,
    SETPOINT_BOSLUK,
    SINIF_YOK,
    ArizaBildirici,
    sebepten_kodla,
)
from prototype.telemetry.saat_guveni import saat_guvenilir_mi
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


def _pipe_kilidiyle(f):
    """FAZ 5 (§1.17): `self._pipe`'a dokunan geri çağrı bu kilitle sarılır.

    Tek RLock → kilitlenme yapısal olarak imkânsız; varsayılan gruptaki
    çağrılar kendi aralarında zaten seri olduğundan tek gerçek yarış algı
    grubu ↔ kontrol grubu arasındadır ve bedeli yukarıdaki blokta ölçülü.
    """
    @functools.wraps(f)
    def _sarili(self, *a, **k):
        with self._pipe_kilidi:
            return f(self, *a, **k)
    return _sarili


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
        # F-P.9 — REPLAN FRENİ (13.08.2026). RRT* bu düğümün TEK thread'inde
        # koşar; blokladığı sürece hem `cmd_vel` susar hem düğüm KENDİ
        # aboneliklerini işleyemez (sonuç: kendi "poz bayat" bekçisi ateşler).
        # Fren sabit değil, son planın ölçülen süresine bağlıdır — ayrıntı ve
        # türetme `PlanningPipelineConfig` içinde.
        self.declare_parameter("replan_bosluk_katsayisi", 3.0)
        self.declare_parameter("replan_max_interval_s", 1.9)  # s
        # F-P.10 — RRT* AYRI SÜREÇTE. Ampirik ölçüm (bu Jetson, 10 Hz döngü,
        # CUDA'lı ebeveyn): senkron planda döngünün en kötü gecikmesi 370,7 ms,
        # asenkron kolda 1,1 ms. ⚠ Ayrı THREAD yetmez (Python GIL); işçi
        # `spawn` ile kurulur (`fork` CUDA bağlamını bozar).
        self.declare_parameter("plan_isci_enabled", True)
        self.declare_parameter("plan_isci_zaman_asimi_s", 5.0)  # s
        self.declare_parameter("mppi_K", 1000)
        self.declare_parameter("mppi_T", 50)
        self.declare_parameter("heartbeat_timeout_s", 5.0)  # MAVROS geçidi
        # F-P.1: son odom bu süreden eskiyse MPPI KOŞULMAZ (thrust sıfır).
        # fusion_node F8.2 bekçisi poz kaynağı susunca odom yayınını keser
        # ("bayat pozla plan yapılmasın") — ama planning son durumu saklayıp
        # 10 Hz sürmeye devam ediyordu → GPS/EKF kesilse bile araç KÖR sürer.
        # Eşik fusion'ın pose_timeout_s'iyle aynı mantıkta (1 s); 0 → kapalı.
        self.declare_parameter("odom_timeout_s", 1.0)
        # F-F.1 (§0.98a): pozun MAKULLÜK kapısı — `fusion_node`'dakinin ikizi.
        # Burada da var çünkü poz kaynağı tek değil: `use_isam2:=false` video
        # kolunda uçuş kontrolcüsünün pozu iletilir, sanal gölde sahte kaynak
        # yayınlar. Kapıyı yalnız üreticiye koymak, üretici değişince korumayı
        # sessizce kaybettirirdi. 0 -> kapalı.
        self.declare_parameter("poz_makul_menzil_m", 5000.0)
        # F-P.2 (robustness taraması): obstacle_map için de F-P.1 ile aynı
        # bekçi — perception_lidar_node kaynağı (Livox sürücüsü/USB) donarsa
        # son bilinen engel listesi SONSUZA DEK kullanılmasın (var olmayan
        # bir engelden kaçınmaya devam edebilir ya da gerçek bir engelin
        # gittiğini sanıp üstüne sürebilir). Topic her taramada (engel olsun
        # olmasın) publish edildiği için tazelik kontrolü güvenli. 0 → kapalı.
        self.declare_parameter("obstacle_timeout_s", 2.0)
        # 🔴 19.08.2026 — GÜVENLİK TAVANI (canlı gölde tekrarlayan "cmd_vel
        # kesildi / MPPI durdu" arızasının bulunan mekanizması). Kök neden
        # `edge_memory.py`nin kendi dondurulmuş testi (test_BUYUK_gurultu_
        # hafizayi_PATLATIYOR_belgelenmis_kisit) tarafından zaten belgeli:
        # bozuk/sıçrayan odometri (KAR-05/06) aynı cismi her karede yeni kayıt
        # sanıp hafızayı patlatıyor — GERÇEK ÇÖZÜM ORADA, bu parametre onu
        # DEĞİŞTİRMEZ. Ama patlama gerçekleştiğinde `_huni_payi`'nin O(n²)
        # taraması ve MPPI'nin (K,T+1,N) engel tensörü saniyelerce sürüyor ve
        # kontrol döngüsünü TAMAMEN durduruyor — ÖLÇÜLDÜ (laptop, tekrar
        # üretilebilir sanal göl senaryosu): N~94 kayıtta 60+ saniye kontrol
        # kilitlenmesi. Bu, kök nedeni DÜZELTMEZ — yalnız SONUCUNU sınırlar:
        # aracın en YAKIN bu kadar cismi bilmesi, çok uzaktaki/duplike
        # kayıtları hiç bilmemesinden HER ZAMAN güvenlidir (yakın olan
        # çarpışma riskidir). Sıralama _on_classified'da mesafeye göre.
        # ⚠ 150 ÖLÇÜLMÜŞ bir optimum DEĞİL — sağlıklı çalışma aralığının
        # (gözlenen 30-94 kayıt) 1,5-5 katı, yani normal koşumu HİÇ etkilemez
        # ama patlamayı (gözlenen 1362'ye kadar) küçük bir sabite sınırlar.
        # 0 = KAPALI = eski davranış birebir.
        self.declare_parameter("engel_azami_sayisi", 150)
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
        # F-F.22 İLERİ TERCİHİ — saha yüzeyi. İkisi de KAPALI varsayılan;
        # `ileri_kisit` SERT (Nav2 vx_min=0 karşılığı, garantili),
        # `w_ileri` YUMUŞAK (PreferForwardCritic karşılığı, garanti YOK).
        self.declare_parameter("mppi_ileri_kisit", _mppi.ileri_kisit)
        self.declare_parameter("mppi_w_ileri", _mppi.w_ileri)
        # F-F.27 — hedef engel içindeyse plan reddedilmesin, hedefe EN YAKIN
        # serbest noktaya planlansın (Nav2 navfn `tolerance` karşılığı).
        # 0.0 = ESKİ DAVRANIŞ. 17.08 bandında `RRT-RED` 43 kez ateşledi.
        self.declare_parameter("rrt_hedef_kurtarma_m", 0.0)
        # 🔴 19.08 — ERİŞİLEMEYEN İKİ KOL BAĞLANDI. İkisi de ayar sınıfında
        # tanımlıydı ama hiçbir parametreye/yaml'a bağlı değildi; yani ölçülmüş
        # ve işe yarayan kollar SAHADA DENENEMİYORDU (§1.60b, `04bddb7` tuzağı).
        #  · `stuck_recovery_enabled`: kendi belgesi "A/B / acil kapatma için"
        #    diyor — ama kapatmanın YOLU YOKTU.
        #  · `geri_hiz_yasak`: geri sürüşü HIZ uzayında eler, freni serbest
        #    bırakır. `mppi_ileri_kisit`in (itki uzayı) fren kaybı sorunu
        #    bunda YOK — kaptan kararı `ileri_kisit` için verildi,
        #    `geri_hiz_yasak` hiç denenemedi.
        self.declare_parameter("stuck_recovery_enabled", True)
        self.declare_parameter("mppi_geri_hiz_yasak", False)
        # F-F.28 — hedefe varan yol yoksa AĞACIN ulaştığı en yakın düğüme
        # kadar KISMİ plan üret. 0.0 = ESKİ DAVRANIŞ (düz çizgiye düş).
        # Ölçüm: 95 engelde uzay 2/5 sahnede gerçekten tıkalı ve bütçe
        # artırmak hiç işe yaramıyor (1500 ↔ 6000: 3/5 ↔ 3/5).
        self.declare_parameter("rrt_kismi_plan_min_m", 0.0)

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
        # Kenar hafızası UNUTMA MENZİLİ = yayım yarıçapı × BU KATSAYI.
        # 🔴 VARSAYILAN 2.0 = 17.08 öncesi davranış, BİT BİREBİR AYNI.
        #
        # NEDEN AYARLANABİLİR OLDU (17.08 bant ölçümü, 16.08 18:36 oturumu):
        # Katsayı 2.0 ⇒ menzil 50 m; ölçülen çalışma alanı 20×35 m (köşegen
        # ~40 m) ⇒ menzil alanın TAMAMINI kapsıyor, unutma HİÇ devreye
        # girmiyor. Gerçek `EdgeBuoyMemory` bantla beslendiğinde torba
        # **843 kayda** çıkıyor (yayımlanan `edge_buoys` yalnız 120 gösteriyor
        # — o konu yayım menzili içindekileri basıyor, tarama maliyeti ise
        # torbanın TAMAMINA göre; §1.13'ün "2404 kayıt → döngü 117→1062 ms"
        # ölçümüyle aynı sınıf).
        #
        # A/B (11.644 kare, gerçek kütüphane çağrılarak):
        #   katsayı  menzil  tepe  son   ort    unutulan  KURTARILAN
        #      2.0     50 m   843  190  328.2       750      22 928
        #      1.0     25 m   689   47  229.8     2 909      22 713   ← bedava
        #      0.5     12 m   426   45  112.1   159 815      11 749   🔴 zararlı
        # 1.0'da torba %30 küçülüyor ama `kurtarılan` yalnız %0,9 düşüyor ⇒
        # 09.08'in "duba geçici görünmez olunca hafıza kurtarsın" gerekçesi
        # korunuyor. 0.5'te kurtarma YARIYA iniyor = gerçek duba kaybı.
        #
        # ⚠️ KANITLANMAYAN: bunun kapı sıçramasını azalttığı. Deney yalnız
        # hafıza katmanını izole etti, `GateFollower` zincirde yoktu.
        # "Torba küçülür → kapı seçimi kararlı olur" HÂLÂ HİPOTEZ; bu
        # parametre tam da onu SAHADA/izole koşumda sınamak için açıldı.
        # Kabul ölçütü önceden sabitlendi (§1.30 tabanı, 15.08 16:34):
        #   `/girdap/planning/gate` >1 m atlama oranı %7,39 — bu düşmeli.
        #   Aynı katsayıyla İKİ koşu arasındaki fark = gürültü tabanı;
        #   2.0↔1.0 farkı o tabandan büyük değilse SONUÇ İLAN EDİLMEZ.
        self.declare_parameter("edge_unutma_katsayisi", 2.0)
        # Pivot itkisi yön hatasıyla ÖLÇEKLENSİN mi?
        # 🔴 VARSAYILAN False = 17.08 öncesi BANG-BANG davranışı, BİT BİREBİR.
        # Ölçüldü (16.08 183648, 92 pivot atağı): atak süresi medyan 9,5 s
        # (geometrik beklenti 2,3 s) · atakların %28'inde dönüş YÖNÜ değişiyor,
        # %14'ünde salınım — bang-bang aşımının doğrudan izi.
        # Ama aynı ölçüm dönüş hızının atak içinde 7,7→16,6 °/s ARTTIĞINI da
        # gösterdi ⇒ birincil sebep FF'in üçte bir olması. Orantılı pivot
        # İKİNCİL düzeltmedir; sahada A/B edilmeli, ölçüt PIVOT oranı (%71).
        self.declare_parameter("pivot_orantili", False)
        self.declare_parameter("pivot_taban", 0.30)
        # classified_obstacles aktığında obstacle_map'in yerine geçsin mi?
        # false → eski davranış (sınıfsız harita), kapı dubaları da engel kalır.
        self.declare_parameter("use_classified_obstacles", True)
        # Dosya-3 haritasına yalnız SINIFLANMIŞ nesneler çizilsin mi?
        # True (kaptan kararı 15.08) → CLASS_UNKNOWN=99 haritada YOK.
        # False → eski davranış (kontrol torbasının tamamı çizilir).
        # ⚠ Her iki hâlde de KONTROL yolu aynıdır; bu salt çizim ayarı.
        self.declare_parameter("harita_yalniz_siniflandirilmis", True)
        # 🔑 Kapı seçiminde AYARLANABİLİR EŞİK YOK (2026-08-03 kararı: "tahmine
        # dayalı hiçbir şey olmasın"). Geriye yalnız ÖLÇÜLMÜŞ tekne boyutları
        # kalıyor; genişlik bandı / menzil / derinlik toleransı / bırakma
        # mesafesi / eşleşme yarıçapı hepsi geometriden türetildi (bkz.
        # GateFollowerConfig). Bu ikisi tekne değişirse güncellenir, sahada
        # "deneyerek" ayarlanmaz.
        self.declare_parameter("hull_width_m", _gate.hull_width_m)
        self.declare_parameter("hull_length_m", _gate.hull_length_m)
        # Parkur-2 GN5 ötelemesi için (`_p2_hedefi_oteye_it`) — mission_manager
        # ile AYNI değer (params.yaml/hardware.yaml `mission.arrival_radius_m`,
        # varsayılan 2.0 m). Bu node mission_manager'ın parametresini doğrudan
        # okuyamaz (ayrı node); değer burada da AYNI isimle mirror'lanır —
        # `hull_width_m`/`hull_length_m` için zaten kullanılan aynı desen.
        self.declare_parameter("arrival_radius_m", 2.0)
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
            replan_bosluk_katsayisi=float(
                self.get_parameter("replan_bosluk_katsayisi").value
            ),
            replan_max_interval_s=float(
                self.get_parameter("replan_max_interval_s").value
            ),
            plan_isci_enabled=bool(
                self.get_parameter("plan_isci_enabled").value
            ),
            plan_isci_zaman_asimi_s=float(
                self.get_parameter("plan_isci_zaman_asimi_s").value
            ),
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
            mppi_ileri_kisit=bool(
                self.get_parameter("mppi_ileri_kisit").value
            ),
            mppi_w_ileri=float(self.get_parameter("mppi_w_ileri").value),
            rrt_hedef_kurtarma_m=float(
                self.get_parameter("rrt_hedef_kurtarma_m").value
            ),
            stuck_recovery_enabled=bool(
                self.get_parameter("stuck_recovery_enabled").value
            ),
            mppi_geri_hiz_yasak=bool(
                self.get_parameter("mppi_geri_hiz_yasak").value
            ),
            rrt_kismi_plan_min_m=float(
                self.get_parameter("rrt_kismi_plan_min_m").value
            ),
        )
        # F-P.9: fren "ne kadar zaman geçti" sorar → düğümün F-S.14 saati
        # (donanımda monotonic, sim zamanında `/clock`). Boru hattının kendi
        # `time.monotonic` varsayılanı sim'de bant oynatmayla ayrışırdı.
        self._pipe = PlanningPipeline(bounds, cfg, saat=self._saat)

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
        self._harita_yalniz_siniflandirilmis = bool(
            self.get_parameter("harita_yalniz_siniflandirilmis").value
        )
        self._gate = GateFollower(
            GateFollowerConfig(
                hull_width_m=float(self.get_parameter("hull_width_m").value),
                hull_length_m=float(self.get_parameter("hull_length_m").value),
            )
        )
        self._arrival_radius_m = float(
            self.get_parameter("arrival_radius_m").value
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
        # Unutma menzili katsayısı (bkz. declare_parameter'daki ölçüm tablosu).
        # 🔴 SIFIR/NEGATİF KABUL EDİLMEZ: 0 verilirse menzil 0 olur ve HER
        #    kayıt her karede silinir — hafıza fiilen kapanır, "duba geçici
        #    kaybolunca kurtar" yeteneği yok olur. Geçersiz değerde uyarı
        #    basılıp varsayılana dönülür (düğüm ÖLMEZ — saha yüzeyinde
        #    yazım hatası koşumu bitirmemeli).
        _kat = float(self.get_parameter("edge_unutma_katsayisi").value)
        if not (_kat > 0.0) or not math.isfinite(_kat):
            self.get_logger().warn(
                f"edge_unutma_katsayisi={_kat} GEÇERSİZ (>0 olmalı) — "
                f"varsayılan 2.0 kullanılıyor")
            _kat = 2.0
        self._edge_unutma_kat = _kat
        self._pivot_orantili = bool(
            self.get_parameter("pivot_orantili").value)
        _tb = float(self.get_parameter("pivot_taban").value)
        # 🔴 Taban 0,05'in altı: bırakma eşiğinde tekne fiilen DURUR ve pivot
        #    HİÇ BİTMEZ (ölçüldü: 0,035'te 0,8 °/s kalıyor). Üstü 1,0: anlamsız.
        if not (0.05 <= _tb <= 1.0) or not math.isfinite(_tb):
            self.get_logger().warn(
                f"pivot_taban={_tb} GEÇERSİZ (0,05-1,0 olmalı) — 0,30 kullanılıyor")
            _tb = 0.30
        self._pivot_taban = _tb
        if self._pivot_orantili:
            self.get_logger().info(
                f"[pivot] ORANTILI kip AÇIK · taban {self._pivot_taban:.2f} "
                f"⇒ bırakma eşiğinde ~%{100*self._pivot_taban:.0f} itki")
        self.get_logger().info(
            f"[kenar hafızası] unutma menzili = {self._harita_yaricapi:.0f} m "
            f"× {self._edge_unutma_kat:.2f} = "
            f"{self._harita_yaricapi * self._edge_unutma_kat:.0f} m")
        self._edge_mem_son_acilan = 0        # log penceresi başına yeni kayıt
        self._son_cmd_vel_t: Optional[float] = None   # çıkış kadansı bekçisi
        # Damgaya göre poz araması için kısa geçmiş: (t, x, y, psi).
        # 🔑 ÖLÇÜLDÜ 18.08 (`session_20260817_193312`, 7455 mesaj): tamponu
        # besleyen `/girdap/fusion/odom` **10,00 Hz** yayınlıyor (ortanca
        # aralık 100,0 ms), 50 Hz DEĞİL ⇒ 300 örnek = **30 saniye**.
        # Aşağıdaki "6 s @ 50 Hz" gerekçesi yazılırken kadans varsayılmıştı;
        # gerçek derinlik beşte bir değil BEŞ KATI. Aynı bantta damganın
        # tampon penceresini aşma oranı **0/5965**; kalan ıskaların tamamı
        # (%5,5) pencere darlığından değil, damganın en yeni örnekten
        # İLERİDE olmasından (ortanca 56 ms, hepsi < 1 odom periyodu).
        # ⛔ Tamponu küçültmek isteyen önce bu ölçümü tekrarlasın.
        # Eskiden 2 s (100 örnek) — ama
        # `perception_lidar_node`'un KENDİ ölçümü (09.07 tezgah, yoğun bulut)
        # clustering'in 1-3,3 s'ye çıkabildiğini gösteriyor (bu makinede
        # üretilemedi, en kötü 112 ms — ama tezgahta GERÇEKTEN ölçüldü,
        # gözardı edilemez). 2 s tampon o gecikmeyle ÇAKIŞIR: damga pencere
        # dışına düşer, `_poz_damgada` None döner, EN SON poza sessizce
        # düşülür — tam da 18.08 düzeltmesinin önlemeye çalıştığı hâl.
        #: Damgasız odom (stamp=0) sayısı — tampona YAZILMAZ. Sahada sürekli
        #: artıyorsa yayıncı damga basmıyor demektir ve poz tamponu fiilen
        #: kapalıdır (tek görünürlük kanalı bu sayaç; SSH yok).
        self._damgasiz_odom = 0
        #: Damga geriye sıçradığı için tamponun temizlendiği kez.
        self._saat_geri_gitti = 0
        self._poz_tampon: "deque[tuple[float, float, float, float]]" = deque(
            maxlen=300)
        self._damga_disi_sayaci = 0      # damga tampon dışında kaldı (teşhis)
        self._backend_loglandi = False       # MPPI hesap yolu bir kez yazılır
        self._gate_post_margin = float(
            self.get_parameter("gate_post_margin_m").value
        )
        # Kenar dubaları DÜNYA ENU'da (classified_obstacles'tan her taramada
        # tazelenir). Boş liste = kapı görünmüyor → gate_follower ham GN'ye düşer.
        self._edge_buoys: list[tuple[float, float]] = []
        # PARKUR-3: dünya çerçevesine çevrilmiş hedef adayları + istenen renk
        self._hedefler: list = []
        self._hedef_t: float = 0.0
        self._istenen_renk: int = 0
        self._hedef_kilit_bildirildi = False
        # Hedefe kilitlen ve görüntü kesilse de nişanı koru. Ölçüldü (13.08):
        # 0,3 m altında stereo ölüyor ⇒ son yarım metrede tespit KESİNLİKLE
        # kesilir; kilitsiz nişan tam temas anında ham görev noktasına düşerdi.
        self._hedef_kilidi = HedefKilidi()
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
        # F-F.1: son gelen odom makul müydü? Kapı listesinde POZ-SACMA üretir.
        self._poz_makul_menzil = float(
            self.get_parameter("poz_makul_menzil_m").value
        )
        self._poz_sacma = False
        self._poz_sacma_warn_t = 0.0
        # F-P.2: obstacle_map bayatlık takibi
        self._obstacle_timeout = float(
            self.get_parameter("obstacle_timeout_s").value
        )
        self._engel_azami_sayisi = int(
            self.get_parameter("engel_azami_sayisi").value
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

        # 🔴 FAZ 5 (15.08, GIRDAP_DURUM §1.17a-b) — İKİ İŞ PARÇACIĞI, TEK KİLİT.
        # Ölçüldü: tek iş parçacıklı `rclpy.spin`'de algı işlemesi kontrol
        # zamanlayıcısını boğuyordu (kadans↔hafıza korelasyonu r=+0,94; 317
        # "sınıflı algı gelmiyor" uyarısının %100'ü sahteydi — akış sürüyordu,
        # iş parçacığı tıkalıydı). Resmî Humble deseni uygulanıyor: ağır algı
        # geri çağrıları AYRI MutuallyExclusiveCallbackGroup'ta koşar
        # (kendi içinde seri — `_edge_memory` kilitsiz güvenli kalır), geri
        # kalan her şey düğümün varsayılan grubunda kalır; `main()`
        # MultiThreadedExecutor(num_threads=2) kurar.
        # ⚠ Paylaşılan durum kuralı: `self._pipe`'a DOKUNAN her geri çağrı
        # `_pipe_kilidiyle` sarılıdır (aşağıdaki dekoratör). Tek kilit →
        # kilitlenme (deadlock) yapısal olarak imkânsız; RLock → aynı iş
        # parçacığının iç içe çağrıları serbest. Bedeli sınırlı: algı, kontrol
        # adımının MPPI süresi kadar (~60 ms) bekleyebilir; kontrol ise artık
        # en fazla vektörleştirilmiş taramanın süresi (~6 ms, §1.18) kadar.
        # GIL notu: saf Python taramaları FAZ 5'te numpy'ye alındı; numpy/cupy
        # çekirdekleri GIL'i bıraktığı için iki grup gerçekten örtüşebiliyor.
        self._pipe_kilidi = threading.RLock()
        # Araştırma uyarısı (docs.ros.org Humble): gruba referans tutulmazsa
        # geri çağrılar HİÇ çağrılmaz — bu yüzden üye değişken.
        self._grup_algi = MutuallyExclusiveCallbackGroup()
        # 🔴 FAZ 5'İN EKSİK KALAN YARISI (18.08, Gazebo'da ölçüldü).
        # FAZ 5 ağır ALGI'yı varsayılan gruptan çıkardı ama POZ ALIMINI
        # kontrol zamanlayıcısının arkasında bıraktı. Varsayılan grup
        # MutuallyExclusive'dir: kontrol adımı (MPPI) koşarken `_on_odom`
        # SEVK EDİLMEZ — kilidi beklemez, sıraya bile alınmaz. Poz 50 Hz
        # akarken mesajlar QoS kuyruğunda (depth=10, yani 0,2 s) taşar.
        # Ölçüm (Gazebo, numpy/float64 hesap yolu, adım ~144 ms):
        #   poz yaşı 1,1-1,2 s → `POZ-BAYAT` → MPPI durur → thrust sıfır →
        #   cmd_vel akışı kesilir → ArduPilot son DÖNÜŞ komutunu tutar →
        #   tekne yerinde döner ve çıkamaz. Kontrol kilidi dağılımı bunu
        #   birebir gösterdi: 101 × `YOK|PIVOT` ↔ 101 × `POZ-BAYAT|PIVOT`.
        # Yani düğüm kendi bekçisini AÇLIKTAN tetikliyordu (§1.11'in aynısı).
        # Çözüm: durum girdileri kendi grubunda sevk edilsin. Veri yarışı YOK
        # — `_on_odom` da `_on_control_step` de aynı `_pipe_kilidiyle` sarılı;
        # poz artık en fazla TEK adım (kilit süresi) bekler, kuyrukta yaşlanmaz.
        self._grup_durum = MutuallyExclusiveCallbackGroup()
        # 🛟 KADANS BEKÇİSİ grubu (18.08). Kontrol adımı YAVAŞLADIĞINDA —
        # çökmediğinde, yalnız bütçeyi aştığında — kimse cmd_vel basmıyordu
        # ve araç SON komutu tutuyordu. Ölçüldü (Gazebo): kontrol 1,6 Hz,
        # setpoint akışında 48-178 sn boşluk, tekne son DÖNÜŞ komutuyla
        # sonsuza kadar yerinde döndü. `_safe_stop` bunu yakalamıyor çünkü o
        # yalnız kontrol adımı ÇÖKERSE çağrılıyor.
        # Bu grup bilerek AYRI ve `_pipe_kilidiyle` KULLANMAZ: kilidi alsaydı
        # MPPI'nin arkasında bekler, yani tam da koruması gereken durumda
        # susardı.
        self._grup_bekci = MutuallyExclusiveCallbackGroup()
        # 🔴 19.08.2026 — KONTROL VE HARİTA ZAMANLAYICILARI KENDİ GRUPLARINA
        # ALINDI. Eskiden ikisi de GRUPSUZ oluşturuluyordu ⇒ düğümün VARSAYILAN
        # `MutuallyExclusiveCallbackGroup`una düşüyorlardı; orada ayrıca üç
        # abonelik (`waypoints`, `targets`, `hedef_rengi`) ve arıza
        # zamanlayıcısı var. Mutually-exclusive grupta AYNI ANDA yalnız BİR
        # geri çağrı koşabilir — kaç iş parçacığı olduğu fark etmez.
        # ÖLÇÜLDÜ (§1.68b): 31,9 s boyunca odom AKMAYA DEVAM ETTİ (316 mesaj)
        # ama kontrol VE harita zamanlayıcılarının İKİSİ BİRDEN durdu; kadans
        # bekçisi (`_grup_bekci`, AYRI grup) o sırada ÇALIŞMAYA DEVAM ETTİ ve
        # boşlukları bastı (0,53 · 1,43 · 3,35 s). Yani donan şey iş parçacığı
        # ya da hesap değil (`_on_control_step` toplam %4,6 CPU), donan şey
        # GRUBUN KENDİSİ. Ayrı grupta olan her şey çalışmaya devam etti.
        # ⚠ ArduPilot GUIDED'da 3 s setpoint kesiyor ⇒ 32 s komutsuz sürüş.
        self._grup_kontrol = MutuallyExclusiveCallbackGroup()
        self._grup_harita = MutuallyExclusiveCallbackGroup()

        # --- Subscribers ---
        # 🔬 18.08 — POZ KUYRUK DERİNLİĞİ ÖLÇÜLEBİLİR ŞALTER.
        # Abonelik `depth=10` + RELIABLE ile açılıyordu. Poz 10 Hz akarken bu
        # BİR SANİYELİK birikim demek: kontrol adımı bir kez gecikirse geri
        # çağrı, sıradaki güncel pozu görmeden ÖNCE 10 eski pozu işler.
        # `_grup_durum` açlığı bitirdi ama BİRİKİMİ bitirmez — ikisi ayrı
        # kusur. ROS 2 QoS kılavuzunun "yalnız en sonu isteyen tüketici"
        # deseni `Keep Last = 1`dir (derin kuyruk + RELIABLE = komut birikmesi
        # ve gecikme). VARSAYILAN 10 = ESKİ DAVRANIŞ; 1'e çekmek ölçümle
        # gerekçelendirilmeden yapılmaz (`test_poz_kuyruk_derinligi`).
        self.declare_parameter("odom_qos_depth", 10)
        self._odom_qos_depth = max(1, int(
            self.get_parameter("odom_qos_depth").value))
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom,
            self._odom_qos_depth,
            callback_group=self._grup_durum,      # kontrol adımının arkasında BEKLEMEZ
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10,
            callback_group=self._grup_durum,
        )
        self._sub_mav_state = self.create_subscription(
            MavState, "/mavros/state", self._on_mav_state, 10,
            callback_group=self._grup_durum,
        )
        self._sub_obs = self.create_subscription(
            PoseArray, "/perception/obstacle_map", self._on_obstacles, 10,
            callback_group=self._grup_algi,       # FAZ 5: ağır algı ayrı iş parçacığı
        )
        # F-S.9 füzyon çıktısı — sınıflı engeller. Aktığı anda obstacle_map'in
        # yerine geçer; turuncu kenar dubaları buradan kapı takibine gider.
        self._sub_classified = self.create_subscription(
            Detection3DArray,
            "/perception/classified_obstacles",
            self._on_classified,
            10,
            callback_group=self._grup_algi,       # FAZ 5: ağır algı ayrı iş parçacığı
        )
        self._sub_wp = self.create_subscription(
            Path, "/girdap/mission/waypoints", self._on_waypoints, 10
        )
        # ── PARKUR-3 (FAZ 3, 2026-08-13) ────────────────────────────────
        # Hedef adayları algı ekibinin AYRI topic'inden gelir; /perception/
        # buoys'a hiç dokunulmaz ⇒ P1/P2 tanım gereği etkilenmez.
        self.create_subscription(
            Detection3DArray, "/perception/targets", self._on_targets, 10
        )
        # Hedef rengi: sahibi `KamikazeHedefKapisi` ilan eder (LATCH'li —
        # geç abone son değeri alır; VOLATILE olsaydı node sırası yüzünden
        # kaçırabilirdik, 13.08 av turu bulgusu).
        self.create_subscription(
            String, "/girdap/mission/hedef_rengi", self._on_hedef_rengi,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # Video bypass (use_rrt=false): mission_manager'dan doğrudan hedef.
        # 🔴 18.08 — VARSAYILAN GRUPTAN ALINDI (`_on_odom` ile AYNI açlık
        # sınıfı). `_on_target` yalnız video kolunda referans kurmuyor: RRT*
        # kolunda da `_pivot_yedek_hedef`i (F-F.23) besliyor, yani bir DURUM
        # girdisi. Varsayılan grup MutuallyExclusive'dir ve içinde kontrol
        # adımı (MPPI) var — 10 Hz bütçesi 100 ms, ölçülen adım ~144 ms ⇒
        # grup sürekli dolu, bu abonelik kuyrukta yaşlanıyordu.
        # ⚠ Bugünkü etkisi GİZLİ: `pivot_yedek_referans` varsayılanı false,
        # yani yedek hedefi kimse OKUMUYOR. O şalter açıldığı an (ya da
        # `use_rrt=false` video kolunda) pivot kapısı BAYAT hedefe kerteriz
        # alırdı. Şalter açılmadan önce düzeltiliyor ki tuzak kurulmasın.
        self._sub_target = self.create_subscription(
            PoseStamped, "/girdap/mission/current_target", self._on_target, 10,
            callback_group=self._grup_durum,
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
        # F-F.10 (14.08, §0.98o): LATCH'Lİ yayın. Bu topic yalnız METİN
        # DEĞİŞTİĞİNDE yayınlanır; volatile QoS'ta yayına sonradan bağlanan
        # tüketici (canlı nöbetçi · `ros2 topic echo` · **yeniden açılan
        # Mission Planner**) bir sonraki değişime kadar KÖR kalıyordu — nöbetçi
        # ilk 90 saniye "kilit: YOK" sandı, oysa araç `FSM-DISI(KILL)` idi.
        # ⚠ mission_manager'daki "latched abone bir ÖNCEKİ koşunun mesajını
        # alır" uyarısı BURAYA GEÇMEZ: orada yayıncı MAVROS'tur ve koşumlar
        # arasında yaşar; burada yayıncı bu düğümdür — düğüm yeniden doğduğunda
        # geçmişi de sıfırlanır, bayat değer taşınamaz.
        # ⚠ Geriye uyumlu: TRANSIENT_LOCAL yayıncı, VOLATILE aboneyle uyumludur
        # (yayıncının sunduğu ≥ abonenin istediği) — mevcut tüketiciler etkilenmez.
        self._pub_inhibit = self.create_publisher(
            String, "/girdap/control/inhibit_reason", latched_qos(depth=1)
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
        # F-F.18 (14.08, §0.99u): cmd_vel EĞİM SINIRLAYICI.
        # Su koşumunda ardışık komut farkı **azami 0,982 m/s** ölçüldü (10 Hz'te);
        # teknenin fiilen yaptığı hızlanma ise %99'da 0,87-0,95 m/s². Yani
        # komutun büyük kısmı takip edilemez ve düşük hız bölgesinde araç
        # **iki katı** gidiyor (+0,34 m/s) — kapıya yaklaşırken çarpma riski.
        # Varsayılan 0,8 m/s²: hem ölçülen %99'un altında hem `ATC_ACCEL_MAX`
        # (1,0) ile yarışmıyor. Açısal eksen ölçülmediği için KAPALI (0.0).
        self.declare_parameter("cmd_vel_azami_ivme_mps2", 0.8)
        self.declare_parameter("cmd_vel_azami_acisal_ivme_rps2", 0.0)
        self._egim = EgimSinirlayici(
            EgimSinirlayiciConfig(
                azami_ivme_mps2=float(
                    self.get_parameter("cmd_vel_azami_ivme_mps2").value
                ),
                azami_acisal_ivme_rps2=float(
                    self.get_parameter("cmd_vel_azami_acisal_ivme_rps2").value
                ),
            )
        )
        # F-F.20 (14.08, §1.01): PIVOT KAPISI — hedef arkadayken önce dön.
        # 14.08 ölçümü: waypoint dönüşünde 8 m ilerlemek 45,4 s ve 21,2 m yol
        # aldı (verim 0,38); komutun %36,7'si GERİ ve işaret 139 kez değişti.
        # Eşikler teknenin KENDİ uçuş kontrolcüsü ayarlarından: `WP_PIVOT_ANGLE`
        # =60 (tetik) ve ArduPilot'un "10° içinde devam et" kuralı (bırakma).
        # ⚠ Bunlar ArduPilot'ta KURULU ama GUIDED'da hız setpoint'i
        # gönderdiğimiz için uçuş kontrolcüsünün seyir mantığı devre dışı —
        # yani doğru davranışı bizim katmanımızda geri kurmak zorundayız.
        self.declare_parameter("pivot_tetik_derece", 60.0)
        self.declare_parameter("pivot_birak_derece", 10.0)
        self.declare_parameter("pivot_ufuk_m", 3.0)
        # F-F.24 — yakın alan körlüğü. 0,50 = ESKİ davranış. Ölçülen/literatür
        # değeri 1,57 m (2 × gövde 0,785; LOS "circle of acceptance" = 2 gemi
        # boyu). Sahada A/B: `planning.pivot_yakin_esik_m:=1.57`.
        self.declare_parameter("pivot_yakin_esik_m", 0.50)
        # F-F.23 — plan boşken yedek referans. False = ESKİ davranış birebir.
        self.declare_parameter("pivot_yedek_referans", False)
        self._pivot = PivotKapisi(
            PivotKapisiConfig(
                tetik_derece=float(self.get_parameter("pivot_tetik_derece").value),
                birak_derece=float(self.get_parameter("pivot_birak_derece").value),
                ufuk_m=float(self.get_parameter("pivot_ufuk_m").value),
                yakin_esik_m=float(
                    self.get_parameter("pivot_yakin_esik_m").value),
            )
        )
        self._pivot_yedek_referans = bool(
            self.get_parameter("pivot_yedek_referans").value)
        self._pivot_yedek_hedef: Optional[Tuple[float, float]] = None
        self._pivot_yedek_sayaci = 0
        self._pivot_sayaci = 0
        self._isci_uyarildi = False
        # 🔴 14.08: bekçiyi kapatmak SESSİZ olmamalı. LiDAR yokken bilerek
        # sürmek meşru bir test ihtiyacı (gate/dataset koşusu), ama biri test
        # için kapatıp unutursa yarışmaya KÖR girilir. Kapalıysa hem açılışta
        # hem her kilit raporunda görünür.
        if self._obstacle_timeout <= 0.0:
            self.get_logger().error(
                "🔴 ENGEL BEKCISI KAPALI (obstacle_timeout_s=0) — arac engel "
                "verisi OLMADAN da surer. Bu yalniz bilincli bir test icin "
                "olmali; yarisma kosusunda ACIK olmak zorunda (Parkur-2 duba "
                "kacinmasi buna bagli)."
            )
        if self._odom_timeout <= 0.0:
            self.get_logger().error(
                "🔴 POZ BEKCISI KAPALI (odom_timeout_s=0) — arac bayat/hic "
                "olmayan pozla surer."
            )
        self._isci_zaman_asimi_son = 0
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
        self._timer = self.create_timer(
            1.0 / rate, self._on_control_step,
            callback_group=self._grup_kontrol,
        )

        # --- Yerel harita yayım döngüsü (Dosya-3, ~10 Hz) ---
        map_rate = float(self.get_parameter("map_rate_hz").value)
        self._map_timer = self.create_timer(
            1.0 / map_rate, self._publish_local_map,
            callback_group=self._grup_harita,
        )

        # --- 🛟 Kadans bekçisi (18.08) ---
        # Kontrol adımı bütçeyi aşınca cmd_vel akışı kesiliyor ve ArduPilot
        # SON komutu tutuyor (dönüş komutuysa araç durmadan döner). Bu bekçi
        # sabit kadansta koşar, kontrol adımından BAĞIMSIZDIR ve akış kesilince
        # AÇIKÇA SIFIR basar — sessizlik yerine "dur" der.
        # ⚠ Eşik ArduPilot'un kendi setpoint zaman aşımından (3 sn) KÜÇÜK
        #   olmalı; 18.08 gözlemi ArduPilot'un o süre dolunca aracı
        #   durdurmadığını, dönmeye devam ettiğini gösterdi — yani bu bekçi
        #   o davranışa GÜVENMEZ.
        self.declare_parameter("setpoint_bekci_esik_s", 0.5)
        self._setpoint_bekci_esik_s = float(
            self.get_parameter("setpoint_bekci_esik_s").value
        )
        self._bekci_durus_sayaci = 0
        if self._setpoint_bekci_esik_s > 0.0:
            self._bekci_timer = self.create_timer(
                self._setpoint_bekci_esik_s / 5.0,   # eşiğin beşte biri
                self._setpoint_bekcisi,
                callback_group=self._grup_bekci,
            )

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

    # 🔴 FAZ 5 boşluğu (16.08): bu geri çağrı `self._pipe.yeniden_basla()` ve
    # `self._edge_buoys` yazıyor ama `_pipe_kilidiyle` SARILI DEĞİLDİ.
    # `ResetAbonesi` aboneliği düğümün VARSAYILAN grubunda (callback_group
    # verilmiyor), algı abonelikleri ise `_grup_algi`'da → iki ayrı
    # MutuallyExclusiveCallbackGroup, `MultiThreadedExecutor(num_threads=2)`
    # altında GERÇEKTEN aynı anda koşabilirler. Yani sıfırlama, algı taraması
    # boru hattını okurken araya girebiliyordu. Dar pencere ama pahalı an:
    # yeniden başlama hakkı yarışmada BİR kez (md 5.5.3.1).
    @_pipe_kilidiyle
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
    @_pipe_kilidiyle
    def _on_odom(self, msg: Odometry) -> None:
        """ENU pose + velocity → durum vektörü [x, y, ψ, u, v, r]."""
        self._last_odom_t = self._now()          # F-P.1: bayatlık saati
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        psi = 2.0 * math.atan2(q.z, q.w)             # z-eksen quaternion → yaw
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        # 🔴 F-F.1 (§0.98a): SAÇMA POZ MPPI DURUMUNA GİRMEZ.
        # `_last_odom_t` bilerek YUKARIDA güncellendi: mesaj GELDİ, kaynak
        # susmuş değil. Onu güncellememek POZ-BAYAT üretirdi ve operatöre
        # yanlış yeri gösterirdi ("kaynak sustu" ≠ "kaynak saçmalıyor").
        # `set_state` ATLANIR — bozuk durum MPPI'ye girerse warm-start,
        # kayan referans çapası ve kenar hafızası da kirlenir; sonraki
        # sağlıklı poz gelse bile bunlar geri gelmez.
        if not self._poz_makul(p.x, p.y, psi):
            self._poz_sacma = True
            return
        self._poz_sacma = False

        self._pipe.set_state(np.array([p.x, p.y, psi, v.x, v.y, w.z]))
        self._last_xy = (p.x, p.y)               # bypass absolute hedef için
        self._last_psi = psi                     # gövde→dünya dönüşümü için
        # 🔴 POZ TAMPONU (18.08.2026) — HAYALET DUBANIN KÖKÜ.
        # Gövde çerçevesindeki tespitler EN SON pozla dünyaya çevriliyordu;
        # tarama anı ile işleme anı arasındaki gecikme, tekne DÖNERKEN aynı
        # dubayı yay boyunca kaydırıyor ve kenar hafızası onu YENİ kayıt
        # açıyor. Kontrollü ölçüm (Gazebo, 18.08): dönerken 5 sn'de +1/+5/+2
        # yeni kayıt, dönüş kesilince ÜST ÜSTE DÖRT pencerede **+0**.
        # Standart çözüm (ROS/tf2 deseni): tespiti KENDİ DAMGASINDAKİ poza
        # göre çevir; tf2'de `lookupTransform(..., stamp)` bunu interpolasyonla
        # yapar. Burada tf2 zinciri yok (poz `/girdap/fusion/odom`'dan geliyor),
        # o yüzden kısa bir tampon tutup damgada interpolasyon yapıyoruz.
        #
        # 🔴🔴 18.08 — TAMPON ARTIK MESAJIN KENDİ DAMGASINDA (saat tabanı hatası).
        # Buraya `self._last_odom_t` yazılıyordu; o **bayatlık saati** ve
        # donanımda `time.monotonic` (`saat_kaynagi.bayatlik_saati` — kendi
        # docstring'i *"mesaj damgası yapılmaz"* diyor). Aranan anahtar ise
        # `msg.header.stamp` = ROS **duvar saati**. ÖLÇÜLDÜ (gerçek
        # `_poz_damgada` ile): tampon 4.630 sn'de, damga 1.787.051.062 sn'de —
        # **~57 yıl** arayla ⇒ `ilk_t <= t <= son_t` ASLA tutmuyor ⇒ her çağrı
        # `None` ⇒ `_damga_pozu_ya_da_son` EN SON POZA düşüyor = düzeltmenin
        # kapatmak istediği hayalet duba yolu AÇIK kalıyor.
        # 🪤 Neden fark edilmedi: ölçüm **Gazebo**'da yapıldı ve orada
        # `use_sim_time=true` ⇒ `bayatlik_saati` ROS saatine döner, tabanlar
        # ÇAKIŞIR ve düzeltme gerçekten çalışır. `hardware.launch.py`
        # varsayılanı `use_sim_time=false` ⇒ **teknede sessizce devre dışı**.
        # (Kodun kendi uyarı metni bunu zaten soruyordu: *"Yayıncı ile bizim
        # saat tabanımız aynı mı (use_sim_time)?"*)
        # ⚠ `_last_odom_t` DEĞİŞMEDİ: bayatlık ölçümü monotonic kalmalı (saat
        # adımına bağışık, F-P.1). İki görev artık iki ayrı değerde.
        # GERİ ALINIRSA: poz tamponu donanımda yine hiç eşleşmez ve dönerken
        # hayalet kenar kaydı üretilir (ölçüm: 30°/s'de 8 m'de 0,85 m kayma,
        # `edge_memory` eşleşme bandı 0,60 m).
        self._poz_tamponuna_yaz(msg.header.stamp, p.x, p.y, psi)

    def _poz_tamponuna_yaz(self, stamp, x: float, y: float, psi: float) -> None:
        """Pozu **mesajın damgasıyla** tampona yaz (bkz. `_on_odom` gerekçesi).

        İki koruma:
          · **Damga yoksa (0) YAZILMAZ.** Yazsaydık tampon iki zaman tabanını
            karıştırırdı ve interpolasyon 57 yıllık bir boşluğu geçerdi —
            sessizce uydurma poz üretirdi. Yazmamak bugünkü davranışa
            (`None` → en son poz) düşer; bu kötü ama BİLİNEN ve loglanan hâl.
          · **Saat geri giderse tampon TEMİZLENİR.** `girdap-saat-gec` fix
            gelince duvar saatini adımlayabilir (ARM kapısı var, koşu ortasında
            beklenmiyor) ve yayıncı yeniden başlayabilir; karışık tamponda
            interpolasyon iki farklı epoch arasında yapılırdı.
        """
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t <= 0.0:
            self._damgasiz_odom += 1
            return
        if self._poz_tampon and t < self._poz_tampon[-1][0]:
            self._poz_tampon.clear()
            self._saat_geri_gitti += 1
        self._poz_tampon.append((t, x, y, psi))

    def _poz_makul(self, x: float, y: float, psi: float) -> bool:
        """F-F.1: gelen poz sonlu ve makul menzilde mi (§0.98a).

        `fusion_node._poz_makul` ile aynı ölçüt; ikisi ayrı ayrı durur çünkü
        poz kaynağı değişebilir (iSAM2 / uçuş kontrolcüsü geçişi / sanal göl).
        `isfinite` menzil testinden ÖNCE gelmeli: `nan` her karşılaştırmada
        `False` döndürür, yani `hypot(nan,nan) <= menzil` testi `nan`'ı
        "makul" saymaz ama `nan` psi'yi tek başına yakalayamaz.
        """
        if self._poz_makul_menzil <= 0.0:
            return True
        if (math.isfinite(x) and math.isfinite(y) and math.isfinite(psi)
                and math.hypot(x, y) <= self._poz_makul_menzil):
            return True
        simdi = self._now()
        if simdi - self._poz_sacma_warn_t >= 5.0:
            self._poz_sacma_warn_t = simdi
            self.get_logger().error(
                f"🔴 POZ SACMA: x={x:.3e} y={y:.3e} psi={psi:.3e} "
                f"(sinir {self._poz_makul_menzil:.0f} m) — MPPI durumu "
                "GUNCELLENMEDI, thrust sifirlanacak. Fuzyon diverjansi "
                "olabilir (§0.98a); kacis: use_isam2:=false."
            )
        return False

    # ----- frame dönüşümü -----

    def _poz_damgada(self, stamp) -> Optional[tuple[float, float, float]]:
        """Verilen ROS damgasındaki (x, y, ψ) — poz tamponunda interpolasyon.

        🔴 NEDEN: gövde çerçevesindeki tespiti EN SON pozla çevirmek, araç
        DÖNERKEN dubayı yay boyunca kaydırır ve aynı fiziksel duba her turda
        YENİ kayıt olur. 18.08 kontrollü ölçümü: dönerken 5 sn'de +1/+5/+2
        yeni kayıt · dönüş kesilince dört pencere üst üste **+0**. Yani
        hayaletin kaynağı algı değil, ZAMAN HİZALAMASI.

        Dönen platformda standart çözüm budur (tf2 `lookupTransform(..., stamp)`
        aynı işi interpolasyonla yapar). Burada tf2 zinciri yok — poz
        `/girdap/fusion/odom`'dan geliyor — o yüzden tamponda arıyoruz.

        `None` dönerse çağıran EN SON poza düşer ama bunu SESSİZ yapmaz.
        Damga tampon penceresinin dışındaysa (ör. yayıncı benzetim saatinde,
        biz duvar saatindeyken) bu ayırt edilebilsin diye sayaç tutulur.
        """
        if not self._poz_tampon:
            return None
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t <= 0.0:
            return None                       # damga hiç doldurulmamış
        ilk_t = self._poz_tampon[0][0]
        son_t = self._poz_tampon[-1][0]
        if not (ilk_t <= t <= son_t):
            # Pencere dışı: ya çok bayat ya da BAŞKA ZAMAN TABANI.
            self._damga_disi_sayaci += 1
            return None
        onceki = self._poz_tampon[0]
        for ornek in self._poz_tampon:
            if ornek[0] >= t:
                t0, x0, y0, p0 = onceki
                t1, x1, y1, p1 = ornek
                if t1 <= t0:
                    return (x1, y1, p1)
                a = (t - t0) / (t1 - t0)
                # ψ dairesel: farkı sar, sonra interpolasyon yap.
                dpsi = math.atan2(math.sin(p1 - p0), math.cos(p1 - p0))
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), p0 + a * dpsi)
            onceki = ornek
        return (onceki[1], onceki[2], onceki[3])

    def _damga_pozu_ya_da_son(self, stamp, ne: str):
        """Damgadaki poz; yoksa EN SON poz + GÜRÜLTÜLÜ uyarı."""
        poz = self._poz_damgada(stamp)
        if poz is not None:
            return poz
        if self._last_xy is None:
            return None
        self.get_logger().warn(
            f"{ne}: damgada poz bulunamadı (tampon dışı/boş, toplam "
            f"{self._damga_disi_sayaci}) → EN SON poza düşüldü. Araç DÖNERKEN "
            "bu, aynı dubayı yay boyunca kaydırıp HAYALET kayıt üretir "
            "(18.08 ölçümü). Yayıncı ile bizim saat tabanımız aynı mı "
            "(use_sim_time)?",
            throttle_duration_sec=10.0,
        )
        return (self._last_xy[0], self._last_xy[1], self._last_psi)

    def _body_to_world(self, bx: float, by: float,
                       poz: Optional[tuple[float, float, float]] = None,
                       ) -> tuple[float, float]:
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
        if poz is None:
            if self._last_xy is None:
                # Poz yok: döndüremeyiz. Gövde koordinatını dünya sanmak
                # (eski davranış) sessiz bir hata olurdu — çağıran atlar.
                raise ValueError("odom yok, gövde→dünya dönüşümü yapılamaz")
            poz = (self._last_xy[0], self._last_xy[1], self._last_psi)
        px, py, ppsi = poz
        c, s = math.cos(ppsi), math.sin(ppsi)
        return (
            px + bx * c - by * s,
            py + bx * s + by * c,
        )

    @_guard
    @_pipe_kilidiyle
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
        # Dönüşüm TARAMANIN DAMGASINDAKİ poza göre (hayalet kökü, 18.08).
        _poz = self._damga_pozu_ya_da_son(msg.header.stamp, "engel haritası")
        if _poz is None:
            return
        obstacles = [
            CircleObstacle(
                *self._body_to_world(pp.position.x, pp.position.y, _poz),
                abs(pp.orientation.z))
            for pp in msg.poses
        ]
        self._obstacles_world = [(o.cx, o.cy, o.r) for o in obstacles]
        self._pipe.set_obstacles(obstacles)
        # Sınıfsız kol: burada "99" diye bir kavram YOK — kümelerin hiçbirinin
        # sınıfı yok, hepsi ham LiDAR. Gösterim süzgecini KALDIRIYORUZ, yoksa
        # sınıflı akış düştüğü anda Dosya-3 haritası bomboş kalırdı (kaptanın
        # istediği "99'u gösterme", "haritayı boşalt" değil).
        self._pipe.set_gosterim_engelleri(None)
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

    @_pipe_kilidiyle
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
        # Sınıflı kol da damgadaki poza göre çevrilir (aynı hayalet kökü).
        _poz_sinifli = self._damga_pozu_ya_da_son(
            msg.header.stamp, "sınıflı algı")
        if _poz_sinifli is None:
            return
        for det in msg.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None            # sayısal olmayan sınıf → engel say
            c = det.bbox.center.position
            try:
                wx, wy = self._body_to_world(c.x, c.y, _poz_sinifli)
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
                unutma_menzili=self._harita_yaricapi * self._edge_unutma_kat,
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

        # 🔴 GÜVENLİK TAVANI (bkz. `engel_azami_sayisi` declare_parameter'ı) —
        # `_huni_payi`'nin O(n²) taramasından VE MPPI'nin (K,T+1,N) engel
        # tensöründen ÖNCE, hafıza patlamışsa en YAKIN (çarpışma riski en
        # yüksek) kayıtlarla sınırla. Kapı takibi de korunur: edges ayrı
        # sınırlanır ki uzak bir engel yoğunluğu iki gerçek direği torbadan
        # atmasın.
        if self._engel_azami_sayisi > 0:
            ax, ay = self._last_xy
            if len(edges) > self._engel_azami_sayisi:
                edges.sort(key=lambda e: (e[0] - ax) ** 2 + (e[1] - ay) ** 2)
                edges = edges[: self._engel_azami_sayisi]
            if len(obstacles) > self._engel_azami_sayisi:
                obstacles.sort(
                    key=lambda o: (o.cx - ax) ** 2 + (o.cy - ay) ** 2)
                obstacles = obstacles[: self._engel_azami_sayisi]

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
        # 🔴 Dosya-3 HARİTASI: eşleşmemiş küme (CLASS_UNKNOWN=99) ÇİZİLMEZ —
        # kaptan kararı 15.08.2026 (*"canlı haritada da 99 verileri
        # olmayacak"*). Ölçüm: gerçek göl koşumunda tespitlerin %98,6'sı 99,
        # harita gri bir bulut oluyordu. ⚠ KONTROL YOLU DEĞİŞMEZ — 99'lar
        # yukarıdaki `set_obstacles` torbasında KALIR (güvenlik: füzyon
        # sözleşmesi "bilinmeyeni atma"; 99'suz kaçınma fiilen kör olurdu).
        if self._harita_yalniz_siniflandirilmis:
            self._pipe.set_gosterim_engelleri([
                CircleObstacle(wx, wy, r)
                for (wx, wy, r, cls) in tespitler
                if cls is not None and cls != CLASS_UNKNOWN
            ])
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

        `W` = bu direğin, **geçilebilir boşluk oluşturan** (≥ `min_passable_width`,
        FAZ 2) en yakın diğer kenar dubasına ölçülen mesafesi.
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
        # 🔴 FAZ 2 (15.08, GIRDAP_DURUM §1.13d): gövdenin SIĞAMAYACAĞI kadar
        # yakın bir komşu W hesabına GİRMEZ. Gerekçe kodun kendi tanımı
        # (`select_gate`: "gövde sığmıyorsa bu bir kapı değildir"): böyle bir
        # komşu ya aynı dubanın ikiz kaydıdır ya duvar parçasıdır — iki hâlde
        # de oradan GEÇİLMEYECEKtir, yani "geçmem gereken en dar boşluk" o
        # değildir. Eski hâl payı sıfırlıyordu: göl bandında ikiz kayıtlar
        # yüzünden direklerin %84,8'i PAYSIZ kaldı ve tekne direğe nişan aldı
        # (17 dar-kapı epizodu, 6'sında nişana girildi). Bu süzgeçle çarpışma
        # koruması hafıza kirliliğinden bağımsızlaşır. Kullanılan sınır yeni
        # bir eşik değil, kapı kabulünün kendi sınırı (`min_passable_width` =
        # gövde + 2r); gerçekten dar ama GEÇİLEBİLİR boşlukta pay eskisi gibi
        # kendiliğinden küçülür.
        min_gecilebilir = self._gate._cfg.min_passable_width
        # İndeksle dışla, koordinatla DEĞİL: iki tespit aynı noktaya düşerse
        # koordinat karşılaştırması ikisini birden eler ve pay tavana çıkardı.
        adaylar = [
            d
            for j, (kx, ky) in enumerate(kenarlar)
            if j != i
            and (d := math.hypot(dx - kx, dy - ky)) >= min_gecilebilir
        ]
        if not adaylar:
            # Geçilebilir boşluk oluşturan komşu yok → direk normal engel gibi
            # tam payını korur (len<2 koluyla aynı mantık).
            return self._gate_post_margin
        serbest = min(adaylar) - self._gate._cfg.hull_width_m - 2.0 * BUOY_RADIUS_M
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
            f"sınıfı güncellenen {self._edge_memory.celiskiyle_silinen}, "
            # F-A.1 teşhisi: onaya ulaşan konum sayısı ve onaysız kaldığı için
            # ENGEL olarak bırakılan turuncu kare sayısı. İkincisi yüksek,
            # birincisi sıfırsa kameranın turuncuları tek kare parlamasıdır.
            f"onaylanan kenar {self._edge_memory.onaylanan} "
            f"(onaysız turuncu kare {self._edge_memory.onay_bekleyen_kare}) "
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

    def _koridoru_besle(self, result) -> None:
        """F-S.16 (§1.51): parkur koridorunu MPPI'ye ver — PARKUR DIŞI kuvveti.

        🔴 **Neden gerekli:** `w_boundary` duvarının kutusu `_etkin_sinir()` ile
        "tekne/hedef ± 30 m" kuruluyor (F-S.17), yani 12 m'lik kenar duba
        koridorunda **hiç ateşlenemiyor**. Sanal gölde ölçüldü (§1.50): dört
        koşumun DÖRDÜ de koridordan çıktı — dalgasız koşumda bile — koşum
        başına ortalama **9 puan** (şartname s.24-25: her çıkış 6 puan).

        🔑 **Omurga neden `gecilen_kapilar` + kilitli kapı:** koridor en az iki
        kapı ister ve o iki kapının GERÇEK olması gerekir. `select_gate`'in
        kabul ettiği bütün çiftler kullanılamaz — çapraz çiftler bilerek
        serbest (F-K.3 notu: bu parkurda yumuşatıcı görev görüyorlar) ve
        onların orta noktası iki kapının ARASINDA duruyor; koridor omurgası
        olarak alınsalar sahte bir eksen çıkardı. Kilitlenmiş/geçilmiş kapılar
        ise nişanın kendisi — yani zaten doğrulanmış geometri.

        Terim tek yönlüdür (içeride bedel sıfır) ve yarı genişlikten gövde
        yarısı düşülür; iki kapı toplanmadan MPPI'de kendiliğinden susar.
        """
        omurga: list = [
            ((gx, gy), yari) for gx, gy, yari in self._gate.gecilen_kapilar
        ]
        kilitli = getattr(result, "gate", None)
        if kilitli is not None:
            omurga.append((kilitli.midpoint, 0.5 * kilitli.width))
        self._pipe.set_koridor(omurga)

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
        # ── PARKUR-3 NİŞANI (FAZ 3) — kapı mantığından ÖNCE ─────────────
        # Kapı ve hedef aynı anda anlamlı değil: P3'te kapı yok, hedef var.
        p3 = self._parkur3_nisani(coarse)
        if p3 is not None:
            return p3

        # ── PARKUR-2: SAF ENGEL-KAÇINMA (19.08 gece, saha ölçümüyle bulundu) ─
        # Şartname md 5.5.2.4: "Engel Bulunan Ortamda Nokta Takip" — kenar
        # dubaları burada P1'deki gibi kilitlenip GEÇİLECEK bir kapı değil,
        # sarı engel dubaları gibi kaçınılacak bir engel (kaptan doğrulaması).
        # `GateFollower`ın eşleştirme mantığı P1 için doğru ama burada YANLIŞ
        # model: bu parkurun kapıları (20-22 m) kamera FOV'u (69°) + LiDAR
        # menzili (25 m) ile iki direğin AYNI ANDA hiç görülemeyeceği kadar
        # geniş/yakın aralıklı — ölçüldü (parkur2_orani.py): GateFollower
        # komşu kapıların direklerini birbirine eşleyip HAYALET bir hedefe
        # kilitleniyor, tekne önündeki gerçek (kilitsiz) kapının huni payına
        # sıkışıp KALICI DURUYOR (900 sn üretim kalitesinde 2/11 kapı).
        # Kenar dubaları zaten obstacle torbasına giriyor (B2 huni,
        # `_huni_payi`, aşağıda `_on_classified`) — kapı mantığı OLMADAN, ham
        # GN'ye gidip MPPI'nin engelden kaçınmasına bırakınca ölçümde
        # KENDİLİĞİNDEN 10/11 kapı geçildi (aynı araç, GateFollower hiç
        # çağrılmadan).
        if self._pipe.mission_state == "PARKUR2":
            return self._p2_hedefi_oteye_it(coarse)

        if not self._gate_enabled or self._last_xy is None:
            return coarse
        result = self._gate.update(
            self._last_xy, coarse, self._edge_buoys, self._obstacles_world,
            gozlem_no=self._algi_no,
        )
        self._koridoru_besle(result)
        surus_hedefi = self._fallback_hedefi_sinirla(result)
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
            # Teşhis kanalı NİŞANI gösterir (kapının kimliği, RViz'de kapı
            # ortası); kontrole giden nokta ise onun ÖTESİDİR (F-K.1).
            self._publish_gate(result.target)
        else:
            self._warn_sessiz_ret()
        self._publish_gate_count()
        # 🔴 F-K.1: MPPI referansı kapının ÖTESİNE kurulur — kapı bir varış
        # noktası değil, geçilecek eşiktir. Nişan düzlemin üstünde bırakılırsa
        # referans orada biter, MPPI'nin terminal hedefi `ref[-1]`e kırpılır
        # (mppi._terminal_goal) ve araç tam kapı ortasında frenler; düzlem
        # geçilmediği için kilit de çözülmez → görev bir daha ilerlemez.
        # Sanal gölde ölçüldü: tekne (0.02, 24.95)'te kilitlendi, thrust
        # 0,13 N. Kapı yokken `surus_hedefi == target` (ham GN) — kapısız
        # davranış birebir korunur.
        return surus_hedefi

    def _p2_hedefi_oteye_it(self, coarse: tuple[float, float]) -> tuple[float, float]:
        """🔴 19.08 (aynı gece, yarış öncesi) — GN5 VARIŞ NOKTASI DEĞİL,
        GEÇİLECEK SON KAPININ EŞİĞİDİR (F-K.1'in Parkur-2 karşılığı).

        Şartname md 5.5.2.4: Parkur-2 tamamlama şartı GN5'e "varmak" değil,
        **son duba ikilisinden GEÇEREK** varmaktır. Ölçüldü
        (`parkur2_orani.py`): ham GN5'e doğrudan hedeflenince tekne, kapının
        içinde ama `arrival_radius_m` (varsayılan 2 m) GN5'e daha varmadan
        "ulaştı" sayılıp dwell'e giriyor — kapının tam kirişini fiilen
        GEÇMEDEN duruyor. P1'de aynı arıza `gecis_hedefi`/F-K.1 ile
        çözülmüştü ("kapı bir varış noktası değil, geçilecek eşiktir").

        Çözüm P1'le AYNI ilke, GateFollower'a dokunmadan: referans noktası
        araç→GN5 doğrultusunda `hull_length_m` (ölçülmüş gövde boyu, tahmin
        DEĞİL) kadar ÖTEYE itilir.

        ⚠ **Daha büyük öteleme DENENDİ ve GERİ ALINDI** (`arrival_radius_m +
        hull_length_m`, 19.08 gece): 5 farklı başlangıçta (normal, ±3 m
        yanal, ±20° açı hatalı, üretim kalitesi K=1000/T=50) ölçüldü — daha
        büyük öteleme "normal" başlangıçta gövde payını 0,51 m'den 0,15 m'ye
        düşürdü (tekne engelin daha yakınından geçmeye zorlanıyor) ve genel
        sonucu İYİLEŞTİRMEDİ (bazı koşullarda hâlâ 10/11). `hull_length_m`
        TEK BAŞINA: 5 koşulun 5'inde de GN5'e ulaşıldı, 0 ÇARPMA, pay hep
        pozitif (0,15-0,51 m) — 2/5'inde son kapının tam kirişi katı
        geometrik testle doğrulanamadı ama GN5 kapının 22 m açıklığının tam
        ortasında olduğu için bu ölçüm katılığı, güvenlik açığı değil.
        Daha büyük öteleme daha az güvenli VE daha az başarılıydı; bu yüzden
        küçük (ve daha güvenli) öteleme korunuyor.
        """
        if self._last_xy is None:
            return coarse
        vx, vy = self._last_xy
        dx, dy = coarse[0] - vx, coarse[1] - vy
        mesafe = math.hypot(dx, dy)
        if mesafe < 1e-6:
            return coarse
        oteleme = self._gate._cfg.hull_length_m
        return (coarse[0] + dx / mesafe * oteleme,
                coarse[1] + dy / mesafe * oteleme)

    def _fallback_hedefi_sinirla(self, result):
        """🔴 19.08 (aynı gece) — FALLBACK'TE UZAK/ALAKASIZ GN'YE KİLİTLENME.

        `GateFollower` kapı bulamayınca (`used_fallback`) ham GN'yi olduğu
        gibi döner — bu DOĞRU (kendi işi bu, bkz. modül docstring'i). Ama
        görev rotasının kapı sayısıyla 1:1 örtüşmesi ŞART DEĞİL (hakemin
        noktası kapı ortasında olmayabilir, md 5.5.2.2) — bu yüzden ham GN
        bazen algı menzilinin ÇOK ötesinde kalabilir. `kapi_orani.py`'de
        ölçüldü: 8 kapılı bir parkurda 5 GN'li rotada tekne son kapıya
        (yarı görünür — FOV kamerayı sadece bir dubasını gösteriyor) 57 m
        ötedeki hedefe kilitlenip önündeki huninin (komşusu görünmediği için
        maksimum, 1.4 m) payına sıkıştı: 400 sn'lik koşumun **146 sn'si**
        sıfır hızda geçti. Düzeltme SONRASI aynı koşum: 37 sn (5.3× azalma),
        en uzun epizot 146→17 sn, kapı 6/8→7/8.

        Kök neden `GateFollower`'da DEĞİL (o "fallback" demekte haklı) —
        entegrasyon katmanında: menzil dışı bir noktaya kör kilitlenmek
        yerine MEVCUT YÖNDE devam etmek MPPI'ye ilerleyebileceği bir referans
        verir; engel/huni maliyeti güvenliği zaten koruyor. `_harita_yaricapi`
        YENİ bir tahmin değil — yerel maliyet haritasının (ve `EdgeBuoyMemory.
        hatirlananlar`ın) zaten kullandığı ölçülmüş algı penceresi.
        """
        hedef = result.surus_hedefi
        if not result.used_fallback:
            return hedef
        x, y = self._last_xy
        if math.hypot(hedef[0] - x, hedef[1] - y) <= self._harita_yaricapi:
            return hedef
        psi = self._last_psi
        return (
            x + self._harita_yaricapi * math.cos(psi),
            y + self._harita_yaricapi * math.sin(psi),
        )

    def _parkur3_nisani(self, coarse):
        """PARKUR-3'te görülen hedefe nişan al. Uygun değilse `None`.

        `None` dönmek "hedef yok" demek DEĞİL, "**bu çağrıda P3 yolu devrede
        değil**" demek — çağıran o zaman bugünkü davranışına (kapı takibi ya
        da ham görev noktası) düşer. Dört kapı da geçilmeden nişan değişmez:

        1. **PARKUR3'te miyiz** — P1/P2'de bu yol tanım gereği kapalı.
        2. **Hedef rengi yüklü mü** — hakem rengi vermemişse hedefe kendi
           kendimize karar vermeyiz (yanlış hedef TS3: 100→50→**5**).
        3. **Tespit TAZE mi** — algı sussa (node öldü, kamera koptu) bayat
           konuma nişan almak, olmayan bir şeye sürmektir.
        4. **İstenen renkte, çapı makul, EN YAKIN aday var mı.**

        ⚠️ Konum, tespitin geldiği andaki değil **şu anki** pozla dünyaya
        çevrildi (`_on_targets`); 2 Hz'te 1,5 m/s'de ~0,75 m gecikme payı var.
        Aynı yaklaşım engel yolunda da kullanılıyor — tutarlı, ama temas
        anında bu payın ölçülmesi gerekiyor (**suda**).
        """
        # 🔴 2026-08-16: burada `self._mission_state` okunuyordu ama o alan HİÇ
        # atanmıyordu ⇒ her çağrıda AttributeError. `@_guard` hatayı yuttuğu
        # için node ölmüyor, ama `_refine_target` (KAPI TAKİBİ) ve `_on_target`
        # / `_on_waypoints` tamamen atlanıyordu ⇒ MPPI'ye referans HİÇ
        # kurulmuyor. P3 kodu, P1/P2 yolunu sessizce öldürüyordu.
        # Tek kaynak boru hattının kendisi (testler de `_pipe.set_mission_state`
        # ile sürüyor) — kopya alan tutulmuyor.
        parkur = self._pipe.mission_state
        taze = nisan_hedefi(
            parkur, self._istenen_renk, self._hedefler,
            self._last_xy, self._now() - self._hedef_t,
        )
        # PARKUR3 dışındaysak kilit BIRAKILIR (yeniden başlama / parkur geçişi).
        if parkur != "PARKUR3" or not self._istenen_renk:
            self._hedef_kilidi.sifirla()
            return None
        secilen = self._hedef_kilidi.guncelle(taze)
        if secilen is None:
            if self._hedef_kilit_bildirildi:
                self._hedef_kilit_bildirildi = False
                self.get_logger().warn(
                    "PARKUR-3 hedefi KAYBOLDU → ham görev noktasına dönüldü"
                )
            return None
        if not self._hedef_kilit_bildirildi:
            self._hedef_kilit_bildirildi = True
            self.get_logger().warn(
                f"PARKUR-3 HEDEFİ KİLİTLENDİ: ({secilen.x:.1f}, {secilen.y:.1f}), "
                f"renk kodu {secilen.renk_kodu}, ölçülen çap {secilen.cap_m:.2f} m "
                "— MPPI referansı buraya kuruluyor"
            )
        return (secilen.x, secilen.y)

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
    @_pipe_kilidiyle
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
    @_pipe_kilidiyle
    def _on_target(self, msg: PoseStamped) -> None:
        """Video bypass: mission_manager hedefi → düz çizgi MPPI referansı.

        current_target base_link'te araç-göreli ENU ofsetidir; absolute hedef
        için son odom pozuna eklenir (RRT* atlanır), sonra kapı ortasıyla
        rafine edilir.
        """
        if self._last_xy is None:
            return
        tx = self._last_xy[0] + msg.pose.position.x
        ty = self._last_xy[1] + msg.pose.position.y
        # 🔴 F-F.23 — PİVOT KAPISININ YEDEK REFERANSI.
        # Bu geri çağırma RRT* kolunda hemen aşağıda ERKEN DÖNER; o kolda
        # `global_path`ı RRT* kurar ve plan boşsa/bayatsa pivot kapısı
        # `referans yok` deyip SESSİZCE kapanır (`pivot_kapisi.guncelle`).
        # 17.08 ölçümü: yön hatası ortanca 130° olan GERİ komutların **%91'inde
        # pivot kapalıydı** — kapı çalışsaydı o komutlar hiç çıkmayacaktı.
        # Hedef bu yüzden İKİ KOLDA DA saklanır; yalnız pivot kapısı okur,
        # MPPI referansı bundan ETKİLENMEZ.
        # ⚠ Ham hedef saklanır (`_refine_target` UYGULANMADAN): rafine yalnız
        # kapı ortasına kaydırır, pivotun sorduğu soru ise "hangi yöne
        # döneyim" — ham hedef o soru için yeterli ve her iki kolda tanımlı.
        self._pivot_yedek_hedef = (tx, ty)
        if self._use_rrt:
            return
        tx, ty = self._refine_target((tx, ty))
        self._pipe.set_reference_direct(tx, ty)
        path = self._pipe.global_path
        if path is not None:
            self._publish_path(path)

    @_guard
    def _on_hedef_rengi(self, msg: String) -> None:
        """Hedef rengi yüklendi/temizlendi (sahibi: `KamikazeHedefKapisi`).

        Boş dize = hedef atanmamış ⇒ **tüm P3 nişanı kapalı** (P1/P2 aynen).
        """
        from prototype.mission.renk_kodu import RENK_KOD, _anahtarla
        yeni = RENK_KOD.get(_anahtarla(msg.data), 0) if msg.data.strip() else 0
        if yeni != self._istenen_renk:
            self._istenen_renk = yeni
            self.get_logger().warn(
                f"PARKUR-3 hedef rengi = {msg.data.strip() or 'ATANMAMIS'} "
                f"(kod {yeni})"
            )

    # 🔴 19.08: KİLİT EKLENDİ. Bu geri çağrı kilitsizdi ve kontrol adımıyla
    # karşılıklı dışlaması YALNIZ ikisinin aynı varsayılan grupta olmasından
    # geliyordu. Kontrol zamanlayıcısı kendi grubuna alınınca o örtük
    # koruma kalkar ⇒ dışlama artık AÇIKÇA kilitten gelir.
    @_pipe_kilidiyle
    def _on_targets(self, msg: Detection3DArray) -> None:
        """`/perception/targets` → dünya çerçevesinde hedef adayları.

        🔴 Çerçeve: algı topic'leri **GÖVDE (base_link)**, boru hattı **DÜNYA**
        çalışır ⇒ `_body_to_world` ŞART (2026-08-03'te engel yolunda tam bu
        dönüşüm eksikti ve engeller yanlış yere düşüyordu).

        ⚠️ 2026-08-16: bu metot `acc6247` birleştirmesinde SESSİZCE DÜŞTÜ —
        satır 520'deki abonelik kaldı, gövdesi gitti. Sonuç: `PlanningNode`
        kurucusunda `AttributeError` ⇒ planning_node HİÇ açılmıyor ⇒ MPPI yok,
        thrust yok. `girdap-karar` servisi boot'ta 3 kez deneyip vazgeçiyordu.
        Geri alınırsa aynı çökme döner (`test_planning_node.py` 71 test kırmızı).
        """
        hedefler = []
        # P3 hedefleri de damgadaki poza göre.
        _poz_p3 = self._damga_pozu_ya_da_son(
            msg.header.stamp, "P3 hedefleri")
        if _poz_p3 is None:
            return
        for det in msg.detections:
            if not det.results:
                continue
            try:
                kod = int(det.results[0].hypothesis.class_id)
            except (ValueError, TypeError):
                continue                       # sayısal olmayan sınıf → atla
            wx, wy = self._body_to_world(det.bbox.center.position.x,
                                         det.bbox.center.position.y, _poz_p3)
            hedefler.append(Hedef(wx, wy, kod, float(det.bbox.size.x),
                                  float(det.results[0].hypothesis.score)))
        self._hedefler = hedefler
        self._hedef_t = self._now()

    @_pipe_kilidiyle
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

    @_pipe_kilidiyle
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
            # F-P.10: ayrı süreçteki RRT* bitirdiyse yolu BURADA kur. Maliyet
            # bir kuyruk yoklaması; sonuç yoksa hiçbir şey olmaz. MPPI'den
            # ÖNCE çağrılır ki taze yol aynı turda kullanılsın.
            self._pipe.plan_sonucunu_isle()

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
            # F-F.20 — PIVOT KAPISI. Yeri BİLİNÇLİ: MPPI'den SONRA (onun
            # kararını ezer) ama bütün bekçilerden ÖNCE (bekçi sıfırı pivotu
            # her zaman ezer, yani kapı hiçbir duruşu geciktiremez).
            u = self._pivot_uygula(u)

            if gate.zero_thrust:                     # disarm / KILL → motor stop
                u = np.zeros(2)
                sebepler.append("DISARM-VEYA-KILL")
            if self._poz_sacma:                       # F-F.1: poz patladı → kör sürme
                # POZ-BAYAT'tan ÖNCE: ikisi aynı anda doğru olabilir (saçma
                # poz akarken kaynak sonra susarsa), ama operatöre gösterilecek
                # olan SEBEP budur — "bayat" onu yanlış yere bakmaya iter.
                u = np.zeros(2)
                sebepler.append("POZ-SACMA")
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

            self._plan_isci_sagligi_denetle()
            self._publish_inhibit(sebepler, gate)
            self._ariza_kilitlerden_guncelle(sebepler)
            self._ariza.temizle(KONTROL_HATA)     # bu tur çökmeden tamamlandı
            self._publish_thrust(u)
            if gate.allow_cmd_vel:                   # yalnız GUIDED + armed
                self._setpoint_akisini_denetle()
                # F-F.18 güvenlik sözleşmesi: `sebepler` doluysa bu tur bir
                # BEKÇİ duruşudur (u yukarıda sıfırlandı) — eğim sınırlayıcı
                # devre dışı, duruş ANINDA gider. Yalnız normal sürüşte yumuşat.
                self._publish_cmd_vel(u, egim_sinirla=not sebepler)
            else:
                # Geçit kapandı → sayaç sıfırlanır, yoksa bir sonraki arm'da
                # kasıtlı sessizlik "kesinti" diye raporlanır.
                self._son_setpoint_t = None
                # 🛟 Kadans bekçisi de düşer: buradaki sessizlik KASITLIDIR
                # (disarm / GUIDED dışı). Sıfırlanmazsa bekçi her turda
                # "akış kesildi" sanıp gereksiz sıfır basar ve gerçek
                # kesintiyi gürültüye boğar.
                self._son_cmd_vel_t = None
                # Sınırlayıcı da düşer: bir sonraki arm'da eski komuttan
                # rampalamak, aradaki duruşu yok sayıp araca sıçrama yaptırırdı.
                self._egim.sifirla()
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
        # Kapatılmış bekçi, kilit raporunda da görünür: operatör "sebep YOK"
        # okuyup güvende sanmasın — bekçi kapalıysa zaten sebep üretilemez.
        kapali = [ad for ad, v in (("ENGEL", self._obstacle_timeout),
                                   ("POZ", self._odom_timeout)) if v <= 0.0]
        if kapali:
            metin += "|BEKCI-KAPALI:" + "+".join(kapali)
        if not gate.allow_cmd_vel:
            metin += "|SETPOINT-KAPALI"
        # F-F.20: pivot bir KİLİT DEĞİL (itki üretiliyor, araç bilerek dönüyor)
        # — bu yüzden `sebepler`e girmez, arıza koduna çevrilmez. Ama operatör
        # için ayrımı hayati: 14.08'de araç 45 saniye yerinde salındı ve dışarıdan
        # "takıldı mı?" ile "dönüyor" birbirinden ayırt edilemedi.
        if self._pivot.aktif:
            metin += "|PIVOT"
        # 🔴 F-F.25: pivot KAPALIYKEN NEDEN kapalı olduğu da yazılır.
        # 17.08 göl bandında yön hatası ortanca 130° olan geri komutların
        # %91'inde kapı kapalıydı ve sebebi hiçbir yerde yoktu — üç ayrı
        # sebep (`REFERANS-YOK` / `COK-YAKIN` / `HATA-KUCUK`) aynı görünüyordu.
        # `HATA-KUCUK` beklenen ve sık hâl, o yüzden YAZILMAZ (§7: "bir alarm
        # her zaman yanıyorsa alarm değildir"); yalnız kapının **ölçüm
        # yapamadığı** iki hâl rapor edilir, çünkü onlar sessiz arızadır.
        elif self._pivot.son_sebep in (
            self._pivot.SEBEP_REFERANS_YOK, self._pivot.SEBEP_COK_YAKIN
        ):
            metin += "|PIVOT-OLCEMEDI:" + self._pivot.son_sebep
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
        """RRT* bu saniyede düz çizgiye düştü mü (kaptanın 'RRT reddetti').

        🔴 F-F.26: SEBEP de loglanır. 17.08 göl bandında bu arıza **43 kez**
        ateşledi ve telsize giden metin yalnız *"global plan uretilemedi"*
        diyordu — hangi ucun suçlu olduğu (hedef engel içinde mi, başlangıç
        mı, yoksa iterasyon bütçesi mi) hiçbir kayıtta yoktu. Üçünün çaresi
        farklı; ayırmayan alarm teşhis ettirmez. Telsiz metni KISA kalmak
        zorunda (statustext 50 bayt), o yüzden sebep loga yazılır.
        """
        sayac = self._pipe.duz_cizgiye_dusuldu
        yeni = sayac > self._son_duz_cizgi_sayaci
        self._ariza.ayarla(RRT_RED, aktif=yeni)
        if yeni:
            self.get_logger().error(
                f"RRT-RED #{sayac}: {self._pipe.son_rrt_sebep or 'sebep yok'} "
                "— düz çizgi referansına düşüldü. 'goal engel/sınır içinde' "
                "ise suçlu ENGEL TORBASI (17.08: torbanın %98,6'sı "
                "CLASS_UNKNOWN, yarıçaplar 7,75 m'ye kadar), 'çözüm bulamadı' "
                "ise iterasyon bütçesi.",
                throttle_duration_sec=5.0,
            )
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

        # SAAT-YOK (§0.61h): sistem saati bir referansa (GPS) göre kuruldu mu?
        # Kaptanın isteği — *"fix oldu ya da olmadı diye pixhawkta göreyim"*.
        # Kodun VARLIĞI "kurulmadı", YOKLUĞU "kuruldu" demek; `girdap-saat-gec`
        # fix gelince saati kurunca bu kod KENDİLİĞİNDEN düşer (§0.58b: arıza
        # kodu DURUMDUR, olay değil). Ölçüt teslim damgalarınınkiyle AYNI
        # (`saat_guveni`, çekirdek STA_UNSYNC) — ikisi ayrışamaz.
        # ⚠ 1 Hz'de bir `adjtimex` salt-okunur çağrısı: yetki istemez, ağ
        # istemez, saniyenin çok altında sürer.
        saat_ok, _gerekce = saat_guvenilir_mi()
        self._ariza.ayarla(SAAT_YOK, aktif=not saat_ok)

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

    @_pipe_kilidiyle
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

    def _plan_isci_sagligi_denetle(self) -> None:
        """F-P.10 asenkron planlayıcı çöktüyse ya da zaman aşıyorsa BAĞIR.

        13.08 kod incelemesi bulgusu: işçi kurulamazsa boru hattı SESSİZCE
        senkron kola dönüyordu — kontrol döngüsü yeniden 370 ms'ye kadar
        bloklanır (KAR-11/KAR-09 belirtilerinin kaynağı) ve dışarıdan hiçbir
        şey görünmezdi. Mekanizma doğruydu, görünürlük yoktu.
        """
        try:
            s = self._pipe.plan_isci_saglik()
        except Exception:                       # noqa: BLE001
            return
        if not s["acik"]:
            return                              # asenkron kol kapalı — normal
        if s["kullanilabilir"] is False and not self._isci_uyarildi:
            self._isci_uyarildi = True
            self.get_logger().error(
                "🔴 ASENKRON PLANLAYICI DUSTU — boru hatti SENKRON kola dondu. "
                "RRT* artik kontrol dongusunu blokluyor (olculen en kotu 370 ms; "
                "asenkron kolda 1,1 ms). Gorev yurur ama kontrol dongusu "
                "butcesini asar — KAR-11/KAR-09 belirtileri geri gelebilir."
            )
        if s["zaman_asimi"] > self._isci_zaman_asimi_son:
            self._isci_zaman_asimi_son = s["zaman_asimi"]
            self.get_logger().error(
                f"plan iscisi ZAMAN ASIMI (toplam {s['zaman_asimi']}) — her biri "
                "5 s bekleme + isci yeniden kurulumu demek; ust uste olursa "
                "yeniden planlama fiilen durur."
            )

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

    def _pivot_uygula(self, u: np.ndarray) -> np.ndarray:
        """F-F.20: yön hatası büyükse itkiyi SAF DÖNÜŞLE değiştir.

        Kapı kapalıysa (`pivot_tetik_derece <= 0`), referans yoksa ya da hata
        küçükse `u` **dokunulmadan** döner — yani eski davranış birebir korunur.
        """
        durum = self._pipe._state
        referans = self._pipe.global_path
        # F-F.23: plan yoksa/boşsa yedek referans (yalnız şalter açıkken).
        # `global_path` numpy dizisi olabilir → uzunluk AÇIKÇA sorulur,
        # `not referans` belirsizlik hatası verir.
        if self._pivot_yedek_referans and self._pivot_yedek_hedef is not None:
            if referans is None or len(referans) == 0:
                referans = [self._pivot_yedek_hedef]
                self._pivot_yedek_sayaci += 1
                if self._pivot_yedek_sayaci in (1, 10) or \
                        self._pivot_yedek_sayaci % 100 == 0:
                    self.get_logger().warn(
                        "PIVOT yedek referans kullanıldı (plan BOŞ) — "
                        f"{self._pivot_yedek_sayaci}. kez. Kapı bu durumda "
                        "eskiden sessizce kapanıyordu (F-F.23). Sürekli "
                        "tekrarlıyorsa asıl sorun RRT* planının boş olması.",
                        throttle_duration_sec=5.0,
                    )
        aktif, hata = self._pivot.guncelle(
            float(durum[0]), float(durum[1]), float(durum[2]),
            referans,
        )
        if not aktif or hata is None:
            return u
        self._pivot_sayaci += 1
        self.get_logger().info(
            f"PIVOT: yön hatası {math.degrees(hata):+.0f}° > "
            f"{self._pivot.config.tetik_derece:.0f}° — yerinde dönülüyor "
            f"(bırakma {self._pivot.config.birak_derece:.0f}°). "
            "İleri komut sıfırlandı; araç TAKILMADI, bilerek dönüyor (F-F.20).",
            throttle_duration_sec=2.0,
        )
        return np.asarray(
            pivot_itkisi(
                hata,
                float(self._pipe._dyn.p.max_thrust),
                orantili=self._pivot_orantili,
                taban=self._pivot_taban,
                tetik_derece=self._pivot.config.tetik_derece,
                birak_derece=self._pivot.config.birak_derece,
            ),
            dtype=float,
        )

    def _publish_cmd_vel(self, u: np.ndarray, *, egim_sinirla: bool = True) -> None:
        # 🛟 `egim_sinirla=False` → EĞİM SINIRLAYICI TAMAMEN BYPASS (F-F.18).
        # Bu bayrak bir ayar değil, GÜVENLİK SÖZLEŞMESİdir: bu node'daki bütün
        # bekçiler (`DISARM-VEYA-KILL`, `POZ-SACMA`, `POZ-BAYAT`, `ENGEL-BAYAT`,
        # kontrol adımı çökmesi) `u = zeros(2)` yazarak durur ve AYNI yayın
        # yolundan geçer. Sınırlayıcı simetrik uygulanırsa o duruşlar da rampaya
        # girer, yani sınırlayıcı deponun TÜM güvenlik kapılarını sakatlar.
        # Bekçi kaynaklı sıfır bu yüzden hiç uğramaz + sınırlayıcı sıfırlanır.
        #
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
        hedef_u = float((u[0] + u[1]) / max(1e-6, abs(p.Xu)))
        hedef_r = float(
            (u[1] - u[0]) * (p.thruster_spacing / 2.0) / max(1e-6, abs(p.Nr))
        )
        if egim_sinirla:
            hedef_u, hedef_r = self._egim.uygula(hedef_u, hedef_r, self._saat())
        else:
            self._egim.sifirla()
        twist = Twist()
        twist.linear.x = hedef_u
        twist.angular.z = hedef_r
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

    def _setpoint_bekcisi(self) -> None:
        """Kontrol adımı bütçeyi aşarsa AÇIKÇA sıfır bas — sessiz kalma.

        🔴 18.08.2026, Gazebo'da ölçüldü. Zincir şuydu:

            MPPI adımı 625 ms (bütçe 100 ms) → kontrol 1,6 Hz →
            `POZ-BAYAT` → MPPI durur → cmd_vel akışı KESİLİR →
            ArduPilot son **dönüş** komutunu tutar → tekne yerinde
            döner ve çıkamaz (kapı 0/8, 280 sn boyunca)

        Kritik ayrım: bu bir ÇÖKME değildi, yalnız yavaşlamaydı — bu yüzden
        `_safe_stop` (yalnız istisnada çağrılır) hiç devreye girmedi. Yığın
        "thrust sıfır" diyordu ama o sıfır ARACA HİÇ ULAŞMIYORDU; sıfır
        komutu yayınlamak ile hiç yayınlamamak ArduPilot için AYNI ŞEY DEĞİL.

        ⚠ Kasıtlı sessizlik (geçit kapalı: disarm / GUIDED dışı) bekçiyi
        tetiklemez — `_on_control_step` o durumda `_son_cmd_vel_t`'yi
        `None`'a çeker.
        ⚠ `_pipe_kilidiyle` YOK: kilit alınsaydı bekçi tam da koruması
        gereken anda (MPPI kilidi tutarken) bloke olurdu.
        """
        try:
            son = self._son_cmd_vel_t
            if son is None:
                return                       # akış kasıtlı kapalı
            gecen = self._now() - son
            if gecen < self._setpoint_bekci_esik_s:
                return
            self._pub_cmd_vel.publish(Twist())        # açık SIFIR
            self._bekci_durus_sayaci += 1
            self.get_logger().warn(
                f"🛟 KADANS BEKÇİSİ: cmd_vel {gecen:.2f} sn'dir yayınlanmadı "
                f"(eşik {self._setpoint_bekci_esik_s:.2f}) → AÇIK SIFIR basıldı "
                f"(toplam {self._bekci_durus_sayaci}). Kontrol döngüsü bütçeyi "
                "aşıyor; araç son komutla sürüklenmesin diye durduruldu.",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:                       # bekçi ASLA çökmemeli
            self.get_logger().error(
                f"kadans bekçisi hatası: {exc!r}", throttle_duration_sec=10.0)

    def _safe_stop(self) -> None:
        """Fail-safe motor durdurma: kontrol adımı çökerse sıfır thrust + sıfır
        cmd_vel yayınla (son komut kalıcı olmasın). Yayım da çökerse yapacak
        bir şey kalmaz — MAVROS kendi setpoint-timeout failsafe'ine düşer."""
        try:
            self._publish_thrust(np.zeros(2))
            # F-F.18: fail-safe duruşu ASLA rampalanmaz (güvenlik sözleşmesi).
            self._publish_cmd_vel(np.zeros(2), egim_sinirla=False)
        except Exception:                            # yayım da çöktü — son çare
            pass

    @_guard
    @_pipe_kilidiyle
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
    # 🔴 FAZ 5 (§1.17a): tek iş parçacıklı spin, algı taramasının kontrol
    # zamanlayıcısını boğmasına yol açıyordu (kadans 10 → 1,9 Hz, r=+0,94).
    # İki iş parçacığı yeter ve bilerek 2: varsayılan grup da algı grubu da
    # kendi içinde MutuallyExclusive — daha fazla iş parçacığı yalnız rclpy
    # yürütücü ek yükü getirir (bilinen sorun: rclpy #1223/#1452).
    # 18.08: ÜÇÜNCÜ iş parçacığı — durum girdileri (poz/mod/görev) artık kendi
    # grubunda (`_grup_durum`). İki iş parçacığıyla o grup, kontrol ya da algı
    # bitene kadar sevk sırası bulamıyordu; poz 1,1 s yaşlanıp `POZ-BAYAT`
    # tetikliyordu. Üç grup var ⇒ üç iş parçacığı. Daha fazlası yalnız rclpy
    # yürütücü ek yükü getirir (rclpy #1223/#1452). 18.08: DÖRT grup var
    # (varsayılan · algı · durum · kadans bekçisi) ⇒ dört iş parçacığı.
    #
    # 🔴 19.08 (aynı gece, BU commit'in kök nedeni) — 19.08'İN KENDİ B1
    # DÜZELTMESİ (kontrol + harita zamanlayıcılarını `_grup_kontrol` /
    # `_grup_harita`'ya ayırdı) grup sayısını DÖRTTEN ALTIYA çıkardı ama
    # bu satır GÜNCELLENMEDİ — "N grup ⇒ N iş parçacığı" kuralının kendisi
    # ÇİĞNENDİ (4 iş parçacığı, 6 grup). Sonuç: canlı gölde ve tekrar
    # üretilebilir bir laptop senaryosunda (`gol_kos_akinti.sh` + realistik
    # algı gürültüsü) kontrol döngüsü 5-70+ saniye TAMAMEN kilitlendi
    # ("cmd_vel kesildi", kullanıcının canlı gölde de bağımsız bildirdiği
    # arıza). py-spy KAYIT modunda (15 s, 50 Hz örnekleme, 870 örnek)
    # yakalandı: örneklerin **%75,5'i** `rclpy/executors.py:780
    # wait_for_ready_callbacks`'te — yani sistem HESAP YAPMIYORDU, bir
    # şeyin sevke hazır olmasını bekliyordu (MPPI'nin kendisi örneklerin
    # <%2'sinde göründü — ilk teşhisim "MPPI/engel sayısı yavaşlıyor"
    # YANLIŞTI, izole ölçüldü: K=1000/T=50 rollout tek başına ~50 ms).
    # Literatür (Polymath Robotics, "Evolution of Execution Management in
    # rclcpp"; ros2/rclpy #1159): ROS 2 Humble'ın klasik (poll tabanlı)
    # executor'ı entity sayısıyla DOĞRUSAL, MultiThreadedExecutor'da ayrıca
    # keşif fazı `wait_mutex_` ile TÜM iş parçacıkları arasında SERİLEŞİYOR
    # — yetersiz iş parçacığı sayısında bazı gruplar sevk sırası bulamıyor
    # (tam olarak 18.08'in kendi ölçtüğü desenin AYNISI, bu kez B1'in
    # eklediği iki yeni grup için). Gerçek çözüm (EventsExecutor) yalnız
    # Jazzy/Rolling'de var — bu proje Humble'a KİLİTLİ (CLAUDE.md), o yüzden
    # kullanılamaz. Ölçüldü (aynı senaryo, aynı tohum): num_threads=4 → 3/3
    # koşumda kilitlenme; num_threads=6 (kuralla birebir: 5 açık grup +
    # varsayılan) → 100+ s kesintisiz, 0 boşluk. Mutasyonla doğrulandı
    # (4'e dönünce AYNI senaryo yine kilitlendi).
    # ⚠ Kural GELECEKTE yeni bir `MutuallyExclusiveCallbackGroup()` eklenirse
    # YİNE ÇİĞNENEBİLİR — bu satırı da güncellemeyi unutma.
    executor = MultiThreadedExecutor(num_threads=6)
    try:
        rclpy.spin(node, executor=executor)
    finally:
        # F-P.10: işçi sürecini düzgünce durdur. Düğüm kurulumu yarıda
        # kaldıysa `_pipe` olmayabilir — kapanış yolu asla çökmemeli.
        pipe = getattr(node, "_pipe", None)
        if pipe is not None:
            pipe.kapat()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

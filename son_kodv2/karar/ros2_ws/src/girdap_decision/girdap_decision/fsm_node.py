"""
Girdap İDA — Görev FSM node'u (Layer 2).

Şartname referansları:
    5.5.2.2 — Parkur 1→2→3 geçişleri tamamen otonom.
    4.1     — Görev başladıktan sonra YKİ→İDA komut yasak (KILL hariç).
              Bu node sadece /girdap/mission/start ve /girdap/mission/kill
              servislerini sunar; başka komut kabul etmez.

Subscribed topics:
    /mavros/state                    mavros_msgs/State
        Pixhawk arm/mod durumu — BOOT→ARM ve ARM→BEKLEMEDE geçişi için.
        Ayrıca md 3.3.1(3) YKİ'den başlatma: BEKLEMEDE'de operatör modu
        `start_on_mode`'a (varsayılan GUIDED) ÇEVİRİNCE görev başlar
        (QGC → RFD868 → FCU → mavros; WiFi yasak olduğundan kıyıdan servis
        çağrısı mümkün değil — tek kablosuz komut yolu mod değişimi).
        Kenar tetikli: bilinen FARKLI bir moddan geçiş şart; boot'ta mod
        zaten GUIDED ise arm etmek görevi BAŞLATMAZ (iki ayrı komut ilkesi).
    /girdap/fusion/odom              nav_msgs/Odometry
        Smooth pozdan PARKUR1→PARKUR2 mesafe yakınsaması (son wp < 1.5 m).
    /mavros/imu/data                 sensor_msgs/Imu
        PARKUR3→TAMAMLANDI için ham IMU şok algılama.
    /perception/gate_passed          std_msgs/Bool
        Görev/perception kütüphanesinin duba ikilisi geçiş tespiti
        (PARKUR2→PARKUR3 tetiği). PLACEHOLDER — perception ekibi teslim edince
        topic ismi/tipi netleşecek.
    /girdap/mission/waypoint_reached Int32   (Sprint 4 parkur katmanı)
        mission_manager bir waypoint'e varınca yayınlar (index). Parkur geçiş
        logic'i (waypoint-index tabanlı) bu sinyalle ilerler.
    /girdap/parkur/impact            std_msgs/Bool   (Sprint 4 placeholder)
        Parkur-3 çarpma onayı → PARKUR_3 tamamlanır. Sprint 5'te IMU şok kanalı
        besleyecek (şimdilik dışarıdan/manuel test).

Published topics:
    /girdap/mission/state            std_msgs/String
        Mevcut FSM durumu (BOOT, ARM, ..., PARKUR1, PARKUR2, ...). planning_node
        ve telemetry_node bu kanalı dinler. (int8 alternatifi yerine String —
        planning_node sözleşmesiyle tutarlılık için.)
    /girdap/mission/last_gate_passed std_msgs/Bool
        Son duba ikilisi geçildi mi (PARKUR3+ evresi). FSM otoritesinden
        türetilir; planning_node kamikaze evresini bu sinyalle teyit eder.
    /girdap/parkur/state             std_msgs/String   (Sprint 4 parkur katmanı)
        Waypoint-index tabanlı parkur durumu (PARKUR_1/2/3/COMPLETED). Mevcut
        MissionFSM'den BAĞIMSIZ paralel katman — parkur ilerlemesini waypoint
        dizisinden türetir (Şartname: duba sayısına bağlı akış yasak).

Services:
    /girdap/mission/start            std_srvs/Trigger
        BEKLEMEDE'de iken FSM.request_start() tetikler → PARKUR1.
    /girdap/mission/kill             std_srvs/Trigger
        Yazılım kill butonu — her durumdan FSM.kill() → KILL.

Notlar:
    - FSM tick 10 Hz (görev yönetimi düşük frekans yeter; MPPI kontrolü ayrı
      50/20 Hz döngüde planning_node'da koşar).
    - Şok eşiği ve P1→P2 mesafesi parametre — saha karakterizasyonunda tune.
    - Tüm karar mantığı prototype.fsm.mission_fsm.MissionFSM'de; bu node
      yalnızca ROS 2 mesaj/servis alanlarını Observation'a bağlar.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from mavros_msgs.msg import State as MavState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation
from prototype.mission.parkur_fsm import (
    ParkurState,
    ParkurTransitionLogic,
    load_parkur_labels,
)


class FSMNode(Node):
    """MissionFSM ROS 2 sarmalayıcısı."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("fsm_node", **node_kwargs)

        # --- Parametreler ---
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("shock_threshold_g", 5.0)    # |a|/g eşiği
        self.declare_parameter("last_waypoint_xy", [0.0, 0.0])
        self.declare_parameter("p1_to_p2_dist", 1.5)        # CLAUDE.md
        # Sprint 4: parkur katmanı görev dosyası (waypoint parkur etiketleri).
        # Boş → tek parkur (video) — parkur logic PARKUR_1'de kalır, bozulmaz.
        self.declare_parameter("mission_file", "")
        # F-P.8 (robustness taraması, 2026-07-15): mission_manager_node'un
        # KENDİ mission_source'uyla AYNI değer — burada yalnız TEŞHİS için
        # okunur (bkz. _build_parkur_logic'teki fc+çoklu-parkur uyarısı).
        self.declare_parameter("mission_source", "file")
        # md 3.3.1(3): YKİ'den başlatma — bu moda GEÇİŞ görülünce start.
        # "" → tetik kapalı (başlatma yalnız /girdap/mission/start servisi).
        self.declare_parameter("start_on_mode", "GUIDED")
        # F-V.6: BEKLEMEDE'ye armed olarak girildiğinde mod ZATEN start_on_mode
        # ise (mod kenarı hiç görülmeden) görev başlasın mı?
        #   AUTO video (true): operatör QGC'de modu AUTO yapıp SONRA arm ederse
        #     ArduRover görevi başlatır ama bizde kenar oluşmaz → FSM BEKLEMEDE'de
        #     kalır → telemetry setpoint sütunlarını boş bırakır (F-V.2) → Ekran-2'nin
        #     ZORUNLU setpoint eğrileri boş çıkar (md 3.3.1.1). Burada FC zaten
        #     görevi koşuyor; FSM yalnızca GERÇEĞİ İZLER, motor sürmez (AUTO'da
        #     planning geçidi kapalı) → başlatmak güvenli VE gerekli.
        #   Yarışma GUIDED+MPPI (false, VARSAYILAN): FC zaten GUIDED'dayken arm
        #     edilirse görev KENDİLİĞİNDEN başlamamalı — MPPI motorları sürerdi.
        #     Kasıtlı mod komutu (kenar) şart kalır.
        self.declare_parameter("start_on_arm_in_mode", False)

        self._fsm = MissionFSM()
        self._fsm.P1_TO_P2_DIST = float(
            self.get_parameter("p1_to_p2_dist").value
        )
        self._obs = Observation()

        # --- Parkur geçiş katmanı (waypoint-index tabanlı, MissionFSM'den ayrı) ---
        self._parkur = self._build_parkur_logic()
        self._parkur_state_last = self._parkur.state       # geçiş log tespiti

        # Son alınan poz / mavros durumu
        self._pose_xy: Optional[Tuple[float, float]] = None
        self._mav_armed: bool = False
        self._start_mode = str(self.get_parameter("start_on_mode").value)
        self._start_on_arm_in_mode = bool(
            self.get_parameter("start_on_arm_in_mode").value
        )
        self._last_mode: str = ""        # "" = henüz mod görülmedi (kenar yok)
        # F-P.23 (2026-07-17): armed olup BEKLEMEDE'de takılı kalma bekçisi —
        # 2026-07-16 gerçek donanım testinde start_on_mode ("AUTO" video-modu
        # varsayılanı) ile araç gerçek modu (GUIDED) uyuşmadığı için FSM hiç
        # BEKLEMEDE'den çıkmadı, mission_manager hiç tetiklenmedi, current_
        # target/cmd_vel hiç yayınlanmadı — SESSİZCE, hiçbir hata/uyarı
        # basılmadan (F-V.6'nın aynısı, gerçek donanımda fark edilmeden
        # tekrarlandı). Artık armed+BEKLEMEDE X saniyeyi geçerse GÜRÜLTÜLÜ uyarı.
        self._armed_since: Optional[float] = None
        self.declare_parameter("armed_bekleme_watchdog_s", 15.0)
        self._armed_watchdog_s = float(
            self.get_parameter("armed_bekleme_watchdog_s").value
        )
        self._armed_watchdog_warned = False

        # --- Subscribers ---
        self._sub_mav = self.create_subscription(
            MavState, "/mavros/state", self._on_mav_state, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        # IMU mavros'ta BEST_EFFORT yayınlanır → sensor_data QoS ile abone ol.
        self._sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu, sensor_data_qos()
        )
        self._sub_gate = self.create_subscription(
            Bool, "/perception/gate_passed", self._on_gate_passed, 10
        )
        # Görev yöneticisi tüm waypoint'leri bitirdi → TAMAMLANDI terminal (F12.2).
        # Video senaryosu (tek parkur, kamikaze yok) buradan temiz durur.
        self._sub_complete = self.create_subscription(
            Bool, "/girdap/mission/complete", self._on_mission_complete, 10
        )
        # Sprint 4 parkur katmanı: waypoint-varış + çarpma placeholder.
        self._sub_wp_reached = self.create_subscription(
            Int32, "/girdap/mission/waypoint_reached", self._on_waypoint_reached, 10
        )
        self._sub_impact = self.create_subscription(
            Bool, "/girdap/parkur/impact", self._on_impact, 10
        )

        # --- Publishers ---
        self._pub_state = self.create_publisher(
            String, "/girdap/mission/state", 10
        )
        self._pub_last_gate = self.create_publisher(
            Bool, "/girdap/mission/last_gate_passed", 10
        )
        self._pub_parkur = self.create_publisher(
            String, "/girdap/parkur/state", 10
        )

        # --- Services ---
        self._srv_start = self.create_service(
            Trigger, "/girdap/mission/start", self._on_start_srv
        )
        self._srv_kill = self.create_service(
            Trigger, "/girdap/mission/kill", self._on_kill_srv
        )

        # --- Tick döngüsü ---
        rate = float(self.get_parameter("tick_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_tick)

        # Durum on_enter callback'leri (sahada thruster armament vb.)
        self._fsm.register(
            MissionState.KILL,
            on_enter=lambda: self.get_logger().error(
                "*** KILL — motorlar durduruluyor ***"
            ),
        )
        self._fsm.register(
            MissionState.TAMAMLANDI,
            on_enter=lambda: self.get_logger().info(
                "Görev tamamlandı, telemetri devam ediyor"
            ),
        )

        self.get_logger().info(
            f"fsm_node aktif (tick={rate} Hz, "
            f"P1→P2 eşik={self._fsm.P1_TO_P2_DIST} m, "
            f"parkur son index'leri={self._parkur.last_index_of_parkur})"
        )

    # ----- parkur katmanı kurulumu -----

    def _build_parkur_logic(self) -> ParkurTransitionLogic:
        """mission_file'dan parkur etiketlerini yükle → ParkurTransitionLogic.

        Dosya yoksa/okunamazsa boş etiketle kurulur (tek parkur davranışı,
        PARKUR_1'de kalır — video görevi bozulmaz).
        """
        path = str(self.get_parameter("mission_file").value)
        if not path:
            return ParkurTransitionLogic([])
        try:
            labels = load_parkur_labels(path)
        except Exception as exc:                    # dosya yok/format bozuk
            self.get_logger().warn(
                f"parkur etiketleri okunamadı ({path}): {exc} — tek parkur modu"
            )
            return ParkurTransitionLogic([])
        # F-P.8 (robustness taraması, 2026-07-15) — CRITICAL: mission_source
        # =fc'de gerçek waypoint sırası/sayısı QGC'nin FC'ye YÜKLEDİĞİ
        # görevden gelir (mission_manager_node), ama parkur SINIRLARI HÂLÂ bu
        # STATİK mission_file'dan okunuyor. fc'nin FC-kaynaklı waypoint'leri
        # HER ZAMAN parkur=1 alır (prototype.mission.mission_manager.
        # fc_items_to_waypoints_with_seqs — FC formatı parkur taşımaz), o
        # yüzden bu dosya BİRDEN FAZLA parkur içeriyorsa (ör. yanlışlıkla
        # competition_mission.yaml + mission_source=fc) waypoint_reached
        # index'leri iki farklı kaynaktan gelir — parkur geçişi ya hiç
        # tetiklenmez ya da yanlış index'te tetiklenir. Kod düzeyinde
        # otomatik senkronize edilemez (QGC yüklemesi elle) — en azından
        # operatörü GÜRÜLTÜLÜ uyar.
        source = str(self.get_parameter("mission_source").value).lower()
        if source == "fc" and len(set(labels)) > 1:
            self.get_logger().error(
                "mission_source=fc AMA mission_file ÇOKLU parkur içeriyor "
                f"({path}, parkurlar={sorted(set(labels))}) — FC'den yüklenen "
                "gerçek görev waypoint'leri HER ZAMAN parkur=1 sayılır "
                "(mission_manager.fc_items_to_waypoints), bu dosyanın parkur "
                "sınırlarıyla SENKRON DEĞİL. Parkur geçişleri (waypoint-index "
                "tabanlı) YANLIŞ ZAMANDA tetiklenebilir ya da HİÇ tetiklenmez "
                "— yarışma öncesi QGC görevini bu dosyayla EL İLE doğrula."
            )
        # F-P.9: ParkurTransitionLogic artık contiguous-olmayan (veri girişi
        # hatası) etiketlerde ValueError fırlatır — burada da yakalanır,
        # tek parkur güvenli moduna düşülür (node çökmesin).
        try:
            return ParkurTransitionLogic(labels)
        except ValueError as exc:
            self.get_logger().error(
                f"parkur etiketleri geçersiz ({path}): {exc} — tek parkur "
                "GÜVENLİ moduna düşüldü (görev dosyasını düzelt)"
            )
            return ParkurTransitionLogic([])

    # ----- subscriber callback'leri -----

    def _on_mav_state(self, msg: MavState) -> None:
        was_armed = self._mav_armed
        self._mav_armed = msg.armed
        if msg.armed and not was_armed:
            self._armed_since = self.get_clock().now().nanoseconds * 1e-9
            self._armed_watchdog_warned = False
        elif not msg.armed:
            self._armed_since = None
            self._armed_watchdog_warned = False
        # BOOT → ARM: mavros bağlantısı kuruldu
        self._obs.boot_ok = msg.connected
        # md 3.3.1(3): BEKLEMEDE'de operatörün mod komutu görevi başlatır.
        # F14.3 gereği auto_guided görev-öncesi GUIDED basmaz → BEKLEMEDE'de
        # görülen bu geçiş kesin operatör kaynaklıdır. Kenar şartı (_last_mode
        # dolu ve farklı) boot'ta-zaten-GUIDED durumunu dışlar.
        if (
            self._start_mode
            and msg.mode == self._start_mode
            and self._last_mode
            and self._last_mode != self._start_mode
            and self._fsm.state is MissionState.BEKLEMEDE
        ):
            self._fsm.request_start()
            self.get_logger().info(
                f"YKİ mod komutu ({self._last_mode}→{msg.mode}) — "
                f"görev başlatıldı (md 3.3.1/3)"
            )
        self._last_mode = msg.mode

    def _maybe_start_without_edge(self) -> None:
        """F-V.6: BEKLEMEDE'ye armed girildi ve mod ZATEN start_on_mode.

        Mod kenarı hiç oluşmadığı için `_on_mav_state` tetiklemez. AUTO
        videosunda operatör "önce AUTO, sonra ARM" yaparsa (QGC Start Mission
        akışı) FC görevi koşarken FSM BEKLEMEDE'de kalır → Ekran-2'nin setpoint
        eğrileri boş çıkar. Bu yol yalnız `start_on_arm_in_mode: true` iken
        açıktır (yarışma varsayılanı kapalı — bkz. parametre yorumu).
        """
        if not (self._start_on_arm_in_mode and self._start_mode):
            return
        if self._fsm.state is not MissionState.BEKLEMEDE:
            return
        if self._last_mode != self._start_mode:
            return
        self._fsm.request_start()
        self.get_logger().info(
            f"ARM + mod zaten {self._start_mode} (kenar yok) — görev "
            "başlatıldı (F-V.6; FC görevi koşuyor, FSM izliyor)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._pose_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        # PARKUR1→PARKUR2 yakınsaması: son waypoint'e anlık mesafe.
        # F12.1: [0,0] = AYARLANMAMIŞ varsayılan (odom origin = boot konumu →
        # görev başlar başlamaz dist=0 = sahte geçiş). Ayarlanmamışsa mesafe
        # HESAPLANMAZ (inf kalır); asıl tetik _on_waypoint_reached'te.
        last_wp = self.get_parameter("last_waypoint_xy").value
        if len(last_wp) == 2 and (last_wp[0], last_wp[1]) != (0.0, 0.0):
            dx = self._pose_xy[0] - last_wp[0]
            dy = self._pose_xy[1] - last_wp[1]
            self._obs.dist_to_last_wp_p1 = math.hypot(dx, dy)

    def _on_imu(self, msg: Imu) -> None:
        """F-S.8: gerçek IMU çarpma darbesi HEM üst-katman MissionFSM'i
        (_obs.shock_detected_p3) HEM waypoint-index parkur katmanını
        (ParkurTransitionLogic.confirm_impact) beslemeli.

        Önceden yalnız ilki bağlıydı; ikincisi hiç publish edilmeyen
        `/girdap/parkur/impact` placeholder'ına bağımlıydı (Sprint 5 notu) —
        gerçek yarışmada `/girdap/parkur/state` PARKUR_3'te sonsuza dek
        takılı kalırdı (MissionFSM doğru TAMAMLANDI'ya geçse bile).
        confirm_impact() idempotent (yalnız PARKUR_3'te etkili) — burada
        her darbede çağrılması güvenli.
        """
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        a_mag = math.sqrt(ax * ax + ay * ay + az * az) / 9.81
        threshold = float(self.get_parameter("shock_threshold_g").value)
        if a_mag > threshold:
            self._obs.shock_detected_p3 = True
            self._parkur.confirm_impact()
            self._emit_parkur_transition()

    def _on_gate_passed(self, msg: Bool) -> None:
        """Perception duba ikilisi geçiş tespiti → PARKUR2→PARKUR3 tetiği."""
        if msg.data:
            self._obs.last_gate_passed_p2 = True

    def _on_mission_complete(self, msg: Bool) -> None:
        """Görev yöneticisi tüm waypoint'leri bitirdi → TAMAMLANDI terminal.

        Latching: bir kez True olunca sıfırlanmaz (görev bitti, geri dönüş yok).
        """
        if msg.data:
            self._obs.mission_complete = True

    def _on_waypoint_reached(self, msg: Int32) -> None:
        """mission_manager waypoint varış sinyali → parkur geçiş logic'i.

        F12.1: parkur-1'in SON waypoint'ine varış = MissionFSM'in P1→P2
        yakınsama gözlemi (dist=0). Waypoint-index + parkur etiketi tabanlı
        (CLAUDE.md FSM ilkesi); yeni topic/çerçeve dönüşümü gerektirmez.
        """
        idx = int(msg.data)
        # BULGU 1 (Yahya, son_kod video koşul matrisi 2026-07-14): parkur-2
        # yoksa (tek parkurlu görev) bu sinyal beslenmemeli — aksi halde
        # PARKUR1→PARKUR2 sahte geçişi, mission_complete (dwell_time_s
        # gecikmeli) gelene dek birkaç saniye yanlış PARKUR2 gösterir.
        if (
            idx == self._parkur.last_index_of_parkur.get(1)
            and 2 in self._parkur.last_index_of_parkur
        ):
            self._obs.dist_to_last_wp_p1 = 0.0
        self._parkur.current_waypoint_reached(idx)
        self._emit_parkur_transition()

    def _on_impact(self, msg: Bool) -> None:
        """Parkur-3 çarpma placeholder (Sprint 5 IMU besleyecek) → COMPLETED."""
        if msg.data:
            self._parkur.confirm_impact()
            self._emit_parkur_transition()

    def _emit_parkur_transition(self) -> None:
        """Parkur değişince tek seferlik log — '[FSM] Parkur-1 tamamlandı → ...'.

        Geçiş öncesi durum daima PARKUR_1/2/3 (COMPLETED'den çıkış yok), bu
        yüzden numarası enum adının son karakteridir.
        """
        new = self._parkur.state
        if new is self._parkur_state_last:
            return
        old_no = self._parkur_state_last.value[-1]        # "1"/"2"/"3"
        if new is ParkurState.COMPLETED:
            self.get_logger().info(
                f"[FSM] Parkur-{old_no} tamamlandı → görev COMPLETED"
            )
        else:
            self.get_logger().info(
                f"[FSM] Parkur-{old_no} tamamlandı → Parkur-{new.value[-1]} başladı"
            )
        self._parkur_state_last = new

    # ----- servis callback'leri -----

    def _on_start_srv(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        if self._fsm.state is not MissionState.BEKLEMEDE:
            res.success = False
            res.message = (
                f"start sadece BEKLEMEDE'de geçerli "
                f"(şu an {self._fsm.state.value})"
            )
            return res
        self._fsm.request_start()
        res.success = True
        res.message = "görev başlatıldı"
        return res

    def _on_kill_srv(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        self._fsm.kill("YKİ kill servisi")
        res.success = True
        res.message = "kill alındı"
        return res

    # ----- tick döngüsü -----

    def _on_tick(self) -> None:
        # ARM → BEKLEMEDE: Pixhawk armed → kill switch fiziksel olarak OFF
        self._obs.kill_switch_off = self._mav_armed

        # F-P.23: armed+BEKLEMEDE'de takılı kalma bekçisi (bkz. __init__ notu).
        if (
            self._mav_armed
            and self._armed_since is not None
            and not self._armed_watchdog_warned
            and self._fsm.state is MissionState.BEKLEMEDE
        ):
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._armed_since
            if elapsed > self._armed_watchdog_s:
                self._armed_watchdog_warned = True
                self.get_logger().error(
                    f"Araç {elapsed:.0f}s'dir ARMED ama görev hâlâ BEKLEMEDE'de "
                    f"— mevcut mod='{self._last_mode}', beklenen "
                    f"start_on_mode='{self._start_mode}'. Mod eşleşmiyorsa "
                    "görev HİÇ başlamaz, current_target/cmd_vel hiç "
                    "yayınlanmaz (F-P.23 — 2026-07-16 gerçek donanım testinde "
                    "sessizce yaşanan sorunun aynısı: launch'a doğru "
                    "fsm.start_on_mode:=<gerçek mod> verildiğinden emin ol)"
                )

        new_state = self._fsm.tick(self._obs)

        # F-V.6: BEKLEMEDE'ye YENİ girildiyse ve mod zaten start_on_mode ise
        # (kenar yok) görevi burada başlat — tick sonrası, çünkü BEKLEMEDE'ye
        # geçiş bu tick'te oluyor.
        if new_state is MissionState.BEKLEMEDE:
            self._maybe_start_without_edge()
            new_state = self._fsm.state          # başladıysa PARKUR1 yayınlansın

        # Tek atış sinyalleri tüketildiğinde sıfırla
        if self._obs.last_gate_passed_p2 and new_state is MissionState.PARKUR3:
            self._obs.last_gate_passed_p2 = False
        if self._obs.shock_detected_p3 and new_state is MissionState.TAMAMLANDI:
            self._obs.shock_detected_p3 = False

        # Durum yayını
        state_msg = String()
        state_msg.data = new_state.value
        self._pub_state.publish(state_msg)

        # Son duba geçiş bayrağı (FSM otoritesinden türetilmiş)
        gate_msg = Bool()
        gate_msg.data = self._fsm.last_gate_passed
        self._pub_last_gate.publish(gate_msg)

        # Parkur katmanı durumu (waypoint-index tabanlı, MissionFSM'den ayrı)
        parkur_msg = String()
        parkur_msg.data = self._parkur.state.value
        self._pub_parkur.publish(parkur_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

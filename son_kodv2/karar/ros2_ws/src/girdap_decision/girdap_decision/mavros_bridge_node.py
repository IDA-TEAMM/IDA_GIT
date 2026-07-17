"""
Girdap İDA — MAVROS köprü node'u (Layer 2).

Pixhawk 6C ↔ mavros ↔ ROS 2 köprüsünün güvenlik/mod yöneticisi. Karar mantığı
ROS-bağımsız `prototype.control.mavros_bridge.MavrosBridge`'de; bu node yalnız
ROS 2 kablolamasını yapar (state okur, servis çağırır, KILL tetikler).

Firmware: ArduRover (ArduPilot). Mod adı `mode_name` param'ından gelir
(varsayılan GUIDED — ArduPilot mod ismi; PX4 olsaydı OFFBOARD olurdu).

Sorumluluklar (CLAUDE.md MAVROS bölümü + Şartname 4.1):
    1. Mod ayarı — GÖREV AKTİFKEN (FSM PARKUR1/2/3, F14.3) mod hedef
       (mode_name) değilse `/mavros/set_mode` çağrılır (auto_guided=false ile
       tamamen kapatılabilir). Görev öncesi ve sonrası (md 3.3.1/3 manuel
       dönüş) operatörün RC mod seçimi zorlanmaz.
    2. Arm/disarm — `/girdap/bridge/arm` ve `/girdap/bridge/disarm` (Trigger)
       servisleri `/mavros/cmd/arming` (CommandBool) çağırır. Arm bilinçli
       operatör eylemidir; node kendiliğinden arm ETMEZ.
       ArduRover PRE-ARM: EKF yakınsamamış / GPS fix yok / pusula kalibresiz
       ise arming REDDEDİLİR (result != 0). Bu durumda `arming_retry_max`
       kez `arming_retry_delay_s` aralıkla yeniden denenir; tükenince
       hata loglanır ve DURULUR — KILL TETİKLENMEZ (araç zaten disarm ve
       hareketsiz; pre-arm reddi bir görev iptali değil, başlangıç durumudur).
    3. Failsafe — daha önce arm olmuşken `armed=False` görülürse (beklenmedik
       disarm) → KILL.
    4. Heartbeat — `heartbeat_timeout_s` içinde `/mavros/state` gelmezse
       bağlantı koptu → KILL. ArduRover /mavros/state ~1 Hz yayınlar; 5 s
       (≈5 kaçan heartbeat) eşiği uygundur.
    5. RC donanım kill-switch (F-S.1) — `/mavros/rc/in` kanal
       `rc_kill_channel` (varsayılan 7, 0-indexed = RC kanal 8) eşik PWM'in
       (`rc_kill_threshold_pwm`, varsayılan 1500) altına düşerse → KILL.
       Yazılım/servis KILL yollarından bağımsız, companion computer canlı
       olmasa bile RC alıcısı üzerinden doğrudan çalışır.
    6. RC manuel-override (F-S.4) — `rc_manual_channel` (varsayılan 4,
       0-indexed = RC kanal 5) eşik PWM'in (`rc_manual_threshold_pwm`,
       varsayılan 1700) üstündeyken `_maybe_auto_guided()` GUIDED istemeyi
       bırakır — pilot RC'den manuel moda geçmek istediğinde yazılım
       kavga etmez.

KILL, `/girdap/mission/kill` (fsm_node, Trigger) çağrılarak yayılır: FSM KILL
durumuna geçer, planning_node sıfır thrust yayınlar → motorlar durur. Böylece
her topic'in tek yazma otoritesi korunur (planning thrust'ı, fsm durumu).

Subscribed:
    /mavros/state          mavros_msgs/State   (connected, armed, guided, mode)
    /girdap/mission/state  std_msgs/String     (görev-aktif geçidi, F14.3)
    /mavros/rc/in          mavros_msgs/RCIn    (RC donanım kill-switch, F-S.1)
Service client:
    /mavros/set_mode       mavros_msgs/SetMode
    /mavros/cmd/arming     mavros_msgs/CommandBool
    /girdap/mission/kill   std_srvs/Trigger
Service server:
    /girdap/bridge/arm     std_srvs/Trigger
    /girdap/bridge/disarm  std_srvs/Trigger

Not: mock modda /mavros/* servisleri yoktur; node bunları bekler ama bloklamaz
(service_is_ready kontrolü) ve çökmeden çalışır. Mock `armed=True, GUIDED`
yayınladığından ne set_mode ne KILL tetiklenir.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from std_msgs.msg import String
from std_srvs.srv import Trigger

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig


class MavrosBridgeNode(Node):
    """MAVROS güvenlik/mod köprüsü — MavrosBridge çekirdeğini sarar."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("mavros_bridge", **node_kwargs)

        # --- Parametreler ---
        self.declare_parameter("heartbeat_timeout_s", 5.0)
        self.declare_parameter("mode_name", "GUIDED")   # ArduRover mod ismi
        self.declare_parameter("monitor_rate_hz", 2.0)
        self.declare_parameter("auto_guided", True)
        # ArduRover pre-arm (EKF/GPS/pusula) reddinde yeniden deneme politikası.
        self.declare_parameter("arming_retry_max", 3)
        self.declare_parameter("arming_retry_delay_s", 2.0)
        # F-M.6: bağlantı kurulunca FC'den istenecek MAVLink akış hızı (Hz).
        # 0 → devre dışı (FC SR0_* parametreleri elle yönetiliyorsa).
        self.declare_parameter("stream_rate_hz", 10)
        # F-S.1: RC donanım kill-switch — ida_topics/control_node.py ile aynı
        # varsayılanlar (RC kanal 8, 0-indexed 7; PWM eşiği 1500).
        self.declare_parameter("rc_kill_channel", 7)
        self.declare_parameter("rc_kill_threshold_pwm", 1500)
        # F-S.4: RC manuel-override — ida_topics ile aynı varsayılanlar
        # (RC kanal 5, 0-indexed 4; PWM eşiği 1700).
        self.declare_parameter("rc_manual_channel", 4)
        self.declare_parameter("rc_manual_threshold_pwm", 1700)
        # F-P.24 (2026-07-17): 2026-07-16 gerçek donanım testinde TELEM2/FTDI
        # hattı, teknedeki telemetri radyosunun RF paraziti/girişimi
        # (muhtemelen radyo antenlerinin birbirine çok yakın olmasından)
        # ArduPilot'un dahili yönlendiricisi üzerinden bu temiz kablolu porta
        # da bulaşınca çöktü — mavros_router diagnostics'i "Remotes count"un
        # 174'e, sonra 323'e çıktığını gösteriyordu (sağlıklı hatta 1-3
        # olmalı), ama bunu ELLE `ros2 topic echo /diagnostics` ile bulmak
        # 30+ dakika sürdü. Artık bu anormallik otomatik izlenir.
        self.declare_parameter("link_remotes_warn_threshold", 5)

        cfg = MavrosBridgeConfig(
            heartbeat_timeout_s=float(
                self.get_parameter("heartbeat_timeout_s").value
            ),
            target_mode=str(self.get_parameter("mode_name").value),
            rc_kill_threshold_pwm=int(
                self.get_parameter("rc_kill_threshold_pwm").value
            ),
            rc_manual_threshold_pwm=int(
                self.get_parameter("rc_manual_threshold_pwm").value
            ),
        )
        self._bridge = MavrosBridge(cfg)
        self._rc_kill_channel = int(self.get_parameter("rc_kill_channel").value)
        self._rc_manual_channel = int(
            self.get_parameter("rc_manual_channel").value
        )
        self._auto_guided = bool(self.get_parameter("auto_guided").value)
        self._stream_rate_hz = int(self.get_parameter("stream_rate_hz").value)
        self._arm_retry_max = int(self.get_parameter("arming_retry_max").value)
        self._arm_retry_delay = float(
            self.get_parameter("arming_retry_delay_s").value
        )
        self._link_remotes_warn_threshold = int(
            self.get_parameter("link_remotes_warn_threshold").value
        )

        # Latching durumlar
        self._killed = False
        self._was_armed = False
        self._mode_req_pending = False
        # F-P.15 (robustness taraması, 2026-07-15): _mode_req_pending
        # yalnız /mavros/set_mode'un done-callback'inde (_on_mode_result)
        # temizleniyordu — future hiç sonuçlanmazsa (mavros restart/hang
        # sırasında servis çağrısı askıda kalırsa, ki EKF failsafe→HOLD
        # olayında tam bu yaşandı) bayrak SONSUZA DEK True kalır,
        # _maybe_auto_guided() bir daha ASLA GUIDED istemez — araç kalıcı
        # olarak otonomi dışında sıkışır. Zaman aşımıyla kendini kurtarır.
        self._mode_req_sent_t: float | None = None
        self._mode_req_timeout_s = 5.0
        self._arm_attempts = 0
        self._arm_retry_timer = None

        # --- Subscriber: /mavros/state RELIABLE (state kaçırılmamalı) ---
        self._sub_state = self.create_subscription(
            MavState, "/mavros/state", self._on_state, 10
        )
        # F14.3: FSM durumu görev-aktif geçidini besler (auto-GUIDED yalnız
        # PARKUR1/2/3'te). fsm_node ölürse bayrak False kalır → mod zorlanmaz;
        # görev de FSM'siz koşamayacağı için güvenli taraf budur.
        self._sub_mission = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )
        # F-S.1: RC donanım kill-switch — companion computer'dan bağımsız,
        # tek RC alıcısı üzerinden gelen fiziksel anahtar. mavros /mavros/rc/in
        # BEST_EFFORT yayınlar (bkz ida_topics CLAUDE.md notu) — qos_profiles
        # sensor_data_qos ile eşlenmezse RELIABLE varsayılanı sessizce veri
        # kaybına yol açar (hata fırlatmaz).
        self._sub_rc = self.create_subscription(
            RCIn, "/mavros/rc/in", self._on_rc_in, sensor_data_qos()
        )
        # F-P.24: mavros_router'ın kendi /diagnostics'i — "Remotes count"
        # anormalliğini (RF girişimi belirtisi) erken yakalar.
        self._sub_diag = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self._link_warn_active = False

        # --- Servis istemcileri ---
        self._cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self._cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._cli_kill = self.create_client(Trigger, "/girdap/mission/kill")
        self._cli_stream = self.create_client(
            StreamRate, "/mavros/set_stream_rate"
        )

        # --- Operatör arm/disarm servisleri ---
        self._srv_arm = self.create_service(
            Trigger, "/girdap/bridge/arm", self._on_arm_request
        )
        self._srv_disarm = self.create_service(
            Trigger, "/girdap/bridge/disarm", self._on_disarm_request
        )

        # --- Güvenlik izleme döngüsü ---
        rate = float(self.get_parameter("monitor_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_monitor)

        self.get_logger().info(
            f"mavros_bridge aktif (heartbeat={cfg.heartbeat_timeout_s}s, "
            f"hedef mod={cfg.target_mode}, auto_guided={self._auto_guided})"
        )

    # ----- zaman -----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- /mavros/state callback -----

    def _on_state(self, msg: MavState) -> None:
        self._bridge.update_state(
            self._now(), msg.connected, msg.armed, msg.guided, msg.mode
        )
        self._maybe_request_stream_rate()
        self._maybe_auto_guided()

    def _on_mission_state(self, msg: String) -> None:
        self._bridge.set_mission_state(msg.data)
        # F-M.3: operatör/YKİ kill'i fsm_node'dan geçer, bridge'in kendi
        # _trigger_kill yolu hiç çalışmaz — FSM KILL'e düştüyse FCU'yu da
        # disarm et (latch; kill servisi çağrısı FSM zaten KILL'de olduğundan
        # idempotent).
        if msg.data == "KILL" and not self._killed:
            self.get_logger().error("FSM KILL gözlendi → FCU disarm (F-M.3)")
            self._trigger_kill()
            return
        # PARKUR1'e girişte /mavros/state'i (~1 Hz) beklemeden hemen dene —
        # görev başlar başlamaz cmd_vel'in kabulü için mod hazır olsun.
        self._maybe_auto_guided()

    # ----- RC donanım kill-switch — F-S.1 -----

    def _on_rc_in(self, msg: RCIn) -> None:
        """RC donanım kill anahtarı — yazılım/servis KILL yollarından bağımsız.

        ida_topics/control_node.py'de zaten vardı (RC_KILL_CHANNEL=7,
        RC_KILL_THRESHOLD=1500); girdap_decision'da bu köprü şimdiye kadar
        yalnız heartbeat/beklenmedik-disarm/fsm-servisi KILL yollarını
        biliyordu, fiziksel anahtarı hiç izlemiyordu. Aynı tek KILL otoritesi
        (_trigger_kill, latch) korunuyor — bu yalnız bir tetikleyici daha.
        """
        if self._killed:
            return
        idx = self._rc_kill_channel
        channel_pwm = msg.channels[idx] if len(msg.channels) > idx else None
        if self._bridge.is_rc_kill_active(channel_pwm):
            self.get_logger().error(
                f"RC KILL ANAHTARI AKTİF (kanal {idx + 1}, PWM={channel_pwm}) "
                f"→ FCU disarm (F-S.1)"
            )
            self._trigger_kill()
            return
        self._on_rc_manual_check(msg)

    def _on_rc_manual_check(self, msg: RCIn) -> None:
        """F-S.4: pilot RC'den manuel istiyorsa yazılım GUIDED için kavga etmez.

        ida_topics/control_node.py'deki manual_override ile aynı güvenlik
        önceliği: kanal eşik üstündeyken `needs_mode_change()` False döner,
        `_maybe_auto_guided()` mod isteği göndermeyi bırakır — pilotun RC'den
        seçtiği mod (ör. MANUAL) yazılım tarafından geri zorlanmaz.
        """
        idx = self._rc_manual_channel
        channel_pwm = msg.channels[idx] if len(msg.channels) > idx else None
        active = self._bridge.is_rc_manual_active(channel_pwm)
        if active != self._bridge.rc_manual_override:
            self.get_logger().info(
                f"RC manuel-override {'AKTİF' if active else 'pasif'} "
                f"(kanal {idx + 1}, PWM={channel_pwm}) (F-S.4)"
            )
        self._bridge.set_rc_manual_override(active)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        """F-P.24: mavros_router'ın "Remotes count" anormalliğini izle.

        2026-07-16 gerçek donanım testinde teknedeki telemetri radyosunun RF
        paraziti/girişimi (muhtemelen radyo antenlerinin birbirine çok yakın
        olması), ArduPilot'un dahili MAVLink yönlendiricisi üzerinden temiz
        TELEM2/FTDI hattına da bulaştı — sağlıklı bir hatta 1-3 olması
        gereken "Remotes count" 174'e, sonra 323'e çıktı. Bu, `ros2 topic
        echo /diagnostics` ile ELLE bulundu (30+ dakika debug). Artık her
        `mavros_router: endpoint` diagnostic'inde otomatik kontrol edilir.
        """
        for status in msg.status:
            if "mavros_router: endpoint" not in status.name:
                continue
            remotes = None
            for kv in status.values:
                if kv.key == "Remotes count":
                    try:
                        remotes = int(kv.value)
                    except ValueError:
                        pass
                    break
            if remotes is None:
                continue
            if remotes > self._link_remotes_warn_threshold:
                if not self._link_warn_active:
                    self._link_warn_active = True
                    self.get_logger().error(
                        f"LINK ANORMALLİĞİ: {status.name} 'Remotes count'="
                        f"{remotes} (sağlıklı: 1-3) — muhtemelen telemetri "
                        "radyosunun RF paraziti/girişimi bu porta da "
                        "bulaşıyor (F-P.24). Radyo antenlerini birbirinden "
                        "uzaklaştırmayı dene, ya da bu hattı bırakıp USB-C "
                        "gibi ayrı bir bağlantıya geç"
                    )
            elif self._link_warn_active:
                self._link_warn_active = False
                self.get_logger().info(
                    f"{status.name} 'Remotes count' normale döndü "
                    f"({remotes}) — link anormalliği geçti (F-P.24)"
                )

    # ----- FC akış hızı (SR0) — F-M.6 -----

    def _maybe_request_stream_rate(self) -> None:
        """Bağlantı kenarında FC'den MAVLink akış hızı iste.

        ArduPilot taze bağlantıda SR0_* parametrelerine göre yayınlar; masada
        ölçülen ~1 Hz. Sonuçları: (a) Ekran-2 grafikleri basamaklı — md 3.3.1.1
        "görüntü net değilse BAŞARISIZ"; (b) fusion_node pose_timeout_s=1.0
        bekçisi 1 Hz akışı bayat sayıp odom'u KESER; (c) MPPI 10 Hz'te 1 Hz'lik
        pozla plan yapar (salınım → istemsiz hareket).

        REQUEST_DATA_STREAM oturumluktur — FC'nin SR0_* EEPROM parametrelerine
        YAZMAZ (Eyüp kararı: "FC paramlarına dokunmayın"). Kalıcı çözümü FC
        ekibi seçerse bu istek zararsız kalır (aynı hızı ister).
        """
        if self._stream_rate_hz <= 0:                # elle yönetim → devre dışı
            return
        if not self._bridge.should_request_stream_rate():
            return
        if not self._cli_stream.service_is_ready():
            # mavros henüz servisi açmadı — bayrak set EDİLMEZ, sonraki
            # /mavros/state mesajında (~1 Hz) yeniden denenir.
            self.get_logger().warn(
                "/mavros/set_stream_rate hazır değil — akış hızı isteği ertelendi"
            )
            return
        req = StreamRate.Request()
        req.stream_id = 0                            # STREAM_ALL
        req.message_rate = self._stream_rate_hz
        req.on_off = True
        self._bridge.note_stream_rate_requested()
        fut = self._cli_stream.call_async(req)
        fut.add_done_callback(self._on_stream_rate_result)
        self.get_logger().info(
            f"FC akış hızı isteniyor: {self._stream_rate_hz} Hz (STREAM_ALL)"
        )

    def _on_stream_rate_result(self, future) -> None:
        # StreamRate yanıtı BOŞ (mavros srv'sinde alan yok) — yalnız çağrının
        # hatasız döndüğünü doğrularız.
        try:
            future.result()
        except Exception as exc:
            self._bridge.note_stream_rate_failed()   # sonraki state'te yeniden dene
            self.get_logger().error(
                f"set_stream_rate çağrısı başarısız: {exc} — yeniden denenecek"
            )

    def _mode_req_stuck(self) -> bool:
        """F-P.15: bekleyen bir set_mode isteği timeout'u aştıysa (future
        hiç sonuçlanmadı) True — yeniden denemeye izin ver."""
        if not self._mode_req_pending or self._mode_req_sent_t is None:
            return False
        return (self._now() - self._mode_req_sent_t) > self._mode_req_timeout_s

    def _maybe_auto_guided(self) -> None:
        """Görev aktif + bağlı + mod hedeften farklıysa GUIDED iste (F14.3)."""
        if self._mode_req_stuck():
            self.get_logger().warn(
                f"/mavros/set_mode isteği {self._mode_req_timeout_s:.0f}s'dir "
                "yanıtsız — sıkışmış sayılıp yeniden denenecek (F-P.15)"
            )
            self._mode_req_pending = False
        if (
            self._auto_guided
            and self._bridge.needs_mode_change()
            and not self._mode_req_pending
        ):
            self._request_guided()

    # ----- güvenlik izleme -----

    def _on_monitor(self) -> None:
        # F-M.7: izleme ancak FC en az bir kez connected=true görüldükten sonra
        # başlar. mavros bağlanamazken de state (connected=false) basar; restart
        # port devrinde >5 sn'lik state boşluğu FC hiç görülmeden KILL
        # latch'liyordu (journal 2026-07-14 18:13). İlk bağlantı öncesi thrust'ı
        # control_gate zaten "FCU baglantisi yok" ile kesiyor.
        if not self._bridge.ever_connected or self._killed:
            return

        now = self._now()

        # 1) Heartbeat
        if not self._bridge.heartbeat_alive(now):
            dt = self._bridge.seconds_since_update(now)
            self.get_logger().error(
                f"FAILSAFE — heartbeat kaybı ({dt:.1f}s) → KILL"
            )
            self._trigger_kill()
            return

        # 2) Beklenmedik disarm (arm True→False) = failsafe. Ama KOMUTLU disarm
        #    (operatör/görev sonu) failsafe DEĞİL — F14.2, çekirdekte sınıflanır.
        armed = self._bridge.is_armed()
        if self._bridge.is_unexpected_disarm(self._was_armed, armed):
            self.get_logger().error("FAILSAFE — beklenmedik disarm → KILL")
            self._trigger_kill()
            return
        # F-M.2: _was_armed = ÖNCEKİ tick'in değeri (kenar takibi). Eski
        # `or armed` latch'i disarm kenarını her tick yeniden "görüyordu";
        # tek atımlık _expected_disarm bayrağı ilk tick'te tükendiğinden
        # kasıtlı disarm bir tick sonra sahte FAILSAFE/KILL üretiyordu
        # (masa olayı 2026-07-12 — gerçek FCU'da birebir yaşandı).
        self._was_armed = armed

    # ----- servis istemcisi yardımcıları -----

    def _request_guided(self) -> None:
        if not self._cli_mode.service_is_ready():
            self.get_logger().warn(
                "/mavros/set_mode hazır değil — GUIDED isteği ertelendi"
            )
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = self._bridge.config.target_mode
        self._mode_req_pending = True
        self._mode_req_sent_t = self._now()          # F-P.15: zaman aşımı saati
        fut = self._cli_mode.call_async(req)
        fut.add_done_callback(self._on_mode_result)

    def _on_mode_result(self, future) -> None:
        self._mode_req_pending = False
        try:
            res = future.result()
        except Exception as exc:                     # servis hata döndürdü
            self.get_logger().error(f"set_mode çağrısı başarısız: {exc}")
            return
        if res.mode_sent:
            self.get_logger().info(
                f"{self._bridge.config.target_mode} mod isteği gönderildi"
            )
        else:
            self.get_logger().warn("set_mode reddedildi (mode_sent=False)")

    # --- ARM (pre-arm reddinde retry) ---

    def _request_arm(self) -> bool:
        """ARM dizisini başlat. ArduRover pre-arm reddinde retry uygulanır.

        İlk denemeyi gönderir; servis hazır değilse False. Sonraki denemeler
        `_on_arm_result` içinden zamanlanır.
        """
        self._arm_attempts = 0
        return self._dispatch_arm()

    def _dispatch_arm(self) -> bool:
        if not self._cli_arm.service_is_ready():
            self.get_logger().warn("/mavros/cmd/arming hazır değil")
            return False
        self._arm_attempts += 1
        req = CommandBool.Request()
        req.value = True
        fut = self._cli_arm.call_async(req)
        fut.add_done_callback(self._on_arm_result)
        return True

    def _on_arm_result(self, future) -> None:
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"arming çağrısı başarısız: {exc}")
            return
        if res.success:
            self.get_logger().info(f"ARM başarılı ({self._arm_attempts}. deneme)")
            return
        # Reddedildi → ArduRover pre-arm (EKF/GPS fix/pusula) sağlanmıyor olabilir.
        if self._arm_attempts < self._arm_retry_max:
            self.get_logger().warn(
                f"ARM reddedildi (result={res.result}) — pre-arm bekleniyor, "
                f"{self._arm_retry_delay:.0f}s sonra yeniden dene "
                f"({self._arm_attempts}/{self._arm_retry_max})"
            )
            self._schedule_arm_retry()
        else:
            self.get_logger().error(
                f"ARM {self._arm_retry_max} denemede reddedildi (result="
                f"{res.result}) — pre-arm kontrolleri (EKF/GPS fix) "
                f"sağlanmıyor. Operatör müdahalesi gerekli. KILL tetiklenmez "
                f"(araç zaten disarm/hareketsiz)."
            )

    def _schedule_arm_retry(self) -> None:
        """Tek atımlık retry timer'ı kur (öncekini temizleyerek)."""
        if self._arm_retry_timer is not None:
            self.destroy_timer(self._arm_retry_timer)
        self._arm_retry_timer = self.create_timer(
            self._arm_retry_delay, self._on_arm_retry_tick
        )

    def _on_arm_retry_tick(self) -> None:
        if self._arm_retry_timer is not None:
            self.destroy_timer(self._arm_retry_timer)     # tek atım
            self._arm_retry_timer = None
        self._dispatch_arm()

    # --- DISARM (retry yok; disarm daima uygulanmalı) ---

    def _request_disarm(self) -> bool:
        if not self._cli_arm.service_is_ready():
            self.get_logger().warn("/mavros/cmd/arming hazır değil")
            return False
        # F14.2: komutlu disarm → sonraki arm→disarm gözlemi failsafe sayılmasın
        # (video güç-kesme gösteriminde sahte KILL basılmaz).
        self._bridge.note_command_disarm()
        req = CommandBool.Request()
        req.value = False
        fut = self._cli_arm.call_async(req)
        fut.add_done_callback(self._on_disarm_result)
        return True

    def _on_disarm_result(self, future) -> None:
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"disarm çağrısı başarısız: {exc}")
            return
        if res.success:
            self.get_logger().info("DISARM başarılı")
        else:
            self.get_logger().warn(f"DISARM reddedildi (result={res.result})")

    def _trigger_kill(self) -> None:
        """KILL: FCU'yu disarm et + FSM üzerinden sıfır thrust yay. Latching.

        F14.1: Önceki sürüm yalnız FSM→sıfır-thrust yapıyordu; araç ARMED kalıyor
        ve companion↔FCU hattı canlıyken bile FCU disarm edilmiyordu. Artık
        doğrudan `/mavros/cmd/arming False` çağrılır (kesin durdurma). Heartbeat/
        bağlantı kaybı senaryosunda bu komut FCU'ya ULAŞMAYABİLİR — o durumda
        koruma FCU'nun KENDİ failsafe'idir (ArduPilot GCS/throttle failsafe →
        otomatik disarm/hold), FC parametresi olarak ayrı doğrulanmalı.
        """
        self._killed = True
        # 1) FCU disarm (hat canlıysa kesin motor kesme). _killed=True olduğundan
        #    bu disarm _on_monitor'da failsafe döngüsüne girmez (erken dönüş).
        if self._cli_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = False
            self._cli_arm.call_async(req)
        # 2) Yazılım KILL'i FSM üzerinden de yay (sıfır thrust — tek otorite).
        if self._cli_kill.service_is_ready():
            self._cli_kill.call_async(Trigger.Request())
        else:
            self.get_logger().error(
                "/girdap/mission/kill hazır değil — motorlar bağımsız "
                "kesilmeli (RC/YKİ kill)"
            )

    # ----- operatör servisleri -----

    def _on_arm_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok = self._request_arm()
        response.success = ok
        response.message = (
            f"arm dizisi başlatıldı (retry_max={self._arm_retry_max})"
            if ok else "arming servisi hazır değil"
        )
        return response

    def _on_disarm_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok = self._request_disarm()
        response.success = ok
        response.message = (
            "disarm isteği gönderildi" if ok else "arming servisi hazır değil"
        )
        return response


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MavrosBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

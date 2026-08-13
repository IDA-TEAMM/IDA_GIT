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
       ⚠️ **`-1` (ya da herhangi bir negatif) = bu yol KAPALI.** Yarışmada RC
       kullanılmayacağı için (kaptan kararı 2026-08-04) config'de -1 verilir.
       F-S.12: negatif kontrolü OLMADAN Python `channels[-1]`'i SON kanal
       olarak okuyup rastgele bir PWM'e bakardı — yanlış KILL riski.
    6. RC manuel-override (F-S.4) — `rc_manual_channel` (varsayılan 4,
       0-indexed = RC kanal 5) eşik PWM'in (`rc_manual_threshold_pwm`,
       varsayılan 1700) üstündeyken `_maybe_auto_guided()` GUIDED istemeyi
       bırakır — pilot RC'den manuel moda geçmek istediğinde yazılım
       kavga etmez. ⚠️ `-1` = KAPALI (F-S.12, yukarıdaki not).

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
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, StreamRate
from std_msgs.msg import String
from std_srvs.srv import Trigger

from girdap_decision.qos_profiles import sensor_data_qos
from girdap_decision.saat_kaynagi import SaatSicramaBekcisi, bayatlik_saati
from girdap_decision.yeniden_baslama import ResetAbonesi
from prototype.control.param_denetimi import OLUMCUL, denetle
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig


class MavrosBridgeNode(Node):
    """MAVROS güvenlik/mod köprüsü — MavrosBridge çekirdeğini sarar."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("mavros_bridge", **node_kwargs)

        # --- Parametreler ---
        self.declare_parameter("heartbeat_timeout_s", 5.0)
        # 🔴 PAR-04: `/mavros/state` **0,17 Hz** (≈6 s aralık) ölçüldü; eşik
        # 5,0 s olduğu için HER aralıkta aşıldı → köprü KILL'e latch'ledi ve
        # oturumun %86'sı ölü geçti. Kaynakta bu risk zaten yazılıydı
        # (`hardware.yaml`: "5 Hz'in altına İNME") ama kimse ÖLÇMÜYORDU:
        # akış hızı isteği bir kez gönderiliyor, uygulanıp uygulanmadığı
        # hiç teyit edilmiyordu.
        #
        # Operatör tarafındaki karşılığı: "kod değiştirip düğümleri yeniden
        # başlatınca sistem anında KILL verdi" — taze bağlantıda ArduPilot
        # SR0_* varsayılanıyla ~1 Hz yayınlar, eşik hemen aşılır.
        self.declare_parameter("akis_denetim_pencere", 5)
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
        # F-S.12: -1 (negatif) = RC kill yolu KAPALI. Yarışmada RC yok.
        self.declare_parameter("rc_kill_channel", 7)
        self.declare_parameter("rc_kill_threshold_pwm", 1500)
        # F-S.15: OTOMATİK kaynaklı KILL, sebebi geçtikten sonra kendini
        # temizler (heartbeat geri geldi / araç yeniden ARM edildi). Operatör
        # kaynaklı KILL (RC anahtarı, yer istasyonu) temizlenMEZ — yalnız
        # `/girdap/mission/reset` ile. false → eski mandallı davranış.
        # ⚠ Temizleme aracı ARM ETMEZ ve thrust vermez; yalnız köprünün
        # İZLEMESİNİ geri açar. Eski hâlde tek bir KILL'den sonra heartbeat
        # bekçisi oturumun sonuna kadar KAPALI kalıyordu.
        self.declare_parameter("kill_otomatik_temizleme", True)
        self.declare_parameter("kill_temizleme_bekleme_s", 3.0)
        # F-S.4: RC manuel-override — ida_topics ile aynı varsayılanlar
        # (RC kanal 5, 0-indexed 4; PWM eşiği 1700).
        # F-S.12: -1 (negatif) = manuel override KAPALI.
        self.declare_parameter("rc_manual_channel", 4)
        self.declare_parameter("rc_manual_threshold_pwm", 1700)
        # F-S.13: operatör hedef moddan çıkarsa yazılım geri zorlamaz. RC
        # kanalından BAĞIMSIZ — Mission Planner'dan gelen mod değişimini de
        # kapsar (rc_manual_channel=-1 iken tek koruma budur).
        self.declare_parameter("operator_mode_override", True)
        # F-P.24: "Remotes count" nöbetçisi. Sağlıklı hatta 1-3; yüzlere
        # çıkması hattan MAVLink olarak çözülemeyen bayt geldiği anlamına
        # gelir (2026-07-16: 174 → 323; 2026-08-12: 17-38).
        # 🔑 Ölçülen kök neden HAT HIZI UYUŞMAZLIĞI'dır — ayrıntı ve teyit
        # komutu `_on_diagnostics` docstring'inde. Elle `ros2 topic echo
        # /diagnostics` ile aramak 30+ dakika sürüyordu, artık otomatik.
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
            operator_override_enabled=bool(
                self.get_parameter("operator_mode_override").value
            ),
        )
        self._bridge = MavrosBridge(cfg)
        self._kill_oto_temizle = bool(
            self.get_parameter("kill_otomatik_temizleme").value
        )
        self._kill_temizleme_bekleme_s = float(
            self.get_parameter("kill_temizleme_bekleme_s").value
        )
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
        # F-S.15: KILL artık "kim tetikledi"yi de taşıyor — kurtarma politikası
        # kaynağa göre AYRI (bkz. `_kill_toparlanmayi_dene`). Kod zaten
        # "sebep etiketinde de ayrı, çünkü kurtarma politikası farklı" diyordu
        # (F-S.1 yorumu); eksik olan politikanın kendisiydi.
        self._kill_kaynagi: Optional[str] = None
        self._kill_saglikli_t: Optional[float] = None
        self._was_armed = False
        # PAR-04 akış ölçümü
        self._akis_pencere = max(2, int(self.get_parameter("akis_denetim_pencere").value))
        self._son_state_t: Optional[float] = None
        self._state_araliklari: list[float] = []
        self._akis_uyarildi = False
        # 🔴 KAR-02: KILL'e NEDEN girildiği hiçbir yere yazılmıyordu. Kaptanın
        # ifadesiyle "en büyük teşhis boşluğu bu": bir oturumun %82'si KILL'de
        # geçti, RC kanal 8 tüm oturum boyunca sabit 1000 (yani kill-switch HİÇ
        # tetiklenmedi) — demek ki yazılım içinden geldi, ama hangisinden
        # geldiği bag'den ayırt edilemedi.
        self._pub_kill_reason = self.create_publisher(
            String, "/girdap/mission/kill_reason", 10
        )
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
        # F-P.25 (2026-07-17): 2026-07-16 gerçek donanım testinde `/mavros/
        # set_mode` `mode_sent=True` DÖNDÜ (FC isteği kabul etti) ama gerçek
        # mod hiçbir zaman hedefe geçmedi (HOLD'da kaldı) — muhtemelen EKF/
        # GPS sağlığı veya link kalitesi (F-P.24) sebebiyle FC'nin kendi
        # GUIDED-giriş ön kontrolü reddediyordu. Bu, F-P.15'in yakaladığı
        # "hiç yanıt gelmedi" durumundan FARKLI — burada yanıt geldi ama
        # ETKİSİ hiç olmadı, hiçbir yerde loglanmıyordu.
        self._mode_ack_t: float | None = None
        self.declare_parameter("mode_ack_timeout_s", 3.0)
        self._mode_ack_timeout_s = float(
            self.get_parameter("mode_ack_timeout_s").value
        )
        self._mode_ack_warned = False
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
        # tek RC alıcısı üzerinden gelen fiziksel anahtar.
        # 🔴 2026-08-07 DÜZELTMESİ: buradaki gerekçe YANLIŞTI. Yorum "mavros
        # /mavros/rc/in BEST_EFFORT yayınlar" diyordu; gerçek donanımda
        # `ros2 topic info -v /mavros/rc/in` ile ÖLÇÜLDÜ → yayıncı **RELIABLE**
        # (MAVROS 2.x, ArduRover 4.6.3). BEST_EFFORT abone RELIABLE yayıncıyla
        # uyumludur (abone daha azını ister), yani kod ÇALIŞIYOR — ama güvenilir
        # bir kanalda güvenilirliği gereksiz yere bırakıyoruz.
        # Ölçülen QoS'lar: /mavros/state RELIABLE · /mavros/mission/reached
        # RELIABLE · /mavros/rc/in RELIABLE · /mavros/imu/data BEST_EFFORT ·
        # /mavros/local_position/velocity_body BEST_EFFORT.
        # ⚠ Tehlikeli yön TERSİ: BEST_EFFORT yayıncı + RELIABLE abone = sessiz
        # veri kaybı. Sensör topic'lerinde sensor_data_qos() bu yüzden ŞART.
        # Yarışmada RC kullanılmadığı için (rc_kill_channel=-1) bu satır fiilen
        # devre dışı; bench'te çalışıyor.
        self._sub_rc = self.create_subscription(
            RCIn, "/mavros/rc/in", self._on_rc_in, sensor_data_qos()
        )
        # F-P.24: mavros_router'ın kendi /diagnostics'i — "Remotes count"
        # anormalliğini (önce hat hızı uyuşmazlığı, sonra radyo
        # girişimi belirtisi) erken yakalar.
        self._sub_diag = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self._link_warn_active = False

        # --- Servis istemcileri ---
        self._cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self._cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        # 🔴 FC PARAMETRE ÖZ-DENETİMİ — parametreleri belirlemek takımda
        # BAŞKASININ görevi ve her testten sonra güncelleniyor. 13.08'de
        # bağlanıldığında 39 parametre değişmiş bulundu: ölçülmüş IMU
        # konumlarımız sıfırlanmış, batarya izleme kapatılmış, failsafe
        # eylemi kaldırılmıştı. Farkı elle ayıklamak yarım saat sürdü.
        # Artık her FCU bağlantısında kendiliğinden taranıyor ve YALNIZ
        # ölümcül sapmalar bildiriliyor (bkz. prototype/control/param_denetimi).
        self._cli_param = self.create_client(ParamGet, "/mavros/param/get")
        self._param_denetlendi = False
        self._param_okunan: dict = {}
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
        # §0.61: bayatlık ölçümü SIÇRAMAYA BAĞIŞIK saatle yapılır. Duvar saati
        # tek adımda 1497,6 s ilerlediğinde heartbeat "kayıp" sayılıp KILL
        # mandallanmıştı; hat ise hiç kopmamıştı.
        self._saat = bayatlik_saati(self)
        self._sicrama_bekcisi = SaatSicramaBekcisi()
        # F-S.15: köprü şimdiye kadar sıfırlama yayınının DIŞINDAYDI. Operatör
        # `/girdap/mission/reset` çağırınca FSM KILL'den çıkıyor, planlama
        # sürüyor — ama köprünün `_killed` mandalı asılı kaldığı için heartbeat
        # / beklenmedik-disarm / RC-kill izlemesinin ÜÇÜ DE kapalı kalıyordu.
        # Yani yeniden başlama sonrası tekne FAILSAFE'SİZ sürüyordu.
        self._reset = ResetAbonesi(self, self._yeniden_basla)
        rate = float(self.get_parameter("monitor_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_monitor)

        self.get_logger().info(
            f"mavros_bridge aktif (heartbeat={cfg.heartbeat_timeout_s}s, "
            f"hedef mod={cfg.target_mode}, auto_guided={self._auto_guided})"
        )

    # ----- zaman -----

    def _now(self) -> float:
        """Bayatlık saati — TEK YÖNLÜ (§0.61). Mutlak an olarak kullanılmaz."""
        return self._saat()

    # ----- /mavros/state callback -----

    def _on_state(self, msg: MavState) -> None:
        if msg.connected:
            self._param_denetimi_baslat()
        else:
            self._param_denetlendi = False      # yeniden baglanista tekrar dene
        self._akis_periyodunu_olc()
        onceki_devir = self._bridge.operator_override
        self._bridge.update_state(
            self._now(), msg.connected, msg.armed, msg.guided, msg.mode
        )
        self._operator_override_logla(onceki_devir, msg.mode)
        self._check_mode_ack_effect(msg.mode)         # F-P.25
        self._maybe_request_stream_rate()
        self._maybe_auto_guided()

    def _operator_override_logla(self, onceki: bool, mode: str) -> None:
        """F-S.13: devir kenarını GÖRÜNÜR yap — sessiz mandal en kötüsüdür.

        13.08 koşumunda mod 17 kez GUIDED↔MANUAL çırpındı ve hiçbir yere tek
        satır bile yazılmadı; kaptan yer istasyonunda hep "GUIDED" gördü
        (§0.60a). Kenar başına tek satır basılır, her tick değil.
        """
        simdi = self._bridge.operator_override
        if simdi == onceki:
            return
        if simdi:
            self.get_logger().warn(
                f"OPERATÖR DEVRALDI — mod {self._bridge.config.target_mode} → "
                f"{mode!r}. Yazılım {self._bridge.config.target_mode} istemeyi "
                "BIRAKTI (F-S.13); otonomi için yer istasyonundan ya da "
                f"kumandadan yeniden {self._bridge.config.target_mode} seç."
            )
        else:
            self.get_logger().info(
                f"operatör {mode} moduna geri verdi — otonomi yeniden devrede "
                "(F-S.13)"
            )

    def _param_denetimi_baslat(self) -> None:
        """FCU'ya bağlanınca ölümcül parametreleri bir kez oku ve karşılaştır.

        Neden BİR KEZ: parametreler koşu sırasında değişmez; her tick'te
        okumak MAVLink hattını gereksiz doldurur. Bağlantı koparsa bayrak
        sıfırlanır, yeniden bağlanınca tekrar denetlenir — parametre
        sorumlusu tam da o aralıkta değiştirmiş olabilir.
        """
        if self._param_denetlendi:
            return
        if not self._cli_param.service_is_ready():
            return                      # mavros henüz servisi açmadı, sonra
        self._param_denetlendi = True
        self._param_okunan = {}
        for ad in OLUMCUL:
            req = ParamGet.Request()
            req.param_id = ad
            fut = self._cli_param.call_async(req)
            fut.add_done_callback(
                lambda f, _ad=ad: self._param_yaniti(_ad, f)
            )

    def _param_yaniti(self, ad: str, future) -> None:  # noqa: ANN001
        """Tek parametre yanıtı; hepsi gelince raporu bas."""
        try:
            r = future.result()
            # ParamGet: integer VEYA real dolu gelir (tipine göre).
            deger = float(r.value.integer) if r.value.integer else float(r.value.real)
            self._param_okunan[ad] = deger if r.success else None
        except Exception:                               # noqa: BLE001
            self._param_okunan[ad] = None
        if len(self._param_okunan) < len(OLUMCUL):
            return

        bulgular = denetle(self._param_okunan)
        if not bulgular:
            self.get_logger().info(
                f"FC parametre denetimi TEMIZ ({len(OLUMCUL)} olumcul parametre kontrol edildi)"
            )
            return
        self.get_logger().error(
            f"🔴 FC PARAMETRE SAPMASI — {len(bulgular)} OLUMCUL deger beklenenden farkli. "
            "Parametreler takimda baskasi tarafindan yonetiliyor; bunlar BILEREK mi "
            "degistirildi, sor. Ayrinti:"
        )
        for b in bulgular:
            self.get_logger().error(f"   · {b}")

    def _akis_periyodunu_olc(self) -> None:
        """PAR-04: `/mavros/state` GERÇEK periyodunu ölç ve eşikle karşılaştır.

        Akış hızı isteği (`REQUEST_DATA_STREAM`) bir kez gönderiliyor ve
        **uygulanıp uygulanmadığı hiç teyit edilmiyordu**. FC isteği yok
        sayarsa (SR0_* EEPROM değerleri baskın, hat 57600 baud dolmuş,
        istek servisi hiç hazır olmamış) sistem sessizce yanlış varsayımla
        koşuyor — ta ki heartbeat eşiği aşılıp KILL gelene kadar.

        Ölçüt `timeout / 2`: periyot bunun üstündeyse tek bir gecikmiş mesaj
        bile eşiği aşmaya yeter, yani sistem KILL'e bir adım uzakta demektir.
        Medyan kullanılıyor — tek bir sıçrama (DDS keşfi, düğüm başlangıcı)
        yanlış alarm basmasın.
        """
        simdi = self._now()
        onceki = self._son_state_t
        self._son_state_t = simdi
        if onceki is None:
            return
        self._state_araliklari.append(simdi - onceki)
        if len(self._state_araliklari) < self._akis_pencere:
            return

        medyan = sorted(self._state_araliklari)[len(self._state_araliklari) // 2]
        self._state_araliklari.clear()
        sinir = self._bridge.config.heartbeat_timeout_s / 2.0
        if medyan <= sinir:
            if self._akis_uyarildi:
                self._akis_uyarildi = False
                self.get_logger().info(
                    f"/mavros/state periyodu normale dondu ({medyan:.2f}s)"
                )
            return

        if not self._akis_uyarildi:
            self._akis_uyarildi = True
            self.get_logger().error(
                f"🔴 /mavros/state periyodu {medyan:.2f}s — heartbeat esiginin "
                f"YARISINDAN buyuk ({sinir:.2f}s). Tek bir gecikmis mesaj "
                "KILL'e yeter. FC akis hizi istegi UYGULANMAMIS olabilir "
                "(SR0_* EEPROM baskin / 57600 baud hatti dolu). PAR-04: bu "
                "durumda bir oturumun %86'si KILL'de gecti. Istek "
                "tekrarlaniyor; duzelmezse FC'de SR0_EXTRA1/SR0_EXT_STAT "
                "kalici yazilmali."
            )
        # Isteği TEKRARLA — tek seferlik istek yeterli olmadığı ölçüldü.
        self._bridge.note_stream_rate_failed()      # bayrağı düşür → yeniden iste

    def _on_mission_state(self, msg: String) -> None:
        self._bridge.set_mission_state(msg.data)
        # F-M.3: operatör/YKİ kill'i fsm_node'dan geçer, bridge'in kendi
        # _trigger_kill yolu hiç çalışmaz — FSM KILL'e düştüyse FCU'yu da
        # disarm et (latch; kill servisi çağrısı FSM zaten KILL'de olduğundan
        # idempotent).
        if msg.data == "KILL" and not self._killed:
            self.get_logger().error("FSM KILL gözlendi → FCU disarm (F-M.3)")
            self._trigger_kill("fsm_kill:operator_veya_yki")
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
        if idx < 0:                      # F-S.12: kanal atanmamış → RC kill yolu KAPALI
            self._on_rc_manual_check(msg)
            return
        channel_pwm = msg.channels[idx] if len(msg.channels) > idx else None
        if self._bridge.is_rc_kill_active(channel_pwm):
            self.get_logger().error(
                f"RC KILL ANAHTARI AKTİF (kanal {idx + 1}, PWM={channel_pwm}) "
                f"→ FCU disarm (F-S.1)"
            )
            # ⚠ RC kaynaklı KILL TEK YÖNLÜ kalmalı — donanım her zaman kazanır.
            # Sebep etiketinde de ayrı, çünkü kurtarma politikası farklı.
            self._trigger_kill(f"rc_kill:kanal{idx + 1}_pwm{channel_pwm}")
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
        if idx < 0:                      # F-S.12: kanal atanmamış → override KAPALI
            return
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

        Sağlıklı bir hatta 1-3 olması gereken sayaç yüzlere çıkıyorsa hatta
        MAVLink çerçevesi olarak çözülemeyen bayt akıyor demektir; her sahte
        çerçeve yeni bir "uzak adres" gibi kaydedilir.

        🔑 KÖK NEDEN SIRASI (2026-08-12'de ölçüldü, sıra önemli):

        1. **Hat hızı uyuşmazlığı — ilk bakılacak yer.** `fcu_url`'deki hız
           uçuş kontrolcüsünün `SERIAL2_BAUD`'undan farklıysa hattan rastgele
           bayt gelir ve sayaç tırmanır. 12.08'de tam bu yaşandı: hat
           57600'deydi, uçuş kontrolcüsü **921600** konuşuyordu; hız
           düzeltilince sayaç 17-38'den 1-3'e indi ve bağlantı kuruldu.
           Teyit: `ros2 param get /mavros/param SERIAL2_BAUD` (921 = 921600)
           ile `fcu_url` aynı sayıyı göstermeli.
        2. **Telemetri radyosunun radyo frekansı girişimi** — 2026-07-16
           testindeki ilk açıklama (sayaç 174, sonra 323). Ölçümle
           doğrulanmadı; 12.08'de yakalanan trafikte `RADIO_STATUS` mesajı
           YOKTU ve hattın ucunda doğrudan TELEM2 kablosu olduğu udev
           kuralıyla teyit edildi. Artık ikinci sıradaki aday.

        ⚠️ Bu uyarı hattın **çöp akıtmasını** yakalar, **kopmasını** değil.
        Aygıt evrensel seri veri yolundan tamamen düşerse (`ftdi_sio ...
        now disconnected`) bu geri çağrı hiç tetiklenmez; orada `connected`
        bayrağı ve çekirdek günlüğü bakılır (12.08'de beş kez oldu).
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
                        f"{remotes} (sağlıklı: 1-3) — hattan MAVLink olarak "
                        "çözülemeyen bayt geliyor (F-P.24). ÖNCE HAT HIZINI "
                        "KONTROL ET: yukarıdaki bağlantı adresindeki hız, "
                        "uçuş kontrolcüsünün SERIAL2_BAUD değeriyle aynı mı? "
                        "(ros2 param get /mavros/param SERIAL2_BAUD — 921 = "
                        "921600). 12.08'de sebep buydu. Hızlar tutuyorsa "
                        "ikinci aday telemetri radyosunun girişimi: "
                        "antenleri birbirinden uzaklaştır"
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

    #: F-S.15: sebebi GÖZLENEBİLİR ve GEÇİCİ olan KILL kaynakları — sebep
    #: ortadan kalkınca mandal kendini temizler. Buraya YAZILMAYAN her kaynak
    #: (RC anahtarı, yer istasyonu/FSM kill'i) operatör niyetidir ve YALNIZ
    #: `/girdap/mission/reset` ile temizlenir: donanım/operatör her zaman kazanır.
    _OTOMATIK_KILL_KAYNAKLARI = ("heartbeat_kaybi", "beklenmedik_disarm")

    def _kill_toparlanmayi_dene(self) -> bool:
        """KILL mandalı temizlenebilir mi? Temizlediyse True.

        🔑 NEDEN GÜVENLİ: temizleme aracı **ARM ETMEZ** ve thrust vermez. KILL
        anında FCU zaten disarm edildi; mandalın kalkması yalnız köprünün
        İZLEMESİNİ geri açar. Tekne ancak operatör yeniden ARM edip görev
        durumu ARM→BEKLEMEDE→PARKUR zincirini geçerse hareket eder. Yani
        toparlanma "motorları geri veren" bir işlem değil, "bekçiyi geri açan"
        bir işlemdir.

        ⚠ Görev durumu KILL'den KENDİLİĞİNDEN çıkmaz — o operatörün yeniden
        başlama hakkıdır (md 5.5.3.1, puan sıfırlanır) ve otomatikleştirilemez.
        """
        if not self._kill_oto_temizle:
            return False
        if self._kill_kaynagi not in self._OTOMATIK_KILL_KAYNAKLARI:
            return False

        now = self._now()
        son = self._bridge.last_state
        # `connected=false` DE bir /mavros/state mesajıdır: heartbeat tazedir
        # ama hat yoktur (§0.48a'daki `mode: ''` hâli). İkisi birden şart.
        saglikli = (
            son is not None
            and son.connected
            and self._bridge.heartbeat_alive(now)
        )
        if self._kill_kaynagi == "beklenmedik_disarm":
            # Beklenmedik disarm'ın sebebi ancak araç YENİDEN ARM edilince
            # geçmiş sayılır — bu zaten operatörün (ya da FC'nin) eylemidir.
            saglikli = saglikli and self._bridge.is_armed()

        if not saglikli:
            self._kill_saglikli_t = None
            return False

        # Histerezis: hat çırpınırken KILL→temizle→KILL sarmalına girmesin.
        if self._kill_saglikli_t is None:
            self._kill_saglikli_t = now
            return False
        if (now - self._kill_saglikli_t) < self._kill_temizleme_bekleme_s:
            return False

        self._kill_temizle(
            f"{self._kill_kaynagi} sebebi gecti "
            f"({self._kill_temizleme_bekleme_s:.0f}s saglikli)"
        )
        return True

    def _kill_temizle(self, neden: str) -> None:
        """Mandalı düşür ve bunu GÖRÜNÜR yap (sessiz mandal en kötüsüdür)."""
        eski = self._kill_kaynagi
        self._killed = False
        self._kill_kaynagi = None
        self._kill_saglikli_t = None
        # F-M.2 kenar takibi: mandal boyunca arm durumu değişmiş olabilir;
        # eski değerle devam etmek sahte "beklenmedik disarm" üretirdi.
        self._was_armed = self._bridge.is_armed()
        self.get_logger().warn(
            f"KILL MANDALI TEMIZLENDI (eski sebep: {eski}) — {neden}. "
            "Bekci yeniden acildi; arac ARM EDILMEDI, thrust verilmedi."
        )
        self._pub_kill_reason.publish(String(data=f"temizlendi:{eski}"))

    def _yeniden_basla(self) -> None:
        """`/girdap/mission/reset` fan-out'u: KAYNAĞI NE OLURSA OLSUN temizle.

        Operatör kaynaklı KILL'in (RC anahtarı, yer istasyonu) tek çıkışı
        budur. ⚠ RC anahtarı HÂLÂ kill konumundaysa bir sonraki `/mavros/rc/in`
        mesajı KILL'i derhâl geri koyar — donanım her zaman kazanır (çekirdek
        `MissionFSM.reset()` de aynı ilkeyle çalışıyor).
        """
        if self._killed:
            self._kill_temizle("operator yeniden baslatma (md 5.5.3.1)")

    def _on_monitor(self) -> None:
        # §0.61: sıçrama artık KILL üretmiyor ama SESSİZ de kalmamalı — kayıt
        # damgaları bu andan sonra kayar (§0.53e), suda "o an ne oldu"nun
        # cevabı burada. Erken dönüşlerin ÖNÜNDE: bağlanmadan önce de olur.
        sapma = self._sicrama_bekcisi.kontrol()
        if sapma is not None:
            self.get_logger().warn(
                f"SISTEM SAATI {sapma:+.1f}s ADIMLANDI (NTP/GPS duzeltmesi) — "
                "bayatlik olcumleri tek yonlu saatte, failsafe ETKILENMEDI"
            )

        # F-M.7: izleme ancak FC en az bir kez connected=true görüldükten sonra
        # başlar. mavros bağlanamazken de state (connected=false) basar; restart
        # port devrinde >5 sn'lik state boşluğu FC hiç görülmeden KILL
        # latch'liyordu (journal 2026-07-14 18:13). İlk bağlantı öncesi thrust'ı
        # control_gate zaten "FCU baglantisi yok" ile kesiyor.
        if not self._bridge.ever_connected:
            return
        if self._killed:
            # F-S.15: eskiden burada KOŞULSUZ dönülüyordu — tek bir KILL,
            # oturumun geri kalanında bekçiyi tamamen kapatıyordu. Artık önce
            # toparlanma denenir; temizlenemezse (operatör kaynaklı KILL) yine
            # dönülür.
            if not self._kill_toparlanmayi_dene():
                return

        now = self._now()

        # 1) Heartbeat
        if not self._bridge.heartbeat_alive(now):
            dt = self._bridge.seconds_since_update(now)
            self.get_logger().error(
                f"FAILSAFE — heartbeat kaybı ({dt:.1f}s) → KILL"
            )
            self._trigger_kill(f"heartbeat_kaybi:{dt:.1f}s")
            return

        # 2) Beklenmedik disarm (arm True→False) = failsafe. Ama KOMUTLU disarm
        #    (operatör/görev sonu) failsafe DEĞİL — F14.2, çekirdekte sınıflanır.
        armed = self._bridge.is_armed()
        if self._bridge.is_unexpected_disarm(self._was_armed, armed):
            self.get_logger().error("FAILSAFE — beklenmedik disarm → KILL")
            self._trigger_kill("beklenmedik_disarm")
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
        # F-P.25: yeni istek — önceki denemenin ack takibini sıfırla, bu
        # denemeye taze bir şans ver.
        self._mode_ack_t = None
        self._mode_ack_warned = False
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
            self._mode_ack_t = self._now()            # F-P.25: etkiyi izle
        else:
            self.get_logger().warn("set_mode reddedildi (mode_sent=False)")

    def _check_mode_ack_effect(self, current_mode: str) -> None:
        """F-P.25: mode_sent=True olsa da gerçek mod hiç değişmeyebilir —
        FC'nin kendi GUIDED-giriş ön kontrolü (EKF/GPS sağlığı) sessizce
        reddedebilir. Bunu ELLE fark etmek 2026-07-16'da uzun sürdü."""
        if self._mode_ack_t is None or self._mode_ack_warned:
            return
        target = self._bridge.config.target_mode
        if current_mode == target:
            self._mode_ack_t = None                  # başarılı, izlemeyi bırak
            return
        if self._now() - self._mode_ack_t > self._mode_ack_timeout_s:
            self._mode_ack_warned = True
            self.get_logger().error(
                f"set_mode 'kabul edildi' (mode_sent=True) ama "
                f"{self._mode_ack_timeout_s:.0f}s'dir gerçek mod hâlâ "
                f"'{current_mode}' (hedef: '{target}') — FC'nin kendi "
                "GUIDED-giriş ön kontrolü (EKF/GPS sağlığı) reddediyor "
                "olabilir; link kalitesi de sebep olabilir (F-P.24 uyarısına "
                "bak) (F-P.25)"
            )

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

    def _trigger_kill(self, sebep: str = "bilinmiyor") -> None:
        """KILL: FCU'yu disarm et + FSM üzerinden sıfır thrust yay. Latching.

        F14.1: Önceki sürüm yalnız FSM→sıfır-thrust yapıyordu; araç ARMED kalıyor
        ve companion↔FCU hattı canlıyken bile FCU disarm edilmiyordu. Artık
        doğrudan `/mavros/cmd/arming False` çağrılır (kesin durdurma). Heartbeat/
        bağlantı kaybı senaryosunda bu komut FCU'ya ULAŞMAYABİLİR — o durumda
        koruma FCU'nun KENDİ failsafe'idir (ArduPilot GCS/throttle failsafe →
        otomatik disarm/hold), FC parametresi olarak ayrı doğrulanmalı.
        """
        self._killed = True
        # F-S.15: kaynak sınıfı = sebebin ilk parçası (`heartbeat_kaybi:6.2s`
        # → `heartbeat_kaybi`). Kurtarma politikası buna bakar.
        self._kill_kaynagi = sebep.split(":", 1)[0]
        self._kill_saglikli_t = None
        # KAR-02: sebebi ÖNCE yayınla — disarm çağrısı hat kopukken bloke
        # olabilir, teşhis onun arkasında kalmasın.
        self._pub_kill_reason.publish(String(data=sebep))
        self.get_logger().error(f"KILL SEBEBI: {sebep}")
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

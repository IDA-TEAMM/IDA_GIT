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
from mavros_msgs.msg import StatusText
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu

from girdap_decision.qos_profiles import latched_qos, sensor_data_qos
from girdap_decision.saat_kaynagi import bayatlik_saati
from girdap_decision.yeniden_baslama import (
    RESET_SERVICE,
    ResetYayinci,
)
from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation
from prototype.mission.p3_cikis import P3CikisIzleyici
from prototype.mission.parkur_fsm import (
    ParkurState,
    ParkurTransitionLogic,
    load_parkur_labels,
)


#: Odometri bu süreden eskiyse hız ölçümü GEÇERSİZ sayılır — bayat odomla
#: "tekne durdu" sonucu çıkarılamaz (sahte temas ⇒ görev erken biterdi).
_ODOM_BAYATLIK_S = 2.0


class FSMNode(Node):
    """MissionFSM ROS 2 sarmalayıcısı."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("fsm_node", **node_kwargs)
        # §0.61: takılma bekçisi (BOOT/ARM/BEKLEMEDE), çift-yığın denetimi ve
        # statustext tazeleme periyodu tek yönlü saatte — duvar saati adımı
        # bunların hepsini aynı anda yalancı olarak ateşliyordu.
        self._saat = bayatlik_saati(self)

        # --- Parametreler ---
        self.declare_parameter("tick_rate_hz", 10.0)
        # 🔴 5.0 → 3.0 (2026-08-06, LOG 58'DEN ÖLÇÜLDÜ — GIRDAP_DURUM §0.8l).
        # 5.0 hiçbir ölçüme dayanmıyordu. Log 58'in gerçek IMU'su (50 Hz,
        # 270 s, suya indirme + elle taşıma + manevralar dahil):
        #     otonom görev  p99.9 = 1.048 g   MAKS = 1.067 g
        #     TÜM log       p99.9 = 1.254 g   MAKS = 1.474 g
        #     1.5 g üstü tek bir örnek YOK.
        # Eşik iki koşulu birden sağlamalı:
        #   ALT: gerçek işletme gürültüsünün ÜSTÜ → 3.0 = ölçülen maksimumun
        #        2.0 katı (yarışmadaki deniz hâli göldekinden hareketli olacak)
        #   ÜST: sert bir çarpışmanın ulaşabileceği yerde kalmalı (1 m/s'lik
        #        temas 50 ms'de dururken ~2 g) → 3.0 hâlâ erişilebilir, 5.0 değil
        # ⚠ ASİMETRİ (eşiği seçen şey bu): sahte tetik = P3 hedefe varmadan
        # "tamamlandı" der ve motorlar durur → 145 puan gider. Kaçırılan darbe
        # ise görevi ÖLDÜRMEZ: tüm waypoint'ler bitince MissionFSM zaten
        # TAMAMLANDI'ya geçiyor (mission_fsm.py "görev tamamlandı"). Yani
        # yüksek eşiğin bedeli yalnız parkur-durumu raporu.
        self.declare_parameter("shock_threshold_g", 3.0)    # |a|/g eşiği
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
        # 🔴 PAR-09: `mission_source=fc` + çoklu parkur `mission_file`
        # kombinasyonunda parkur SINIRLARI statik dosyadan, waypoint'lerin
        # KENDİSİ ise FC'den (YKİ yüklemesi) gelir. İki kaynak senkron
        # OLMAYABİLİR ve bu, açılışta BİLİNEMEZ — gerçek görev henüz gelmedi.
        #
        # Bu yüzden karar açılışa değil, görevin GELDİĞİ ana bırakıldı
        # (`_on_waypoints`): FC görevinin waypoint SAYISI dosyadaki etiket
        # sayısıyla eşleşiyorsa index'ler hizalıdır, etiketler benimsenir;
        # eşleşmiyorsa TEK PARKUR güvenli modunda kalınır.
        #
        # ⚠ Neden açılışta RuntimeError ile durdurmuyoruz (kaptanın PAR-09
        # önerisi #1): `fc` kaynağı md 3.3.1(2) gereği ZORUNLU (görev YKİ'de
        # tanımlanıp yüklenir) ve `fc` ile parkur geçişi veren BAŞKA bir
        # yapılandırma yok. Sert ret, çelişkiyi çözmek yerine yarışma yolunu
        # tamamen kapatırdı — tekne hiç açılmazdı.
        self.declare_parameter("parkur_senkron_dogrula", True)
        # 🔴 KAR-01: `/girdap/mission/state` üzerinde İKİ çelişkili durum akışı
        # bulundu — 1.190 geçiş, %92,7'si 0,5 s'den kısa, `20 ms / 80 ms` SABİT
        # faz farkıyla. Tek bir FSM'in salınımı bunu üretmez (simetrik olurdu);
        # bu, iki bağımsız 10 Hz yayıncının imzasıdır. Üretim kodunda tek
        # yayıncı var (bu düğüm), yani ya ikinci bir `fsm_node` örneği koştu ya
        # da yayın dışarıdan (test sızıntısı, PAR-01) geldi.
        #
        # Bag'den SONRADAN çıkarılması haftalar aldı; ROS bunu doğrudan
        # söyleyebiliyor. `count_publishers` keşif tamamlandıkça güncellenir,
        # o yüzden açılışta bir kez değil PERİYODİK bakılıyor.
        self.declare_parameter("cift_yayinci_denetim_s", 5.0)

        self._fsm = MissionFSM()
        self._fsm.P1_TO_P2_DIST = float(
            self.get_parameter("p1_to_p2_dist").value
        )
        self._obs = Observation()
        # Parkur-3 çıkışı — şok kanalı P3'ü BİTİREMEZ (temas 0,03-0,14 g ↔
        # eşik 3,0 g), giriş de artık `mission_complete`'ten geldiği için o
        # terminal P3'te atlanıyor ⇒ başka bir çıkış ZORUNLU.
        # Çekirdek ROS'suz: `prototype/mission/p3_cikis.py`.
        self._p3_izleyici = P3CikisIzleyici()
        self._hiz_mps = 0.0
        self._hiz_t = None          # odom bayatlığı (sahte temas koruması)

        # --- Parkur geçiş katmanı (waypoint-index tabanlı, MissionFSM'den ayrı) ---
        self._cift_denetim_s = float(
            self.get_parameter("cift_yayinci_denetim_s").value
        )
        self._son_cift_denetim = 0.0
        self._cift_yayinci_uyarildi = False
        self._senkron_dogrula = bool(
            self.get_parameter("parkur_senkron_dogrula").value
        )
        self._parkur_etiketleri: list[int] = []   # dosyadan; senkron beklemede
        self._parkur_senkron_sonucu: Optional[bool] = None
        # 🔴 13.08 (belge madde 0 / B5.2): senkron sonucu YALNIZ ROS log'una
        # yazılıyordu. Operatör sahada Mission Planner'a bakıyor; senkron
        # tutmazsa PARKUR2/PARKUR3'e HİÇ geçilmez, yani KAMİKAZE YAPILMAZ ve
        # bunu ancak koşu bittikten sonra bag'den anlardık. Sonuç artık YKİ
        # ekranına da gidiyor — boksta, koşudan ÖNCE görülsün diye.
        self._senkron_ilani: Optional[str] = None
        #: tek atışlık ilanın seviyesi. None → metinde "YOK" arayan eski sezgi.
        self._ilan_ciddi: Optional[bool] = None
        self._parkur = self._build_parkur_logic()
        self._parkur_state_last = self._parkur.state       # geçiş log tespiti
        #: algının bildirdiği geçilen FARKLI geçit sayısı (md 5.5.2.4 denetimi).
        #: -1 = algıdan hiç haber gelmedi (sayaç 0'dan AYIRT EDİLİR: 0 "hiç
        #: geçit geçmedik" demek, -1 "ölçen kimse yok" demek — ikisi farklı
        #: arıza ve farklı mesaj gerektirir).
        self._gecit_sayisi = -1

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
        # `armed_bekleme_watchdog_s` parametresi ve `_armed_since` alanı
        # 12.08'de KALDIRILDI — F-P.23 bekçisi `_kilit_denetle`'ye taşındı.
        # Parametreyi geriye uyumluluk için bırakmadım: hiçbir yaml/launch
        # onu geçmiyordu (arandı), duran bir isim ileride "bu ayar bir şey
        # yapıyor" yanılgısı üretirdi.
        # F-A.4: görev/parkur durumunu MAVLink STATUSTEXT ile YKİ'ye (Mission
        # Planner → Messages) yolla. Şartname md 4.2 gereği. Kapatmak için
        # false (ör. mavros'suz masa testi).
        self.declare_parameter("statustext_enabled", True)
        # 🔴 11.08 SAHADA ÖLÇÜLDÜ — kod doğruydu ama YKİ'de HİÇ görünmüyordu.
        # Sebep: aşağıdaki `text == self._last_statustext` kapısı mesajı yalnız
        # DEĞİŞİMDE yolluyor. FSM açılışta BOOT→ARM→BEKLEMEDE(→PARKUR1)
        # geçişlerini ilk saniyelerde bitirip oturuyor; ondan sonra hat
        # SONSUZA KADAR sessiz. Canlı ölçüm: yığın koşarken 20 saniye boyunca
        # `/mavros/statustext/send`'de **sıfır** mesaj, FSM `PARKUR_1`'de.
        # Operatör MP'yi ne zaman açsa geçişleri kaçırmış oluyor.
        # 868 MHz telemetride kopma+yeniden bağlanma NORMAL olduğu için bu
        # yarışmada da tekrarlanır → periyodik tazeleme ZORUNLU.
        # 10 s: 20 dk görevde ~120 satır (MP Messages sekmesi okunabilir kalır);
        # 10 Hz tick'te her tick yollamak MAVLink hattını doldururdu.
        # 0.0 → tazeleme kapalı (eski yalnız-değişimde davranışı).
        self.declare_parameter("statustext_periyot_s", 10.0)
        # 🔴 KAR-03: BOOT'ta bu süreden uzun kalınırsa YÜKSEK SESLE teşhis.
        # Kaptanın bag analizi (`session_20260811_171943`): sistem 25 DAKİKA
        # BOOT'ta kilitli kaldı, bu sırada tüm topic'ler 10 Hz akmaya devam
        # etti. Operatör `ros2 topic hz` ile "sağlıklı" gördü; gerçek arıza
        # (MAVROS hiç bağlanmamış) topic akışının altında kayboldu — 25 dakika
        # ve 18,5 MB bag boşa gitti. Sessiz kalmak burada en pahalı seçenekti.
        # 0 → bekçi kapalı.
        self.declare_parameter("boot_uyari_s", 60.0)
        # Aynı bekçinin BEKLEMEDE ayağı (KAR-08). Kaptanın önerisi 30 s.
        self.declare_parameter("bekleme_uyari_s", 30.0)

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
        # 🔴 14.08 — ALGININ BASTIĞI AMA KİMSENİN DİNLEMEDİĞİ SAYAÇ.
        # `duba_gecis_navigator` 04.08'den beri `/perception/gate_count`
        # (geçilen FARKLI geçit sayısı) yayınlıyor; karar tarafında hiçbir
        # abonesi yoktu. Algı ekibi `gate_passed` bayrağını tam da bu yüzden
        # kapalı tutup (GATE_PASSED_YAYINLA=False) "dürüst olanı" bu sayaçla
        # veriyordu — biz okumayınca md 5.5.2.4'ün "en az iki duba ikilisi"
        # şartını ölçen tek kanal boşa akıyordu.
        self._sub_gate_count = self.create_subscription(
            Int32, "/perception/gate_count", self._on_gate_count, 10
        )
        # Görev yöneticisi tüm waypoint'leri bitirdi → TAMAMLANDI terminal (F12.2).
        # Video senaryosu (tek parkur, kamikaze yok) buradan temiz durur.
        self._sub_complete = self.create_subscription(
            Bool, "/girdap/mission/complete", self._on_mission_complete, 10
        )
        # 🔴 16.08 — P3 EMNİYET KAPISI. `p3_bekleniyor` hiçbir yerde True
        # YAPILMIYORDU; 14.08'de tetik `p2_waypoints_done`'a taşınmıştı ama
        # renk şartı YOKTU ⇒ İHA rengi bulamasa da FSM PARKUR3'e geçerdi.
        # Şartname s.25: yanlış hedefe temas 100→50, iki yanlış 100→5 ⇒ renk
        # yoksa SALDIRMAMAK doğru karar. Ayrıca Eyüp kararı (16.08): P1/P2
        # ölçülürken P3 kapalı kalsın — `kamikaze_target_color` varsayılanı ""
        # olduğu için bu kapı onu sağlıyor.
        # Latched QoS: renk kalkıştan ÖNCE bir kez yayınlanıyor (md s.22);
        # latch olmasa bu node yeniden başladığında mesajı bir daha görmezdi.
        self._sub_hedef_rengi = self.create_subscription(
            String, "/girdap/mission/hedef_rengi", self._on_hedef_rengi,
            latched_qos(),
        )
        # Sprint 4 parkur katmanı: waypoint-varış + çarpma placeholder.
        # PAR-09: gerçek görev geldiğinde parkur senkronunu doğrula.
        self._sub_waypoints = self.create_subscription(
            Path, "/girdap/mission/waypoints", self._on_waypoints, 10
        )
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
        # F-A.4 — şartname md 4.2: "Aracın anlık durum ve mod bilgileri İDA YKİ
        # ekranında görülecektir." MOD bilgisi MAVLink'ten zaten geliyor (Mission
        # Planner gösteriyor); DURUM (görev/parkur) hiçbir yerde görünmüyordu.
        # STATUSTEXT tek yönlü telemetridir (aşağı yön) → md 4.1'e uygun, md
        # 5.5.3.1'in yasakladığı "araca komut" DEĞİLDİR.
        self._statustext_enabled = bool(
            self.get_parameter("statustext_enabled").value
        )
        self._pub_statustext = (
            self.create_publisher(StatusText, "/mavros/statustext/send", 10)
            if self._statustext_enabled
            else None
        )
        self._last_statustext = ""      # değişimde ANINDA gönder (10 Hz spam yok)
        self._statustext_periyot_s = float(
            self.get_parameter("statustext_periyot_s").value
        )
        self._statustext_son_gonderim: Optional[float] = None
        self._statustext_abone_uyarildi = False
        # KAR-03 BOOT bekçisi
        self._boot_uyari_s = float(self.get_parameter("boot_uyari_s").value)
        self._bekleme_uyari_s = float(self.get_parameter("bekleme_uyari_s").value)
        self._kilit_baslangic = self._saat()
        self._kilit_durum: Optional[MissionState] = None
        self._kilit_uyarildi = False
        self._mavros_mesaji_geldi = False   # /mavros/state HİÇ geldi mi
        self._kilit_teshis = ""              # statustext'e eklenecek kısa sebep

        # --- Services ---
        self._srv_start = self.create_service(
            Trigger, "/girdap/mission/start", self._on_start_srv
        )
        # madde #11 (md 5.5.3.1): yeniden baslama hakki. Servisi YALNIZ bu node
        # sunar; digerleri fan-out topic'inden haber alir (yeniden_baslama.py).
        self._reset_pub = ResetYayinci(self)
        self._srv_reset = self.create_service(
            Trigger, RESET_SERVICE, self._on_reset_srv
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
            # 🔴 12.08 (PAR-09/KAR-08): eskiden burada YALNIZ log basılıyor,
            # sonra senkron OLMADIĞI BİLİNEN etiketler yine de kullanılıyordu.
            # İki devam seçeneğinin ikisi de kötü, ama eşit değil:
            #   · senkron olmayan etiketler → parkur geçişi YANLIŞ index'te
            #     tetiklenir; PARKUR3 profili kamikaze çekicisidir (hedefe
            #     NEGATİF maliyet), yanlış anda açılması AKTİF TEHLİKEDİR.
            #   · tek parkur → kamikaze hiç yapılmaz, puan düşer ama güvenli.
            # Bu yüzden hangi kolda olursak olalım artık senkron olmayan
            # etiketler ASLA kullanılmıyor.
            self.get_logger().error(
                "mission_source=fc AMA mission_file ÇOKLU parkur içeriyor "
                f"({path}, parkurlar={sorted(set(labels))}) — FC'den yüklenen "
                "gerçek görev waypoint'leri HER ZAMAN parkur=1 sayılır "
                "(mission_manager.fc_items_to_waypoints), bu dosyanın parkur "
                "sınırlarıyla SENKRON DEĞİL. Parkur geçişleri (waypoint-index "
                "tabanlı) YANLIŞ ZAMANDA tetiklenebilir ya da HİÇ tetiklenmez "
                "— yarışma öncesi YKİ görevini bu dosyayla EL İLE doğrula."
            )
            if self._senkron_dogrula:
                # Etiketleri SAKLA ama HENÜZ KULLANMA: gerçek görev gelince
                # (`_on_waypoints`) sayı eşleşirse benimsenecek. O ana kadar
                # tek parkur güvenli modu — yanlış index'te PARKUR3'e (kamikaze
                # çekicisi, hedefe negatif maliyet) geçmek AKTİF TEHLİKEDİR.
                self._parkur_etiketleri = list(labels)
                self.get_logger().warn(
                    "parkur etiketleri ASKIYA ALINDI — FC gorevi gelince "
                    f"waypoint sayisi {len(labels)} ile eslesirse benimsenecek. "
                    "O ana kadar TEK PARKUR guvenli modu."
                )
                return ParkurTransitionLogic([])
            self.get_logger().error(
                "parkur_senkron_dogrula=false → senkron OLMAYABILECEK "
                "etiketler dogrudan kullaniliyor. Parkur gecisi yanlis "
                "index'te tetiklenebilir (PARKUR3 = kamikaze)."
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
        self._mavros_mesaji_geldi = True
        self._mav_armed = msg.armed
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
        v = msg.twist.twist.linear
        self._hiz_mps = math.hypot(v.x, v.y)
        self._hiz_t = self._saat()
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
        """Perception duba ikilisi geçiş tespiti → PARKUR2→PARKUR3 tetiği.

        ✅ **14.08 — AÇIK TUZAK KAPATILDI: B seçeneği uygulandı.**

        ESKİDEN burası gelen HERHANGİ bir `True`'yu "Parkur-2'nin SON ikilisi
        geçildi" sayıp `MissionFSM`'i doğrudan PARKUR3'e (kamikaze) atıyordu →
        **ilk kapıda** Parkur-2 yarıda kesilirdi; md 5.5.2.4'ün *"en az iki duba
        ikilisi"* şartı sağlanmaz ve md 657 gereği Parkur-3'ün **145 puanı hiç
        açılmazdı**. Canlı arıza değildi (bu topic'i üreten yayıncı yok; algı
        `GATE_PASSED_YAYINLA=False` tutuyor) ama "yayıncıyı açalım" diyen ilk
        kişi Parkur-2'yi kırardı — yayıncı eklenmeden çözülmesi istenmişti.

        **Seçilen yol (B):** geçişi **waypoint ilerlemesi** sürer
        (`Observation.p2_waypoints_done`, `_on_waypoint_reached`'te set edilir);
        bu sinyal yalnız **kanıt/telemetri**dir. Gerekçe: CLAUDE.md FSM ilkesi
        *"geçiş waypoint-index + parkur etiketi ile; duba sayısına bağlı akış
        tasarlamak YASAK"* (md 5.5.2.2) + P1→P2 zaten böyle çalışıyor + iki
        katman (MissionFSM ↔ ParkurTransitionLogic) artık çelişemiyor.

        A seçeneği (sayaç ≥ 2 şartı) REDDEDİLDİ: P3'ü kendi kapı sayacımıza
        bağlardı; algı zinciri suda doğrulanmamışken tek yanlış negatif 145
        puanı götürürdü.

        ⚠ Yayıncı açıldığında burası artık güvenlidir — ama `GateFollower.
        passed_gate_count` (B3) bağlanınca bu kanalın **kanıt** rolünü koru,
        tetiğe terfi ettirme.
        """
        if msg.data:
            self._obs.last_gate_passed_p2 = True

    def _on_gate_count(self, msg: Int32) -> None:
        """Algının geçilen FARKLI geçit sayısı — md 5.5.2.4 denetim kanalı.

        Geçişi SÜRMEZ (yetkili tetik waypoint ilerlemesi, bkz.
        `_on_gate_passed`). İki işi var:
          1. Parkur-2 bitmeden önce şartnamenin **en az iki duba ikilisi**
             şartının tutup tutmadığını ölçmek,
          2. tutmuyorsa YKİ ekranına BAĞIRMAK — çünkü bunu koşu bitmeden
             öğrenirsek hâlâ düzeltilebilir; sonradan öğrenirsek puan gitmiştir.

        ⚠ Sayaç **engellemez**: algı zinciri suda doğrulanmadı (YOLO modeli
        yok), tek bir yanlış negatif Parkur-3'ün 145 puanını kapatırdı. Aynı
        gerekçeyle A seçeneği reddedilmişti; burada da ölçüp söylüyoruz,
        karar vermiyoruz.
        """
        self._gecit_sayisi = int(msg.data)

    def _on_hedef_rengi(self, msg: String) -> None:
        """Hedef rengi yüklendi/temizlendi → `p3_bekleniyor` kapısı.

        Boş dize = renk atanmamış ⇒ P3 **hiç açılmaz**; tekne son waypoint'te
        temiz durur ve P1+P2 puanı korunur.
        """
        yeni = bool(msg.data.strip())
        if yeni != self._obs.p3_bekleniyor:
            self._obs.p3_bekleniyor = yeni
            # WARN bilinçli: operatör koşu öncesi bunu GÖRMELİ.
            self.get_logger().warn(
                f"PARKUR-3 kapisi {'ACIK' if yeni else 'KAPALI'} — "
                f"hedef rengi = {msg.data.strip() or 'ATANMAMIS'}"
            )
            # 19.08 — hangi tetikle P3'e geçeceğimizi operatöre SÖYLE.
            # İki yol var ve ikisi de geçerli:
            #   · görevde `parkur: 3` etiketli nokta VARSA → o noktanın bir
            #     öncesine (P2'nin son wp'si) varınca `p2_waypoints_done`;
            #   · YOKSA (yarışmada beklenen hâl — kamikazede görev noktası
            #     verilmiyor) → TÜM waypoint'ler bitince `mission_complete`.
            # Eskiden burada "P3'e HİÇ geçilmez" diye HATA basılıyordu; ikinci
            # yol eklendiği için o teşhis artık YANLIŞ olurdu.
            if yeni:
                p3_wp = 3 in self._parkur.last_index_of_parkur
                self.get_logger().warn(
                    "PARKUR-3 TETIGI: "
                    + ("gorevde 'parkur: 3' etiketli nokta VAR -> P2'nin son "
                       "waypoint'inde gecilecek"
                       if p3_wp else
                       "gorevde 'parkur: 3' noktasi YOK -> TUM waypoint'ler "
                       "bitince (mission_complete) gecilecek")
                    + f" (parkur son index'leri={self._parkur.last_index_of_parkur})"
                )

    def _on_mission_complete(self, msg: Bool) -> None:
        """Görev yöneticisi tüm waypoint'leri bitirdi mi? → TAMAMLANDI / PARKUR3.

        🔴 19.08 — **LATCH KALDIRILDI.** Eskiden `if msg.data: ... = True` idi
        ve bir daha sıfırlanmıyordu. P3 girişi bu bayrağa bağlandığı andan
        itibaren bu bir ARIZA olurdu:

          · md 5.5.3.1 **yeniden başlama hakkı** kullanılınca `mission_manager`
            IDLE'a döner ve `is_complete` **False** yayınlar, ama mandallı
            gözlem True kalırdı ⇒ araç PARKUR1'e girer girmez (renk yüklüyse)
            **anında kamikazeye** atlardı — iskeleye 5 m mesafede.
          · Eyüp'ün senaryosu (19.08): *"WP varken kapansın, WP yokken açılsın,
            aç-kapa yapabilsin"* — kapının canlı olması tam olarak bu.

        Güvenli, çünkü yayıncı `is_complete`'i **her tick** basıyor
        (`mission_manager_node:523`), yani bu bir durum akışı; tek mesaj
        düşerse 100 ms sonra yenisi gelir. TAMAMLANDI da terminal durum —
        oraya geçildikten sonra bayrağın False olması bir şeyi geri almaz.

        ⚠️ **Boş görev tuzağı yok**: waypoint listesi boşken `MissionManager`
        IDLE'da kalır (`start()` yalnız `self._wps` doluysa ACTIVE'e geçer)
        ⇒ `is_complete` **False** ⇒ P3 açılmaz. Sonradan WP yüklenirse görev
        normal akar. Ölçüldü değil, kaynaktan okundu: `mission_manager.py:221`.
        """
        self._obs.mission_complete = bool(msg.data)

    def _on_waypoints(self, msg: Path) -> None:
        """PAR-09: FC görevi geldi — parkur etiketleri onunla hizalı mı?

        `mission_source=fc` iken waypoint'ler YKİ yüklemesinden, parkur
        sınırları ise statik `mission_file`'dan gelir. İki kaynağın hizalı
        olması, waypoint SAYILARININ eşit olmasına bağlıdır: parkur geçişi
        `waypoint_reached` **index'i** ile tetiklendiği için, sayı tutuyorsa
        index'ler de birebir karşılık gelir.

        Sayı tutmuyorsa etiketler benimsenmez — tek parkur güvenli modunda
        kalınır. Kamikaze yapılmaz (puan kaybı), ama yanlış waypoint'te
        PARKUR3 profiline geçilmez (aktif tehlike).

        ⚠ Yalnız görev BAŞLAMADAN önce benimsenir. Koşu ortasında parkur
        mantığını değiştirmek, o ana kadarki ilerlemeyi geçersiz kılardı.
        """
        if not self._parkur_etiketleri:
            return                          # askıda bekleyen etiket yok
        # 🔴 17.08.2026 — ESKİDEN BURADA `if self._parkur_senkron_sonucu is not
        # None: return` VARDI ("karar bir kez verilir") ve YAZILI ÇÖZÜMÜ
        # ÇALIŞMAZ HÂLE GETİRİYORDU:
        #   1. Hakem noktaları verir, sayı tutmaz → "PARKUR SENKRON YOK 7!=5wp",
        #      etiketler askıya alınır, TEK PARKUR güvenli modu.
        #   2. Operatör aşağıdaki hata mesajının dediğini yapar: görevi düzeltip
        #      YKİ'den YENİDEN YÜKLER.
        #   3. Bu geri çağırma yeniden koşar ama karar `False`'a KİLİTLİ olduğu
        #      için **erken döner** ⇒ düzeltme hiç uygulanmaz.
        #   4. `_parkur_senkron_sonucu`'nu sıfırlayan BAŞKA yol da yok —
        #      `_yeniden_basla` servisi bile dokunmuyor (yalnız kurucuda None).
        # ⇒ Operatör talimatı uygular, hiçbir şey değişmez, tek-parkur modunda
        #   kalınır: **P2 (maks 100) ve P3 (maks 145) hiç tetiklenmez.**
        #
        # Kilit KALDIRILDI. Güvenlik kaybı YOK: hemen aşağıdaki durum kapısı
        # zaten yalnız BOOT/ARM/BEKLEMEDE'de benimsemeye izin veriyor — yani
        # "koşu ortasında parkur mantığı değişmesin" güvencesi ONDAN geliyordu,
        # bu kilitten değil. Kalkış öncesi düzeltilmiş görevin benimsenmesi
        # tam olarak istenen davranış.
        if self._fsm.state not in (
            MissionState.BOOT, MissionState.ARM, MissionState.BEKLEMEDE
        ):
            return                          # görev başladı — geç kalındı

        gelen = len(msg.poses)
        beklenen = len(self._parkur_etiketleri)
        if gelen != beklenen:
            self._parkur_senkron_sonucu = False
            self._senkron_ilani = f"PARKUR SENKRON YOK {gelen}!={beklenen}wp"
            self.get_logger().error(
                f"🔴 PARKUR SENKRONU YOK: FC gorevi {gelen} waypoint "
                f"iceriyor, mission_file {beklenen} etiket. Index'ler "
                "hizali DEGIL → parkur etiketleri KULLANILMIYOR, TEK PARKUR "
                "guvenli modunda kalindi. KAMIKAZE (PARKUR3) YAPILMAYACAK. "
                "COZUM: YKI'ye yuklenen gorevi mission_file ile ayni "
                "waypoint sayisina getirip YENIDEN YUKLE — kalkistan once "
                "yuklenirse etiketler kendiliginden benimsenir (17.08: karar "
                "artik kilitli degil). Gorev BASLADIKTAN sonra yuklersen "
                "benimsenmez; o durumda once girdap-karar'i yeniden baslat."
            )
            return

        self._parkur_senkron_sonucu = True
        self._senkron_ilani = f"PARKUR SENKRON OK {gelen}wp"
        try:
            self._parkur = ParkurTransitionLogic(self._parkur_etiketleri)
        except ValueError as exc:
            self._parkur_senkron_sonucu = False
            self.get_logger().error(
                f"parkur etiketleri gecersiz ({exc}) — tek parkur guvenli modu"
            )
            return
        self._parkur_state_last = self._parkur.state
        self.get_logger().info(
            f"✅ PARKUR SENKRONU DOGRULANDI: {gelen} waypoint = {beklenen} "
            f"etiket, parkur sinirlari benimsendi "
            f"({self._parkur.last_index_of_parkur})"
        )

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
        # 🔴 14.08 — P2→P3'ün YETKİLİ tetiği (yukarıdaki P1→P2 deseninin eşi).
        # Aynı korumayla: parkur-3 etiketli waypoint YOKSA beslenmez, yoksa
        # tek/iki parkurlu görevde sahte kamikaze geçişi olurdu.
        if (
            idx == self._parkur.last_index_of_parkur.get(2)
            and 3 in self._parkur.last_index_of_parkur
        ):
            self._obs.p2_waypoints_done = True
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

    def _on_reset_srv(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        """Yumusak yeniden baslatma — md 5.5.3.1 yeniden baslama hakki.

        FSM'i BEKLEMEDE'ye, parkur katmanini PARKUR_1'e alir, sonra fan-out
        yayinini yapar ki diger node'lar (kapi hafizasi + MPPI sicak durumu,
        gorev index'i, CSV/PNG oturumlari) kendilerini toplasin.

        ⚠ SIRA: once KENDI durumumuzu sifirla, SONRA yayinla. Tersi olursa
        diger node'lar sifirlanip bir sonraki tick'te bizim ESKI durumumuzu
        (or. PARKUR2) yeniden yayinlamamizi gorur.

        Puan sifirlanmasi (md 5.5.3.1) `GateFollower.reset_passed_gates()` ile
        planning_node tarafinda yapiliyor — gecis sayaci orada yasiyor.
        """
        onceki = self._fsm.state.value
        self._fsm.yeniden_basla("YKI yeniden baslama servisi (md 5.5.3.1)")
        self._parkur.reset()
        # 14.08: gorev ilerleme bayraklari da sifirlanir. Yoksa yeniden
        # baslatilan arac Parkur-1'i kosarken ESKI kosumdan kalan
        # p2_waypoints_done ile PARKUR2'ye girer girmez kamikazeye atlar.
        self._obs.p2_waypoints_done = False
        self._obs.last_gate_passed_p2 = False
        self._obs.dist_to_last_wp_p1 = math.inf
        n = self._reset_pub.yayinla()
        self.get_logger().warn(
            f"YENIDEN BASLAMA #{n} (md 5.5.3.1): {onceki} -> "
            f"{self._fsm.state.value}, parkur -> {self._parkur.state.value}. "
            f"Puanlar sifirlaniyor; baslatmak icin /girdap/mission/start"
        )
        res.success = True
        res.message = (
            f"yeniden baslama #{n}: {onceki} -> {self._fsm.state.value}"
        )
        return res

    #: md 5.5.2.4 — Parkur-2'nin sayılması için gereken EN AZ duba ikilisi.
    #: Şartnameden gelir, ayar değildir; `params.yaml`'a taşınmamalı.
    ASGARI_GECIT = 2

    def _gecit_sartini_denetle(self) -> None:
        """Parkur-2 biterken md 5.5.2.4 şartı tuttu mu — TUTMADIYSA BAĞIR.

        Parkur-2 puanı (G2/KD2 ×40) ve ödül sıralamasının kapısı buradan
        geçiyor; üstelik md 657 gereği Parkur-3'ün 145 puanı da Parkur-2
        tamamlanmadan açılmıyor. Koşu SIRASINDA öğrenilirse hâlâ telafi
        edilebilir, sonradan öğrenilirse edilemez.

        Üç hâl ayrı ayrı raporlanır — hepsi "eksik" değil:
          * sayaç yok (-1)  → ölçen kimse yok (algı düğümü kapalı/çökmüş).
                              Bu bir ALGI arızasıdır, parkur ihlali değil.
          * sayaç < 2       → gerçek şartname riski.
          * sayaç ≥ 2       → sessiz kalma, olumlu teyidi de yaz (operatör
                              "ölçüldü mü" diye merak etmesin).
        """
        n = self._gecit_sayisi
        if n < 0:
            self.get_logger().warn(
                "PARKUR-2 bitti ama GECIT SAYACI HIC GELMEDI "
                "(/perception/gate_count). md 5.5.2.4'un 'en az iki duba "
                "ikilisi' sarti OLCULEMEDI — algi dugumu kosuyor mu?"
            )
            self._senkron_ilani = "GECIT SAYACI YOK"
            self._ilan_ciddi = True
        elif n < self.ASGARI_GECIT:
            self.get_logger().error(
                f"PARKUR-2 {n} gecitle bitti — md 5.5.2.4 EN AZ "
                f"{self.ASGARI_GECIT} duba ikilisi istiyor. Parkur-2 "
                f"sayilmayabilir (G2/KD2 x40) ve md 657 geregi Parkur-3'un "
                f"145 puani ACILMAYABILIR."
            )
            self._senkron_ilani = f"PARKUR2 EKSIK GECIT {n}<{self.ASGARI_GECIT}"
            self._ilan_ciddi = True
        else:
            self.get_logger().info(
                f"PARKUR-2 {n} gecitle tamamlandi (md 5.5.2.4 asgari "
                f"{self.ASGARI_GECIT} SAGLANDI)."
            )
            self._senkron_ilani = f"PARKUR2 GECIT OK {n}"
            self._ilan_ciddi = False

    # ----- tick döngüsü -----

    def _on_tick(self) -> None:
        # ARM → BEKLEMEDE: Pixhawk armed → kill switch fiziksel olarak OFF
        self._obs.kill_switch_off = self._mav_armed

        # F-P.23'ün eski armed+BEKLEMEDE bekçisi BURADAYDI; `_kilit_denetle`
        # onun yerini aldı. Sebep: `self._mav_armed` şartına bağlıydı ve
        # PAR-03'e göre araç 14 oturumun hiçbirinde ARM edilmedi — bekçi bir
        # kez bile ateşlemedi. Yeni bekçi ARM YOKLUĞUNU da bir sebep sayıyor.

        # Parkur-3 çıkış ölçütleri (yalnız P3'teyken anlamlı).
        simdi = self._saat()
        if self._fsm.state is MissionState.PARKUR3:
            if not self._p3_izleyici.p3te_mi:
                self._p3_izleyici.p3ye_girildi(simdi)
            # Odom SUSARSA `_hiz_mps` son değerinde DONAR; tekne o sırada
            # duruyorsa "ilerleme yok" SAHTE tetiklenir ve görev, temas
            # olmadığı hâlde TAMAMLANDI'ya düşer. Ölçemediğimiz şeyde çelişki
            # iddia etmiyoruz (aynı kural `menzil_tutarli`da da var).
            taze = (self._hiz_t is not None
                    and (simdi - self._hiz_t) <= _ODOM_BAYATLIK_S)
            self._obs.p3_ilerleme_yok, self._obs.p3_sure_doldu = (
                self._p3_izleyici.guncelle(simdi, self._hiz_mps,
                                           hiz_gecerli=taze)
            )
        elif self._p3_izleyici.p3te_mi:
            self._p3_izleyici.sifirla()
            self._obs.p3_ilerleme_yok = False
            self._obs.p3_sure_doldu = False

        new_state = self._fsm.tick(self._obs)

        # F-V.6: BEKLEMEDE'ye YENİ girildiyse ve mod zaten start_on_mode ise
        # (kenar yok) görevi burada başlat — tick sonrası, çünkü BEKLEMEDE'ye
        # geçiş bu tick'te oluyor.
        if new_state is MissionState.BEKLEMEDE:
            self._maybe_start_without_edge()
            new_state = self._fsm.state          # başladıysa PARKUR1 yayınlansın

        # Tek atış sinyalleri tüketildiğinde sıfırla
        if new_state is MissionState.PARKUR3:
            self._gecit_sartini_denetle()
            self._obs.last_gate_passed_p2 = False
            # 14.08: yetkili tetik de tüketilir — yoksa yeniden başlamadan
            # sonra PARKUR2'ye dönüldüğünde bayrak hâlâ True olur ve araç
            # Parkur-2'yi hiç koşmadan kamikazeye geçer.
            self._obs.p2_waypoints_done = False
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

        self._cift_yayinci_denetle()

        # 🔴 KAR-03: BOOT kilitlenmesini TESPİT ET (statustext'ten ÖNCE, çünkü
        # teşhis metni operatöre BOOT satırıyla birlikte gidiyor).
        self._kilit_denetle(new_state)

        # YKİ ekranı (şartname md 4.2)
        self._publish_statustext(new_state)

    def _cift_yayinci_denetle(self) -> None:
        """KAR-01: `/girdap/mission/state`'e bizden BAŞKA yayıncı var mı?

        Kaptanın bag'inde `ARM ↔ PARKUR2` salınımı `20 ms / 80 ms` sabit faz
        farkıyla tekrarlıyordu — toplam periyot tam 100 ms = 10 Hz. Tek bir
        FSM'in salınımı simetrik olurdu; bu desen **iki bağımsız 10 Hz
        yayıncının** imzasıdır. FSM'i dinleyen her düğüm (mission_manager,
        planning_node, mavros_bridge, telemetry) saniyede 10 kez birbiriyle
        çelişen durum gördü; görev-aktif geçidi sürekli açılıp kapandı.

        Sebep iki ihtimalden biri: ikinci bir `fsm_node` örneği, ya da testlerin
        canlı domaine sızması (PAR-01 — `conftest.py` izolasyonuyla kapatıldı).
        İkisini de bu kontrol yakalar, çünkü ikisi de fazladan bir yayıncıdır.

        ⚠ Periyodik, çünkü DDS keşfi anlık değil: açılışta tek bakış, sonradan
        beliren bir ikinci örneği kaçırırdı. Uyarı bir kez basılır (durum
        düzelirse tekrar armlanır) — 10 Hz'te ERROR selini önlemek için.
        """
        if self._cift_denetim_s <= 0.0:
            return
        simdi = self._saat()
        if simdi - self._son_cift_denetim < self._cift_denetim_s:
            return
        self._son_cift_denetim = simdi

        try:
            n = self.count_publishers("/girdap/mission/state")
        except Exception:                       # rclpy sürüm farkı — sessiz geç
            return

        if n > 1:
            if not self._cift_yayinci_uyarildi:
                self._cift_yayinci_uyarildi = True
                self.get_logger().error(
                    f"🔴 /girdap/mission/state uzerinde {n} YAYINCI var "
                    "(bizim disimizda en az bir tane daha) — FSM durumu "
                    "CELISKILI akiyor. Ikinci bir fsm_node ornegi mi kosuyor "
                    "(`ros2 node list`), yoksa testler canli domaine mi "
                    "siziyor (ROS_DOMAIN_ID)? KAR-01: bu, gorev-aktif "
                    "gecidini saniyede 10 kez acip kapatir."
                )
        elif self._cift_yayinci_uyarildi:
            self._cift_yayinci_uyarildi = False
            self.get_logger().info(
                "/girdap/mission/state tek yayinciya dondu — celiski bitti."
            )

    def _kilit_denetle(self, state: MissionState) -> None:
        """Görev İLERLEMİYORSA sebebini AYIRT ET ve operatöre yüksek sesle söyle.

        KAR-03 + KAR-08 tek bekçide birleşti, çünkü ikisi aynı arızanın iki
        durağı: FSM bir bekleme durumunda takılıyor, sistemin geri kalanı
        bunu umursamadan 10 Hz akmaya devam ediyor, operatör sağlıklı bir
        sistem görüyor. 14 oturumun **hiçbirinde** görev PARKUR'a geçemedi.

        Takılmanın kendisi çoğu zaman DOĞRU davranıştır (MAVROS yoksa BOOT'ta
        kalınır, başlat komutu yoksa BEKLEMEDE'de beklenir). Hata, sebebin
        hiçbir yerde söylenmemesi.

        🔴 Neden F-P.23'ün yerini alıyor: eski BEKLEMEDE bekçisi
        `self._mav_armed` şartına bağlıydı — yani ancak araç ARM edildikten
        SONRA konuşabiliyordu. PAR-03: 14 oturumdaki 41.524 `/mavros/state`
        mesajının **hiçbirinde** `armed=true` yok. Bekçi bir kez bile
        ateşlemedi; tam olarak teşhis etmesi gereken duruma karşı kördü.
        (KAR-03'te fuzyon/planlama bekçilerinde bulunan desenin üçüncü örneği:
        *bekçi, işlerin kısmen yürüdüğü hâli varsayıyor.*)

        Sebepler ve operatörün bakacağı yer:

        | teşhis | anlamı | nereye bakılır |
        |---|---|---|
        | `MAVROS-YOK` | `/mavros/state` hiç gelmedi | launch/servis, ROS_DOMAIN_ID |
        | `FCU-KOPUK` | geliyor ama `connected=false` | kablo, fcu_url, baud, güç |
        | `ARM-YOK` | araç ARM edilmemiş | Mission Planner pre-arm uyarıları |
        | `MOD-YOK` | ARM var, mod `start_on_mode` değil | YKİ'den mod komutu |
        | `BASLAT-YOK` | ARM + mod doğru, görev yine de başlamadı | anormal, log'a bak |
        """
        simdi = self._saat()

        # 🔴 12.08 CANLI BULGU: ARM da bir BEKLEME durumu ve bekçi onu
        # kapsamıyordu. Sistem yeniden başlatıldığında FSM ARM'da takılı kaldı
        # (MAVROS bağlı → BOOT'tan çıktı, ama araç arm edilemediği için
        # BEKLEMEDE'ye geçemedi) ve hiçbir teşhis basılmadı — kapattığım
        # sessizliğin aynısı bir durum ötede duruyormuş.
        # Kaptanın verisi de bunu söylüyordu: `session_20260811_143741`'de
        # `ARM` **14.644** örnek. Gözden kaçırmışım.
        if state not in (
            MissionState.BOOT, MissionState.ARM, MissionState.BEKLEMEDE
        ):
            self._kilit_durum = None
            self._kilit_baslangic = simdi
            self._kilit_uyarildi = False
            self._kilit_teshis = ""
            return

        # Durum değiştiyse sayaç baştan — BOOT'ta geçen süre BEKLEMEDE'nin
        # hesabına yazılmaz (md 5.5.3.1 yeniden başlamada da aynı şey geçerli).
        if state is not self._kilit_durum:
            self._kilit_durum = state
            self._kilit_baslangic = simdi
            self._kilit_uyarildi = False
            self._kilit_teshis = ""
            return

        esik = (
            self._boot_uyari_s if state is MissionState.BOOT
            else self._bekleme_uyari_s
        )
        if esik <= 0.0:
            return
        gecen = simdi - self._kilit_baslangic
        if gecen < esik:
            return

        teshis, ayrinti = self._kilit_sebebi(state)
        self._kilit_teshis = teshis
        if not self._kilit_uyarildi:
            self._kilit_uyarildi = True
            self.get_logger().error(
                f"🔴 {state.value} durumunda {gecen:.0f}s takili kalindi — "
                f"GOREV ILERLEMIYOR. {ayrinti} (KAR-03/KAR-08: topic'ler "
                "akmaya devam ettigi icin bu arizanin 25 dakika fark "
                "edilmedigi bir oturum yasandi)"
            )

    def _kilit_sebebi(self, state: MissionState) -> tuple[str, str]:
        """Takılma sebebini kısa etiket + operatör talimatı olarak ver."""
        if state is MissionState.BOOT:
            if not self._mavros_mesaji_geldi:
                return "MAVROS-YOK", (
                    "/mavros/state HIC gelmedi — MAVROS dugumu kosuyor mu, "
                    "ROS_DOMAIN_ID dogru mu? (`ros2 node list`)"
                )
            return "FCU-KOPUK", (
                "/mavros/state geliyor ama connected=false — MAVROS ayakta, "
                "FCU hatti olu: kablo / fcu_url portu / baud / Pixhawk gucu"
            )

        # ARM ve BEKLEMEDE — ikisinde de ilk şart arm'dır.
        if not self._mav_armed:
            return "ARM-YOK", (
                "arac ARM edilmemis — gorev ARM olmadan baslamaz. Mission "
                "Planner'da pre-arm uyarilarina bak (PAR-03: 14 oturumun "
                "hicbirinde armed=true olmadi, sorun burada dugumleniyor). "
                "🔑 Pre-arm ret sebebi /mavros/statustext/recv'de gorunur — "
                "12.08'de canli olarak 'PreArm: Logging failed' bulundu "
                "(FC SD kartina yazamiyor)."
            )
        if state is MissionState.ARM:
            # Arm var ama ARM'dan cikilamiyor: kill_switch_off = _mav_armed
            # oldugu icin bu normalde imkansiz. Gorulurse gozlem akisi bozuk.
            return "GECIS-YOK", (
                "arac ARMED ama FSM ARM'dan BEKLEMEDE'ye gecmedi — beklenmedik "
                "durum, /mavros/state akisi kesikli olabilir"
            )
        if self._start_mode and self._last_mode != self._start_mode:
            return "MOD-YOK", (
                f"ARM var ama mod='{self._last_mode}', beklenen "
                f"start_on_mode='{self._start_mode}' — mod eslesmezse gorev "
                "HIC baslamaz, current_target/cmd_vel hic yayinlanmaz "
                "(F-P.23: 16.07 gercek donanim testinde sessizce yasandi)"
            )
        return "BASLAT-YOK", (
            "ARM var, mod dogru, ama gorev yine de baslamadi — beklenmedik "
            "durum. Waypoint listesi bos olabilir; fsm_node log'una bak"
        )

    def _publish_statustext(self, state: MissionState) -> None:
        """Görev durumunu MAVLink STATUSTEXT ile YKİ'ye bildir (md 4.2).

        İki tetik: (a) durum DEĞİŞTİĞİNDE anında, (b) değişmese de
        `statustext_periyot_s`'de bir tazeleme. (b) olmadan operatör MP'yi
        geçişlerden sonra açtığında ekranı boş kalır — 11.08'de sahada tam
        bu yaşandı (bkz. `statustext_periyot_s` parametresinin gerekçesi).
        Her tick'te yollamak ise MAVLink hattını doldurur; STATUSTEXT metni
        MAVLink'te 50 karakterle sınırlı.

        Yayın tek yönlüdür (araç → YKİ): md 4.1 telemetriye izin veriyor, md
        5.5.3.1'in yasakladığı şey ters yön (YKİ → araç komut).
        """
        if self._pub_statustext is None:
            return

        # 🔴 ABONESİZ YAYIN SESSİZCE ÇÖPE GİDER. `girdap-karar` fsm_node ile
        # MAVROS'u birlikte başlatıyor; MAVROS'un `sys` eklentisi bu topic'e
        # abone olana kadar geçen sürede yollanan her mesaj KAYBOLUR — ve
        # açılış geçişleri (BOOT→ARM→BEKLEMEDE) tam o pencereye düşüyor.
        # Gönderilmiş SAYMIYORUZ: `_last_statustext` güncellenmeden dönülüyor,
        # böylece abone belirdiğinde bir sonraki tick aynı metni tekrar dener.
        if self._pub_statustext.get_subscription_count() == 0:
            if not self._statustext_abone_uyarildi:
                self._statustext_abone_uyarildi = True
                self.get_logger().warn(
                    "STATUSTEXT abonesi yok (MAVROS henuz hazir degil?) — "
                    "YKI ekraninda gorev durumu GORUNMEYECEK. Abone "
                    "belirince kendiliginden tekrar denenecek."
                )
            return
        if self._statustext_abone_uyarildi:
            self._statustext_abone_uyarildi = False
            self.get_logger().info("STATUSTEXT abonesi hazir — YKI ekrani canli.")
        # PARKUR* durumlarında parkur katmanını da göster (ikisi ayrı otorite:
        # MissionFSM görev yaşam döngüsü, ParkurTransitionLogic waypoint
        # ilerlemesi — sahada ayrıştıklarında bunu görmek teşhis için kritik).
        # 🔴 14.08 — TEK ATIŞLIK İLAN EN ÖNDE (metin TEK YERDE atanır).
        #
        # ⚠ Buradaki ilk teşhisim YANLIŞTI, kayda geçsin: "parkurdayken ilan
        # operatöre hiç ulaşmıyor" demiştim; ulaşıyordu — aşağıdaki SEVİYE
        # zinciri `text`i ilanla yeniden atıyordu. Mutasyon turu yakaladı
        # (düzeltmeyi geri aldım, test yine geçti → test de iddia da boştu).
        #
        # GERÇEK KUSUR daha küçük ama gerçek: kısma/yineleme kararı
        # (`degisti` + `statustext_periyot_s`) DURUM metnine göre veriliyor,
        # yayınlanan ise İLAN oluyordu. Araç bir parkurda dururken durum metni
        # değişmediği için ilan **tazeleme periyodu (10 s) dolana kadar
        # bekliyordu**; üstelik `_last_statustext` hiç yayınlanmamış bir
        # metni kaydediyordu. İlan tam da beklememesi gereken şeydir.
        #
        # ⇒ İlan en öne alındı: karar da yayın da AYNI metin üzerinden.
        # Seviye zincirindeki yedek `text` ataması kaldırıldı — iki ayrı yerde
        # metin atamak, kaptanın 14.08'de ölü dal olarak yakaladığı hatanın
        # tam da kaynağıydı; tek atama noktası bırakıldı.
        if self._senkron_ilani:
            # Kilit teşhisinden ÖNCE gelir çünkü bu bir KURULUM doğrulaması —
            # operatör görevi yükledikten hemen sonra görmeli.
            text = f"GIRDAP {self._senkron_ilani}"
        elif state in (
            MissionState.PARKUR1, MissionState.PARKUR2, MissionState.PARKUR3
        ):
            text = f"GIRDAP {state.value} {self._parkur.state.value}"
        elif self._kilit_teshis:
            # KAR-03/KAR-08: yalnız "BOOT" / "BEKLEMEDE" yazmak operatöre
            # HİÇBİR ŞEY söylemiyor — sebep aynı satırda gitmeli.
            # ⚠ MAVLink STATUSTEXT 50 karakter: "GIRDAP BEKLEMEDE TAKILDI
            # BASLAT-YOK" = 41, en uzun kombinasyon sığıyor (test donduruyor).
            text = f"GIRDAP {state.value} TAKILDI {self._kilit_teshis}"
        else:
            text = f"GIRDAP {state.value}"
        simdi = self._saat()
        degisti = text != self._last_statustext
        if not degisti:
            # Değişmedi → yalnız tazeleme periyodu doldu mu diye bak.
            if self._statustext_periyot_s <= 0.0:
                return                      # tazeleme kapalı (eski davranış)
            if (
                self._statustext_son_gonderim is not None
                and simdi - self._statustext_son_gonderim
                < self._statustext_periyot_s
            ):
                return
        self._last_statustext = text
        self._statustext_son_gonderim = simdi

        msg = StatusText()
        msg.header.stamp = self.get_clock().now().to_msg()
        # KILL operatörün ANINDA görmesi gereken tek durum → kırmızı seviye.
        # KAR-03 BOOT kilidi de görev-durduran bir arıza; NOTICE seviyesinde
        # Mission Planner mesaj akışında diğer satırların arasında kaybolur.
        if state is MissionState.KILL:
            msg.severity = StatusText.CRITICAL
        elif self._senkron_ilani:
            # ⚠ Burada `text` ATANMAZ — metin yukarıdaki tek noktada kuruldu.
            # 🔴 14.08 — SEVİYE BURADA ATANMALI. Önceden ayrı bir
            # `elif self._senkron_ilani and "YOK" in ...` dalı vardı; koşulu bu
            # dalın ALT KÜMESİ olduğu için **hiç çalışmıyordu** (ölü kod) →
            # `severity` atanmadan kalıyor, varsayılan **0 = EMERGENCY** oluyordu.
            # Sonuç: OLUMLU teyit (`SENKRON OK`) bile MAVLink'in EN YÜKSEK
            # seviyesinden gidiyordu. Bu §0.58b'nin "gerçek arızayı gölgeleme"
            # kuralının ihlali: her görev yüklemesinde EMERGENCY gören operatör
            # gerçek acil durumu ayırt edemez.
            # 14.08: sezgi (metinde "YOK" geçiyor mu) TEK kaynak olamaz —
            # "PARKUR2 EKSIK GECIT 1<2" da ciddidir ama içinde "YOK" yok.
            # `_ilan_ciddi` açıkça verilmişse o kazanır; verilmemişse eski
            # sezgi aynen çalışır (kaptanın 14.08 düzeltmesi korunur).
            ciddi = (
                self._ilan_ciddi if self._ilan_ciddi is not None
                else ("YOK" in self._senkron_ilani)
            )
            msg.severity = StatusText.ERROR if ciddi else StatusText.NOTICE
        elif self._kilit_teshis:
            msg.severity = StatusText.ERROR
        else:
            msg.severity = StatusText.NOTICE
        msg.text = text[:50]
        self._pub_statustext.publish(msg)
        # Tek atışlık ilan gönderildi → düşür; bir sonraki tick normal duruma
        # döner. Sürekli basmak, asıl görev durumunu ekrandan siler.
        self._senkron_ilani = None
        self._ilan_ciddi = None


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

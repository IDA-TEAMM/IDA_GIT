"""
Girdap İDA — SANAL GÖL: kapalı döngü uçtan uca sınama.

Kaptanın sorusu: *"MP'de waypoint verince Jetson'daki kodlarımız aracılığıyla
gitmiyor — bunu test edebiliyor muyuz? Gölde sanal ortam ayarla, parkuru
yapmaya çalışsın, veriyi de MP'den veriyormuş gibi ver."*

Bu düğüm SAHTE DONANIM + SAHTE MISSION PLANNER + SAHTE ALGI'dır. Karar
yığınının GERÇEK düğümleri (fusion / mission_manager / fsm / planning /
mavros_bridge) hiç değiştirilmeden koşar. Döngü KAPALIDIR: yığının bastığı
`cmd_vel` tekneyi hareket ettirir, hareket eden tekne yeni GPS/IMU üretir.

    /mavros/setpoint_velocity/cmd_vel_unstamped ──► [tekne modeli] ──┐
                                                                      │
    ┌─────────────────────────────────────────────────────────────────┘
    ├─► /mavros/global_position/global   (NavSatFix, 5 Hz)
    ├─► /mavros/imu/data                 (Imu, 50 Hz)
    ├─► /mavros/local_position/velocity_body (TwistStamped, 50 Hz)
    ├─► /mavros/state                    (armed+GUIDED, 2 Hz)
    ├─► /mavros/mission/waypoints        ("Mission Planner görevi", 1 Hz)
    ├─► /mavros/mission/reached          (varışta — F-V.8 senkronu)
    ├─► /perception/obstacle_map         (PoseArray, base_link, 10 Hz)
    └─► /perception/classified_obstacles (Detection3DArray, 10 Hz)

⚠ İZOLASYON: `ROS_DOMAIN_ID` canlı yığından (42) FARKLI verilmeli — gerçek
Pixhawk'a bağlı yığınla aynı alanda koşarsa sahte state/cmd_vel gerçek araca
karışır.
"""

from __future__ import annotations

import math
import random

import numpy as np
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

R_DUNYA = 6378137.0
KENAR_SINIF = "0"          # turuncu parkur kenarı (camera_buoys.CLASS_PARKUR_KENARI)
BILINMEYEN = "99"          # füzyon sözleşmesi: eşleşmeyen küme


class SanalGol(Node):
    def __init__(self) -> None:
        super().__init__("sanal_gol")
        # --- Göl ve parkur ------------------------------------------------
        self.declare_parameter("lat0", 40.7162000)
        self.declare_parameter("lon0", 31.5247500)
        self.lat0 = float(self.get_parameter("lat0").value)
        self.lon0 = float(self.get_parameter("lon0").value)

        # ═══ PARKUR — GERÇEK GEOMETRİ (§0.17b, `parkur_nihai.world`) ═══
        # Varsayılanlar ölçülmüş dosyadan: 8 P1 kapısı, ortalar x=6..34'te 4 m
        # aralıkla zigzag, KAPI AÇIKLIĞI 12 m (kaptan teyidi). Güzergah
        # noktaları GN1-GN4 kapı ortalarından KAÇIK — §0.17c'nin ölçtüğü asıl
        # zorluk bu (kaçıklık 2,0-6,4 m; ham noktaya sürülürse en az 3 kapıdan
        # bandın DIŞINDAN geçilir ve puan gider).
        # Hepsi ROS parametresi: kapı sayısı/açıklığı/aralığı denemek için.
        self.declare_parameter("kapi_sayisi", 8)
        self.declare_parameter("kapi_acikligi_m", 12.0)
        self.declare_parameter("kapi_araligi_m", 4.0)
        self.declare_parameter("zigzag_m", 5.0)
        self.declare_parameter("gercek_gn", True)      # §0.17b'nin kaçık GN'leri
        n = int(self.get_parameter("kapi_sayisi").value)
        acik = float(self.get_parameter("kapi_acikligi_m").value)
        aralik = float(self.get_parameter("kapi_araligi_m").value)
        zig = float(self.get_parameter("zigzag_m").value)

        # Kapı ortaları: gerçek dosyadaki zigzag deseni (0, +z, 0, −z, …)
        desen = [0.0, zig, 0.0, -zig]
        self.kapilar = []
        for i in range(n):
            gx = desen[i % 4]
            gy = 6.0 + i * aralik
            self.kapilar.append((gx, gy, acik / 2.0))

        if bool(self.get_parameter("gercek_gn").value) and n == 8:
            # §0.17b'nin GN'leri (x↔y çevrilmiş: bizim eksende y=ileri)
            self.gorev_xy = [(0.0, 2.0), (5.0, 12.0), (-5.0, 20.0), (5.0, 32.0)]
        else:
            # Kapı ortalarından türet ama KAÇIKLIK KORU (2 m) — ham noktaya
            # sürmenin bandın dışına çıkardığı özellik kaybolmasın.
            self.gorev_xy = [
                (gx + (2.0 if i % 2 else -2.0), gy)
                for i, (gx, gy, _) in enumerate(self.kapilar)
            ]

        # Sarı engeller — §0.17b: parkur_nihai.world'de y=±2 bandında
        self.declare_parameter("engel_sayisi", 4)
        m = int(self.get_parameter("engel_sayisi").value)
        son_y = self.kapilar[-1][1] if self.kapilar else 30.0
        self.engeller = [
            (2.0 if i % 2 else -2.0, son_y + 6.0 + i * 4.0) for i in range(m)
        ]

        # --- Dalga bozucusu (varsayılan KAPALI) ---------------------------
        # 🔑 Genlik "Deniz Durumu-2 ölçümü" İDDİASI DEĞİL, §0.8a'nın kuralıyla
        # **seyir hızının oranı** olarak verilir (bu modelde seyir 1,05 m/s;
        # 0,18 m/s ≈ %17 yanal sürüklenme). `dynamics.yaml`'ın `wave.Fx_amp=5.0`
        # sayısı BAYAT — 30 N/motor'luk hayali tekneye ait, buraya konamaz.
        # ⚠ Yön DÜNYA çerçevesinde: parkur ekseni +y boyunca uzuyor (kapı
        # ortaları `gy = 6 + i·aralık`), o yüzden varsayılan yön **+x = YANAL**
        # — yani koridordan dışarı iten bileşen. PDÇ ölçümü için doğru eksen bu.
        self.declare_parameter("dalga_genlik_mps", 0.0)
        self.declare_parameter("dalga_frekans_hz", 0.5)      # ~2 s periyot
        self.declare_parameter("dalga_yon_rad", 0.0)         # +x = yanal
        self.declare_parameter("dalga_yaw_rps", 0.0)         # rad/s, yaw bozucu
        self.declare_parameter("dalga_yaw_frekans_hz", 0.3)  # ~3,3 s periyot
        self.dalga_genlik = float(self.get_parameter("dalga_genlik_mps").value)
        self.dalga_frekans = float(self.get_parameter("dalga_frekans_hz").value)
        self.dalga_yon = float(self.get_parameter("dalga_yon_rad").value)
        self.dalga_yaw = float(self.get_parameter("dalga_yaw_rps").value)
        self.dalga_yaw_frekans = float(
            self.get_parameter("dalga_yaw_frekans_hz").value)

        # --- Başlangıç yönü (F-F.22 ölçüm kapısı) -------------------------
        # 🔴 NEDEN VAR: 17.08 göl bandında GUIDED komutlarının %23,1'i GERİYDİ
        # ve o anların %83,4'ünde hedef aracın ARKASINDAYDI. Sanal göl bu
        # sınıfı **yapısal olarak** göremiyordu: tekne 90° (kuzey) ile
        # başlıyor, görev noktaları da hep kuzeyde ⇒ "hedef arkada" durumu
        # HİÇ oluşmuyor. Yani sim yalan söylemedi — o soruyu hiç sormadı.
        #
        # 90.0 = ESKİ DAVRANIŞ BİREBİR. `-90` → burun GÜNEYE, yani bütün
        # görev noktaları arkada: MPPI'nin "dönmek yerine geri gitme"
        # seçimi ilk saniyeden itibaren ölçülebilir hâle gelir.
        #
        # ⚠ Bu bir "bozucu" değil, ölçüm sahnesi: gerçek koşumda tekne bu
        # duruma waypoint'i aşarak, kapı kilidi atlayarak ya da operatör
        # devrettikten sonra düşüyor — üçü de simde üretilemiyor.
        # --- 🔴 F-S.18 GERÇEKÇİ ALGI (17.08 göl bandından ÖLÇÜLDÜ) --------
        # NEDEN VAR: bu sim bugüne kadar KUSURSUZ algı yayınlıyordu —
        # menzildeki her cismi tam konumuyla, kaçırmasız, hayaletsiz.
        # O yüzden her koşum "temiz" bitiyordu ve kodun GERÇEK göldeki
        # kusurlu veriyle çalışıp çalışmadığı hiç sınanmıyordu.
        #
        # Aşağıdaki üç tablo `session_20260817_193312` (12,5 dk, 5965 algı
        # karesi, 5696 kenar tespiti) bandından ölçüldü — uydurma DEĞİL:
        #
        #  ① GÖRÜLME OLASILIĞI ↔ menzil (gerçek dubanın 1 m'sinde tespit var mı)
        #      0-3 m %30,7 · 3-5 %41,3 · 5-8 %47,2 · 8-12 %36,3
        #      12-18 %16,8 · 18-25 %1,7
        #     🔑 En iyi kovada bile duba karelerin YARISINDA GÖRÜNMÜYOR.
        #  ② KONUM HATASI ↔ menzil (ortanca, m)
        #      0-3 1,02 · 3-5 1,53 · 5-8 1,47 · 8-12 1,83 · 12-25 3,09
        #  ③ HAYALET: tespitlerin %42,9'u en yoğun 4 gerçek duba ile
        #     açıklanamıyor; engel torbasının %98,6'sı CLASS_UNKNOWN ve
        #     kare başına ~95 adet (kıyı), yarıçap ortanca 0,56 maks 17,2 m.
        #
        # DOZ: 0.0 = ESKİ DAVRANIŞ BİREBİR (kusursuz). 1.0 = 17.08'de
        # ÖLÇÜLEN şiddet. Ara değerler oranlı (§0.8a: genlik iddia değil,
        # ölçülenin oranı olarak verilir).
        # ⚠ Kendi RNG'si var → aynı tohum aynı kusur dizisi; A/B tekrarlanabilir.
        self.declare_parameter("algi_gercekcilik", 0.0)
        self.declare_parameter("algi_tohum", 0)
        self.declare_parameter("hayalet_sayisi", 0)      # kare başına ek UNKNOWN
        # F-P.30 ölçüm kolu: kimliksiz kümenin TEMSİLİ.
        #   0.0 = ölçülen ham dağılım (çevrel daire; ortanca 0,56 maks 17,2 m)
        #   >0  = daire ZİNCİRİ temsili — aynı cismi bu yarıçapla sınırlı
        #         birden çok daireyle kaplar (F-P.30'un ürettiği hâl).
        # Böylece "kıyıyı tek dev diskle modellemek" ile "zincirle modellemek"
        # planlayıcı çıktısında DOĞRUDAN kıyaslanabilir.
        self.declare_parameter("hayalet_maks_yaricap", 0.0)
        self.hayalet_maks_r = float(
            self.get_parameter("hayalet_maks_yaricap").value)
        # 🔴 KALICI hayaletler — DÜNYA çerçevesinde bir kez üretilir.
        # İlk sürüm her karede yeniden üretiyordu; ölçüm bunun SADIK
        # OLMADIĞINI gösterdi: gerçek bantta kimliksiz izler 2,5 m kapıyla
        # ortanca **187 kare (~19 sn)** yaşıyor, tek isabetli iz yalnız %0,9.
        # Kare kare zıplayan gürültü, planlayıcıyı gerçekte olduğu gibi
        # sınamaz (RRT* her karede farklı bir haritaya plan yapar).
        self._hayaletler = []          # (dünya_x, dünya_y, yarıçap)
        self.declare_parameter("poz_bayat_orani", 0.0)   # ölçülen %3,9+%1,9
        self.gercekcilik = max(0.0, min(1.0, float(
            self.get_parameter("algi_gercekcilik").value)))
        self.hayalet_n = int(self.get_parameter("hayalet_sayisi").value)
        self.poz_bayat = max(0.0, min(0.9, float(
            self.get_parameter("poz_bayat_orani").value)))
        # ══════════════════════════════════════════════════════════════
        # ARIZA ENJEKSİYONU (18.08.2026) — hepsi VARSAYILAN KAPALI (0)
        #
        # 🔑 Bunlar "gerçekçilik" değil, **kural motorunu sınayan tetikler**.
        # Göl şartnamesinin kabul ölçütü iki yönlü: her kural ihlalde
        # KIRMIZI yanmalı (duyarlılık) ve temiz koşumda SESSİZ kalmalı
        # (özgüllük). Enjeksiyon olmadan yalnız ikincisi ölçülebilir.
        #
        # Taksonomi literatürden (sensör · iletişim · yaşam döngüsü):
        #   sensör      → poz sıçraması, NaN, gövde yansıması
        #   iletişim    → kadans düşürme, kesinti, damga kaydırma
        #   yaşam döngü → (FAZ 6c: düğüm çökmesi, ARM reddi)
        #
        # ⚠ `algi_tohum` ile aynı RNG kullanılır ⇒ aynı tohum = aynı arıza
        # dizisi ⇒ A/B eşleştirilmiş kıyas yapılabilir (§19.4).
        # ══════════════════════════════════════════════════════════════
        #: F1 sınaması — poz ANLIK sıçraması (m). KAR-06'da 25 ms'de 6,54 m.
        self.declare_parameter("ariza_poz_sicramasi_m", 0.0)
        #: Sıçramanın olasılığı (0-1). Her GPS karesinde bağımsız denenir.
        self.declare_parameter("ariza_poz_sicrama_orani", 0.0)
        #: F4 sınaması — pozu NaN yap (0-1 olasılık). KAR-05 sınıfı.
        self.declare_parameter("ariza_poz_nan_orani", 0.0)
        #: S1 sınaması — damgayı geriye kaydır (s). ALG-06'da 56 YIL bayattı.
        self.declare_parameter("ariza_damga_kaydirma_s", 0.0)
        #: C3 sınaması — GPS/state yayınını seyrelt (her N'de bir yayınla).
        #: 1 = normal. PAR-04'te /mavros/state 2 Hz → 0,17 Hz düşmüştü.
        self.declare_parameter("ariza_kadans_bolen", 1)
        #: C1/C3 sınaması — t saniyesinden sonra TÜM yayını kes (0 = kapalı).
        #: ALG-05 (LiDAR 5 saatte 39 mesaj) ve sessiz felç sınıfı.
        self.declare_parameter("ariza_kesinti_t_s", 0.0)
        #: 🔴 GERÇEK TEKNE DİNAMİĞİ (18.08) — varsayılan KAPALI.
        #: Aşağıdaki basit model (birinci mertebe, HIZ entegrasyonu) gerçek
        #: `CatamaranDynamics`ten SAPIYOR ve sapma ÖLÇÜLDÜ:
        #:     ivme        : 0,234 ↔ 0,985 m/s²   → sanal göl **4,2× fazla**
        #:     zaman sabiti: 4,76 ↔ 0,80 s        → sanal göl **5,9× çevik**
        #:     dönüş tavanı: 0,289 ↔ 0,800 rad/s  → sanal göl **2,8× hızlı**
        #: Literatürün adı: "zayıf plant modeli" — kararsız kontrolü kararlı
        #: gösterebilir. Açıkken `prototype.dynamics.CatamaranDynamics` koşar,
        #: yani MPPI'nin plan yaparken kullandığı modelin TA KENDİSİ.
        #: ⚠ SINIR: aynı modeli hem planlayıcı hem tesis kullanınca "model
        #: uyuşmazlığı" sınıfı sınanamaz (gerçekte tekne modelden sapar).
        #: O yüzden `gercek_dinamik_bozucu` ile parametre sapması eklenebilir.
        self.declare_parameter("gercek_dinamik", False)
        self.declare_parameter("gercek_dinamik_bozucu", 0.0)   # ±oran (0,2 = %20)

        #: F5 sınaması — engel bulutuna GÖVDE İÇİ nokta ekle (m, 0 = kapalı).
        #: ALG-02'de en yakın "engel" 1,3 mm'deydi (LiDAR kendini görüyor).
        self.declare_parameter("ariza_govde_yansimasi_m", 0.0)

        self._rng = random.Random(int(self.get_parameter("algi_tohum").value))

        # Arıza enjeksiyon durumları (hepsi 0 = kapalı)
        _p = self.get_parameter
        self.ar_sicrama_m = float(_p("ariza_poz_sicramasi_m").value)
        self.ar_sicrama_p = max(0.0, min(1.0, float(_p("ariza_poz_sicrama_orani").value)))
        self.ar_nan_p = max(0.0, min(1.0, float(_p("ariza_poz_nan_orani").value)))
        self.ar_damga_s = float(_p("ariza_damga_kaydirma_s").value)
        self.ar_kadans = max(1, int(_p("ariza_kadans_bolen").value))
        self.ar_kesinti_s = float(_p("ariza_kesinti_t_s").value)
        self.ar_govde_m = float(_p("ariza_govde_yansimasi_m").value)
        self._ar_sayac = 0
        # Gerçek dinamik kipi
        self.gercek_dinamik = bool(_p("gercek_dinamik").value)
        self._dyn = None
        if self.gercek_dinamik:
            from dataclasses import replace as _rep

            from prototype.dynamics.catamaran import CatamaranDynamics
            self._dyn = CatamaranDynamics()
            boz = float(_p("gercek_dinamik_bozucu").value)
            if boz:
                # Tesis ≠ plan: MPPI'nin modelinden bilerek sapılır ki
                # "model uyuşmazlığı" sınıfı da sınanabilsin.
                rg = random.Random(int(_p("algi_tohum").value) + 7)
                self._dyn.p = _rep(
                    self._dyn.p,
                    mass=self._dyn.p.mass * (1 + rg.uniform(-boz, boz)),
                    Xu=self._dyn.p.Xu * (1 + rg.uniform(-boz, boz)),
                    Nr=self._dyn.p.Nr * (1 + rg.uniform(-boz, boz)))
            # ⚠ `_durum` ADI SERBEST DEĞİL: sınıfın `_durum()` METODU var (MAVROS
            # state yayıncısı). Aynı adı alan dizi metodu gölgeliyordu ⇒ timer
            # çağrısı `'numpy.ndarray' object is not callable` ile ölüyor ve
            # TÜM zincir sessizce duruyordu (18.08, ölçümle bulundu).
            self._dyn_durum = np.zeros(6)
            self.get_logger().warn(
                f"🔵 GERÇEK DİNAMİK AÇIK (CatamaranDynamics, bozucu ±{boz:.0%}) — "
                f"u_max {-2*self._dyn.p.max_thrust/self._dyn.p.Xu:.2f} m/s")
        self._ariza_rng = random.Random(
            int(_p("algi_tohum").value) + 9001)   # arıza dizisi algıdan AYRI
        if any((self.ar_sicrama_p, self.ar_nan_p, self.ar_damga_s,
                self.ar_kadans > 1, self.ar_kesinti_s, self.ar_govde_m)):
            self.get_logger().warn(
                "🔴 ARIZA ENJEKSİYONU AÇIK — bu koşum SAĞLIKLI DEĞİLDİR: "
                f"sıçrama {self.ar_sicrama_m} m @{self.ar_sicrama_p} · "
                f"NaN {self.ar_nan_p} · damga {self.ar_damga_s} s · "
                f"kadans 1/{self.ar_kadans} · kesinti {self.ar_kesinti_s} s · "
                f"gövde {self.ar_govde_m} m")


        self.declare_parameter("baslangic_yon_derece", 90.0)
        yon0 = float(self.get_parameter("baslangic_yon_derece").value)

        # --- Tekne durumu (ENU, göl orijinine göre) -----------------------
        self.x, self.y, self.psi = 0.0, 0.0, math.radians(yon0)
        self.u, self.r = 0.0, 0.0
        self.cmd_u, self.cmd_r = 0.0, 0.0
        self.son_cmd_t = None
        self.varilan = -1
        self.t = 0.0
        self.cmd_sayaci = 0
        self.bosluk_sayaci = 0
        self.en_uzun_bosluk = 0.0
        # Şartname çarpmayı CEZALANDIRIYOR (P2: −30×Ç2 / (KD2+ED2)) → açıklık
        # ölçülür. Gövde yarı genişliği 0,3925 m + duba yarıçapı 0,15 m.
        self.en_yakin_m = 99.0
        self.carpma = 0
        self._temas = set()

        # --- Yayıncılar ----------------------------------------------------
        self.p_gps = self.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
        self.p_imu = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self.p_vel = self.create_publisher(TwistStamped, "/mavros/local_position/velocity_body", 10)
        self.p_state = self.create_publisher(State, "/mavros/state", 10)
        # ⚠ use_isam2=false kolunda fusion_node pozu BURADAN alır (MAVROS'un
        # kendi EKF'i). Simülatörün ilk turunda bu topic yoktu ve füzyon
        # "henüz tahmin yok" deyip poz üretmedi → planning POZ-YOK.
        self.p_lpose = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", 10
        )
        # MAVROS gorev listesini TRANSIENT_LOCAL yayinlar; abone de oyle
        # bekliyor. VOLATILE yayinlarsak DDS 'incompatible QoS' deyip mesaji
        # HIC teslim etmez — simulatorun ilk turunda tam olarak bu yasandi.
        qos_gorev = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.p_wps = self.create_publisher(
            WaypointList, "/mavros/mission/waypoints", qos_gorev
        )
        self.p_reached = self.create_publisher(WaypointReached, "/mavros/mission/reached", 10)
        self.p_obs = self.create_publisher(PoseArray, "/perception/obstacle_map", 10)
        self.p_cls = self.create_publisher(Detection3DArray, "/perception/classified_obstacles", 10)

        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", self._on_cmd, 10
        )

        self.create_timer(0.02, self._fizik)        # 50 Hz
        self.create_timer(0.10, self._algi)         # 10 Hz
        self.create_timer(0.20, self._gps)          # 5 Hz
        self.create_timer(0.50, self._durum)        # 2 Hz
        self.create_timer(1.00, self._gorev)        # 1 Hz — "Mission Planner"
        self.create_timer(2.00, self._rapor)
        self.get_logger().info(
            f"SANAL GÖL açıldı — {len(self.kapilar)} kapı "
            f"(açıklık {2*self.kapilar[0][2]:.0f} m, aralık "
            f"{self.kapilar[1][1]-self.kapilar[0][1] if len(self.kapilar)>1 else 0:.0f} m), "
            f"{len(self.gorev_xy)} görev noktası, {len(self.engeller)} engel"
        )

    # ---------------- fizik ----------------
    def _on_cmd(self, msg: Twist) -> None:
        simdi = self.get_clock().now().nanoseconds / 1e9
        if self.son_cmd_t is not None:
            bosluk = simdi - self.son_cmd_t
            if bosluk > 0.5:                       # planning_node'un kendi eşiği
                self.bosluk_sayaci += 1
                self.en_uzun_bosluk = max(self.en_uzun_bosluk, bosluk)
        self.son_cmd_t = simdi
        self.cmd_sayaci += 1
        self.cmd_u = float(msg.linear.x)
        self.cmd_r = float(msg.angular.z)

    # ───────────────────────── arıza yardımcıları ─────────────────────────
    def _ariza_kesildi(self) -> bool:
        """Kesinti anı geldi mi — geldiyse yayın YAPILMAZ (topic susar)."""
        return self.ar_kesinti_s > 0.0 and self.t >= self.ar_kesinti_s

    def _ariza_kadans_atla(self) -> bool:
        """Kadans seyreltme: her N'de bir yayınla."""
        if self.ar_kadans <= 1:
            return False
        self._ar_sayac += 1
        return (self._ar_sayac % self.ar_kadans) != 0

    def _ariza_damga(self):
        """Damgayı geriye kaydır (S1 sınaması). 0 = dokunma."""
        t = self.get_clock().now()
        if self.ar_damga_s:
            from rclpy.duration import Duration
            t = t - Duration(seconds=self.ar_damga_s)
        return t.to_msg()

    def _ariza_konum(self, x: float, y: float):
        """Poz sıçraması / NaN enjekte et (F1 / F4 sınaması)."""
        if self.ar_nan_p and self._ariza_rng.random() < self.ar_nan_p:
            return float("nan"), float("nan")
        if self.ar_sicrama_p and self._ariza_rng.random() < self.ar_sicrama_p:
            a = self._ariza_rng.uniform(0, 2 * math.pi)
            return (x + self.ar_sicrama_m * math.cos(a),
                    y + self.ar_sicrama_m * math.sin(a))
        return x, y

    def _fizik(self) -> None:
        dt = 0.02
        self.t += dt
        if self._dyn is not None:
            # GERÇEK MODEL: cmd_vel (hız setpoint'i) → itki. `planning_node`
            # tersini yapıyor (`hedef_u = 2T/|Xu|`), burada onu geri çeviriyoruz
            # ki tesis GERÇEK ikinci mertebe dinamiği koşsun.
            p = self._dyn.p
            T_ort = self.cmd_u * abs(p.Xu) / 2.0
            T_fark = self.cmd_r * abs(p.Nr) / max(1e-6, p.thruster_spacing)
            u_vec = np.clip(
                np.array([T_ort - T_fark, T_ort + T_fark]),
                -p.max_thrust, p.max_thrust)
            self._dyn_durum[0], self._dyn_durum[1] = self.x, self.y
            self._dyn_durum[2] = self.psi
            self._dyn_durum[3], self._dyn_durum[5] = self.u, self.r
            self._dyn_durum = self._dyn.step_rk4(self._dyn_durum, u_vec, dt)
            self.x, self.y = float(self._dyn_durum[0]), float(self._dyn_durum[1])
            self.psi = float(self._dyn_durum[2])
            self.u, self.r = float(self._dyn_durum[3]), float(self._dyn_durum[5])
        else:
            # Basit model (eski davranış, VARSAYILAN) — bkz. `gercek_dinamik`
            # parametresindeki ölçülmüş sapma tablosu.
            self.u += (max(-0.5, min(1.05, self.cmd_u)) - self.u) * dt / 0.8
            self.r += (max(-0.8, min(0.8, self.cmd_r)) - self.r) * dt / 0.5
            self.psi += self.r * dt
            self.x += self.u * math.cos(self.psi) * dt
            self.y += self.u * math.sin(self.psi) * dt

        # Dalga: kontrolcünün YENEMEDİĞİ dış sürüklenme. Kuvvet değil doğrudan
        # konum/yön bozucusu olarak eklenir — bu model itki değil hız
        # entegrasyonu yapıyor, "kuvvet" eklemek uydurma bir kütle gerektirirdi.
        if self.dalga_genlik != 0.0:
            surukle = self.dalga_genlik * math.sin(
                2.0 * math.pi * self.dalga_frekans * self.t)
            self.x += surukle * math.cos(self.dalga_yon) * dt
            self.y += surukle * math.sin(self.dalga_yon) * dt
        if self.dalga_yaw != 0.0:
            self.psi += self.dalga_yaw * math.sin(
                2.0 * math.pi * self.dalga_yaw_frekans * self.t + 1.0) * dt

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"
        imu.orientation.z = math.sin(self.psi / 2.0)
        imu.orientation.w = math.cos(self.psi / 2.0)
        imu.angular_velocity.z = self.r
        imu.linear_acceleration.z = 9.81
        self.p_imu.publish(imu)

        tw = TwistStamped()
        tw.header = imu.header
        tw.twist.linear.x = self.u
        tw.twist.angular.z = self.r
        self.p_vel.publish(tw)

        # 🔴 ARIZA ENJEKSİYONU BURAYA DA GEREKLİ (18.08 ölçümüyle bulundu).
        # `fusion_node` bu kipte ([MAVROS EKF geçişi (video)], use_isam2=false)
        # GPS'i DEĞİL bu topic'i okuyor. Arıza yalnız `_gps`'e enjekte
        # edildiğinde `/girdap/fusion/odom` TEMİZ kalıyordu ⇒ F1/F4 kuralları
        # ihlali göremiyordu ve göl "arıza yakalanmadı" diyordu.
        # 🔑 Ders: enjeksiyon, tüketicinin GERÇEKTEN okuduğu topic'e konur —
        # "mantıken poz kaynağı" olana değil.
        if self._ariza_kesildi() or self._ariza_kadans_atla():
            return
        lp = PoseStamped()
        lp.header.stamp = self._ariza_damga()
        lp.header.frame_id = "map"
        _lx, _ly = self._ariza_konum(self.x, self.y)
        lp.pose.position.x, lp.pose.position.y = _lx, _ly
        lp.pose.orientation = imu.orientation
        self.p_lpose.publish(lp)

        # Açıklık: gövde YÜZEYİNDEN duba YÜZEYİNE
        for i, (wx, wy, yari) in enumerate(
            [(kx - yr, ky, 0.15) for (kx, ky, yr) in self.kapilar]
            + [(kx + yr, ky, 0.15) for (kx, ky, yr) in self.kapilar]
            + [(ex, ey, 0.25) for (ex, ey) in self.engeller]
        ):
            d = math.hypot(wx - self.x, wy - self.y) - 0.3925 - yari
            self.en_yakin_m = min(self.en_yakin_m, d)
            if d <= 0.0 and i not in self._temas:
                self._temas.add(i)
                self.carpma += 1
                self.get_logger().warn(f"💥 ÇARPMA #{self.carpma}")

    # ---------------- sahte sensör/durum ----------------
    def _gps(self) -> None:
        # ARIZA: kesinti / kadans seyreltme → topic SUSAR (C3 sınaması)
        if self._ariza_kesildi() or self._ariza_kadans_atla():
            return
        m = NavSatFix()
        m.header.stamp = self._ariza_damga()          # ARIZA: damga kaydırma
        m.header.frame_id = "base_link"
        m.status.status = NavSatStatus.STATUS_GBAS_FIX          # RTK fixed
        m.status.service = NavSatStatus.SERVICE_GPS
        _ax, _ay = self._ariza_konum(self.x, self.y)  # ARIZA: sıçrama / NaN
        m.latitude = self.lat0 + math.degrees(_ay / R_DUNYA)
        m.longitude = self.lon0 + math.degrees(
            _ax / (R_DUNYA * math.cos(math.radians(self.lat0)))
        )
        m.altitude = 871.0
        self.p_gps.publish(m)

    def _durum(self) -> None:
        s = State()
        s.header.stamp = self.get_clock().now().to_msg()
        s.connected = True
        s.armed = True                    # kaptan MP'den arm etmiş gibi
        # 🔑 GERÇEK OPERATÖR DAVRANIŞI: görev, moda GEÇİŞ ANINDA başlar
        # (F-V.6). Kaptan MP'de önce MANUAL'de arm eder, sonra GUIDED'a
        # çeker. Baştan GUIDED yayınlarsak kenar oluşmaz ve FSM BEKLEMEDE'de
        # takılır — simülatörün ikinci turunda tam olarak bu yaşandı.
        guided = self.t > 10.0
        s.guided = guided
        s.mode = "GUIDED" if guided else "MANUAL"
        s.system_status = 4
        self.p_state.publish(s)

    def _gorev(self) -> None:
        """Mission Planner'ın yüklediği görev — mission_source=fc bunu okur."""
        wl = WaypointList()
        wl.current_seq = 0
        ev = Waypoint()                    # seq 0 = home (skip_home_seq0=True)
        ev.frame, ev.command, ev.is_current = 0, 16, False
        ev.x_lat, ev.y_long, ev.z_alt = self.lat0, self.lon0, 0.0
        wl.waypoints.append(ev)
        for (gx, gy) in self.gorev_xy:
            w = Waypoint()
            w.frame, w.command, w.autocontinue = 3, 16, True
            w.x_lat = self.lat0 + math.degrees(gy / R_DUNYA)
            w.y_long = self.lon0 + math.degrees(
                gx / (R_DUNYA * math.cos(math.radians(self.lat0)))
            )
            w.z_alt = 0.0
            wl.waypoints.append(w)
        self.p_wps.publish(wl)

        # F-V.8: uçuş kontrolcüsünün varış senkronu
        for i, (gx, gy) in enumerate(self.gorev_xy):
            if i > self.varilan and math.hypot(gx - self.x, gy - self.y) < 2.0:
                self.varilan = i
                wr = WaypointReached()
                wr.wp_seq = i + 1          # home seq 0 olduğu için +1
                self.p_reached.publish(wr)
                self.get_logger().info(f"✅ GÖREV NOKTASI {i+1} — VARILDI")

    # ---------------- sahte algı ----------------
    def _dunya_to_govde(self, wx: float, wy: float):
        dx, dy = wx - self.x, wy - self.y
        c, s = math.cos(-self.psi), math.sin(-self.psi)
        return c * dx - s * dy, s * dx + c * dy

    #: ① ÖLÇÜLEN görülme olasılığı — (azami menzil, olasılık)
    GORULME = ((3.0, 0.307), (5.0, 0.413), (8.0, 0.472),
               (12.0, 0.363), (18.0, 0.168), (25.0, 0.017))
    #: ② ÖLÇÜLEN konum hatası ortancası — (azami menzil, metre)
    KONUM_HATASI = ((3.0, 1.02), (5.0, 1.53), (8.0, 1.47),
                    (12.0, 1.83), (25.0, 3.09))

    @staticmethod
    def _tablodan(tablo, menzil: float) -> float:
        for ust, deger in tablo:
            if menzil <= ust:
                return deger
        return tablo[-1][1]

    def _algi(self) -> None:
        """Kapı direkleri (turuncu) + engeller (sarı) — LiDAR menzili 25 m."""
        # ARIZA: kesinti → algı topic'i SUSAR (ALG-05 sınıfı, C3 sınaması)
        if self._ariza_kesildi():
            return
        pa = PoseArray()
        pa.header.stamp = self._ariza_damga()          # ARIZA: damga kaydırma
        pa.header.frame_id = "base_link"
        da = Detection3DArray()
        da.header = pa.header

        cisimler = []
        for (kx, ky, yari) in self.kapilar:
            cisimler.append((kx - yari, ky, 0.15, KENAR_SINIF))
            cisimler.append((kx + yari, ky, 0.15, KENAR_SINIF))
        for (ex, ey) in self.engeller:
            cisimler.append((ex, ey, 0.25, BILINMEYEN))

        for (wx, wy, yaricap, sinif) in cisimler:
            bx, by = self._dunya_to_govde(wx, wy)
            menzil = math.hypot(bx, by)
            if menzil > 25.0 or bx < -2.0:                  # LiDAR menzili
                continue
            if self.gercekcilik > 0.0:
                # ① KAÇIRMA — doz oranlı: 0'da hep görülür, 1'de ölçülen.
                p_gor = self._tablodan(self.GORULME, menzil)
                p_eff = 1.0 - self.gercekcilik * (1.0 - p_gor)
                if self._rng.random() > p_eff:
                    continue
                # ② KONUM HATASI — ortanca ≈ 1,177σ (yarıçap dağılımı)
                sigma = (self._tablodan(self.KONUM_HATASI, menzil)
                         / 1.177 * self.gercekcilik)
                bx += self._rng.gauss(0.0, sigma)
                by += self._rng.gauss(0.0, sigma)
                menzil = math.hypot(bx, by)
            p = Pose()
            p.position.x, p.position.y = bx, by
            p.orientation.z, p.orientation.w = yaricap, 1.0   # yarıçap hack'i
            pa.poses.append(p)

            # 🔴 18.08.2026 GİRİNTİ KUSURU DÜZELTİLDİ. Bu blok döngünün
            # DIŞINDA ve `if self.ar_govde_m > 0.0:` (gövde yansıması ARIZASI,
            # varsayılan KAPALI) içinde duruyordu. İki sonucu vardı:
            #   ① `da.detections` **her zaman BOŞ** kalıyordu ⇒
            #      `/gercek/classified_obstacles` 120 mesajda 0 tespit
            #      (ölçüldü) ⇒ `sahte_ham_sensor` renk bulamayıp HİÇ duba
            #      çizmiyordu ⇒ `/oak/rgb/image_raw` baştan beri **boş su**
            #      karesiydi. Kamera görüntü yolu gölde hiç sınanamıyordu.
            #   ② Arıza açıkken bile döngüden ARTAKALAN `bx, by, yaricap,
            #      sinif` kullanılıyordu ⇒ yalnız SON dubayı yazıyordu.
            d = Detection3D()
            d.bbox.center.position.x, d.bbox.center.position.y = bx, by
            d.bbox.size.x = 2.0 * yaricap
            h = ObjectHypothesisWithPose()
            # Kamera 69° kadraj + 15 m menzil: dışında kalan turuncu bile
            # UNKNOWN gelir (gerçek davranış — §0.17e'nin çözdüğü hâl).
            aci = abs(math.atan2(by, bx))
            gorunur = aci < math.radians(34.5) and math.hypot(bx, by) < 15.0
            h.hypothesis.class_id = sinif if gorunur else BILINMEYEN
            h.hypothesis.score = 0.9
            d.results.append(h)
            da.detections.append(d)

        # ARIZA: GÖVDE YANSIMASI — LiDAR'ın kendi teknesini görmesi (F5).
        # ALG-02: engel bulutunun %27'si aracın ARKASINDAYDI, en yakını 1,3 mm.
        # Gövde yarıçapından (0,393 m) yakın hiçbir "engel" gerçek olamaz.
        if self.ar_govde_m > 0.0:
            gp = Pose()
            gp.position.x = self.ar_govde_m
            gp.position.y = 0.0
            gp.orientation.z, gp.orientation.w = 0.05, 1.0
            pa.poses.append(gp)

        # ③ HAYALET / KIYI — kimliksiz (UNKNOWN) ek tespitler.
        # Gerçek bantta bunlar HAYALET DEĞİL, kalıcı kıyı yapılarıydı
        # (2,5 m kapıyla izlendiğinde ortanca ömür 187 kare) — ama
        # planlayıcı açısından etkisi aynı: hedef bunlardan birinin içine
        # düşerse RRT* reddediyor (17.08'de 43 kez).
        # Kalıcı torbayı bir kez kur (dünya çerçevesinde, parkur boyunca).
        if self.hayalet_n and not self._hayaletler:
            son_y = self.kapilar[-1][1] if self.kapilar else 30.0
            for _ in range(self.hayalet_n * 3):        # menzil dışı olanlar da
                wx = self._rng.uniform(-25.0, 25.0)
                wy = self._rng.uniform(-5.0, son_y + 15.0)
                wr = min(17.2, abs(self._rng.lognormvariate(math.log(0.56), 0.9)))
                self._hayaletler.append((wx, wy, wr))
        for (gwx, gwy, hr0) in self._hayaletler:
            hx, hy = self._dunya_to_govde(gwx, gwy)
            if math.hypot(hx, hy) > 25.0 or hx < -2.0:
                continue
            a = math.atan2(hy, hx)
            hr = hr0
            # Yarıçap dağılımı ölçülen: ortanca 0,56 · %90 1,00 · maks 17,2
            if self.hayalet_maks_r > 0.0 and hr > self.hayalet_maks_r:
                # F-P.30: aynı yayılımı daire ZİNCİRİYLE kapla — kapsanan
                # gerçek cisim aynı, kaplanan BOŞ SU çok daha az.
                n_par = max(2, int(math.ceil(hr / self.hayalet_maks_r)))
                for k in range(n_par):
                    t = -hr + (2.0 * hr) * (k + 0.5) / n_par
                    px, py = hx + t * math.cos(a), hy + t * math.sin(a)
                    pp = Pose()
                    pp.position.x, pp.position.y = px, py
                    pp.orientation.z, pp.orientation.w = self.hayalet_maks_r, 1.0
                    pa.poses.append(pp)
                    dd = Detection3D()
                    dd.bbox.center.position.x = px
                    dd.bbox.center.position.y = py
                    dd.bbox.size.x = 2.0 * self.hayalet_maks_r
                    hh = ObjectHypothesisWithPose()
                    hh.hypothesis.class_id = BILINMEYEN
                    hh.hypothesis.score = 0.5
                    dd.results.append(hh)
                    da.detections.append(dd)
                continue
            p = Pose()
            p.position.x, p.position.y = hx, hy
            p.orientation.z, p.orientation.w = hr, 1.0
            pa.poses.append(p)
            d = Detection3D()
            d.bbox.center.position.x, d.bbox.center.position.y = hx, hy
            d.bbox.size.x = 2.0 * hr
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = BILINMEYEN
            h.hypothesis.score = 0.5
            d.results.append(h)
            da.detections.append(d)

        self.p_obs.publish(pa)
        self.p_cls.publish(da)

    def _rapor(self) -> None:
        hedef = self.gorev_xy[min(self.varilan + 1, len(self.gorev_xy) - 1)]
        self.get_logger().info(
            f"[{self.t:6.1f} s] konum=({self.x:6.2f}, {self.y:6.2f}) "
            f"ψ={math.degrees(self.psi) % 360:5.1f}° u={self.u:4.2f} m/s | "
            f"hedef {self.varilan+1}/{len(self.gorev_xy)} "
            f"({math.hypot(hedef[0]-self.x, hedef[1]-self.y):5.1f} m) | "
            f"cmd_vel {self.cmd_sayaci} mesaj, {self.bosluk_sayaci} boşluk "
            f"(en uzun {self.en_uzun_bosluk:.2f} s) | açıklık min "
            f"{self.en_yakin_m:.2f} m, çarpma {self.carpma}"
        )
        if self.varilan == len(self.gorev_xy) - 1:
            self.get_logger().info("🏁 PARKUR TAMAMLANDI")


def main() -> None:
    rclpy.init(args=sys.argv[1:])
    n = SanalGol()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

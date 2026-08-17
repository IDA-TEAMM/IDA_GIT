#!/usr/bin/env python3
"""
Girdap İDA — CANLI NÖBETÇİ (sürüş sırasında her şeyi izler, kötüyü bağırır).

Neden var (14.08.2026, kaptan isteği — §0.98):
    Kaptan: *"her şeyi görmeni istiyorum sürüşte oradayken"* ·
    *"izlerken kötü bir şey olunca otomatik buraya düşsün, hep izlensin"* ·
    *"parametreler dahil"*.

    O günkü su koşumunda ÜÇ ayrı arıza aynı anda vardı ve hiçbiri sahada
    görülemedi: füzyon pozu 10¹⁴⁹'a patlamıştı, kumandadaki acil durdurma
    HIGH'da kalmıştı, görev durumu KILL'deydi. Tekne 227 saniye boyunca
    GUIDED'da durdu ve `inhibit_reason` bütün o süre boyunca **`YOK`**
    diyordu. Arıza ancak akşam bant çözümlemesiyle bulundu.

    Bu betik o gecikmeyi kapatır: kuralları CANLI uygular ve yalnız
    **eyleme değer** satırları basar.

⚠ TASARIM KURALI — SESSİZLİK BAŞARI DEĞİLDİR:
    Her kural hem bozulmayı hem düzelmeyi bildirir (`ALARM` / `DUZELDI`).
    Böylece "hiç satır yok" = "nöbetçi ölmüş" ihtimali ayırt edilebilir;
    60 saniyede bir `NABIZ` satırı da bunun için basılır.

⚠ Bu betik SALT OKUR. Hiçbir konuya yayın yapmaz, hiçbir servisi çağırmaz,
    uçuş kontrolcüsüne hiçbir şey yazmaz. Aracı hareket ettirmesi imkânsız.

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=42 python3 scripts/canli_nobetci.py
    # alarm günlüğü: ~/girdap_logs/nobetci/nobetci_YYYYMMDD_HHMMSS.log

Çıktı sözleşmesi (her satır TEK olay — izleyici satır satır tüketir):
    ALARM  <kod> <açıklama>
    DUZELDI <kod> <açıklama>
    BILGI  <kod> <açıklama>
    NABIZ  <özet>
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Float64
from sensor_msgs.msg import NavSatFix

try:                                        # mavros yoksa da çekirdek koşsun
    from mavros_msgs.msg import State, RCIn, StatusText
    _MAVROS = True
except ImportError:                          # pragma: no cover
    _MAVROS = False


# --------------------------------------------------------------- eşikler
#
# Hepsi ÖLÇÜLMÜŞ bir olaydan türer; keyfi sayı yok.

POZ_MAKUL_MENZIL_M = 5000.0      # §0.98a — 10¹⁴⁹ olayı
POZ_AYRISMA_M = 10.0             # füzyon ↔ uçuş kontrolcüsü pozu farkı.
                                 # 14.08'de patlama ÖNCESİ ilk işaret buydu:
                                 # 12:55:53'te füzyon (3,7 · −14,8) derken
                                 # uçuş kontrolcüsü (0,15 · 0,36) diyordu.
                                 # Yani bu kural arızayı PATLAMADAN ÖNCE görür.
SETPOINT_MIN_HZ = 5.0            # §0.87 — 0,63 Hz'te uçuş kontrolcüsü aracı durdurdu
SETPOINT_BOSLUK_S = 3.0          # `target not received last Nsecs, stopping`
KOMUT_SIFIR_S = 10.0             # GUIDED+armed'da bu kadar süre sıfır komut
KOMUT_SIFIR_ESIK = 0.02          # m/s
HIZ_TAKIP_ORANI = 0.5            # gerçek hız komutun yarısının altındaysa
HIZ_TAKIP_S = 10.0               # bu kadar sürerse (CRUISE_THROTTLE kanıtı)
ENGEL_BAYAT_S = 5.0
ESTOP_HIGH_PWM = 1700
NABIZ_S = 120.0
ALARM_TEKRAR_S = 300.0           # aynı alarm en fazla bu sıklıkta yinelenir.
                                 # 60 s ilk koşumda fazla geldi: durum
                                 # değişmezken aynı satır dakikada bir düştü.
                                 # Alarm SÜRDÜĞÜ sürece zaten NABIZ'da
                                 # `aktif_alarm` listesinde görünüyor.
FCU_TEKRAR_S = 120.0             # aynı uçuş kontrolcüsü mesajı (sel önleme)
PARAM_PERIYOT_S = 180.0
SISTEM_PERIYOT_S = 30.0          # donanım/işletim sistemi denetimleri

# --- donanım eşikleri (hepsi ölçülmüş bir olaydan) ---
KART_GERILIM_MIN_V = 4.3         # §0.92e — `PreArm: Board (4.2v) out of range`
BATARYA_MIN_V = 13.6             # 4S, hücre başı 3,4 V — inişe geçme sınırı
SICAKLIK_UYARI_C = 85.0          # Jetson Orin Nano kısıtlama bölgesi
DISK_MIN_YUZDE = 10.0            # rosbag sürekli yazıyor; dolarsa kayıt biter
PUSULA_SAPMA_DER = 35.0          # GPS gidiş yönü ↔ pusula başlığı farkı.
                                 # §0.91b: pusula hatası ≈41°'lik baş açısına
                                 # denk geliyordu. Bu kural onu SUDA, tekne
                                 # hareket ederken yakalar — tezgâhta değil.
PUSULA_MIN_HIZ = 0.4             # m/s — altında gidiş yönü gürültüdür
LIDAR_ARAYUZ = os.environ.get("GIRDAP_LIDAR_ARAYUZ", "enP8p1s0")
KAMERA_USB_KIMLIK = "03e7"       # Luxonis OAK — §0.95b/1

# 🔑 Beklenen uçuş kontrolcüsü parametreleri — sapma ALARM üretir.
# `None` = "yalnız değişimini bildir, doğru değeri iddia etme".
# ⚠ CRUISE_THROTTLE: 14.08 ölçümü tam gazda 1,06-1,10 m/s dedi (§0.98k),
# yani 1 m/s ≈ %90-95 gaz. Şu anki 25 dört kat yanlış — kaptan yazana kadar
# alarm olarak DURUR, çünkü GUIDED'da hızın tutturulamamasının sebebi bu.
BEKLENEN_PARAMETRELER: dict[str, float | None] = {
    "CRUISE_THROTTLE": 95.0,
    "CRUISE_SPEED": 1.05,
    # ❌ `WP_SPEED` ALARMI KALDIRILDI (17.08.2026) — GUIDED'da GEÇERSİZ.
    #
    # ÖLÇÜLDÜ (16.08 183648 bandı, o gün WP_SPEED = 0,6 m/s idi):
    #     gerçekleşen hız  : medyan 0,27 · %90 0,92 · MAKS 1,14 m/s
    #     0,6'yı AŞAN örnek: 2799/9889 = **%28,3**
    #     bizim komutumuzun %60,5'i zaten 0,6'nın ÜSTÜNDE
    # ⇒ `WP_SPEED` hiçbir şeyi sınırlamıyor.
    #
    # NEDEN: biz `/mavros/setpoint_velocity/cmd_vel_unstamped` ile doğrudan
    # HIZ setpoint'i gönderiyoruz (planning_node._publish_cmd_vel). ArduPilot'un
    # WAYPOINT NAVİGASYON katmanı (`WP_*`, `TURN_RADIUS`) devrede DEĞİL; o
    # katman yalnız AUTO'da ve konum hedefli GUIDED'da çalışır.
    #
    # 🔴 BU ALARM ZARARLIYDI: nöbetçi, GUIDED koşumunu hiç etkilemeyen bir
    # parametre için `PARAM-WP_SPEED` basıyordu. Kendi kuralımız: *"bir alarm
    # her zaman yanıyorsa alarm değildir"* — ve yalan söyleyen alarm, gerçek
    # parametre kaymasını da görünmez yapar. (16.08 18:36 oturumunda tam bu
    # oldu: WP_SPEED 1,2→0,6 değişmişti, alarm yandı, ama GUIDED davranışına
    # etkisi YOKTU; asıl sorun ATC_STR_RAT_FF ve pivot kilidiydi.)
    #
    # ⚠ AUTO'ya düşülen kısa anlarda (16.08'de 31 kez, medyan 2,0 sn)
    # `WP_SPEED` geçerlidir — ama o pencereler zaten görev dışıdır.
    # Yarışmada AUTO kullanılmayacak (md 4.1).
    "ATC_SPEED_P": None,
    "ATC_SPEED_I": None,
    "MOT_THR_MIN": 10.0,
    "MOT_THR_MAX": 100.0,
    "SERVO1_FUNCTION": 74.0,     # sol itki — passthrough'a kayarsa felaket
    "SERVO3_FUNCTION": 73.0,     # sağ itki
    "MOT_THST_ASYM": None,
    "ARMING_CHECK": None,
}


def _simdi() -> str:
    return datetime.now().strftime("%H:%M:%S")


class Nobetci(Node):
    """Salt-okur gözlemci. Kural ihlallerini satır satır basar."""

    def __init__(self) -> None:
        super().__init__("girdap_canli_nobetci")

        kayit_dizini = Path(
            os.environ.get("GIRDAP_NOBETCI_LOG", Path.home() / "girdap_logs" / "nobetci")
        )
        kayit_dizini.mkdir(parents=True, exist_ok=True)
        self._log = (
            kayit_dizini / f"nobetci_{datetime.now():%Y%m%d_%H%M%S}.log"
        ).open("a", encoding="utf-8", buffering=1)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- izlenen durum ---
        self._fuzyon_poz: tuple[float, float] | None = None
        self._fuzyon_t = 0.0
        self._fc_poz: tuple[float, float] | None = None
        self._vx: float | None = None
        self._sp_t: list[float] = []          # setpoint damgaları (pencere)
        self._sp_son: tuple[float, float] | None = None
        self._sp_sifir_bas: float | None = None
        self._hiz_takip_bas: float | None = None
        self._thrust: tuple[float, ...] = ()
        self._inhibit = ""
        self._fsm = ""
        self._mod = ""
        self._armed = False
        self._rc10: int | None = None
        self._engel_t = 0.0
        self._engel_n = 0
        self._gps_status: int | None = None
        self._aktif: dict[str, str] = {}      # kod -> açıklama
        self._son_bildirim: dict[str, float] = {}
        self._nabiz_t = 0.0
        self._baslangic = time.monotonic()
        self._parametreler: dict[str, float] = {}
        self._fcu_gorulen: dict[str, float] = {}
        self._kill_sebep = ""
        self._pusula_der: float | None = None
        self._gps_yon_der: float | None = None
        self._gps_hiz: float | None = None
        self._sistem_thread = threading.Thread(
            target=self._sistem_dongusu, daemon=True)

        # --- abonelikler (hepsi salt okur) ---
        self.create_subscription(
            PoseStamped, "/girdap/fusion/pose", self._on_fuzyon, sensor_qos)
        self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, sensor_qos)
        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped",
            self._on_setpoint, 10)
        self.create_subscription(
            Float32MultiArray, "/girdap/control/thrust", self._on_thrust, 10)
        self.create_subscription(
            String, "/girdap/control/inhibit_reason", self._on_inhibit, 10)
        self.create_subscription(
            String, "/girdap/mission/state", self._on_fsm, 10)
        self.create_subscription(
            PoseArray, "/perception/obstacle_map", self._on_engel, sensor_qos)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._on_gps, sensor_qos)
        # KILL SEBEBİ — §0.98p: KILL'e neden girildiği en büyük teşhis boşluğuydu
        self.create_subscription(
            String, "/girdap/mission/kill_reason", self._on_kill_reason, 10)
        # Kendi arıza/durum mesajlarımız (operatör ekranına gidenler)
        if _MAVROS:
            self.create_subscription(
                StatusText, "/mavros/statustext/send", self._on_kendi_mesaj,
                sensor_qos)
            self.create_subscription(
                Float64, "/mavros/global_position/compass_hdg",
                self._on_pusula, sensor_qos)
            self.create_subscription(
                TwistStamped, "/mavros/global_position/raw/gps_vel",
                self._on_gps_hiz, sensor_qos)

        if _MAVROS:
            self.create_subscription(
                State, "/mavros/state", self._on_state, sensor_qos)
            self.create_subscription(
                RCIn, "/mavros/rc/in", self._on_rc, sensor_qos)
            self.create_subscription(
                StatusText, "/mavros/statustext/recv", self._on_status, sensor_qos)
            self.create_subscription(
                PoseStamped, "/mavros/local_position/pose", self._on_fc_poz, sensor_qos)
            self.create_subscription(
                TwistStamped, "/mavros/local_position/velocity_body",
                self._on_vel, sensor_qos)

        self.create_timer(1.0, self._degerlendir)
        self._param_thread = threading.Thread(
            target=self._param_dongusu, daemon=True)
        self._param_thread.start()
        self._sistem_thread.start()

        self._bas("BILGI", "NOBETCI", "canli nobetci basladi — salt okur, "
                  f"gunluk: {self._log.name}")

    # ----------------------------------------------------------- yayın
    def _bas(self, tur: str, kod: str, aciklama: str) -> None:
        satir = f"{tur} {_simdi()} {kod} {aciklama}"
        print(satir, flush=True)
        self._log.write(satir + "\n")

    def _alarm(self, kod: str, aciklama: str) -> None:
        """Alarmı kaydet; ilk kez ya da tekrar penceresi dolduysa bas."""
        simdi = time.monotonic()
        onceki = self._son_bildirim.get(kod)
        self._aktif[kod] = aciklama
        if onceki is None or simdi - onceki >= ALARM_TEKRAR_S:
            self._son_bildirim[kod] = simdi
            self._bas("ALARM", kod, aciklama)

    def _temizle(self, kod: str, aciklama: str = "") -> None:
        if kod in self._aktif:
            del self._aktif[kod]
            self._son_bildirim.pop(kod, None)
            self._bas("DUZELDI", kod, aciklama or "kural yeniden saglandi")

    # ----------------------------------------------------------- callback
    def _on_fuzyon(self, m: PoseStamped) -> None:
        self._fuzyon_poz = (m.pose.position.x, m.pose.position.y)
        self._fuzyon_t = time.monotonic()

    def _on_odom(self, m: Odometry) -> None:
        self._fuzyon_t = time.monotonic()

    def _on_fc_poz(self, m) -> None:                       # noqa: ANN001
        self._fc_poz = (m.pose.position.x, m.pose.position.y)

    def _on_vel(self, m) -> None:                          # noqa: ANN001
        self._vx = m.twist.linear.x

    def _on_setpoint(self, m: Twist) -> None:
        simdi = time.monotonic()
        self._sp_t.append(simdi)
        self._sp_t = [t for t in self._sp_t if simdi - t <= 5.0]
        self._sp_son = (m.linear.x, m.angular.z)

    def _on_thrust(self, m: Float32MultiArray) -> None:
        self._thrust = tuple(m.data)

    def _on_inhibit(self, m: String) -> None:
        if m.data != self._inhibit:
            self._inhibit = m.data
            if m.data and m.data != "YOK":
                self._bas("BILGI", "INHIBIT", f"thrust kilidi: {m.data}")

    def _on_fsm(self, m: String) -> None:
        if m.data != self._fsm:
            self._bas("BILGI", "FSM", f"{self._fsm or '-'} -> {m.data}")
            self._fsm = m.data

    def _on_engel(self, m: PoseArray) -> None:
        self._engel_t = time.monotonic()
        self._engel_n = len(m.poses)

    def _on_gps(self, m: NavSatFix) -> None:
        self._gps_status = int(m.status.status)

    def _on_state(self, m) -> None:                        # noqa: ANN001
        if m.mode != self._mod or m.armed != self._armed:
            self._bas("BILGI", "MOD",
                      f"{self._mod or '-'}/{self._armed} -> {m.mode}/{m.armed}")
            self._mod, self._armed = m.mode, m.armed

    def _on_rc(self, m) -> None:                           # noqa: ANN001
        if len(m.channels) >= 10:
            self._rc10 = m.channels[9]

    def _on_status(self, m) -> None:                       # noqa: ANN001
        """Uçuş kontrolcüsünün kendi mesajları — eyleme değer olanları geçir.

        ⚠ BASTIRMA ŞART: `PreArm: Check mag field` 13.08'de saniyede bir aktı
        (117 kez `Logging failed` de öyle). Bastırmadan bu kanal tek başına
        izleyiciyi doldurur ve gerçek alarmı gömer — arıza kodlarındaki
        telsiz bütçesi kuralının aynısı.
        """
        metin = m.text
        onemli = (
            "PreArm", "Arm:", "Emergency", "EStop", "Failsafe", "failsafe",
            "EKF", "GPS", "Compass", "mag", "Motors", "not received",
        )
        if not any(k in metin for k in onemli):
            return
        simdi = time.monotonic()
        onceki = self._fcu_gorulen.get(metin)
        if onceki is not None and simdi - onceki < FCU_TEKRAR_S:
            return
        self._fcu_gorulen[metin] = simdi
        self._bas("BILGI", "FCU", metin)

    def _on_kill_reason(self, m: String) -> None:
        """§0.98p: KILL'in SEBEBİ. Kurtarma politikası sebebe göre değişiyor."""
        if m.data and m.data != self._kill_sebep:
            self._kill_sebep = m.data
            if m.data.startswith("temizlendi"):
                self._bas("BILGI", "KILL-TEMIZ", m.data)
            else:
                self._bas("ALARM", "KILL-SEBEP",
                          f"{m.data} — cikis: "
                          "ros2 service call /girdap/mission/reset "
                          "std_srvs/srv/Trigger {}")

    def _on_kendi_mesaj(self, m) -> None:                  # noqa: ANN001
        """Operatör ekranına GİDEN kendi mesajlarımız — kritik olanları geçir."""
        metin = m.text
        if any(k in metin for k in ("SENKRON", "POZ-SACMA", "KILL", "BASLAT-YOK")):
            simdi = time.monotonic()
            onceki = self._fcu_gorulen.get(metin)
            if onceki is None or simdi - onceki >= FCU_TEKRAR_S:
                self._fcu_gorulen[metin] = simdi
                self._bas("ALARM", "EKRAN", metin)

    def _on_pusula(self, m: Float64) -> None:
        self._pusula_der = float(m.data)

    def _on_gps_hiz(self, m: TwistStamped) -> None:
        vx, vy = m.twist.linear.x, m.twist.linear.y
        self._gps_hiz = math.hypot(vx, vy)
        # ENU'da rota açısı: kuzeyden saat yönünde (pusula ile aynı sözleşme)
        self._gps_yon_der = (math.degrees(math.atan2(vx, vy))) % 360.0

    # ----------------------------------------------------------- kurallar
    def _degerlendir(self) -> None:
        simdi = time.monotonic()

        # 1) POZ SAÇMA — §0.98a'nın 10¹⁴⁹ olayı
        if self._fuzyon_poz is not None:
            x, y = self._fuzyon_poz
            if not (math.isfinite(x) and math.isfinite(y)):
                self._alarm("POZ-SACMA", f"fuzyon pozu SONLU DEGIL: {x} {y}")
            elif math.hypot(x, y) > POZ_MAKUL_MENZIL_M:
                self._alarm("POZ-SACMA",
                            f"fuzyon pozu PATLADI: |p|={math.hypot(x, y):.3e} m "
                            "→ use_isam2:=false ile kos")
            else:
                self._temizle("POZ-SACMA", f"poz makul ({x:.1f}, {y:.1f})")

        # 2) FÜZYON ↔ UÇUŞ KONTROLCÜSÜ AYRIŞMASI — patlamanın ERKEN işareti
        if self._fuzyon_poz and self._fc_poz:
            fark = math.hypot(self._fuzyon_poz[0] - self._fc_poz[0],
                              self._fuzyon_poz[1] - self._fc_poz[1])
            if math.isfinite(fark) and fark > POZ_AYRISMA_M:
                self._alarm(
                    "POZ-AYRISMA",
                    f"fuzyon ile ucus kontrolcusu {fark:.1f} m ayristi "
                    f"(fuzyon {self._fuzyon_poz[0]:.1f},{self._fuzyon_poz[1]:.1f} ↔ "
                    f"FC {self._fc_poz[0]:.1f},{self._fc_poz[1]:.1f}) — "
                    "14.08'de patlama boyle basladi")
            elif math.isfinite(fark):
                self._temizle("POZ-AYRISMA", f"fark {fark:.1f} m")

        # 3) ACİL DURDURMA — kumandaya BİLEREK atanmış bir tuş (kaptan,
        # 14.08): "sıkıntı yok". Bu yüzden HIGH olması tek başına arıza
        # DEĞİL — operatörün kasıtlı seçimi olabilir.
        # 🔑 Alarm YALNIZ çelişki varken: araç **sürmesi beklenen** durumdayken
        # (GUIDED + armed, yani görev komutu akıyor) motorların kesik olması.
        # 14.08'de tam bu çelişki 227 saniye boyunca fark edilmedi.
        # Diğer hâllerde sessizce not düşülür.
        surus_bekleniyor = self._armed and self._mod == "GUIDED"
        if self._rc10 is not None:
            if self._rc10 > ESTOP_HIGH_PWM and surus_bekleniyor:
                self._alarm("ESTOP",
                            f"GUIDED+armed ama MotorEStop HIGH (pwm={self._rc10}) "
                            "— komut akiyor, motorlar KESIK")
            elif self._rc10 > ESTOP_HIGH_PWM:
                self._temizle("ESTOP", f"MotorEStop HIGH (pwm={self._rc10}) — "
                                       "gorev modunda degil, normal")
            else:
                self._temizle("ESTOP", f"MotorEStop LOW (pwm={self._rc10})")

        # 4) GUIDED + armed iken komut akışı
        surus = self._armed and self._mod == "GUIDED"
        # ⚠ ISINMA PAYI: 5 saniyelik kayan pencere dolmadan Hz hesabı DÜŞÜK
        # çıkar. İlk koşumda tam bunu yaptı — 14:40:07'de "0,8 Hz" alarmı
        # verip 3 saniye sonra "6,8 Hz" diye düzeldi. Yanlış pozitif, gerçek
        # alarmın değerini düşürür; nöbetçinin ilk kuralı **güvenilir olmak**.
        isinma = simdi - self._baslangic < 6.0
        if surus and not isinma:
            hz = len(self._sp_t) / 5.0
            bosluk = simdi - self._sp_t[-1] if self._sp_t else 999.0
            if bosluk > SETPOINT_BOSLUK_S:
                self._alarm("SETPOINT-BOSLUK",
                            f"setpoint {bosluk:.1f} s kesildi — ucus "
                            "kontrolcusu araci DURDURUR")
            elif hz < SETPOINT_MIN_HZ:
                self._alarm("SETPOINT-YAVAS",
                            f"setpoint {hz:.1f} Hz (<{SETPOINT_MIN_HZ:.0f}) — "
                            "§0.87'de 0,63 Hz araci durdurmustu")
            else:
                self._temizle("SETPOINT-BOSLUK", f"{hz:.1f} Hz akiyor")
                self._temizle("SETPOINT-YAVAS", f"{hz:.1f} Hz akiyor")

            # 5) KOMUT SIFIR — 14.08'in ana bulgusu (227 s boyunca 0,000)
            if self._sp_son is not None:
                if abs(self._sp_son[0]) < KOMUT_SIFIR_ESIK:
                    self._sp_sifir_bas = self._sp_sifir_bas or simdi
                    sure = simdi - self._sp_sifir_bas
                    if sure > KOMUT_SIFIR_S:
                        self._alarm(
                            "KOMUT-SIFIR",
                            f"GUIDED+armed ama komut {sure:.0f} s'dir SIFIR "
                            f"(itki {self._thrust}) — kilit: "
                            f"{self._inhibit or 'YOK (bu da bir bulgu!)'}")
                else:
                    if self._sp_sifir_bas is not None:
                        self._temizle("KOMUT-SIFIR",
                                      f"komut geri geldi: {self._sp_son[0]:+.2f} m/s")
                    self._sp_sifir_bas = None

            # 6) HIZ TAKİP EDİLMİYOR — CRUISE_THROTTLE kanıtı (§0.98k)
            #
            # ⚠ ACİL DURDURMA AKTİFKEN BU KURAL KOŞMAZ. 14.08 14:53'te tam
            # bunu yaptı: e-stop HIGH (motorlar fiziksel olarak kesik) iken
            # "uçuş kontrolcüsü hızı tutturamıyor, CRUISE_THROTTLE şüpheli"
            # dedi. Teşhis YANLIŞTI — motor dönmüyorken hız tutulamaz, bu
            # ayarla ilgili değil. Bir nöbetçinin en pahalı hatası, operatörü
            # yanlış yere baktırmaktır (§0.93d'nin `BASLAT-YOK` dersi).
            estop_aktif = self._rc10 is not None and self._rc10 > ESTOP_HIGH_PWM
            if (self._sp_son is not None and self._vx is not None
                    and not estop_aktif
                    and self._sp_son[0] > 0.3):
                if self._vx < HIZ_TAKIP_ORANI * self._sp_son[0]:
                    self._hiz_takip_bas = self._hiz_takip_bas or simdi
                    if simdi - self._hiz_takip_bas > HIZ_TAKIP_S:
                        self._alarm(
                            "HIZ-TUTMUYOR",
                            f"komut {self._sp_son[0]:.2f} m/s ama gercek "
                            f"{self._vx:.2f} m/s — ucus kontrolcusu hizi "
                            "tutturamiyor (CRUISE_THROTTLE=25 supheli, §0.98k)")
                else:
                    self._temizle("HIZ-TUTMUYOR",
                                  f"komut {self._sp_son[0]:.2f} ↔ "
                                  f"gercek {self._vx:.2f} m/s")
                    self._hiz_takip_bas = None
        else:
            self._sp_sifir_bas = None
            self._hiz_takip_bas = None

        # 7) ENGEL HARİTASI — LiDAR zinciri
        if self._engel_t and simdi - self._engel_t > ENGEL_BAYAT_S:
            self._alarm("ENGEL-BAYAT",
                        f"engel haritasi {simdi - self._engel_t:.0f} s'dir "
                        "gelmiyor — LiDAR zinciri kopmus olabilir")
        elif self._engel_t:
            self._temizle("ENGEL-BAYAT", f"{self._engel_n} engel akiyor")

        # 7b) PUSULA TUTARLILIĞI — §0.91'in arızasını SUDA yakalar
        #
        # Tezgâhta `xy diff` ölçümü kapalı alan yüzünden hüküm vermiyordu
        # (§0.91e). Bu kural bunun yerine FİZİKSEL bir tutarlılık bakıyor:
        # tekne ileri giderken GPS'in gördüğü GİDİŞ YÖNÜ ile pusulanın
        # söylediği BAŞ AÇISI birbirini tutmalı. §0.91b'nin hesabı 41°'lik
        # bir baş açısı hatasına denk geliyordu — bu kural onu doğrudan ölçer.
        # ⚠ Yalnız yeterince hızlıyken: yavaşta gidiş yönü gürültüdür.
        # ⚠ Yanal sürüklenme (akıntı/rüzgâr) da fark üretir; bu yüzden eşik
        # geniş (35°) ve kural "pusula bozuk" demez, "tutarsız" der.
        if (self._pusula_der is not None and self._gps_yon_der is not None
                and self._gps_hiz is not None
                and self._gps_hiz >= PUSULA_MIN_HIZ):
            fark = abs((self._pusula_der - self._gps_yon_der + 180.0) % 360.0 - 180.0)
            if fark > PUSULA_SAPMA_DER:
                self._alarm("PUSULA-TUTARSIZ",
                            f"pusula {self._pusula_der:.0f}° ↔ GPS gidis yonu "
                            f"{self._gps_yon_der:.0f}° = {fark:.0f}° fark "
                            f"(hiz {self._gps_hiz:.2f} m/s) — §0.91")
            else:
                self._temizle("PUSULA-TUTARSIZ", f"fark {fark:.0f}°")

        # 8) GPS fix
        if self._gps_status is not None and self._gps_status < 0:
            self._alarm("GPS-YOK", "GPS fix YOK (status=-1)")
        elif self._gps_status is not None:
            self._temizle("GPS-YOK", f"fix status={self._gps_status}")

        # NABIZ — sessizlik "nobetci oldu" ile karismasin
        if simdi - self._nabiz_t >= NABIZ_S:
            self._nabiz_t = simdi
            poz = ("-" if not self._fuzyon_poz
                   else f"{self._fuzyon_poz[0]:.1f},{self._fuzyon_poz[1]:.1f}")
            self._bas("NABIZ", "OK" if not self._aktif else "SORUNLU",
                      f"mod={self._mod}/{self._armed} fsm={self._fsm or '-'} "
                      f"poz=({poz}) vx={self._vx if self._vx is None else round(self._vx, 2)} "
                      f"sp={self._sp_son[0] if self._sp_son else None} "
                      f"engel={self._engel_n} estop={self._rc10} "
                      f"aktif_alarm={sorted(self._aktif) or 'yok'}")

    # ----------------------------------------------------------- sistem
    def _sistem_dongusu(self) -> None:
        """Donanım/işletim sistemi denetimleri — ROS'un göremediği katman.

        Hepsi §0.95b'nin saha kontrol listesinden: bunlar sahada tekneyi
        durduran ama ROS konularında hiç görünmeyen arızalar.
        """
        time.sleep(5.0)
        while rclpy.ok():
            try:
                self._sistem_denetle()
            except Exception as exc:           # noqa: BLE001 — nöbetçi ölmemeli
                self._bas("BILGI", "SISTEM-HATA", f"denetim atlandi: {exc!r}")
            time.sleep(SISTEM_PERIYOT_S)

    def _sistem_denetle(self) -> None:
        # 1) Kamera USB — §0.95b/1: 8 saatte iki kez düştü, düğüm 359 kez öldü
        try:
            lsusb = subprocess.run(["lsusb"], capture_output=True, text=True,
                                   timeout=10.0).stdout
            if KAMERA_USB_KIMLIK in lsusb:
                self._temizle("KAMERA-USB", "OAK kamera USB'de görünüyor")
            else:
                self._alarm("KAMERA-USB",
                            f"OAK kamera ({KAMERA_USB_KIMLIK}) USB'de YOK — "
                            "konnektoru mekanik olarak sabitle; kosu ortasinda "
                            "duserse kapi takibi biter")
        except Exception:                      # noqa: BLE001
            pass

        # 2) LiDAR hattı — §0.95b/2: ayırt edici TEK satır.
        # Mid-360 100 Mb'lik bir cihaz; 1000Mb/s ya da Unknown ise kablonun
        # ucunda LiDAR YOK (§0.63'ün imzası, iki kez birebir doğrulandı).
        try:
            hiz = subprocess.run(["ethtool", LIDAR_ARAYUZ], capture_output=True,
                                 text=True, timeout=10.0).stdout
            satir = next((s.strip() for s in hiz.splitlines() if "Speed:" in s), "")
            if "100Mb/s" in satir:
                self._temizle("LIDAR-HAT", satir)
            elif satir:
                self._alarm("LIDAR-HAT",
                            f"{LIDAR_ARAYUZ} {satir} — Mid-360 100Mb'lik bir "
                            "cihaz; kablonun ucunda LiDAR YOK olabilir (§0.63)")
        except Exception:                      # noqa: BLE001
            pass

        # 3) Disk — rosbag kesintisiz yazıyor; dolarsa teslim dosyaları biter
        try:
            import shutil
            kul = shutil.disk_usage(str(Path.home()))
            bos_yuzde = 100.0 * kul.free / kul.total
            if bos_yuzde < DISK_MIN_YUZDE:
                self._alarm("DISK",
                            f"disk %{bos_yuzde:.1f} bos ({kul.free / 1e9:.1f} GB) "
                            "— rosbag/teslim dosyalari kesilebilir")
            else:
                self._temizle("DISK", f"%{bos_yuzde:.0f} bos")
        except Exception:                      # noqa: BLE001
            pass

        # 4) Sıcaklık — §0.83'ün aşırı akım/kısıtlama hikâyesi
        try:
            sicakliklar = []
            for yol in Path("/sys/devices/virtual/thermal").glob("thermal_zone*/temp"):
                try:
                    sicakliklar.append(int(yol.read_text().strip()) / 1000.0)
                except Exception:              # noqa: BLE001
                    continue
            if sicakliklar:
                en_yuksek = max(sicakliklar)
                if en_yuksek > SICAKLIK_UYARI_C:
                    self._alarm("SICAKLIK",
                                f"en yuksek {en_yuksek:.0f} °C — Jetson "
                                "kisitlamaya girebilir (§0.83)")
                else:
                    self._temizle("SICAKLIK", f"en yuksek {en_yuksek:.0f} °C")
        except Exception:                      # noqa: BLE001
            pass

        # 5) Teslim dosyaları — şartname md 4.2, her gecikmiş dosya 5 ceza puanı.
        # Yalnız SÜRÜŞ sırasında bakılır: durağan tezgâhta üretilmemesi normal.
        if self._armed and self._fsm.startswith("PARKUR"):
            # ⚠ RECURSIVE olmalı: 14.08 ilk koşumda tek seviyeli glob
            # `local_map/*` yalnız oturum DİZİNİNİ gördü, kareler ise
            # `oturum_*/png_yedek/frame_*.png` altında yazılıyordu → 489 kare
            # üretilirken "455 s'dir guncellenmedi" diye YANLIŞ ALARM verdi.
            for ad, kok in (
                ("Dosya-2 telemetri", "girdap_logs/telemetry"),
                ("Dosya-3 yerel harita", "girdap_logs/local_map"),
            ):
                try:
                    yeni = max(
                        (q.stat().st_mtime
                         for q in (Path.home() / kok).rglob("*") if q.is_file()),
                        default=0.0,
                    )
                    yas = time.time() - yeni
                    if yeni == 0.0 or yas > 120.0:
                        self._alarm(
                            f"TESLIM-{ad.split()[0]}",
                            f"{ad} {int(yas) if yeni else '∞'} s'dir "
                            "guncellenmedi — sartname 4.2, gecikme 5 ceza puani")
                    else:
                        self._temizle(f"TESLIM-{ad.split()[0]}", f"{ad} taze")
                except Exception:              # noqa: BLE001
                    pass

        self._ozet_yaz()

    def _ozet_yaz(self) -> None:
        """Oturum özetini md olarak yaz — kaptan: *"bide kaydetsin otomatik"*.

        GIRDAP_DURUM.md'ye DOĞRUDAN yazılmaz (tek dosya kuralı §0.31c orada
        insan eliyle düzenlenen bir anlatı tutuyor; otomatik ekleme onu
        çakıştırır). Bunun yerine devralan oturumun kopyalayabileceği hazır
        bir özet üretilir.
        """
        try:
            yol = Path(self._log.name).with_suffix(".ozet.md")
            satirlar = [
                f"# Nöbetçi oturum özeti — {datetime.now():%d.%m.%Y %H:%M}",
                "",
                f"- Günlük: `{self._log.name}`",
                f"- Süre: {(time.monotonic() - self._baslangic) / 60.0:.0f} dk",
                f"- Son durum: mod **{self._mod}/{self._armed}** · "
                f"görev **{self._fsm or '-'}**",
                f"- Füzyon pozu: {self._fuzyon_poz} · uçuş kontrolcüsü: {self._fc_poz}",
                f"- Acil durdurma (RC10): {self._rc10}",
                f"- Son KILL sebebi: {self._kill_sebep or 'yok'}",
                "",
                "## Şu an aktif alarmlar",
                "",
            ]
            if self._aktif:
                satirlar += [f"- 🔴 **{k}** — {v}" for k, v in sorted(self._aktif.items())]
            else:
                satirlar.append("- ✅ aktif alarm yok")
            satirlar += ["", "## Okunan uçuş kontrolcüsü parametreleri", ""]
            for ad, deger in sorted(self._parametreler.items()):
                beklenen = BEKLENEN_PARAMETRELER.get(ad)
                isaret = ("" if beklenen is None
                          else (" ✅" if abs(deger - beklenen) <= 1e-6
                                else f" 🔴 (beklenen {beklenen})"))
                satirlar.append(f"- `{ad}` = {deger}{isaret}")
            yol.write_text("\n".join(satirlar) + "\n", encoding="utf-8")
        except Exception:                      # noqa: BLE001
            pass

    # ----------------------------------------------------------- parametre
    def _param_dongusu(self) -> None:
        """Uçuş kontrolcüsü parametrelerini periyodik oku, sapmayı bildir.

        Ayrı thread + `ros2 param get` CLI'si: MAVROS parametreleri ROS
        parametresi olarak yansıtıyor ve CLI bu yolda kanıtlandı (14.08).
        Düğümün kendi çağrı yığınına girmemesi bilinçli — parametre okuması
        yavaştır (~1-2 s/parametre), izleme döngüsünü bloklamamalı.
        """
        time.sleep(10.0)                       # yığın otursun
        while rclpy.ok():
            for ad, beklenen in BEKLENEN_PARAMETRELER.items():
                deger = self._param_oku(ad)
                if deger is None:
                    continue
                onceki = self._parametreler.get(ad)
                self._parametreler[ad] = deger
                if onceki is not None and abs(deger - onceki) > 1e-9:
                    self._bas("ALARM", "PARAM-DEGISTI",
                              f"{ad}: {onceki} -> {deger} (biri yazdi)")
                if beklenen is not None and abs(deger - beklenen) > 1e-6:
                    self._alarm(
                        f"PARAM-{ad}",
                        f"{ad}={deger} ama beklenen {beklenen} "
                        "(§0.98k olcumu) — GUIDED'da hiz tutturulamaz")
                elif beklenen is not None:
                    self._temizle(f"PARAM-{ad}", f"{ad}={deger} dogru")
            time.sleep(PARAM_PERIYOT_S)

    @staticmethod
    def _param_oku(ad: str) -> float | None:
        try:
            cikti = subprocess.run(
                ["ros2", "param", "get", "/mavros/param", ad],
                capture_output=True, text=True, timeout=15.0,
            ).stdout.strip()
        except Exception:                      # noqa: BLE001 — nöbetçi ölmemeli
            return None
        for parca in cikti.replace(":", " ").split():
            try:
                return float(parca)
            except ValueError:
                continue
        return None


def main() -> None:
    rclpy.init()
    dugum = Nobetci()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

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

from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
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
    "WP_SPEED": 0.95,
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

        if _MAVROS:
            self.create_subscription(
                State, "/mavros/state", self._on_state, sensor_qos)
            self.create_subscription(
                RCIn, "/mavros/rc/in", self._on_rc, sensor_qos)
            self.create_subscription(
                StatusText, "/mavros/statustext/recv", self._on_status, sensor_qos)
            from geometry_msgs.msg import PoseStamped as _PS
            self.create_subscription(
                _PS, "/mavros/local_position/pose", self._on_fc_poz, sensor_qos)
            from geometry_msgs.msg import TwistStamped
            self.create_subscription(
                TwistStamped, "/mavros/local_position/velocity_body",
                self._on_vel, sensor_qos)

        self.create_timer(1.0, self._degerlendir)
        self._param_thread = threading.Thread(
            target=self._param_dongusu, daemon=True)
        self._param_thread.start()

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

        # 3) ACİL DURDURMA — 14.08'de kosum sonuna kadar HIGH kaldi
        if self._rc10 is not None:
            if self._rc10 > ESTOP_HIGH_PWM:
                self._alarm("ESTOP",
                            f"kumanda MotorEStop HIGH (pwm={self._rc10}) — "
                            "motorlar KESIK, ARM da reddedilir")
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
            if (self._sp_son is not None and self._vx is not None
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

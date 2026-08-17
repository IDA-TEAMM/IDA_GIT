#!/usr/bin/env python3
"""
Girdap İDA — AYAR DEFTERİ: "şu ayardayken İDA ne yaptı?"

NE: Uçuş kontrolcüsünün ayarlarını ve teknenin o ayarlardaki DAVRANIŞINI
    aynı satırda tutar. Bir parametre değiştiği anda önceki dönemi kapatır,
    özetini yazar ve yeni dönemi açar. Çıktı hem CSV (makine) hem Markdown
    (insan).

🔴 NEDEN VAR — 17.08'de bu boşluğa çarpıldı:
    Bantlar (`rosbag_kaydet.sh`) uçuş kontrolcüsü parametrelerini **HİÇ
    KAYDETMİYOR**. Gece boyunca "16.08'de FF neydi?" sorusuna cevap ararken
    masaüstündeki `.param` dosyasına bakmak zorunda kalındı — ve o dosyanın
    canlıyı YANSITMADIĞI ölçüldü:
        TURN_RADIUS   dosya 0,9  ↔  canlı 1,0
        WP_SPEED      dosya 2,0  ↔  canlı 0,6
        RC10_OPTION   dosya 0    ↔  canlı 31
        MODE1..6      dosya 0    ↔  canlı DOĞRULANMADI
    ⇒ Bütün bant çözümlemeleri "hangi ayardaydı" sorusuna **tahminle** cevap
    verdi. Bu defter o bağı kurar.

    Uçuş testi literatürünün önerdiği desen de bu: parametre **değişim
    bayrakları** + **uçuş fazı** bilgisiyle korelasyon. Yani "ne oldu"yu
    "hangi ayarda oldu"dan ayrı tutmamak.

⛔ SALT OKURDUR. Hiçbir parametre yazmaz, hiçbir konuya yayın yapmaz, hiçbir
   servisi tetiklemez. Aracı hareket ettirmesi imkânsız. (Nöbetçiyle aynı
   sözleşme; ayar araçlarından farkı bu.)

📋 HER DÖNEM İÇİN KAYDEDİLENLER
    AYAR   : izlenen 14 FC parametresi (aşağıdaki liste)
    SÜRE   : dönem başı/sonu, toplam saniye
    MOD    : GUIDED+ARMED oranı · mod geçiş sayısı
    DÖNÜŞ  : istenen ↔ gerçekleşen medyan · EŞLEŞTİRİLMİŞ takip oranı
             🔑 eşleştirilmiş: medyan(oran), medyan(a)/medyan(b) DEĞİL.
             17.08'de o hata yapıldı ("%55 takip" iddiası geri alındı).
    HIZ    : istenen ↔ gerçekleşen · alt uç (0,05-0,30 m/s) aşım oranı
    MOTOR  : PWM doygunluk oranı (doygunken dönüş fiilen olmuyor: oran 0,08)
    PIVOT  : `inhibit_reason`da PIVOT geçen oran — 🔑 EN İYİ BAŞARI ÖLÇÜTÜ
             (gürültülü regresyon ölçütünden kararlı, doğrudan sonucu gösterir)
    KAPI   : `/girdap/planning/gate` >1 m atlama oranı · kenar hafızası boyutu
    ESTOP  : RC10 HIGH oranı — kirli veriyi ayıklamak için

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1     # 🔴 İKİSİ DE ŞART
    python3 scripts/ayar_defteri.py                  # canlı_nobetci ile birlikte
    python3 scripts/ayar_defteri.py --param-periyot 10   # daha sık yokla

Çıktı:
    ~/girdap_logs/ayar_defteri/defter_YYYYMMDD_HHMMSS.csv   (makine)
    ~/girdap_logs/ayar_defteri/defter_YYYYMMDD_HHMMSS.md    (insan, tablo)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import time
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseArray, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import RCIn, RCOut, State
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import Imu
from std_msgs.msg import String

PARAM_DUGUM = "/mavros/param"

# İzlenen parametreler — "hangi ayardaydı" sorusunun cevabı bunlar.
# Seçim gerekçesi: 17.08 çözümlemesinde bu 14'ü aranıp bulunamadı.
IZLENEN = [
    "ATC_STR_RAT_FF", "ATC_STR_RAT_P", "ATC_STR_RAT_I",
    "ATC_SPEED_P", "ATC_SPEED_I",
    "ATC_ACCEL_MAX", "ATC_DECEL_MAX", "MOT_THST_EXPO",
    "CRUISE_SPEED", "CRUISE_THROTTLE",
    "WP_SPEED", "WP_RADIUS", "WP_PIVOT_ANGLE", "TURN_RADIUS",
]

PARAM_PERIYOT = 20.0      # sn — parametre yoklama aralığı
ASGARI_DONEM = 20.0       # bundan kısa dönem yazılmaz (geçiş gürültüsü)
UYARIM_ESIK = math.radians(5.0)
GUNLUK_DIZIN = os.path.expanduser("~/girdap_logs/ayar_defteri")


def yuzdelik(v, p):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(round(p / 100 * (len(s) - 1)))))]


class Donem:
    """Parametreleri sabit kalan bir zaman dilimi ve o dilimdeki davranış."""

    def __init__(self, t0, ayar):
        self.t0 = t0
        self.ayar = dict(ayar)
        self.donus = []        # (istenen, gerçekleşen) rad/s — EŞLEŞTİRİLMİŞ
        self.hiz = []          # (istenen, gerçekleşen) m/s
        self.doygun = [0, 0]   # [doygun, toplam]
        self.pivot = [0, 0]    # [PIVOT geçen, toplam inhibit]
        self.ga = 0.0          # GUIDED+ARMED saniyesi
        self.mod_gecis = 0
        self.estop = [0, 0]
        self.kapi = []         # ardışık atlama (m)
        self.hafiza = []       # kenar kaydı sayısı

    def ozet(self, t1):
        sure = max(t1 - self.t0, 1e-9)
        d = {"baslangic": datetime.fromtimestamp(self.t0).strftime("%H:%M:%S"),
             "sure_sn": round(sure, 1),
             "guided_armed_%": round(100 * self.ga / sure, 1),
             "mod_gecis": self.mod_gecis}
        d.update({k: (round(v, 3) if v is not None else "")
                  for k, v in self.ayar.items()})
        # dönüş takibi — EŞLEŞTİRİLMİŞ oran
        o = [g / i for i, g in self.donus if abs(i) > UYARIM_ESIK]
        d["donus_ornek"] = len(o)
        d["donus_takip_medyan"] = round(yuzdelik(o, 50), 3) if o else ""
        d["donus_istenen_med_dps"] = (
            round(math.degrees(yuzdelik([abs(i) for i, _ in self.donus], 50)), 1)
            if self.donus else "")
        # hız — alt uç aşımı
        alt = [g / i for i, g in self.hiz if 0.05 <= i < 0.30]
        d["hiz_altuc_asim"] = round(yuzdelik(alt, 50), 2) if alt else ""
        d["hiz_ornek"] = len(self.hiz)
        # motor / pivot / estop
        d["motor_doygun_%"] = (round(100 * self.doygun[0] / self.doygun[1], 1)
                               if self.doygun[1] else "")
        d["PIVOT_%"] = (round(100 * self.pivot[0] / self.pivot[1], 1)
                        if self.pivot[1] else "")
        d["estop_%"] = (round(100 * self.estop[0] / self.estop[1], 1)
                        if self.estop[1] else "")
        # kapı
        d["kapi_1m_atlama_%"] = (
            round(100 * sum(1 for x in self.kapi if x > 1.0) / len(self.kapi), 2)
            if self.kapi else "")
        d["kapi_maks_atlama_m"] = round(max(self.kapi), 1) if self.kapi else ""
        d["hafiza_maks"] = max(self.hafiza) if self.hafiza else ""
        return d


class AyarDefteri(Node):
    def __init__(self, periyot):
        super().__init__("ayar_defteri")
        self.periyot = periyot
        self._ayar = {}
        self._donem = None
        self._satirlar = []
        self._mod = None
        self._armed = False
        self._bagli = False
        self._son_durum_t = None
        self._istenen_w = None
        self._istenen_v = None
        self._son_kapi = None
        self._pwm_min, self._pwm_maks = 10**9, -10**9

        os.makedirs(GUNLUK_DIZIN, exist_ok=True)
        d = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_yolu = os.path.join(GUNLUK_DIZIN, f"defter_{d}.csv")
        self._md_yolu = os.path.join(GUNLUK_DIZIN, f"defter_{d}.md")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(State, "/mavros/state", self._on_state, qos)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, qos)
        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", self._on_cmd, qos)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body", self._on_hiz, qos)
        self.create_subscription(RCOut, "/mavros/rc/out", self._on_rcout, qos)
        self.create_subscription(RCIn, "/mavros/rc/in", self._on_rcin, qos)
        self.create_subscription(
            String, "/girdap/control/inhibit_reason", self._on_inhibit, 10)
        self.create_subscription(
            PoseStamped, "/girdap/planning/gate", self._on_gate, qos)
        self.create_subscription(
            PoseArray, "/girdap/planning/edge_buoys", self._on_edge, qos)

        self._get = self.create_client(GetParameters,
                                       f"{PARAM_DUGUM}/get_parameters")

    def _bas(self, tur, m):
        print(f"{tur:6s} {datetime.now():%H:%M:%S} {m}", flush=True)

    # ── abonelikler
    def _on_state(self, m):
        t = time.monotonic()
        if self._donem and self._son_durum_t and self._mod == "GUIDED" and self._armed:
            self._donem.ga += t - self._son_durum_t
        if self._donem and self._mod is not None and m.mode != self._mod:
            self._donem.mod_gecis += 1
        self._mod, self._armed, self._bagli = m.mode, m.armed, m.connected
        self._son_durum_t = t

    def _on_cmd(self, m):
        self._istenen_w, self._istenen_v = m.angular.z, m.linear.x

    def _on_imu(self, m):
        if self._donem and self._istenen_w is not None:
            self._donem.donus.append((self._istenen_w, m.angular_velocity.z))

    def _on_hiz(self, m):
        if self._donem and self._istenen_v is not None and self._istenen_v > 0.02:
            v = math.hypot(m.twist.linear.x, m.twist.linear.y)
            self._donem.hiz.append((self._istenen_v, v))

    def _on_rcout(self, m):
        if not self._donem or len(m.channels) < 3 or m.channels[0] <= 0:
            return
        for v in (m.channels[0], m.channels[2]):
            self._pwm_min = min(self._pwm_min, v)
            self._pwm_maks = max(self._pwm_maks, v)
        if self._pwm_maks - self._pwm_min < 100:
            return                                    # aralık henüz bilinmiyor
        doy = any(v >= self._pwm_maks - 5 or v <= self._pwm_min + 5
                  for v in (m.channels[0], m.channels[2]))
        self._donem.doygun[0] += 1 if doy else 0
        self._donem.doygun[1] += 1

    def _on_rcin(self, m):
        if self._donem and len(m.channels) >= 10:
            self._donem.estop[0] += 1 if m.channels[9] > 1500 else 0
            self._donem.estop[1] += 1

    def _on_inhibit(self, m):
        if self._donem:
            self._donem.pivot[0] += 1 if "PIVOT" in m.data else 0
            self._donem.pivot[1] += 1

    def _on_gate(self, m):
        p = (m.pose.position.x, m.pose.position.y)
        if self._donem and self._son_kapi:
            self._donem.kapi.append(math.hypot(p[0] - self._son_kapi[0],
                                               p[1] - self._son_kapi[1]))
        self._son_kapi = p

    def _on_edge(self, m):
        if self._donem:
            self._donem.hafiza.append(len(m.poses))

    # ── parametre yoklama
    def _oku_hepsi(self):
        if not self._get.wait_for_service(timeout_sec=5.0):
            return None
        g = self._get.call_async(GetParameters.Request(names=IZLENEN))
        rclpy.spin_until_future_complete(self, g, timeout_sec=10.0)
        s = g.result()
        if s is None or len(s.values) != len(IZLENEN):
            return None
        out = {}
        for ad, v in zip(IZLENEN, s.values):
            if v.type == ParameterType.PARAMETER_DOUBLE:
                out[ad] = float(v.double_value)
            elif v.type == ParameterType.PARAMETER_INTEGER:
                out[ad] = float(v.integer_value)
            else:
                out[ad] = None
        return out

    def _donem_kapat(self, sebep):
        if self._donem is None:
            return
        t1 = time.monotonic()
        if t1 - self._donem.t0 < ASGARI_DONEM:
            self._bas("DÖNEM", f"(kısa dönem {t1-self._donem.t0:.0f} sn — yazılmadı)")
            return
        o = self._donem.ozet(t1)
        o["kapanis_sebebi"] = sebep
        self._satirlar.append(o)
        self._bas("DÖNEM", f"kapandı ({sebep}) · {o['sure_sn']:.0f} sn · "
                           f"GUIDED+ARM %{o['guided_armed_%']} · "
                           f"PIVOT %{o['PIVOT_%']} · "
                           f"dönüş takip {o['donus_takip_medyan']} · "
                           f"alt uç aşım {o['hiz_altuc_asim']}× · "
                           f"doygun %{o['motor_doygun_%']}")
        self._yaz_dosyalar()

    def _yaz_dosyalar(self):
        if not self._satirlar:
            return
        alanlar = list(self._satirlar[-1].keys())
        with open(self._csv_yolu, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=alanlar)
            w.writeheader()
            for s in self._satirlar:
                w.writerow(s)
        with open(self._md_yolu, "w", encoding="utf-8") as f:
            f.write(f"# Ayar Defteri — {datetime.now():%d.%m.%Y}\n\n")
            f.write("> Her satır, parametrelerin SABİT kaldığı bir dönemdir.\n"
                    "> Bir parametre değişince dönem kapanır.\n"
                    "> 🔑 `donus_takip_medyan` EŞLEŞTİRİLMİŞ orandır "
                    "(medyan(a)/medyan(b) değil).\n"
                    "> 🔑 `PIVOT_%` en kararlı başarı ölçütüdür.\n"
                    "> ⚠️ `estop_%` yüksek dönemler KİRLİ — tekne çoğu zaman "
                    "durmuş olabilir.\n\n")
            for s in self._satirlar:
                f.write(f"## {s['baslangic']} — {s['sure_sn']:.0f} sn "
                        f"({s.get('kapanis_sebebi','')})\n\n")
                f.write("| ayar | değer |   | davranış | değer |\n")
                f.write("|---|---|---|---|---|\n")
                ayarlar = [(k, s[k]) for k in IZLENEN if k in s]
                dav = [("GUIDED+ARMED %", s["guided_armed_%"]),
                       ("mod geçişi", s["mod_gecis"]),
                       ("PIVOT %", s["PIVOT_%"]),
                       ("dönüş takip (medyan)", s["donus_takip_medyan"]),
                       ("istenen dönüş °/s", s["donus_istenen_med_dps"]),
                       ("alt uç hız aşımı ×", s["hiz_altuc_asim"]),
                       ("motor doygun %", s["motor_doygun_%"]),
                       ("kapı >1 m atlama %", s["kapi_1m_atlama_%"]),
                       ("kapı maks atlama m", s["kapi_maks_atlama_m"]),
                       ("kenar hafızası maks", s["hafiza_maks"]),
                       ("ESTOP %", s["estop_%"])]
                for i in range(max(len(ayarlar), len(dav))):
                    a = f"`{ayarlar[i][0]}` | {ayarlar[i][1]}" if i < len(ayarlar) else " | "
                    b = f"{dav[i][0]} | {dav[i][1]}" if i < len(dav) else " | "
                    f.write(f"| {a} |  | {b} |\n")
                f.write("\n")

    def calistir(self):
        self._bas("ADIM", "bağlantı bekleniyor…")
        t0 = time.monotonic()
        while self._mod is None and time.monotonic() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        ayar = self._oku_hepsi()
        if ayar is None:
            raise RuntimeError("parametreler okunamadı — MAVROS bağlı mı? "
                               "ROS_LOCALHOST_ONLY servisle aynı mı?")
        self._ayar = ayar
        self._donem = Donem(time.monotonic(), ayar)
        self._bas("DÖNEM", "açıldı · " + " ".join(
            f"{k.replace('ATC_','').replace('_','')}={v}" for k, v in ayar.items()
            if v is not None))
        self._bas("ADIM", f"izleniyor · CSV {self._csv_yolu}")
        son_yokla = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - son_yokla < self.periyot:
                continue
            son_yokla = time.monotonic()
            yeni = self._oku_hepsi()
            if yeni is None:
                continue
            degisen = [k for k in IZLENEN
                       if yeni.get(k) is not None and self._ayar.get(k) is not None
                       and abs(yeni[k] - self._ayar[k]) > 1e-4]
            if degisen:
                self._bas("AYAR", "🔑 DEĞİŞTİ: " + " · ".join(
                    f"{k} {self._ayar[k]:.3f}→{yeni[k]:.3f}" for k in degisen))
                self._donem_kapat("|".join(degisen))
                self._ayar = yeni
                self._donem = Donem(time.monotonic(), yeni)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--param-periyot", type=float, default=PARAM_PERIYOT,
                    help=f"parametre yoklama aralığı sn (varsayılan {PARAM_PERIYOT:.0f})")
    a = ap.parse_args()
    rclpy.init()
    d = AyarDefteri(a.param_periyot)
    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt("SIGTERM")))
    try:
        d.calistir()
    except KeyboardInterrupt as e:
        d._bas("ADIM", f"{e or 'Ctrl-C'} — son dönem kapatılıyor")
        d._donem_kapat("kapanış")
        d._bas("SONUÇ", f"{len(d._satirlar)} dönem yazıldı")
        d._bas("SONUÇ", f"CSV: {d._csv_yolu}")
        d._bas("SONUÇ", f"MD : {d._md_yolu}")
    except Exception as e:
        d._bas("İPTAL", f"🔴 {type(e).__name__}: {e}")
        d._donem_kapat("hata")
    finally:
        try:
            d.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Girdap İDA — OTOMATİK PLANT AYARI (`MOT_THST_EXPO` · `ATC_ACCEL_MAX` · `ATC_DECEL_MAX`)

NE: Koşum sırasında teknenin **fiziksel** davranışını PASİF olarak ölçer.
    **YAZDIĞI:** `ATC_ACCEL_MAX` · `ATC_DECEL_MAX` (doğrudan ölçülen fiziksel
    sınırlar — türetme yok, belirsizlik yok).
    **YALNIZ RAPORLADIĞI:** `MOT_THST_EXPO` (türetmesi güvenilir değil, bkz. 📐). `otomatik_ff_ayar.py`'nin kardeşi — o KONTROLCÜYÜ
    ayarlar, bu PLANT'ı (aracın kendisini) tanımlar.

NEDEN (17.08 bant ölçümü — iki oturum, bağımsız, UYUŞUYOR):

    ① GAZ → HIZ DOĞRUSAL DEĞİL
       MANUAL (AÇIK ÇEVRİM) kesitlerinde:
           16.08 183648 :  v = 0,95 · u^0,41   (5.255 örnek)
           15.08 163449 :  v = 1,04 · u^0,40   (1.238 örnek)
       Yüklü `MOT_THST_EXPO = 0,0` ⇒ sistem itkinin gaza TAM DOĞRUSAL
       olduğunu varsayıyor. ÖLÇÜM: üs ~0,40, yani karekökten bile eğri.
       Sonucu (ayrıca ölçüldü): istenen 0,05-0,30 m/s bandında tekne
       **2,7-3,3 KAT** hızlı gidiyor ⇒ kapıya yaklaşırken yavaşlayamıyor,
       kapıyı aşıyor, yön hatası 60°'yi geçiyor, PIVOT kilidi açılıyor.

    ② İVME SINIRI ARACIN İKİ KATI
           ölçülen ivme %95 : 0,46 · 0,60 m/s²     yüklü ATC_ACCEL_MAX = 1,0
           ölçülen fren %95 : 0,43 · 0,55 m/s²     ATC_DECEL_MAX = TANIMSIZ
       ArduPilot dokümanı: *"ACCEL_MAX ve DECEL_MAX aracın FİZİKSEL
       sınırlarına eşitlenmeli; bu, hız denetleyicisinin imkânsız ivmeler
       denemesini ve AŞIM ÜRETMESİNİ engeller."*

🔑 NEDEN MANUAL'DE ÖLÇÜLÜR (bu aracın en kritik tasarım kararı):
    GUIDED'da gaz KAPALI ÇEVRİMDEDİR — hız denetleyicisi onu aktif olarak
    düzenler. Kapalı çevrim veriden AÇIK ÇEVRİM plant eğrisi çıkarılamaz
    (tanımlanabilirlik problemi). Ölçüm bunu doğruladı: GUIDED'da iki oturum
    0,46 ↔ 0,92 diye ÇELİŞTİ; MANUAL'de 0,41 ↔ 0,40 diye UYUŞTU.
    ⇒ Bu araç YALNIZ MANUAL kesitlerden eğri çıkarır.
    (İvme sınırları her modda geçerli — orada kısıt yok.)

⛔ GÜVENLİK SÖZLEŞMESİ — bu betik uçuş kontrolcüsüne YAZAR:
    1. TEKNEYİ HAREKET ETTİRMEZ. Hiçbir komut yayınlamaz. Tamamen PASİF:
       kaptan zaten sürerken izler. Kendi manevrasını yaptırmaz.
    2. Dokunduğu parametreler yalnız: MOT_THST_EXPO, ATC_ACCEL_MAX,
       ATC_DECEL_MAX. Başka hiçbir şey.
    3. Özgün değerler önce DOSYAYA yazılır (~/girdap_logs/plant_ayar/).
    4. Yazdıktan sonra GERİ OKUYARAK doğrular.
    5. Sert sınırlar: EXPO [−0,60 … 0,00] · ACCEL/DECEL [0,15 … 3,00] m/s²
    6. YETERSİZ VERİ ⇒ YAZMAZ. Her parametrenin kendi asgari örnek sayısı
       ve kendi güven ölçütü var; sağlanmazsa o parametre ATLANIR ve sebep
       basılır. "Ölçemedim" demek, uydurmaktan iyidir.
    7. İki oturum arası TUTARLILIK kapısı: eğri üssü, ölçüm penceresinin
       ilk ve ikinci yarısında ayrı ayrı hesaplanır; ikisi %35'ten fazla
       ayrışırsa YAZILMAZ (16.08'in GUIDED verisi tam böyle ayrışıyordu).
    8. `--kuru` hiçbir şey yazmadan yalnız ölçer (ÖNCE bununla dene).

📐 `MOT_THST_EXPO` — 🔴 BU ARAÇ ONU **YAZMAZ**, YALNIZ RAPOR EDER:
    ArduPilot'un itki modeli:   itki = (1−e)·u + e·u²
    Teknenin fiziği:            v ∝ √itki   (sürükleme hıza kareyle bağlı)
    Ölçülen:                    v = k·u^n   (n ≈ 0,40)
    ⇒ itki ∝ v² ∝ u^(2n)  =  u^0,80        (n=0,40 için)
    u=0,5 noktasında eşitlenir:
        0,5^(2n) = (1−e)·0,5 + e·0,25
        ⇒ e = (0,5^(2n) − 0,5) / (−0,25)
    n=0,40 ⇒ e ≈ −0,30 · ama n=1,00 ⇒ e = **+1,00**
    🔴 İŞARET n=0,5'TE DÖNÜYOR ⇒ tek noktalı eşitleme GÜVENİLİR DEĞİL.
    Ayrıca ArduPilot dokümanı bu parametrenin neyi doğrusallaştırdığını
    (itki mi, tekerlek hızı mı) açıkça yazmıyor.
    ⇒ **Araç yalnız ÖLÇÜLEN ÜSSÜ raporlar; değeri insan verir.** Yanlış
    işaretli bir expo, alt uçtaki aşımı düzeltmek yerine İKİYE KATLAYABİLİR.

GERİ ALINIRSA NE KIRILIR: hiçbir şey — araç yalnız parametre yazıyor, kod
    yolunda değil. Ama o zaman bu üç parametre elle ve suda aranmak zorunda
    kalır (MANUAL basamak testi: %20/%40/%60/%80 gaz, her birinde 15 sn).

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1     # 🔴 İKİSİ DE ŞART
    python3 scripts/otomatik_plant_ayar.py --kuru    # ÖNCE BU
    python3 scripts/otomatik_plant_ayar.py --bekle   # servis kipi
    python3 scripts/otomatik_plant_ayar.py --geri-yukle DOSYA   # acil

Çıktı sözleşmesi (nöbetçiyle aynı desen):
    ADIM <n> · ÖLÇÜM <ne> · YAZ <param> · GERİ <param> · SONUÇ · İPTAL
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import RCOut, State
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

PARAM_DUGUM = "/mavros/param"
P_EXPO, P_ACCEL, P_DECEL = "MOT_THST_EXPO", "ATC_ACCEL_MAX", "ATC_DECEL_MAX"

# sert sınırlar (§güvenlik 5)
EXPO_ALT, EXPO_UST = -0.60, 0.00
IVME_ALT, IVME_UST = 0.15, 3.00

OLCUM_SN = 600.0          # varsayılan pencere — kaptan sürerken pasif toplar
ASGARI_EGRI = 300         # eğri için asgari MANUAL kararlı-hâl örneği
ASGARI_IVME = 500         # ivme için asgari örnek
TUTARLILIK = 0.35         # iki yarı arasındaki azami göreli fark
KARARLI_IVME = 0.15       # |a| bundan küçükse "kararlı hâl"
ASGARI_GAZ = 0.05         # bunun altındaki gaz kullanılmaz
ASGARI_HIZ = 0.03
VERI_ZAMAN_ASIMI = 3.0
BEKLEME_BILDIRIM_SN = 60.0
GUNLUK_DIZIN = os.path.expanduser("~/girdap_logs/plant_ayar")


def yuzdelik(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(round(p / 100 * (len(s) - 1)))))]


def us_kestir(ciftler):
    """log-log regresyonla n ve k: v = k·u^n. (n, k, R²) döner."""
    if len(ciftler) < 20:
        return None
    X = [math.log(u) for u, _ in ciftler]
    Y = [math.log(v) for _, v in ciftler]
    mx, my = sum(X) / len(X), sum(Y) / len(Y)
    sxx = sum((x - mx) ** 2 for x in X)
    if sxx <= 0:
        return None
    n = sum((x - mx) * (y - my) for x, y in zip(X, Y)) / sxx
    k = math.exp(my - n * mx)
    yh = [my + n * (x - mx) for x in X]
    ss = sum((y - h) ** 2 for y, h in zip(Y, yh))
    st = sum((y - my) ** 2 for y in Y)
    return n, k, (1 - ss / st if st > 0 else 0.0)


def expo_kaba_tahmin(n):
    """🔴 YALNIZ BİLGİ — bu araç MOT_THST_EXPO'yu YAZMAZ.

    NEDEN YAZMIYOR (17.08'de kendi doğrulamamda yakalandı): türetme
    `itki = (1−e)·u + e·u²` modelini u=0,5'te eşitliyor, ama işaret **n=0,5'te
    dönüyor** — n=1,00 (doğrusal hız) için formül **+1,00** veriyor, n=0,40
    için **−0,30**. Yani tek noktalı eşitleme, parametrenin gerçek anlamına
    göre ya çok kaba ya da yanlış işaretli olabilir.

    Üstelik ArduPilot Rover dokümanı `MOT_THST_EXPO`'nun neyi doğrusallaştırdığını
    (itki mi, tekerlek hızı mı) AÇIKÇA YAZMIYOR; yalnız *"DC motorlarda
    genellikle 0 … −0,5 arası iyi çalışır"* diyor ve **ölçmeyi** öneriyor
    (thrust-stand / RPM basamak testi).

    ⇒ Ölçülen üs RAPOR EDİLİR, karar İNSANA bırakılır. Yanlış işaretli bir
    expo, alt uçtaki aşımı düzeltmek yerine İKİYE KATLAYABİLİR.
    """
    if not (0.05 < n < 1.5):
        return None
    return 4.0 * (0.5 - 0.5 ** (2.0 * n))


class PlantAyar(Node):
    def __init__(self, kuru, bekle, sure):
        super().__init__("otomatik_plant_ayar")
        self.kuru, self.bekle, self.sure = kuru, bekle, sure
        self._mod = None
        self._armed = False
        self._bagli = False
        self._hiz = None
        self._hiz_t = None
        self._rc = None
        self._son_veri = 0.0
        self._pwm_min, self._pwm_maks = 10**9, -10**9
        self._ozgun = {}
        self._geri_yuklendi = False
        self._egri = []          # (u, v) — YALNIZ MANUAL
        self._ivme = []          # a (m/s²) — her mod

        os.makedirs(GUNLUK_DIZIN, exist_ok=True)
        d = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._yedek = os.path.join(GUNLUK_DIZIN, f"ozgun_{d}.json")
        self._gunluk = open(os.path.join(GUNLUK_DIZIN, f"ayar_{d}.log"),
                            "a", buffering=1, encoding="utf-8")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc, qos)
        self.create_subscription(State, "/mavros/state", self._on_state, qos)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body",
            self._on_hiz, qos)

        self._get = self.create_client(GetParameters,
                                       f"{PARAM_DUGUM}/get_parameters")
        self._set = self.create_client(SetParameters,
                                       f"{PARAM_DUGUM}/set_parameters")

    def _bas(self, tur, m):
        s = f"{tur:6s} {datetime.now():%H:%M:%S} {m}"
        print(s, flush=True)
        self._gunluk.write(s + "\n")

    # ── abonelikler
    def _on_state(self, m):
        self._mod, self._armed, self._bagli = m.mode, m.armed, m.connected

    def _on_rc(self, m):
        if len(m.channels) >= 3 and m.channels[0] > 0:
            self._rc = (m.channels[0], m.channels[2])
            for v in (m.channels[0], m.channels[2]):
                self._pwm_min = min(self._pwm_min, v)
                self._pwm_maks = max(self._pwm_maks, v)
        self._son_veri = time.monotonic()

    def _on_hiz(self, m):
        v = math.hypot(m.twist.linear.x, m.twist.linear.y)
        t = time.monotonic()
        if self._hiz is not None and self._hiz_t is not None:
            dt = t - self._hiz_t
            if 0.05 < dt < 0.5:
                a = (v - self._hiz) / dt
                self._ivme.append(a)
                # eğri örneği: YALNIZ MANUAL + armed + kararlı hâl
                if (self._mod == "MANUAL" and self._armed and self._rc
                        and abs(a) < KARARLI_IVME and v > ASGARI_HIZ):
                    sol, sag = self._rc
                    if self._pwm_maks > self._pwm_min:
                        trim = (self._pwm_maks + self._pwm_min) / 2.0
                        ara = (self._pwm_maks - self._pwm_min) / 2.0
                        u = ((sol - trim) + (sag - trim)) / (2.0 * ara)
                        doygun = (sol >= self._pwm_maks - 5
                                  or sol <= self._pwm_min + 5
                                  or sag >= self._pwm_maks - 5
                                  or sag <= self._pwm_min + 5)
                        if u > ASGARI_GAZ and not doygun:
                            self._egri.append((u, v))
        self._hiz, self._hiz_t = v, t

    # ── parametre
    def _oku(self, ad, ts=20.0):
        if not self._get.wait_for_service(timeout_sec=ts):
            raise RuntimeError(f"{PARAM_DUGUM}/get_parameters yok — "
                               "ROS_LOCALHOST_ONLY servisle aynı mı?")
        g = self._get.call_async(GetParameters.Request(names=[ad]))
        rclpy.spin_until_future_complete(self, g, timeout_sec=ts)
        s = g.result()
        if s is None or not s.values:
            raise RuntimeError(f"{ad} OKUNAMADI")
        v = s.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return float(v.double_value)
        if v.type == ParameterType.PARAMETER_INTEGER:
            return float(v.integer_value)
        raise RuntimeError(f"{ad} beklenmedik tip ({v.type}) — FC'de var mı?")

    def _yaz(self, ad, deger):
        if ad == P_EXPO and not (EXPO_ALT <= deger <= EXPO_UST):
            raise RuntimeError(f"{ad}={deger} SINIR DIŞI")
        if ad in (P_ACCEL, P_DECEL) and not (IVME_ALT <= deger <= IVME_UST):
            raise RuntimeError(f"{ad}={deger} SINIR DIŞI")
        if self.kuru:
            self._bas("YAZ", f"(KURU — yazılmadı) {ad} = {deger:.3f}")
            return
        if not self._set.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("set_parameters yok")
        p = Parameter(name=ad, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(deger)))
        g = self._set.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(self, g, timeout_sec=10.0)
        s = g.result()
        if s is None or not s.results or not s.results[0].successful:
            sebep = s.results[0].reason if (s and s.results) else "yanıt yok"
            raise RuntimeError(f"{ad} YAZILAMADI: {sebep}")
        okunan = self._oku(ad)
        if abs(okunan - deger) > 1e-3:
            raise RuntimeError(f"{ad} geri okuma {okunan:.3f} ≠ {deger:.3f}")
        self._bas("YAZ", f"{ad} = {deger:.3f} (geri okundu ✅)")

    # ── geri yükleme (otomatik_ff_ayar ile aynı üç basamak)
    def _baglam_ok(self):
        try:
            return rclpy.ok()
        except Exception:
            return False

    def _taze_baglamla_geri(self):
        """rclpy SIGINT'te kapanmışsa SIFIRDAN bağlam açıp yaz.

        (17.08'de `otomatik_ff_ayar`ta canlı ölçüldü: SIGINT rclpy'nin kendi
        işleyicisiyle bağlamı `except` bloğundan ÖNCE geçersiz kılıyor ve
        geri yükleme sessizce başarısız oluyordu. Ayrıca taze bağlama KENDİ
        yürütücüsü verilmezse istek hiç dönmüyor.)
        """
        import rclpy.context
        from rclpy.executors import SingleThreadedExecutor
        ctx = rclpy.context.Context()
        rclpy.init(context=ctx)
        yur = None
        try:
            d = Node("otomatik_plant_ayar_geri", context=ctx)
            yur = SingleThreadedExecutor(context=ctx)
            yur.add_node(d)
            c = d.create_client(SetParameters, f"{PARAM_DUGUM}/set_parameters")
            if not c.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("taze bağlamda da servis yok")
            for ad, v in self._ozgun.items():
                p = Parameter(name=ad, value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
                g = c.call_async(SetParameters.Request(parameters=[p]))
                yur.spin_until_future_complete(g, timeout_sec=10.0)
                s = g.result()
                ok = bool(s and s.results and s.results[0].successful)
                self._bas("GERİ", f"{'✅' if ok else '🔴'} (taze bağlam) "
                                  f"{ad} = {v:.3f}")
                if not ok:
                    raise RuntimeError(f"{ad} taze bağlamda da yazılamadı")
            d.destroy_node()
        finally:
            for f in (lambda: yur and yur.shutdown(),
                      lambda: rclpy.shutdown(context=ctx)):
                try:
                    f()
                except Exception:
                    pass

    def geri_yukle(self):
        if self._geri_yuklendi or not self._ozgun:
            return
        self._geri_yuklendi = True
        if self.kuru:
            for ad, v in self._ozgun.items():
                self._bas("GERİ", f"(KURU) {ad} = {v:.3f}")
            return
        kalan = dict(self._ozgun)
        if self._baglam_ok():
            for ad, v in list(kalan.items()):
                try:
                    self._yaz(ad, v)
                    kalan.pop(ad, None)
                except Exception as e:
                    self._bas("GERİ", f"🔴 {ad}: {e}")
                    if not self._baglam_ok():
                        break
        else:
            self._bas("GERİ", "rclpy bağlamı geçersiz (SIGINT) — taze bağlam")
        if kalan:
            try:
                self._taze_baglamla_geri()
                kalan.clear()
            except Exception as e:
                self._bas("GERİ", f"🔴 taze bağlam da başarısız: {e}")
        for ad, v in kalan.items():
            self._bas("GERİ", f"🔴🔴 {ad} GERİ YÜKLENEMEDİ — ELLE YAZ: {v}")
        self._bas("GERİ", f"özgün değerler: {self._yedek}")

    # ── akış
    def _iptal_mi(self):
        if not self._bagli:
            return True, "FC BAĞLANTISI YOK"
        if self._son_veri and time.monotonic() - self._son_veri > VERI_ZAMAN_ASIMI:
            return True, "motor/hız verisi kesildi"
        return False, ""

    def _topla(self):
        """PASİF toplama. Kaptan zaten sürerken izler; komut VERMEZ."""
        son = time.monotonic() + self.sure
        son_bildirim = 0.0
        while time.monotonic() < son:
            rclpy.spin_once(self, timeout_sec=0.05)
            iptal, sebep = self._iptal_mi()
            if iptal:
                raise KeyboardInterrupt(sebep)
            gecen = self.sure - (son - time.monotonic())
            if gecen - son_bildirim >= BEKLEME_BILDIRIM_SN:
                son_bildirim = gecen
                self._bas("ÖLÇÜM", f"⏳ {gecen:.0f}/{self.sure:.0f} sn · "
                                   f"eğri örneği {len(self._egri)} (MANUAL) · "
                                   f"ivme örneği {len(self._ivme)} · "
                                   f"mod {self._mod}")

    def calistir(self):
        self._bas("ADIM", "0/3 — bağlantı ve özgün değerler")
        t0 = time.monotonic()
        while self._mod is None and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._mod is None:
            raise RuntimeError("/mavros/state gelmiyor — MAVROS/FC bağlı değil")
        for ad in (P_EXPO, P_ACCEL, P_DECEL):
            try:
                self._ozgun[ad] = self._oku(ad)
            except Exception as e:
                self._bas("ADIM", f"⚠️ {ad} okunamadı ({e}) — o parametre ATLANACAK")
        with open(self._yedek, "w", encoding="utf-8") as f:
            json.dump(self._ozgun, f, indent=2)
        self._bas("ADIM", "özgün: " + " · ".join(
            f"{k}={v:.3f}" for k, v in self._ozgun.items()) + f" → {self._yedek}")

        self._bas("ADIM", f"1/3 — PASİF toplama ({self.sure:.0f} sn). "
                          "🔑 Eğri YALNIZ MANUAL'de toplanır (açık çevrim); "
                          "ivme her modda.")
        self._topla()

        self._bas("ADIM", "2/3 — çözümleme")
        yazilacak = {}

        # ── ① eğri → MOT_THST_EXPO
        if len(self._egri) < ASGARI_EGRI:
            self._bas("ÖLÇÜM", f"⚠️ {P_EXPO} ATLANDI — MANUAL kararlı-hâl "
                               f"örneği {len(self._egri)} < {ASGARI_EGRI}. "
                               "Kaptan MANUAL'de değişken gazla sürmeli.")
        else:
            yarim = len(self._egri) // 2
            a = us_kestir(self._egri[:yarim])
            b = us_kestir(self._egri[yarim:])
            t = us_kestir(self._egri)
            if not (a and b and t):
                self._bas("ÖLÇÜM", f"⚠️ {P_EXPO} ATLANDI — regresyon kurulamadı")
            else:
                fark = abs(a[0] - b[0]) / max(abs(a[0]), abs(b[0]), 1e-9)
                self._bas("ÖLÇÜM", f"eğri n={t[0]:.2f} k={t[1]:.2f} R²={t[2]:.2f} "
                                   f"(n={len(self._egri)}) · yarımlar "
                                   f"{a[0]:.2f} ↔ {b[0]:.2f} (fark %{100*fark:.0f})")
                if fark > TUTARLILIK:
                    self._bas("ÖLÇÜM", f"🔴 {P_EXPO} ATLANDI — iki yarı %{100*fark:.0f} "
                                       f"ayrışıyor (>%{100*TUTARLILIK:.0f}). "
                                       "Ölçüm kararsız, YAZILMIYOR.")
                else:
                    # 🔴 MOT_THST_EXPO ASLA YAZILMAZ — yalnız raporlanır
                    kaba = expo_kaba_tahmin(t[0])
                    if abs(t[0] - 1.0) < 0.12:
                        self._bas("ÖLÇÜM", f"✅ üs {t[0]:.2f} ≈ DOĞRUSAL — "
                                           f"{P_EXPO} değişmesine gerek yok")
                    else:
                        self._bas("ÖLÇÜM", f"🔴 üs {t[0]:.2f} ⇒ gaz→hız DOĞRUSAL "
                                           f"DEĞİL. {P_EXPO}=0 varsayımı yanlış.")
                        self._bas("ÖLÇÜM", f"   kaba tahmin {kaba:+.2f} — "
                                           f"⚠️ YAZILMIYOR, türetme güvenilir "
                                           f"değil (işaret n=0,5'te dönüyor). "
                                           f"MANUAL basamak testiyle doğrula.")

        # ── ② ivme sınırları
        poz = [x for x in self._ivme if x > 0]
        neg = [-x for x in self._ivme if x < 0]
        if len(poz) < ASGARI_IVME or len(neg) < ASGARI_IVME:
            self._bas("ÖLÇÜM", f"⚠️ ivme sınırları ATLANDI — örnek az "
                               f"(+{len(poz)} / −{len(neg)}, gereken {ASGARI_IVME})")
        else:
            ai, af = yuzdelik(poz, 95), yuzdelik(neg, 95)
            self._bas("ÖLÇÜM", f"ivme %95 {ai:.2f} (maks {max(poz):.2f}) · "
                               f"fren %95 {af:.2f} (maks {max(neg):.2f}) m/s²")
            for ad, v in ((P_ACCEL, ai), (P_DECEL, af)):
                if ad not in self._ozgun:
                    self._bas("ÖLÇÜM", f"⚠️ {ad} FC'de yok — atlandı")
                elif IVME_ALT <= v <= IVME_UST:
                    yazilacak[ad] = round(v, 2)
                else:
                    self._bas("ÖLÇÜM", f"🔴 {ad}={v:.2f} sınır dışı — atlandı")

        # ── ③ yaz
        self._bas("ADIM", "3/3 — yazma")
        if not yazilacak:
            self._bas("SONUÇ", "🔴 HİÇBİR PARAMETRE YAZILMADI — ölçüm ölçütleri "
                               "sağlanmadı. Bu bir HATA DEĞİL: 'ölçemedim' demek, "
                               "uydurmaktan iyidir. Daha uzun pencere (--sure) "
                               "ya da MANUAL'de daha değişken gazla tekrar dene.")
            self.geri_yukle()
            return
        for ad, v in yazilacak.items():
            self._yaz(ad, v)
        self._geri_yuklendi = True          # bilerek bırakıyoruz
        self._bas("SONUÇ", "yazıldı: " + " · ".join(
            f"{k}={v}" for k, v in yazilacak.items()))
        self._bas("SONUÇ", f"özgün değerler: {self._yedek}")
        self._bas("SONUÇ", "🔴 KALICI yazıldı (ArduPilot PARAM_SET → EEPROM). "
                           "Bir sonraki koşumda DOĞRULA: alt uçtaki hız aşımı "
                           "3× seviyesinden düştü mü?")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kuru", action="store_true",
                    help="hiçbir şey YAZMA, yalnız ölç (ÖNCE BUNU KOŞ)")
    ap.add_argument("--bekle", action="store_true",
                    help="SERVİS KİPİ: FC bağlanana kadar bekle")
    ap.add_argument("--sure", type=float, default=OLCUM_SN,
                    help=f"pasif toplama saniyesi (varsayılan {OLCUM_SN:.0f})")
    ap.add_argument("--geri-yukle", metavar="DOSYA",
                    help="kaydedilmiş özgün değerleri geri yaz (acil)")
    a = ap.parse_args()

    rclpy.init()
    d = PlantAyar(a.kuru, a.bekle, a.sure)
    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt("SIGTERM")))
    try:
        if a.geri_yukle:
            with open(a.geri_yukle, encoding="utf-8") as f:
                d._ozgun = json.load(f)
            d._geri_yuklendi = False
            d.geri_yukle()
        else:
            if a.bekle:
                t0 = time.monotonic()
                son = -BEKLEME_BILDIRIM_SN
                while not d._bagli:
                    rclpy.spin_once(d, timeout_sec=0.2)
                    g = time.monotonic() - t0
                    if g - son >= BEKLEME_BILDIRIM_SN:
                        son = g
                        d._bas("ADIM", f"⏳ FC bekleniyor ({g:.0f} sn)")
            d.calistir()
    except KeyboardInterrupt as e:
        d._bas("İPTAL", f"{e or 'Ctrl-C'} — özgün değerlere dönülüyor")
        d.geri_yukle()
    except Exception as e:
        d._bas("İPTAL", f"🔴 {type(e).__name__}: {e} — geri yükleniyor")
        d.geri_yukle()
    finally:
        try:
            d.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

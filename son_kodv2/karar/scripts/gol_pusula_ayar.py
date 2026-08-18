#!/usr/bin/env python3
"""
Girdap İDA — GÖLDE OTOMATİK PUSULA AYARI (sabit-yön yöntemi)

NE: Kaptan MANUAL'de DÜZ giderken, GPS'in yer rotasını gerçek yön kabul edip
    uçuş kontrolcüsüne **tek bir** `MAV_CMD_FIXED_MAG_CAL_YAW` (42006) komutu
    gönderir. ArduPilot dünya manyetik modelinden pusula ofsetlerini hesaplar.
    Tekneyi döndürmeye, elle "mag cal dansı" yapmaya gerek yok.

🔑 NEDEN BU YÖNTEM (tekne için doğru olan bu):
    Standart pusula kalibrasyonu aracın ÜÇ EKSENDE de döndürülmesini ister.
    Katamaran yalpa/yunuslama yapamaz — yalnız sapma (yaw) döner. ArduPilot
    tam bu yüzden "büyük araç" yöntemini sunuyor: aracı BİLİNEN bir yöne
    çevir, o yönü söyle, ofsetler dünya manyetik modelinden çıkarılsın.
    Bizde "bilinen yön" bedava: **GPS'in yer rotası** (course over ground),
    pusuladan tamamen bağımsız.

🔴 NEDEN GUIDED'DA DEĞİL (kaptanın önerisine cevap):
    Kalibrasyon pusula ofsetlerini DEĞİŞTİRİR ⇒ yön kestirimi o an güvenilmez
    olur. Tekne GUIDED'da kendi seyrederken bunu yapmak, seyrin dayandığı
    sinyali seyir sırasında oynatmak demektir. Ayrıca GUIDED'da dümen KAPALI
    ÇEVRİMDİR: denetleyici rotayı sürekli düzeltir, "düz gidiyor" hâli
    aracın kendi eseridir, ölçüm değil.
    ⇒ Tetik **MANUAL + düz seyir**. Kaptan zaten göle varınca elle sürüyor;
    araç o sırada kendiliğinden yakalar. Elle uğraşma yok, GUIDED riski yok.

⛔ GÜVENLİK SÖZLEŞMESİ:
    1. TEKNEYİ HAREKET ETTİRMEZ. Hiçbir hız/yön komutu yayınlamaz. Kaptan
       zaten sürerken izler.
    2. Gönderdiği tek şey: `MAV_CMD_FIXED_MAG_CAL_YAW`. Başka komut yok.
    3. Komutu **bir kez** gönderir, sonra durur (`--surekli` verilmedikçe).
    4. GUIDED/AUTO'da **hiçbir koşulda** göndermez — mod kapısı serttir.
    5. Öncesi ve sonrası `COMPASS_OFS_*` okunup basılır; ayrıca yön hatası
       ölçülür (aşağıya bak) ⇒ "işe yaradı mı" iddia değil ÖLÇÜMDÜR.
    6. `--kuru` hiçbir şey göndermeden yalnız ölçer (ÖNCE bununla dene).

📐 "İŞE YARADI MI" NASIL ÖLÇÜLÜYOR — KONTROL GRUPLU:
    Düz seyirde pusula başlığı ile GPS yer rotası **aynı şeyi** göstermeli.
    Araç kalibrasyondan ÖNCE ve SONRA bu ikisi arasındaki farkı ayrı ayrı
    biriktirir. Hata belirgin düşmediyse kalibrasyon "başarılı" SAYILMAZ.
    (§1.43c'nin gürültü tabanı desenin yön karşılığı.)

🌊 AKINTI — YAN KAYMA YOK SAYILMAZ, ÖLÇÜLÜP ÇIKARILIR (kaptan: "akıntı var"):
    GPS yer rotası teknenin GİTTİĞİ yön; burnunun BAKTIĞI yön DEĞİL. Akıntı
    tekneyi yan kaydırır (crab) ve bu açı doğrudan pusula ofsetine yazılırdı.
    ⇒ Bu yüzden araç **İKİ TERS BACAK** ister ve akıntıyı ayrıştırır:

        V_yer1 = +V_su + V_akıntı        (1. bacak)
        V_yer2 = −V_su + V_akıntı        (ters bacak)
        V_akıntı = (V_yer1 + V_yer2) / 2
        V_su     = (V_yer1 − V_yer2) / 2   ← YÖNÜ gerçek başlıktır

    Suya göre hız işaret değiştirir, akıntı değiştirmez (yer çerçevesinde
    sabit) — ayrımı mümkün kılan tam olarak bu. Bedava kazanç: **akıntı
    vektörü** de ölçülmüş olur, görev planlaması için ayrıca değerli.

    ⚠️ VARSAYIM (kapılanıyor, varsayılmıyor): iki bacakta aynı gaz, akıntı
    iki bacak boyunca sabit. Suya göre hız `ASGARI_HIZ`'in altına düşerse ya
    da akıntı ondan büyük çıkarsa araç **sayı ilan etmez**.

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1     # 🔴 İKİSİ DE ŞART
    python3 scripts/gol_pusula_ayar.py --kuru        # ÖNCE BU (göndermez)
    python3 scripts/gol_pusula_ayar.py               # gölde; servis de bunu koşar

Sahada elle koşum İMKÂNSIZ (md 4.1: WiFi/BT kapalı ⇒ SSH yok) — bu yüzden
`girdap-pusula-ayar.service` olarak açılışta koşar. §1.41e'nin dersi:
elle başlatılması gereken kapı, sahada hiç açılmayan kapıdır.

Çıktı sözleşmesi: ADIM · ÖLÇÜM · YAZ · SONUÇ · İPTAL
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
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64

PARAM_DUGUM = "/mavros/param"
KOMUT_SERVIS = "/mavros/cmd/command"

# 🔑 MAV_CMD_FIXED_MAG_CAL_YAW — ardupilotmega.xml'de "42006 … moved to
# common.xml" notuyla doğrulandı; imza MAVProxy'nin `magcal yaw` kipinden:
#   param1 = yön (derece)  param2 = pusula maskesi (0 = hepsi)
#   param3 = enlem (0 = mevcut GPS)  param4 = boylam (0 = mevcut GPS)
KOMUT_SABIT_PUSULA = 42006
PUSULA_MASKESI = 0.0

OFS = ("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z")
P_LEARN = "COMPASS_LEARN"

# ── düz seyir kapısı ────────────────────────────────────────────────────────
ASGARI_HIZ = 0.45         # m/s — altında GPS rotası gürültülü ve kayma büyür
AZAMI_DONUS = 8.0         # °/s — bundan hızlı dönüyorsa "düz" değil
ROTA_KARARLILIK = 6.0     # ° — pencere içi rota standart sapması bu altında
PENCERE_SN = 8.0          # bu kadar kesintisiz düz seyir gerekir
ASGARI_ORNEK = 40
ASGARI_FIX = 0            # NavSatFix.status: 0 = tek nokta, 2 = RTK

# ── "işe yaradı mı" ölçütü ──────────────────────────────────────────────────
IYILESME_ORANI = 0.5      # sonraki hata, öncekinin en fazla yarısı olmalı
BACAK_TERS_TOL = 40.0     # 2. bacak, 1.'nin tersinden bu kadar sapabilir
                          # (akıntı zaten rotayı kaydırır — dar tutma)

VERI_ZAMAN_ASIMI = 5.0
BILDIRIM_SN = 20.0
YENIDEN_DENE_SN = 30.0    # servis kipinde iki deneme arası
GUNLUK_DIZIN = os.path.expanduser("~/girdap_logs/pusula_ayar")


def aci_farki(a, b):
    """İki başlık arasındaki işaretli fark, (−180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def sarmala(aci):
    """[0, 360) aralığına indir.

    ⚠️ Düz `% 360.0` YETMEZ: çok küçük negatif bir sayı (atan2'nin −1e−16'sı)
    kayan noktada `360.0` döndürür — yani "kuzey" 0° yerine 360° basılır.
    Ölçüm zincirinin en başında yakalanmazsa raporlara sızar.
    """
    a = aci % 360.0
    return 0.0 if a >= 360.0 - 1e-9 else a


def aci_ortalama(acilar):
    """Dairesel ortalama — 359° ile 1°'in ortası 0°'dir, 180° değil."""
    if not acilar:
        return None
    s = sum(math.sin(math.radians(a)) for a in acilar)
    c = sum(math.cos(math.radians(a)) for a in acilar)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return sarmala(math.degrees(math.atan2(s, c)))


def aci_sapma(acilar):
    """Dairesel standart sapma (derece)."""
    if len(acilar) < 2:
        return None
    ort = aci_ortalama(acilar)
    if ort is None:
        return None
    kare = sum(aci_farki(a, ort) ** 2 for a in acilar) / len(acilar)
    return math.sqrt(kare)


def akinti_coz(yer1, yer2):
    """İki TERS bacaktan akıntıyı ve gerçek yönü ayrıştır.

    🔑 GÖLDE AKINTI VAR — bu fonksiyon onun cevabı.
    Yer hızı = suya göre hız + akıntı. İki ters bacakta suya göre hız işaret
    değiştirir, akıntı DEĞİŞTİRMEZ (akıntı yer çerçevesinde sabittir):

        V_yer1 = +V_su + V_akıntı
        V_yer2 = −V_su + V_akıntı
        ⇒ V_akıntı = (V_yer1 + V_yer2) / 2      (toplarsan V_su gider)
        ⇒ V_su     = (V_yer1 − V_yer2) / 2      (çıkarırsan akıntı gider)

    `V_su`'nun YÖNÜ teknenin burnunun baktığı gerçek yöndür — GPS rotası
    değil. Yan kayma (crab) böylece yok sayılmaz, **ölçülüp çıkarılır**.

    Bedava kazanç: akıntı vektörünün kendisi de elde kalır; görev planlaması
    için ayrıca değerlidir.

    ⚠️ VARSAYIM: iki bacakta suya göre hız BÜYÜKLÜĞÜ aynı (kaptan aynı gazı
    verdi) ve akıntı iki bacak boyunca sabit. İkisi de çağıran tarafta
    kapılanır — burada varsayılmaz, belgelenir.

    (doğu, kuzey) ikilileri alır; (akıntı, su_hızı, gerçek_yön_derece) döner.
    """
    ak = ((yer1[0] + yer2[0]) / 2.0, (yer1[1] + yer2[1]) / 2.0)
    su = ((yer1[0] - yer2[0]) / 2.0, (yer1[1] - yer2[1]) / 2.0)
    if math.hypot(*su) < 1e-9:
        return ak, su, None
    return ak, su, sarmala(math.degrees(math.atan2(su[0], su[1])))


class PusulaAyar(Node):
    def __init__(self, kuru, bekle=False):
        super().__init__("gol_pusula_ayar")
        self.kuru, self.bekle = kuru, bekle
        self._mod = None
        self._armed = False
        self._bagli = False
        self._fix = None
        self._pusula = None          # derece, kuzeyden saat yönü
        self._donus = None           # °/s
        self._son_veri = 0.0
        self._pencere = []           # (t, rota, hiz, pusula)
        self._gonderildi = False
        self._hata_once = None
        self._hata_sonra = None

        os.makedirs(GUNLUK_DIZIN, exist_ok=True)
        d = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._yedek = os.path.join(GUNLUK_DIZIN, f"ozgun_{d}.json")
        self._gunluk = open(os.path.join(GUNLUK_DIZIN, f"ayar_{d}.log"),
                            "a", buffering=1, encoding="utf-8")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(State, "/mavros/state", self._on_state, qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/raw/fix",
                                 self._on_fix, qos)
        self.create_subscription(TwistStamped,
                                 "/mavros/global_position/raw/gps_vel",
                                 self._on_gps_vel, qos)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 self._on_pusula, qos)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, qos)

        self._get = self.create_client(GetParameters,
                                       f"{PARAM_DUGUM}/get_parameters")
        self._komut = self.create_client(CommandLong, KOMUT_SERVIS)

    def _bas(self, tur, m):
        s = f"{tur:6s} {datetime.now():%H:%M:%S} {m}"
        print(s, flush=True)
        self._gunluk.write(s + "\n")

    # ── abonelikler
    def _on_state(self, m):
        self._mod, self._armed, self._bagli = m.mode, m.armed, m.connected

    def _on_fix(self, m):
        self._fix = m.status.status

    def _on_pusula(self, m):
        self._pusula = sarmala(float(m.data))

    def _on_imu(self, m):
        self._donus = math.degrees(m.angular_velocity.z)

    def _on_gps_vel(self, m):
        """MAVROS ENU yayınlar: x=Doğu, y=Kuzey ⇒ rota = atan2(Doğu, Kuzey)."""
        dogu, kuzey = m.twist.linear.x, m.twist.linear.y
        hiz = math.hypot(dogu, kuzey)
        self._son_veri = time.monotonic()
        if hiz < 1e-6:
            return
        rota = sarmala(math.degrees(math.atan2(dogu, kuzey)))
        if self._pusula is None or self._donus is None:
            return
        self._pencere.append((time.monotonic(), rota, hiz, self._pusula, dogu, kuzey))
        kes = time.monotonic() - PENCERE_SN
        self._pencere = [p for p in self._pencere if p[0] >= kes]

    # ── kapılar
    def _mod_uygun_mu(self):
        """🔴 SERT KAPI: yalnız MANUAL. GUIDED/AUTO'da asla."""
        return self._mod == "MANUAL"

    def _duz_seyir_mi(self):
        """Pencere boyunca kesintisiz düz seyir var mı? (uygun, sebep, veri)"""
        if not self._armed:
            return False, "tekne DISARM", None
        if not self._mod_uygun_mu():
            return False, f"mod {self._mod} — yalnız MANUAL (bkz. başlık)", None
        if self._fix is None or self._fix < ASGARI_FIX:
            return False, f"GPS fix yok (status={self._fix})", None
        if len(self._pencere) < ASGARI_ORNEK:
            return False, f"örnek {len(self._pencere)}/{ASGARI_ORNEK}", None
        if self._pencere[-1][0] - self._pencere[0][0] < PENCERE_SN * 0.8:
            return False, "pencere henüz dolmadı", None
        hizlar = [p[2] for p in self._pencere]
        if min(hizlar) < ASGARI_HIZ:
            return False, (f"hız {min(hizlar):.2f} < {ASGARI_HIZ} m/s "
                           "(yavaşta GPS rotası gürültülü, yan kayma büyür)"), None
        if self._donus is not None and abs(self._donus) > AZAMI_DONUS:
            return False, f"dönüş {abs(self._donus):.1f}°/s — düz git", None
        rotalar = [p[1] for p in self._pencere]
        sap = aci_sapma(rotalar)
        if sap is None:
            # Dairesel ortalama tanımsız (rotalar çembere yayılmış). Bu "sapma
            # büyük"ten daha kötü bir hâl: ortalama diye basılacak bir sayı YOK.
            return False, ("rota sapması hesaplanamadı — rotalar çembere "
                           "yayılmış, düz seyir değil"), None
        if sap > ROTA_KARARLILIK:
            return False, f"rota sapması {sap:.1f}° > {ROTA_KARARLILIK}°", None
        rota = aci_ortalama(rotalar)
        pus = aci_ortalama([p[3] for p in self._pencere])
        return True, "", (rota, pus, sap, len(self._pencere))

    # ── akıntı ayrıştırma (iki ters bacak)
    def _bacak_vektoru(self):
        """Pencerede biriken YER HIZI vektörünün ortalaması (doğu, kuzey)."""
        if not self._pencere:
            return None
        n = len(self._pencere)
        return (sum(p[4] for p in self._pencere) / n,
                sum(p[5] for p in self._pencere) / n)

    # ── parametre okuma
    def _oku(self, ad, ts=15.0):
        if not self._get.wait_for_service(timeout_sec=ts):
            return None
        g = self._get.call_async(GetParameters.Request(names=[ad]))
        rclpy.spin_until_future_complete(self, g, timeout_sec=ts)
        s = g.result()
        if s is None or not s.values:
            return None
        v = s.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return float(v.double_value)
        if v.type == ParameterType.PARAMETER_INTEGER:
            return float(v.integer_value)
        return None

    def _ofsetleri_oku(self):
        return {a: self._oku(a) for a in OFS}

    def _ogrenme_kapisi(self):
        """🔴 `COMPASS_LEARN` açıkken bu araç ÖLÇEMEZ — çakışma kapısı.

        (17.08: takımdan biri `COMPASS_LEARN=3` (InFlight) param dosyasına
        işlemiş; canlı FC'de henüz 0. İkisi aynı anda yürürlükte olursa
        ofsetleri iki ayrı özne değiştirir.)

        Neden ölçemez: bu aracın "işe yaradı mı" ölçütü, kalibrasyondan
        ÖNCE ve SONRA aynı büyüklüğü karşılaştırmaktır. InFlight learning
        ofsetleri ölçüm penceresinin ORTASINDA da değiştirir ⇒ iyileşmenin
        kime ait olduğu ayrılamaz. Kontrol grubu bozulur.

        Karar aracın değil insanın: araç yalnız durumu bildirir ve
        ölçemeyeceğini söyler.
        """
        v = self._oku(P_LEARN)
        if v is None:
            self._bas("ADIM", f"⚠️ {P_LEARN} okunamadı — çakışma kapısı "
                              "uygulanamadı")
            return True
        if abs(v) < 0.5:
            return True
        self._bas("SONUÇ", f"🔴 {P_LEARN} = {v:.0f} (AÇIK) — bu araç ölçemez.")
        self._bas("SONUÇ", "   InFlight/EKF öğrenmesi ofsetleri ölçüm "
                           "penceresinin ortasında da değiştirir; öncesi↔sonrası "
                           "karşılaştırması anlamını yitirir.")
        self._bas("SONUÇ", "   İKİSİNDEN BİRİ seçilmeli — bu bir İNSAN kararı. "
                           f"Sabit-yön kalibrasyonu isteniyorsa {P_LEARN}=0 olmalı.")
        return False

    # ── komut
    def _kalibre_et(self, yon):
        if self.kuru:
            self._bas("YAZ", f"(KURU — gönderilmedi) FIXED_MAG_CAL_YAW "
                             f"yön={yon:.1f}°")
            return True
        if not self._komut.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{KOMUT_SERVIS} yok — MAVROS ayakta mı?")
        istek = CommandLong.Request(
            broadcast=False, command=KOMUT_SABIT_PUSULA, confirmation=0,
            param1=float(yon), param2=float(PUSULA_MASKESI),
            param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
        g = self._komut.call_async(istek)
        rclpy.spin_until_future_complete(self, g, timeout_sec=15.0)
        s = g.result()
        if s is None:
            raise RuntimeError("komut servisi yanıt vermedi")
        if not s.success:
            raise RuntimeError(f"FC komutu REDDETTİ (result={s.result})")
        self._bas("YAZ", f"FIXED_MAG_CAL_YAW gönderildi · yön={yon:.1f}° "
                         f"· maske={PUSULA_MASKESI:.0f} (hepsi) · result={s.result}")
        return True

    # ── hata ölçümü
    def _hata_biriktir(self, sure, etiket):
        """Düz seyirde |pusula − GPS rotası| topla. Kontrol grubu ölçümü."""
        son = time.monotonic() + sure
        hatalar = []
        son_bildirim = 0.0
        while time.monotonic() < son:
            rclpy.spin_once(self, timeout_sec=0.05)
            uygun, _sebep, veri = self._duz_seyir_mi()
            if uygun:
                rota, pus, _s, _n = veri
                hatalar.append(abs(aci_farki(pus, rota)))
            gecen = sure - (son - time.monotonic())
            if gecen - son_bildirim >= BILDIRIM_SN:
                son_bildirim = gecen
                self._bas("ÖLÇÜM", f"⏳ {etiket} {gecen:.0f}/{sure:.0f} sn · "
                                   f"geçerli örnek {len(hatalar)}")
        if not hatalar:
            return None
        return sum(hatalar) / len(hatalar)

    # ── akış
    def _bekle_duz_seyir(self):
        son_bildirim = -BILDIRIM_SN
        t0 = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._bagli:
                raise KeyboardInterrupt("FC BAĞLANTISI YOK")
            uygun, sebep, veri = self._duz_seyir_mi()
            if uygun:
                return veri
            gecen = time.monotonic() - t0
            if gecen - son_bildirim >= BILDIRIM_SN:
                son_bildirim = gecen
                self._bas("ADIM", f"⏳ düz seyir bekleniyor ({gecen:.0f} sn) — {sebep}")

    def _bekle_bacak_ters(self, rota1):
        """2. bacağı bekle — 1. bacağın TERSİ olmalı.

        Kapı gerekli: kaptan yanlışlıkla aynı yönde ikinci bir düz seyir
        yaparsa `akinti_coz` suya göre hızı sıfıra yakın bulur ve yön
        uydurulamaz hâle gelir. Ters olmayan bacak sessizce kabul edilmemeli.
        """
        hedef = sarmala(rota1 + 180.0)
        son_bildirim = -BILDIRIM_SN
        t0 = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._bagli:
                raise KeyboardInterrupt("FC BAĞLANTISI YOK")
            uygun, sebep, veri = self._duz_seyir_mi()
            if uygun:
                sapma = abs(aci_farki(veri[0], hedef))
                if sapma <= BACAK_TERS_TOL:
                    return veri
                sebep = (f"düz gidiyor ama yön {veri[0]:.0f}°, "
                         f"gereken ~{hedef:.0f}° (±{BACAK_TERS_TOL:.0f}°)")
            gecen = time.monotonic() - t0
            if gecen - son_bildirim >= BILDIRIM_SN:
                son_bildirim = gecen
                self._bas("ADIM", f"⏳ 2. bacak bekleniyor ({gecen:.0f} sn) — {sebep}")

    def calistir(self):
        self._bas("ADIM", "0/5 — bağlantı ve özgün ofsetler")
        t0 = time.monotonic()
        while self._mod is None and time.monotonic() - t0 < 20.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._mod is None:
            raise RuntimeError("/mavros/state gelmiyor — MAVROS/FC bağlı değil")

        if not self._ogrenme_kapisi():
            return

        once = self._ofsetleri_oku()
        with open(self._yedek, "w", encoding="utf-8") as f:
            json.dump(once, f, indent=2)
        self._bas("ADIM", "özgün ofset: " + " · ".join(
            f"{k.split('_')[-1]}={v:.1f}" if v is not None else f"{k}=YOK"
            for k, v in once.items()) + f" → {self._yedek}")

        self._bas("ADIM", "1/5 — 1. BACAK. KAPTAN: MANUAL'de DÜZ git, sabit "
                          "gazla. Araç kendiliğinden yakalar; komut VERMEZ.")
        rota1, pus1, sap1, n1 = self._bekle_duz_seyir()
        yer1 = self._bacak_vektoru()
        self._bas("ÖLÇÜM", f"1. bacak · GPS rotası {rota1:.1f}° · "
                           f"pusula {pus1:.1f}° · sapma {sap1:.1f}° ({n1} örnek)")

        self._bas("ADIM", "2/5 — kalibrasyon ÖNCESİ yön hatası ölçülüyor")
        self._hata_once = self._hata_biriktir(PENCERE_SN * 2, "önce")
        if self._hata_once is None:
            raise RuntimeError("öncesi ölçülemedi — düz seyir kopuyor")
        self._bas("ÖLÇÜM", f"ÖNCE: |pusula − GPS rotası| ortalama "
                           f"{self._hata_once:.1f}°")

        # ── 🔑 AKINTI: ikinci bacak olmadan gerçek yön BİLİNEMEZ
        self._bas("ADIM", f"3/5 — 🔄 KAPTAN: GERİ DÖN ve TERS yönde git "
                          f"(~{sarmala(rota1 + 180):.0f}°), AYNI gazla. "
                          "Akıntı ancak iki ters bacakla ölçülür.")
        self._pencere.clear()
        rota2, _pus2, _sap2, n2 = self._bekle_bacak_ters(rota1)
        yer2 = self._bacak_vektoru()
        self._bas("ÖLÇÜM", f"2. bacak · GPS rotası {rota2:.1f}° ({n2} örnek)")

        akinti, su, yon2_ham = akinti_coz(yer1, yer2)
        if yon2_ham is None:
            raise RuntimeError("iki bacaktan suya göre hız çıkmadı — "
                               "bacaklar gerçekten ters miydi?")
        ak_h, su_h = math.hypot(*akinti), math.hypot(*su)
        ak_yon = sarmala(math.degrees(math.atan2(akinti[0], akinti[1])))
        # yon2_ham 1. bacağın yönü; tekne ŞU AN 2. bacakta, yani tam tersinde.
        gercek_yon = sarmala(yon2_ham + 180.0)
        self._bas("ÖLÇÜM", f"🌊 AKINTI {ak_h:.2f} m/s → {ak_yon:.0f}° · "
                           f"suya göre hız {su_h:.2f} m/s")
        self._bas("ÖLÇÜM", f"gerçek yön (2. bacak) {gercek_yon:.1f}° ↔ "
                           f"ham GPS rotası {rota2:.1f}° · "
                           f"yan kayma {abs(aci_farki(rota2, gercek_yon)):.1f}°")
        if su_h < ASGARI_HIZ:
            raise RuntimeError(f"suya göre hız {su_h:.2f} < {ASGARI_HIZ} m/s — "
                               "iki bacak aynı gazla sürülmemiş olabilir")
        if ak_h > su_h:
            raise RuntimeError(f"akıntı ({ak_h:.2f}) suya göre hızdan ({su_h:.2f}) "
                               "BÜYÜK — bu ayrıştırma güvenilmez, daha hızlı sür")

        self._bas("ADIM", "4/5 — sabit-yön kalibrasyonu (yan kayma çıkarılmış)")
        self._kalibre_et(gercek_yon)
        self._gonderildi = True
        time.sleep(2.0)
        sonra = self._ofsetleri_oku()
        self._bas("ÖLÇÜM", "yeni ofset: " + " · ".join(
            f"{k.split('_')[-1]}={v:.1f}" if v is not None else f"{k}=YOK"
            for k, v in sonra.items()))
        degisti = any(
            once.get(k) is not None and sonra.get(k) is not None
            and abs(once[k] - sonra[k]) > 0.5 for k in OFS)
        if not self.kuru and not degisti:
            self._bas("SONUÇ", "🔴 OFSETLER DEĞİŞMEDİ — FC komutu kabul etmiş "
                               "görünüyor ama bir şey yazmadı. GPS fix var mı? "
                               "(bu yöntem dünya manyetik modeli için konum ister)")
            return

        self._bas("ADIM", "5/5 — kalibrasyon SONRASI yön hatası (kontrol)")
        self._hata_sonra = self._hata_biriktir(PENCERE_SN * 2, "sonra")
        if self._hata_sonra is None:
            self._bas("SONUÇ", "⚠️ sonrası ölçülemedi — düz seyir koptu. "
                               "Ofsetler YAZILDI ama doğrulanmadı; "
                               "tekrar düz gidip `--kuru` ile sına.")
            return
        oran = self._hata_sonra / max(self._hata_once, 1e-9)
        self._bas("ÖLÇÜM", f"SONRA: {self._hata_sonra:.1f}° "
                           f"(önce {self._hata_once:.1f}° · oran {oran:.2f})")
        if oran <= IYILESME_ORANI:
            self._bas("SONUÇ", f"✅ yön hatası {self._hata_once:.1f}° → "
                               f"{self._hata_sonra:.1f}° düştü. Bir sonraki "
                               "arm denemesinde DOĞRULA: `Check mag field` "
                               "uyarısı kalktı mı?")
        else:
            self._bas("SONUÇ", f"🔴 hata belirgin DÜŞMEDİ ({oran:.2f} > "
                               f"{IYILESME_ORANI}). Kalibrasyon başarılı "
                               "SAYILMIYOR. Muhtemel sebep: yan kayma (rüzgâr/"
                               f"akıntı) ya da teknede sabit manyetik kaynak. "
                               f"Özgün ofsetler: {self._yedek}")


def _servis_dongusu(dugum):
    """SERVİS KİPİ: kalibrasyon YAPILANA kadar yaşamaya devam et.

    🔴 NEDEN (§1.41e'nin dersi + §1.42/§1.43'ün kardeş araçlardaki hâli):
    sahada SSH yok, betiği elle başlatmak fiilen imkânsız. Ölçüm koşulu
    ise İNSANA bağlı: kaptan MANUAL'de iki ters bacak sürene kadar oluşmaz.
    Boot anında o koşul tanımı gereği yoktur ⇒ tek koşup çıkan araç sahada
    HİÇ çalışmaz ve `Restart=on-failure` onu diriltmez (0 ile çıkar).

    Döngüyü yalnız iki şey bitirir: kalibrasyonun YAPILMASI ve SIGTERM.
    """
    dene = 0
    while True:
        dene += 1
        try:
            dugum.calistir()
            if dugum._gonderildi:
                dugum._bas("SONUÇ", f"✅ kalibrasyon yapıldı ({dene}. denemede)"
                                    " — servis çıkıyor")
                return
            dugum._bas("ADIM", "koşullar oluşmadı — yeniden beklenecek")
        except KeyboardInterrupt as e:
            dugum._bas("İPTAL", f"{e or 'Ctrl-C'}")
            if str(e) in ("SIGTERM", ""):
                return
        except Exception as e:
            dugum._bas("İPTAL", f"🔴 {type(e).__name__}: {e}")
        dugum._bas("ADIM", f"🔁 servis kipi — {YENIDEN_DENE_SN:.0f} sn sonra "
                           f"{dene + 1}. deneme")
        try:
            time.sleep(YENIDEN_DENE_SN)
        except KeyboardInterrupt:
            return
        dugum._pencere.clear()
        dugum._hata_once = dugum._hata_sonra = None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kuru", action="store_true",
                    help="komut GÖNDERME, yalnız ölç (ÖNCE BUNU KOŞ)")
    ap.add_argument("--bekle", action="store_true",
                    help="SERVİS KİPİ: kalibrasyon yapılana kadar yaşa")
    a = ap.parse_args()

    rclpy.init()
    d = PusulaAyar(a.kuru, a.bekle)
    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt("SIGTERM")))
    try:
        if a.bekle:
            _servis_dongusu(d)
        else:
            d.calistir()
    except KeyboardInterrupt as e:
        d._bas("İPTAL", f"{e or 'Ctrl-C'}")
    except Exception as e:
        d._bas("İPTAL", f"🔴 {type(e).__name__}: {e}")
    finally:
        try:
            d.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

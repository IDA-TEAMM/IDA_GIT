#!/usr/bin/env python3
"""
Girdap İDA — OTOMATİK DÖNÜŞ HIZI AYARI (`ATC_STR_RAT_FF/P/I`)

NE: Koşum sırasında `ATC_STR_RAT_FF`'i aday değerler arasında sırayla dener,
    her adayda takip başarımını ve salınımı ÖLÇER, en iyisini seçip yazar.
    ArduPilot'un `rover-quicktune.lua` appletinin yaptığı işi Jetson'dan yapar.

NEDEN (16.08.2026 bant ölçümü):
    Göl bantlarından teknenin dönme kazancı çıkarıldı — ÜÇ bağımsız yolla
    (16.08'in İKİ ayrı bandı, GUIDED-only):
        medyan(direksiyon/dönüş)      0,644 · 0,671
        regresyon eğimi               0,566 · 0,533
        sistem kimliklendirme 1/K     0,591     (K=1,693 rad/s, τ=1,42 s)
    + bağımsız dördüncü doğrulama (paralel oturum, R = v/ω): tekne
      0,62 m/s'de 9,2°/s dönüyor ⇒ gerçek yarıçap 3,86 m; istenen 16,6°/s
      ⇒ 2,14 m. Yüklü TURN_RADIUS 1,0 m ⇒ tekne ayarlandığının ~3,9 katı
      genişlikte dönüyor.
    ❌ GERİ ALINDI — `ArduPilot formülü TURN_RADIUS×π/4 = 0,785`: formül
      aracın FİZİKSEL dönüş yarıçapını bekliyor, oraya AYAR parametresini
      koymuşum. Gerçek yarıçap 3,86 m ölçüldüğüne göre girdisi tartışmalı;
      delil sayılmıyor. (Uyarıyı paralel oturum verdi.)
    Kalan yollar **0,53-0,67** bandında. Yüklü değer **0,20** — üçte biri.
    Sonucu: istenen dönüş hızının yalnız %55-63'ü gerçekleşiyor, tekne
    kapıya yeterince keskin dönemiyor, etrafında geniş yaylar çiziyor.
    18:36 oturumunda ölçüldü: **840 sn kesintisiz aynı yöne dönüş**, net
    2,6 tam tur, 20,1 tur değerinde direksiyon eforu — verim %13.

    QuickTune yerine bu: `SCR_ENABLE`+reboot+SD kart+RC anahtarı gerekmiyor
    (SCR_ENABLE şu an 0 ve açmak EKF'i riske atabilir — bkz. §1.20g'de yeni
    açılan EKF kilidi). Ayrıca hedef bandı ZATEN ÖLÇÜLDÜ, yani kör arama
    değil odaklı süpürme yapıyoruz: 4 aday × 2 tur ≈ 11 dakika.

NEDEN SİMÜLASYONLA DEĞİL: bandın kendisinden çıkarılan model doğrulamayı
    GEÇEMEDİ — serbest koşum R² 1 sn'de +0,69 ama 5 sn'de −0,21. Yani
    teknenin dönüşü ~2 sn'den sonra direksiyonla açıklanmıyor (rüzgâr/akıntı/
    dalga baskın). Kapalı çevrim ancak SUDA ölçülebilir. SITL daha da kötü
    olurdu: jenerik tekne fiziği simüle eder, bizimkini değil.

⛔ GÜVENLİK SÖZLEŞMESİ — bu betik uçuş kontrolcüsüne YAZAR:
    1. TEKNEYİ HAREKET ETTİRMEZ. Hiçbir hız/yön komutu yayınlamaz. Yalnız üç
       parametre yazar. Tahrik komutu daima görev yığınından gelir.
    2. PASİF UYARIM: kendi manevrasını yaptırmaz, görevin ZATEN verdiği dönüş
       komutlarını izler. Yeterli dönüş yoksa o adayı ÖLÇMEZ, atlar.
    3. Dokunduğu parametreler yalnız: ATC_STR_RAT_FF, _P, _I. Başka hiçbir şey.
    4. Başlamadan önce üçünün özgün değeri hem belleğe hem DOSYAYA yazılır.
    5. ŞU HÂLLERDE ANINDA GERİ YÜKLER ve çıkar:
         Ctrl-C · beklenmedik hata · GUIDED'dan çıkış · disarm ·
         acil durdurma (RC10 HIGH) · FC bağlantısı kopması · veri kesilmesi
    6. FF için sert sınır: [0,10 … 1,00]. Dışına asla yazılmaz.
    7. `--kuru` ile hiçbir şey yazmadan yalnız ölçüm yapar (önce bununla dene).
    8. 🔴 GÜRÜLTÜ KAPISI: adaylar TUR TUR serpiştirilerek (A-B-A-B) ölçülür;
       en iyi ile ikincinin farkı, adayların KENDİ İÇİNDEKİ en büyük
       yayılımdan küçükse **SONUÇ İLAN EDİLMEZ** ve özgün değerlere dönülür.
       NEDEN: ölçütün gürültü tabanı ölçüldü — FF SABİT 0,20 iken, aynı
       oturumda 8 ardışık 75 sn penceresinde regresyon eğimi 0,471-0,863
       arası gezindi (yayılım 0,392). Tek pencereyle "en iyi" ilan etmek
       yazı-tura olurdu. Araç artık "kullanılabilir fark bulamadım"
       diyebiliyor — uydurmaktan iyidir.

GERİ ALINIRSA NE KIRILIR: hiçbir şey — araç yalnız parametre yazıyor, kod
    yolunda değil. Ama o zaman FF elle ve suda kademeli aranmak zorunda kalır
    (0,20 → 0,40 → 0,60, her adımda bant indirip çözümleyerek).

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1     # 🔴 İKİSİ DE ŞART
    python3 scripts/otomatik_ff_ayar.py --kuru       # ÖNCE BU

🔴 `ROS_LOCALHOST_ONLY=1` ATLANIRSA (17.08'de cihazda yaşandı): `ros2 topic
   list` 44 mavros konusunu GÖSTERİR, `topic info` yayıncıyı GÖSTERİR, ama
   HİÇBİR VERİ AKMAZ ve hiçbir servis bulunamaz. Servisler bu değişkenle
   koşuyor (`girdap-yarisma-localhost.conf`); farklı değerdeki bir süreç ayrı
   keşif dünyasında kalır. Belirti "MAVROS bağlı değil" gibi görünür — DEĞİLDİR.
    python3 scripts/otomatik_ff_ayar.py                       # gerçek (2 tur, ~11 dk)
    python3 scripts/otomatik_ff_ayar.py --tur 3               # daha güvenli, ~16 dk
    python3 scripts/otomatik_ff_ayar.py --tur 1               # ~5 dk, GÜRÜLTÜ
                                                              # KAPISI UYGULANAMAZ
    python3 scripts/otomatik_ff_ayar.py --geri-yukle DOSYA    # acil geri alma

Çıktı sözleşmesi (nöbetçiyle aynı desen — her satır TEK olay):
    ADIM   <n> <açıklama>      ÖLÇÜM  <ff> <sonuçlar>
    YAZ    <param> <değer>     GERİ   <param> <değer>
    SONUÇ  <özet>              İPTAL  <sebep>
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn, State
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Imu

# 🔴 MAVROS 2'de `/mavros/param/get` SERVİSİ YOKTUR (16.08.2026'da cihazda
#    ölçüldü: `ros2 node info /mavros/param` → yalnız `pull`, `set` (ParamSetV2)
#    ve STANDART ROS2 parametre API'si). MAVROS 1'in `ParamGet/ParamSet`
#    servisleri taşınmadı. Uçuş kontrolcüsü parametreleri `/mavros/param`
#    düğümünün ROS2 parametreleri olarak sunuluyor:
#        ros2 param get /mavros/param ATC_STR_RAT_FF   → Double value is: 0.2
#    Bu yüzden okuma/yazma standart `rcl_interfaces` servisleriyle yapılıyor.
PARAM_DUGUM = "/mavros/param"

# ── ayarlanacak parametreler ────────────────────────────────────────────────
PARAM_FF = "ATC_STR_RAT_FF"
PARAM_P = "ATC_STR_RAT_P"
PARAM_I = "ATC_STR_RAT_I"

# Adaylar: 0,20 = ŞU ANKİ (kıyas tabanı, ATLANMAZ) · 0,60 = ölçülen hedef.
# 0,80 hedefin üstünü de yoklar — salınım eşiği oradaysa görürüz.
ADAYLAR = (0.20, 0.40, 0.60, 0.80)
FF_ALT, FF_UST = 0.10, 1.00           # sert sınır (§güvenlik 6)

P_ORANI = 0.20                        # ArduPilot: P ≈ %20×FF, I ≈ P

OLCUM_SN = 75.0                       # aday başına ölçüm penceresi
OTURMA_SN = 4.0                       # parametre yazıldıktan sonra bekleme
ASGARI_ORNEK = 40                     # bu kadar uyarımlı örnek yoksa ÖLÇME
UYARIM_ESIK = math.radians(5.0)       # |istenen| bunun üstündeyse "uyarım var"

VERI_ZAMAN_ASIMI = 3.0                # sn — veri kesilirse iptal
BEKLEME_BILDIRIM_SN = 60.0            # servis kipinde "hâlâ bekliyorum" aralığı
GUNLUK_DIZIN = os.path.expanduser("~/girdap_logs/ff_ayar")


def yuzdelik(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(round(p / 100 * (len(s) - 1)))))]


class FFAyar(Node):
    def __init__(self, kuru: bool, adaylar, olcum_sn: float,
                 bekle: bool = False, tur: int = 2):
        super().__init__("otomatik_ff_ayar")
        self.kuru = kuru
        self.adaylar = adaylar
        self.olcum_sn = olcum_sn
        self.bekle = bekle
        self.tur = max(1, int(tur))

        self._istenen = None
        self._gerceklesen = None
        self._son_veri = 0.0
        self._mod = None
        self._armed = False
        self._baglı = False
        self._rc10 = None
        self._ozgun = {}
        self._geri_yuklendi = False

        os.makedirs(GUNLUK_DIZIN, exist_ok=True)
        damga = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._yedek_yolu = os.path.join(GUNLUK_DIZIN, f"ozgun_{damga}.json")
        self._gunluk_yolu = os.path.join(GUNLUK_DIZIN, f"ayar_{damga}.log")
        self._gunluk = open(self._gunluk_yolu, "a", buffering=1, encoding="utf-8")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, qos)
        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped",
            self._on_cmd, qos)
        self.create_subscription(State, "/mavros/state", self._on_state, qos)
        self.create_subscription(RCIn, "/mavros/rc/in", self._on_rc, qos)

        self._get = self.create_client(
            GetParameters, f"{PARAM_DUGUM}/get_parameters")
        self._set = self.create_client(
            SetParameters, f"{PARAM_DUGUM}/set_parameters")

    # ── çıktı
    def _bas(self, tur, metin):
        satir = f"{tur:6s} {datetime.now():%H:%M:%S} {metin}"
        print(satir, flush=True)
        self._gunluk.write(satir + "\n")

    # ── abonelikler
    def _on_imu(self, m):
        self._gerceklesen = float(m.angular_velocity.z)
        self._son_veri = time.monotonic()

    def _on_cmd(self, m):
        self._istenen = float(m.angular.z)

    def _on_state(self, m):
        self._mod, self._armed, self._baglı = m.mode, m.armed, m.connected

    def _on_rc(self, m):
        if len(m.channels) >= 10:
            self._rc10 = m.channels[9]

    # ── parametre erişimi
    def _oku(self, ad, zaman_asimi=20.0):
        if not self._get.wait_for_service(timeout_sec=zaman_asimi):
            raise RuntimeError(
                f"{PARAM_DUGUM}/get_parameters servisi yok — MAVROS ayakta mı? "
                "ROS_LOCALHOST_ONLY servisle AYNI olmalı (bkz. başlık)")
        gel = self._get.call_async(GetParameters.Request(names=[ad]))
        rclpy.spin_until_future_complete(self, gel, timeout_sec=zaman_asimi)
        s = gel.result()
        if s is None or not s.values:
            raise RuntimeError(f"{ad} OKUNAMADI (yanıt yok)")
        v = s.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return float(v.double_value)
        if v.type == ParameterType.PARAMETER_INTEGER:
            return float(v.integer_value)
        raise RuntimeError(f"{ad} beklenmedik tip ({v.type}) — FC'de var mı?")

    def _yaz(self, ad, deger, zorla=False):
        if ad == PARAM_FF and not (FF_ALT <= deger <= FF_UST) and not zorla:
            raise RuntimeError(f"{ad}={deger} SINIR DIŞI [{FF_ALT},{FF_UST}]")
        if self.kuru:
            self._bas("YAZ", f"(KURU — yazılmadı) {ad} = {deger:.3f}")
            return True
        if not self._set.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{PARAM_DUGUM}/set_parameters servisi yok")
        p = Parameter(name=ad, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(deger)))
        gel = self._set.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(self, gel, timeout_sec=10.0)
        s = gel.result()
        if s is None or not s.results or not s.results[0].successful:
            sebep = s.results[0].reason if (s and s.results) else "yanıt yok"
            raise RuntimeError(f"{ad}={deger} YAZILAMADI: {sebep}")
        # 🔑 GERİ OKUYARAK DOĞRULA — "yazdım" demek "yazıldı" demek değil.
        okunan = self._oku(ad)
        if abs(okunan - deger) > 1e-3:
            raise RuntimeError(
                f"{ad} yazıldı DENDİ ama geri okuma {okunan:.3f} ≠ {deger:.3f}")
        self._bas("YAZ", f"{ad} = {deger:.3f} (geri okundu ✅)")
        return True

    # ── güvenlik
    def _iptal_mi(self):
        """(iptal_mi, sebep) — GERİ DÖNÜLEMEZ hâller: hemen dur ve geri yükle."""
        if not self._baglı:
            return True, "FC BAĞLANTISI YOK"
        if not self._armed:
            return True, "DISARM"
        if self._rc10 is not None and self._rc10 > 1500:
            return True, f"ACİL DURDURMA etkin (RC10={self._rc10})"
        if self._son_veri and time.monotonic() - self._son_veri > VERI_ZAMAN_ASIMI:
            return True, "IMU verisi kesildi"
        return False, ""

    def _olculur_mu(self):
        """SADECE GUIDED'da ölçüm yapılır (Eyüp kararı, 17.08).

        MANUAL/AUTO'da denetleyici çevrimi bizim ölçtüğümüz şeyi yapmıyor:
        MANUAL'de setpoint yayınlanır ama UYGULANMAZ, AUTO'da hız/yön
        planlaması FC'nin kendi S-eğrisinden gelir. İkisinden de ölçülen
        oran YANILTICI olur (16.08 bant çözümlemesi bunu gösterdi).

        🔑 Mod GUIDED'dan çıkarsa İPTAL ETMEZ, DURAKLAR: göl oturumunda
        12:06-12:19 arası altı kez GUIDED↔AUTO geçişi ölçüldü (§1.21c); her
        geçişte ayarı iptal etmek aracı kullanılamaz kılardı.
        """
        return self._mod == "GUIDED"

    def _bekle(self, sn):
        """Spin ederek bekle; geri dönülemez hâlde istisna at."""
        son = time.monotonic() + sn
        while time.monotonic() < son:
            rclpy.spin_once(self, timeout_sec=0.05)
            iptal, sebep = self._iptal_mi()
            if iptal:
                raise KeyboardInterrupt(sebep)

    # ── ölçüm
    def _olc(self, ff):
        """Bir aday FF için takip ve salınım ölç. PASİF — komut vermez.

        Ölçüm saati YALNIZ GUIDED'da işler; başka modda geçen süre pencereden
        sayılmaz (yoksa MANUAL'de beklerken pencere boşa dolardı).
        """
        ornek = []
        onceki_w = None
        gecis = 0
        gecen = 0.0
        son_tik = time.monotonic()
        duraklama_bildirildi = False
        while gecen < self.olcum_sn:
            rclpy.spin_once(self, timeout_sec=0.05)
            iptal, sebep = self._iptal_mi()
            if iptal:
                raise KeyboardInterrupt(sebep)

            simdi = time.monotonic()
            dt = simdi - son_tik
            son_tik = simdi

            if not self._olculur_mu():
                if not duraklama_bildirildi:
                    self._bas("ÖLÇÜM", f"⏸ mod {self._mod} — GUIDED değil, "
                                       f"ölçüm DURAKLADI ({gecen:.0f}/"
                                       f"{self.olcum_sn:.0f} sn birikti)")
                    duraklama_bildirildi = True
                time.sleep(0.05)
                continue
            if duraklama_bildirildi:
                self._bas("ÖLÇÜM", "▶ GUIDED geri geldi, ölçüm sürüyor")
                duraklama_bildirildi = False

            gecen += dt
            if self._istenen is None or self._gerceklesen is None:
                continue
            w, wd = self._gerceklesen, self._istenen
            if onceki_w is not None and w * onceki_w < 0 and abs(w) > 0.05:
                gecis += 1
            onceki_w = w
            if abs(wd) > UYARIM_ESIK:
                ornek.append((wd, w))
            time.sleep(0.02)

        if len(ornek) < ASGARI_ORNEK:
            return None, f"UYARIM YETERSİZ ({len(ornek)}<{ASGARI_ORNEK} örnek) — " \
                         "görev bu pencerede dönüş komutu vermedi"

        oranlar = [w / wd for wd, w in ornek if abs(wd) > 1e-6]
        med = yuzdelik(oranlar, 50)
        sxy = sum(wd * w for wd, w in ornek)
        sxx = sum(wd * wd for wd, _ in ornek)
        reg = sxy / sxx if sxx else float("nan")
        salinim = gecis / (2 * self.olcum_sn)
        # puan: 1,0'dan sapma + salınım cezası (0,3 Hz üstü ağır ceza)
        puan = abs(1.0 - reg) + max(0.0, salinim - 0.05) * 4.0
        return {"ff": ff, "n": len(ornek), "medyan": med, "regresyon": reg,
                "salinim": salinim, "puan": puan}, None

    # ── geri yükleme
    def _baglam_gecerli_mi(self):
        """rclpy bağlamı hâlâ ayakta mı — servis çağırmadan ÖNCE sorulur."""
        try:
            return rclpy.ok()
        except Exception:
            return False

    def _taze_baglamla_geri_yaz(self):
        """🔴 SON ÇARE: rclpy kapandıysa YENİ bağlam açıp geri yaz.

        NEDEN VAR (17.08.2026'da CANLI ÖLÇÜLDÜ — `ayar_20260817_001622.log`):
        `KillSignal=SIGINT` (ya da Ctrl-C) geldiğinde rclpy'nin KENDİ sinyal
        işleyicisi bağlamı bizim `except` bloğumuz koşmadan ÖNCE geçersiz
        kılıyor. Sonuç: geri yükleme üç denemesinin DOKUZU da
            "failed to check service availability: rcl node's context is invalid"
        ile düştü ve araç yalnız günlüğe *"ELLE YAZ"* yazabildi.

        O koşuda zararsızdı (0/4 adımında iptal, hiç aday yazılmamıştı) ama
        süpürme ORTASINDA aynı şey olsaydı FC **son denenen aday değerinde**
        (örn. FF=0,80) kalırdı — üstelik sahada SSH yok, o satırı okuyacak
        kimse de yok. Yani "acil durumda geri al" yolunun kendisi acil
        durumda çalışmıyordu.

        Bu yüzden geri yükleme, bağlam ölmüşse SIFIRDAN kurulur.
        """
        import rclpy.context
        ctx = rclpy.context.Context()
        rclpy.init(context=ctx)
        try:
            dugum = Node("otomatik_ff_ayar_geri", context=ctx)
            ist = dugum.create_client(
                SetParameters, f"{PARAM_DUGUM}/set_parameters")
            if not ist.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("taze bağlamda da param servisi yok")
            for ad, d in self._ozgun.items():
                p = Parameter(name=ad, value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(d)))
                gel = ist.call_async(SetParameters.Request(parameters=[p]))
                rclpy.spin_until_future_complete(dugum, gel, timeout_sec=10.0)
                s = gel.result()
                ok = bool(s and s.results and s.results[0].successful)
                self._bas("GERİ", f"{'✅' if ok else '🔴'} (taze bağlam) "
                                  f"{ad} = {d:.3f}")
            dugum.destroy_node()
        finally:
            try:
                rclpy.shutdown(context=ctx)
            except Exception:
                pass

    def geri_yukle(self):
        if self._geri_yuklendi or not self._ozgun:
            return
        self._geri_yuklendi = True

        if self.kuru:
            for ad, d in self._ozgun.items():
                self._bas("GERİ", f"(KURU) {ad} = {d:.3f}")
            self._bas("GERİ", f"özgün değerler dosyada: {self._yedek_yolu}")
            return

        # 1. yol: mevcut bağlam hâlâ geçerliyse normal yazma
        kalan = dict(self._ozgun)
        if self._baglam_gecerli_mi():
            for ad, d in list(kalan.items()):
                for deneme in range(3):
                    try:
                        self._yaz(ad, d, zorla=True)
                        self._bas("GERİ", f"{ad} = {d:.3f} ✅")
                        kalan.pop(ad, None)
                        break
                    except Exception as e:
                        self._bas("GERİ",
                                  f"🔴 {ad} geri yüklenemedi ({deneme+1}/3): {e}")
                        if not self._baglam_gecerli_mi():
                            self._bas("GERİ", "bağlam ÖLDÜ — taze bağlama "
                                              "geçiliyor")
                            break
                        time.sleep(1.0)
        else:
            self._bas("GERİ", "rclpy bağlamı zaten geçersiz (SIGINT) — "
                              "doğrudan taze bağlam")

        # 2. yol: bağlam öldüyse SIFIRDAN kur
        if kalan:
            try:
                self._taze_baglamla_geri_yaz()
                kalan.clear()
            except Exception as e:
                self._bas("GERİ", f"🔴 taze bağlam da başarısız: {e}")

        for ad, d in kalan.items():
            self._bas("GERİ", f"🔴🔴 {ad} GERİ YÜKLENEMEDİ — ELLE YAZ: {d}")
        if kalan:
            self._bas("GERİ", "ELLE:  ros2 param set /mavros/param <AD> <DEĞER>"
                              "   (ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1)")
        self._bas("GERİ", f"özgün değerler dosyada: {self._yedek_yolu}")

    # ── bekleme kipi (servis olarak açılışta koşarken)
    def _kosullari_bekle(self):
        """ARMED + GUIDED olana kadar SABIRLA bekle. Hiçbir şey yazmadan.

        🔑 Servis kipinde şart: açılışta tekne DAİMA disarm'dır. `_iptal_mi`
        disarm'ı iptal sebebi sayar — o kural ÖLÇÜM SIRASINDA doğru (yarıda
        kalan ayar geri alınmalı) ama BAŞLAMADAN ÖNCE yanlış olurdu; servis
        her açılışta anında ölürdü.

        Burada hiçbir parametreye dokunulmamıştır ⇒ beklemenin riski yok.
        """
        son_bildirim = -BEKLEME_BILDIRIM_SN
        t0 = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)
            gecen = time.monotonic() - t0
            hazir = (self._baglı and self._armed and self._olculur_mu()
                     and (self._rc10 is None or self._rc10 <= 1500))
            if hazir:
                self._bas("ADIM", f"koşullar sağlandı ({gecen:.0f} sn beklendi): "
                                  "ARMED + GUIDED — ayar başlıyor")
                return
            if gecen - son_bildirim >= BEKLEME_BILDIRIM_SN:
                son_bildirim = gecen
                eksik = []
                if not self._baglı:
                    eksik.append("FC bağlantısı")
                if not self._armed:
                    eksik.append("ARM")
                if not self._olculur_mu():
                    eksik.append(f"GUIDED (şu an {self._mod})")
                if self._rc10 is not None and self._rc10 > 1500:
                    eksik.append(f"acil durdurma KAPALI olmalı (RC10={self._rc10})")
                self._bas("ADIM", f"⏳ bekleniyor ({gecen:.0f} sn) — eksik: "
                                  f"{', '.join(eksik) or '?'}")

    # ── ana akış
    def calistir(self):
        self._bas("ADIM", "0/4 — bağlantı ve özgün değerler")
        t0 = time.monotonic()
        while self._mod is None and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._mod is None:
            raise RuntimeError("/mavros/state gelmiyor — MAVROS/FC bağlı değil")

        for ad in (PARAM_FF, PARAM_P, PARAM_I):
            self._ozgun[ad] = self._oku(ad)
        with open(self._yedek_yolu, "w", encoding="utf-8") as f:
            json.dump(self._ozgun, f, indent=2)
        self._bas("ADIM", f"özgün: FF={self._ozgun[PARAM_FF]:.3f} "
                          f"P={self._ozgun[PARAM_P]:.3f} I={self._ozgun[PARAM_I]:.3f} "
                          f"→ {self._yedek_yolu}")

        if self.bekle:
            # servis kipi: koşullar oluşana kadar sabırla bekle
            self._kosullari_bekle()
        else:
            iptal, sebep = self._iptal_mi()
            if iptal:
                self._bas("İPTAL", f"başlangıç koşulu sağlanmıyor: {sebep}")
                self._bas("İPTAL", "tekne ARMED olmalı; ölçüm yalnız GUIDED'da "
                                   "yapılır ve görev dönüş komutu veriyor olmalı "
                                   "(pasif uyarım — araç komut VERMEZ). "
                                   "Servis kipi için: --bekle")
                return
            if not self._olculur_mu():
                self._bas("ADIM", f"mod {self._mod} — GUIDED'a geçilmesi bekleniyor "
                                  "(ölçüm yalnız GUIDED'da işler)")

        # ═══════════════════════════════════════════════════════════════
        # SERPİŞTİRMELİ ÇOKLU TUR (A-B-A-B) — 17.08 düzeltmesi
        #
        # 🔴 NEDEN (paralel oturum ölçtü, kabul edildi): ölçütün GÜRÜLTÜ
        # TABANI, adaylar arası beklenen farktan büyük olabilir. FF SABİT
        # 0,20 iken, aynı oturumda, ardışık 75 sn'lik 8 pencerede regresyon
        # eğimi: medyan 0,782 · min 0,471 · maks 0,863 ⇒ YAYILIM 0,392.
        # Yani hiçbir şey değişmezken bile ölçüt 0,47-0,86 arası geziniyor.
        #
        # Tek pencereyle "en iyi" ilan etmek YAZI-TURA olurdu. Üstelik eski
        # sürüm tabanı (0,20) koşunun BAŞINDA bir kez ölçüp sonraki adaylarla
        # FARKLI koşullarda (rüzgâr, manevra, duba yoğunluğu) kıyaslıyordu.
        #
        # ÇÖZÜM: adaylar TUR TUR, sırayla dolaşılır (A,B,C,D · A,B,C,D …).
        # Böylece yavaş değişen koşullar bütün adaylara EŞİT dağılır.
        # Her adayın kendi yayılımı ölçülür ve KARAR KURALI şudur:
        #     en iyi ile ikincinin farkı < gürültü tabanı  ⇒  İLAN ETME
        # ═══════════════════════════════════════════════════════════════
        olcumler = {ff: [] for ff in self.adaylar}
        toplam = self.tur * len(self.adaylar)
        adim = 0
        for tur in range(1, self.tur + 1):
            for ff in self.adaylar:
                adim += 1
                p = round(P_ORANI * ff, 3)
                self._bas("ADIM", f"{adim}/{toplam} (tur {tur}/{self.tur}) — "
                                  f"FF={ff:.2f} P=I={p:.3f} "
                                  f"({self.olcum_sn:.0f} sn)")
                self._yaz(PARAM_FF, ff)
                self._yaz(PARAM_P, p)
                self._yaz(PARAM_I, p)
                self._bekle(OTURMA_SN)
                r, hata = self._olc(ff)
                if r is None:
                    self._bas("ÖLÇÜM", f"FF={ff:.2f} tur{tur} ⚠️ {hata}")
                    continue
                olcumler[ff].append(r)
                self._bas("ÖLÇÜM",
                          f"FF={ff:.2f} tur{tur} | n={r['n']:4d} | "
                          f"takip {r['regresyon']:.2f} | "
                          f"salınım {r['salinim']:.2f} Hz | puan {r['puan']:.3f}")

        # ── aday başına özet + YAYILIM
        sonuclar = []
        self._bas("SONUÇ", "─" * 62)
        self._bas("SONUÇ", "aday      tur  puan(medyan)   YAYILIM   takip(medyan)")
        for ff in self.adaylar:
            rs = olcumler[ff]
            if not rs:
                self._bas("SONUÇ", f"FF={ff:.2f}    0   — ölçülemedi")
                continue
            puanlar = sorted(r["puan"] for r in rs)
            reg = sorted(r["regresyon"] for r in rs)
            yayilim = puanlar[-1] - puanlar[0] if len(puanlar) > 1 else float("nan")
            ozet = {"ff": ff, "tur": len(rs),
                    "puan": yuzdelik(puanlar, 50),
                    "yayilim": yayilim,
                    "regresyon": yuzdelik(reg, 50),
                    "salinim": yuzdelik(sorted(r["salinim"] for r in rs), 50),
                    "n": sum(r["n"] for r in rs)}
            sonuclar.append(ozet)
            yz = "—" if len(rs) < 2 else f"{yayilim:.3f}"
            self._bas("SONUÇ", f"FF={ff:.2f}   {len(rs):2d}   {ozet['puan']:>9.3f}   "
                               f"{yz:>7}   {ozet['regresyon']:>9.2f}")

        if not sonuclar:
            self._bas("İPTAL", "hiçbir aday ölçülemedi — görev yeterli dönüş "
                               "komutu vermedi. Tekne kapı takibi yaparken "
                               "tekrar dene.")
            self.geri_yukle()
            return

        sirali = sorted(sonuclar, key=lambda r: r["puan"])
        en_iyi = sirali[0]

        # ═══ GÜRÜLTÜ KAPISI — asıl karar burada ═══════════════════════
        # Gürültü tabanı = adayların KENDİ İÇİNDEKİ yayılımlarının en
        # büyüğü. Aynı FF'te aynı koşulda bile ölçüt bu kadar oynuyorsa,
        # adaylar arası bundan küçük bir fark BİLGİ TAŞIMAZ.
        #
        # Neden maksimum (ortalama değil): tek bir adayda büyük yayılım
        # görülmesi, ölçüm koşullarının o kadar oynayabildiğini gösterir —
        # ve o oynama sıralamayı da çevirebilir. Kötümser taraf güvenli.
        yayilimlar = [r["yayilim"] for r in sonuclar
                      if r["tur"] >= 2 and math.isfinite(r["yayilim"])]
        if yayilimlar:
            gurultu = max(yayilimlar)
            fark = (sirali[1]["puan"] - en_iyi["puan"]) if len(sirali) > 1 \
                else float("inf")
            self._bas("SONUÇ", "─" * 62)
            self._bas("SONUÇ", f"gürültü tabanı (en büyük aday-içi yayılım): "
                               f"{gurultu:.3f}")
            self._bas("SONUÇ", f"en iyi ↔ ikinci farkı              : {fark:.3f}")
            if fark < gurultu:
                self._bas("SONUÇ", "🔴 FARK GÜRÜLTÜDEN KÜÇÜK — SONUÇ İLAN "
                                   "EDİLMİYOR.")
                self._bas("SONUÇ", f"   FF={en_iyi['ff']:.2f} 'en iyi' göründü "
                                   f"ama bu sıralama tekrarlanabilir DEĞİL.")
                self._bas("SONUÇ", "   Özgün değerlere dönülüyor. Daha uzun "
                                   "pencere (--sure) ya da daha çok tur "
                                   "(--tur) ile tekrar dene; ya da elle "
                                   "kademeli ayarla (0,20→0,40→0,60).")
                self.geri_yukle()
                return
            self._bas("SONUÇ", "✅ fark gürültü tabanının ÜSTÜNDE — sonuç "
                               "kullanılabilir")
        else:
            self._bas("SONUÇ", "⚠️ TEK TUR ölçüldü (--tur 1) — yayılım "
                               "bilinmiyor, gürültü kapısı UYGULANAMADI. "
                               "Sonuca temkinli bak.")

        if en_iyi["salinim"] > 0.30:
            self._bas("SONUÇ", "🔴 EN İYİ ADAYDA BİLE SALINIM VAR — "
                               "özgün değerlere dönülüyor, elle bak")
            self.geri_yukle()
            return

        p_iyi = round(P_ORANI * en_iyi["ff"], 3)
        self._bas("SONUÇ", f"SEÇİLEN: FF={en_iyi['ff']:.2f} P=I={p_iyi:.3f}")
        self._yaz(PARAM_FF, en_iyi["ff"])
        self._yaz(PARAM_P, p_iyi)
        self._yaz(PARAM_I, p_iyi)
        self._geri_yuklendi = True          # bilerek bırakıyoruz
        self._bas("SONUÇ", f"yazıldı. Özgün değerler: {self._yedek_yolu}")
        self._bas("SONUÇ", "🔴 FC'de KALICI olması için parametreler yazıldı; "
                           "yeniden başlatmadan önce bir koşumla DOĞRULA.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kuru", action="store_true",
                    help="hiçbir şey YAZMA, yalnız ölç (ÖNCE BUNU KOŞ)")
    ap.add_argument("--geri-yukle", metavar="DOSYA",
                    help="kaydedilmiş özgün değerleri geri yaz (acil durum)")
    ap.add_argument("--adaylar", default=",".join(str(x) for x in ADAYLAR),
                    help=f"virgüllü FF adayları (varsayılan: {ADAYLAR})")
    ap.add_argument("--tur", type=int, default=2,
                    help="her adaydan kaç tur ölçülsün (A-B-A-B serpiştirme). "
                         "2'nin altında GÜRÜLTÜ KAPISI uygulanamaz — "
                         "varsayılan 2")
    ap.add_argument("--bekle", action="store_true",
                    help="SERVİS KİPİ: ARMED+GUIDED olana kadar bekle; ayar "
                         "yarıda kalırsa geri yükleyip yeniden beklemeye dön")
    ap.add_argument("--sure", type=float, default=OLCUM_SN,
                    help=f"aday başına ölçüm saniyesi (varsayılan {OLCUM_SN:.0f})")
    a = ap.parse_args()

    rclpy.init()
    dugum = FFAyar(a.kuru, [float(x) for x in a.adaylar.split(",")],
                   a.sure, bekle=a.bekle, tur=a.tur)

    def _sinyal(_s, _f):
        raise KeyboardInterrupt("SIGTERM")
    signal.signal(signal.SIGTERM, _sinyal)

    try:
        if a.geri_yukle:
            with open(a.geri_yukle, encoding="utf-8") as f:
                dugum._ozgun = json.load(f)
            dugum._geri_yuklendi = False
            dugum.geri_yukle()
        else:
            dugum.calistir()
    except KeyboardInterrupt as e:
        dugum._bas("İPTAL", f"{e or 'Ctrl-C'} — özgün değerlere dönülüyor")
        dugum.geri_yukle()
        if a.bekle and str(e) not in ("SIGTERM", ""):
            # 🔑 Servis kipinde ayar yarıda kaldıysa (disarm / acil durdurma /
            #    bağlantı kopması) SÜREÇ ÖLMEZ: özgün değerler geri yüklendi,
            #    koşullar yeniden oluşunca baştan dener. Sahada tekne bir kez
            #    disarm olunca ayarın bir daha hiç koşmaması istenmiyor.
            dugum._bas("ADIM", "🔁 servis kipi — koşullar yeniden bekleniyor")
            dugum._geri_yuklendi = False
            dugum._ozgun = {}
            try:
                dugum.calistir()
            except KeyboardInterrupt as e2:
                dugum._bas("İPTAL", f"{e2} — geri yükleniyor")
                dugum.geri_yukle()
    except Exception as e:
        dugum._bas("İPTAL", f"🔴 {type(e).__name__}: {e} — geri yükleniyor")
        dugum.geri_yukle()
    finally:
        try:
            dugum.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

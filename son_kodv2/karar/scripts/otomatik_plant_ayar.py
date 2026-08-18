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
    9. 🔑 KONTROL GRUPLU ÖLÇÜM: ivme örnekleri YALNIZ tekne ARMED iken
       sayılır; DISARM iken toplananlar ayrı kovada **gürültü tabanı**
       olur (disarm tekne itki üretemez ⇒ o ivme fiziği değil kestirim
       gürültüsüdür). Ölçüm, kendi gürültü tabanının GURULTU_ORANI katını
       aşmıyorsa YAZILMAZ. 17.08'de bu kapı yoktu ve araç yerinde duran
       teknede 3,42 m/s² "ölçtü" (gerçeği 0,28 — 12 kat).

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

# ── 🔴 17.08 REBOOT KOŞUMUNDA ÖLÇÜLDÜ — ARAÇ GÜRÜLTÜYÜ İVME SANDI ─────────
# Boot koşumu 4.301 "ivme" örneği topladı ve **%95'i 3,42 m/s²** çıkardı
# (tepe 12,22). Havuzun 4 oturumluk ölçülmüş değeri **0,28 m/s²** — yani
# bulunan sayı teknenin gerçek sınırının **12 katı**. Tekne o sırada
# karadaydı: `eğri örneği 0` (hiç arm olmadı, hiç kımıldamadı) ve EKF3
# `is using GPS` ancak **10:55:49**'da, ölçüm bittikten **13 dakika sonra**
# geldi. Toplanan şey hızın kendisi değil, EKF'in yerinde duran tekneyi
# metrelerce gezdiren poz gürültüsünün türeviydi.
#
# 🔑 KUSUR ŞUYDU: `_on_hiz` ivme örneğini KOŞULSUZ topluyordu — eğri örneği
#    `armed`+`MANUAL`+hareket şartına bağlıyken ivmenin hiçbir şartı yoktu.
#    Aracın "tekne kımıldıyor mu?" diye soran hiçbir kapısı yoktu.
#
# ⚠️ ASIL TEHLİKE SESSİZ OLANDI: yazmayı durduran tek şey `IVME_UST=3,00`
#    sert sınırıydı ve onu **tesadüfen** yakaladı. Gürültü %95'i 3,42 yerine
#    2,5 çıksaydı araç `ATC_ACCEL_MAX=2,5`'i FC'ye **KALICI** (EEPROM)
#    yazacak, "geri okundu ✅" basacak ve başarıyla çıkacaktı — gerçek
#    sınırın 9 katı bir değerle. Sert sınır bir kapı değil, son çareydi.
#
# ÇÖZÜM — KONTROL GRUPLU ÖLÇÜM (§1.41b'nin `--bant` deseninin fizik hâli):
#   Disarm hâlindeki tekne **itki üretemez** ⇒ o sırada ölçülen her ivme
#   TANIMI GEREĞİ gürültüdür. Araç bunu ayrı kovada biriktirir ve kendi
#   gürültü tabanını ölçer. Gerçek ölçüm bu tabanı belirgin aşmıyorsa
#   sayı ilan EDİLMEZ. Eşiği sahadan tahmin etmeye gerek yok — kontrol
#   grubu her koşumda kendini yeniden ölçer.
#   Bu kapı donanım şartı GEREKTİRMEZ: disarm tekne her koşumda vardır.
GURULTU_ORANI = 3.0        # gerçek ölçüm, gürültü tabanının bu katını aşmalı
ASGARI_GURULTU = 300       # gürültü tabanını ilan etmek için asgari örnek

# Servis kipi: ayar YAZILANA kadar yaşa (kardeş araçla aynı desen).
PARAM_BEKLEME_AZAMI = 90.0   # elle koşumda: bu kadar bekle, sonra sebebi söyle
YENIDEN_DENE_SN = 30.0       # servis kipinde iki deneme arası
GUNLUK_DIZIN = os.path.expanduser("~/girdap_logs/plant_ayar")

# ── YÖNLENDİRİLMİŞ BASAMAK TESTİ (--basamak) ────────────────────────────────
# 🔑 NEDEN VAR: gaz→hız eğrisi GUIDED'da ölçülemez. Ölçüldü (17.08):
#   · kapalı çevrim LS  → MANUAL'den %37 ve %129 sapıyor (YANLI)
#   · IV yöntemi        → %68 ve %107 sapıyor (DAHA KÖTÜ)
#   IV'nin araç değişkeni GEÇERSİZ: rüzgâr tekneyi yavaşlatınca planlayıcı
#   komutu değiştiriyor ⇒ referans bozucuyla İLİŞKİLİ hâle geliyor. Otonomi
#   döngüsünün İÇİNDEKİ hiçbir sinyal geçerli araç değişken değil.
#   Uyarım eksikliği DEĞİL (ölçüldü: MANUAL 0,72 ↔ GUIDED 0,62 ln-std, benzer).
#   ⇒ Geçerli yol: AÇIK ÇEVRİM (MANUAL) veri. Bu kip onu düzenli toplatır.
#
# ⛔ ARAÇ TEKNEYE KOMUT VERMEZ — kaptana NE YAPACAĞINI söyler, veriyi
#    doğrular. Aktüatör insan, ölçüm ve karar araçta.
BASAMAKLAR = (0.20, 0.40, 0.60, 0.80)   # hedef gaz seviyeleri
BASAMAK_TOL = 0.10                       # ±%10 bandında sayılır
BASAMAK_ORNEK = 40                       # kova başına asgari kararlı örnek
BASAMAK_BILDIRIM_SN = 8.0

# ── ÖLÇÜLMÜŞ BAŞLANGIÇ DEĞERLERİ (17.08, 4 oturum havuzu) ──────────────────
# 🔑 NEDEN VAR: ölçüm yetersiz kalırsa GUIDED'a BİLİNEN-KÖTÜ değerle
# girmeyelim. Bunlar tahmin DEĞİL — 11.218 kararlı-hâl örneği (eğri) ve
# 295.782 ivme örneği (4 oturum, 15-16.08) üzerinden hesaplandı:
#
#   gaz %20 → 0,47 m/s   ·   %40 → 0,71   ·   %60 → 0,86   ·   %80 → 0,78
#   birleşik eğri: v = 0,87 · u^0,41
#   üs dört oturumda da tutarlı: 0,41 · 0,47 · 0,38 · 0,45
#   ivme %95: 0,28 m/s²   ·   fren %95: 0,28 m/s²
#
# ⚠️ İVME SINIRINI DÜŞÜRMENİN YAN ETKİSİ (ArduPilot dokümanı):
#   *"Azami ivmeler düşürülürse araç KÖŞELERİ DAHA ÇOK KESER"* — bağlayıcı
#   kısıt `min(ATC_ACCEL_MAX, ATC_DECEL_MAX, ATC_TURN_MAX_G×9,81)`.
#   Bizde TURN_MAX_G=0,6 ⇒ 5,9 m/s², yani ivme bağlayıcı olan.
#   Kapıya yaklaşırken köşe kesmek DUBAYA ÇARPMAK olabilir (Ç2 cezası).
#   Karşı argüman: tekne gerçekten 0,28 yapabiliyor; 1,0 yazmak planlayıcının
#   YAPILAMAYACAK manevra planlaması demek (aşımın kaynağı). Ölçüme sadık
#   kalınıyor ama ilk koşumda kapı yaklaşması GÖZLE İZLENMELİ.
#
# ⚠️ `ATC_DECEL_MAX = 0` EKSİK DEĞİL, GEÇERLİ VARSAYILAN: "ivme değerini
#   yavaşlama için de kullan" demek. Ölçüm ivme≈fren (0,28↔0,28) olduğunu
#   gösterdiği için SIFIR BIRAKMAK DOĞRU — doküman da öyle diyor.
#   ⇒ Bu araç DECEL'i yalnız ivmeden BELİRGİN ayrıldığında yazar.
BASLANGIC = {P_ACCEL: 0.30}          # ölçülen 0,28 → temkinli yukarı yuvarlama
DECEL_AYRIM_ESIGI = 0.25             # fren ivmeden %25+ farklıysa DECEL yazılır


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
    def __init__(self, kuru, bekle, sure, basamak=False):
        super().__init__("otomatik_plant_ayar")
        self.kuru, self.bekle, self.sure = kuru, bekle, sure
        self.basamak = basamak
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
        # 🔑 "İş bitti mi?" sorusunun TEK doğru cevabı. `_geri_yuklendi` bunu
        # cevaplayamaz (hem "yazdım, bıraktım" hem "geri yükledim" hâlinde
        # True olur). Yedek değer yazmak da BU bayrağı kaldırmaz — yedek
        # "ölçtüm" demek değil, "bilinen-kötüye düşme" demektir.
        self._ayar_yazildi = False
        self._egri = []          # (u, v) — YALNIZ MANUAL
        self._ivme = []          # a (m/s²) — YALNIZ ARMED (itki üretilebilir)
        self._gurultu = []       # a (m/s²) — YALNIZ DISARM = kontrol grubu

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
                # 🔑 KONTROL GRUPLU AYIRIM (bkz. GURULTU_ORANI'nın başındaki
                # not). Disarm tekne itki ÜRETEMEZ ⇒ o sırada ölçülen ivme
                # teknenin fiziği değil, hız kestiriminin gürültüsüdür.
                # İkisini aynı kovaya atmak 17.08'de 12 kat şişmiş bir
                # "fiziksel sınır" üretti.
                (self._ivme if self._armed else self._gurultu).append(a)
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
    def _oku(self, ad, ts=20.0, hosgor=False):
        """FC parametresini oku. `hosgor=True` iken "henüz gelmedi" → None.

        🔴 Ayrım 17.08 reboot'unda ölçüldü: MAVROS ayağa kalkar kalkmaz
        `get_parameters` servisi VARDIR ama FC'den parametreleri henüz
        indirmemiştir; okuma `PARAMETER_NOT_SET` (tip 0) döner. O koşumda
        `ATC_DECEL_MAX` bu yüzden *"FC'de var mı?"* denip **koşumun tamamı
        boyunca atlandı** — oysa parametre FC'de duruyordu, sadece 1 saniye
        erken sorulmuştu.
        """
        if not self._get.wait_for_service(timeout_sec=ts):
            if hosgor:
                return None
            raise RuntimeError(f"{PARAM_DUGUM}/get_parameters yok — "
                               "ROS_LOCALHOST_ONLY servisle aynı mı?")
        g = self._get.call_async(GetParameters.Request(names=[ad]))
        rclpy.spin_until_future_complete(self, g, timeout_sec=ts)
        s = g.result()
        if s is None or not s.values:
            if hosgor:
                return None
            raise RuntimeError(f"{ad} OKUNAMADI")
        v = s.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return float(v.double_value)
        if v.type == ParameterType.PARAMETER_INTEGER:
            return float(v.integer_value)
        if hosgor:
            return None
        raise RuntimeError(f"{ad} beklenmedik tip ({v.type}) — FC'de var mı?")

    def _ozgunleri_oku(self, zorunlu, istege_bagli=()):
        """Özgün değerleri FC'den inene kadar SABIRLA oku.

        `zorunlu` hepsi gelene kadar beklenir (servis kipinde süresiz —
        beklemenin riski YOK, bu noktada hiçbir parametreye dokunulmamıştır).
        `istege_bagli` yalnız zorunlular geldiği ANDA elde varsa alınır:
        o noktada FC parametre indirmesi bitmiştir, hâlâ boş dönen parametre
        gerçekten FC'de yoktur.
        """
        deger = {}
        son_bildirim = -BEKLEME_BILDIRIM_SN
        t0 = time.monotonic()
        while True:
            for ad in zorunlu:
                if ad not in deger:
                    v = self._oku(ad, hosgor=True)
                    if v is not None:
                        deger[ad] = v
            if len(deger) == len(zorunlu):
                gecen = time.monotonic() - t0
                if gecen > 2.0:
                    self._bas("ADIM", f"FC parametreleri geldi ({gecen:.0f} sn "
                                      "beklendi — MAVROS indirme gecikmesi)")
                for ad in istege_bagli:
                    v = self._oku(ad, hosgor=True)
                    if v is not None:
                        deger[ad] = v
                    else:
                        self._bas("ADIM", f"⚠️ {ad} FC'de YOK (zorunlular "
                                          "geldiği hâlde boş döndü) — atlanıyor")
                return deger
            gecen = time.monotonic() - t0
            if not self.bekle and gecen > PARAM_BEKLEME_AZAMI:
                eksik = [a for a in zorunlu if a not in deger]
                raise RuntimeError(
                    f"{', '.join(eksik)} {gecen:.0f} sn'de FC'den gelmedi. "
                    "MAVROS bağlı mı (`ros2 topic echo /mavros/state`)? "
                    "Servis kipinde (--bekle) bu bir hata değildir, beklenir.")
            if gecen - son_bildirim >= BEKLEME_BILDIRIM_SN:
                son_bildirim = gecen
                eksik = [a for a in zorunlu if a not in deger]
                self._bas("ADIM", f"⏳ FC parametreleri bekleniyor ({gecen:.0f} sn)"
                                  f" — eksik: {', '.join(eksik)}")
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.8)

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

    # ── kontrol grubu (gürültü tabanı)
    def _gurultu_tabani(self):
        """Disarm hâlinde ölçülen ivmenin %95'i — yani ölçüm zemininin gürültüsü.

        Örnek yetmezse `None` (taban ilan edilemez, kapı uygulanamaz).
        """
        m = [abs(x) for x in self._gurultu]
        if len(m) < ASGARI_GURULTU:
            return None
        return yuzdelik(m, 95)

    def _gurultu_kapisi(self, olculen, taban):
        """Ölçülen ivme, kendi gürültü tabanından ayırt edilebiliyor mu?

        🔑 Bu kapı bir EŞİK AYARI DEĞİL, bir KONTROL GRUBUDUR: taban her
        koşumda o günün donanımı/ortamıyla yeniden ölçülür. Kapalı alanda
        (GPS fix yok) taban tavana vurur ve araç *"ölçemem"* der — 17.08'de
        olması gereken buydu.

        `(geçti_mi, sebep)` döner.
        """
        if taban is None:
            # Kapı UYGULANAMADI. Bu bir geçiş değil — sessizce geçmiş
            # sayılmasın diye ayrıca raporlanır.
            self._bas("ÖLÇÜM", f"⚠️ gürültü tabanı ilan edilemedi (disarm "
                               f"örneği {len(self._gurultu)} < {ASGARI_GURULTU}) "
                               "— kontrol grubu YOK, ölçüm sert sınırlara emanet")
            return True, ""
        oran = olculen / max(taban, 1e-9)
        self._bas("ÖLÇÜM", f"kontrol grubu: disarm gürültü %95 {taban:.2f} m/s² "
                           f"({len(self._gurultu)} örnek) ↔ ölçülen {olculen:.2f} "
                           f"⇒ oran {oran:.2f} (gereken ≥{GURULTU_ORANI:.1f})")
        if oran < GURULTU_ORANI:
            return False, (
                f"ölçülen {olculen:.2f} m/s², gürültü tabanının yalnız "
                f"{oran:.2f} katı. Tekne ya hiç hareket etmedi ya hız kaynağı "
                "sağlıksız (kapalı alan / GPS fix yok / EKF hazır değil). "
                "Bu sayı teknenin fiziği DEĞİL, kestirim gürültüsüdür.")
        return True, ""

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
                                   f"ivme örneği {len(self._ivme)} (ARMED) · "
                                   f"gürültü {len(self._gurultu)} (disarm) · "
                                   f"mod {self._mod} "
                                   f"{'ARMED' if self._armed else 'DISARM'}")

    def _basamak_topla(self):
        """Kaptanı yönlendirerek kova kova veri topla. KOMUT VERMEZ."""
        kova = {b: 0 for b in BASAMAKLAR}
        son_bildirim = 0.0
        son_sayilan = len(self._egri)
        t0 = time.monotonic()
        self._bas("BASAMAK", "🔑 MANUAL moda geç. Araç sana ne yapacağını "
                             "söyleyecek; TEKNEYE KOMUT VERMEZ.")
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            iptal, sebep = self._iptal_mi()
            if iptal:
                raise KeyboardInterrupt(sebep)
            # yeni gelen eğri örneklerini kovalara dağıt
            for u, _v in self._egri[son_sayilan:]:
                for b in BASAMAKLAR:
                    if abs(u - b) <= BASAMAK_TOL:
                        kova[b] += 1
                        break
            son_sayilan = len(self._egri)
            eksik = [b for b in BASAMAKLAR if kova[b] < BASAMAK_ORNEK]
            if not eksik:
                self._bas("BASAMAK", "✅ DÖRT KOVA DA DOLDU — test bitti, "
                                     "gazı serbest bırakabilirsin")
                return True
            gecen = time.monotonic() - t0
            if gecen - son_bildirim >= BASAMAK_BILDIRIM_SN:
                son_bildirim = gecen
                if self._mod != "MANUAL":
                    self._bas("BASAMAK", f"⏸ mod {self._mod} — bu test MANUAL "
                                         f"gerektirir (açık çevrim)")
                    continue
                hedef = eksik[0]
                durum = " · ".join(f"%{int(100*b)}:{kova[b]}/{BASAMAK_ORNEK}"
                                   for b in BASAMAKLAR)
                self._bas("BASAMAK", f"👉 şimdi ~%{int(100*hedef)} gaz ver ve "
                                     f"SABİT TUT   [{durum}]")
            if gecen > self.sure:
                self._bas("BASAMAK", f"⏱ süre doldu — dolan kova: "
                                     f"{sum(1 for b in BASAMAKLAR if kova[b]>=BASAMAK_ORNEK)}"
                                     f"/{len(BASAMAKLAR)}")
                return sum(1 for b in BASAMAKLAR if kova[b] >= BASAMAK_ORNEK) >= 3

    def calistir(self):
        self._bas("ADIM", "0/3 — bağlantı ve özgün değerler")
        t0 = time.monotonic()
        while self._mod is None and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._mod is None:
            raise RuntimeError("/mavros/state gelmiyor — MAVROS/FC bağlı değil")
        self._ozgun.update(self._ozgunleri_oku((P_EXPO, P_ACCEL),
                                               istege_bagli=(P_DECEL,)))
        with open(self._yedek, "w", encoding="utf-8") as f:
            json.dump(self._ozgun, f, indent=2)
        self._bas("ADIM", "özgün: " + " · ".join(
            f"{k}={v:.3f}" for k, v in self._ozgun.items()) + f" → {self._yedek}")

        if self.basamak:
            self._bas("ADIM", "1/3 — YÖNLENDİRİLMİŞ BASAMAK TESTİ "
                              "(araç komut VERMEZ, sana söyler)")
            self._basamak_topla()
        else:
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

        # ── ② ivme sınırları (ÖNCE kontrol grubu, SONRA ölçüm)
        taban = self._gurultu_tabani()
        if taban is not None:
            # 🔑 Ölçüm atlansa bile BAS: bu sayı "bu ortamda ölçüm yapılabilir
            # mi?" sorusunun cevabı. Havuzun gerçek değeri 0,28 m/s² — taban
            # ona yakın ya da üstündeyse ortam (kapalı alan / GPS fix yok)
            # bu ölçümü fiziken taşımıyor demektir, tekne arm edilse bile.
            self._bas("ÖLÇÜM", f"gürültü tabanı (disarm, {len(self._gurultu)} "
                               f"örnek): %95 {taban:.2f} m/s² · "
                               f"ölçüm için gereken en az "
                               f"{GURULTU_ORANI * taban:.2f} m/s²"
                               + ("  🔴 havuzun ölçtüğü 0,28'in üstünde — bu "
                                  "ortamda ivme sınırı ÖLÇÜLEMEZ (hız kaynağı "
                                  "sağlıksız: kapalı alan / GPS fix yok / EKF "
                                  "hazır değil)" if GURULTU_ORANI * taban > 0.28
                                  else ""))
        poz = [x for x in self._ivme if x > 0]
        neg = [-x for x in self._ivme if x < 0]
        if len(poz) < ASGARI_IVME or len(neg) < ASGARI_IVME:
            self._bas("ÖLÇÜM", f"⚠️ ivme sınırları ATLANDI — ARMED örnek az "
                               f"(+{len(poz)} / −{len(neg)}, gereken {ASGARI_IVME})"
                               + (f". Disarm gürültü kovasında {len(self._gurultu)} "
                                  "örnek var: tekne hiç arm olmadı, ölçülecek "
                                  "itki YOK." if len(self._gurultu) > len(poz) else ""))
        else:
            ai, af = yuzdelik(poz, 95), yuzdelik(neg, 95)
            self._bas("ÖLÇÜM", f"ivme %95 {ai:.2f} (maks {max(poz):.2f}) · "
                               f"fren %95 {af:.2f} (maks {max(neg):.2f}) m/s²")
            gecti, sebep = self._gurultu_kapisi(ai, taban)
            if not gecti:
                self._bas("ÖLÇÜM", f"🔴 {P_ACCEL} YAZILMIYOR — {sebep}")
            elif P_ACCEL not in self._ozgun:
                self._bas("ÖLÇÜM", f"🔴 {P_ACCEL} yazılamadı — FC'de okunamadı "
                                   "(parametre yok ya da MAVROS indirmedi)")
            elif not (IVME_ALT <= ai <= IVME_UST):
                self._bas("ÖLÇÜM", f"🔴 {P_ACCEL}={ai:.2f} SERT SINIR DIŞI "
                                   f"[{IVME_ALT} … {IVME_UST}] — yazılmıyor. "
                                   "Bu sınır bir kapı değil SON ÇAREDİR; "
                                   "buraya düşülüyorsa ölçüm zaten şüpheli.")
            else:
                yazilacak[P_ACCEL] = round(ai, 2)
            # DECEL yalnız ivmeden BELİRGİN ayrılırsa yazılır (0 = "ivmeyi kullan")
            ayrim = abs(af - ai) / max(ai, 1e-9)
            if ayrim < DECEL_AYRIM_ESIGI:
                self._bas("ÖLÇÜM", f"✅ fren ≈ ivme (fark %{100*ayrim:.0f}) ⇒ "
                                   f"{P_DECEL} SIFIR bırakılıyor (ArduPilot: "
                                   "sıfır = 'ivme değerini kullan')")
            elif gecti and P_DECEL in self._ozgun and IVME_ALT <= af <= IVME_UST:
                yazilacak[P_DECEL] = round(af, 2)
                self._bas("ÖLÇÜM", f"fren ivmeden %{100*ayrim:.0f} ayrılıyor ⇒ "
                                   f"{P_DECEL} ayrıca yazılıyor")

        # ── ③ yaz
        self._yazma_asamasi(yazilacak)

    def _yazma_asamasi(self, yazilacak):
        """Ölçülenleri yaz, eksikleri yedeğe düşür, 'iş bitti mi'ye karar ver.

        🔑 Ayrı metot çünkü bu aşamanın kararı sınanabilir olmalı: hangi
        yazmanın servisi bitirdiği (`_ayar_yazildi`) 17.08 kusurunun tam
        kalbi. Ölçüm toplama ROS gerektirir, bu karar gerektirmez.
        """
        self._bas("ADIM", "3/3 — yazma")
        olculdu = set(yazilacak)          # 🔑 gerçekten ÖLÇÜLENLER
        # Ölçüm yetersizse ÖLÇÜLMÜŞ BAŞLANGIÇ değerine düş (bilinen-kötüden iyi).
        # ⚠️ Bu bir SONUÇ DEĞİL, bir emniyet tabanıdır: `_ayar_yazildi`
        # bayrağını KALDIRMAZ, yani servis kipi bunu "iş bitti" saymaz ve
        # gerçek ölçüm için beklemeye devam eder. Yüklü değer zaten yedeğe
        # eşitse hiç yazılmaz (her turda aynı değeri tekrar yazmaz).
        for ad, v in BASLANGIC.items():
            if ad in yazilacak or ad not in self._ozgun:
                continue
            if abs(self._ozgun[ad] - v) < 0.02:
                continue
            yazilacak[ad] = v
            self._bas("ÖLÇÜM", f"⚠️ {ad} bu koşumda ölçülemedi — 17.08 havuzunun "
                               f"ÖLÇÜLMÜŞ değeri yazılıyor: {v:.2f} "
                               f"(yüklü {self._ozgun[ad]:.2f}). "
                               "Tahmin değil: 295.782 ivme örneği, 4 oturum. "
                               "İŞ BİTMEDİ — gerçek ölçüm hâlâ bekleniyor.")
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
        self._ayar_yazildi = bool(olculdu)  # yalnız GERÇEK ölçüm işi bitirir
        self._bas("SONUÇ", "yazıldı: " + " · ".join(
            f"{k}={v}" + ("" if k in olculdu else " (yedek — ölçüm değil)")
            for k, v in yazilacak.items()))
        self._bas("SONUÇ", f"özgün değerler: {self._yedek}")
        self._bas("SONUÇ", "🔴 KALICI yazıldı (ArduPilot PARAM_SET → EEPROM). "
                           "Bir sonraki koşumda DOĞRULA: alt uçtaki hız aşımı "
                           "3× seviyesinden düştü mü?")


def _servis_dongusu(dugum):
    """SERVİS KİPİ: GERÇEK ölçüm yazılana kadar yaşamaya devam et.

    🔴 NEDEN (17.08 reboot koşumunda ölçüldü): eski akış tek 600 sn'lik
    pencere koşuyor, ölçemese bile **0 ile çıkıyordu**. `Restart=on-failure`
    olan servis, çıkış başarılı göründüğü için onu diriltmedi:

        10:42:51  Deactivated successfully.  (Consumed 20.475s CPU time)

    Yani tekne suya girdiğinde plant aracı **ortada olmayacaktı** ve
    MOT_THST_EXPO için gereken gaz→hız eğrisi hiç ölçülmeyecekti — üstelik
    `systemctl is-enabled` hâlâ "enabled" diyor, kimse fark etmiyor.

    ⚠️ Bu araç için beklemek ZORUNLU, çünkü ölçüm koşulu insana bağlı:
    eğri YALNIZ kaptan MANUAL'de değişken gazla sürerken oluşur (açık
    çevrim). Boot anında o koşul tanımı gereği yoktur.

    Döngüyü YALNIZ iki şey bitirir: gerçek ölçümün yazılması ve SIGTERM.
    Yedek değere düşmek bitirmez (`_ayar_yazildi` kalkmaz) — "bilinen-kötüye
    düşmedim" ile "ölçtüm" aynı şey değildir.
    """
    dene = 0
    while True:
        dene += 1
        try:
            dugum.calistir()
            if dugum._ayar_yazildi:
                dugum._bas("SONUÇ", f"✅ ölçüm yazıldı ({dene}. denemede) — "
                                    "servis çıkıyor")
                return
            dugum._bas("ADIM", "ölçüm sonuç vermedi — koşullar yeniden beklenecek")
        except KeyboardInterrupt as e:
            dugum._bas("İPTAL", f"{e or 'Ctrl-C'} — özgün değerlere dönülüyor")
            dugum.geri_yukle()
            if str(e) in ("SIGTERM", ""):
                return                      # systemd durduruyor: itiraz etme
        except Exception as e:
            dugum._bas("İPTAL", f"🔴 {type(e).__name__}: {e} — geri yükleniyor")
            dugum.geri_yukle()
        dugum._bas("ADIM", f"🔁 servis kipi — {YENIDEN_DENE_SN:.0f} sn sonra "
                           f"{dene + 1}. deneme")
        try:
            time.sleep(YENIDEN_DENE_SN)
        except KeyboardInterrupt:
            return
        # Sonraki deneme temiz başlasın.
        dugum._geri_yuklendi = False
        dugum._ozgun = {}
        dugum._egri = []
        dugum._ivme = []
        dugum._gurultu = []


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kuru", action="store_true",
                    help="hiçbir şey YAZMA, yalnız ölç (ÖNCE BUNU KOŞ)")
    ap.add_argument("--bekle", action="store_true",
                    help="SERVİS KİPİ: FC bağlanana kadar bekle")
    ap.add_argument("--sure", type=float, default=OLCUM_SN,
                    help=f"pasif toplama saniyesi (varsayılan {OLCUM_SN:.0f})")
    ap.add_argument("--basamak", action="store_true",
                    help="YÖNLENDİRİLMİŞ BASAMAK TESTİ: kaptana %%20/%%40/%%60/%%80 "
                         "gaz sırasını söyler, kovalar dolunca biter (~10 dk). "
                         "Araç tekneye KOMUT VERMEZ. En temiz eğri verisi.")
    ap.add_argument("--geri-yukle", metavar="DOSYA",
                    help="kaydedilmiş özgün değerleri geri yaz (acil)")
    a = ap.parse_args()

    rclpy.init()
    d = PlantAyar(a.kuru, a.bekle, a.sure, basamak=a.basamak)
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
                _servis_dongusu(d)          # ölçene kadar yaşa
            else:
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

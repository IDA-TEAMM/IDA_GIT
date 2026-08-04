#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GİRDAP İDA — Duba Geçiş Navigatörü v3 (TEKNOFEST 2026)
=======================================================
YOLOv11n, OAK-D Lite'ın İÇİNDE (Myriad X VPU) çalışır. Bu node Jetson'da
koşar: tespitlerden geçit seçer, geçit sayar ve seçilen MOD'a göre çıkış
üretir. Algı + görev mantığı iki modda da AYNI, sadece çıkış katmanı değişir.

v5 değişiklikleri (2026 şartnamesi birinci kaynaktan doğrulandı):
  - Dosya-1 kaydedici: bbox+sınıf overlay'li, HER KARESİ zaman etiketli mp4
    (şartname 4.2: ≥1 Hz zorunlu çıktı; teslim edilmeyen dosya 5 ceza puanı,
    md. 5.5.4.3.5). 2 dk'lık çökme-dayanımlı segmentler, tüm modlarda aktif.
  - MIN_GECIT 1→2: Parkur-2 tamamlama şartı "EN AZ 2 duba ikilisi arasından
    geçiş + son görev noktası" (md. 5.5.2.4). Puan geçiş oranıyla arttığından
    (G2/KD2×40) şart sonrası da geçmeye devam edilir (GOREVDE_DUR=False).
  - GEÇİT TANIMI DÜZELTİLDİ: puanlanan geçit kenar×engel DEĞİL, karşılıklı
    KENAR×KENAR (turuncu) çiftidir (puanlama G/KD tanımı). Sarı engel yalnız
    kaçınılır. Eski kenar×engel eşleşmesi ENGEL_YEDEK=False bayrağına taşındı.

v4 değişiklikleri (girdap-decision entegrasyonu):
  - YENİ MOD "algi_yayin": karar stack'i arkadaşın girdap-decision reposu
    çıktı (Nav2 DEĞİL, kendi RRT*+MPPI'sı). Bu mod onun perception
    sözleşmesini besler; navigasyon kararı tamamen ona geçer.
  - Geçiş doğrulama poz kaynağı bu modda TF değil /girdap/fusion/odom
    (girdap-decision stack'i TF yayınlamıyor, pozu topic'ten veriyor).

v3 değişiklikleri (DepthAI 3.7.1 = Temmuz 2026 itibarıyla en güncel sürüm):
  - Plan A'da geçiş sayacı artık ZAMANLA DEĞİL ODOMETRİYLE doğrulanır
    (MPPI'nın hızı bizden bağımsız; zaman varsayımı yanlış sayabiliyordu)
  - Plan A arama hedefi son görülen tarafa hafif açılı (FOV taraması)
  - AF varyant OAK-D Lite için opsiyonel sabit fokus (titreşimde AF avlanır)

ÜÇ MOD:
  MOD = "algi_yayin"  ..... PLAN A (takım mimarisi — girdap-decision sözleşmesi)
      Sürüş/karar TAMAMEN arkadaşın girdap-decision stack'inde. Bu node onun
      perception_camera_node mock'unun GERÇEK OAK karşılığıdır; yayınlar:
        /perception/buoys       vision_msgs/Detection2DArray (bbox piksel uzayı)
        /perception/gate_passed std_msgs/Bool (odometriyle DOĞRULANMIŞ geçiş
                                → fsm_node'un parkur geçiş kanalı)
        /perception/buoys_3d    geometry_msgs/PoseArray (BONUS: stereo 3D duba
                                konumu; obstacle_map şemasıyla aynı hack)
      Hiçbir hedef/hız komutu BASMAZ. GEREKSİNİM: /girdap/fusion/odom yayında
      olmalı + ros-humble-vision-msgs kurulu.

  MOD = "mppi_hedef"  ..... ESKİ Plan A (Nav2 varsayımı — ARŞİV/YEDEK)
      Nav2 bt_navigator'a /goal_pose basar. Takımın gerçek karar mimarisi
      Nav2 değil girdap-decision çıktı; bu mod ancak Nav2 kurulursa anlamlı.
      GEREKSİNİM: Nav2 açık + TF (odom->base_link) + cmd_vel->MAVROS köprüsü.

  MOD = "dogrudan_surus" .. PLAN B (SAHA YEDEĞİ)
      Bu node dümeni kendisi tutar, MAVROS'a TwistStamped hız basar.
      Nav2 / TF / odometri GEREKMEZ. Sadece MAVROS + GUIDED + ARM yeter.
      Ana zincir sahada tökezlerse buna geçilir.

!!! TEK DÜMEN KURALI !!!
  - algi_yayin ve mppi_hedef modlarında bu node ASLA hız komutu basmaz.
  - dogrudan_surus modundayken girdap-decision planning_node / Nav2 KAPALI
    olmalı (iki kaynak çakışır).

PLAN B hızlı başlangıç:
  ros2 param set /mavros setpoint_velocity.mav_frame BODY_NED
  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
"""

import math
import os
import time
from dataclasses import dataclass

import depthai as dai
import rclpy

try:                                  # paket içi saf mantık (kamerasız testli)
    from girdap_ida_algi import gecit_mantik as gm
except ImportError:                   # dosya doğrudan çalıştırılırsa
    import gecit_mantik as gm
from rclpy.node import Node
from rclpy.time import Time as RclTime
from geometry_msgs.msg import TwistStamped, PoseStamped

# ================== MOD SEÇİMİ ==================
MOD = "algi_yayin"   # "algi_yayin" (Plan A) | "mppi_hedef" (arşiv) | "dogrudan_surus" (Plan B)

if MOD == "mppi_hedef":
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401 - PoseStamped TF dönüşümünü kaydeder
    from rclpy.duration import Duration
elif MOD == "algi_yayin":
    from geometry_msgs.msg import Pose, PoseArray
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Int32, String
    from vision_msgs.msg import (            # sudo apt install ros-humble-vision-msgs
        Detection2D, Detection2DArray, ObjectHypothesisWithPose)

# ================== AYARLAR ==================
# ---- Model: YOLOv11n @ 416x416 ----
# HubAI'den çevrilen NN Archive (RVC2, 6 shave, IR 2022.3.0). 416x416 giriş
# boyutu arşivin İÇİNDE tanımlı, kodda ayrıca ayarlanmaz — model değişirse
# boyut arşivle birlikte gelir. FPS bandı bu boyutla ölçüldü.
MODEL_NNARCHIVE = "/home/girdap/models/yolo11n_duba_rvc2.tar.xz"

# Sınıf indeksleri — YALNIZCA YEDEK. Gerçek indeksler çalışma anında NN
# Archive'ın sınıf İSİMLERİNDEN çözülür (_sinif_indeksleri_coz).
# Sabit indekse güvenmek tehlikeli: elimizdeki eğitilmiş model
# Gazebonew.pt'nin sırası {0: "Engel Dubasi", 1: "Kenar Dubasi"} — yani
# aşağıdaki sabitlerin TERSİ. Ters sırada geçit tespiti iki SARI engeli
# kenar çifti sanar ve Parkur-2 sessizce çöker.
KENAR_CLASS = 0        # turuncu duba (RAL 2003) - parkur kenarı  [yedek]
ENGEL_CLASS = 1        # sarı duba   (RAL 1026) - engel           [yedek]
CONF_ESIK = 0.5
FPS = 12               # sensorFps üst sınırı. Saha ölçümü: 10-14 FPS bandı, tipik
                       # ~11.6 (YOLO 416x416 + stereo birlikte = VPU sınırı).
                       # 14'e çıkarmak kâğıt üstünde cazip ama VPU zaten doymuş:
                       # kuyrukta bayat kare birikir, gecikme artar. 12'de bırak.
FPS_UYARI_ESIK = 8.0   # ölçülen NN FPS bunun altına düşerse logda uyar

# OAK-D Lite RGB'si iki varyant: AF (otofokus) | FF (sabit odak). AF varyantı
# dalga/titreşimde odak avlar (Luxonis titreşimli araçta sabitlemeyi önerir).
# None = dokunma. Değer 0(uzak)..255(yakın); 2-15 m duba bandı için düşük
# değerler dene ve NETLİĞİ MASA TESTİNDE DOĞRULA (duba_kamera_test.py).
RGB_SABIT_FOKUS = None    # örn. 60 — FF varyantta None bırak

# ---- ARAÇ ÖLÇÜLERİ ----
# Ekibin ölçüm formundan (son_kodv2/karar/docs/olcum_formu.md, 2026-08-04):
# tekne boyu ÖLÇÜLDÜ = 1,03 m. Genişlik 0,78 m ekibin gate_follower'ında
# "ölçülmüş" olarak geçiyor (kaynak doküman bu makinede yok → ⚠️ TEYİT İSTE).
# Geniş olanı almak GÜVENLİ taraf: geçilebilirlik testini katılaştırır.
ARAC_EN = 0.78             # m - katamaran TOPLAM genişliği
ARAC_BOY = 1.03            # m - toplam uzunluk (ölçüldü)
KAMERA_KIC_MESAFE = ARAC_BOY  # kamera BURUN hizasında varsayıldı; daha gerideyse azalt
EMNIYET_PAYI = 0.30        # m - geçitte gövdenin her iki yanında istenen boşluk
DUBA_CAP = 0.30            # m - şartname duba çapı (X/Z duba MERKEZİNİ verir)
# Kamerayı orta hatta (merkez eksene), suya ~paralel bakacak şekilde monte et.

# ---- mppi_hedef (PLAN A) ayarları ----
GOAL_TOPIC = "/goal_pose"      # bt_navigator'ın dinlediği hedef topic'i
HEDEF_FRAME = "odom"           # hedefin yayınlanacağı sabit çerçeve ("map" de olabilir)
BASE_FRAME = "base_link"
KAMERA_OFSET_ILERI = 0.50      # m - kameranın base_link ORİJİNİNE göre ileri konumu (ÖLÇ!)
HEDEF_OTELEME = KAMERA_KIC_MESAFE + 1.0  # m - hedef, geçit ortasının bu kadar ÖTESİNE
GOAL_GUNCELLE_MESAFE = 0.5     # m - hedef bundan az kaydıysa yeniden yayınlama
GOAL_GUNCELLE_SN = 2.0         # s - hedef en sık bu aralıkla yayınlanır
ARAMA_ILERI = 3.0              # m - geçit yokken arama hedefi mesafesi
ARAMA_HEDEF_SN = 4.0           # s - arama hedefi yayın periyodu
ARAMA_YAW = 0.30               # rad - arama hedefi son görülen geçit tarafına bu kadar açılı
GECIS_ZAMAN_KATSAYI = 3.0      # geçiş zaman aşımı = tahmini sürenin bu katı (odom esas ölçüt)

# ---- algi_yayin (PLAN A) ayarları ----
# girdap-decision sözleşmesi (perception_camera_node ile birebir aynı şema).
BUOYS_TOPIC = "/perception/buoys"        # Detection2DArray — fusion node dinler
GATE_TOPIC = "/perception/gate_passed"   # Bool — ⚠️ aşağıdaki uyarıyı OKU
GATE_COUNT_TOPIC = "/perception/gate_count"    # Int32 — geçilen FARKLI geçit sayısı
GATE_TARGET_TOPIC = "/perception/gate_target"  # PoseStamped — geçidin ötesindeki hedef
MISSION_STATE_TOPIC = "/girdap/mission/state"  # String — yeniden başlamayı yakalamak için

# 🔴 SÖZLEŞME UYARISI (2026-08-04 bulundu, senaryo avı):
# `fsm_node._on_gate_passed` gelen HERHANGİ bir True'yu `last_gate_passed_p2`
# yapıyor ve `mission_fsm` bunu görünce PARKUR2 → PARKUR3 (kamikaze) geçiyor
# ("son duba ikilisi geçildi"). Yani bizim her geçitte bastığımız sinyal,
# Parkur-2'yi İLK geçitte bitirip tekneyi koridorun ortasında kamikaze moduna
# sokuyor: P2 tamamlanmaz (md 5.5.2.4 en az 2 ikili + son görev noktası),
# (G2/KD2)×40 puanı gider, ödül sıralaması (en az P1+P2) kaybedilir.
#
# Algı hangi geçidin SONUNCU olduğunu BİLEMEZ (KD çalışma anında bilinmez;
# şartname "duba sayılarına göre akış tasarlanmaması" diyor). Bu yüzden
# taşıyamayacağımız anlamı taşıyan sinyali VARSAYILAN OLARAK BASMIYORUZ;
# onun yerine dürüst olanı yayınlıyoruz: geçilen FARKLI geçit SAYISI.
# P2→P3 geçişi karar tarafında waypoint ilerlemesinden sürülmeli (şartname
# P2 bitişini "son görev noktasına ulaşmak" diye tanımlıyor).
# FSM düzeltilince bu bayrak True yapılabilir.
GATE_PASSED_YAYINLA = False
BUOYS3D_TOPIC = "/perception/buoys_3d"   # PoseArray — BONUS: stereo 3D konum,
                                         # obstacle_map şemasıyla aynı hack
                                         # (position.{x,y}=merkez, orientation.z=yarıçap).
                                         # Karar tarafı isterse MPPI'ya doğrudan besler.
ODOM_TOPIC = "/girdap/fusion/odom"       # karar stack'inin pürüzsüz poz çıkışı
                                         # (o stack TF yayınlamaz — geçiş doğrulaması
                                         # bu modda TF yerine buradan beslenir)
KAMERA_FRAME = "oak_rgb"

# Sınıf eşlemesi — girdap-decision class_id STRING sözleşmesi:
# "0"=parkur_kenarı (turuncu), "1"=engel (sarı). Asıl eşleme çalışma anında
# kurulan self.sinif_esleme'dir; bu sabit yalnız sınıf isimleri okunamazsa
# devreye girer.
SINIF_ESLEME = {KENAR_CLASS: "0", ENGEL_CLASS: "1"}

# bbox piksel uzayı: fusion_node camera_image_width_px=640 varsayıyor — AYNI kal.
IMG_W, IMG_H = 640, 480
# LETTERBOX dikey düzeltme: 4:3 kare (640x480) kare NN girişine (416x416)
# sığdırılırken üst/alt şerit eklenir. YATAY normalizasyon değişmez (bearing
# füzyonu zaten yalnız yatayı kullanır); dikey için şerit payı çıkarılır.
# NOT: DepthAI v3'ün bbox'ı NN çerçevesinde mi orijinalde mi normalize verdiğini
# masa testinde duba_kamera_test.py ile DOĞRULA; ters çıkarsa _LB_PAY = 0.0 yap.
_LB_ICERIK = IMG_H / IMG_W               # 0.75 — kare çerçevede içerik oranı
_LB_PAY = (1.0 - _LB_ICERIK) / 2.0       # 0.125 — üst şerit (normalize)

# ---- dogrudan_surus (PLAN B) ayarları ----
CRUISE_HIZ = 1.0       # m/s - görev hızı
SEARCH_HIZ = 0.3
MIN_HIZ = 0.4          # sert dönüşte bile dümen suyu için alt sınır
KP_YAW = 1.2           # rad/s başına rad hata (P kontrolcü)
MAX_YAW = 0.8
SEARCH_YAW = 0.3
YAW_ISARET = 1.0       # SAHADA TERS DÖNERSE -1.0 YAP

# Engelden kaçınma (SADECE Plan B'de; Plan A'da MPPI costmap'ten kaçınır)
ENGEL_KACIN_Z = 4.0
ENGEL_KORIDOR = ARAC_EN / 2 + 0.4  # m - yarım gövde + pay = çarpışma koridoru
K_KACIN = 0.6

# ---- Geçit geometrisi (iki modda ortak) ----
# Merkezden merkeze min genişlik = araç eni + 2 yan pay + duba çapı
# ❌ KALDIRILDI (2026-08-04): `GECIT_MIN_GEN` (= gövde + 2×emniyet + çap),
#    `GECIT_MAX_GEN = 10 m`, `GECIT_MAX_DZ = 4 m`. Üçü de metre cinsinden
#    TAHMİNDİ; şartname "kenar dubaları arasındaki mesafeler yarışma alanına
#    göre değişkenlik gösterecektir" (s.20) ve "duba sayılarına göre akış
#    tasarlanmaması" (s.23) diyor. Yerlerine ölçek-bağımsız ölçütler geçti:
#    `gm.gecilebilir_mi` (fizik: gövde sığıyor mu) + `gm.yan_yana_mi` (45°
#    geometrik ayrım). Emniyet payı artık algı filtresi DEĞİL — dar ama gerçek
#    bir kapıyı görünmez kılıp puan kaybettiriyordu; pay sürüş katmanının işi.
# 🔴 MENZİL TAVANI (2026-08-04 araştırmasıyla 15.0 → 8.0 İNDİRİLDİ):
#   (a) Piksel sınırı: HFOV 69°, NN 416 px → 30 cm duba 15 m'de ~6 px, 10 m'de
#       ~9 px, 5 m'de ~18 px. 6 pikselli nesne YOLO için pratikte yok.
#   (b) Stereo sınırı: OAK-D Lite baseline 7,5 cm; 480P'de 3-5 px disparite
#       6,6-11 m'ye denk → ~8 m ötesinde Z hatası METRELERCE (Luxonis doc).
#   İki bağımsız sınır aynı yere çıkıyor: güvenilir geçit tespiti ~6-8 m.
#   Sahada ölçülünce (gerçek duba, gerçek model) bu sayı GÜNCELLENECEK.
GECIT_MAX_MESAFE = 8.0
# Stereo Z ile pinhole (genişlikten) menzil arasındaki kabul edilen bağıl fark.
# Aşılırsa ölçümler çelişiyor demektir → çift güvenilmez, atılır.
MENZIL_BAGIL_TOL = 0.35
# Aynı geçidin tekrar sayılmaması için orta noktalar arası min ayrım (m).
# Şartname G tanımı "FARKLI karşılıklı kenar dubaları arasından geçiş sayısı".
GECIT_AYIRT_M = 3.0
# Geçit = KARŞILIKLI İKİ KENAR (TURUNCU) DUBASI. Şartname puanlama tablosu
# G/KD tanımı: "Farklı Karşılıklı KENAR Dubaları Arasından Geçiş Sayısı" —
# sarı engel dubası geçidin parçası DEĞİLDİR, yalnızca kaçınılır (Ç cezası
# kenar+engel tüm çarpmaları sayar).
ENGEL_YEDEK = False    # saha yedeği: hiç kenar çifti yoksa kenar+engel çiftine
                       # düş (sınıflandırma hatası telafisi; puan GARANTİ ETMEZ,
                       # sadece parkurda ilerlemeyi sürdürür)

# ---- Geçiş tetikleme / sayaç (iki modda ortak) ----
PASS_TETIK_Z = 2.0     # orta nokta bu kadar yaklaşınca "geçiş" fazına gir
PASS_KAYIP_Z = 3.2     # geçit bu mesafeden yakınken FOV'dan çıkarsa da geç say
PASS_EK_YOL = KAMERA_KIC_MESAFE + 0.5  # kıçın da geçidi temizlemesi için ek yol

# ---- Görev ----
# Şartname 2026 md. 5.5.2.4 Parkur-2 tamamlama şartı: "En az 2 adet duba
# İKİLİSİ arasından geçmek (+ son görev noktasına ulaşmak)". Puan geçiş
# ORANIYLA artar — (G2/KD2)×40 — bu yüzden GOREVDE_DUR=False bilinçli:
# şart sağlandıktan sonra da geçit geçmeye devam edilir.
MIN_GECIT = 2
GOREVDE_DUR = False    # True: şart sağlanınca yeni hedef/hız üretme

# ---- Dosya-1 kamera kaydı (şartname 4.2 — ZORUNLU çıktı, tüm modlarda) ----
# "İşlenmiş kamera verisi: en az 1 Hz, her bir frame zaman etiketli, mp4;
#  tespit obje çerçeveleri ve sınıf bilgileri görünecek." Karaya alımdan
# itibaren 20 dk içinde USB ile teslim; her eksik dosya 5 ceza puanı
# (md. 5.5.4.3.5). Kayıt hatası görevi ASLA durdurmaz.
KAYIT_AKTIF = True
KAYIT_HZ = 2.0            # şart ≥1 Hz; 2 Hz pay bırakır (VPU'ya ek yük yok,
                          # passthrough zaten üretilen kare — sadece USB kopyası)
KAYIT_SEGMENT_SN = 120.0  # çökme dayanımı: 2 dk'lık mp4 segmentleri (yarıda
                          # kesilirse en fazla son segment zarar görür)
KAYIT_DIZIN = os.path.expanduser("~/girdap_logs/kamera")

if KAYIT_AKTIF:
    import cv2            # overlay çizimi + mp4 yazımı (yalnız kayıt aktifken)

KONTROL_HZ = 15.0
HEDEF_KAYIP_SN = 1.0   # s - tespit tazeliği; 10-14 FPS bandında 10+ kareye denk
ARAMA_TIMEOUT_SN = 25.0
# =============================================


@dataclass
class Duba:
    cls: int
    x: float   # m, kamera çerçevesi: sağ +
    z: float   # m, ileri +
    conf: float
    # bbox, NN çerçevesinde normalize [0..1] (algi_yayin modunda kullanılır)
    cx: float = 0.0
    cy: float = 0.0
    w: float = 0.0
    h: float = 0.0


def pipeline_kur():
    """OAK-D Lite üzerinde çalışan pipeline: RGB + Stereo + YOLO (VPU'da)."""
    nn_archive = dai.NNArchive(MODEL_NNARCHIVE)

    pipeline = dai.Pipeline()

    # depthai 3.7.1'e karşı doğrulandı — Tem 2026 itibarıyla EN GÜNCEL sürüm
    # (resmi SpatialDetectionNetwork örneğiyle aynı desen)
    cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=FPS)
    if RGB_SABIT_FOKUS is not None:
        cam_rgb.initialControl.setManualFocus(RGB_SABIT_FOKUS)
    mono_sol = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    mono_sag = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    # ROBOTICS preseti (Luxonis dokümanı, 2026-08-04 araştırması): "navigasyon,
    # engel tespiti", çalışma aralığı **0-10 m**; DEFAULT 0-15 m'ye yayılır.
    # Bizim geçit bandımız ≤8 m (piksel + baseline hesabı) → menzili dar tutmak
    # aynı VPU bütçesinde daha temiz derinlik verir. İkisi de HIGH_DENSITY +
    # 7x7 medyan + 3-bit subpixel; fark aralık ayarında.
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # resmi örnekte yok (build hallediyor), açıkça yazmak zararsız

    # 🔴 DÜZELTİLDİ (2026-08-04): eskiden (640,400) isteniyordu — mono sensör
    # 640x480 olduğu için bu istek üstten/alttan KIRPIYOR (requestOutput
    # varsayılanı CROP), dikey FOV ~%17 daralıyordu. "Ufka bakan kamerada sorun
    # değil" gerekçesi YANLIŞTI: geçide 2 m kala duba karenin ALT bölgesinde
    # görünür — kırpılan bölge tam orası. Derinlik gelmezse tespit `z<=0.05`
    # filtresine takılıp DÜŞER, yani geçidi tam tetikleme anında kaybederiz.
    # Tam sensör çözünürlüğü isteniyor; FPS etkisi masa testinde ölçülecek.
    mono_sol.requestOutput((640, 480)).link(stereo.left)
    mono_sag.requestOutput((640, 480)).link(stereo.right)

    sdn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
        cam_rgb, stereo, nn_archive,
        resizeMode=dai.ImgResizeMode.LETTERBOX  # tam yatay FOV korunur (CROP kenarlardaki dubaları keser)
    )
    sdn.input.setBlocking(False)
    sdn.setBoundingBoxScaleFactor(0.5)
    sdn.setDepthLowerThreshold(300)      # mm
    # 20 m → 10 m (2026-08-04): 8 m ötesinde stereo Z hatası metrelerce
    # (baseline 7,5 cm); uzak "derinlik" çöpü hayalet duba konumu üretir ve
    # geçit çiftini bozar. ROBOTICS presetinin 0-10 m aralığıyla da uyumlu.
    sdn.setDepthUpperThreshold(10000)    # mm
    sdn.setConfidenceThreshold(CONF_ESIK)

    det_q = sdn.out.createOutputQueue(maxSize=4, blocking=False)
    rgb_q = None
    if KAYIT_AKTIF:
        # Dosya-1 için NN giriş karesi (letterbox). VPU'ya EK YÜK YOK — kare
        # zaten üretiliyor, sadece USB'den kopyası çekilir. FPS'e etkisini
        # yine de masa testinde doğrula (duba_kamera_test.py aynı akışı çeker).
        rgb_q = sdn.passthrough.createOutputQueue(maxSize=2, blocking=False)
    siniflar = sdn.getClasses()          # NN Archive'daki sınıf isimleri (data.yaml sırası)
    return pipeline, det_q, rgb_q, siniflar


def _sinif_indeksleri_coz(siniflar):
    """Sınıf isimlerinden (kenar, engel) indekslerini çöz.

    Modelin sınıf sırası eğitimdeki data.yaml'dan gelir ve yeniden eğitimde
    sessizce değişebilir. Sabit indeks kullanmak, turuncu/sarı dubaların yer
    değiştirmesi demektir; hiçbir istisna atılmaz, yalnız Parkur-2 kaybedilir.
    İsimle çözüm bu hatayı imkânsız kılar.

    Dönüş: (kenar_idx, engel_idx, isimle_cozuldu)
    """
    if not siniflar:
        return KENAR_CLASS, ENGEL_CLASS, False
    kenar = engel = None
    for i, ad in enumerate(siniflar):
        adl = str(ad).lower()
        if "kenar" in adl:
            kenar = i
        elif "engel" in adl:
            engel = i
    if kenar is None or engel is None or kenar == engel:
        return KENAR_CLASS, ENGEL_CLASS, False
    return kenar, engel, True


class DubaNavigator(Node):
    def __init__(self):
        super().__init__("duba_gecis_navigator")

        if MOD == "algi_yayin":
            self.buoys_pub = self.create_publisher(Detection2DArray, BUOYS_TOPIC, 10)
            self.gate_pub = self.create_publisher(Bool, GATE_TOPIC, 10)
            self.buoys3d_pub = self.create_publisher(PoseArray, BUOYS3D_TOPIC, 10)
            # Dürüst sinyal: geçilen FARKLI geçit sayısı (parkur bitişi İDDİA ETMEZ)
            self.gate_count_pub = self.create_publisher(Int32, GATE_COUNT_TOPIC, 10)
            # M2: geçide yönlendirecek hedef — karar tarafı waypoint'ler arasında
            # geçit ortasını ara-hedef olarak kullanabilsin diye yayınlanır.
            self.gate_target_pub = self.create_publisher(
                PoseStamped, GATE_TARGET_TOPIC, 10)
            self.odom_sub = self.create_subscription(
                Odometry, ODOM_TOPIC, self.odom_geldi, 10)
            # B6: yeniden başlama hakkı kullanılırsa geçit hafızası sıfırlanmalı
            self.state_sub = self.create_subscription(
                String, MISSION_STATE_TOPIC, self.gorev_durumu_geldi, 10)
            self.son_gorev_durumu = None
            self.son_odom = None           # (x, y, yaw) — geçiş doğrulaması için
        elif MOD == "mppi_hedef":
            self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.son_goal = None           # (x, y) HEDEF_FRAME içinde
            self.son_goal_t = 0.0
            self.son_arama_goal_t = 0.0
        elif MOD == "dogrudan_surus":
            self.cmd_pub = self.create_publisher(
                TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        else:
            raise ValueError(f"Geçersiz MOD: {MOD}")

        self.pipeline, self.det_q, self.rgb_q, siniflar = pipeline_kur()
        self._siniflar = siniflar or []
        # Dosya-1 kayıt durumu (şartname 4.2)
        self._kayit_bozuk = not KAYIT_AKTIF
        self._vw = None            # cv2.VideoWriter (tembel açılır)
        self._vw_t0 = 0.0
        self._seg_no = 0
        self._son_kayit_t = 0.0
        if KAYIT_AKTIF:
            self._kayit_dizin = os.path.join(
                KAYIT_DIZIN, time.strftime("session_%Y%m%d_%H%M%S"))
            try:
                os.makedirs(self._kayit_dizin, exist_ok=True)
            except OSError as e:
                self._kayit_bozuk = True
                self.get_logger().error(f"Dosya-1 dizini açılamadı: {e}")
        self.pipeline.start()
        self.get_logger().info(f"OAK-D Lite hazır — YOLO VPU'da. MOD = {MOD}")
        self.kenar_cls, self.engel_cls, isimle = _sinif_indeksleri_coz(self._siniflar)
        self.sinif_esleme = {self.kenar_cls: "0", self.engel_cls: "1"}
        if isimle:
            self.get_logger().info(
                f"Model sınıf sırası: {self._siniflar} → "
                f"kenar={self.kenar_cls} ('{self._siniflar[self.kenar_cls]}'), "
                f"engel={self.engel_cls} ('{self._siniflar[self.engel_cls]}') "
                "— isimle çözüldü.")
            if (self.kenar_cls, self.engel_cls) != (KENAR_CLASS, ENGEL_CLASS):
                self.get_logger().warn(
                    f"Model sırası yedek sabitlerin TERSİ "
                    f"(yedek: kenar={KENAR_CLASS}, engel={ENGEL_CLASS}). "
                    "İsimle çözülen sıra kullanılıyor — sabitler artık ölü.")
        else:
            self.get_logger().error(
                f"Sınıf isimleri çözülemedi (getClasses={self._siniflar!r}). "
                f"YEDEK sabitlere düşüldü: kenar={KENAR_CLASS}, engel={ENGEL_CLASS}. "
                "SIRA TERSSE TURUNCU/SARI YER DEĞİŞİR — sahaya çıkmadan doğrula!")

        self.dubalar = []
        self.son_tespit_t = 0.0
        self.durum = "ARAMA"
        self.arama_baslangic = time.time()
        self.gecit_sayisi = 0
        self.gorev_tamam = False
        self.son_gecit = None      # (bearing, orta_z, zaman)
        self.son_taraf = 1.0
        self.pass_bitis_t = 0.0
        self.gecit_cizgi = None    # (mx, my, nx, ny) HEDEF_FRAME'de — Plan A geçiş doğrulaması
        self.gecit_yari_gen = None # m — geçidin yarı genişliği (yanal sınır)
        self.son_yari_gen = None   # m — en son GÖRÜLEN geçidin yarı genişliği (B5)
        # Tanılama sayaçları — en tehlikeli arıza SESSİZ RET: turuncu dubalar
        # görünüyor ama hiçbir çift kapı olmuyor, araç ham GPS noktasına gidiyor
        # ve HİÇBİR hata basılmadan puan kaybediyoruz. Sahada SSH yok, bu yüzden
        # sayaçlar journal'a periyodik basılır. (Fikir: ekibin GateDiagnostics'i.)
        self._tani = {"dar": 0, "dizili": 0, "arada_duba": 0, "menzil_celiski": 0}
        # Geçilen geçitlerin orta noktaları (dünya/odom çerçevesi). Şartname G
        # tanımı "FARKLI karşılıklı kenar dubaları arasından geçiş sayısı" →
        # aynı geçitten tekrar geçilirse SAYILMAZ (bkz. gm.yeni_gecit_mi).
        self.gecilen_gecitler = []
        # Pinhole odak (normalize bbox için): D = f·W/w_norm
        self._f_norm = gm.odak_px(1.0)
        # Letterbox payı: başlangıçta sabit yedek, ilk tespit mesajında
        # cihazın kendi dönüşüm bilgisiyle DEĞİŞTİRİLİR (varsayım yok).
        self._lb_pay = _LB_PAY
        # S2: Dosya-1 (md 4.2) "her frame zaman etiketli" olmak zorunda. Jetson'da
        # RTC pili yoksa saat boot'ta geride açılır (ölçüldü: ~2 ay) → etiketler
        # yanlış olur, teslimde 5 ceza riski. Kod saati düzeltemez; SESSİZ KALMASIN.
        if time.localtime().tm_year < 2026:
            self.get_logger().error(
                f"SAAT YANLIŞ görünüyor ({time.strftime('%Y-%m-%d %H:%M')}) — "
                "Dosya-1 zaman etiketleri geçersiz olur (md 4.2). "
                "Koşudan ÖNCE: sudo date -s '...' → sonra bu node'u yeniden başlat.")
        self._son_log = 0.0
        self._fps_n = 0            # ölçülen NN FPS (beklenen bant: 10-14)
        self._fps_t0 = time.time()
        self.olculen_fps = 0.0

        self.timer = self.create_timer(1.0 / KONTROL_HZ, self.dongu)

    # ---------- Algılama (ortak) ----------
    def tespitleri_oku(self):
        msg = None
        while True:
            m = self.det_q.tryGet()
            if m is None:
                break
            msg = m
            self._fps_n += 1
        t = time.time()
        if t - self._fps_t0 >= 5.0:
            self.olculen_fps = self._fps_n / (t - self._fps_t0)
            self._fps_n, self._fps_t0 = 0, t
            if self.olculen_fps < FPS_UYARI_ESIK:
                self.get_logger().warn(
                    f"NN FPS düşük: {self.olculen_fps:.1f} (beklenen bant 10-14) — "
                    "USB bağlantısı / VPU ısınması / kablo kontrol")
        if msg is None:
            return
        dets = []
        for d in msg.detections:
            if d.confidence < CONF_ESIK:
                continue
            x = d.spatialCoordinates.x / 1000.0
            z = d.spatialCoordinates.z / 1000.0
            if z <= 0.05:
                continue
            dets.append(Duba(
                int(d.label), x, z, float(d.confidence),
                cx=(d.xmin + d.xmax) / 2.0, cy=(d.ymin + d.ymax) / 2.0,
                w=(d.xmax - d.xmin), h=(d.ymax - d.ymin)))
        # Letterbox payını CİHAZDAN öğren (varsayım yerine ölçü). Dönüşüm
        # bilgisi yoksa dosya başındaki sabit yedeğe düşülür.
        try:
            tr = msg.getTransformation()
            if tr is not None:
                nn_w, nn_h = tr.getSize()
                src_w, src_h = tr.getSourceSize()
                pay = gm.letterbox_payi(nn_w, nn_h, src_w, src_h)
                if pay is not None and abs(pay - self._lb_pay) > 1e-6:
                    self.get_logger().info(
                        f"LETTERBOX payı cihazdan: {pay:.4f} "
                        f"(NN {nn_w}x{nn_h} ← kaynak {src_w}x{src_h}); "
                        f"önceki {self._lb_pay:.4f}")
                    self._lb_pay = pay
        except Exception as e:      # eski firmware / alan yok → yedek pay
            self.get_logger().warn(f"Dönüşüm bilgisi okunamadı, sabit pay: {e}",
                                   throttle_duration_sec=30.0)
        self.dubalar = dets
        self.son_tespit_t = time.time()
        if MOD == "algi_yayin":
            self.tespit_yayinla()   # taze NN karesi → sözleşme topic'leri

    # ---------- Dosya-1: işlenmiş kamera kaydı (şartname 4.2, s.14) ----------
    def kayit_adimi(self):
        """bbox+sınıf overlay'li, zaman etiketli mp4 karesi yaz (~KAYIT_HZ).

        Şartname: ≥1 Hz, her frame zaman etiketli, tespit çerçeveleri + sınıf
        görünür. Kayıt hatası görevi ASLA durdurmaz — devre dışı kalır, loglanır.
        """
        frame = None
        while True:                      # kuyruğu boşalt, en taze kareyi al
            f = self.rgb_q.tryGet()
            if f is None:
                break
            frame = f
        if frame is None:
            return
        try:
            img = frame.getCvFrame()
            h, w = img.shape[:2]
            for d in self.dubalar:
                # bbox NN çerçevesinde normalize; passthrough karesi de NN
                # çerçevesi -> doğrudan ölçekle (unletterbox GEREKMEZ)
                x1 = int((d.cx - d.w / 2.0) * w)
                y1 = int((d.cy - d.h / 2.0) * h)
                x2 = int((d.cx + d.w / 2.0) * w)
                y2 = int((d.cy + d.h / 2.0) * h)
                renk = (0, 140, 255) if d.cls == self.kenar_cls else (0, 230, 255)
                cv2.rectangle(img, (x1, y1), (x2, y2), renk, 2)
                ad = (self._siniflar[d.cls] if d.cls < len(self._siniflar)
                      else str(d.cls))
                cv2.putText(img, f"{ad} {d.conf:.2f} Z:{d.z:.1f}m",
                            (x1, max(14, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, renk, 1)
            t = time.time()
            etiket = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))
                      + f".{int((t % 1.0) * 1000):03d}")
            cv2.putText(img, etiket, (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(img, f"{self.durum} gecit={self.gecit_sayisi}"
                             f" NN {self.olculen_fps:.1f}FPS",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 0), 1)
            self._kayit_yaz(img, t)
        except Exception as e:
            self._kayit_bozuk = True
            self.get_logger().error(
                f"Dosya-1 kaydı DEVRE DIŞI (görev sürüyor): {e}")

    def _kayit_yaz(self, img, t):
        """Segmentli mp4 yazımı — çökmede en fazla son segment zarar görür."""
        if self._vw is not None and t - self._vw_t0 >= KAYIT_SEGMENT_SN:
            self._vw.release()
            self._vw = None
        if self._vw is None:
            self._seg_no += 1
            yol = os.path.join(self._kayit_dizin, f"seg_{self._seg_no:04d}.mp4")
            self._vw = cv2.VideoWriter(
                yol, cv2.VideoWriter_fourcc(*"mp4v"),
                KAYIT_HZ, (img.shape[1], img.shape[0]))
            if not self._vw.isOpened():
                raise RuntimeError(f"VideoWriter açılamadı: {yol}")
            self._vw_t0 = t
            self.get_logger().info(f"Dosya-1 segmenti: {yol}")
        self._vw.write(img)

    def kayit_kapat(self):
        """Son segmenti düzgün kapat (moov atomu yazılsın) — main finally."""
        if self._vw is not None:
            self._vw.release()
            self._vw = None

    # ---------- PLAN A çıkışı: girdap-decision perception sözleşmesi ----------
    def odom_geldi(self, msg):
        """/girdap/fusion/odom → (x, y, yaw). Karar stack'inin poz kaynağı."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.son_odom = (p.x, p.y, yaw)

    def gorev_durumu_geldi(self, msg):
        """FSM durumu — yeniden başlama (md 5.5.3.1) yakalanır.

        Yeniden başlamada puanlar sıfırlanır ve parkur BAŞTAN koşulur; geçit
        hafızası temizlenmezse aynı geçitler "zaten geçildi" sanılır ve yeni
        turda HİÇBİRİ sayılmaz.
        """
        yeni = msg.data.strip().upper() if msg.data else None
        if gm.sifirlama_gerekir(self.son_gorev_durumu, yeni):
            self.get_logger().warn(
                f"YENİDEN BAŞLAMA algılandı ({self.son_gorev_durumu} → {yeni}): "
                f"geçit hafızası sıfırlanıyor (önceki sayı {self.gecit_sayisi})")
            self.gecilen_gecitler = []
            self.gecit_sayisi = 0
            self.gorev_tamam = False
            self.gecit_cizgi = None
            self.gecit_yari_gen = None
            self.duruma_gec("ARAMA")
            self.gate_count_pub.publish(Int32(data=0))
        self.son_gorev_durumu = yeni

    def gecit_hedefi_publish(self, a, b):
        """M2: geçidin ötesindeki hedefi base_link'te yayınla.

        Şartname (s.23): "görev noktaları doğrudan iki kenar dubasının arasında
        bir nokta olmayabilir" → yalnız GPS waypoint'e sürmek geçitleri
        ıskalatır. Algı geçidi GÖRDÜĞÜ için nereye gidilmesi gerektiğini
        söyleyebilir; kullanıp kullanmamak karar tarafının seçimi (biz dümen
        tutmuyoruz — tek dümen kuralı).
        """
        ox, oy, px, py = self.gecit_geometri(a, b)
        hx, hy = gm.gecit_hedefi(ox, oy, px, py, HEDEF_OTELEME)
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = BASE_FRAME
        p.pose.position.x = float(hx)
        p.pose.position.y = float(hy)
        yaw = math.atan2(py, px)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        self.gate_target_pub.publish(p)

    def tespit_yayinla(self):
        """Taze tespitleri /perception/buoys (2D, sözleşme) + /perception/buoys_3d
        (BONUS stereo 3D) olarak yayınla. Boş liste de yayınlanır — fusion'ın
        zaman senkronu ve 'görüş alanında duba yok' bilgisi için."""
        stamp = self.get_clock().now().to_msg()

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = KAMERA_FRAME
        arr3d = PoseArray()
        arr3d.header.stamp = stamp
        arr3d.header.frame_id = BASE_FRAME

        for d in self.dubalar:
            det = Detection2D()
            det.header = arr.header
            # Yatay: LETTERBOX tam FOV korur, normalize doğrudan ölçeklenir.
            det.bbox.center.position.x = d.cx * IMG_W
            # Dikey: üst/alt şerit payı çıkarılıp 4:3 görüntüye geri açılır.
            icerik = max(1e-6, 1.0 - 2.0 * self._lb_pay)
            cy = min(1.0, max(0.0, (d.cy - self._lb_pay) / icerik))
            det.bbox.center.position.y = cy * IMG_H
            det.bbox.size_x = d.w * IMG_W
            det.bbox.size_y = min(1.0, d.h / icerik) * IMG_H
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.sinif_esleme.get(d.cls, str(d.cls))
            hyp.hypothesis.score = d.conf
            det.results.append(hyp)
            arr.detections.append(det)

            # 3D: kamera -> base_link (ileri x = Z + ofset, sol y = -X)
            p = Pose()
            p.position.x = d.z + KAMERA_OFSET_ILERI
            p.position.y = -d.x
            p.orientation.z = DUBA_CAP / 2.0   # obstacle_map hack'i: z = yarıçap
            p.orientation.w = 1.0
            arr3d.poses.append(p)

        self.buoys_pub.publish(arr)
        self.buoys3d_pub.publish(arr3d)

    # ---------- Geçit seçimi (ortak) ----------
    def gecit_bul(self):
        """Puanlanan geçit = karşılıklı iki KENAR (turuncu) dubası arası.

        Şartname puanlama tablosu (G/KD): 'Farklı Karşılıklı Kenar Dubaları
        Arasından Geçiş Sayısı' — geçit çiftleri YALNIZ kenar×kenar kurulur.
        Sarı engel dubaları geçit değildir; Plan B'de kacinma_bias, Plan A'da
        karar stack'inin engel haritası onlardan kaçınır.
        """
        kenar = sorted((d for d in self.dubalar if d.cls == self.kenar_cls),
                       key=lambda d: d.z)
        engel = [d for d in self.dubalar if d.cls == self.engel_cls]

        # Stereo ile ÇELİŞEN tespitleri en baştan ele: bbox genişliğinden
        # pinhole menzil (D = f·W/w_px) stereo'dan BAĞIMSIZ ikinci ölçümdür.
        # Duba çapı 30 cm sabittir; yüksekliği değildir (şartname: su üstünde
        # kalan yükseklik o anki şartlara bağlı) — bu yüzden GENİŞLİKTEN.
        kenar = [d for d in kenar if self._menzil_saglikli(d)]

        ciftler = [(kenar[i], kenar[j])
                   for i in range(len(kenar)) for j in range(i + 1, len(kenar))]
        if not ciftler and ENGEL_YEDEK:
            # Saha yedeği: turuncu yanlış sınıflanmışsa kör kalmamak için
            ciftler = [(k, e) for k in kenar for e in engel]

        en_iyi, en_iyi_z = None, 1e9
        for a, b in ciftler:
            orta_z = (a.z + b.z) / 2.0
            if orta_z > GECIT_MAX_MESAFE:
                continue
            # (a) GEÇİLEBİLİRLİK — gövde sığmıyorsa kapı değildir (fizik).
            #     Üst sınır YOK: kapı genişliği şartnameye göre alana göre
            #     değişiyor, sabit tavan koymak tahmin olurdu.
            ayrik = math.hypot(a.x - b.x, a.z - b.z)
            if not gm.gecilebilir_mi(ayrik, ARAC_EN, DUBA_CAP):
                self._tani["dar"] += 1
                continue
            # (b) KURSA DİK Mİ — |Δileri| < |Δyanal| (ölçek-bağımsız 45° ayrımı;
            #     eski sabit `GECIT_MAX_DZ=4 m` eşiğinin yerine geçti).
            if not gm.yan_yana_mi(a.z, a.x, b.z, b.x):
                self._tani["dizili"] += 1
                continue
            # A-5: aralarında ÜÇÜNCÜ bir kenar dubası varsa bu "karşılıklı
            # ikili" değildir — koridorun iki ayrı tarafından birer duba
            # seçilmiştir; oraya sürmek parkur dışına çıkmak demek.
            digerleri = [(k.x, k.z) for k in kenar if k is not a and k is not b]
            if gm.arada_duba_var(a.x, a.z, b.x, b.z, digerleri):
                self._tani["arada_duba"] += 1
                continue
            if orta_z < en_iyi_z:
                en_iyi, en_iyi_z = (a, b), orta_z
        return en_iyi

    def _menzil_saglikli(self, d):
        """Stereo Z ile bbox genişliğinden gelen pinhole menzil çelişiyor mu?

        bbox normalize olduğu için ölçek 416/640'tan bağımsız: f_norm=odak_px(1.0).
        Ölçümlerden biri yoksa çelişki iddia edilmez (True döner)."""
        d_mono = gm.mesafe_genislikten(d.w, gm.DUBA_CAP_M, self._f_norm)
        ok = gm.menzil_tutarli(d.z, d_mono, MENZIL_BAGIL_TOL)
        if not ok:
            self.get_logger().warn(
                f"Tespit atıldı — stereo {d.z:.1f} m ↔ genişlikten {d_mono:.1f} m "
                "çelişiyor (uzak/kısmi görünen duba?)",
                throttle_duration_sec=5.0)
        return ok

    # ---------- Durum makinesi (ortak) ----------
    def duruma_gec(self, yeni):
        if yeni != self.durum:
            if yeni == "ARAMA":
                self.arama_baslangic = time.time()
            self.durum = yeni

    def gecis_baslat(self, orta_z, gecit_bl=None, yari_gen=None):
        """GECIS fazına gir. gecit_bl: base_link'te (ox, oy, px, py) geçit çizgisi.

        Plan A'da geçiş ODOMETRİYLE doğrulanır: MPPI'nın gerçek hızı bizden
        bağımsızdır (vx_max, engel yavaşlaması, hedefe yavaşlama), CRUISE_HIZ
        varsayımıyla zaman saymak geçilmemiş geçidi 'geçildi' sayabilir.
        Zaman burada sadece SON ÇARE aşımıdır; TF alınamazsa eski davranışa düşer.
        """
        self.durum = "GECIS"
        self.son_gecit = None
        # İki duba merkezi arası mesafenin yarısı — "dubaların ARASINDAN mı
        # geçti" kontrolünün sınırı. None ise geçit tek bearing'den kuruldu,
        # genişlik bilinmiyor (yalnız düzlem testi yapılır).
        self.gecit_yari_gen = yari_gen
        tahmin = (orta_z + PASS_EK_YOL) / CRUISE_HIZ
        if MOD in ("algi_yayin", "mppi_hedef") and gecit_bl is not None:
            self.gecit_cizgi = self.cizgi_hedef_frame(gecit_bl)
        else:
            self.gecit_cizgi = None
        if self.gecit_cizgi is not None:
            self.pass_bitis_t = time.time() + max(GECIS_ZAMAN_KATSAYI * tahmin, 8.0)
        else:
            self.pass_bitis_t = time.time() + tahmin   # odom yok: eski zaman tahmini
        self.get_logger().info(f">> Geçide giriliyor (orta nokta {orta_z:.1f} m)")

    def dongu(self):
        self.tespitleri_oku()
        simdi = time.time()

        # Dosya-1: görev durumundan BAĞIMSIZ kayıt (şartname ≥1 Hz; görev
        # tamamlansa da karaya alınana dek kayıt sürer)
        if not self._kayit_bozuk and simdi - self._son_kayit_t >= 1.0 / KAYIT_HZ:
            self._son_kayit_t = simdi
            self.kayit_adimi()

        if self.gorev_tamam and GOREVDE_DUR:
            if MOD == "dogrudan_surus":
                self.hiz_yayinla(0.0, 0.0)
            return  # mppi modunda yeni hedef basılmaz, MPPI son hedefte durur

        # --- GECIS fazı ---
        if self.durum == "GECIS":
            if self.gecit_cizgi is not None:
                # Plan A esas ölçütü: araç geçit çizgisini odometride GERÇEKTEN aştı mı?
                gecti = False
                p = self.arac_poz_yaw()
                if p is not None:
                    mx, my, nx, ny = self.gecit_cizgi
                    # Teğet = normalin dikeyi (geçit çizgisi boyunca).
                    # Hem "düzlemi aştı mı" hem "dubaların ARASINDAN mı geçti"
                    # kontrol edilir — yandan dolaşma geçiş SAYILMAZ.
                    gecti = gm.gecitten_gecti(
                        p[0], p[1], mx, my, nx, ny, -ny, nx,
                        self.gecit_yari_gen, PASS_EK_YOL)
                if not gecti:
                    if simdi >= self.pass_bitis_t:
                        self.get_logger().warn(
                            "Geçiş zaman aşımı — odometri geçişi DOĞRULAMADI, sayılmadı. "
                            "(MPPI takılmış olabilir: obstacle_margin / engel haritasına bak)")
                        self.gecit_cizgi = None
                        self.duruma_gec("ARAMA")
                    self.durum_log()
                    return
            elif simdi < self.pass_bitis_t:
                # Plan B (veya TF'siz son çare): zaman tahmini
                if MOD == "dogrudan_surus":
                    self.hiz_yayinla(CRUISE_HIZ, 0.0)
                # mppi modunda hedef zaten geçidin ötesinde: MPPI sürüyor, bekliyoruz
                self.durum_log()
                return
            # A-3: FARKLI geçit mi? (şartname G tanımı) — geçidin orta noktası
            # dünya çerçevesinde tutulur; aynı geçitten tekrar geçilirse
            # sayılmaz ve fsm'e sahte geçiş sinyali GİTMEZ.
            orta_dunya = self.gecit_cizgi[:2] if self.gecit_cizgi else None
            self.gecit_cizgi = None
            if orta_dunya is not None and not gm.yeni_gecit_mi(
                    orta_dunya[0], orta_dunya[1], self.gecilen_gecitler,
                    GECIT_AYIRT_M):
                self.get_logger().info(
                    "Geçit geçildi ama AYNI geçit (daha önce sayılmıştı) — "
                    "'farklı geçiş' sayılmadı (md 5.5.4.2 G tanımı)")
                self.duruma_gec("ARAMA")
                self.durum_log()
                return
            if orta_dunya is not None:
                self.gecilen_gecitler.append(orta_dunya)
            self.gecit_sayisi += 1
            self.get_logger().info(f"### GEÇİT {self.gecit_sayisi} TAMAMLANDI ###")
            if MOD == "algi_yayin":
                # Dürüst sinyal: kaç FARKLI geçit geçildi (parkur bitişi İDDİA ETMEZ)
                self.gate_count_pub.publish(Int32(data=int(self.gecit_sayisi)))
                if GATE_PASSED_YAYINLA:
                    # ⚠️ Yalnız FSM'in P2→P3 geçişi waypoint'e taşındıktan SONRA
                    # açılmalı — yoksa ilk geçit tekneyi kamikazeye sokar.
                    self.gate_pub.publish(Bool(data=True))
            # A-4: bu bayrak YALNIZCA Parkur-2'nin geçit ayağını (≥2 ikili,
            # md 5.5.2.4) temsil eder. Parkur-1'in şartı ORANSAL —
            # (G1/KD1)×10 ≥ 5 ⇒ G1/KD1 ≥ 0,5 (md 5.5.2.3) — ve KD1 (parkurdaki
            # karşılıklı ikili sayısı) çalışma anında BİLİNMİYOR; şartname
            # "duba sayılarına göre akış tasarlanmaması" diyor. Bu yüzden
            # P1 için "tamamlandı" İDDİA EDİLMEZ: geçit geçmeye devam edilir
            # (GOREVDE_DUR=False) ve parkur kararı FSM/görev katmanına aittir.
            if (gm.p2_gecit_sarti(self.gecit_sayisi, MIN_GECIT)
                    and not self.gorev_tamam):
                self.gorev_tamam = True
                self.get_logger().info(
                    "*** P2 geçit ayağı sağlandı: en az 2 FARKLI duba ikilisinden "
                    "geçildi (son görev noktası şartı FSM'de) ***")
            self.duruma_gec("ARAMA")

        taze = (simdi - self.son_tespit_t) < HEDEF_KAYIP_SN
        gecit = self.gecit_bul() if taze else None

        if gecit is not None:
            a, b = gecit
            mx = (a.x + b.x) / 2.0
            mz = (a.z + b.z) / 2.0
            bearing = math.atan2(mx, mz)         # + ise hedef SAĞDA
            self.son_gecit = (bearing, mz, simdi)
            self.son_taraf = 1.0 if mx >= 0 else -1.0
            # B5: geçidin yarı genişliğini SAKLA — geçit FOV'dan çıkarsa yanal
            # kontrol bilgisiz kalmasın (yandan dolaşma orada da sayılmasın).
            self.son_yari_gen = 0.5 * math.hypot(a.x - b.x, a.z - b.z)
            self.duruma_gec("YAKLASMA")
            if MOD == "algi_yayin":
                self.gecit_hedefi_publish(a, b)   # M2: geçide yönlendirme bilgisi

            if mz < PASS_TETIK_Z and abs(bearing) < math.radians(20):
                if MOD == "mppi_hedef":
                    self.gecit_hedefi_yayinla(a, b, zorla=True)  # son bir kez garanti
                # Yarı genişlik = iki duba MERKEZİ arası mesafenin yarısı;
                # aracın yanal sapması bunu aşarsa dubaların arasından
                # geçmemiş demektir.
                yari_gen = 0.5 * math.hypot(a.x - b.x, a.z - b.z)
                self.gecis_baslat(mz, self.gecit_geometri(a, b), yari_gen)
                if MOD == "dogrudan_surus":
                    self.hiz_yayinla(CRUISE_HIZ, 0.0)
            elif MOD == "mppi_hedef":
                self.gecit_hedefi_yayinla(a, b)
            elif MOD == "dogrudan_surus":
                w = -KP_YAW * bearing * YAW_ISARET
                w += self.kacinma_bias((a, b))
                w = max(-MAX_YAW, min(MAX_YAW, w))
                v = max(MIN_HIZ, CRUISE_HIZ * math.cos(bearing))
                self.hiz_yayinla(v, w)
            # algi_yayin: yaklaşmada çıkış yok — sürüş karar stack'inde
        else:
            sg = self.son_gecit
            if (sg is not None
                    and (simdi - sg[2]) < 2.0 * HEDEF_KAYIP_SN
                    and sg[1] < PASS_KAYIP_Z
                    and abs(sg[0]) < math.radians(25)):
                # Geçide iyice yaklaşmışken dubalar FOV'dan çıktı -> geçiyoruz.
                # Geçit çizgisini son bilinen bearing/mesafeden kur (yaklaşıkla yeter)
                ox = sg[1] + KAMERA_OFSET_ILERI
                oy = -(sg[1] * math.tan(sg[0]))
                d = math.hypot(ox, oy) or 1.0
                self.gecis_baslat(sg[1], (ox, oy, ox / d, oy / d),
                                  self.son_yari_gen)   # B5: son bilinen genişlik
                if MOD == "dogrudan_surus":
                    self.hiz_yayinla(CRUISE_HIZ, 0.0)
            else:
                self.duruma_gec("ARAMA")
                if MOD == "mppi_hedef":
                    self.arama_hedefi_yayinla(simdi)
                elif MOD == "dogrudan_surus":
                    if simdi - self.arama_baslangic > ARAMA_TIMEOUT_SN:
                        self.hiz_yayinla(0.0, 0.0)   # güvenlik: dur
                    else:
                        self.hiz_yayinla(SEARCH_HIZ,
                                         -SEARCH_YAW * self.son_taraf * YAW_ISARET)
                # algi_yayin: arama davranışı yok — hedefleri mission_manager üretir

        self.durum_log()

    # ---------- PLAN A çıkışı: Nav2/MPPI'ya hedef ----------
    def gecit_geometri(self, a, b):
        """base_link'te geçit ortası (ox,oy) + geçide dik İLERİ birim vektör (px,py).
        Kamera -> base_link: ileri x = Z + ofset, sol y = -X (kamera X sağ+)."""
        ax, ay = a.z + KAMERA_OFSET_ILERI, -a.x
        bx, by = b.z + KAMERA_OFSET_ILERI, -b.x
        ox, oy = (ax + bx) / 2.0, (ay + by) / 2.0
        # Geçit çizgisine dik, İLERİ bakan birim vektör
        gx, gy = bx - ax, by - ay
        n = math.hypot(gx, gy) or 1.0
        px, py = -gy / n, gx / n
        if px < 0:
            px, py = -px, -py
        return ox, oy, px, py

    def arac_poz_yaw(self, timeout_s=0.0):
        """Aracın sabit çerçevedeki (x, y, yaw) pozu; kaynak MOD'a göre:
        algi_yayin → /girdap/fusion/odom aboneliği (karar stack'i TF yayınlamaz),
        mppi_hedef → TF (odom->base_link). Poz yoksa None."""
        if MOD == "algi_yayin":
            return self.son_odom
        try:
            tf = self.tf_buffer.lookup_transform(
                HEDEF_FRAME, BASE_FRAME, RclTime(),
                timeout=Duration(seconds=timeout_s))
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = tf.transform.translation
        return (t.x, t.y, yaw)

    def cizgi_hedef_frame(self, gecit_bl):
        """base_link'teki geçit çizgisini (orta + normal) sabit çerçeveye taşı."""
        pz = self.arac_poz_yaw(timeout_s=0.2)
        if pz is None:
            self.get_logger().warn("Geçiş doğrulaması için poz yok, zamana düşüldü")
            return None
        tx, ty, yaw = pz
        ox, oy, px, py = gecit_bl
        c, s = math.cos(yaw), math.sin(yaw)
        return (tx + c * ox - s * oy, ty + s * ox + c * oy,
                c * px - s * py, s * px + c * py)

    def gecit_hedefi_yayinla(self, a, b, zorla=False):
        """Geçit orta noktasının ötesine, geçide dik yönelimli hedef bas."""
        ox, oy, px, py = self.gecit_geometri(a, b)
        hx = ox + px * HEDEF_OTELEME      # hedef geçidin ÖTESİNDE ki araç
        hy = oy + py * HEDEF_OTELEME      # ortada durmasın, komple geçsin
        self.pose_yayinla(hx, hy, math.atan2(py, px), zorla)

    def arama_hedefi_yayinla(self, simdi):
        """Geçit görünmüyorken periyodik arama hedefi: son görülen tarafa açılı.
        (Plan B'nin taramasının Plan A karşılığı — kamera FOV'unu geçide çevirir.)"""
        if simdi - self.arama_baslangic > ARAMA_TIMEOUT_SN:
            self.get_logger().warn("Uzun süredir geçit yok — yeni hedef basılmıyor.",
                                   throttle_duration_sec=10.0)
            return
        if simdi - self.son_arama_goal_t < ARAMA_HEDEF_SN:
            return
        self.son_arama_goal_t = simdi
        yaw = -ARAMA_YAW * self.son_taraf   # son_taraf=+1: geçit SAĞDA görülmüştü -> sağa
        self.pose_yayinla(ARAMA_ILERI * math.cos(yaw), ARAMA_ILERI * math.sin(yaw),
                          yaw, zorla=True)

    def pose_yayinla(self, x, y, yaw, zorla=False):
        """base_link'teki (x,y,yaw) hedefini HEDEF_FRAME'e çevirip yayınla."""
        p = PoseStamped()
        p.header.frame_id = BASE_FRAME
        p.header.stamp = RclTime().to_msg()   # zaman=0: eldeki en son TF kullanılır
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        try:
            hedef = self.tf_buffer.transform(p, HEDEF_FRAME,
                                             timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(
                f"TF yok ({BASE_FRAME}->{HEDEF_FRAME}): {e}",
                throttle_duration_sec=5.0)
            return

        simdi = time.time()
        if not zorla and self.son_goal is not None:
            dx = hedef.pose.position.x - self.son_goal[0]
            dy = hedef.pose.position.y - self.son_goal[1]
            if (math.hypot(dx, dy) < GOAL_GUNCELLE_MESAFE
                    and (simdi - self.son_goal_t) < GOAL_GUNCELLE_SN):
                return   # hedef pek kaymadı: Nav2'yi hedef yağmuruna tutma

        hedef.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(hedef)
        self.son_goal = (hedef.pose.position.x, hedef.pose.position.y)
        self.son_goal_t = simdi

    # ---------- PLAN B çıkışı: doğrudan hız ----------
    def kacinma_bias(self, cift):
        bias = 0.0
        for d in self.dubalar:
            if d in cift:
                continue
            if d.z < ENGEL_KACIN_Z and abs(d.x) < ENGEL_KORIDOR:
                guc = K_KACIN * (1.0 - d.z / ENGEL_KACIN_Z)
                bias += math.copysign(guc, d.x) * YAW_ISARET
        return bias

    def hiz_yayinla(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    # ---------- Log ----------
    def durum_log(self):
        simdi = time.time()
        if simdi - self._son_log < 2.0:
            return
        self._son_log = simdi
        k = sum(1 for d in self.dubalar if d.cls == self.kenar_cls)
        e = sum(1 for d in self.dubalar if d.cls == self.engel_cls)
        self.get_logger().info(
            f"[{MOD}|{self.durum}] kenar={k} engel={e} | geçit={self.gecit_sayisi}"
            f" | NN {self.olculen_fps:.1f} FPS"
            + (" | GÖREV TAMAM" if self.gorev_tamam else ""))
        # SESSİZ RET uyarısı: kenar dubası görüyoruz ama kapı kuramıyoruz.
        if k >= 2 and self.durum == "ARAMA" and any(self._tani.values()):
            self.get_logger().warn(
                f"kapı kurulamıyor — red sebepleri: dar={self._tani['dar']} "
                f"dizili={self._tani['dizili']} arada_duba={self._tani['arada_duba']} "
                f"menzil_çelişki={self._tani['menzil_celiski']}",
                throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = DubaNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if MOD == "dogrudan_surus":
            node.hiz_yayinla(0.0, 0.0)   # güvenlik: çıkarken motorları sıfırla
        try:
            node.kayit_kapat()           # Dosya-1 son segmentini kapat
        except Exception:
            pass
        try:
            node.pipeline.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

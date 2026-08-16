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
# ⏱️ SAAT KURALI (2026-08-09) — İKİ SAAT VAR, KARIŞTIRMA:
#   SÜRE / zaman aşımı / periyot   -> time.monotonic()   (duvar saati DEĞİL)
#   MUTLAK an (damga, etiket, tarih) -> time.time() / ROS saati
# NEDEN: Jetson'da RTC pili yok, boot'ta saat geride açılıyor (ölçüldü: bir kez
# ~15 saat, bir kez bir gün) ve kıyı yordamımız saati NODE ÇALIŞIRKEN
# düzeltiyor (`sudo date -s`, ya da tethering gelince NTP adımı).
# `time.time()` ayarlanabilir (`time.get_clock_info('time').adjustable=True`),
# `time.monotonic()` değildir. Duvar saati ileri sıçrarsa geçiş penceresi
# (`pass_bitis_t`) ANINDA dolar (G puanı), geri sıçrarsa `durum_log` susar
# (sahadaki tek görünürlük kanalı) ve Dosya-1 kaydı durur (md 4.2 = 5 ceza p.).
# (Bu ayrımı veri seti toplayıcısı da yapıyordu; toplayıcı 16.08'de repodan
#  kaldırıldı — ayrım burada kendi başına geçerli, tarihsel not olarak duruyor.)
# Regresyon testi: test/test_saat_kaynagi.py
from dataclasses import dataclass

import depthai as dai
import rclpy

try:                                  # paket içi saf mantık (kamerasız testli)
    from girdap_ida_algi import gecit_mantik as gm
    from girdap_ida_algi import oak_baglanti as ob
    from girdap_ida_algi import saat as st
except ImportError:                   # dosya doğrudan çalıştırılırsa
    import gecit_mantik as gm
    import oak_baglanti as ob
    import saat as st
# ⚠️ `ob` import'u 05.08 smoke testinde eklendi — daha önce YOKTU ama kod
# ob.dayanikli_ac çağırıyordu. Fark edilmedi çünkü node model dosyası
# olmadan ilk satırda çöküyor, bu satıra hiç ulaşılamıyordu. Ders:
# "import atlanmış mı" ancak modül GERÇEKTEN çalıştırılınca görülür.
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
    from vision_msgs.msg import (
        Detection3D,
        Detection3DArray,            # sudo apt install ros-humble-vision-msgs
        Detection2D, Detection2DArray, ObjectHypothesisWithPose)

# ================== AYARLAR ==================
# ---- Model: YOLOv11n @ 512x512 (depthai v2: .blob + yanında config.json) ----
# 🔴 v2'de NNArchive YOK (2.30.0.0'dan introspection ile doğrulandı) → model
# .blob olarak verilir (HubAI/modelconverter çıktısı NNArchive tar.xz ise
# içinden blob + config.json ÇIKARILIR — arşivde ikisi de var, tar -xf yeter).
# Sınıf isimleri blob'un YANINDAKİ config.json'dan okunur (NNArchive config
# formatı: model.heads[0].metadata.classes) → isimden sınıf çözümü (A-7)
# v2'de de yaşar. config.json yoksa yedek sabitlere düşülür.
# 🔴 Blob ≤4 SHAVE derlenmiş olmalı (05.08 kamerada ölçüldü): 12MP RGB modu
# (tam 4:3 FOV) 6 CMX dilimi yiyor → NN'e 4 shave + 4 CMX kalıyor; 6-shave
# blob "compiled for 6 shaves, only 4 available" ile YÜKLENMEDİ. 4-shave'le
# ölçüm: 10,9 FPS @ 11 istendi, tavan 19,8 — 11 hedefi rahat. (Eski "≤7"
# bütçesi v3 + küçük RGB çıkışı dönemindendi.) FOV eşitliği (veri seti ↔
# deploy) shave sayısından kritik: 16:9'a inmek dikey FOV'u kırpar, model
# geçide 2 m kala alt bölgedeki dubayı kaybeder.
MODEL_BLOB = "/home/girdap/models/yolo11n_duba_rvc2.blob"
NN_GIRIS = 512          # NN giriş boyutu; blob bu boyutla derlenir, İKİSİ BİRLİKTE değişir
# 🔴 2026-08-12: 416 → 512. Gerekçe ÖLÇÜM, tercih değil — darboğazımız MENZİL:
# recall dubanın piksel genişliğine çok duyarlı (3-4 px %30 · 4-5 px %80 ·
# 5-6 px %90 · 8-11 px %99) ve 20 m'deki duba 416'da 4,5 px (=%80 bandı),
# 512'de 5,6 px (=%90 bandı). Kabul testi (aynı valid, aynı video tezgâhı):
# 20+ m recall %49,3 → %62,7 · genel recall %96,1 → %97,0 · kapı kurulabilen
# kare %92,5 → %94,9 · yanlış kutu 56 → 47.
# ⚠️ Sınıf hatası 3 → 7 (kabul kapısı ≤5'ti, GEÇMEDİ): 6'sı 416'nın hiç
# göremediği 13-27 m dubalarında — yeni hata değil, hata yapma FIRSATI.
# Yalnız 1'i gerçek gerileme (5,9 m). Bu yüzden 512 "denemek için" alındı;
# yarışma kararı cihazda KONTROL 3 + sahadan sonra.
# 🔴 608 ELENDİ: 4,63 FPS (sert duvar — ara katmanlar 4 shave'in yerel
# belleğine sığmayıp dış belleğe taşıyor); 512→608 için 1 piksel başına
# 5 FPS ödeniyor. Üç boyut da 4 shave'e SIĞIYOR (ölçüldü).

# Sınıf indeksleri — YALNIZCA YEDEK. Gerçek indeksler çalışma anında NN
# Archive'ın sınıf İSİMLERİNDEN çözülür (_sinif_indeksleri_coz).
# Sabit indekse güvenmek tehlikeli: elimizdeki eğitilmiş model
# Gazebonew.pt'nin sırası {0: "Engel Dubasi", 1: "Kenar Dubasi"} — yani
# aşağıdaki sabitlerin TERSİ. Ters sırada geçit tespiti iki SARI engeli
# kenar çifti sanar ve Parkur-2 sessizce çöker.
KENAR_CLASS = 0        # turuncu duba (RAL 2003) - parkur kenarı  [yedek]
ENGEL_CLASS = 1        # sarı duba   (RAL 1026) - engel           [yedek]
CONF_ESIK = 0.5
FPS = 8                # sensorFps üst sınırı. 🔴 2026-08-12: 11 → 8 (NN_GIRIS 512
                       # ile BİRLİKTE). ÖLÇÜLDÜ (2026-08-11, dağıtımın birebir
                       # aynı ayarıyla: 12MP + ispScale 1/3, stereo AÇIK, USB2):
                       # boru hattı tavanı 416 → 12,00 · **512 → 9,83** · 608 → 4,63.
                       # 🪤 Kayıtlardaki "20,6 FPS" YANLIŞTI (stereo kapalı ölçülmüş)
                       # — 11 hedefi meğer 416 tavanının hemen altındaymış.
                       # 8 seçildi (9 değil): 512@9'da güneş altı termal tahmini
                       # ~90 °C, uyarı eşiğimiz 85 / kritik 95 ve cihazda OTOMATİK
                       # KISMA YOK (çökme 125 °C). 8 = tavanın %19 altı.
                       # ⚠️ Bu termal sayı TAHMİN — 512@8 platosu Jetson'da 15 dk
                       # ölçülüp buraya yazılacak. Gölgelik yine de ŞART (FPS'ten
                       # ~10 kat etkili kaldıraç).
                       # 🔑 8 FPS YETERLİ: dümeni algı sürmüyor, MPPI 20 Hz koşuyor;
                       # 1,5 m/s'te kare arası yol 18,8 cm ↔ duba çapı 30 cm.
                       # Şartname md 4.2 Dosya-1 şartı ≥1 Hz.
FPS_UYARI_ESIK = 6.5   # ölçülen NN FPS bunun altına düşerse logda uyar.
                       # 🪤 HEDEF FPS'İN ALTINDA KALMALI. Eskiden 8.0 idi; FPS 11
                       # iken doğruydu, ama FPS 8'e inince eşik hedefin ÜSTÜNE
                       # çıkmış olurdu ⇒ alarm HER KARE yanardı. Bizde bunun dersi
                       # yazılı: "her zaman yanan alarm alarm değildir" (09.08,
                       # mono_menzil vakası). Eşik hedefin ~%80'i.

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
# 🔴 ODOM BAYATLIK SINIRI (2026-08-13, ölçüldü) — poz "var ama donuk" olabilir.
# Karar tarafı `fusion_node` girdi akışı kesilince odom yayınını BİLEREK KESİYOR
# (F8.2, `fusion_node.py:560-568`; varsayılan `pose_timeout_s = 1.0` s ve
# `hardware.yaml:83` bunu açıkça uyarıyor: "1-2 Hz'de odom yayını KESİLİR").
# Yayın kesilince bizim `son_odom` SON DEĞERDE DONAR ve eski kod onu geçerli
# poz sayardı. Ölçülen sonuç (scratchpad/dogrula2.py, aynı fiziksel geçiş):
#     odom HİÇ gelmemiş -> geçit SAYILDI (1)
#     odom BAYAT/DONUK  -> geçit SAYILMADI (0)
# Yani donuk poz, poz olmamasından DAHA KÖTÜ: uzun pencereli çizgi kuruluyor,
# `gecitten_gecti` donuk pozla hep False dönüyor, geçiş "doğrulanamadı" diye
# elenip (G/KD)×10 ve ×40 puanı sessizce gidiyor.
# 1,5 s = karar tarafının kendi eşiği (1,0) + bir tık pay; altında kalan kısa
# kesintiler zaten normal jitter'dır ve davranışı DEĞİŞTİRMEZ.
ODOM_BAYAT_SN = 1.5
KAMERA_FRAME = "oak_rgb"

# Sınıf eşlemesi — girdap-decision class_id STRING sözleşmesi:
# "0"=parkur_kenarı (turuncu), "1"=engel (sarı). Asıl eşleme çalışma anında
# kurulan self.sinif_esleme'dir; bu sabit yalnız sınıf isimleri okunamazsa
# devreye girer.
SINIF_ESLEME = {KENAR_CLASS: "0", ENGEL_CLASS: "1"}

# ---- GERÇEK kamera karesi ----
# 🔴 SABİT YOK — bilerek. Deploy zinciri: 12MP → ispScale(1,3) = 1352×1014 (ISP)
# → preview 512×512 (NN). Kayıt overlay'i kare boyutunu `frame.shape`'ten
# OKUR (kayit_adimi), sabitten değil — eskiden burada duran `IMG_W/IMG_H =
# 640, 480` çifti 05.08'deki 12MP taşımasından sonra GERÇEĞİ YANSITMIYORDU
# ve yalnız ölü letterbox hesabını besliyordu (aşağıya bkz.).

# ---- YAYINLANAN bbox piksel uzayı (Detection2D piksel uzayı) ----
# 🔴 BU İKİSİ, TÜKETİCİNİN PARAMETRESİYLE AYNI OLMAK ZORUNDA:
#   son_kodv2 perception_fusion_node → camera_image_width_px / height_px
#   (hardware.launch.py:211-212 · hardware.yaml:207-208 · params.yaml:228-229
#    üçünde de 1280x720 — 2026-07-17'de oakd_driver 1280x720'e çıkınca)
# Mesaj görüntü boyutunu TAŞIMADIĞI için tüketici bbox merkezini bu sayıya
# böler; uyuşmazsa hiçbir hata basılmaz, yalnız bearing sessizce kayar
# (640 yayınlayıp 1280'e bölmek → kare ortasındaki duba +17°'de görünür,
# tolerans 8,6° → kamera tespiti hiçbir LiDAR kümesine eşleşmez → sınıf
# düşer → geçit bulunamaz: P1 G1/KD1≥0,5 ve P2 ≥2 ikili sağlanmaz).
# GERÇEK kare boyutundan BAĞIMSIZDIR — bilinçli: tüketici yalnız bbox
# merkezini bu sayıya böldüğü için ölçek yeterli, HFOV değişmez.
BBOX_W, BBOX_H = 1280, 720

# ---- LETTERBOX PAYI = 0 (sabit değil, ön işlemenin SONUCU) ----
# 🔴 Deploy ön işlemesi SIKIŞTIRMA (stretch): setPreviewKeepAspectRatio(False)
# 4:3 kareyi 512×512'ye EZER → üst/alt gri şerit OLUŞMAZ → çıkarılacak pay YOK.
# Bu yüzden __init__'te `self._lb_pay = 0.0` sabitlenir (bkz. tespitleri_oku).
# Eskiden burada `_LB_ICERIK/_LB_PAY = 0.125` hesabı vardı; ölüydü (hiçbir yerde
# okunmuyordu) ama "deploy letterbox yapıyor" izlenimi veriyordu — SİLİNDİ.
# ⚠️ Letterbox'a DÖNÜLÜRSE: keepAspectRatio(True) + `gm.letterbox_payi()`
# (saf, testli katman) birlikte devreye girer ve EĞİTİM ön işlemesi de
# aynı anda letterbox'a çevrilmelidir — ikisi ASLA ayrı değişmez.

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
#   (a) Piksel sınırı: HFOV 69°, NN 512 px → 30 cm duba 15 m'de ~7,4 px, 10 m'de
#       ~11 px, 5 m'de ~22 px. (416'da sırasıyla 6 / 9 / 18 idi.) 🔴 DEĞER
#       DEĞİŞMEDİ: 512'ye geçmek bu tavanı yükseltmiş OLABİLİR ama burada
#       ölçüm yok — hesap değişti, ölçüm değişmedi. Suda ölçülene kadar 8,0.
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
# ── PARKUR-3 hedef yayını (FAZ 2, 2026-08-13) ────────────────────────────
#: `/perception/targets` yayın hızı. Kare zaten Dosya-1 için 2 Hz'te
#: çekiliyor; hedef adımı ondan hızlı koşamaz, ek kare ÇEKMİYORUZ.
HEDEF_HZ = 2.0
TARGETS_TOPIC = "/perception/targets"

#: 🔴 PARKUR-3 HEDEF YAYINI ANA ŞALTERİ — VARSAYILAN **KAPALI** (16.08.2026).
#: Eyüp kararı: *"Parkur 3'ü şimdilik aktif etme, testte bozar; Parkur 1 ve
#: Parkur 2'yi ölçüyoruz."*
#:
#: Kapalıyken: `/perception/targets` HİÇ yayınlanmaz, bbox kırpma + renk
#: analizi HİÇ koşmaz ⇒ P1/P2 ölçümü P3 kodundan **tamamen etkilenmez**
#: (ne CPU, ne log gürültüsü, ne de topic trafiği).
#:
#: ⚠ BU ŞALTER *BÜYÜK CİSİM SÜZGECİNİ* KAPATMAZ — o bilerek parkurdan
#: bağımsız ve her zaman açık: P3 hedefi Ø64 cm, 25 m'den 7,7 px eder ve
#: tekne daha **P2'nin İÇİNDEYKEN** görür. Kenar/engel diye yayınlanırsa
#: füzyon `EdgeBuoyMemory`'ye KALICI kenar yazar → hayalet kapı → **P2'de
#: rota bozulur**. Yani süzgeç P3 için değil, P2'yi KORUMAK için var.
#:
#: Açmak için (yarışma günü, kalkıştan önce) — servis dosyasına ekle:
#:     Environment=GIRDAP_P3_HEDEF=1
#: (ROS parametresi değil: bu node'da `declare_parameter` altyapısı yok;
#: ortam değişkeni tek satırlık ve systemd'den yönetilebilir.)
P3_HEDEF_YAYINI = os.environ.get("GIRDAP_P3_HEDEF", "0").strip() in (
    "1", "true", "True", "evet"
)

#: 🔴 STEREO YOKKEN hedef ayrımı — 16.08.2026, ÖLÇÜMLE belirlendi.
#: Menzil yoksa boyut kapısı kurulamaz (mono z zaten Ø0,30 varsayımından
#: türetilir ⇒ kıyas dairesel). Geriye tek ayırt edici kalıyor: RENK.
#: 6.042 GERÇEK kenar/engel kutusunda (eski+yeni oturum, karanlık/ters ışık
#: dahil) hedef renk eşiklerinin yanlış pozitif oranı:
#:   kapsama eşiği | yeşil  | siyah  | kırmızı
#:   0,25          | 0,000% | 0,232% | 13,87%
#:   0,35          | 0,000% | 0,050% |  7,75%
#:   **0,45**      | **0,000%** | **0,000%** |  4,83%
#:   0,65          | 0,000% | 0,000% |  0,53%
#: ⇒ 0,45'te yeşil ve siyah TEMİZ; kırmızı hiçbir eşikte temizlenmiyor
#: (RAL 3026 ↔ turuncu kenar dubamız RAL 2003 tonda komşu) ⇒ **kırmızı yok**.
#: ⛔ GERİ ALINIRSA: >10 m'de hedef dubası yine `/perception/buoys`'a girer,
#: `EdgeBuoyMemory`'de KALICI kenar kaydı açar (unutma yok) ⇒ hayalet kapı
#: (P1/P2 rotası bozulur) + MPPI hedeften KAÇAR (P3 angajmanı kaçar).
MONO_HEDEF_ORANI = 0.45
MONO_HEDEF_RENKLERI = ("yesil", "siyah")
#: 🔴 Bu topic P1/P2'nin kullandığı `/perception/buoys`'tan **AYRI**. Hedef
#: tespitleri oraya ASLA karışmaz: karışırsa füzyon `EdgeBuoyMemory`'ye kalıcı
#: kenar kaydı açar → iki hedef arasında hayalet kapı → P2 rotası bozulur
#: (11.08 gerekçesi, `buyuk_cisim_mi` docstring'i).

KAYIT_SEGMENT_SN = 120.0  # çökme dayanımı: 2 dk'lık mp4 segmentleri (yarıda
                          # kesilirse en fazla son segment zarar görür)
KAYIT_DIZIN = os.path.expanduser("~/girdap_logs/kamera")

if KAYIT_AKTIF:
    import cv2            # overlay çizimi + mp4 yazımı (yalnız kayıt aktifken)

KONTROL_HZ = 15.0
HEDEF_KAYIP_SN = 1.0   # s - tespit tazeliği; 8 FPS'te 8 kareye denk (tam sınırda
                       # değil: 10 FPS'e düşerse 10 kare kalır, kare düşünce
                       # tazelik filtresi tetiklenebilir — 11 bu yüzden 10'a yeğlendi)
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
    # Menzilin nereden geldiği: "stereo" (varsayılan) | "mono" (pinhole yedeği).
    # Tanılama için ve çapraz kontrolün kendi kendini doğrulamasını önlemek için.
    kaynak: str = "stereo"


def _model_siniflarini_oku(blob_yolu: str):
    """Blob'un yanındaki config.json'dan (NNArchive formatı) sınıf isimleri.

    v2'de NNArchive/getClasses() yok → isimler sidecar dosyadan okunur ki
    isimden sınıf çözümü (A-7: yeniden eğitimde sıra değişse bile turuncu/sarı
    karışmaz) v2'de de çalışsın. Okunamazsa [] döner → yedek sabitler.
    Aranan yerler: <blob_dizini>/config.json, sonra <blob_adı>.json.
    """
    import json
    dizin = os.path.dirname(blob_yolu)
    adaylar = (os.path.join(dizin, "config.json"),
               os.path.splitext(blob_yolu)[0] + ".json")
    for yol in adaylar:
        try:
            with open(yol, encoding="utf-8") as f:
                cfg = json.load(f)
            heads = cfg.get("model", {}).get("heads", [])
            siniflar = heads[0].get("metadata", {}).get("classes", []) if heads else []
            if siniflar:
                return [str(s) for s in siniflar]
        except (OSError, ValueError, KeyError, IndexError):
            continue
    return []


def _blob_denetle(blob_yolu: str, siniflar, logger=None):
    """Blob ↔ config.json ↔ koddaki NN_GIRIS uyumunu AÇILIŞTA doğrula.

    🔴 NEDEN (2026-08-10 derin tarama): blob `/home/girdap/models/` altına
    ELLE kopyalanıyor (JETSON-KURULUM adım 2). Yanlış/eski blob konursa:
      · giriş boyutu tutmazsa  -> preview 512 ↔ blob 416 => çöp tespit
      · sınıf sayısı tutmazsa  -> decode kayar => turuncu↔sarı KARIŞIR => Ç2
    İkisi de **belirti vermeden** olur. Blob başlığı bu bilgiyi taşıyor
    (giriş dims + çıkış kanal sayısı = 5 + nc, yolov6r2 kafası) ⇒ bedava kontrol.

    Uyarı basar, ASLA yükseltmez: tanı kodu node'u öldürmemeli
    (09.08 dersi — teşhis için eklenen satır node'u öldürmüştü).
    Döner: sorun listesi (boşsa temiz).
    """
    sorunlar = []
    try:
        b = dai.OpenVINO.Blob(blob_yolu)
        giris = list(next(iter(b.networkInputs.values())).dims)[:2]
        if giris != [NN_GIRIS, NN_GIRIS]:
            sorunlar.append(f"blob girişi {giris} ≠ NN_GIRIS {NN_GIRIS}×{NN_GIRIS}")
        kanallar = {list(v.dims)[2] for v in b.networkOutputs.values()}
        if len(kanallar) == 1:
            nc = kanallar.pop() - 5          # yolov6r2: x,y,w,h,obj + nc
            if siniflar and nc != len(siniflar):
                sorunlar.append(
                    f"blob {nc} sınıf ↔ config.json {len(siniflar)} sınıf "
                    f"({list(siniflar)}) — DECODE KAYAR, sınıflar karışır")
        if b.numShaves > 4:
            sorunlar.append(f"numShaves={b.numShaves} > 4 — cihaz REDDEDER")
    except Exception as e:                    # blob okunamadı: pipeline zaten patlar
        sorunlar.append(f"blob başlığı okunamadı: {type(e).__name__}: {e}")
    if sorunlar and logger is not None:
        for s in sorunlar:
            logger.error(f"🔴 MODEL DENETİMİ: {s}")
        logger.error("🔴 Yanlış model BELİRTİ VERMEDEN çöp tespit üretir — "
                     "doğru blob: models/yolo11n_duba_rvc2.blob (+ config.json)")
    return sorunlar


def pipeline_kur(kaydet=None):
    """OAK-D Lite üzerinde çalışan pipeline: RGB + Stereo + YOLO (VPU'da).

    depthai v2 (2.30.0.0) API'si — 2026-08-05'te v3'ten taşındı (v3 firmware'i
    mono kameraları açamıyor → stereo %0; v2'de stereo 29,7 FPS ölçüldü).
    Desen, bu cihazda 20 dk kesintisiz koşan ölçüm aracıyla
    (scripts/oak_derinlik_termal_testi.py) birebir aynı; tek bilinçli fark
    RGB sensör modu (aşağıda).

    Args:
        kaydet: opsiyonel log fonksiyonu (tek konumsal argüman alır).
                `cihazi_bekle`ye geçirilir — cihaz USB'de yokken bekleyiş
                SESSİZ kalmasın diye. None ise bekleyiş görünmez olur; node
                bunu her zaman `self.get_logger().warn` ile besler.
    """
    siniflar = _model_siniflarini_oku(MODEL_BLOB)
    _blob_denetle(MODEL_BLOB, siniflar)      # logger yok: node __init__'te tekrar basılır

    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    # 🔴 THE_12_MP + ispScale(1,3) = 1352×1014, TAM 4:3, KIRPMASIZ tam FOV.
    # Ölçüm aracı THE_1080_P (16:9) kullanıyordu — o mod sensörü DİKEY KIRPAR.
    # Veri seti 05.08'den beri tam 4:3 (1352×1014) toplanıyor; deploy aynı
    # çerçeveyi görmeli, yoksa model sahada öğrendiğinden farklı FOV görür.
    # (THE_1440X1080 enum'da var ama bu cihazda SESSİZCE 0 kare üretiyor —
    # 05.08 mod taraması; 12MP+ispScale ölçüldü: RGB-only 18,6 FPS tavan.)
    # ⚠️ FPS/termal referans ölçümleri 1080P ile alındı → FPS bandı bu modla
    # masa testinde YENİDEN doğrulanmalı (duba_kamera_test.py).
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    cam_rgb.setIspScale(1, 3)
    cam_rgb.setPreviewSize(NN_GIRIS, NN_GIRIS)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    # False = tam kare 512×512'ye SIKIŞTIRILIR (letterbox şeridi YOK) —
    # Luxonis'in resmî YOLO örnekleriyle ve bu cihazdaki ölçülü desenle aynı.
    # Sonuçları: (a) tam FOV korunur, (b) bbox normalize koordinatları kaynak
    # kareye DOĞRUDAN ölçeklenir → letterbox payı 0 (bkz. tespitleri_oku),
    # (c) eğitim de AYNI ön işlemeyle yapılmalı (models/README'ye işlendi).
    cam_rgb.setPreviewKeepAspectRatio(False)
    cam_rgb.setFps(FPS)
    if RGB_SABIT_FOKUS is not None:
        cam_rgb.initialControl.setManualFocus(RGB_SABIT_FOKUS)

    mono_sol = pipeline.create(dai.node.MonoCamera)
    mono_sag = pipeline.create(dai.node.MonoCamera)
    mono_sol.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_sag.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for m in (mono_sol, mono_sag):
        # THE_480_P = mono sensörün (OV7251) TAM karesi. 2026-08-04 düzeltmesi
        # v2'de de geçerli: 400P üstten/alttan kırpar, dikey FOV ~%17 daralır;
        # geçide 2 m kala duba karenin ALT bölgesinde → derinlik kaybı →
        # tespit z<=0.05 filtresine takılıp tam tetikleme anında düşer.
        m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        m.setFps(FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    # ROBOTICS preseti (Luxonis dokümanı, 2026-08-04 araştırması): "navigasyon,
    # engel tespiti", çalışma aralığı **0-10 m**; DEFAULT 0-15 m'ye yayılır.
    # 🟠 05.08 ölçümü: iç mekânda DEFAULT %53 geçerli piksel, ROBOTICS %43-44,
    # hız aynı — SUDA tekrar kıyaslanacak, şimdilik ROBOTICS kalıyor.
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    # 🔴 ZORUNLU (05.08 ölçümü): setDepthAlign tek başına derinliği RGB
    # çözünürlüğüne ölçekler (4,1 MB/kare) → USB dolar → 8,1 FPS.
    # setOutputSize(640,400) ile 14,7 FPS.
    stereo.setOutputSize(640, 400)
    # LRC bedava (12,19 vs 12,20 FPS — 05.08) ve yanlış eşleşmeleri eler.
    stereo.setLeftRightCheck(True)
    mono_sol.out.link(stereo.left)
    mono_sag.out.link(stereo.right)

    sdn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    sdn.setBlobPath(MODEL_BLOB)
    sdn.setConfidenceThreshold(CONF_ESIK)
    # numClasses blob'a gömülü DEĞİL (v2) → config.json'dan; okunamazsa 2
    # (bizim duba modeli 2 sınıf). YANLIŞ sayı sessiz bozuk decode demektir.
    sdn.setNumClasses(len(siniflar) if siniflar else 2)
    sdn.setCoordinateSize(4)
    sdn.setIouThreshold(0.5)
    # ⚠️ YOLOv11 anchor-FREE: setAnchors/setAnchorMasks ÇAĞRILMAZ (05.08'de bu
    # cihazda anchor'sız doğru tespit ölçüldü: person conf=0.79).
    sdn.input.setBlocking(False)
    sdn.setBoundingBoxScaleFactor(0.5)
    sdn.setDepthLowerThreshold(300)      # mm
    # 20 m → 10 m (2026-08-04): 8 m ötesinde stereo Z hatası metrelerce
    # (baseline 7,5 cm); uzak "derinlik" çöpü hayalet duba konumu üretir ve
    # geçit çiftini bozar. ROBOTICS presetinin 0-10 m aralığıyla da uyumlu.
    sdn.setDepthUpperThreshold(10000)    # mm
    cam_rgb.preview.link(sdn.input)
    stereo.depth.link(sdn.inputDepth)

    xout_det = pipeline.create(dai.node.XLinkOut)
    xout_det.setStreamName("tespit")
    sdn.out.link(xout_det.input)
    if KAYIT_AKTIF:
        # Dosya-1 için NN giriş karesi. VPU'ya EK YÜK YOK — kare zaten
        # üretiliyor, sadece USB'den kopyası çekilir.
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_rgb.input.setBlocking(False)
        xout_rgb.input.setQueueSize(2)
        sdn.passthrough.link(xout_rgb.input)

    # 🔴 USB2'ye ZORLANIYOR (2026-08-05 ölçümü, bu Jetson'da):
    # SuperSpeed linki `tegra-xusb`ın U1/U2 güç durumu pazarlığında çöküyor —
    # cihaz bootlanıp SuperSpeed'e geçiyor, link dağılıyor, ROM'a düşüyor
    # (`X_LINK_DEVICE_NOT_FOUND`). HIGH'a zorlanınca **5/5** açılış.
    # Bant genişliği kaybı YOK: bu pipeline ~15 MB/s, USB2 tavanı ~35-40 MB/s.
    # Kilit gelirse dayanikli_ac() sudo'suz USB reset atıp yeniden dener —
    # teknede kameraya fiziksel erişim olmayacağı için bu ZORUNLU.
    # v2'de cihaz pipeline İLE açılır (v3'teki pipeline.start() yok).
    #
    # 🔑 F-A.4: `dayanikli_ac` artık her denemeden ÖNCE `cihazi_bekle()`
    # çağırıyor (16.08'de bağlandı — 15.08'de yazılmış ama çağrılmamıştı).
    # Kamera USB'de yoksa düğüm ÖLMEZ, bekler; `kaydet=` verdiğimiz için
    # bekleyiş 30 sn'de bir journal'a düşer. "Cihaz VAR ama açılmıyor" hâli
    # gerçek arızadır ve eski davranışını (USB reset + 4 deneme + raise) korur.
    dev = ob.dayanikli_ac(lambda: dai.Device(pipeline, dai.UsbSpeed.HIGH),
                          kaydet=kaydet)

    det_q = dev.getOutputQueue("tespit", maxSize=4, blocking=False)
    rgb_q = dev.getOutputQueue("rgb", maxSize=2, blocking=False) if KAYIT_AKTIF else None
    return dev, det_q, rgb_q, siniflar


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
            # FAZ 2 — Parkur-3 hedefleri. /perception/buoys'tan AYRI topic.
            self.targets_pub = self.create_publisher(
                Detection3DArray, TARGETS_TOPIC, 10)
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
            self.son_odom_t = -math.inf    # monotonic — bayatlık ölçümü (ODOM_BAYAT_SN)
        elif MOD == "mppi_hedef":
            self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.son_goal = None           # (x, y) HEDEF_FRAME içinde
            self.son_goal_t = -math.inf
            self.son_arama_goal_t = 0.0
        elif MOD == "dogrudan_surus":
            self.cmd_pub = self.create_publisher(
                TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        else:
            raise ValueError(f"Geçersiz MOD: {MOD}")

        # kaydet=: cihaz USB'de yokken bekleyiş journal'a düşsün (F-A.4).
        # Sessiz bekleyiş, çökme döngüsünden bile kötüdür: "sessizlik başarı
        # değildir" — kimse düğümün neyi beklediğini göremezdi.
        self.dev, self.det_q, self.rgb_q, siniflar = pipeline_kur(
            kaydet=self.get_logger().warn)
        self._siniflar = siniflar or []
        # Dosya-1 kayıt durumu (şartname 4.2)
        self._kayit_bozuk = not KAYIT_AKTIF
        self._vw = None            # cv2.VideoWriter (tembel açılır)
        self._vw_t0 = 0.0
        self._seg_no = 0
        self._son_kayit_t = -math.inf
        if KAYIT_AKTIF:
            self._kayit_dizin = os.path.join(
                KAYIT_DIZIN, time.strftime("session_%Y%m%d_%H%M%S"))
            try:
                os.makedirs(self._kayit_dizin, exist_ok=True)
            except OSError as e:
                self._kayit_bozuk = True
                self.get_logger().error(f"Dosya-1 dizini açılamadı: {e}")
        # v2: cihaz pipeline_kur() içinde pipeline'la birlikte açıldı; ayrıca
        # start() yok. USB link hızı journal'a yazılır (kıyı kontrolü).
        try:
            self.get_logger().info(
                f"OAK-D Lite hazır — YOLO VPU'da. MOD = {MOD}, "
                f"USB = {str(self.dev.getUsbSpeed()).rsplit('.', 1)[-1]} [HIGH istendi]")
        except Exception:
            self.get_logger().info(f"OAK-D Lite hazır — YOLO VPU'da. MOD = {MOD}")
        for _s in _blob_denetle(MODEL_BLOB, self._siniflar, self.get_logger()):
            pass                      # logger içinde basıldı; node'u ÖLDÜRMEZ
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
        # FAZ 2 — P3 hedef adayları: `tespit_yayinla` içinde büyük cisim
        # süzgecine takılan tespitler. Her karede sıfırlanır (bayat hedef
        # yayınlamamak için).
        self._hedef_adaylari = []
        # Kare TEK YERDEN tazelenir (`_kare_tazele`); kayıt ve hedef adımı
        # ikisi de bunu okur. Numarayla takip: aynı kare iki kez işlenmesin,
        # ve biri diğerinin karesini "tüketmiş" olmasın.
        self._son_kare = None
        self._kare_no = 0
        self._kayit_kare_no = -1
        self._hedef_kare_no = -1
        self._son_hedef_t = 0.0
        self.son_tespit_t = -math.inf
        self.durum = "ARAMA"
        self.arama_baslangic = time.monotonic()
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
        self._tani = {"dar": 0, "dizili": 0, "arada_duba": 0, "menzil_celiski": 0,
                      # P3 hedef dubası (Ø0,64 m) sanılıp yayından SÜZÜLEN tespit.
                      # Sahada P2 boyunca 0, P3'e yaklaşınca artıyorsa DOĞRU çalışıyor.
                      "buyuk_cisim": 0,
                      # Stereo ölçemeyip pinhole yedeğine düşülen tespit sayısı.
                      # Sahada YÜKSEK çıkarsa stereo suda çalışmıyor demektir —
                      # SSH olmadığı için tek görünürlük kanalı bu sayaç.
                      "mono_menzil": 0,
                      # Stereo yokken RENKTEN P3 hedefi sayılan tespit
                      # (yalnız yeşil/siyah, `MONO_HEDEF_ORANI`). Sahada 0
                      # kalırsa ya hedef görünmedi ya renk çözülemedi —
                      # ikisi de P3'ün sessizce çalışmadığı anlamına gelir.
                      "mono_hedef": 0,
                      # Ne stereo ne mono menzil üretilemedi → tespit atıldı.
                      "menzil_yok": 0}
        # 🔴 2026-08-13: sessiz-ret alarmının ÖNCEKİ görüntüsü. Sayaçlar
        # KÜMÜLATİF (bilerek — toplamı sahada görmek istiyoruz) ama alarm
        # doğrudan onların sıfırdan büyük olmasına bakıyordu ⇒ ölçüldü:
        # tek bir geçmiş red, sonraki HER turda alarmı yakıyordu (5/5).
        # Bu, kendi yazılı dersimizin ihlali: "her zaman yanan alarm alarm
        # değildir" (09.08, mono_menzil). Alarm artık TOPLAMA değil, son
        # log'dan bu yana olan ARTIŞA bakıyor.
        self._tani_onceki = dict(self._tani)
        # Geçilen geçitlerin orta noktaları (dünya/odom çerçevesi). Şartname G
        # tanımı "FARKLI karşılıklı kenar dubaları arasından geçiş sayısı" →
        # aynı geçitten tekrar geçilirse SAYILMAZ (bkz. gm.yeni_gecit_mi).
        self.gecilen_gecitler = []
        # Pinhole odak (normalize bbox için): D = f·W/w_norm
        self._f_norm = gm.odak_px(1.0)
        # Letterbox payı = 0: v2'de preview keepAspectRatio(False) tam kareyi
        # 512×512'ye SIKIŞTIRIR — şerit yok, bbox normalize koordinatları kaynak
        # kareye her iki eksende DOĞRUDAN ölçeklenir. (gm.letterbox_payi saf
        # katmanı testli duruyor; letterbox'a dönülürse hazır — ama o gün
        # EĞİTİM ön işlemesi de birlikte değişmeli.)
        self._lb_pay = 0.0
        # S2: Dosya-1 (md 4.2) "her frame zaman etiketli" olmak zorunda; geçersiz
        # dosya başına 5 ceza (md 5.5.4.3.5). Kod saati DÜZELTEMEZ, ama sessiz
        # kalmamalı.
        # 🔴 2026-08-06'da GÜÇLENDİRİLDİ: eski kontrol `tm_year < 2026` idi —
        # yani yalnız YIL yanlışsa uyarırdı. Aynı gün Jetson ~15 saat bayat
        # saatle açıldı (05.08 19:50 iken gerçek 06.08 11:07); yıl doğru olduğu
        # için bu kontrol HİÇBİR ŞEY DEMEDİ ve veri seti tarafında 18 kare
        # yanlış tarihle "güvenilir" damgalandı. Artık çekirdeğin senkron
        # bayrağı soruluyor (adjtimex/STA_UNSYNC — ağ GEREKTİRMEZ).
        # Ayrıntı ve 8,9 saatlik sınır: girdap_ida_algi/saat.py
        self._saat_senkron = st.cekirdek_senkron_mu()
        self.get_logger().info(f"Saat: {st.saat_raporu(self._saat_senkron)}")
        if not st.saat_guvenilir_mi(time.time(), senkron=self._saat_senkron):
            self.get_logger().error(
                f"SAAT KANITLANAMIYOR ({time.strftime('%Y-%m-%d %H:%M:%S')}) — "
                "Dosya-1 zaman etiketleri yanlış olabilir (md 4.2, teslimde "
                "5 ceza riski). Koşudan ÖNCE: tethering ile NTP senkronu, ya da "
                "`sudo date -s '...'` → sonra bu node'u yeniden başlat.")
        self._son_log = -math.inf
        self._fps_n = 0            # ölçülen NN FPS (beklenen: ~8, 512 tavanı 9,83)
        self._fps_t0 = time.monotonic()
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
        t = time.monotonic()
        if t - self._fps_t0 >= 5.0:
            self.olculen_fps = self._fps_n / (t - self._fps_t0)
            self._fps_n, self._fps_t0 = 0, t
            if self.olculen_fps < FPS_UYARI_ESIK:
                self.get_logger().warn(
                    f"NN FPS düşük: {self.olculen_fps:.1f} (beklenen ~8, tavan 9,8) — "
                    "USB bağlantısı / VPU ısınması / kablo kontrol")
        if msg is None:
            return
        dets = []
        for d in msg.detections:
            if d.confidence < CONF_ESIK:
                continue
            cx = (d.xmin + d.xmax) / 2.0
            w_n = (d.xmax - d.xmin)
            z_stereo = d.spatialCoordinates.z / 1000.0
            # 🔴 2026-08-08: eskiden `if z <= 0.05: continue` vardı — stereo
            # ölçemeyince duba GÖRÜLDÜĞÜ HÂLDE atılıyordu. Su, derinlik için en
            # zor yüzey (dokusuz/aynasal) ve setDepthUpperThreshold(10000)
            # yüzünden 10 m ötesi zaten geçersiz dönüyor. Artık genişlikten
            # pinhole yedeği devreye giriyor; yalnız ATILACAK olanı kurtarır.
            z, kaynak = gm.menzil_coz(z_stereo, w_n, self._f_norm)
            if z is None:
                self._tani["menzil_yok"] += 1
                continue
            if kaynak == "mono":
                # Stereo yoksa X'i de geçersizdir (0 gelir) → bbox merkezinden
                # geometriyle üret, yoksa duba tam karşımızda sanılır.
                x = gm.yanal_konum(z, cx, self._f_norm)
                self._tani["mono_menzil"] += 1
            else:
                x = d.spatialCoordinates.x / 1000.0
            dets.append(Duba(
                int(d.label), x, z, float(d.confidence),
                cx=cx, cy=(d.ymin + d.ymax) / 2.0,
                w=w_n, h=(d.ymax - d.ymin), kaynak=kaynak))
        # v2 notu: v3'teki msg.getTransformation() yok — gerek de yok. Preview
        # keepAspectRatio(False) tam kareyi sıkıştırdığı için şerit oluşmuyor;
        # _lb_pay __init__'te 0 sabitlendi (letterbox'a dönülürse gm.letterbox_payi
        # saf katmanı hazır, 78+ testli).
        self.dubalar = dets
        self.son_tespit_t = time.monotonic()
        if MOD == "algi_yayin":
            self.tespit_yayinla()   # taze NN karesi → sözleşme topic'leri

    # ---------- Dosya-1: işlenmiş kamera kaydı (şartname 4.2, s.14) ----------
    def _kare_tazele(self):
        """RGB kuyruğunu **TEK YERDEN** boşalt, en taze kareyi sakla.

        🔴 Neden tek yer: `tryGet()` kareyi kuyruktan ALIR. İki ayrı boşaltıcı
        olsaydı hangisi önce koşarsa kareyi o kapardı, diğeri `None` görürdü —
        Dosya-1'de boşluk (md 4.2, ≥1 Hz) ya da hedefin hiç görülmemesi.
        Saklanan kare **ÇİZİLMEMİŞ** ham karedir; kayıt kendi kopyasına çizer.
        """
        frame = None
        while True:                      # kuyruğu boşalt, en tazesini al
            f = self.rgb_q.tryGet()
            if f is None:
                break
            frame = f
        if frame is None:
            return
        try:
            self._son_kare = frame.getCvFrame()
            self._kare_no += 1
        except Exception as e:           # noqa: BLE001
            self.get_logger().warn(f"kare alınamadı: {e}",
                                   throttle_duration_sec=5.0)

    def hedef_adimi(self, simdi):
        """Parkur-3 hedef adaylarını renklendirip `/perception/targets`'a yayınla.

        🔴 **Dosya-1'den TAMAMEN BAĞIMSIZ.** İki yönlü:
          · Buradaki bir hata kaydı **etkilemez** (kendi try'ı var; kaydın
            `except`'i `_kayit_bozuk=True` yapıp Dosya-1'i KALICI kapatıyor —
            md 4.2, eksik dosya **5 ceza puanı**).
          · Kayıt bozulsa da (`_kayit_bozuk`) hedef yayını **sürer**; aksi
            hâlde Parkur-3 Dosya-1'le birlikte sessizce ölürdü.

        Boş liste de yayınlanır: tüketici *"hedef yok"* ile *"node ölmüş"*ü
        ayırt edebilsin.
        """
        # 🔴 ANA ŞALTER — varsayılan KAPALI (bkz. P3_HEDEF_YAYINI).
        if not P3_HEDEF_YAYINI:
            return
        if simdi - self._son_hedef_t < 1.0 / HEDEF_HZ:
            return
        self._son_hedef_t = simdi
        try:
            arr = Detection3DArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.header.frame_id = BASE_FRAME       # perception = GÖVDE çerçevesi
            kare = self._son_kare
            kare_taze = kare is not None and self._kare_no != self._hedef_kare_no
            if kare_taze:
                self._hedef_kare_no = self._kare_no
            for d in self._hedef_adaylari:
                renk = None
                if kare is not None:
                    renk = self._hedef_rengi_coz(kare, d)
                det = Detection3D()
                det.header = arr.header
                # base_link: x=ileri, y=sol (buoys_3d ile AYNI dönüşüm)
                det.bbox.center.position.x = d.z + KAMERA_OFSET_ILERI
                det.bbox.center.position.y = -d.x
                # ÖLÇÜLEN çap (varsayım değil): pinhole tersinden
                # D = w_norm · z / f_norm. Tüketici "bu gerçekten 0,64 m mi"
                # diye kendi kapısını kurabilsin diye yayınlanıyor.
                cap = float(d.w * d.z / self._f_norm) if self._f_norm else 0.0
                det.bbox.size.x = cap
                det.bbox.size.y = cap
                hyp = ObjectHypothesisWithPose()
                # Sözleşme: 0=renk çözülemedi · 1=kırmızı · 2=yeşil · 3=siyah
                hyp.hypothesis.class_id = str(gm.HEDEF_RENK_KODU[renk])
                hyp.hypothesis.score = d.conf
                det.results.append(hyp)
                arr.detections.append(det)
            self.targets_pub.publish(arr)
            if arr.detections:
                self._tani["hedef_yayin"] += len(arr.detections)
        except Exception as e:           # noqa: BLE001
            # ⚠️ Dosya-1'in aksine BURADA kalıcı devre dışı bırakma YOK:
            # P3 geçici bir hatadan sonra kendini toparlayabilmeli.
            self.get_logger().warn(f"hedef yayını atlandı: {e}",
                                   throttle_duration_sec=5.0)

    def _hedef_rengi_coz(self, kare, d):
        """Tespitin bbox'ını karede kırp, rengini çöz. Çözülemezse None."""
        h, w = kare.shape[:2]
        x1 = max(0, int((d.cx - d.w / 2.0) * w))
        y1 = max(0, int((d.cy - d.h / 2.0) * h))
        x2 = min(w, int((d.cx + d.w / 2.0) * w))
        y2 = min(h, int((d.cy + d.h / 2.0) * h))
        if x2 <= x1 or y2 <= y1:         # kırpık/taşkın bbox
            return None
        return gm.hedef_rengi_bgr(kare[y1:y2, x1:x2])[0]

    def _mono_hedef_mi(self, d) -> bool:
        """Stereo yokken: bu tespit P3 hedef dubası MI (renkten)?

        Yalnız **yeşil ve siyah** için True döner ve normal eşikten daha SIKI
        bir kapsama ister (`MONO_HEDEF_ORANI`). Gerekçe ölçüm:
        6.042 gerçek kenar/engel kutusunda 0,45 eşiğiyle yeşil **%0,000**,
        siyah **%0,000** yanlış pozitif; kırmızı 0,65'te bile %0,53 ⇒
        **kırmızı bu yoldan GEÇMEZ**, stereo ister.

        Kare yoksa/bayatsa False — kör kabul yok (yanlış angajman TS3'te
        100 → 50 puan, iki yanlış → 5).
        """
        kare = self._son_kare
        if kare is None or self._f_norm is None:
            return False
        h, w = kare.shape[:2]
        x1 = max(0, int((d.cx - d.w / 2.0) * w))
        y1 = max(0, int((d.cy - d.h / 2.0) * h))
        x2 = min(w, int((d.cx + d.w / 2.0) * w))
        y2 = min(h, int((d.cy + d.h / 2.0) * h))
        if x2 <= x1 or y2 <= y1:
            return False
        renk, kapsama = gm.hedef_rengi_bgr(kare[y1:y2, x1:x2],
                                           oran=MONO_HEDEF_ORANI)
        return renk in MONO_HEDEF_RENKLERI and kapsama > MONO_HEDEF_ORANI

    def kayit_adimi(self):
        """bbox+sınıf overlay'li, zaman etiketli mp4 karesi yaz (~KAYIT_HZ).

        Şartname: ≥1 Hz, her frame zaman etiketli, tespit çerçeveleri + sınıf
        görünür. Kayıt hatası görevi ASLA durdurmaz — devre dışı kalır, loglanır.
        """
        # Kareyi artık `_kare_tazele` sağlıyor (tek boşaltıcı). Aynı kareyi
        # iki kez yazmayız — eskiden "kuyruk boşsa atla" davranışı vardı,
        # kare numarası onu birebir koruyor.
        if self._son_kare is None or self._kare_no == self._kayit_kare_no:
            return
        self._kayit_kare_no = self._kare_no
        try:
            # 🔴 KOPYA ŞART: aşağıda görüntünün ÜSTÜNE çiziliyor (bbox
            # dikdörtgenleri + yeşil zaman etiketi). numpy dizisi referanstır;
            # kopyalamazsak `_son_kare` de boyanır ve Parkur-3 renk analizi
            # kendi çizdiğimiz turuncu/sarı çerçeveyi okur. Ölçüldü (13.08):
            # kopya 0,05 ms (Jetson ~0,25 ms) = 500 ms bütçenin %0,05'i.
            img = self._son_kare.copy()
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
            # 🔴 İKİ AYRI SAAT: görünen etiket DUVAR saati olmak zorunda
            # (md 4.2 "her frame zaman etiketine sahip"), segment süresi ise
            # MONOTONIC — saat sıçrarsa segment sayacı bozulmasın.
            t_duvar = time.time()
            etiket = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_duvar))
                      + f".{int((t_duvar % 1.0) * 1000):03d}")
            cv2.putText(img, etiket, (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(img, f"{self.durum} gecit={self.gecit_sayisi}"
                             f" NN {self.olculen_fps:.1f}FPS",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 0), 1)
            self._kayit_yaz(img, time.monotonic())
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
        """/girdap/fusion/odom → (x, y, yaw). Karar stack'inin poz kaynağı.

        ⏱️ Varış anı MONOTONIC saatte saklanır (duvar saati DEĞİL — saat kuralı,
        dosya başı): `girdap-saat-gec.service` koşu öncesi saati adımlayabilir ve
        duvar saatiyle ölçülen "bayatlık" o an sıçrardı.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.son_odom = (p.x, p.y, yaw)
        self.son_odom_t = time.monotonic()

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
        # 🔴 Her karede sıfırlanır: temizlenmezse eski hedefler birikir ve
        # tekne çoktan geçmiş bir hedefe nişan almaya devam eder.
        self._hedef_adaylari = []

        for d in self.dubalar:
            # 🔴 BÜYÜK CİSİM SÜZGECİ (2026-08-11) — P3 hedef dubası bizim
            # sınıflarımızdan biri sanılıp yayınlanırsa füzyon `EdgeBuoyMemory`'ye
            # KALICI kenar kaydı yazar (orada unutma YOK) → iki hedef arasında
            # hayalet kapı → P2'de rota bozulur, P3'te angajman kaçar.
            # Hedefler parkurun SONUNDA ama 64 cm 25 m'den 7,7 px eder ⇒ tekne
            # daha P2'nin İÇİNDEYKEN görüyor. Bu yüzden süzgeç parkurdan
            # BAĞIMSIZ, her zaman açık: mod yok, tetik yok, kilitlenme yok.
            mono = getattr(d, "kaynak", "stereo") == "mono"
            if not mono and gm.buyuk_cisim_mi(d.z, d.w, self._f_norm):
                self._tani["buyuk_cisim"] += 1
                # FAZ 2: ATMIYORUZ ARTIK — Parkur-3 hedef adayı olarak
                # ayrı tutuluyor. `/perception/buoys` sözleşmesi DEĞİŞMEDİ:
                # aşağıdaki `continue` aynen duruyor, oraya hâlâ girmiyor.
                self._hedef_adaylari.append(d)
                self.get_logger().warn(
                    f"Tespit SÜZÜLDÜ — stereo {d.z:.1f} m ile bbox genişliği "
                    f"uyuşmuyor: cisim Ø0,30 m'den büyük (P3 hedef dubası?). "
                    "Kenar/engel olarak YAYINLANMADI.",
                    throttle_duration_sec=5.0)
                continue
            if mono and self._mono_hedef_mi(d):
                # 🔴 16.08.2026 — STEREO YOKKEN HEDEF KAYBOLUYORDU (ve engele
                # dönüşüyordu). Mekanizma: `buyuk_cisim_mi` mono'da atlanıyor —
                # haklı olarak, çünkü mono `z` zaten Ø0,30 varsayarak
                # genişlikten türetiliyor ⇒ kıyas DAİRESEL olurdu. Ama sonuç:
                # stereo tavanı (10 m) ötesinde hedef dubası `/perception/buoys`'a
                # NORMAL DUBA olarak giriyordu ⇒ `EdgeBuoyMemory` kalıcı kenar
                # kaydı (unutma YOK) ⇒ hayalet kapı + MPPI'nin `w_obstacle=50`
                # ile KAÇTIĞI engel. Süzgecin kendi gerekçesi *"tekne P2'nin
                # içindeyken görüyor"* tam da o menzilde çalışmıyordu.
                # ÇÖZÜM: menzil yoksa RENK karar versin — ama yalnız renk
                # AYIRDIĞI yerde. 6.042 gerçek kenar/engel kutusunda ölçüldü
                # (16.08, eski+yeni oturum, karanlık/ters ışık dahil):
                #   kapsama eşiği 0,45'te  yeşil %0,000 · siyah %0,000 yanlış pozitif
                #   kırmızı ise 0,65'te bile %0,53 (turuncu kenar dubamızla
                #   karışıyor: RAL 3026 ↔ RAL 2003) ⇒ KIRMIZI STEREO İSTER.
                self._tani["mono_hedef"] += 1
                # 🔴 MENZİLİ HEDEF HİPOTEZİYLE YENİDEN KUR (16.08, ikinci tur).
                # `menzil_coz` mono yedeğini **Ø0,30 varsayarak** kurar; o sayıyı
                # olduğu gibi bırakırsak iki şey birden bozulur:
                #   1) yayınlanan çap `w·z/f` = **her zaman 0,30** (dairesel) ⇒
                #      tüketicinin `cap_makul_mu` bandı (0,40-1,00) bunu ELER,
                #      yani süzmüş ama kurtarmamış oluruz — hedef yine seçilemez.
                #   2) menzil **2,13 kat KISA**: gerçek 10,5 m'deki hedefe
                #      "4,9 m" der ⇒ nişan noktası yarı yolda kurulur.
                # Ayrımı RENK yaptı (ölçüm: yeşil/siyah %0,000 yanlış pozitif);
                # o hâlde menzil de aynı hipotezle, Ø0,64 ile hesaplanmalı.
                # ⚠️ Bu, çapı bilgi taşımayan bir sabite çevirir — tüketicinin
                # boyut kapısı bu adaylar için ETKİSİZDİR, ayrımı renk yapmıştır.
                d.z = gm.mesafe_genislikten(d.w, gm.HEDEF_CAP_M, self._f_norm)
                d.x = gm.yanal_konum(d.z, d.cx, self._f_norm)
                self._hedef_adaylari.append(d)
                self.get_logger().warn(
                    f"Tespit SÜZÜLDÜ — stereo yok, mono {d.z:.1f} m; rengi "
                    "hedef paletinde (yeşil/siyah) ⇒ P3 hedef adayı. "
                    "Kenar/engel olarak YAYINLANMADI.",
                    throttle_duration_sec=5.0)
                continue
            det = Detection2D()
            det.header = arr.header
            # Piksel uzayı = tüketicinin beklediği uzay (BBOX_W/BBOX_H
            # açıklamasına bak) — gerçek kare boyutu DEĞİL.
            bx, by, bw, bh = gm.bbox_piksel(
                d.cx, d.cy, d.w, d.h, self._lb_pay, BBOX_W, BBOX_H)
            det.bbox.center.position.x = bx
            det.bbox.center.position.y = by
            det.bbox.size_x = bw
            det.bbox.size_y = bh
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

        bbox normalize olduğu için ölçek 512/640'tan bağımsız: f_norm=odak_px(1.0).
        Ölçümlerden biri yoksa çelişki iddia edilmez (True döner).

        🔴 2026-08-08: menzil zaten mono'dan geldiyse çapraz kontrol ANLAMSIZ —
        aynı sayıyı kendisiyle karşılaştırır, her zaman "tutarlı" çıkar. O hâlde
        kontrol atlanır; sessiz-onay olmasın diye ayrı sayaçta (mono_menzil)
        zaten görünüyor."""
        if getattr(d, "kaynak", "stereo") == "mono":
            return True
        d_mono = gm.mesafe_genislikten(d.w, gm.DUBA_CAP_M, self._f_norm)
        ok = gm.menzil_tutarli(d.z, d_mono, MENZIL_BAGIL_TOL)
        if not ok:
            # 🔴 2026-08-06: bu sayaç tanımlıydı ve durum_log'da BASILIYORDU ama
            # hiçbir yerde ARTIRILMIYORDU → "sessiz ret" tablosunda menzil
            # çelişkisi kalemi her zaman 0 görünüyordu. Tanılamanın tek amacı
            # sahada (SSH yok) "dubaları görüyorum ama kapı kuramıyorum" hâlinin
            # SEBEBİNİ göstermek; kör bir kalem o tabloyu yanıltıcı yapar.
            self._tani["menzil_celiski"] += 1
            self.get_logger().warn(
                f"Tespit atıldı — stereo {d.z:.1f} m ↔ genişlikten {d_mono:.1f} m "
                "çelişiyor (uzak/kısmi görünen duba?)",
                throttle_duration_sec=5.0)
        return ok

    # ---------- Durum makinesi (ortak) ----------
    def duruma_gec(self, yeni):
        if yeni != self.durum:
            if yeni == "ARAMA":
                self.arama_baslangic = time.monotonic()
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
            self.pass_bitis_t = time.monotonic() + max(GECIS_ZAMAN_KATSAYI * tahmin, 8.0)
        else:
            self.pass_bitis_t = time.monotonic() + tahmin  # odom yok: eski zaman tahmini
        self.get_logger().info(f">> Geçide giriliyor (orta nokta {orta_z:.1f} m)")

    def dongu(self):
        self.tespitleri_oku()
        simdi = time.monotonic()

        # Kare TEK YERDEN tazelenir; kayıt ve Parkur-3 ikisi de bunu okur.
        self._kare_tazele()

        # Dosya-1: görev durumundan BAĞIMSIZ kayıt (şartname ≥1 Hz; görev
        # tamamlansa da karaya alınana dek kayıt sürer)
        if not self._kayit_bozuk and simdi - self._son_kayit_t >= 1.0 / KAYIT_HZ:
            self._son_kayit_t = simdi
            self.kayit_adimi()

        # Parkur-3 hedef yayını — KAYITTAN BAĞIMSIZ (`_kayit_bozuk` olsa da
        # sürer; buradaki hata da kaydı etkilemez).
        self.hedef_adimi(simdi)

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
                    if simdi < self.pass_bitis_t:
                        self.durum_log()
                        return
                    if p is None:
                        # 🔴 2026-08-13: "DOĞRULANAMADI" ile "GEÇMEDİ" aynı şey
                        # değil. Burada doğrulayacak ÖLÇÜT yok — odom kesildi ya
                        # da bayatladı (karar tarafı `pose_timeout_s=1.0` ile
                        # yayını bilerek keser, `fusion_node.py:560-568`).
                        # Eski kod bu hâlde geçidi ELİYORDU; ölçüldü ki odom HİÇ
                        # gelmemişken AYNI geçiş sayılıyordu ⇒ tutarsız ve
                        # doğrudan (G/KD)×10 + ×40 kaybettiren dal.
                        # Burada odomsuz yolun zaten yaptığı şeye düşüyoruz:
                        # zaman tahminine güven. `gecit_cizgi` BİLEREK
                        # sıfırlanmıyor — orta noktası aşağıda `yeni_gecit_mi`
                        # ile tekrar-sayma korumasını besliyor.
                        self.get_logger().warn(
                            "Geçiş penceresi doldu ve odom BAYAT/KESİK — poz "
                            "doğrulaması yapılamadı, zaman tahminiyle SAYILDI. "
                            f"(bayatlık eşiği {ODOM_BAYAT_SN} s) "
                            "Sürekli görülüyorsa /girdap/fusion/odom akışına bak.")
                        # aşağı düş → geçit sayımı
                    else:
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
        mppi_hedef → TF (odom->base_link). Poz yoksa None.

        🔴 BAYAT POZ = POZ YOK (2026-08-13): `son_odom` yalnız mesaj GELDİĞİNDE
        güncelleniyor; karar tarafı yayını kesince (F8.2, `pose_timeout_s=1.0`)
        değer DONAR. Donuk pozu geçerli saymak, poz hiç olmamasından daha kötü
        sonuç veriyordu — ölçüm ve gerekçe `ODOM_BAYAT_SN` tanımında.
        None dönmek kodun ZATEN bildiği "poz yok" yoluna düşürür (zaman tahmini).
        """
        if MOD == "algi_yayin":
            if self.son_odom is None:
                return None
            if (time.monotonic() - self.son_odom_t) > ODOM_BAYAT_SN:
                return None
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

        simdi = time.monotonic()
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
        simdi = time.monotonic()
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
        # 🔴 2026-08-09: `mono_menzil` TETİKLEYİCİYE KATILMAZ — o bir red sebebi
        # değil, tespiti KURTARAN pinhole yedeği. Sayaçlar kümülatif (hiçbir yerde
        # sıfırlanmıyor) olduğu için alarma katılsaydı ilk mono düşüşünden sonra
        # uyarı bir daha susmazdı; sürekli yanan uyarıyı sahada kimse okumaz.
        # Bilgi olarak yine basılıyor: YÜKSEK değeri "stereo suda çalışmıyor"
        # demek ve SSH olmadığı için bunu görebileceğimiz tek kanal bu.
        # 🔴 11.08: "buyuk_cisim" LİSTEDE YOKTU — büyük cisim süzgeci gerçek
        # dubayı elese sahada TEK SATIR bile basılmıyordu (SSH yok, journal tek
        # görünürlük kanalı). 06.08'deki "sessiz ret" hatasının aynısı.
        RED_KALEMLERI = ("dar", "dizili", "arada_duba", "menzil_celiski",
                         "menzil_yok", "buyuk_cisim")
        # 🔴 2026-08-13: tetikleyici TOPLAM değil, son log'dan bu yana ARTIŞ.
        # Sayaçlar kümülatif olduğu için `any(self._tani[a])` bir kez dolunca
        # bir daha sönmüyordu (ölçüldü: 1 geçmiş red → sonraki 5/5 turda alarm).
        # Sürekli yanan uyarıyı sahada kimse okumaz ve GERÇEK arızayı gizler;
        # SSH yok, journal tek görünürlük kanalımız. Toplamlar mesajda YİNE
        # basılıyor (tanı için lazım), yalnız ALARM anlık olaya bağlandı.
        artan = {a: self._tani[a] - self._tani_onceki.get(a, 0)
                 for a in RED_KALEMLERI}
        self._tani_onceki = dict(self._tani)
        if k >= 2 and self.durum == "ARAMA" and any(v > 0 for v in artan.values()):
            # ⚠️ TEK KONUMSAL ARGÜMAN — pazarlıksız. rclpy imzası
            # `log(message, severity, **kwargs)`; printf tarzı ek konumsal
            # argüman TypeError atar, istisna timer callback'inden `rclpy.spin()`e
            # çıkar ve NODE ÖLÜR (09.08.2026, humble 3.3.21'de ölçüldü — üstelik
            # tam da bu uyarının gerektiği anda). Regresyon testi:
            # test/test_log_imzalari.py
            self.get_logger().warn(
                f"kapı kurulamıyor — red sebepleri: dar={self._tani['dar']} "
                f"dizili={self._tani['dizili']} arada_duba={self._tani['arada_duba']} "
                f"menzil_çelişki={self._tani['menzil_celiski']} "
                f"menzil_yok={self._tani['menzil_yok']} "
                f"| bilgi: mono_menzil={self._tani['mono_menzil']} "
                f"(pinhole yedeği; yüksekse stereo suda ölçemiyor)",
                throttle_duration_sec=10.0)


def _sigterm_kur():
    """SIGTERM'i KeyboardInterrupt'a çevir — `finally` çalışsın.

    🔴 ÖLÇÜLDÜ (11.08.2026): Python'un SIGTERM varsayılanı süreci ANINDA öldürür,
    `finally` bloğu ÇALIŞMAZ. rclpy de SIGTERM'e dokunmuyor (yalnız SIGINT'e).
    systemd `stop`/`reboot`/güç kesme SIGTERM gönderir ⇒ `kayit_kapat()` hiç
    çağrılmaz ⇒ mp4'ün **moov atomu yazılmaz** ⇒ dosya OYNATILAMAZ.
    Sonuç: md 4.2 Dosya-1 geçersiz = **5 CEZA PUANI** (md 5.5.4.3.5).
    Ayrıca `dogrudan_surus` modunda motorlar sıfırlanmadan kalır.
    """
    import signal

    def _kapat(signum, frame):          # noqa: ANN001
        raise KeyboardInterrupt         # main'deki except/finally zincirine düşer

    try:
        signal.signal(signal.SIGTERM, _kapat)
    except Exception:
        pass                            # ana iş parçacığı değilse sessiz geç


def main():
    _sigterm_kur()
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
            node.dev.close()      # v2 (cihaz teardown'da çökebilir — yut, kayıt zaten kapandı)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

---
name: sartname-ida-2026
description: "TEKNOFEST 2026 İDA şartnamesi — birinci kaynaktan okunan kritik gereklilikler, puanlama, ceza; PDF ~/ida_sartname_2026.pdf"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 2afb4def-d306-47e8-8bf0-f118690f51af
---

Resmî 2026 İDA şartnamesi (18.05 sürümü, 29 sayfa) 2026-07-10'da birinci kaynaktan okundu. PDF: `/home/eyup/ida_sartname_2026.pdf` (scratchpad'den kopyalandı). Kaynak URL: teknofest.org İDA yarışma sayfası → `cdn.teknofest.org/.../2026_İnsansız_Deniz_Araci_Şartnamesi_TR_18_05_Mh9Vx.pdf`. İlgili proje: [[girdap-decision-entegrasyon]].

## Zorunlu veri teslimi (md 4.2 s.14-15 + ceza md 5.5.4.3.5 s.26)

> Bulma tarifi (Eyüp PDF'te arayıp bulamadı, 2026-07-10): ayrı başlık DEĞİL —
> **madde 4.2 → "Faydalı Yük ve Otonomi" madde imi → "Otonomi amacıyla
> kullanılan aşağıdaki veriler kaydedilecek ve teslim edilecektir", s.14 alt**.
> PDF'te Türkçe karakter araması tutmuyor; "Dosya" veya "mp4" diye ara.

- Karaya alımdan **20 dk içinde USB ile**; eksik **her dosya 5 ceza puanı**.
- **Dosya 1** "Otonomi Sensörleri Veri seti": (a) **işlenmiş kamera**: ≥1 Hz, her frame zaman etiketli, **mp4**, bbox+sınıf görünür → BİZİM İŞ, `KAYIT_AKTIF` kaydedici commit `2282241`; (b) diğer sensörler (LiDAR kümeleme) ayrı mp4 → arkadaşın işi. NOT: "Dosya 1a/1b" adlandırması şartnamede YOK — arkadaşın CLAUDE.md uydurması; Eyüp "1a olmayacakmış" diye duymuştu, isim farkı karışıklığı.
- Dosya 2: telemetri csv (lat/lon, hız, roll/pitch/heading, hız+yön setpoint, header'lı) → arkadaşta hazır. Dosya 3: lokal harita ≥1 Hz → arkadaşta hazır.

## Parkur şartları ve puanlama (md 5.5.2, 5.5.4)

- **Parkur-2 tamamlama: EN AZ 2 duba İKİLİSİ arasından geçiş + son görev noktası** (md 5.5.2.4) → koddaki `MIN_GECIT` 1→2 düzeltildi (`2282241`). Puan **(G2/KD2)×40** oranla artar → şart sonrası geçmeye devam (GOREVDE_DUR=False doğru).
- Maks puanlar: P1=55, P2=100, P3=145, KTR raporu=15. Çarpma cezası puan formülünde (P2: (−30×Ç2)/(KD2+ED2)+30); aynı dubaya 30 sn temas = 2 çarpma. Parkur dışı çıkma cezalı; 40 sn üstü dışarıda = 2 çıkış.
- Parkurlar arası geçiş OTOMATİK algılanacak (dış komut yasak). Yeniden başlama 1 hak, puanlar sıfırlanır + pas geçme 10 ceza.
- Görev noktaları dd.ddddddd formatında dosyayla verilir (USB, hakem çadırı), YKİ'de tanımlanır; görev yükleme İDA'ya güç verildikten sonra.

## Dubalar (md 5.5.2.1 s.17-18) — ALGI İÇİN KRİTİK

- Kenar: armut tip, 30 cm çap, 50 cm yükseklik, **turuncu RAL 2003** ✓ (Eyüp'ün kodu doğru; arkadaşın CLAUDE.md'si "RAL 2008" diyor — YANLIŞ).
- Engel: armut tip 30 cm, **sarı RAL 1026** ✓ (arkadaşınki "RAL 1003" — YANLIŞ).
- **Hedef dubaları (Parkur-3): 640 mm çap × 950 mm, ÜÇ RENK: RAL 9005 (siyah), RAL 3026 (kırmızı), RAL 6037 (yeşil)** — angajman hedefi RENK ile belirlenir, renk bilgisi görev başlamadan İDA'ya yüklenir (başladıktan sonra aktarım YASAK). ⚠️ **Eyüp'ün YOLO modelinde hedef sınıfı YOK** (yalnız kenar/engel) → model yeniden eğitilmeli (3 hedef rengi sınıf olarak) VEYA büyük-duba tespiti + bbox içi HSV renk ayrımı eklenmeli. AÇIK İŞ.
- Sadece armut tip; parkur dışını çevreleyen beyaz sosis dubalar var (karıştırma).

## 🔴 Otonomi Kabiliyeti Videosu = ELEME KAPISI (md 3 + 3.3, s.10-12) — 21.07.2026 17:00

**md 3:** "Teknik Yeterlilik Raporu, Kritik Tasarım Raporu ve Otonomi Kabiliyeti videosu **göndermeyen takımlar yarışmaya katılmaya hak kazanamayacaklardır.**"
**md 3.3:** "Bu aşamayı geçebilen takımlar **final aşamasında yarışmaya hak kazanacaktır.**"
Takvim (Tablo 3, s.9 — 2026-07-12'de TAM tablo yeniden okundu): 21.07.2026 17:00 video son teslim → **27.07.2026 finalist takımlar açıklanır** → **⚠️ YARIŞMA TARİHİ-YERİ: "Ağustos-Eylül 2026" (DSB — henüz belirlenmedi!)** → 30 Eylül-4 Ekim 2026 = TEKNOFEST festivali. **DÜZELTME: önceki "yarışma 30 Eylül" varsayımı YANLIŞ — 30 Eylül festival tarihi; saha yarışı Ağustos'ta bile olabilir. T1 tamponu sanılandan kısa olabilir; tarih açıklanınca plan yeniden sıkılaştırılmalı (mail grubu/KYS takibi şart, md 1.3.3 bildirimler iletişim sorumlusuna gider).**

⇒ **Video geçilmezse P1(55)+P2(100)+P3(145) hepsi SIFIR.** Her şeyden önce gelir.

### md 3.3.1 — videoda gösterilmesi ZORUNLU olanlar (6 madde)
1. İDA ↔ YKİ **kablosuz bağlantı** kurulu; **İDA bilgileri YKİ üzerinde gösterilecek.**
2. **YKİ üzerinden 4 noktalı görev TANIMLANACAK** (dikdörtgen) **ve bu görev İDA'ya GÖNDERİLECEK.**
3. YKİ ya da RC kumandadan **bir komutla görev başlayacak**; 4. noktaya ulaşınca otonom görev tamamlanır. Sonrası **manuel** dönüş serbest.
4. Dönüşte motorlar/güç **güvenlik butonu/anahtarıyla** kesilecek; RC'den komut verilse de motorların dönmediği gösterilecek.
5. İDA'nın **kapakları açılıp su almadığı** gösterilecek.
6. Hepsi **tek gösterimde, kesintisiz.**

### md 3.3.1.1 — video biçimi
- 3 bölmeli ekran: **1)** YKİ ekran görüntüsü · **2)** senkron grafikler: gerçek hız + **hız setpoint**, gerçek heading/yaw + **yaw setpoint**, **thrusterlardan kuvvet isteği** · **3)** dış kamera.
- ≥720p, **2–5 dk**, YouTube (liste dışı), link KYS'ye yüklenir — **linkte sorun = eleme.**
- ⚠ "hareket eksenlerinde **istemsiz** ve hareket yönünden bağımsız dönüş/sürüş" görülürse **BAŞARISIZ** → MPPI kararlılığı doğrudan video kriteri.
- Görüntü/hareket net değilse **BAŞARISIZ**.

### Eyüp'ün video kararı (2026-07-10)
Video **dış kameradan** çekilecek, İDA'nın bakış açısından DEĞİL; ekranda MPPI gibi grafikler gösterilecek.
- ✅ Dış kamera doğru ve zaten **zorunlu** yol: md 4.1 İDA'dan yere görüntü aktarımını YASAKLIYOR, md 3.3.1.1 Ekran-3 zaten "dış kamera görüntüsü" diyor.
- ⚠ **DÜZELTME 1:** "sadece grafik" yetmez — **Ekran-1 = YKİ ekran görüntüsü ZORUNLU** (md 3.3.1.1) ve md 3.3.1(1) "İDA bilgileri YKİ üzerinde gösterilecek" diyor. Üç bölme de şart.
- ⚠ **DÜZELTME 2:** Ekran-2 serbest grafik değil. Şartname üç sinyali SAYIYOR: (a) gerçek hız + **hız setpoint**, (b) gerçek heading/yaw + **yaw setpoint**, (c) **thrusterlardan kuvvet isteği**. MPPI maliyet/rollout eğrisi bunların yerine GEÇMEZ. Ayrıca "İDA'nın hareketleriyle **senkron**" olmalı.

### Videodan ÇIKARIMLAR (2026-07-10)
- **Algı (kamera/LiDAR/duba) videoda HİÇ YOK.** F5.1, sınıf sırası, bearing — hiçbiri videoyu bloke etmiyor. Bunlar yarışma günü (30 Eylül) sorunları.
- Videoyu bloke edenler: **MAVROS köprüsü** (arm/mod/cmd_vel/kill), **mission_manager** (4 nokta), **planning_node bypass + MPPI kararlılığı**, **telemetri** (Ekran-2 grafikleri).
### YKİ = QGroundControl (Eyüp teyit etti, 2026-07-10)

- ✅ **`gcs_url: ""` ENDİŞESİ GERİ ALINDI.** QGC, Pixhawk'a **MicoAir LR868 @ 868 MHz** telemetri radyosuyla doğrudan bağlanıyor (⚠️ eski notlardaki "RFD868x" BAYAT — bkz. [[haberlesme-frekans-uyum]]); MAVROS ise Jetson↔Pixhawk USB üzerinden (`fcu_url: serial:///dev/ttyACM0:57600`). İki ayrı MAVLink kanalı, birlikte yaşarlar. `gcs_url=""` **doğru**. (Önceki "videonun 1. şartını kapatıyor" teşhisi YANLIŞTI.)
- ⚠ QGC **WiFi ile bağlanamaz** — md 4.1: 2.4-2.8 ve 5.15-5.85 GHz YASAK, dahili WiFi kapalı. Tek yol 868 MHz radyo.
- ✅ Ekran-1 = QGC ekran kaydı. md 3.3.1(1) "İDA bilgileri YKİ'de gösterilecek" → QGC FC telemetrisini gösteriyor.

### 🔴 ASIL SORUN: görev yükleme yolu yok (hem VİDEO hem YARIŞMA)

QGC'den yüklenen waypoint'ler **Pixhawk'ın görev hafızasına** yazılır. Ama otonomi yığını (`planning_node`) hedefleri `/girdap/mission/waypoints`'ten alıyor ve onu `mission_manager_node` **araç üstündeki `config/*.yaml` dosyasından** üretiyor. FC'den görev okuma kodu **hiç yok** (`WaypointList`/`mission/pull` araması sıfır sonuç). ⇒ **QGC'den gönderilen görev, İDA'nın icra ettiği görev DEĞİL.**

Bu sadece video sorunu değil:
- md 3.3.1(2): "YKİ üzerinden 4 noktalı görev tanımlanacak ve bu görev İDA'ya **gönderilecektir**."
- md 5.5.2.2 (s.19): "Her bir takım yarışma sırası gelmeden önce görev noktalarını **YKİ'lerinde tanımlamış olacaktır**. **Görev yükleme**, yarışma alanına girişe müteakip İDA'ya **güç verildikten sonra** yapılacaktır."

⇒ **Her iki etkinlik de YKİ'den yükleme istiyor.** YAML dosyası ikisini de karşılamıyor.

**Çözüm (tek mekanizma, ikisini de kapatır):** `mission_manager_node`'a `mission_source: {file, fc}` parametresi ekle; `fc` modunda MAVROS waypoint plugin'inden (`/mavros/mission/waypoints`, `mavros_msgs/WaypointList`) görevi oku, lat/lon→ENU çevir (haversine kodu zaten var), `/girdap/mission/waypoints` olarak yayınla. QGC → Pixhawk → MAVROS → mission_manager → MPPI. `/girdap/control/thrust` de canlı kalır (Ekran-2c).

**Alternatif (zayıf):** FC'yi AUTO moda al, ArduRover görevi kendi uçursun. Şartnameye uyar ama takımın MPPI yığını hiç kullanılmaz ve Ekran-2 sinyallerinin kaynağı değişir.

## Yarışma günü operasyon kuralları (md 5.2/5.3/5.5.2.2/5.5.3.1 — 2026-07-12 birinci kaynaktan okundu)

- **Teknik kontrol (5.2):** finale kalan her araç 4.1/4.2/4.3 isterlerine göre denetlenir + **masaüstü motor testi**; geçemeyen YARIŞAMAZ. Kalemler: kırmızı fiziksel güç kesme, uzaktan güç kesme (sinyal kesmek yetmez, GÜÇ kesilecek), çeki demiri, **takım ismi 3 yerde (iskele/sancak/kıç, boyut ≥ gövdenin yarısı)**, pervane korumalı/nozullu+köreltilmiş, batarya bölümü sızdırmaz, sensörler sabit, kablolar gergin değil. Anons sonrası 3 dk içinde gelinmezse 5 ceza.
- **Sunum (5.3):** teknik kontrol sonrası **maks 15 dk prototip sunumu** (Hakem Kurulu + en az 2 üye; 4.1-4.3 isterleri + süreç/tasarım/özgünlük) → sunum dosyası HAZIRLANACAK İŞ. "En Özgün Yazılım" değerlendirmesi için rapor/akış diyagramı önerilir (md 6).
- **Alan/rol kısıtları (5.5.3.1):** alanda maks 6 üye; İDA'yı denize bırakan ≤4 kişi Yarışma Çadırına GİREMEZ; çadırda maks 2 kişi ve bunlar yarış boyunca **alanı ve İDA'yı GÖREMEZ**; dışarıdan bilgilendirme = yarışma dışı riski → rol dağılımı önceden yazılmalı.
- **Görev/komut akışı:** görev yükleme kablolu VEYA kablosuz olabilir (güç verildikten sonra, karada/denizde); **görev BAŞLAT komutu kablolu VERİLEMEZ** (bizim QGC GUIDED tetiği RFD üzerinden = uyumlu ✓); YKİ'de SADECE YKİ arayüzü açık olacak, "kod başlatma" yapılmayacak; İDA hareket edince acil motor kesme dışında komut YASAK. Görev noktaları dosyası hakem çadırından TAKIMIN USB'siyle alınır (dd.ddddddd), sıra gelmeden YKİ'de tanımlı olacak.
- **Frekans:** hakem test+yarış için frekans tahsis eder; yarışma öncesi belirlenen kanala geçilmiş olmalı (geçilmemişse harcanan süre takımındır) → **MicoAir LR868 (868 MHz, yasal)** kanal ayarı prova edilecek. Yarış halindeki takım varken modül çalıştırmak 150 ceza. **RC kumanda bandı ayrı iş — yasal banda çekilecek** ([[haberlesme-frekans-uyum]]).
- **Haklar/bonus:** yeniden başlama 1 hak (puanlar sıfırlanır, süre DURMAZ); pas geçme 1 hak (10 ceza, sıra en sona); **ilk yarışmaya gönüllü olana 15 bonus** (diğer herkes pas isterse). Yarışma süresi 20 dk; P3 sonrası dönüş süreye dahil değil.
- **🎯 Ödül sıralaması şartı: EN AZ P1 + P2 TAMAMLANMALI (s.24)** — P1+P2 odağının şartname teyidi; P3 (145p) ödül kapısı için şart değil.
- İHA'sız yarışılabilir; hedef RENK bilgisi İHA'sız takıma Yarışma Alanına girişte VERİLİR ve görev başlamadan İDA'ya aktarılabilir (sonra aktarım yasak) → P3 açılırsa renk parametre olarak yüklenir, İHA zorunlu değil (İHA yalnız +45 bonus, başarılı angajman koşuluyla).

## Diğer
- İDA boyut 75×75×30 min – 150×200×200 max (katamaran 105×75 ✓ ama min genişlikte tam sınırda), ≤50 kg, Deniz Durumu-2, kırmızı fiziksel güç kesme + uzaktan güç kesme şart. 2.4-2.8/5.15-5.85 GHz yasak, dahili WiFi kapalı, YKİ'de işleme yasak, görüntü aktarımı yasak. Yarışma süresi 20 dk.
- **⚠️ Güç kesme (md 4.2): "motorlara giden SİNYAL akışını kesmek YETERLİ DEĞİL, GÜCÜN kesilmesi ŞART."** → yazılım disarm videoyu karşılar (motor dönmez) ama yarışma emniyeti + teknik kontrol için FİZİKSEL güç kesen röle gerekir (mekanik/FC). Bkz. [[haberlesme-frekans-uyum]].

## 2026-07-14 birinci-kaynak yeniden okuma (V1.2, Jetson)
PDF artık Jetson'da: `~/Downloads/2026_İnsansız_Deniz_Araci_Şartnamesi_TR_18_05_Mh9Vx.pdf` (CDN'den indirildi, 29 sayfa). Tüm şartname yeniden okundu; memory birebir tuttu. Keskinleşen 3 nokta: (1) Ekran-2b metinde açıkça "heading/yaw **AÇISI** isteği (setpoint)" → F-V.1 doğrulandı; (2) "Görev/Hareket **Başlat komutu KABLOLU verilmeyecek**" (md 5.5.3.1) → AUTO/GUIDED tetiği telemetri radyosundan olmalı, USB/SSH'den değil; (3) güç-kesme=sinyal-kesme değil (yukarı). Konum web'de Şanlıurfa, İDA saha tarihi hâlâ DSB.

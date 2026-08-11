# SAHA YAPILACAKLAR — masa başında kapatılamayan işler

> Tek kaynak. Her maddenin **kapatma ölçütü** var; ölçüt sağlanmadan "yapıldı"
> denmez. İş bitince üstü çizilir (silinmez), tarih + commit eklenir.
>
> Neden ayrı dosya: 11-12.08'de kapatılan bulguların çoğu kod tarafındaydı ama
> **kökleri sahada**. Kaptanın 14 oturumluk analizinin en sert cümlesi buydu:
> araç **hiçbir oturumda ARM edilmedi** (PAR-03) — yani yazılım ne kadar
> düzelirse düzelsin, o kapı açılmadan hiçbir şey doğrulanamaz.

---

## A · BU AKŞAM — okul bahçesi (otopark, araç yok, 30-60 dk)

Sıra ÖNEMLİ: pusula düzelmeden arm denemesi anlamsız, arm olmadan da geri kalanı
ölçülemez.

### A1 · Pusula kalibrasyonu — Large Vehicle MagCal

⚠️ **12.08 DÜZELTMESİ (kaptanın `girdap-durum` §0.41/§0.42 kaydından):** pusula
**zaten kalibre edildi** (11.08 akşamı, kapalı alanda). Ölçüm: `PreArm: Compass
not calibrated` 15:07:21'de kesildi, `EKF3 MAG0 initial yaw alignment complete`,
`GPS and AHRS differ` **0 kez**, `/mavros/state` → `mode: GUIDED, guided: true`.

Yani bu madde artık *"kalibrasyon yok"* değil, **"kalibrasyon kapalı alanda
yapıldı, kalitesi şüpheli"**. Bugün force ile arm edilince görülen `Kotu AHRS`
+ failsafe bununla tutarlı: kalibrasyon geçerli sayılıyor ama ofsetler
atölyenin çelik/elektrik alanını taşıyor.

⇒ Yapılacak: **açık alanda YENİDEN** kalibrasyon (üzerine yazar).

- Mission Planner → Setup → Mandatory Hardware → Compass → **Large Vehicle MagCal**
- Teknenin gerçek pruva yönünü (derece) gir — **tekneyi döndürmek gerekmez**,
  yöntemin tamamı bunun için seçildi (suya indirmeden yapılabilsin diye).
- Pruvayı otoparkın boş tarafına çevir; beton, araç, direk, trafo yakınında olma.

**Kapatma ölçütü:** `COMPASS_OFS_X/Y/Z` yazıldı ve kaydedildi; Mission Planner
HUD'da heading gerçek yönle ±5° içinde. Değerleri bana söyle, param dökümüne
işlerim. Ayrıca **eski ofsetlerle karşılaştıracağız** — fark büyükse kapalı
alan kalibrasyonunun gerçekten bozuk olduğu kanıtlanmış olur.

### A2 · 🔴 SD KART / LOGLAMA — arm denemesinden ÖNCE

**12.08 gecesi CANLI ölçüldü, PAR-03'ün somut cevabı:** Pixhawk bağlıyken
MAVROS log'unda 5 saniyede bir şu satır düşüyor:

```
FCU: PreArm: Logging failed
```

ArduPilot SD karta yazamıyor → pre-arm reddi. **Pusula ne kadar iyi kalibre
edilirse edilsin araç ARM OLMAZ.** Bu yüzden bu madde A1'in de önüne geçti.

Yan bulgu: bu mesajın çıkması `ARMING_CHECK`'in artık **0 olmadığının kanıtı**
— 0 olsaydı pre-arm hiç koşmaz, mesaj da çıkmazdı. Yani bizim param yazımımız
tuttu, kaptanın §0.41③ endişesi kapandı.

**Yapılacak:** SD kartı çıkar → kart var mı, doluysa temizle, bozuksa FAT32
formatla ya da yenisiyle değiştir → FC'yi yeniden başlat.

**Kapatma ölçütü:** `PreArm: Logging failed` mesajı KESİLDİ (MAVROS log'unda
ya da Mission Planner Messages'ta bir daha görünmüyor).

⚠ `ARMING_CHECK`'ten loglama bitini düşürerek susturma — o, arızayı çözmez
gizler ve şartname çıktısı olan uçuş log'unu da kaybederiz.

### A3 · ARM denemesi — `force` KULLANMADAN  🔴 PAR-03

Bu, listenin **en kritik maddesi**. 14 oturumda 41.524 `/mavros/state` mesajının
hiçbirinde `armed=true` yok; 11.08'de force ile arm edildiğinde `Kotu AHRS`
çıkmış ve failsafe gelmişti.

- Mission Planner → ARM (force **yok**).
- Reddedilirse **Messages sekmesindeki ret sebebini birebir yaz** — kısaltma.
- 🔴 `ARMING_CHECK` bizim son param dökümümüzde **1**, ama kaptanın 11.08
  ölçümünde FC'de **0**'dı ve *"tüm ön-kontroller kapalı, tekne bozuk
  kestirimle arm oluyor, her şey yeşil görünüyor"* diye kaydetmiş (§0.41).
  Mission Planner'da FC'deki GERÇEK değerin 1 olduğunu doğrula — 0 ise bu
  test hiçbir şey kanıtlamaz.
- Artık `/mavros/statustext/recv` rosbag'e kaydediliyor, yani sebep bu kez
  kaybolmayacak (12.08'de eklendi).

**Kapatma ölçütü:** ya arm başarılı, ya da ret sebebi metni elimizde.
"Olmadı" tek başına kapatmaz.

### A4 · LiDAR yaw kalibrasyonu

Önce **mekanik**: LiDAR kapağa değil, kapaktan bağımsız RİJİT bir yatağa
oturmalı. 11.08'de kapak yamuk takıldığı için 34°'lik sahte bir eğiklik ölçtük;
kapak her açılışta değişirse kalibrasyon tekrarlanamaz, yani ölçmenin anlamı
kalmaz.

- Duba tam pruva hattında, **10 m**.
- `ros2 topic echo /perception/obstacle_map` → `orientation.z` ≈ 0,15-0,25 olan
  cluster (duba yarıçapı) seçilir.
- Sapma = `atan2(y, x)`; sıfır olmalı.

**Kapatma ölçütü:** duba `x`/`y` değerleri elimde; sapma hesaplandı ve
`hardware.yaml` `tf`'ye işlendi.

---

## B · İLK SU TESTİ — arm çalıştıktan sonra

### B1 · Yeni teşhislerin YKİ'de göründüğünü doğrula

12.08'de eklenen teşhisler ROS log'unda değil **Mission Planner Messages**
sekmesinde görünmeli. Görünmüyorsa kanal kopuktur ve sahada hiçbir işe yaramaz.

- MAVROS'u kapalı tutup düğümü başlat → 60 s içinde `GIRDAP BOOT TAKILDI MAVROS-YOK`
- MAVROS açık, arm yok → 30 s içinde `GIRDAP BEKLEMEDE TAKILDI ARM-YOK`

**Kapatma ölçütü:** iki metin de Messages sekmesinde ERROR seviyesinde görüldü.

### B2 · `inhibit_reason` gerçek koşuda okunabiliyor mu

`ros2 topic echo /girdap/control/inhibit_reason` — tekne kıpırdamıyorsa sebep
burada yazmalı (`FSM-DISI(...)`, `KONTROLCU-HAZIR-DEGIL`, `POZ-YOK`, ...).

**Kapatma ölçütü:** en az bir kilit durumunda doğru sebep gözlendi.

### B3 · Sürüklenme metriği — gerçek donanımda  🔴 KAR-07

Sentetikte ölçüldü (RTK sigmasında 3,84 m/dk, eşik 5). Gerçek GPS'te
doğrulanmadı ve asıl tehlike **bildirilen sigma ile gerçeğin ayrışması**
(94× fark).

- Tekne **hareketsiz**, GPS fix'i oturmuş, 2 dakika `/girdap/fusion/odom` kaydı.
- Ardışık poz farklarının toplamı **< 5 m/dakika** olmalı.

**Kapatma ölçütü:** ölçülen m/dk değeri yazıldı; eşiğin üstündeyse
`gps_sigma_by_status` tablosu gerçek fix tipiyle karşılaştırıldı.

### B4 · Donma sebebi: disk G/Ç mi, termal mi  🟠 KAR-09

Alet hazır — `rosbag_kaydet.sh` artık `tegrastats`'ı kendiliğinden yanında
kaydediyor. Yapılacak tek şey uzun bir koşu.

- En az 20 dakika kayıt (ham LiDAR **kapalı**, varsayılan liste).
- Sonra `tegrastats.txt`'te donma anlarındaki CPU/GPU/sıcaklık bakılır.

**Kapatma ölçütü:** donma yaşandıysa sebebi ayrıldı; yaşanmadıysa "20 dk'da
kesinti yok" kaydı düşüldü (ham LiDAR çıkarmanın etkisi de böyle ölçülür).

### B5 · İlk kapıyı gerçekten görüyor muyuz

`/girdap/planning/gate` — kapı ortası. `obstacle_margin` üst sınırının dayanağı
zayıf (varsayılan geçit ~1,35 m açıklık, şartnamede karşılığı yok); gerçek geçit
daha darsa 1,0 m payı geçidi kapatabilir.

**Kapatma ölçütü:** kapı görüldüğünde topic doluyor ve orta nokta iki dubanın
arasında.

### B6 · Kamera yaw'ını yeniden doğrula

+2,38° (0,0415 rad) iskeleye sapma ölçüldü ve `hardware.yaml`'a işlendi — ama
LiDAR kapağı söküldüğü için montaj değişti.

**Kapatma ölçütü:** 10 m'deki duba görüntünün yatay ortasında (±1°) ya da yeni
sapma ölçülüp yazıldı.

---

## C · ATÖLYE / DONANIM — masa başı değil ama saha da değil

### C1 · Kontaktör (Yapılacaklar #1)

- 50 A yeterli mi — BMS deşarj sınırı **60 A**.
- Bobin sürücüsü 3,3 V → 12 V.
- NO/NC davranışı: **enerji kesilince motor gücü KESİLMELİ** (md 4.2 uzaktan güç
  kesme).
- `RELAY1_FUNCTION` hâlâ **0** — kontaktör bağlanınca ayarlanacak.

**Kapatma ölçütü:** YKİ'den `DO_SET_RELAY` ile motor gücü kesildi ve geri verildi.

### C2 · DALY BMS → UART

Donanımcılar TELEM3'e takacak = **`SERIAL5`**. TX/RX çapraz, GND ortak, VCC yok.

- Bağlandıktan sonra `SERIAL5_PROTOCOL` + `BATT_MONITOR` **isimli açılır
  menüden** seçilecek (ham sayı yazma).
- Şu an `BATT_MONITOR=3` (yalnız voltaj) — PM06 yalnız regülatörden dönen akımı
  okuyabildiği için akım kanalı ölü olduğu **kontrollü yük deneyiyle kanıtlandı**.

**Kapatma ölçütü:** Mission Planner'da gerçek akım okunuyor; param dökümü
güncellendi.

### C3 · LiDAR düzeltmeleri REPOYA  🔴 ALG-05

`MID360_config.json` IP'leri ve `xfer_format=0` şu an yalnız
`~/livox_ws/install/` altında yaşıyor — `src/` kopyaları düzeltilmedi.
**Bir `colcon build` ikisini de geri alır ve LiDAR yine sessizce ölür.**

**Kapatma ölçütü:** düzeltmeler `src/` altında, commit'li; temiz build'den sonra
LiDAR veri veriyor.

---

## D · TEDARİK — süre gerektirir, bugün başlat

| Ne | Neden | Aciliyet |
|---|---|---|
| RC seti **868/915 MHz** | Mevcut RadioLink R9DS **2,4 GHz = şartname ihlali**, 55 ceza puanı | 🔴 tedarik süresi var |
| CR1225 pil | Pixhawk RTC pilsiz → her açılışta saat sıfırdan | 🟡 |

---

## Sahada BENDEN isteyeceklerin

- Ölçüm sayılarını söyle, param dökümüne ve `docs/olcum_formu.md`'ye ben işlerim.
- Pre-arm ret metnini birebir ilet — sebebe göre hangi parametreye bakılacağını
  söylerim.
- Bir şey beklenmedik davranırsa `journalctl -u girdap-karar -n 200` çıktısını
  at; yeni teşhis satırları oradan da okunur.

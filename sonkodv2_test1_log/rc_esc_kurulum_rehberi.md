# RC Kalibrasyonu + ESC/Motor Kurulum Rehberi (QGroundControl)

2026-07-16/17 gerçek donanım testinde bu adımlarda ciddi karışıklık
yaşandı (RCMAP yanlış ayarlandı, ESC fonksiyonları karıştırıldı, motorlar
uzun süre dönmedi). Bir dahaki kuruluma hız kazandırmak için net, sıralı
bir referans.

## 1. RC Kalibrasyonu (QGC → Vehicle Setup → Radio)

1. Pixhawk'ı QGC'ye bağla (USB-C direkt ya da telemetri radyosu).
2. **Radio** sekmesine gir, **Calibrate** butonuna bas.
3. Sihirbaz sırayla ister: tüm switch'leri nötr al → throttle'ı uç uca
   hareket ettir → roll → pitch → yaw → tüm aux switch'ler (kill,
   manuel-override kanalları dahil).
4. **Throttle ters çalışıyorsa:** QGC'nin Radio ekranında kanal bazlı
   "Reverse" kutusu YOK (PX4'ten farklı) — ArduPilot'ta bu **parametre**
   ile yapılır: **Parameters → `RC3_REVERSED` → 0'dan 1'e**. Throttle
   dışındaki kanallar için `RC1_REVERSED` (roll), `RC2_REVERSED` (pitch),
   `RC4_REVERSED` (yaw) — kanal numarasına göre `RCx_REVERSED`.

## 2. RCMAP — KARIŞTIRMA

`RCMAP_ROLL`/`RCMAP_PITCH`/`RCMAP_THROTTLE`/`RCMAP_YAW` parametreleri,
**hangi fiziksel RC kanalının hangi kontrol eksenine karşılık geldiğini**
söyler — motor/ESC çıkışıyla HİÇ ilgisi yok. Standart Mode 2 verici için
varsayılan (ve doğru) değerler:

| Parametre | Değer |
|---|---|
| `RCMAP_ROLL` | 1 |
| `RCMAP_PITCH` | 2 |
| `RCMAP_THROTTLE` | 3 |
| `RCMAP_YAW` | 4 |

Bunları asla "hepsini aynı kanala" ayarlama — böyle yapılırsa (2026-07-17
canlı testinde tam bu hata yapıldı) tüm eksenler tek kanalı okur, throttle
oynatınca roll/pitch/yaw da değişiyormuş gibi görünür.

## 3. İki ESC'i Motor Çıkışlarına Bağlama

**Fiziksel bağlantı:** ESC 1 (sol motor) → Pixhawk **MAIN OUT 1**, ESC 2
(sağ motor) → Pixhawk **MAIN OUT 3**. Bu numaralandırma projenin kendi
kod varsayımıyla (`hardware.yaml`: `telemetry.fc_thrust_left_ch: 1`,
`fc_thrust_right_ch: 3`) birebir eşleşiyor — farklı pinlere bağlarsan
telemetri yanlış motoru okur.

**Nihai/otonom kontrol için (yarışma/gerçek navigasyon):**
- `Parameters → SERVO1_FUNCTION` → **73 (Throttle Left)**
- `Parameters → SERVO3_FUNCTION` → **74 (Throttle Right)**
- Bu ayarla ArduPilot throttle+yaw'ı **kendisi otomatik** diferansiyel
  olarak iki motora karıştırır (Skid Steering) — GUIDED/AUTO modda
  `decision_node`/MPPI'nin ürettiği hız komutları gerçekten motorlara
  ulaşır.

**Bench test (pervanesiz, hangi ESC'in hangi kanala/yöne tepki verdiğini
doğrulamak) için — GEÇİCİ:**
- `SERVO1_FUNCTION` → **52 (RCIN2)** — çıkış 1 doğrudan kanal 2'yi yansıtır
- `SERVO3_FUNCTION` → **53 (RCIN3)** — çıkış 3 doğrudan kanal 3'ü yansıtır
- ⚠ **Bu, hangi uçuş modunda olursa olsun aktif kalır** — GUIDED/AUTO'da
  bile otopilot çıktısını YOK SAYAR, sadece ham RC'yi basar. Bench testini
  bitirince **MUTLAKA** `73`/`74`'e geri al, yoksa otonom mod motorları
  hiç süremez (bu oturumda tam bu unutulup ilerideki testte fark edildi).

## 4. Motorlar Dönmüyor — Kontrol Sırası

1. **Safety Switch** — Pixhawk üzerindeki küçük buton, kırmızı/beyaz LED
   yanıp sönüyor olmalı. **Basılı tut** LED sabitlenene kadar (~1-2s).
   Bu yapılmadan PWM çıkışları tamamen kilitli — en sık unutulan adım.
2. **Arm durumu** — QGC'de disarm (kırmızı) mı? Arm et.
3. **ESC güç** — Pixhawk'ın kendi gücünden AYRI, ESC/motor bataryası
   bağlı ve açık mı? İlk güçte ESC'ler bir "beep" dizisi çıkarmalı.
4. **Parametre gerçekten kaydedildi mi?** — `SERVO1/3_FUNCTION` değerini
   Parameters ekranından tekrar oku, 52/53 ya da 73/74 GERÇEKTEN yazılı mı
   (bazen Enter'a basılmadan/başka alana tıklamadan değer commit olmaz).

## 5. Frame/Motors Sekmesi

Motors sekmesinde "Unable to determine motor count" uyarısı çıkarsa,
önce **Frame** sekmesinde araç tipi (Rover, mümkünse "Boat"/"Skid
Steering") seçili olduğundan emin ol. ArduPilot'ta deniz aracı için AYRI
bir "Boat" firmware'i YOKTUR — **Rover (ArduRover) firmware'i doğru
seçimdir**, deniz aracı olması bunu değiştirmez; farklılaşma yalnız
Frame Class seçiminde.

## 6. RTK GPS (Base + Rover)

- **Rover** (teknedeki GPS) → doğrudan Pixhawk GPS1 portuna bağlı, hiç
  dokunulmaz.
- **Base** (ayrı, elde tutulan ünite) → USB ile **QGC'nin çalıştığı
  bilgisayara** bağlanır (Rover'a değil). QGC'de RTK GPS ayarına girip
  **Survey-in** başlatılır (birkaç dakika, Base'i hareket ettirme).
- Düzeltme verisi QGC'nin araca olan **aktif MAVLink bağlantısı**
  üzerinden akar. **⚠ Kritik:** bu ekipte Base'in kendi ayrı/adanmış bir
  radyosu YOK (2026-07-17'de doğrulandı) — yani RTK, paylaşılan
  telemetri hattının sağlıklı olmasına bağımlı. Hat bozulursa (bkz. link
  bekçisi notu) RTK de gider.

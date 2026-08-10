# Jetson saatini GPS'ten kurmak — araştırma ve plan

**Tarih:** 2026-08-09 · **Alan:** Alt Alan B (FC/navigasyon)
**Durum:** ✅ **KAPANDI — 2026-08-11.** Kod yazıldı · Jetson'da kuruldu ·
emniyet yolu ölçüldü ve geçti · **GPS'li mutlu yol gerçek donanımda geçti**

## ✅ MUTLU YOL — 2026-08-11 02:44, GEÇTİ

Tekne beslenmiş, FC bağlı, GPS fix'li. Jetson boot'ta **1970-01-01**'den geldi
(RTC pilsiz — aşağıya bak), yani düzeltme gerçek ve azami büyüklükte oldu:

| Ölçüt | Beklenen | Ölçülen | |
|---|---|---|---|
| Ayrıştırıcı yolu | bağımsız (pymavlink yok) | `pymavlink yok → bagimsiz ayristirici` | ✅ |
| SYSTEM_TIME akıyor mu | istek gerekmeden | `GPS saati alindi: 2026-08-10T23:39:42+00:00` | ✅ |
| Saat kuruldu mu | evet | `SAAT KURULDU (duzeltme +1786404608.4 s = 56,6 yıl)` | ✅ |
| GPS saati **doğru** mu | senkron laptopla uyumlu | **27 s fark** (= ölçüm/kontrol arası geçen süre) | ✅ |
| RTC | yazıldı | `RTC guncellendi (hwclock --systohc)` | ✅ |
| `timedatectl` | `synchronized: yes` | **`yes`** (iki hata düzeltildikten SONRA — aşağıda) | ✅ |
| Eyüp'ün ölçütü | `True` | `saat_guvenilir_mi -> (True, 'cekirdek senkron')` | ✅ |
| İkinci koşu (saat zaten doğru) | dokunmaz | `fark 2.0 s toleransinin altinda — saate DOKUNULMADI`, GPS farkı **0.0 s** | ✅ |

Son satır bağımsız bir çapraz doğrulama: 8 dakika sonra FC'nin GPS saatiyle
sistem saati arasındaki fark **0.0 s** — hem çözücü hem kurulum doğru.

### 🔴 Mutlu yol iki gerçek hata ortaya çıkardı (emniyet yolu göstermemişti)

**Hata 1 — `STA_UNSYNC` bitini temizlemek YETMİYOR.** İlk koşuda saat **doğru**
olduğu hâlde `timedatectl` `synchronized: no` demeye devam etti. Ölçüm:
`status=0x0040`, `maxerror=16000000 us` = **tam `NTP_PHASE_LIMIT`**.
`clock_settime` çekirdeğin `maxerror`'unu azamiye çıkarıyor; sınırın üstünde
kaldığı sürece `second_overflow()` bayrağı **her saniye geri koyuyor.**
Sonucu önemli: teslim dosyaları saat doğruyken `_SAAT-GUVENILMEZ` damgası
alacaktı — md 4.2'de kaçınmaya çalıştığımız yanlış-negatifin tam kendisi.
Düzeltme: `ADJ_STATUS | ADJ_MAXERROR | ADJ_ESTERROR` birlikte yazılıyor,
`maxerror = 100 000 us`. **Sıfır yazılmadı** — yalan olurdu: SYSTEM_TIME 57600
baud seri hattan geliyor, gecikme onlarca ms. Yan fayda: 500 µs/s büyümeyle
(16 000 000 − 100 000)/500 = **8,8 saat** senkron kalır.

**Hata 2 — sıra yanlıştı.** Temizleme `clock_settime` ile `hwclock` **arasında**
yapılıyordu; ikisi de çekirdeğin saat durumuna dokunuyor. Artık **en sonda**.

Ayrıca temizleme artık **kendini doğruluyor** (`adjtimex` geri okunup
`STA_UNSYNC`'in gerçekten düştüğü kontrol ediliyor) — "yazdım, sonucuna
bakmadım" hatasını bir daha yapmamak için.

### 🔴 RTC PİLSİZ — kesinleşti

Güç kesilince Jetson `1970-01-01`'e döndü (`RTC time: 1970-01-01`). Bir gün
önce RTC'nin doğru görünmesi, gücün hiç kesilmemiş olmasındanmış. Sonuç:
**her açılışta GPS fix'i beklemek zorunlu.** CR1225 (şarj edilemez) tedarik
listesinde kalıyor; takılırsa bu bekleme kalkar, kalkmazsa sistem yine çalışır
(emniyet yolu bunu zaten karşılıyor).

## ✅ EMNİYET YOLU TESTİ — 2026-08-09/10, GEÇTİ

**Neden bu test mutlu yoldan önemli:** emniyet yolu bozuksa **yığın hiç
başlamaz** ve bu ancak yarışma sabahı fark edilir. FC kapalıyken (GPS yok)
tam o durum taklit edildi: `sudo systemctl start girdap-saat`.

| Ölçüt | Beklenen | Ölçülen | |
|---|---|---|---|
| Süre sınırlı mı | ≈45 s | **`real 0m45.127s`** | ✅ |
| Servis durumu | `failed` DEĞİL | **`active (exited)`**, `status=2` | ✅ |
| Düzgün pes | açık mesaj | `SAAT KURULMADI (kod 2) — teslimler saat_guvenilir=false ile damgalanmali` | ✅ |
| systemd sonucu | Finished | `Finished GIRDAP IDA — sistem saatini…` | ✅ |
| Karar yığını | ayakta | `active`, node'lar koşuyor | ✅ |
| Sistem saati | DEĞİŞMEMİŞ | `Sat Aug 8 22:28` (öncesiyle aynı) | ✅ |

`SuccessExitStatus=0 1 2 3 4` fiilen çalıştı: çıkış kodu 2 olduğu hâlde unit
`failed` olmadı → `Wants=` zinciri kırılmadı → **GPS gelmese de görev başlar.**

Log ayrıca bağımsız yolun devreye girdiğini kanıtladı:
`pymavlink yok → bagimsiz ayristirici kullaniliyor (SR2_EXTRA3=10, SYSTEM_TIME
kendiliginden akiyor; istek gerekmez)`.

🔎 Yan gözlem: MAVROS portu açık tutarken `pyserial` aynı portu **açabildi**
(hata yok). Açılışta çakışma zaten olmaz (`Before=girdap-karar` → saat servisi
MAVROS'tan önce koşup portu bırakıyor), ama elle test ederken iki okuyucunun
birbirinden bayt kaçırabileceği akılda tutulmalı.

✅ **Bu doğrulama 2026-08-11'de yapıldı ve geçti** — yukarıdaki "MUTLU YOL"
bölümüne bak. Bu maddede açık iş kalmadı.

⚠️ **Yarışma sabahı tek kontrol:** yığını başlatmadan önce
`timedatectl | grep synchronized` → `yes` görülmeli. `no` ise
`sudo systemctl restart girdap-saat` (FC beslenmiş olmalı). 8,8 saatlik pencere
yüzünden Jetson sabah açılıp akşam koşuyorsa **tekrar** koşturmak gerekir.

## ✅ Ne yazıldı (2026-08-09)

| Dosya | Ne yapar |
|---|---|
| `scripts/girdap_saat_kur.py` | pymavlink ile seri porttan `SYSTEM_TIME` **ister** (MAVROS'a bağlı değil), makul aralık kontrolünden geçirir, `clock_settime` ile saati kurar, `STA_UNSYNC`'i temizler, `hwclock --systohc` ile RTC'ye yazar |
| `scripts/girdap-saat.service` | Açılışta oneshot, `Before=girdap-karar.service` · `SuccessExitStatus=0 1 2 3 4` (kurulamazsa yığın **yine** başlar) |
| `scripts/girdap-karar.service` | `After=`/`Wants=girdap-saat.service` eklendi (**Requires değil** — görev başlamazsa hiç puan yok) |
| `prototype/telemetry/saat_guveni.py` | ROS-bağımsız ölçüt (çekirdek `STA_UNSYNC`). Hata hâlinde **güvenilmez** döner (fail-safe) |
| `prototype/tests/test_saat_guveni.py` | 5 test: çekirdek sabitleri + `struct timex` **208 byte** hizalaması (yanlış hizalama `status`'u kaydırır → sessizce yanlış sonuç) |
| `hardware.launch.py` | Ölçütü **TEK yerde** çağırır, `saat_guvenilir`'i **üç teslim node'una** geçirir + `LogInfo` ile operatöre basar |
| `telemetry_node.py` | `saat_guvenilir` parametresi (önceden **hiç yoktu**); güvenilmezse Dosya-2 adı `..._SAAT-GUVENILMEZ.csv` + WARN |

**Yahya'nın `local_map_node`/`lidar_kayit_node` dosyalarına DOKUNULMADI** —
ikisi `saat_guvenilir`'i zaten okuyordu, launch'tan beslenince kendiliğinden
çalışıyor. Yarım tesisat böylece bağlandı.

**Dosya-2'de neden kolon değil dosya adı:** `CSV_HEADER` md 4.2 alan
sözleşmesi ve `test_telemetry_logger` onu **birebir** çiviliyor; kolon eklemek
şemayı bozar. Dosya adı hiçbir toplayıcıdan kaçmaz. Saat güvenilirse ad
**değişmez** → sıfır regresyon.

Testler: **520 geçti** (`test_p1_saha_senaryolari`'ndaki 1 hata önceden vardı,
bu işle ilgisiz).

> **Neden:** md 4.2 teslimleri (Dosya-1/2/3) **zaman etiketli** olmak zorunda,
> geçersiz dosya başına **5 ceza puanı** (md 5.5.4.3.5). Jetson 06.08'de
> **~15 saat**, 07.08'de **~3 saat** yanlış saatle açıldı (ölçüldü, tahmin
> değil). Yanlış saat sessizdir — dosyalar üretilir, kimse fark etmez.

---

## Neden NTP bir çözüm değil

Şartname md 4.1: **tüm bilgisayarlarda dahili WiFi kapalı**, 2.4-2.8 ve
5.15-5.85 GHz yasak, hücresel yasak. Yani **yarışma günü internet YOK** →
`systemd-timesyncd`/NTP saati düzeltemez. NVIDIA forumlarındaki standart tavsiye
("ethernet tak, NTP aç") bu yarışmada **uygulanamaz**.

Elimizde ağ gerektirmeyen tek mutlak zaman kaynağı: **GPS**.

---

## ✅ Bulgu 1 — FC'nin saati GPS'ten geliyor, ve bu param dökümüyle teyitli

`docs/fc_mevcut_parametreler_2026-08-07.param` satır 153:

```
BRD_RTC_TYPES,1
```

ArduPilot `AP_RTC` bit maskesi: **bit0=1 → GPS** · bit1=2 → MAVLINK_SYSTEM_TIME ·
bit2=4 → onboard HW clock. Yani FC saatini **YALNIZ GPS'ten** alıyor.

**İki sonucu var, ikisi de bizim işimize geliyor:**

1. FC'de **doğru GPS UTC** saati duruyor → okunabilir bir referansımız var.
2. 🔵 **Jetson FC'nin saatini BOZAMAZ.** MAVROS `sys_time` eklentisi
   companion→FC yönünde `SYSTEM_TIME` **gönderir** (ArduPilot dev dokümanı:
   *"mavros sends time to the flight controller"*). `BRD_RTC_TYPES=2` olsaydı
   FC bunu kabul ederdi ve **3 saat geri olan Jetson, FC'nin saatini ve
   dolayısıyla `.BIN` log damgalarını bozardı.** `1` olduğu için FC bu mesajı
   yok sayıyor. **Bu parametre 2 veya 3 yapılMAMALI.**

---

## ✅ Bulgu 2 — MAVROS companion'ın saatini kurmaz

ArduPilot'un resmî `ros-timesync` dokümanı yalnız iki yönü tanımlıyor:
`SYSTEM_TIME` (companion→FC) ve `TIMESYNC` (karşılıklı ofset, EKF harmanlaması
için). **Jetson'ın kendi sistem saatini kurmak için hiçbir mekanizma yok.**
FC'nin saati ROS'a `/mavros/time_reference` (`sensor_msgs/TimeReference`) olarak
gelir — ama onu okuyup sistem saatine yazmak **bize kalıyor**.

Repoda `/mavros/time_reference` **hiçbir yerde kullanılmıyor** (grep boş).

---

## ❌ Klasik yol (gpsd + chrony) BU TEKNEDE İMKANSIZ

Standart stratum-1 tarifi: GPS → seri port → `gpsd` → chrony SHM/SOCK refclock.
**Bizde çalışmaz:** F9P Rover'ın tek birleşik konnektörü var ve yalnız Pixhawk
GPS1 portuna bağlı; bağımsız ikinci UART **yok** (FTDI ile spare pin'lere tapping
denendi, tanımlanamayan ikili protokol geldi — bkz. CLAUDE.md). `gpsd` GPS'i
göremez. Zaman ancak **FC üzerinden** alınabilir.

Zaten **PPS hassasiyetine ihtiyacımız yok**: teslim damgaları için **saniye**
mertebesi yeter, mikrosaniye değil.

---

## 🔧 Önerilen tasarım

### Katman 1 — açılışta GPS'ten kur (birincil)

`girdap-saat.service` — **oneshot**, `girdap-karar.service`'ten **ÖNCE**:

```
pymavlink ile seri porta bağlan → SYSTEM_TIME bekle → time_unix_usec makul mü
→ clock_settime() ile sistem saatini kur → STA_UNSYNC'i temizle → çık
```

**Neden MAVROS üzerinden DEĞİL — iki tuzak:**

1. 🔴 **Dairesel bağımlılık.** MAVROS'u `girdap-karar.service` başlatıyor. Saat
   kurucu MAVROS'u bekleyecek olsa, karar servisinin saat kurucudan sonra
   başlaması gerekirdi → döngü. Çözüm: saat kurucu **kendi** MAVLink
   bağlantısını kurar (pymavlink), işini yapar, **çıkar**.
2. 🔴 **Seri port tekildir.** MAVROS `/dev/ttyUSB0`'ı tutar; iki süreç aynı
   portu açamaz. Bu yüzden saat kurucu **oneshot** olmak ZORUNDA — MAVROS
   başlamadan koşar, portu bırakır.

**Neden sıra önemli:** `local_map_node`, `lidar_kayit_node` ve `telemetry_node`
oturum dizinini/dosya adını **başlangıçta** üretiyor
(`datetime.now().strftime("oturum_%Y%m%d_%H%M%S")`). Saat sonradan düzelse bile
**dosya adları yanlış kalır**. Bu yüzden saat, karar servisinden önce doğru
olmalı.

```
[Unit]
Before=girdap-karar.service
# girdap-karar.service'e: After=girdap-saat.service
```

**GPS fix beklemesi:** geçerli `time_unix_usec` için fix şart. Servis
zaman aşımıyla (örn. 90 s) yeniden dener; yetişmezse **saati kurmaz** ve
Katman 2 devreye girer. Fix'i beklemek için `girdap-karar`'ı süresiz
geciktirmek YANLIŞ olur — görev başlamazsa hiç puan yok.

**`STA_UNSYNC` neden temizlenmeli:** Eyüp'ün `girdap_ida_algi/saat.py`'si saat
güvenini çekirdeğin `adjtimex` STA_UNSYNC bayrağından okuyor. `date -s` /
`clock_settime` bu bayrağı **temizlemez** → saat artık doğru olduğu hâlde
kareler "güvenilmez" damgalanır (yanlış-negatif; kendi dosyasında yazıyor).
`adjtimex` `ADJ_STATUS` ile temizlenirse bayrak **dürüst** olur — çünkü saat
gerçekten bir referansa (GPS) göre disipline edilmiştir.

### Katman 2 — kurulamadıysa dürüst ol (yedek)

GPS'ten kurulamadıysa `saat_guvenilir=false` üç teslim node'una geçirilir.

🔴 **Şu an bu tesisat YARIM BAĞLI:** `local_map_node` ve `lidar_kayit_node`
`saat_guvenilir` parametresini **okuyor** ama **hiçbir yerden beslenmiyor**
(ne `hardware.launch.py`'da ne yaml'larda geçiyor) → varsayılan `True`'da
kalıyor, yani sistem **saat yanlışken de "güvenilir" diyor**.
`telemetry_node`'da (Dosya-2) parametre **hiç yok** — dosya adı ve **her
satırın `zaman` sütunu** doğrudan `datetime.now(timezone.utc)`.

> ⚠️ Yahya'nın 07.08 düzeltmesi (*"damga duvar saatinden değil
> `header.stamp`'ten"*) **başka bir sorunu** çözüyor: hangi ANA ait olduğunu
> (yazma anı ≠ ölçüm anı). Ama `header.stamp` de `get_clock().now()`, yani
> **aynı sistem saati**. Saatin DOĞRULUĞU o düzeltmeyle ele alınmadı.

### Katman 3 — RTC pili (donanım, tamamlayıcı)

Jetson Orin Nano geliştirme kitinin **BBAT** pininde saat pili yoksa saat her
güç kesintisinde kayar/sıfırlanır — gözlenen "bayat saatle açılma" deseni buna
uyuyor.

| | |
|---|---|
| Pil tipi | **CR1225, şarj edilemez** (datasheet'in belirttiği) |
| ⚠️ Kullanılmayacak | Şarjlı **ML1220 + şarj devresi** — Orin Nano'nun BBAT tasarımıyla uyumsuz (NVIDIA forum) |
| Bilinen tuzaklar | Yanlış RTC cihazı (`/dev/rtc0` ↔ `rtc1`), `CONFIG_RTC_HCTOSYS_DEVICE`, güç kesintisinden sonra 1970'e düşme |

Pil **tek başına yetmez** (doğru saati bir kez kurmak gerekir) ama bir kez
kurulan saati güç kesintileri arasında **taşır** → GPS fix'inin yetişmesine
bağımlılık azalır.

### Katman 4 — insan yedeği (elle)

Kıyıda `sudo date -s "..."` (telefon saatinden). Eyüp'ün toplayıcısında bunun
için `--saat-elle-dogrulandi` bayrağı **zaten var**. Karar tarafında karşılığı
yok.

---

## ⏳ Uygulamadan önce ÖLÇÜLECEK

| # | Ne | Neden |
|---|---|---|
| 1 | `ros2 topic hz /mavros/time_reference` — **akıyor mu, kaç Hz?** | `SYSTEM_TIME` bir EXTRA stream grubunda; `bridge.stream_rate_hz: 10` istediğimiz halde bu mesaj gelmiyor olabilir. **Akmıyorsa tüm plan çöker** — pymavlink yolu MAVROS'tan bağımsız olduğu için oradan da ayrıca doğrulanmalı |
| 2 | `/mavros/time_reference`'ın `time_ref`'i ile sistem saati arasındaki fark | Gerçekten GPS UTC mi, yoksa boot'tan beri geçen süre mi (ArduPilot bazı mesajlarda öyle yapıyor) |
| 3 | GPS fix'ten `SYSTEM_TIME`'ın geçerli olmasına kadar geçen süre | Servisin zaman aşımı buna göre seçilecek |
| 4 | Jetson'da RTC pili **var mı**, hangi tip | Katman 3 |
| 5 | `adjtimex` aracı Jetson'da kurulu mu (`apt install adjtimex`) | STA_UNSYNC temizlemek için |

🔴 **1 numara olmadan kod yazmak anlamsız.** Jetson'a bağlanıp tekne açıkken
ölçülmeli — masa testinde de olur (GPS kapalı alanda fix almayabilir, o zaman
dışarıda).

---

## Kapsam notu

Bu **kod değil, sistem yapılandırması** — A-2'deki systemd drop-in işine benzer.
Karar node'larının hiçbirine dokunulmuyor: sistem saati bir kez doğru olursa
`datetime.now()` / `get_clock().now()` kullanan **her şey** (üç teslim, ROS
log'ları, Eyüp'ün kamera mp4'ü, dosya adları) kendiliğinden doğru olur.

> **Tasarım kararı (kullanıcı, 09.08):** damgayı yalnız yazma anında düzeltmek
> **reddedildi** — o zaman Dosya-2 düzeltilmiş, diğer her şey yanlış sistem
> saatinde kalırdı, yani **iki ayrı zaman tabanı** oluşurdu. Doğrusu: tek saat,
> kaynağı GPS.

---

## Kaynaklar

- [ArduPilot — Clock/Time Synchronisation (ros-timesync)](https://ardupilot.org/dev/docs/ros-timesync.html)
- [chrony — Configuration examples](https://chrony-project.org/examples.html)
- [NVIDIA forum — RTC time reset after reboot (Orin)](https://forums.developer.nvidia.com/t/rtc-time-date-are-correct-but-system-time-date-are-reset-after-reboot/256963)
- [Orin Nano RTC pili kurulumu / tip uyumsuzluğu](https://nvidia-jetson.piveral.com/jetson-orin-nano/jetson-orin-nano-rtc-battery-issue-setup-and-troubleshooting/)

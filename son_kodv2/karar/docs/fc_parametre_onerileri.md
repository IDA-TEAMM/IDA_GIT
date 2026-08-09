# FC (Pixhawk/ArduRover) Parametre Önerileri — GİRDAP İDA

> **Kime:** FC ekibi. **Neden:** 2026-07-12 masa oturumu bulguları — kaçak
> motor OLAY'ı (RC/AUTO ile FC'deki eski görevin kendi kendine koşması),
> "Radio failsafe on" PreArm engeli, RC kalibrasyonsuzluğu ve F14.1 notu
> (Jetson KILL'i FCU'yu disarm ediyor ama FCU'nun KENDİ failsafe'leri de
> doğru kurulmalı). Kaynak: resmî ArduPilot Rover dokümanları
> (ardupilot.org/rover — rover-failsafes, flight-mode-configuration,
> arming-your-rover; 2026-07-12'de bakıldı). Bu bir ÖNERİ listesidir —
> son karar FC ekibinin; "KARAR" işaretli satırlar özellikle tartışılmalı.
>
> Doldurma: "Mevcut" sütununu QGC → Parameters ekranından okuyup yazın;
> sonucu `docs/olcum_formu.md` FC bölümüyle birlikte geri gönderin.

## 0.0 🔴🔴 MEVCUT RC SETİ ŞARTNAMEYE AYKIRI — 2.4 GHz (2026-08-04)

**Takılı alıcı: RadioLink R9DS v2.1** (fotoğrafla teyit). RadioLink R9DS ve
eşleştiği vericiler (AT9 / AT9S / AT10 / AT10II) **2.4 GHz** bandındadır.

> Şartname md 4.1: *"İDA, İDA-YKİ, İHA, İHA-YKİ(varsa), **RC kumandalar** ve
> telemetri modülleri dahilinde **2.4-2.8 GHz**, 5.15-5.85 GHz aralıklarında
> çalışan herhangi bir bileşen ya da modül kullanılmayacaktır."*

**Ceza:** md 5.5.4.3.2 → yasaklı frekans kullanımı **55 ceza puanı**. Ayrıca
teknik kontrollerde (md 5.2) tespit edilir.

**Zincirleme etkileri:**
- md 4.2 uzaktan güç kesme *"RC kumandadan verilebilecektir"* diyor ve
  **minimum gereksinimdir**. Uyumlu RC yoksa bu fonksiyon YKİ yazılımı
  üzerinden çözülmek zorunda (§4.5 röle tasarımı buna göre gözden geçirilecek).
- `rc_kill_channel`, `MODE_CH`, `rc_manual_channel` kanal planı yeni sete göre
  **yeniden yapılacak**.
- Şimdi yapılacak RC kalibrasyonu yeni sette **tekrarlanacak** (yine de bench
  testleri için gerekli).

**Uyumlu alternatifler** (2.4 ve 5.15-5.85 dışı): ExpressLRS 868/915 MHz ·
TBS Crossfire 868/915 MHz · FrSky R9 (900 MHz) · 433 MHz LRS.
Not: telemetri zaten RFD868x (868 MHz) — ekip bu banda aşina.

### ✅ KAPTAN KARARI (2026-08-04): yarışmada RC HİÇ KULLANILMAYACAK

RC seti yalnız otonomi kabiliyeti videosu ve masa/bench testleri içindi.
Yarışmada her şey otonom, kumanda yok → **2.4 GHz ihlali ortadan kalkıyor**
(modül yarışma alanına götürülmeyecek).

Şartname uygunluğu ✅: md 4.2 uzaktan güç kesmeyi *"İDA YKİ yazılımı üzerinden
**ya da** RC kumandadan"* diye tanımlar — RC zorunlu değil.

**Ama bu karar üç şeyi değiştiriyor:**

**(a) Uzaktan güç kesme yolu değişti (§4.5 güncellenecek).** Eski birincil
tasarım "RC alıcı kanalı → doğrudan röle, FC'den BAĞIMSIZ" idi. Yeni yol:
```
Mission Planner → MAVLink DO_SET_RELAY → RELAY1_PIN → kontaktör
```
⚠️ **Sağlamlık kaybı bilinçli kabul edilmeli:** RC-doğrudan yolda FC çökse
bile röle çalışıyordu. Artık zincir **FC + telemetri linkine bağımlı**; FC
donarsa uzaktan kesme de gider, geriye yalnız araç üstü kırmızı buton kalır
(o da uzaktan değil). Şartname yasaklamıyor ama tek nokta arıza yaratıyor.

**(b) 🔴 RC'siz ARM olmayabilir.** `FS_THR_ENABLE=1` açıkken alıcı yoksa FC
sürekli "Radio failsafe on" görür ve pre-arm'da arm'ı reddeder (bu proje
daha önce tam bu duvara çarptı — `dogrulama_matrisi.md`: *"RC kumanda bağlı
(PreArm 'Radio failsafe on' çözülmeden ARM olmaz)"*). Alıcı sökülünce
`FS_THR_ENABLE=0` ve arming kontrollerinin RC ayağı kapatılmalı —
**yarışmadan önce test edilmeden bırakılmayacak.**

**(c) Görev sonrası geri getirme.** md 5.5.3.1 *"İDA'sını kalkış noktasına
manuel olarak getirebilecektir"* diyor. RC yoksa: Mission Planner'dan mod
değiştirip sürmek (görev bittiği için komut yasağı kalkar) ya da fiziksel
alma (kayık/kanca). Ekibin planı netleşmeli.

**(d) Kod tarafı:** `rc_kill_channel` ve `rc_manual_channel` yolları yarışmada
ÖLÜ olacak. Yanlış güven yaratmamaları için yarışma config'inde (`-1`)
kapatılmalı — bkz. açık iş listesi madde 3/8.

> Mevcut 2.4 GHz seti **yalnız masa/bench testlerinde** kullanılabilir;
> yarışma alanına götürülmeyecek (md 5.5.3.1: yarışma süresince atölye
> çadırında bile yasaklı bantta modül çalıştırılamaz).

**Ek durum (2026-08-04):** alıcı LED'i **kırmızı sabit** = beslenmiş ama
vericiyle **bind DEĞİL**. `RC3/RC5/RC8_MIN/TRIM/MAX` hâlâ fabrika
varsayılanında olması bununla tutarlı — bu FC'de RC hattı hiç çalışmamış
görünüyor. Dolayısıyla `rc_kill_channel=8` varsayımı, `FS_THR_VALUE=910`
eşiği ve `ARMING_RUDDER=2` ile çubuktan arm/disarm **hiçbiri fiilen
doğrulanmadı**.

---

## 0. ~~🔴 KANAL ÇAKIŞMASI~~ — KONUSUZ KALDI (RC kaldırıldı, §0.0)

> **Bu bölüm artık uygulanmıyor.** Yarışmada RC kullanılmayacağı için kanal
> çakışması diye bir sorun kalmadı: `rc_kill_channel`/`rc_manual_channel` `-1`
> (kapalı, `config/yarisma.yaml`), röle de RC kanalından değil MAVLink
> `DO_SET_RELAY` ile tetikleniyor. Kayıt olarak bırakıldı — **bench testinde
> RC kullanılırsa** çakışma yine geçerlidir.

### (arşiv) Özgün bulgu

2026-08-04 param dökümünden çıktı:

| Kim | Ne yapıyor | Kanal |
|---|---|---|
| ArduPilot | `MODE_CH=8` → **mod seçici** | RC 8 |
| `mavros_bridge_node` | `rc_kill_channel: 7` (0-indeksli) → **KILL** | RC 8 |
| Planlanan (§4.5) | Uzaktan güç kesme rölesi | RC 8 |

**Aynı fiziksel anahtar üç işi birden yapıyor.** Şu an görünür bir arıza yok
çünkü `MODE1..MODE6` hepsi `0` (MANUAL) — anahtar hangi konumda olursa olsun
mod değişmiyor. Ama bu **tesadüfi bir güvenlik**:

- İleride bir konuma GUIDED/AUTO atanırsa, **kill switch'e basmak mod
  değiştirir** — tam da acil durumda istenmeyecek şey.
- `RCx_OPTION = "Relay On/Off"` ataması da bu kanala yapılacaktı; mod seçici
  bir kanala ikinci fonksiyon bindirmek karışıklık üretir.

**Öneri:** `MODE_CH`'i **kullanılmayan bir kanala taşı** (ör. 7 — `RC7_OPTION=0`,
boşta) ve **kanal 8'i tamamen kill/röle'ye ayır**. Böylece:
```
RC 7 → mod seçici (ArduPilot)
RC 8 → kill switch: röle + mavros_bridge KILL (tek amaç)
RC 5 → manuel override (yarışmada kapalı, bkz. madde 8)
```

> ~~Karar FC ekibinin; ama kanal 8'e röle atanmadan önce çözülmeli.~~
> → Röle artık RC kanalına bağlanmıyor; madde kapandı.

---

## 1. OLAY'ı bir daha yaşamamak için (öncelik 🔴)

| Parametre | Öneri | Mevcut | Neden |
|---|---|---|---|
| `MODE_CH` (vars. 8) | Mod kanalını BİLİNÇLİ seç; yaylı/kolay çarpılan anahtara bağlama | ___ | OLAY tetiği: kalibrasyon kaydında CH5 iki konum arasında atlıyordu |
| `MODE1`..`MODE6` | **Hiçbir konuma AUTO koymayın.** Öneri: MANUAL / HOLD ağırlıklı; GUIDED istenirse tek bilinçli konum | ___ | Görevi FC'nin AUTO'su değil, Jetson (GUIDED + MAVROS) sürüyor. AUTO'ya geçiş = FC hafızasındaki görevi kendi koşması = kaçak |
| `INITIAL_MODE` | HOLD | ___ | Boot'ta motor komutu üretmeyen mod. Ayrıca başlatma tetiğimiz (GUIDED'a geçiş, T0-j) kenar tetikli — boot'ta GUIDED OLMAMALI |
| `BRD_SAFETY_DEFLT` | **1'e GERİ AL** (masada 0 yapıldı) + emniyet düğmesine fiziksel erişimi çözün (GPS direği üstünde) | ___ | Çıkışlar düğmeye basılmadan aktifleşmesin; OLAY'da 0 olması motorları serbest bırakmıştı |
| — (param değil) | **Her oturum kapanışında FC görev hafızasını silme alışkanlığı** (QGC Plan → Remove All) | — | FC'de duran eski/test görevi + yanlışlıkla AUTO = tam-yol kaçış. Masa runbook'unun "M0-ÖNCESİ" bloğuna da yazıldı |

## 2. Arm/disarm güvenliği

| Parametre | Öneri | Mevcut | Neden |
|---|---|---|---|
| `ARMING_REQUIRE` | Varsayılan kalsın (arm şart) | ___ | 0 yapmak = güç verilince motorlar hazır; asla |
| Ön-arm kontrolleri (`ARMING_CHECK`/`ARMING_SKIPCHK`) | HEPSİ AÇIK (0 = tümü) | ___ | GPS fix'siz/kalibrasyonsuz ARM engellenir — yazılımdaki F-M.1 guard'ıyla aynı yönde çift katman |
| `ARMING_RUDDER` | ~~KAPALI (0) önerilir~~ → **2 KALACAK (ekip kararı 2026-08-04)** | **2** | Öneri 0'dı (kazara arm/disarm riski). **Ekip çubukla arm/disarm istiyor → karar kabul, tartışma kapandı.** Ara seçenek `1` (yalnız ARM) da sunuldu, `2` tercih edildi. **Telafi:** RC kalibrasyonu artık ZORUNLU (aşağıya bkz.) — risk büyük ölçüde değerlerin fabrika varsayılanında olmasından geliyordu. Kalan risk: görev sırasında çubuğa dokunulursa tekne parkur ortasında disarm olur; acil kesme için zaten kanal 8 kill + röle var |
| RC kalibrasyonu | 🟡 **YARIM** — Mission Planner → Mandatory Hardware → Radio Calibration | **KISMEN** | **2026-08-06 canlı döküm** (önceki değerlendirme sahte dosyadandı, düzeltildi): `RC1 = 1000/1476/2000` ve `RC3_TRIM = 1476` → bunlar **gerçek kalibrasyon** izi (yuvarlak olmayan sayılar). Ama `RC2/RC5/RC8 = 1100/1500/1900` → hâlâ **fabrika varsayılanı**. Yani kanal 1 kalibre, gerisi değil. Yarışmada RC kullanılmayacağı için (bkz. §0.0) bu artık **düşük öncelikli**; yalnız bench'te kumanda kullanılırsa kanal 5/8 eşikleri tahmine dayalı kalır |

## 3. Failsafe'ler (F14.1'in FCU ayağı)

| Parametre | Öneri | Mevcut | Neden |
|---|---|---|---|
| `FS_THR_ENABLE` + `FS_THR_VALUE` | AÇIK; VALUE'yu RC kalibrasyonu BİTTİKTEN sonra ayarlayın (throttle min'in altı) | ___ | RC menzil kaybında failsafe. ⚠ RC3 min 915/trim 1075 ölçüldü — kalibrasyonsuz eşik ya hiç tetiklenmez ya sürekli tetiklenir |
| `FS_ACTION` | 2 (Hold) | ___ | RC kaybında motorlar durur. 5 (Disarm) masada cazip ama suda görev sırasında RC hıçkırığında disarm istenmez. (1/3=RTL: GPS'e döner — parkur dışına çıkma cezası riski, önermiyoruz) |
| `FS_TIMEOUT` | 1-2 s | ___ | Varsayılan 1 s |
| `FS_GCS_ENABLE` | **KARAR:** 0 mı 1 mi? | ___ | 1 ise QGC-RFD linki koparsa araç Hold'a düşer → görev kesilir. Jetson'ın kendi 5 s heartbeat-KILL bekçisi zaten var (mavros_bridge). Hangi MAVLink kaynağının "GCS" sayıldığı `SYSID_MYGCS`'e bağlı (MAVROS mu QGC mu) — **masada test edilmeden 1 yapmayın** |
| `FS_CRASH_CHECK` | Şimdilik 0/varsayılan | ___ | ⚠ Parkur-3 kamikaze BİLİNÇLİ çarpma içeriyor — 2 (Hold+Disarm) hedefe çarpınca disarm eder. Yarışma konfigürasyonunda yeniden değerlendirilecek (T2) |
| `BATT_LOW_VOLT` / `BATT_FS_LOW_ACT` | Batarya ekibiyle: 4S Li-ion eşiği (örn. 3.2 V/hücre ⇒ ~12.8 V) + aksiyon 2 (Hold) | ___ | Varsayılan 0 = KAPALI. Batarya failsafe'siz görevde diplere inen hücre = geri dönüşsüz hasar |

## 4. Telemetri hattı (M1 bulgularının devamı)

| Konu | Öneri | Neden |
|---|---|---|
| `SERIAL2_PROTOCOL/BAUD` | Mevcut 57600 çalışıyor (TELEM2, çapraz kablo sonrası); USB-C soketi tamir edilirse USB'ye dönülebilir | 57600 tavanı IMU'yu ~10 Hz'te sınırlıyor; `SR2_*` stream-rate paramlarıyla oynanabilir |
| Pixhawk USB-C soketi | Çapraz test: başka bilgisayara tak; orada da descriptor hatası varsa tamir/RMA | `device descriptor read error -32` — donanım günlüğü §2 reçetesi |

## 4.4. SENSÖR KONUM OFSETLERİ — ölçüldü, girilmeyi bekliyor (2026-08-04)

Araç orijini **gövde geometrik merkezi** olarak tanımlandı (ön uçtan 51.5 cm,
merkez hattı, gövde tabanı). Ölçümler `docs/olcum_formu.md` §0/§2'de.

**Girilecek değerler (metre):**

| Parametre | Değer | Mevcut | Fiziksel karşılığı |
|---|---|---|---|
| `INS_POS1_X` | `-0.055` | **0** | Pixhawk 5.5 cm kıçta |
| `INS_POS1_Y` | `-0.1375` | **0** | Pixhawk 13.75 cm iskelede |
| `INS_POS1_Z` | `-0.155` | **0** | Pixhawk 15.5 cm yukarıda |
| `GPS1_POS_X` | `-0.035` | **0** | Anten 3.5 cm kıçta |
| `GPS1_POS_Y` | `-0.16` | **0** | Anten 16 cm iskelede |
| `GPS1_POS_Z` | `-0.365` | **0** | Anten 36.5 cm yukarıda |

> ⚠️ **Parametre adı:** GPS ofsetleri bu firmware'de **`GPS1_POS_X/Y/Z`**
> (eski `GPS_POS1_*` DEĞİL — 2026-08-04 param dökümünden teyit edildi).
> Mevcut değerlerin hepsi **0** → tahmin doğrulandı: konum referansı şu an
> düzeltilmemiş, yani raporlanan konum GPS anteninin konumu.

**Neden gerekli:** Bu parametreler sıfırken ArduPilot "GPS de IMU da orijinde"
varsayar; mutlak konumu GPS çivilediği için raporlanan konum pratikte **GPS
anteninin** konumu olur. Anten merkez hattının 16 cm iskelesinde → tüm konum
16 cm kayık. Kapı ortasına sürerken tekne gövdesi o kadar sancağa kaymış
geçer; kapı ~1.35 m, tekne 0.785 m → yan pay 28.25 cm iken sancak payı yarıya
iner (md 5.5.4.2 geçiş + `Ç1`/`Ç2` çarpma cezası).

> ⚠️ **EKSEN:** ArduPilot body frame = X ileri, **Y sancak +**, **Z aşağı +**.
> ROS/TF ise Y sol +, Z yukarı +. Yukarıdaki tabloda çevrim YAPILMIŞTIR —
> "iskelede" ve "yukarıda" olanlar negatif. `hardware.yaml tf:` bloğundaki
> aynı fiziksel konumlar ters işaretli görünür, bu normaldir.

> ⚠️ ArduPilot dokümanı ofsetleri **ağırlık merkezine** göre tanımlar; biz
> geometrik merkezi kullandık (küçük teknede yakın, düzeltmeyi asıl belirleyen
> GPS↔IMU göreli geometrisi zaten doğru). CoG belirgin şekilde başkaysa
> değerler ötelenir.

**Doğrulama:** Parametreler yazıldıktan sonra Write+reboot; masa testinde
konumun beklendiği gibi davrandığı GÖRÜLMEDEN "tamam" denmeyecek.

---

## 4.5. 🔴 UZAKTAN GÜÇ KESME — şartname MİNİMUM GEREKSİNİMİ (2026-08-03)

> Şartname md 4.2, "Uzaktan Güç Kesme":
> *"Aracın üzerindeki fiziki güç anahtarının yanı sıra **aktive edildiğinde tüm
> motorlardan ve aktüatörlerden anında gücü kesebilecek bir uzak güç kesme
> fonksiyonu olacaktır**. Uzaktan güç kesme İDA YKİ yazılımı üzerinden ya da RC
> kumandadan verilebilecektir.*
> *o **Güç kesme önlemlerinde motorlara gönderilen sinyallerin akışını kesmek
> yeterli değildir, motorların gücünün kesilmesi şarttır.**"*

**Bugünkü durumumuz bu maddeyi KARŞILAMIYOR.** Mevcut zincirin tamamı sinyal
seviyesinde kalıyor:

| Katman | Ne yapıyor | Şartname karşılığı |
|---|---|---|
| RC kanal 8 → `mavros_bridge` KILL | disarm + sıfır thrust | ❌ sinyal kesme |
| `MOT_SAFE_DISARM=1` | disarm'da PWM min'e düşer | ❌ sinyal kesme |
| Araç üstü kırmızı anahtar | gücü fiziksel keser | ✅ ama bu **ayrı** bir madde ("Araç Üzerinden Güç Kesme") — uzaktan değil |

Bu bir **minimum gereksinim**: sağlanmazsa teknik kontrollerden geçilmez
(md 4: *"Minimum gereksinimleri sağlayamayan takımlar final aşamasında yarışmaya
hak kazanamaz"*). Yazılımla kapatılamaz — **donanım işi**.

### 🔴 Röle hattının NEREYE konulacağı (en kritik karar)

Kontaktör/röle **YALNIZCA ESC-motor güç kolunda** olacak. Pixhawk, Jetson, LiDAR,
kamera ve telemetri **ayrı bir kolda** beslenmeye devam edecek.

```
Batarya ─┬─ [KONTAKTÖR] ─→ 4× ESC ─→ thruster'lar     ← uzaktan kesilen kol
         └─ (kesilmez) ──→ Pixhawk + Jetson + sensörler + telemetri
```

**Neden:** Güç kesme anında Jetson da ölürse telemetri CSV'si, kamera mp4'ü ve
lokal harita PNG'leri yarım kalır → md 4.2 Dosya-1/2/3 teslim edilemez, her biri
için **5'er ceza puanı** (md 5.5.4.3.5). Ayrıca acil durum sonrası hakem
"ne oldu" diye sorduğunda elde log kalmaz.

### ✅ KONTAKTÖR TEMİN EDİLDİ — GRDNER HEV50-A12NS (2026-08-06)

| | Etiket değeri |
|---|---|
| Marka / P/N | **GRDNER HEV50-A12NS** (Rev. A) |
| Yük | **50 A**, 12–900 VDC |
| Bobin | **12 VDC** |

> Datasheet internette bulunamadı (GRDNER yaygın indekslenmiş bir marka değil).
> En yakın muadil aile: Altran Magnetics ALEV50. Bu yüzden aşağıdaki açık
> maddeler **deneyle** doğrulanacak, veri sayfasına güvenilmeyecek.

**🔴 BAĞLANTI HATASI — ÖLÇÜMLE KANITLANDI (2026-08-06), DÜZELTME BEKLİYOR.**

Kontaktör *"bataryadan gelen TÜM gücü, güç dağıtım kartından"* kesiyor — yani
Pixhawk ve Jetson da ölüyor. Bu, yukarıdaki "röle hattının nereye konulacağı"
kuralının ihlali.

> **Kanıt (2026-08-06 22:07, Mission Planner):** kill switch'e basıldığında
> ekranda **`WARNING No Data for 3 Seconds`** çıktı — yani **FC'nin kendisi
> güçsüz kaldı**, telemetri kesildi. Tahmin değil, gözlem.

Yarışmadaki karşılığı: acil durdurmaya basıldığı anda Jetson ölür →
Dosya-1/2/3 yarım kalır (md 4.2, **15 ceza puanı**) ve hakem "ne oldu"
dediğinde elde log kalmaz.

**Donanım ekibiyle konuşuldu, karar verildi:** Pixhawk ve Jetson kontaktör
hattından **ayrılacak**, kontaktör yalnız ESC kolunda kalacak.
⏳ **HENÜZ UYGULANMADI** — yukarıdaki ölçüm düzeltmeden önce alındı. Ayrım
yapıldıktan sonra aynı test tekrarlanmalı: kill switch'e basıldığında telemetri
**kesilmemeli**, yalnız motorlar durmalı.

**⏳ Kapanmadan önce doğrulanacak ÜÇ şey:**

| # | Açık | Nasıl doğrulanır | Neden kritik |
|---|---|---|---|
| 1 | **50 A yetiyor mu?** | Suda tam gazda iki motorun **toplam tepe akımını** ölç (`olcum_formu.md §4`, o satır hâlâ boş). ESC etiketi "çekilen" değil "dayanılan" akımdır — gerçek çekiş çok daha az olabilir | Görev sırasında 50 A aşılırsa kontaktör yanar → motor kontrolü kaybedilir |
| 2 | **Bobin sürücüsü var mı?** | Pixhawk röle pini 3.3 V / birkaç mA verir; bobin **12 V** ister. Araya MOSFET veya küçük röle modülü ŞART | Doğrudan bağlanırsa çalışmaz, FC pini zarar görebilir |
| 3 | **NO mu NC mi?** | Kontaktörü çıkar, ana uçlara multimetre (süreklilik). **Bobinde gerilim yokken süreklilik OLMAMALI** = NO ✅ | NC çıkarsa bobin kablosu koptuğunda motorlar **çalışmaya devam eder** — acil durdurmanın tersi |

### Kontaktör seçimi (ölçüt — yukarıdaki ürün buna göre değerlendirilecek)

| Özellik | Gereklilik | Neden |
|---|---|---|
| Akım | Sürekli ≥ **2×** thruster tepe akımı, marj ile | Tekne **2 motorlu** (2026-07-19'da yardımcı thruster'lar ESC'leriyle söküldü, 2026-08-04'te kalıcı olduğu teyit edildi). KTR'deki "4× thruster" ifadesi güncel DEĞİL. ESC anlık çekişi nominalin üstünde |
| Kutuplama | **NO — enerjilendirilince kapanan** (energize-to-run) | Bobin beslemesi/kablosu koparsa motorlar **durur**. NC seçilirse kablo kopması motorları serbest bırakır = ters emniyet |
| Tip | Mekanik kontaktör ya da yüksek akım SSR | İkisi de kabul; SSR'de ısınma payı bırak |
| Konum | Batarya (+) kolu, sigortadan sonra | Sızdırmaz bölme içinde, md 4.2 emniyet |

### Uygulama — RC KALKTIĞI İÇİN GÜNCELLENDİ (2026-08-04)

> ⚠️ Bu bölümün ilk hâlinde birincil yol **"RC alıcı kanalı → doğrudan röle,
> FC'den bağımsız"** idi. Kaptan kararıyla yarışmada RC kullanılmayacağı için
> (§0.0) **o yol artık yok.** Geriye FC üzerinden tetikleme kalıyor.

**Tek yol: YKİ → MAVLink → FC rölesi**
```
Mission Planner  →  MAVLink DO_SET_RELAY  →  RELAY1_PIN  →  kontaktör
                                                          →  ESC gücü kesilir
```
Şartname uygun: md 4.2 uzaktan güç kesmeyi *"İDA YKİ yazılımı üzerinden ya da
RC kumandadan"* diye tanımlar; YKİ yolu yeterlidir.

🔴 **Bilinçli kabul edilen zayıflık:** zincir artık **FC + telemetri linkine
bağımlı**. FC donarsa/telemetri koparsa uzaktan güç kesme de gider; geriye
yalnız araç üstündeki kırmızı buton kalır (o da uzaktan değil). RC-doğrudan
yolda bu bağımlılık yoktu. Ekip bunu bilerek kabul ediyor.

> **Azaltıcı önlem önerisi:** telemetri linki koptuğunda FC'nin kendi
> `FS_GCS_ENABLE=1` failsafe'i devreye girebilir (şu an **0**, §3'te açık
> karar). Güç kesmenin yerini tutmaz ama aracı Hold'a alır.

**FC parametreleri**

| Parametre | Değer | Neden |
|---|---|---|
| `RELAY1_FUNCTION` | `1` (Relay) | Röle çıkışını etkinleştirir |
| `RELAY1_PIN` | ⚠️ **MP'nin parametre açıklamasındaki listeden** AUX çıkışı seç | Pin numaralandırması karta göre değişir — tahmin etme, MP'nin kendi listesinden seç ve doğrula |
| `RELAY1_DEFAULT` | `0` (boot'ta kapalı) | Boot'ta motorlar güçsüz açılsın — `INITIAL_MODE=HOLD` ile aynı felsefe |
| ~~`RCx_OPTION`~~ | — | **GEREKMEZ** — RC yok; röle Mission Planner'dan MAVLink `DO_SET_RELAY` ile tetiklenir |

> **Mission Planner'dan tetikleme:** bağlıyken röle çıkışı MP'nin servo/relay
> kontrol arayüzünden veya `DO_SET_RELAY` komutuyla açılıp kapatılır.
> Yarışma öncesi **kuru zeminde denenip** operatörün nereye basacağı
> ezberlenmeli — acil anda menü aranmaz.

### Katman planı (RC kalktıktan sonra)

```
YKİ (Mission Planner)
  ├─→ MAVLink DO_SET_RELAY → RELAY1 → kontaktör → MOTOR GÜCÜ KESİLİR  ← md 4.2
  └─→ /girdap/mission/kill (fsm_node)  → disarm + sıfır thrust        ← 2. katman

Araç üstü kırmızı buton → fiziksel güç kesme (uzaktan DEĞİL, ayrı md 4.2 maddesi)
```

**Yazılım tarafı:** `rc_kill_channel` / `rc_manual_channel` yarışmada `-1`
(kapalı) — `config/yarisma.yaml` overlay'inde ayarlı. ⚠️ `-1`'in gerçekten
kapattığı F-S.12'de düzeltildi; öncesinde negatif indeks Python'da SON kanalı
okuyup görev ortasında yanlış KILL basabilirdi.

### Doğrulama

Bench runbook **ADIM 6B** (pervanesiz, aşağıdaki §5 listesine de eklendi):
anahtara basıldığında multimetreyle **ESC besleme ucunda 0 V** okunmalı —
PWM'in 1000'e düşmesi bu maddenin kanıtı DEĞİLDİR.

---

## 5. Doğrulama (parametreler yazıldıktan sonra, PERVANESİZ)

1. Boot → HOLD'da açıldığını QGC'den teyit et (`INITIAL_MODE`).
2. RC mod anahtarını TÜM konumlarda gez → hiçbirinde AUTO'ya geçmediğini gör.
3. RC'yi kapat → `FS_TIMEOUT` içinde Hold + QGC'de "Radio Failsafe" mesajı.
4. Emniyet düğmesine basmadan ARM dene → reddedilmeli (`BRD_SAFETY_DEFLT=1`).
4b. **Uzaktan güç kesme (§4.5):** ARMED + cmd_vel akarken kill anahtarına bas →
   multimetreyle **ESC besleme ucunda 0 V**; Jetson'ın telemetri CSV'si yazmaya
   DEVAM etmeli (Dosya-2 kesilmemeli). Bkz. runbook ADIM 6B.
5. Sonuçları `docs/olcum_formu.md` FC bölümüne işleyip geri gönderin —
   masa runbook M4-M6 bu değerlerle tekrarlanacak.

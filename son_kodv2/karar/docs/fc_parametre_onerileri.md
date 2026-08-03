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
| Çubukla arm/disarm (`ARMING_RUDDER` — adı QGC param aramasından teyit edin) | KAPALI (0) önerilir | ___ | Arm QGC'den yapılıyor (video md 3.3.1). RC trim'leri tuhafken (CH2 üst uçta dinleniyor) çubuk kombinasyonu yanlışlıkla arm/disarm edebilir |
| RC kalibrasyonu | QGC ile BAŞTAN + `RCMAP_*` kontrolü | — | Masada trim'ler elle yazıldı (geçici). CH2/CH3 uç değerlerde dinleniyor — kanal eşlemesi şüpheli |

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

### Kontaktör seçimi

| Özellik | Gereklilik | Neden |
|---|---|---|
| Akım | Sürekli ≥ 4× thruster tepe akımı, marj ile | 4× 2838 thruster; ESC anlık çekişi nominalin üstünde |
| Kutuplama | **NO — enerjilendirilince kapanan** (energize-to-run) | Bobin beslemesi/kablosu koparsa motorlar **durur**. NC seçilirse kablo kopması motorları serbest bırakır = ters emniyet |
| Tip | Mekanik kontaktör ya da yüksek akım SSR | İkisi de kabul; SSR'de ısınma payı bırak |
| Konum | Batarya (+) kolu, sigortadan sonra | Sızdırmaz bölme içinde, md 4.2 emniyet |

### Uygulama — A ve B, İKİSİ BİRDEN

**A) RC alıcı kanalı → doğrudan röle sürücüsü (BİRİNCİL, önerilen)**
RC alıcının kanal çıkışı FC'ye uğramadan röle bobinini sürer (küçük bir MOSFET
sürücü modülü üzerinden). **FC çökse, yazılım kilitlense, MAVLink kopsa bile
çalışır** — şartnamenin istediği "anında" budur. Tek bağımlılık RC alıcısının
kendisidir.

**B) FC üzerinden röle (İKİNCİL katman)**

| Parametre | Değer | Neden |
|---|---|---|
| `RELAY1_FUNCTION` | `1` (Relay) | Röle çıkışını etkinleştirir |
| `RELAY1_PIN` | ⚠️ **MP'nin parametre açıklamasındaki listeden** AUX çıkışı seç | Pin numaralandırması karta göre değişir — tahmin etme, MP'nin kendi listesinden seç ve doğrula |
| `RELAY1_DEFAULT` | `0` (boot'ta kapalı) | Boot'ta motorlar güçsüz açılsın — `INITIAL_MODE=HOLD` ile aynı felsefe |
| `RCx_OPTION` (kill kanalı) | Dropdown'dan **"Relay On/Off"** | MP'de isimle seçilir, numara ezberlemeye gerek yok |

> ⚠️ B tek başına YETMEZ: FC'ye bağımlıdır. A olmadan "FC kilitlendiğinde ne olacak"
> sorusunun cevabı yok. B'nin değeri, aynı anahtarın hem röleyi hem `mavros_bridge`
> KILL'ini tetiklemesidir (tek hareket, üç katman).

### Kanal planı — mevcut yazılım DEĞİŞMİYOR

`mavros_bridge_node` varsayılanları zaten kanal 8'i (`rc_kill_channel: 7`,
0-indexli) kill için okuyor. Aynı fiziksel anahtar üç şeyi birden yapsın:

```
RC kanal 8 (kill anahtarı)
  ├─→ (A) röle sürücüsü            → MOTOR GÜCÜ KESİLİR   ← şartname maddesi
  ├─→ (B) FC RELAY1                → yedek güç kesme
  └─→ mavros_bridge KILL           → disarm + sıfır thrust (üçüncü katman)
```

Yazılım tarafında yapılacak bir şey yok — kanal numarası değişirse
`hardware.yaml` `rc_kill_channel` güncellenir.

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

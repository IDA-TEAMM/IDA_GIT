# ArduRover Bench Parametre Önerisi — Pixhawk 6C

> **Kime:** Donanım entegrasyonu ekibi (hand-off).
> **İlgili doküman:** [`bench_mavlink_runbook.md`](bench_mavlink_runbook.md) — kuru zemin doğrulama adımları.

---

## AMAÇ

**SITL değil**, gerçek **Pixhawk 6C** üzerinde ArduRover'ı **bench (kuru zemin,
pervanesiz)** modunda güvenli koşturmak. Değerler Mission Planner →
**Config → Full Parameter List** üzerinden girilir (veya `.parm` dosyası olarak
yüklenir). Suya inerken bazı parametreler değişecek — **⚠️ işaretlilere dikkat**.

> **Firmware:** Rover-4.5+ (Boat frame desteği olgun). Katamaran = skid/diferansiyel tahrik.

---

## FRAME (Gövde / Motor Karışımı)

| Parametre | Değer | Neden |
|---|---|---|
| `FRAME_CLASS` | `2` | Boat sınıfı — su aracı dinamiği + skid steering karışımı. |
| `SERVO1_FUNCTION` | `74` | Throttle**Right** — SAĞ thruster çıkışı. |
| `SERVO3_FUNCTION` | `73` | Throttle**Left** — SOL thruster çıkışı. |
| `SERVO1_MIN` / `SERVO3_MIN` | `1000` | Çıkış aralığının alt ucu = **tam GERİ** (aşağıdaki uyarıya bak). |
| `SERVO1_MAX` / `SERVO3_MAX` | `2000` | Çıkış aralığının üst ucu = tam ileri. |
| `SERVO1_TRIM` / `SERVO3_TRIM` | `1487` | Sıfır itki noktası. ⚠️ Nötr 1500'den 13 µs sapık — bkz. aşağıdaki not. |
| `MOT_SAFE_DISARM` | `1` | Disarm'da motor çıkışı kesilir. ⚠️ Ne ürettiği **ölçülecek** (bkz. uyarı). |

> ## 🔴 ÇİFT YÖNLÜ (BIDIRECTIONAL) ESC — 1100 "DUR" DEĞİL, "TAM GERİ"
>
> **ESC'ler çift yönlü, 50 A** (2026-08-04 teyidi). Bu, PWM anlamlarını
> değiştirir:
>
> | PWM | Tek yönlü ESC'de | **Bizim çift yönlü ESC'de** |
> |---|---|---|
> | 1000 | dur | **TAM GERİ** |
> | 1500 | orta hız | **DUR (nötr)** |
> | 2000 | tam ileri | tam ileri |
>
> **Bu dokümanın önceki hâli yanlıştı:** alt uç "motorun güvenli durur PWM'i"
> diye açıklanmıştı ve runbook ADIM 6 disarm'da "PWM 1000" bekliyordu. Çift
> yönlü ESC'de o değer **tam geri** demektir — yani "güvenli duruş" sandığımız
> şey tam ters yönde tam gaz olurdu.
>
> **🔴 GERÇEK DEĞERLER (2026-08-06 canlı döküm):**
> ```
> SERVO1_MIN = SERVO3_MIN = 1000     SERVO1_MAX = SERVO3_MAX = 2000
> SERVO1_TRIM = SERVO3_TRIM = 1487   SERVO1_REVERSED = SERVO3_REVERSED = 0
> ```
> Bu değerler **otonomi kabiliyeti videosunda gerçek suda çalıştı** —
> dokunulmayacak. (Dokümanın önceki hâlindeki `1100/1900/1500` üçlüsü ÖLÇÜM
> DEĞİL VARSAYIMDI ve `parametrelerDefMP.param` adlı, bu tekneyi hiç tarif
> etmeyen bir dosyadan doğrulanmış sanılmıştı — bkz. `hata_defteri.md`.)
>
> ⚠️ **`TRIM=1487`, nötr 1500 değil.** Sıfır itki komutunda FC 13 µs geri
> yönde sinyal üretiyor. Videoda sorun çıkarmamış → ESC'nin ölü bandının
> içinde kalıyor. **Değiştirilmedi**; ilk su testinde sıfır komutta sürüklenme
> görülürse ilk şüpheli budur.
>
> ⚠️ **`MOT_PWM_MIN`/`MOT_PWM_MAX` ArduRover'da YOKTUR** (Copter parametresi,
> canlı dökümde de yok). Rover'ın karşılığı `SERVOn_MIN`/`MAX`.
>
> **🔴 ÖLÇÜLMEDEN VARSAYMA — ADIM 6'da şunu gör:** disarm anında servo
> çıkışında ne var?
> - **Sinyal yok** (pals kesilmiş) → ESC kendi failsafe'iyle durur ✅ beklenen
> - **1487-1500** → nötr, motor durur ✅
> - **1000** → 🔴 **TAM GERİ** — bu çıkarsa `MOT_SAFE_DISARM` /
>   `SERVOx_TRIM` yapılandırması yanlış, suya İNİLMEZ
>
> Not: uzaktan güç kesme kontaktörü (§4.5) bu riskin üstünde ayrı bir katman —
> gücü kestiği için ESC ne komut alırsa alsın motor dönmez. Yine de FC
> tarafının doğru olması gerekir (kontaktör her senaryoda devrede değil).

> **Doğrulama:** Runbook ADIM 5'te sol/sağ PWM ~1600; **ADIM 5B'de dönüş yönü**
> (aşağı bkz.); ADIM 6 disarm'da 1000/min.

> ⚠️ **Tekne 2 MOTORLU.** 2026-07-19'da yardımcı thruster'lar ESC'leriyle birlikte
> fiziksel olarak söküldü; 2026-08-04'te bunun kalıcı olduğu (yarışma dahil)
> teyit edildi. `SERVO2/4_FUNCTION` **atanmayacak**. KTR ve
> `docs/KTR/algoritma_tasarimlari.md` içindeki "4× 2838 thruster" ifadeleri
> güncel donanımı yansıtmıyor — rapor revizyonunda düzeltilmeli.

---

## ESC KALİBRASYONU — ✅ KAPANDI (2026-08-06)

> **Özet:** Kalibrasyon **yapılmayacak** (gerekçe adım 3) · FC parametreleri
> simetrik ✅ · ESC başlama eşikleri eşit ✅ · PWM aralığı ölçüldü, iki kanalda
> **0 fark** ✅ (runbook ADIM 5). Kalan tek belirsizlik ESC'lerin **iç** trim'i,
> o da yalnız suda görülür (aşağıda "KALAN RİSK").

**Neden önemli:** İki ESC farklı kalibre edilirse **aynı PWM farklı itki**
üretir. Sonuç: `angular.z=0` verilirken tekne bir tarafa kayar; navigasyon
katmanı bunu sürekli düzeltmeye çalışır, kapı ortasından geçiş bozulur
(md 5.5.4.2 geçiş puanı). Bu, yazılımda "PID kötü ayarlanmış" gibi görünen ama
kaynağı donanımda olan, teşhisi en zor hatalardan biridir.

**Sıra:**

1. 🔴 **Pervaneler sökülü**, tekne sabit, batarya bağlı.
   (~~RC verici açık~~ — RC iptal edildi, kaptan kararı 2026-08-04.)
2. ~~**ESC: çift yönlü (bidirectional), 50 A**~~ **✅ MODEL TESPİT EDİLDİ 2026-08-06**
   → **Markasız jenerik "Bidirectional ESC 50A"** (motorobit.com, *"Su Altı
     Motoru ile Uyumlu"*). Etiket: `50A` · `BEC 2A 5V` · `LIPO 2S-4S`.
   → Nötr **1500**, ileri 1500→2000, geri 1500→1000.
   → **Geri gidiş var** → MPPI'nin negatif itki komutları kullanılabilir
     (tek yönlü olsaydı boşa giderdi).
3. ~~**Kalibrasyon gerekli mi?**~~ **✅ KARAR: KALİBRASYON YAPILMAYACAK
   (2026-08-06)**

   **🔎 Bulgu — danışılacak belge YOK.** Prosedürün önceki hâli *"önce
   üreticinin kendi dokümanına bak"* diyordu. ESC **markasız**; satıcı
   sayfasında teknik döküm yok, üretici manuali mevcut değil. Yani bu talimatın
   cevabı yok — karar kanıta dayandırıldı:

   | # | Gerekçe |
   |---|---|
   | 1 | Klasik gaz kalibrasyonu (**tam gaz → min gaz** öğretme) **TEK YÖNLÜ** ESC prosedürüdür. Çift yönlüde "tam gaz"/"min gaz" = **tam ileri/tam geri**; prosedür ESC'ye yeni uçlar öğretir ve **nötr noktasını kaydırabilir**. |
   | 2 | Mevcut ayarlar **gerçek suda kanıtlanmış**: `SERVO1/3` değerleri otonomi kabiliyeti videosunda çalıştı, iki kanalda birebir aynı. Kanıtlanmış bir durumu, **belgesi olmayan** bir prosedürle riske atmak yanlış takas. |
   | 3 | Ampirik: 2026-08-05'te güç verildiğinde **ESC'ler öttü** (normal arming) ve Motor Test'te ikisi de %20 güçte düzgün döndü. Bu sınıfta kalibrasyonsuz/bozuk ESC genelde ya hiç arm olmaz ya sinyali tanımaz. |

   ⚠️ **KALAN RİSK ve nerede görünür.** İki ESC'nin fabrika iç trim'i birbirinden
   biraz farklıysa **aynı PWM farklı itki** üretir → sıfır dönüş komutunda tekne
   bir yana kayar.
   - 🔴 **ADIM 5 bunu YAKALAYAMAZ.** İki kanalın FC parametreleri birebir aynı
     olduğu için Servo Output'ta zaten `1600/1600` okunur — bu, ESC'lerin *iç*
     simetrisi hakkında **hiçbir şey söylemez**. ADIM 5 yalnız **FC → ESC sinyal**
     yolunu doğrular, **ESC → itki** eşleşmesini değil. (Runbook'taki "simetri
     kontrolü (ESC kalibrasyonunu sınar)" ifadesi bu yüzden fazla iddialı.)
   - ✅ **Nerede görünür:** ilk **su testinde**, düz gitme komutunda sabit yön
     kayması olarak. Ölçüm: `angular.z=0` ver, heading sürüklenmesini izle.
   - ✅ **Neden ölümcül değil:** ArduPilot'un yaw kontrolcüsü kapalı döngü —
     küçük itki asimetrisini soğurur. Büyükse `SERVOn_TRIM`'de ±birkaç µs
     farkla telafi edilir (kalibrasyon değil, tek parametre).
4. ~~**FC parametreleriyle tutarlılık**~~ **✅ 2026-08-06 — DOĞRULANDI**
5. ~~**İki kanalın değerleri BİREBİR aynı olmalı**~~ **✅ 2026-08-06 — DOĞRULANDI**

   Kaynak: `docs/fc_mevcut_parametreler_2026-08-06.param` — **tekneye bağlıyken
   alınmış canlı döküm** (Pixhawk6C, ArduRover 4.6.3, ttyUSB0):

   | | SERVO1 | SERVO3 | |
   |---|---|---|---|
   | `MIN` | 1000 | 1000 | ✅ aynı |
   | `MAX` | 2000 | 2000 | ✅ aynı |
   | `TRIM` | **1487** | **1487** | ✅ aynı — sıfır itki noktası (nötr 1500'den 13 µs sapık, bilinçli bırakıldı) |
   | `FUNCTION` | 74 (**sağ**) | 73 (**sol**) | ✅ farklı olmalı |
   | `REVERSED` | 0 | 0 | ✅ ikisi de 0 — 05.08 motor testinde yönler DOĞRU çıktı |

   > 🔴 **2026-08-06 DÜZELTMESİ.** Bu tablo aynı gün daha önce `1100/1900/1500`
   > ve `REVERSED 1/0` diye yazılmıştı. **YANLIŞTI** — kaynak alınan
   > `parametrelerDefMP.param` bu tekneden alınmış bir döküm DEĞİLDİ (146
   > parametrede canlıdan farklı; şablon/varsayılan dosyası). Tekneye bağlanıp
   > taze döküm alınınca gerçek değerler çıktı. **Ders: "FC'de şu var" demeden
   > önce dökümün gerçekten O ARAÇTAN ve GÜNCEL olduğunu doğrula.**
   >
   > Ayrıca fonksiyon numaralandırması kesinleşti: **73 = ThrottleLeft**,
   > **74 = ThrottleRight** (canlı `SERVO1=74` ve MP Servo Output ekranında
   > `SERVO1 = ThrottleRight` yazıyordu).

   Simetriyi belirleyen üç değer (`MIN`/`MAX`/`TRIM`) birebir aynı → "aynı PWM
   farklı itki" riski FC tarafında YOK. Kalan asimetri kaynağı yalnız ESC'nin
   kendi iç kalibrasyonu (adım 3).

   > ⚠️ **`MOT_PWM_MIN`/`MOT_PWM_MAX` ArduRover'da YOKTUR** (Copter parametresi;
   > dökümde de yok). Rover'ın karşılığı `SERVOn_MIN`/`MAX` — yukarıda doğru.
   > `Document 11.pdf` A-1 satırındaki "MOT_PWM_MIN=1100 / MAX=1900 yaz"
   > talimatı bu isimle aranmamalı **ve değerleri de geçersiz**: canlı sistem
   > `1000/2000` ile çalışıyor ve bu su testinden geçmiş.

6. **Write Params + reboot** — ⚠️ **şu an yazılacak bir şey YOK**, mevcut
   değerler doğru ve kanıtlı. Runbook **ADIM 5B-1** ✅ (koşuldu, 2026-08-05),
   **ADIM 6** 🟡 (2026-08-06: ARM'da `ch1out=ch3out=1487` ✅ **1000 değil**;
   DISARM ölçülemedi — normal disarm da Force Disarm da reddedildi, güç
   kesilerek durduruldu).

7. ~~**Başlama eşiği simetrisi**~~ **✅ 2026-08-06 — GEÇTİ.** MP Motor Test'te
   güç %1'den yukarı: **Motor C %1'de başladı, Motor D de %1'de başladı.**
   İki ESC'nin ölü bandı ve nötr noktası eşleşiyor → kaba asimetri YOK.
   (Testin çözünürlüğü %1; altındaki fark suda zaten görünmez.)
   ⚠️ `MOT_THR_MIN=10` Motor Test yolunu kırpmıyor — bu test o yüzden çalıştı.

---

## ARMING (BENCH — ⚠️ SU'DA DEĞİŞECEK)

| Parametre | Değer | Neden |
|---|---|---|
| `ARMING_CHECK` | `0` | ⚠️ **Bench:** GPS/EKF fix yokken arming'e izin ver (kuru test). **SU'DA `1`** — tüm pre-arm kontrolleri açık. |
| `ARMING_REQUIRE` | **`1`** | 🔴 **BENCH'TE DE 1 OLMALI** — aşağıdaki kutuya bak. Eski öneri `0`'dı, **YANLIŞTI**. |

> 🔴 Bu iki parametre **su öncesi checklist'in ilk maddesi**. `ARMING_CHECK=0`
> ile suya inmek = EKF sağlıksızken hareket riski. Asla unutma.

> ## 🔴 `ARMING_REQUIRE=0` ACİL DURDURMAYI ÖLDÜRÜYOR (2026-08-07'de bulundu)
>
> Bu dokümanın önceki hâli bench için `ARMING_REQUIRE=0` öneriyordu ("arming
> zorunluluğunu gevşet"). **Bu tavsiye zararlıydı ve düzeltildi.**
>
> ArduPilot'un kendi parametre açıklaması: *"Arming disabled until some
> requirements are met. **If 0, there are no requirements (arm immediately)**"*
> → araç **sürekli armed** sayılır ve **disarm hiçbir yolla çalışmaz.**
>
> **2026-08-06'da ölçüldü:** disarm Mission Planner Arm/Disarm ile, MP Force
> Disarm ile, kumandadan rudder disarm ile, MAVROS `cmd/arming` ile — MANUAL'da
> da HOLD'da da **reddedildi** (`result=4`), FC hiç açıklama basmadı.
>
> **En kritik sonuç:** yazılımımızın KILL yolu (`mavros_bridge` → disarm +
> sıfır thrust) `ARMING_REQUIRE=0` iken **HİÇ ÇALIŞMIYORDU**. Acil durdurma
> sessizce etkisizdi; kimse fark etmemişti çünkü disarm hiç test edilmemişti.
> Şartname md 4.2 emniyet gereksinimi doğrudan etkileniyor.
>
> **2026-08-07 düzeltmesi:** `ARMING_REQUIRE = 1` yazıldı, disarm **ilk
> denemede** çalıştı. Ardından ADIM 6 ölçüldü: disarm'da `ch1out = ch3out =
> 1487` (nötr) — tam geri **değil**, güvenli.
>
> ⚠️ **GERİ ALINMAMALI.** 0 yapılırsa acil durdurma tekrar ölür.
> ⚠️ **Yan etki:** araç artık kendiliğinden armed gelmez, arm edilmesi gerekir.
> Masa testi akışlarını etkiler — takıma haber verildi (2026-08-07).

### Buzzer — açık kalmalı (2026-08-07)

| Parametre | Değer | Neden |
|---|---|---|
| `NTF_BUZZ_TYPES` | `5` | Masa testinde `0`'a çekilmişti (ses rahatsız ediyor). **Geri açıldı.** |
| `NTF_BUZZ_VOLUME` | `100` | Aynı. |

Buzzer, teknenin **armed olduğunu gösteren tek sesli işaret**. `BRD_SAFETY_DEFLT=0`
(emniyet anahtarı devre dışı) olduğu için buzzer kapalıyken tekne **sessizce** her an
motor çalıştırabilir durumda olur — yanına yaklaşan biri bunu anlayamaz.

> ⚠️ `NTF_BUZZ_*` **açılışta okunur** — yazdıktan sonra reboot etmeden devreye girmez.
> 🔎 Doğrulandı 2026-08-07: reboot melodisi + disarm sesi duyuldu.
> 🔎 Yan gözlem: batarya bağlı değilken (USB beslemesi) buzzer **sürekli alarm**
> çalar — `Bat1 0,02 V` < `BATT_CRT_VOLT 12.4` → kritik batarya failsafe'i.
> Bu **doğru davranış** ve gerilim failsafe yolunun çalıştığını kanıtlar.

---

## FAILSAFE

| Parametre | Değer | Neden |
|---|---|---|
| `FS_ACTION` | `2` | Hold — failsafe'te motorları durdur (aracı olduğu yerde tut). |
| `FS_TIMEOUT` | `1.5` | 1.5 s sinyal kaybı failsafe eşiği — hızlı tepki, yanlış tetik dengesi. |
| `FS_GCS_ENABLE` | `1` | GCS (MAVROS) heartbeat kaybında failsafe — Jetson/mavros düşerse araç durur. |
| `FS_THR_ENABLE` | `1` | RC throttle failsafe — RC verici kapanınca/menzil dışında failsafe. |

> **Karar yığını ile ilişki:** `mavros_bridge` yazılım tarafı heartbeat KILL'i
> (5 s) yayınlar; `FS_GCS_ENABLE=1` ise **firmware tarafı** bağımsız ikinci
> katman. İki katman = tek nokta arıza yok (Runbook ADIM 7).

---

## MODE / GCS / Telemetri Rate

| Parametre | Değer | Neden |
|---|---|---|
| `SYSID_MYGCS` | `255` | MAVROS varsayılan GCS ID'si — bridge komutlarının kabulü için eşleşmeli. |
| `SR0_EXTRA1` | `10` | Attitude (roll/pitch/yaw) 10 Hz — Dosya-2 telemetri + video log için taze poz. |
| `SR0_POSITION` | `10` | Local/global position 10 Hz — fusion/planning + Dosya-2 lat/lon akışı. |
| `SR0_EXT_STAT` | `2` | Extended status (mod, arm, sistem) 2 Hz — `/mavros/state` bant genişliği dengesi. |

> **Not:** `SR0_*` = SERIAL0 (USB) mesaj akış hızları. Jetson'a giden USB
> portu; telemetri (RFD868x) ayrı port/rate ister. Dosya-2 ≥1 Hz şartını
> `SR0_POSITION=10` fazlasıyla karşılar.

---

## EKF

| Parametre | Değer | Neden |
|---|---|---|
| `EK3_ENABLE` | `1` | EKF3 füzyon motoru açık — poz/hız kestirimi (yarışma modu iSAM2 bunu tamamlar). |
| `EK3_GPS_TYPE` | `0` | 3D hız + konum — RTK GPS'ten tam ölçüm kullan. |
| `AHRS_EKF_TYPE` | `3` | EKF3'ü ana AHRS kaynağı yap (EKF2 değil) — daha olgun, RTK uyumlu. |

> **Video modu ilişkisi:** `use_isam2=false` iken karar yığını doğrudan bu
> EKF3'ün `/mavros/local_position/pose` çıktısını kullanır (pass-through).
> Yarışma modunda iSAM2 aynı ölçümleri GPS+IMU füzyonuyla pürüzsüzleştirir.

---

## `.parm` Dosyası Üretimi

Yukarıdaki değerler Mission Planner'da tek tek girilebilir veya bir
`config/ardurover_bench.parm` metin dosyası olarak (her satır `PARAM,VALUE`)
**Full Parameter List → Load** ile yüklenebilir. Örnek satır formatı:
```
FRAME_CLASS,2
SERVO1_FUNCTION,73
...
ARMING_CHECK,0
```
> Yükleme sonrası **Write Params** + Pixhawk **reboot** şart (frame/servo
> fonksiyonları reboot'ta oturur).

---

## 🔴 SUYA İNERKEN CHECKLIST

Bench yeşil olsa bile **suya inmeden önce** bu listeyi operatör yüksek sesle okur:

- [ ] `ARMING_CHECK = 1` (pre-arm kontrolleri AÇIK)
- [ ] `ARMING_REQUIRE = 1`
- [ ] Kill switch bağlı **ve test edildi** (Runbook ADIM 6)
- [ ] 🔴 **Uzaktan GÜÇ kesme kontaktörü takılı ve test edildi** (Runbook ADIM 6B —
      ESC ucunda 0 V; şartname md 4.2 minimum gereksinimi, sinyal kesme YETMEZ)
- [ ] RC failsafe test edildi (verici kapatma → Hold)
- [ ] Motor mount sıkı, pervaneler **takılı ve sağlam**
- [ ] Batarya voltajı **> 15 V** (4S için; hücre başına > 3.75 V)
- [ ] Write Params + reboot yapıldı, değişiklikler oturdu
- [ ] GPS fix alındı (`/mavros/state` + EKF healthy=true)

> Bu checklist'te **tek bir madde bile ☐ ise araç suya İNMEZ.**

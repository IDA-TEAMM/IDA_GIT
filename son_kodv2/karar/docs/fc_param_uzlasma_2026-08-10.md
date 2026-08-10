# FC parametre uzlaşması — 2026-08-10

**Durum:** ✅ **FC'YE YAZILDI ve DOĞRULANDI (2026-08-10 01:07)** · **Alan:** Alt Alan B

## ✅ YAZMA DOĞRULANDI

Hedef dosya Mission Planner ile yüklendi ve yazıldı; ardından **taze döküm**
alınıp (`canli_2026-08-07d.param` → repoda
`fc_mevcut_parametreler_2026-08-10-YAZILDI.param`) hedefle kıyaslandı:

```
DEGISEN: 0 · yeni dosyada YOK: 0 · YENI: 0
✅ KRITIK LISTEDE HIC SAPMA YOK
```

Yani **tam istenen 5 parametre yazıldı, fazladan hiçbir şey değişmedi** —
`SERIAL*` ayarlarına dokunulmadı, telemetri hattı kopmadı (öngörüldüğü gibi).

Üç grup gözle de teyit edildi:

| Grup | Sonuç |
|---|---|
| Geri alınan 5 | `BATT_LOW_VOLT 13.2` · `BATT_CRT_VOLT 12.4` · `BATT_FS_LOW_ACT 2` · `BATT_FS_CRT_ACT 3` · `MOT_THR_MIN 10` ✅ |
| Korunan düzeltmeler | `BATT_VOLT_MULT 5.091626` · `ARMING_CHECK 1` · `GPS1_RATE_MS 100` · `GPS1_TYPE 2` · `LOG_DISARMED 1` ✅ |
| Emniyet çekirdeği | `ARMING_REQUIRE 1` · `SERVO1/3_FUNCTION 74/73` · `SERVO*_TRIM 1487` · `FRAME_CLASS 2` · `BRD_RTC_TYPES 1` · `SR2_EXTRA3 10` · `NTF_BUZZ_TYPES 5` ✅ hiç dokunulmadı |

⚠️ **Dosya adı tuzağı:** operatör dökümü `canli_2026-08-07d.param` diye
kaydetti ama tarih **10.08**'dir (adlandırma 07.08 serisinden devam etmiş).
Repoya `...2026-08-10-YAZILDI.param` adıyla alındı. Sonradan okuyan kişi
Masaüstündeki `07d` dosyasını 7 Ağustos'a ait sanmasın.

⏳ **Hâlâ açık:** aşağıdaki "KARAR VERİLMEDİ" tablosu — özellikle **SERIAL
sorusu** (FTDI hangi portta). Yazma işi onu çözmedi, çünkü bilinçli olarak
`SERIAL*`'a dokunulmadı.

## Ne oldu

2026-08-09'da Pixhawk parametreleri **başkası tarafından** değiştirildi.
07.08 temelimizle kıyaslandı (`scripts/param_kiyasla.py`): **928 parametrenin
32'si** farklı. *"Tamamen değiştirilmiş"* değil — ve **en tehlikeli olanların
hiçbirine dokunulmamış:**

`ARMING_REQUIRE=1` · `SERVO1/3_FUNCTION=74/73` · `SERVO*_MIN/MAX/TRIM=
1000/2000/1487` · `SERVO*_REVERSED=0` · `SERVO2/4_FUNCTION=0` · `FRAME_CLASS=2`
· `BRD_RTC_TYPES=1` · `SR2_EXTRA3=10` · `NTF_BUZZ_TYPES=5` · `MODE_CH=8` ·
`INS_POS1_*`/`GPS1_POS_*` (hâlâ 0)

## 🔴 Neden "kendi dosyamızı geri yükle" YAPILMADI

İlk içgüdü *"bizimki doğru, onu yaz"*dı. **Yanlış olurdu, üç sebeple:**

1. **Bizim dosyada gerçek bir HATA var ve o kişi tam onu düzeltmiş.**
   `BATT_VOLT_MULT=18.18` yüzünden batarya **57.7 V** okuyordu (hafızada
   "kalibrasyon şüpheli" diye açık konuydu). Ham gerilim `57.7/18.18 = 3.17 V`;
   yeni katsayıyla `3.17 × 5.0916 = 16.2 V` → **4S paket için tam doğru.**
   Geri yüklemek bu hatayı geri getirirdi.
2. **Taze sensör kalibrasyonunu ezerdi.** Jiroskop yeniden kalibre edilmiş
   (`INS_GYR*OFFS_*`, kalibrasyon sıcaklığı 42 °C → 26 °C). Bizim değerler eski
   ve **farklı sıcaklıkta** alınmış. Elimizde kapanmamış bir **"Kötü AHRS"**
   sorunu varken taze kalibrasyonu bayatla değiştirmek onu kötüleştirebilir.
3. 🔴 **Yazma sırasında canlı telemetri hattını kesebilirdi.** `SERIAL1_BAUD`
   19200'e geri yazılsa ve radyo o portta 57600'de çalışıyorsa bağlantı
   **yazmanın ortasında** kopar → parametrelerin bir kısmı yazılmış, bir kısmı
   yazılmamış, hangisi belli değil. İki durumdan da kötü.

## ✅ Bunun yerine: GERÇEKLİKTEN başla, 5 düzeltme uygula

`docs/fc_hedef_parametreler_2026-08-10.param` = **10.08 canlı dökümü** +
aşağıdaki 5 değişiklik. Sonuç FC'den **tam 5 parametre** farklı (doğrulandı).

**Yönün önemi:** bizim eski dosyadan başlasak **27 bayat değeri** tek tek
elemek gerekirdi. Gerçeklikten başlayınca fark küçük ve denetlenebilir kalıyor.
Ayrıca Mission Planner "Load from file" **yalnız farklı olanları** yazdığı için
`SERIAL*` ayarlarına **hiç dokunulmaz** → konuşulan hat kopmaz.

### Geri alınan 5 parametre (bizim doğrulanmış değerlerimiz)

| Parametre | FC'de | Hedef | Neden |
|---|---|---|---|
| `BATT_LOW_VOLT` | 0 | **13.2** | 3.3 V/hücre. 4S7P Li-ion'da aşırı deşarj hücreleri **kalıcı** bozar |
| `BATT_CRT_VOLT` | 0 | **12.4** | 3.1 V/hücre |
| `BATT_FS_LOW_ACT` | 0 | **2** | low failsafe eylemi — 0 = koruma YOK |
| `BATT_FS_CRT_ACT` | 0 | **3** | critical failsafe eylemi — 0 = koruma YOK |
| `MOT_THR_MIN` | 0 | **10** | düşük hız kalkış eşiği; su testinden geçti. 0'da ince manevra (kapı ortalama) authority kaybı |

> 🔑 **Kritik incelik:** batarya eşikleri ile o kişinin gerilim düzeltmesi
> **birlikte** anlam kazanıyor. Önceden failsafe hiç tetiklenmiyordu çünkü
> 57.7 V daima 13.2'nin üstündeydi. Gerilim artık doğru okunduğu için bu
> failsafe **ilk kez gerçekten çalışacak.**

### ✅ Korunan (o kişinin haklı değişiklikleri)

| Parametre | Değer | Neden korundu |
|---|---|---|
| `BATT_VOLT_MULT` | 5.091626 | **Gerçek düzeltme** — yukarıya bak |
| `ARMING_CHECK` | 1 | Sahada kontroller **açık olmalı**; biz tezgah için 0'lamıştık. Yan etki: arm reddedilebilir ("Kötü AHRS") — arıza değil, kontrolün işini yapması |
| `INS_GYR*OFFS_*`, `INS_GYR*_CALTEMP` | yeni | Taze kalibrasyon, bizimki bayat |
| `GPS1_RATE_MS` | 100 | 5 → **10 Hz**, navigasyon için iyileştirme |
| `GPS1_TYPE` | 2 | Açık uBlox (F9P için doğru) |
| `LOG_DISARMED`, `LOG_FILE_DSRMROT` | 1 | Hata ayıklama kolaylığı |
| `MIS_DONE_BEHAVE` | 0 | 0 = HOLD; tekne için LOITER'dan güvenli |
| `BARO1_GND_PRESS`, `COMPASS_DEC`, `STAT_*`, `MIS_TOTAL` | — | Otomatik/olgusal, müdahale edilmez |
| `SR0_ADSB`, `SR0_RAW_CTRL` | 0 | USB akış hızı azaltımı, zararsız |

### ⏳ KARAR VERİLMEDİ — o kişiye sorulacak

| Parametre | FC'de | Neden bekliyor |
|---|---|---|
| 🔴 `SERIAL2_BAUD` | **921600** (bizde 57600) | — |
| ~~`SERIAL2_OPTIONS`~~ | 8 | ✅ **DEĞİŞMEMİŞ — bizde de zaten 8.** İlk raporda yanlış yazıldı (eski dosyada `OPTIONS` satırı grep'lenmemiş, yokluğu 0 sanıldı). TX/RX swap **bizim kendi ayarımız**, kablolama onu gerektiriyor |
| 🔴 `SERIAL1_BAUD` | **57600** (bizde 19200) | — |
| ⚠️ `BATT_AMP_PERVLT` | **0.44** (bizde 36.36) | 0.44 A/V ile 3.3 V tam skalada ~1.5 A okunur; sistemde **50 A** kontaktör var → akımı ciddi eksik okur. Nasıl bulundu? |
| ⚪ `COMPASS_MOTCT` | 1 | Motor kompanzasyonu açık ama `COMPASS_MOT_*` katsayıları değişmemiş (muhtemelen 0) → etkisiz. Kalibre edildi mi? |

> 🔴 **SERIAL sorusu her şeyi kilitliyor:** bizim `fcu_url` **57600** diyor ve
> dokümanımız FTDI'yi **TELEM2**'de gösteriyor. TELEM2 artık **921600** (swap ikisinde de aynı).
>
> Tek gerçek uyumsuzluk **baud**: `SERIAL2_OPTIONS` (swap) iki tarafta da 8,
> yani kablolama sözleşmesi değişmemiş.
>
> - FTDI **TELEM2'de kaldıysa** → MAVROS **hiç bağlanamaz** (921600 ↔ 57600). 09.08 gecesi
>   Jetson'da görülen `connected: false` bunun belirtisi olabilir (bataryayı
>   kapalı sanmıştım — ayırt edilemedi).
> - FTDI **TELEM1'e taşındıysa** → 57600 tutuyor, çalışır; ama dokümanı ve
>   saat servisinin dayandığı `SR2_EXTRA3` referansını **`SR1`**'e çevirmeliyiz
>   (`SR1_EXTRA3=2` → saat servisi için yeterli).
>
> **Sorulacak tek soru: FTDI kablosu hangi porta takılı?**
> Cevap gelmeden `SERIAL*` parametrelerine dokunulmaz.

## Uygulama

```
Mission Planner → Config → Full Parameter List
  → Load from file → docs/fc_hedef_parametreler_2026-08-10.param
  → değişen 5 satırın işaretlendiğini GÖR (fazlası varsa DUR)
  → Write Params
```

Sonra **taze döküm** al ve teyit et:

```bash
python3 scripts/param_kiyasla.py \
    docs/fc_hedef_parametreler_2026-08-10.param <yeni_dokum>.param
# beklenen: DEGISEN 0
```

## Dosyalar

| Dosya | Ne |
|---|---|
| `fc_mevcut_parametreler_2026-08-07.param` | Bizim doğrulanmış temelimiz (**referans**, geri yazılmaz) |
| `fc_mevcut_parametreler_2026-08-10.param` | Değiştirildikten sonraki canlı hâl |
| `fc_hedef_parametreler_2026-08-10.param` | **Yazılacak olan** (canlı + 5 düzeltme) |
| `scripts/param_kiyasla.py` | Kıyas aracı; kritik listeyi ve "bozulursa ne olur"u ayrı raporlar |

---

# ✅ RC FAILSAFE TESTİ — 2026-08-10, GEÇTİ (ama yarışma için karar gerekiyor)

**Ayar:** `FS_THR_ENABLE 0 → 1`, `FS_ACTION 0 → 2` (Hold). `FS_THR_VALUE=975`
zaten doğruydu (SERVO min 1000'in altında).

**Koşullar:** pervaneler sökülü, ESC'ler beslenmiyor → sıfır riskli ölçüm.

## Ölçümler

| Test | `ch3in` | `ch1out`/`ch3out` | Mod | |
|---|---|---|---|---|
| Kumanda kapalı, gaz **ortada** (13:25) | 1499 | 1487 | Manual | ⚠️ failsafe tetiklenmedi |
| Kumanda kapalı, gaz **açık** (13:33) | **1811** | **1487** | **Hold** | ✅ **GEÇTİ** |

🔴 **Kritik senaryo doğrulandı:** *tam gazda giderken kumanda koptu.* Alıcı
gaz kanalını **1811'de tutmasına rağmen** FC çıkışı **1487'ye (nötr)** çekti ve
**Hold**'a geçti. 1000 (TAM GERİ) görülMEDİ — çift yönlü ESC'de aranan buydu.

## 🔎 Yol boyunca çıkan bulgu: alıcı SON DEĞERİ TUTUYOR

Kumanda kapalıyken tüm girişler duruyor (`ch1in=1506`, `ch3in=1811`,
`ch5in=2000`, `ch10in=1005`). Yani **link kaybı ≠ alıcı susar**; alıcı son
komutu yollamaya devam ediyor.

Sonucu: `FS_THR_ENABLE`'ın klasik tetikleyicisi (gaz < `FS_THR_VALUE`) **hiç
devreye girmiyor** — eşik 975, alıcı 1811 yolluyor. Failsafe'i tetikleyen şey
ArduPilot'un **RC çerçevesi zaman aşımı** (`FS_TIMEOUT=1.5`).
⚠️ 13:25 ölçümünde mod `Manual` kalmıştı — muhtemelen ekran görüntüsü o 1,5
saniyelik zaman aşımından ÖNCE alındı. Su testinde tekrar bakılacak.

> 💡 Daha sağlam olurdu: **alıcının kendi failsafe'ini** (R9DS F/S tuşu) gaz
> 975'in altına gelecek şekilde ayarlamak → iki bağımsız tetikleyici olurdu.
> Şu an tek tetikleyici çerçeve zaman aşımı; o da çalışıyor ama tek nokta.

## 🔴 YARIŞMA İÇİN KARAR GEREKİYOR — bu ayar görevi ENGELLER

Yarışmada RC seti **kullanılmayacak** (2.4 GHz, md 4.1 yasağı) → verici
**kapalı** olacak. Ölçtük: **verici kapalı = Hold.** Yani tekne suya bırakılır,
Jetson görevi başlatmak ister, FC **Hold'da oturur ve hiç hareket etmez.**

| Seçenek | Artı | Eksi |
|---|---|---|
| **A. Yarışma sabahı `FS_THR_ENABLE=0`** | Kanıtlanmış, basit | Unutulursa görev hiç başlamaz → kontrol listesine ŞART |
| **B. `FS_OPTIONS`'ta "AUTO/GUIDED'da devam et" biti** | İkisi birden: testte koruma, yarışmada engel yok | Bit doğrulanmadı + test edilmedi; körlemesine açılmaz |

**Şimdilik A benimsendi** (kanıtlanmış). B araştırılıp test edilirse daha temiz.
Her iki hâlde de **kontrol listesine "yarışma sabahı" adımı** girmeli — WiFi
kapatma ve `girdap-karar-yarisma.conf` drop-in'i gibi.

## ✅ KARAR VERİLDİ ve UYGULANDI — `FS_THR_ENABLE = 0` (yarışma önceliği)

**Kullanıcı kararı (2026-08-10):** *"su testlerinde gemiye uzun bir ip
bağlıyoruz; önemli olan yarışma durumu, ona göre yapalım."*

Gerekçe sağlam — **ip fiziksel bir emniyet ağı** ve yazılım failsafe'inden daha
güvenilir. Üstelik güvenlik kapsaması **boşluksuz** kalıyor:

| Durum | Koruma |
|---|---|
| Kumanda **bağlı**, tekne ters gidiyor | **Acil stop tuşu** (ch10, ölçüldü: 1005 çalışır / 2005 durur) |
| Kumanda **koptu** | **İp** (su testi) |
| Batarya biter | `BATT_FS_LOW_ACT=2` / `BATT_FS_CRT_ACT=3` — **aktif kalıyor** |

Yani `FS_THR_ENABLE=0`'ın açtığı tek boşluk (link kaybı) ipe düşüyor.

**Son hâl (dökümle doğrulandı):** `FS_THR_ENABLE=0` · `FS_ACTION=2` ·
`FS_GCS_ENABLE=0` · `FS_THR_VALUE=975` · `FS_TIMEOUT=1.5`

`FS_ACTION` bilerek **2'de bırakıldı**: şu an atıl (tetikleyen failsafe yok),
ama ileride biri bir failsafe açarsa davranış "hiçbir şey yapma" değil **Hold**
olur. `0`'da bırakmak sessiz bir tuzak olurdu.

> 📌 **Bu turun asıl kazancı parametre değil BİLGİ.** FC'de net değişiklik
> yalnız `FS_ACTION 0→2`. Ama suya inmeden önce bilinmesi gereken üç şey
> öğrenildi: alıcı sinyal kesilince **susmuyor** · failsafe'i tetikleyen şey gaz
> eşiği değil **çerçeve zaman aşımı** · çıkış **nötre (1487)** çekiliyor, tam
> geriye (1000) değil.

## 🔴 SONRAKİ AÇIK — yarışma günü alıcı sökülürse arm reddedilebilir

Şartname md 4.1 2.4 GHz'i yasakladığı için RC seti yarışmada
kullanılmayacak — alıcı **sökülebilir**. Ama `ARMING_CHECK=1` RC kontrollerini
de kapsıyor → **arm reddedilebilir** ("RC not calibrated" vb.).

Yarışma sabahı tekne arm olmazsa iş biter. **Önceden test edilmeli:** alıcıyı
çıkar, arm etmeyi dene. Reddederse `ARMING_CHECK`'ten RC biti düşürülecek.

---

# 🔴 KÖK NEDEN BULUNDU: "Kotu AHRS" = kalibrasyonlar GEÇERSİZ (2026-08-10)

Alıcısız arm testine hazırlanırken arm reddedildi. Mesajlar:

```
18:53:47 : Arm: Compass not calibrated
18:53:47 : Arm: 3D Accel calibration needed
18:53:50 : EKF variance
18:53:44 : EKF failsafe cleared
```

## Zincir

```
ivmeölçer + pusula kalibrasyonu GEÇERSİZ
        ↓
EKF tutarlı çözüm üretemiyor  →  "EKF variance"
        ↓
EKF failsafe  →  HUD'da FAILSAFE
        ↓
"Kotu AHRS"
```

**Oturumlardır açık duran "Kotu AHRS" maddesinin sebebi buymuş.** Ayrıca
`FS_ACTION` ile ilgisi YOK (kullanıcı haklı olarak şüphelenmişti) — kaynak EKF.

## ⚠️ İLK RAPOR YANLIŞTI: "hiç yapılmamış" DEĞİL, "geçersiz sayılıyor"

İlk okumada "kalibrasyonlar hiç yapılmamış" denildi. **Yanlış.** Veri duruyor:

| | Değer | |
|---|---|---|
| `COMPASS_OFS_*` | 22.87 / 25.06 / 2.65 | gerçek kalibrasyon |
| `COMPASS_DIA_*` | 0.979 / 0.942 / 1.014 | elipsoid düzeltmesi var |
| `COMPASS_ODI_*` | sıfır değil | var |
| `COMPASS_DEV_ID` ↔ `PRIO1_ID` | 658953 = 658953 | eşleşiyor |
| `INS_ACCOFFS/SCAL_*` | 0.017/0.027/0.041 · ≈1.000 | kalibre |
| `INS_ACC2OFFS/SCAL_*` | 0.036/0.007/0.077 · ≈0.995 | kalibre |

⇒ Kalibrasyon **geçmişte yapılmış**, ArduPilot **artık kabul etmiyor**.
Tipik sebepler: firmware güncellemesi sensör kimliklerini değiştirmiş ·
parametrelerin bir kısmı geri yüklenmiş/sıfırlanmış · sensör değişmiş.
Son günlerde FC'de çok sayıda parametre değiştirildiği biliniyor.

**Eski veriyi kabul ettirmenin yolu yok — kalibrasyon YENİLENMELİ.**

## 🔴 Ama asıl engel kalibrasyon değil: BİLEŞENLER SABİTLENMEMİŞ

Kullanıcı: *"tekneyi çeviremeyiz, içindeki pil veya diğer komponentler
sabitlenmedi daha."* Doğru — 35 Ah'lik paket serbestken tekneyi ters çevirmek
hem tehlikeli hem her şeye zarar verir.

📌 **O sabitleme zaten yapılmak zorunda** (dalgada kayan batarya = kayan ağırlık
merkezi + kopan kablolar). Yani kalibrasyon ek iş değil, **sabitlemenin
arkasına düşen** iş. Sıra kendiliğinden belli:

```
1. Batarya + bileşenleri SABİTLE        ← mekanik ekip, zaten şart
2. İvmeölçer kalibrasyonu (6 konum)      ← tekneyi çevirerek
3. Pusula kalibrasyonu
4. Arm testi — alıcılı ve alıcısız
```

⚠️ İvmeölçer için kısa yol YOK (6 konum zorunlu). Pusula için Mission
Planner'ın **`Large Vehicle MagCal`**'i çevirmeden yapılabilir.
⚠️ Tekneyi çevirirken **Livox kubbesi** ve pruvadaki kamera kaidesi üstüne
ağırlık binmemeli — ya sökülmeli ya elde tutulmalı.

## Neden ÖNCELİKLİ

- **Force arm bir çözüm değil** — hakem karşısında kullanılamaz, ve kalibresiz
  EKF ile tekne yönünü doğru bilmez.
- **Parkur puanını doğrudan etkiliyor:** kapı ortalama hassasiyeti EKF'in tutum
  çözümüne bağlı (md 5.5.4.2 geçiş puanı). `Kotu AHRS` + `EKF variance` varken
  kapı ortasını tutturmak zorlaşır.
- Pixhawk'ı söküp masada kalibre etmek **önerilmez**: tam aynı yönde geri
  takmak zorunlu (2-3° eğim kalıcı tutum hatası), kablo demeti kısaysa zaten
  çevrilemez, USB-C soketi de arızalı (F-M.9). Riski kazancından fazla.

## ✅ İVMEÖLÇER KALİBRASYONU YAPILDI — "Kotu AHRS" VE "FAILSAFE" GİTTİ (20:48)

**Yöntem (kaptanın yöntemi — tekne çevrilmedi):** Pixhawk yuvasından çıkarılıp
**elde 6 konuma çevrildi**. Geçerli, çünkü ivmeölçer kalibrasyonu **sensörün
kendi iç düzeltmesidir** (ofset + ölçek), montaj yönünden bağımsız.
Bağlantı için geçici olarak **USB** kullanıldı (`/dev/ttyACM0` @ 115200) —
kablo demeti 6 konuma çevirmeye yetmedi.

Sonra Pixhawk **yuvasına** geri takıldı (yuva tam oturuyor, zemine paralel, ok
pruvaya bakıyor → yön garantili), **`Seviye Kalibrasyonu`** yapıldı ve FC
`Ctrl+F → Reboot Pixhawk` ile yeniden başlatıldı.

### Reboot sonrası ölçüm

| Uyarı | Önce | Sonra |
|---|---|---|
| `3D Accel calibration needed` | var | ✅ **GİTTİ** |
| **`Kotu AHRS`** | var | ✅ **GİTTİ** |
| **`FAILSAFE`** (EKF) | var | ✅ **GİTTİ** |
| `Compass not calibrated` | var | ⏳ kaldı (dışarıda yapılacak) |

🎯 **TEŞHİS KANITLANDI.** Oturumlardır açık duran "Kotu AHRS" maddesinin sebebi
geçersiz ivmeölçer kalibrasyonuymuş; düzeltilince `EKF variance` → `EKF
failsafe` → `FAILSAFE` zinciri de birlikte çözüldü. Tahmin değil, ölçüm.

> 💡 **Ders:** kalibrasyon verisi parametre dosyasında **duruyor olabilir ama
> ArduPilot onu geçersiz sayabilir.** `INS_ACCOFFS/SCAL` dolu diye "kalibrasyon
> var" denmez — pre-arm mesajı esastır.

⏳ **Kalan:** pusula (`Large Vehicle MagCal`, **dışarıda** — bina demiri
kalibrasyona karışmasın) → sonra arm testi (alıcılı + alıcısız).

### Seviye kalibrasyonu — iki kez yapıldı, ikincisi geçerli

İlk deneme yanıltıcı çıktı: `Trim OK: roll=1.53 pitch=-4.82` (20:43). **4,82°
büyük bir değer** — 103 cm teknede uçtan uca ~8,7 cm. Telefon su terazisiyle
güverte ölçüldü: **öne-arkaya 3,8°** → tekne masada eğik duruyormuş, yani
kalibrasyon teknenin **eğik duruşunu** "düz" diye kaydetmişti.

Burnun altına takoz konup güverte **1,0°**'ye getirildi ve tekrarlandı:

| | Eğik masada (20:43) | **Düzlenmiş (21:08)** |
|---|---|---|
| `roll` | 1.53° | **1.17°** |
| `pitch` | **−4.82°** | **−2.50°** |

Güverte 3,8° → 1,0° inince trim 4,82° → 2,50° indi — **tutarlı**, düzeltme
gerçekten işe yaradı. Kalan −2,50° = yuvanın güverteye göre kendi eğimi
(~1,5°) + güvertede kalan 1,0°; ikisi de trim'e doğru kaydedildi.
Artık ~1°'lik referans hatası kovalanmadı: teknenin sudaki yüzme trimi zaten
birkaç derece, 1° onun içinde kaybolur.

> 🔑 **Kullanıcı sorusu ve cevabı (kayda değer):** *"Tekne suda burnu hafif
> yukarı yüzüyor, bunu hesaba katalım mı?"* **Hayır — ve sebebi ters gibi
> görünüyor:** `AHRS_TRIM`'in işi eğimi GİZLEMEK değil, DOĞRU RAPORLAMAK.
> Kalibrasyon suda yüzdüğü açıda yapılsaydı FC o eğimi **sıfır** sayardı, yani
> teknenin gerçek duruşunu yalan söylerdi. Doğru referans **geometrik düz**
> (güverte yatay); o zaman tekne suda burnu kalkınca FC dürüstçe "burnum X°
> yukarıda" der. **Bonus:** suya inince HUD'daki pitch okunursa teknenin
> gerçek yüzme trimi ÖĞRENİLMİŞ olur (şu an bilinmeyen bir sayı).

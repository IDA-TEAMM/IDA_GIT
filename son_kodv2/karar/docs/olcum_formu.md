# Ölçüm Formu — GİRDAP İDA

> **Durum:** Bu dosya 2026-08-04'te oluşturuldu. Daha önce **7 ayrı dokümandan
> referans veriliyordu ama hiç yazılmamıştı** — bu yüzden ölçümler hiç
> toplanmadı ve `hata_defteri.md` F5.1 maddesi "BLOKE, mekanik `h` ölçüsü
> bekliyor" durumunda kaldı. Referans veren yerler: `fc_parametre_onerileri.md`
> (×2), `ardurover_bench.parm.md`, `hata_defteri.md`, `dogrulama_matrisi.md`
> (×2), `donanim_gunlugu_2026-07-12.md`, `pc_gunlugu_2026-07-12.md`.
>
> **Kime:** mekanik + donanım + FC ekipleri.
> **Nasıl:** `____` alanlarını doldurup commit'leyin. Tahmin YAZMAYIN — ölçün.
> Ölçemediğiniz bir satırı boş bırakın, "yaklaşık" yazmayın (yaklaşık değer
> sessizce doğru sanılır; boşluk en azından görünür kalır).

---

## §0 — `base_link` NEREDE? (önce bu karara varılmalı)

Diğer tüm ölçümler bu noktaya göre alınacak. **Bir kez seçilir, bir daha
değişmez**, ve araç üzerinde fiziksel olarak işaretlenir (bant/kalem).

### Bu teknede yön ve konumlar (2026-08-04'te fotoğrafla teyit edildi)

| | Nerede |
|---|---|
| **Pruva (ön, +x yönü)** | **Kamera yuvasının (mavi kutu) olduğu taraf** |
| **Pixhawk 6C** | Merkez gövdenin içinde, kapak açılınca **sağ tarafta**, gri IP kutunun sağında; üstünde "Pixhawk 6C" yazılı, renkli kablolar ona giriyor |
| **Livox Mid-360** | Merkez gövdenin **tepesinde**, gümüş soğutuculu kubbe |
| **Kamera (OAK-D)** | ⏳ **henüz TAKILMADI** — pruvadaki mavi kutu onun yuvası. Takılınca §3 ölçülecek |
| Kırmızı acil stop | Merkez gövdenin üstünde (md 4.2 "araç üzerinden güç kesme" ✅ mevcut) |
| GPS (F9P) | Merkez gövdenin sağında, kısa kol üzerinde yuvarlak anten |
| Telemetri anteni | Merkez gövdenin solunda, siyah çubuk |

**Eksen kuralı (ROS REP-103, değiştirilemez):**
- **+x = PRUVA** (ileri, teknenin burnu)
- **+y = İSKELE** (sol taraf)
- **+z = YUKARI**
- Açı birimi: **derece**, saat yönünün TERSİ (+) — yani +yaw = sola dönüş

### ✅ KARAR (2026-08-04): `base_link` = **teknenin geometrik merkezi**

| Eksen | Nerede | Ölçülen |
|---|---|---|
| `x` | Gövde **boy ortası** | Ön uçtan **51.5 cm** (tekne boyu 103 cm) |
| `y` | Gövde **merkez hattı** | LiDAR zaten tam orada |
| `z` | Gövde **tabanı** | Tekne masadayken masa yüzeyi |

> **Neden Pixhawk değil?** İlk öneri Pixhawk'tı; gerekçe "ArduPilot IMU'yu araç
> orijini sayar" idi. **Bu eksikti:** mutlak konumu **GPS çiviliyor** ve
> `GPS1_POS_*` sıfırken EKF, anten ölçümünü olduğu gibi kabul eder — yani
> raporlanan konum pratikte **GPS anteninin** konumudur, Pixhawk'ın değil.
> Bu teknede GPS anteni sancakta bir kol üzerinde, yani orijin daha da kayık.
>
> Ayrıca ölçüm gösterdi ki Pixhawk gövde merkez hattının **13.75 cm
> iskelesinde**. `base_link` orada olsaydı kapı ortasına sürerken tekne gövdesi
> 13.75 cm sancağa kaymış geçerdi (aşağıdaki hesap).
>
> **Doğru kurulum:** orijini biz seçeriz (gövde merkezi), ArduPilot'a da
> "GPS şurada, IMU şurada" deriz → EKF gövde merkezini raporlar, kayma
> kökünden biter.

**⏳ FC'ye girilecek parametreler — hesaplandı, girilmeyi bekliyor:**

```
INS_POS1_X   -0.055     GPS1_POS_X   -0.035
INS_POS1_Y   -0.1375    GPS1_POS_Y   -0.16
INS_POS1_Z   -0.155     GPS1_POS_Z   -0.365
```

> 🔴 **PARAMETRE ADI (2026-08-06 düzeltmesi):** GPS ofsetleri bu firmware'de
> **`GPS1_POS_X/Y/Z`** — bu form önceden **`GPS_POS1_*`** yazıyordu ve
> **öyle bir parametre YOK** (04.08 tarihli 941 satırlık FC dökümünde
> bulunmuyor). Yanlış adı Mission Planner'a yazan kişi parametreyi bulamaz,
> ofset **hiç uygulanmaz** ve herkes yapıldı sanır. `INS_POS1_*` doğruydu.

Türetildiği fiziksel gerçek (gövde merkezine göre):

| | ileri/geri | sağ/sol | yükseklik |
|---|---|---|---|
| Pixhawk (IMU) | 5.5 cm **kıçta** | 13.75 cm **iskelede** | 15.5 cm **yukarıda** |
| GPS anteni | 3.5 cm **kıçta** | 16 cm **iskelede** | 36.5 cm **yukarıda** |

> İşaretler ArduPilot eksenine çevrilmiştir (Y sancak +, Z aşağı +) — bu yüzden
> "iskelede" ve "yukarıda" olan değerler **negatif** yazılıyor.

> ⚠️ **Doğrulanacak varsayım:** ArduPilot dokümanı bu ofsetleri aracın **ağırlık
> merkezine** göre tanımlar; biz **geometrik merkezi** kullandık. Küçük bir
> teknede ikisi yakındır ve düzeltmeyi asıl belirleyen GPS↔IMU arasındaki
> **göreli** geometridir (o doğru). Ağırlık merkezi belirgin şekilde başka
> yerdeyse değerler ötelenir.

> 🔴 **Bu parametreler girilene kadar** `base_link` tanımı ile odom'un
> raporladığı nokta ÇAKIŞMIYOR — sistemde bilinen sabit bir ofset var.
> Girildikten sonra masa testinde konumun beklendiği gibi davrandığı
> doğrulanmalı (ArduPilot'un ofseti fiilen uyguladığı gözle görülmeden
> "tamam" denmeyecek).

**⚠️ EKSEN TUZAĞI — ArduPilot ile ROS ters:**

| | ROS / TF (`hardware.yaml`) | ArduPilot `INS_POS`/`GPS_POS` |
|---|---|---|
| X | ileri + | ileri + (aynı) |
| Y | **sol (iskele) +** | **sağ (sancak) +** ← ters |
| Z | **yukarı +** | **aşağı +** ← ters |

Aynı fiziksel konum iki dosyada farklı işaretle yazılır. Ölçümü hep fiziksel
olarak ("şu kadar sağda", "şu kadar yukarıda") not edin, çevirmeyi tek yerde
yapın.

Farklı bir yer seçilirse buraya yazın:

| | Değer |
|---|---|
| `base_link` seçilen konum (tarif) | `________________________________` |
| Araç üzerinde işaretlendi mi? | ☐ evet |

---

## §0.5 — NASIL ÖLÇÜLÜR (yöntem)

**Malzeme:** şerit metre · **çekül** (ipe bağlı somun yeter) · uzun sicim ·
maskeleme bandı + keçeli kalem · su terazisi (telefon uygulaması olur) ·
düz bir tahta/gönye.

**Altın kural: 3B'de çapraz ölçmeyin.** Her şeyi çekülle güverteye indirip
düzlemde (2B) ölçün — çapraz ölçüm hem zor hem hatalı.

### Hazırlık
1. Tekneyi **düz zemine** koyun, sallanmasın diye takoz koyun.
2. Su terazisiyle güvertenin yatay olduğunu doğrulayın (sağa-sola ve
   öne-arkaya). Eğikse ölçümler bozulur.
3. **`base_link`'i işaretle:** Pixhawk gövdesinin merkezinden çekül sarkıtın,
   ucun güverteye değdiği noktayı bantla işaretleyin. Üstüne "BL" yazın.
4. **Merkez hattını ger:** pruvanın tam ortasından kıçın tam ortasına sicim
   gerin. Bu **+x ekseni**. BL bu sicimin üstünde olmalı (değilse Pixhawk
   merkezde değil demektir — sorun değil, y'sini ölçeceğiz).

### Her sensör için (LiDAR, kamera)
5. Sensörün **referans noktasından** çekül sarkıtın, yere düştüğü yeri
   işaretleyin:
   - **Livox Mid-360:** silindirik gövdenin merkezi, lazer penceresinin orta
     yüksekliği
   - **OAK-D Lite:** ortadaki (RGB) lensin merkezi
6. **`x`** = BL ile sensör işaretinin **sicim boyunca** mesafesi.
   Sensör BL'nin önündeyse **+**, arkasındaysa **−**.
7. **`y`** = sensör işaretinin sicime **dik** mesafesi.
   Sicimin **solunda** (iskele) **+**, sağında (sancak) **−**.
8. **`z`** = yerdeki işaretten sensörün referans noktasına kadar **dik yukarı**
   mesafe. Metreyi düz tutun (tahtayla dayayın).

> **Hassasiyet:** ±2 cm yeter. LiDAR cluster toleransı 0.5 m, duba yarıçapı
> 0.15 m — santimin altını kovalamayın. **İstisna: `z`** — F5.1 filtresini o
> belirliyor, onu dikkatli ölçün.

### Yaw (dönüklük) — mekanik değil, AMPİRİK ölçün
Yaw'ı iletkiyle ölçmeye çalışmayın: **20 m'de 5°'lik hata 1.7 m yanal kayma**
demek, ama 5°'yi elle ölçmek zordur. Bunun yerine:

1. Mekanik olarak: sensör braketi gövdeye hizalı monte edildiyse `yaw ≈ 0`
   yazın (başlangıç değeri).
2. **Sonra doğrulayın:** merkez hattının tam üzerine, ~10 m ileriye bir duba
   (ya da kova/direk) koyun. Yığın açıkken:
   ```bash
   ros2 topic echo /perception/obstacle_map --once
   ```
   Tek engelin `position.y`'si **≈ 0** olmalı. Değilse:
   `yaw_hata(derece) = atan2(y, x) × 180/π` → bulduğunuz açıyı `tf` bloğuna
   (radyan olarak) girin ve tekrarlayın.
3. Aynı testi kamera için: duba görüntünün **tam ortasında** olmalı.

> Bu yöntem hem daha doğru hem de montaj/optik eksen farklarını da yakalar.

### Pitch (yalnız kamera)
Telefonun eğim/su terazisi uygulamasını kameranın **düz üst yüzeyine** koyun.
Ufka göre aşağı bakıyorsa **negatif** yazın. Duba mesafe tahminini doğrudan
etkiler.

---

## §1 — Tekne gövde ölçüleri

`gate_follower` bu iki sayıyı kullanıyor (kapı geçilebilirlik testi — eşik
ayarı değil, fizik). Şu an kodda `hull_width_m=0.78`, `hull_length_m=1.04`
yazıyor (kaynak: GIRDAP_DURUM §1). **Teyit edin:**

| Ölçü | Kodda | Ölçülen | Not |
|---|---|---|---|
| Gövde genişliği (uçtan uca, en geniş yer) | 0.78 m | `____` m | Kapıdan sığma hesabı |
| Gövde boyu | 1.04 m | **1.03 m** ✅ | Ölçüldü 2026-08-04; koddaki 1.04 ile 1 cm fark → tolerans içinde, kod değişmiyor |
| Şamandıra/fender vb. ile toplam genişlik | — | `____` m | Çarpma payı için |

---

## §2 — 🔴 LiDAR (Livox Mid-360) — EN KRİTİK ÖLÇÜM

> ✅ **B0/F5.1 KAPANDI (2026-08-06).** `perception_lidar_node` artık ham
> bulutu **base_link'e taşıyor** (`sensor_to_base`), sonra filtreliyor.
> Taşıma miktarı `hardware.yaml` **`tf.livox_frame`** bloğundan gelir (static
> TF yayıncısıyla tek kaynak) — bu formdaki sayılar oraya girildiği sürece
> doğru çalışır.
>
> *Neydi:* node hiçbir TF uygulamıyor, çıktının `frame_id`'sini `base_link`
> diye **etiketliyordu**. `z_min=0.1` "base_link'e göre su üstü kesim" diye
> tanımlı olduğu için LiDAR gövdenin üstündeyken dubalar LiDAR çerçevesinde
> **negatif z**'de kalıyor ve filtre **hepsini eliyordu** → `obstacle_map` boş.
>
> ⚠️ **Sıradaki eksik `yaw`.** Öteleme girildi; dönüklük hâlâ `____`. Yaw
> yanlışsa engeller açı kadar KAYAR (10 m'de 5° → 0.87 m) — kapı ortası
> hesabını doğrudan bozar. Ampirik ölçüm §0.5'te.

| Ölçü | Değer | Nasıl ölçülür |
|---|---|---|
| `x` — `base_link`'ten ileri (+) / geri (−) | **+0.015 m** ✅ | gövde boy ortasından 1.5 cm pruvada |
| `y` — iskeleye (+) / sancağa (−) | **0.0 m** ✅ | LiDAR gövde merkez hattında |
| **`z` — `base_link`'ten yukarı (h)** | **+0.41 m** ✅ | 🔴 B0'ı çözen sayı — gövde TABANINDAN, aşağıdaki hesap |
| `yaw` — pruvaya göre dönüklük | `____` ° ⏳ | **Ampirik ölç** (§0.5) — iletkiyle uğraşma |
| `pitch` / `roll` — eğik monte edildi mi? | 0 / 0 | Düz monte |
| **Su hattından yükseklik** (yüklü tekne, sakin su) | `____` m ⏳ | İlk suya inişte; `z_min`'in doğru değerini bu belirler |
| Gövdenin LiDAR görüş alanına giren kısmı var mı? | ☐ var ☐ yok | Varsa `min_range` filtresi gerekir (F5.1 ile birlikte) |

> 🔴 **ÇERÇEVE DÜZELTMESİ (2026-08-06):** yukarıdaki satırlar 05.08'e kadar
> **eski `base_link`'e (= Pixhawk)** göre yazılıydı (x +0.07 · y −0.1375 ·
> z +0.255). `base_link` §0'da **gövde merkezine** taşındığı için o sayılar
> artık yanlış referanstaydı; formu okuyup doğrudan giren kişi **yanlış ofset
> yazardı**. Yürürlükteki doğru değerler `hardware.yaml tf.livox_frame` ile
> birebir: `{x: 0.015, y: 0.0, z: 0.41}`. Aşağıdaki masa hesabı Pixhawk'a
> göreli ara adımdır — sonucu gövde tabanına çevirmek gerekir (41.0 cm ölçümü
> zaten masadan, yani gövde tabanından).

**`z` nasıl bulundu (2026-08-04, tekne masada):**
```
LiDAR masadan                        41.0 cm
kapak ağzı masadan                   24.5 cm
kapak ağzı tekne tabanından          15.0 cm
Pixhawk tekne tabanından (3D platform) 6.0 cm
→ Pixhawk kapak ağzından 15.0−6.0  =  9.0 cm aşağıda
→ Pixhawk masadan       24.5−9.0   = 15.5 cm
→ z = 41.0 − 15.5                  = 25.5 cm
```

> **F5.1 bu sayıyla kesinleşti (ve 06.08'de kodda kapatıldı):** `z_min=0.10 m` ham LiDAR çerçevesinde
> uygulanıyor (node TF dönüşümü yapmıyor) → "LiDAR'ın 10 cm ÜSTÜ" demek.
> Duba suyun üstünde ~30 cm; LiDAR ise su hattından ~40 cm+ yukarıda. Yani
> dubanın **tepesi bile** LiDAR'ın altında kalıyor → tüm noktalar negatif
> z'de → filtre hepsini eliyor → `obstacle_map` boş. Atölyedeki "üretim
> config'de 0 engel" gözlemiyle birebir uyuşuyor.

---

### GPS anteni (Holybro H-RTK F9P) — mutlak konumu bu çiviliyor

| Ölçü | Değer |
|---|---|
| Ön uçtan mesafe | 55.0 cm → gövde merkezinden **3.5 cm kıçta** |
| Merkez hattından | **16 cm iskelede** (Pixhawk ile aynı taraf) |
| Masadan (gövde tabanından) yükseklik | **36.5 cm** |

> `GPS1_POS_*`'e girilecek → §0'daki parametre bloğu.

---

### 🔴 Gövde merkezi ofseti — ölçümden türeyen YENİ açık konu

Ölçüm şunu ortaya çıkardı: **LiDAR gövde merkez hattında, Pixhawk 13.75 cm
iskelede.** Yani `base_link` (= Pixhawk = odom'un bildirdiği nokta) teknenin
geometrik merkezinde DEĞİL.

**Etkisi:** Kapı ortasına `base_link`'i sürersek, teknenin gövdesi ortadan
13.75 cm **sancağa kaymış** olarak geçer.

```
kapı açıklığı (md 5.5.2.1'e göre değişken, ~1.35 m varsayımı)   1.350 m
tekne genişliği                                                 0.780 m
→ ideal yan pay (her iki yanda)                                 0.285 m
base_link ofseti yüzünden sancak payı  0.285 − 0.1375       =   0.148 m
                                       iskele payı           =   0.423 m
```

Yani sancak tarafındaki emniyet payı **yarıya iniyor**. Şartname kapı
genişliğinin alana göre değişeceğini söylüyor (md 5.5.2.1/5.5.3.1) — dar bir
kapıda bu fark çarpma cezasına dönüşebilir (`Ç1`/`Ç2`).

**İki çözüm yolu (karar verilecek):**

| | Yöntem | Artı / Eksi |
|---|---|---|
| **A** | ArduPilot `INS_POS_Y` ile IMU'nun gövde merkezine göre ofsetini gir → EKF gövde merkezini raporlasın | Tek parametre; ama TF değerleri de yeniden referanslanmalı. **ArduPilot'un konumu gerçekten düzelttiği masa testinde DOĞRULANMALI** |
| **B** | `base_link` Pixhawk'ta kalsın, kapı hedefine 13.75 cm yanal düzeltme uygulansın | Kod değişikliği, `gate_follower`/`planning_node` = **karar ekibinin alanı** — onlara bildirilecek |

⚠️ Bu konu kapanmadan suya inilirse dar kapılarda sancak tarafı riskli.

---

## §3 — Kamera (OAK-D Lite)

| Ölçü | Değer | Not |
|---|---|---|
| `x` — ileri (+) | `____` m | |
| `y` — iskele (+) | `____` m | |
| `z` — yukarı | `____` m | |
| `yaw` — pruvaya göre | `____` ° | |
| `pitch` — ufka göre (aşağı bakıyorsa −) | `____` ° | Duba mesafesi tahminini doğrudan etkiler |
| Lens ekseni su hattından yükseklik | `____` m | |

---

## §4 — ESC (bkz. `ardurover_bench.parm.md` → ESC KALİBRASYONU)

Tekne **2 motorlu** (sol/sağ). Her iki ESC de aynı model olmalı.

| | Değer |
|---|---|
| Marka / model | **markasız jenerik "Bidirectional ESC 50A"** (motorobit.com, su altı motoru uyumlu) ✅ 2026-08-06. Etiket: `50A` · `BEC 2A 5V` · `LIPO 2S-4S`. **Üretici manuali YOK** — bkz. `ardurover_bench.parm.md` ESC adım 3 |
| **Tek yönlü mü, çift yönlü (reversible) mi?** | ☑ **ÇİFT YÖNLÜ** ✅ (2026-08-04) |
| Akım değeri | **50 A** ✅ |
| PWM aralığı | min **1100** / nötr **1500** / max **1900** µs |
| **FC tarafı simetri** (`SERVOn_MIN/MAX/TRIM` iki kanalda aynı mı?) | ☑ **EVET** ✅ (2026-08-06, canlı param dökümünden) |
| Kalibrasyon yapıldı mı (ikisi de, aynı prosedürle)? | ☑ **YAPILMAYACAK** — bilinçli karar ✅ 2026-08-06. Çift yönlü ESC'de klasik gaz kalibrasyonu nötrü kaydırabilir; nötrümüz zaten doğru ve simetrik. Gerekçe + kalan risk: `ardurover_bench.parm.md` ESC adım 3 |
| **Thruster tepe akımı (tek motor)** | `____` A |
| **2 motor toplam tepe akım** | `____` A |

> Son iki satır uzaktan güç kesme kontaktörünün boyutlandırılması için gerekli
> (`fc_parametre_onerileri.md` §4.5).

---

## §5 — FC parametre okuması

`fc_parametre_onerileri.md` tablolarındaki **"Mevcut" sütunları boş.** Tek tek
yazmak yerine tüm listeyi dosyaya dökün:

```
Mission Planner → Config → Full Parameter List → Save to file
→ docs/fc_mevcut_parametreler_<YYYY-MM-DD>.param  (repoya commit)
```

| | Değer |
|---|---|
| Dosya alındı mı? | ☐ evet, tarih: `______` |
| ArduRover firmware sürümü | `________` |
| RC kalibrasyonu baştan yapıldı mı? | ☐ evet (CH2/CH3 uçlarda dinliyordu — `RCMAP_*` teyidi şart) |

---

## §6 — Güç sistemi

| | Değer | Not |
|---|---|---|
| Batarya konfigürasyonu | `________` | 4S7P bekleniyor, teyit |
| Nominal / dolu / boş voltaj | `____` / `____` / `____` V | `BATT_LOW_VOLT` için |
| Ana sigorta değeri | `____` A | |
| Motor kolu ile FC/Jetson kolu **ayrı mı?** | ☐ evet ☐ hayır | 🔴 Kontaktör için ŞART (`fc_parametre_onerileri.md` §4.5) |

---

## Ölçüm sonrası — bu değerler nereye girilecek

| Ölçüm | Gideceği yer |
|---|---|
| §2 LiDAR x/y/z/yaw | `hardware.launch.py` → `_static_tf("base_link", "livox_frame")` argümanları |
| §2 `h` (z) | F5.1 — `lidar_height_m`; **perception ekibine bildir** (`perception_lidar_node` bu bölgede çalışıyor) |
| §3 Kamera x/y/z/yaw/pitch | `_static_tf("base_link", "oak_frame")` |
| §1 gövde | `gate_follower` `hull_width_m` / `hull_length_m` teyidi |
| §4 ESC PWM | `MOT_PWM_MIN` / `MOT_PWM_MAX` tutarlılığı |
| §4 akım | Kontaktör seçimi |
| §5 | `fc_parametre_onerileri.md` "Mevcut" sütunları |
| §6 | `BATT_*` failsafe parametreleri |

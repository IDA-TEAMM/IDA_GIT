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

> 🔴 **DATUM UYARISI (2026-08-09) — `z = 0` DIŞ GÖVDE TABANIDIR, İÇ PLATFORM
> DEĞİL.** Su alma tehlikesine karşı teknenin **içine ~2 cm yüksekliğinde bir
> platform** yapıldı ve bileşenler onun üzerine sabitlendi. Bu platform artık
> gözle bakınca "iç zemin" gibi görünüyor ve **ölçüm datum'u sanılmaya son
> derece müsait**. Ondan ölçen kişi bütün `z` değerlerini **2 cm eksik**
> okur — ve fark etmez, çünkü sayılar makul görünür.
>
> `z = 0` **tekne masadayken masa yüzeyine değen dış taban**. Platform içeride
> olduğu için dış taban KIPIRDAMADI, dolayısıyla bu formdaki hiçbir `z`
> değişmedi. Ölçerken metreyi **zemine/masaya** dayayın, platforma değil.
>
> **Platformdan ETKİLENMEYEN (yükseklikleri aynı kaldı, ölçümler geçerli):**
> **Pixhawk** (kendi 3D baskı platformu zaten vardı, ölçüm onunla alınmıştı) ·
> **Livox Mid-360** · **GPS anteni**.
> **Platformla ~2 cm YÜKSELEN:** ESC'ler · güç kartı · diğer iç bileşenler.
> Bunların hiçbiri TF'e girmiyor → **kod/parametre değişikliği gerekmez.**
>
> ✅ **Ağırlık merkezi — teyit edildi, ihmal edilebilir (2026-08-09).**
> **Batarya YÜKSELMEDİ**, o da aynı yükseklikte kaldı (kullanıcı teyidi).
> Baskın kütle (4S7P, 28× INR21700-50S) yerinde olduğu için AM kayması yalnız
> ESC + güç kartının yer değiştirmesi kadar — ikisi de hafif. Aşağıdaki
> "ArduPilot AM'ye göre tanımlar, biz geometrik merkezi kullandık" varsayımı
> bu yüzden **gerilmedi**; düzeltmeyi asıl belirleyen GPS↔IMU arası **göreli**
> geometri de hiç değişmedi. Ek işlem gerekmez.
>
> ⏳ Platformun eklediği ağırlık **su çekimini** de artırır → §2'deki "LiDAR'ın
> su hattından yüksekliği" ölçümü **platform takılıyken** alınmalı (zaten
> alınmadı, ilk suya inişte alınacak).

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
Telefonun eğim/su terazisi uygulamasını kameranın **düz üst yüzeyine** koyun
(kaidenin değil — kamera kaideye göre yatık olabilir). Kameranın üst yüzü optik
eksene **paralel** olduğu için okuduğunuz eğim doğrudan pitch'tir.

🔴 **İŞARET:** ufka göre **aşağı** bakıyorsa **ARTI**, **yukarı** bakıyorsa
**EKSİ** (REP-103 / `static_transform_publisher` kuralı). Bu satır 09.08'e kadar
tersini söylüyordu — bkz. §3'teki kırmızı not. Emin olmak için açıyı **fiziksel
olarak** ("yukarı/aşağı, şu kadar derece") not edip çevrimi tek yerde yapın.

Duba mesafesi tahminini doğrudan etkiler.

---

## §1 — Tekne gövde ölçüleri

`gate_follower` bu iki sayıyı kullanıyor (kapı geçilebilirlik testi — eşik
ayarı değil, fizik). Kaynağı belirsiz olan eski değerler `hull_width_m=0.78`,
`hull_length_m=1.04` idi (kaynak: GIRDAP_DURUM §1). **İkisi de ölçüldü:**

| Ölçü | Eski (kodda) | Ölçülen | Not |
|---|---|---|---|
| Gövde genişliği (uçtan uca, en geniş yer) | 0.78 m | **0.785 m** ✅ | Ölçüldü 2026-08-09 (78,5 cm, şeritle, duba yüksekliği bandında). 5 mm fark → **kod 4 yerde güncellendi** (aşağıya bak) |
| Gövde boyu | 1.04 m | **1.03 m** ✅ | Ölçüldü 2026-08-04; koddaki 1.04 ile 1 cm fark → tolerans içinde, kod değişmiyor |
| Şamandıra/fender vb. ile toplam genişlik | — | **0.785 m** ✅ | 2026-08-09: **hiçbir çıkıntı YOK** (fender/şamandıra/cıvata) → toplam genişlik = gövdenin kendisi. Ayrı bir çarpma payı gerekmiyor |

> **Genişlik neden boyun aksine koda işlendi (5 mm için):** boy yalnız burun
> hattı (`hull_length_m/2`) hesabında kullanılıyor, genişlik ise **iki** yerde
> emniyet payı üretiyor — `min_passable_width` (kapı geçilebilir mi) ve
> `planning_node._huni_payi` (kapı direği ceza halkası, 09.08). Huni payı
> ölçümü gövde payını **−0,006 m** bulmuştu; 6 mm'lik bir marjda 5 mm'yi
> yuvarlamak doğru değil. **Yön güvenli tarafta:** büyük genişlik = daha
> muhafazakâr.
>
> Güncellenen 4 yer (drift testi dördünü birbirine bağlar, biri kalırsa CI
> kırmızı): `prototype/mission/gate_follower.py` `GateFollowerConfig` ·
> `launch/hardware.launch.py` `_GATE_DEFAULTS` · `config/params.yaml` ·
> `config/hardware.yaml`.

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
tekne genişliği (ölçüldü 09.08)                                 0.785 m
→ ideal yan pay (her iki yanda)                                 0.2825 m
base_link ofseti yüzünden sancak payı  0.2825 − 0.1375      =   0.145 m
                                       iskele payı           =   0.420 m
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

**Referans nokta: ortadaki (RGB) lensin merkezi.** Kaide değil, kutunun kenarı
değil — füzyon bbox merkezini o lense göre hesaplıyor. Lens **açıkta**, önünde
cam/muhafaza YOK (2026-08-09 fotoğraf teyidi) → vekil nokta + offset hesabı
gerekmedi, doğrudan lensten ölçüldü.

| Ölçü | Değer | Not |
|---|---|---|
| `x` — ileri (+) | **+0.185 m** ✅ | Ön uçlara dayanan **dikey levhadan**, lens hizasında yatay: **33 cm** → `51.5 − 33` |
| `y` — iskele (+) | **0.000 m** ✅ | Lensten sol dışa **39.5**, sağ dışa **39.0** cm → 2.5 mm sancak sapması, tolerans altı |
| `z` — yukarı | **+0.280 m** ✅ | Masadan lens merkezine **28 cm**; tekne doğrudan masada (takoz YOK) |
| `yaw` — pruvaya göre | **0.0** 🟡 | Kaide düzgün vidalandı (mekanik). ⏳ Suda ampirik doğrulanacak (§0.5) |
| `pitch` — ufka göre | **0.0** 🟡 | **AMPİRİK ölçüldü ±5°** — aşağıdaki ufuk yöntemi. İşaret kuralı için kırmızı nota bak |
| `roll` | **0.0** | Ufuk karelerde yatay görünüyor |
| Lens ekseni su hattından yükseklik | `____` m ⏳ | İlk suya inişte |

> ✅ **`pitch` AMPİRİK ÖLÇÜLDÜ (2026-08-09) — mekanik izlenim YANLIŞTI.**
> Montaj fotoğrafına bakıp *"kamera geriye yatık, yukarı bakıyor"* denmişti;
> o kare önden-yukarıdan çekildiği ve kaidenin arka duvarı yüksek olduğu için
> yanıltıcıydı. **Doğrusu: kamera düz ileri bakıyor.**
>
> **Yöntem — ufuk çizgisi.** Kameranın 08.08'de gölde (tekne SUDA yüzerken,
> **aynı kaidede**, kaide o gün sabit) çektiği dataset karelerinde ufkun kare
> içindeki dikey konumu okundu:
>
> | kare | kare boyu | orta | ufuk | sapma | `pitch` |
> |---|---|---|---|---|---|
> | A | 700 px | 350 | ~345 | −5 px | −0.4° |
> | B | 700 px | 350 | ~352 | +2 px | +0.2° |
>
> `pitch = atan(sapma / f)`, `f ≈ 687 px` (4:3 karede HFOV 68.8°'den).
> Mekanik teyit: kaide **yere paralel** vidalanmış (kullanıcı). İki bağımsız
> yol aynı sonucu verdi.
>
> ⚠️ **Neden ±5°, neden daha iyisi değil:** eldeki kareler WhatsApp + Windows
> Photos'tan geçmiş, boyutları tutarsız (946×707 / 941×700 / 934×700) ve
> köşelerinde damga var → **kırpılmış olabilirler**, kırpma merkezi kaydırır.
> ⏳ **Orijinal (ham) dosyalarla ±0.5°'ye indirilebilir** — ufuk satırı
> programatik ölçülür. Dataset klasöründeki ham kareler gerekiyor.
>
> ⚠️ Ufuk yöntemi kameranın **dünyaya göre** açısını verir; TF'e gereken
> **gövdeye göre**. Aradaki fark teknenin sudaki **trim**'i. Katamaran durgun
> suda düz oturur, fark küçüktür — ±5° bandı bunu da kapsıyor.

> 🔀 **BU ÖLÇÜMDEN ÇIKAN, DİĞER EKİPLERİ İLGİLENDİREN BULGULAR (09.08)**
> Karar ekibinin işi değil ama dataset karelerinde görüldü, iletilmeli:
>
> 1. 🔴 **Dubaların üstünde BAYRAK var.** LiDAR tarafı dubayı tek küme sayıyor
>    (`min_cluster_size: 5`, `cluster_tolerance: 0.5 m`). Bayrak direği ince
>    olduğu için gövde ile bayrak **arasında dönüş olmaz** → bir duba **iki
>    ayrı kümeye** bölünebilir. `gate_follower`'ın açıkça korunmaya çalıştığı
>    hata bu ("tek dubanın iki tespite bölünmesi" → sahte kapı çifti).
>    **→ Yahya (LiDAR/karar).**
> 2. ✅ **DATASET 416×416 — KONTROL EDİLDİ, ÜÇ ENDİŞE DE ZATEN ÇÖZÜLMÜŞ.**
>    Aşağıdaki üç madde (2-eski/2b/2c) 09.08'de ham risk olarak yazılmıştı;
>    aynı gün `algi/` aynasındaki kod okunup **üçünün de Eyüp tarafından
>    çözüldüğü** görüldü. Tarihsel kayıt olarak bırakıldı ama **AÇIK İŞ
>    DEĞİLLER — kimseye iletilmeyecek.** Çözümleri:
>
>    | Endişe | Durum | Mekanizma |
>    |---|---|---|
>    | bbox 416-uzayında gelir mi? | ✅ hayır | `duba_gecis_navigator.py` → `BBOX_W, BBOX_H = 1280, 720` **sabit**; `bbox_piksel()` NN-normalize bbox'ı bu uzaya çevirir. Bizim `camera_image_width_px` ile eşleşmek üzere **bilinçli** seçilmiş, kodda `hardware.yaml:207-208`'e referans var |
>    | HFOV kırpmadan 42°'ye düşer mi? | ✅ hayır | `setPreviewKeepAspectRatio(False)` → 4:3 kare 416×416'ya **EZİLİR** (kırpılmaz) → **tam FOV korunur**, letterbox payı 0 → `camera_hfov_rad: 1.2` **doğru** |
>    | 416 yerine 640 kullanılsa? | ✅ kullanılamaz | 416 **bilinçli**: Myriad X VPU sınırı. YOLO11n 416 + StereoDepth = **11 FPS**, ölçülen tavan **12,2**. 640 bu cihazda bütçeyi aşar. Önerim geçersizdi |
>
>    ⚠️ Ayrıca Eyüp **tam bu arızayı** kendi kodunda zaten belgelemiş, birebir
>    aynı aritmetikle: *"640 yayınlayıp 1280'e bölmek → kare ortasındaki duba
>    +17°'de görünür, tolerans 8,6° → hiçbir LiDAR kümesine eşleşmez → sınıf
>    düşer → geçit bulunamaz"*. Yani sözleşme iki taraftan da korunuyor.
>
>    **Ders:** `algi/` bu repoda **ayna olarak duruyor** — karar tarafında bir
>    algı endişesi doğduğunda önce oradaki kod ve `KAYNAK.md` topic sözleşmesi
>    okunmalı, sonra soru sorulmalı. Üç madde de o adım atlandığı için
>    gereksiz "acil" olarak yazıldı.
>
> 2-eski. ~~🔴 **DATASET 416×416 — bbox KOORDİNAT UZAYI ACİLEN TEYİT EDİLMELİ.**~~
>    Kullanıcı bildirdi (09.08): eğitilecek dataset görüntüleri **416×416**.
>    `perception_fusion_node.py:360` şunu yapıyor:
>    ```python
>    bbox_cx = det.bbox.center.position.x / self._image_w   # _image_w = 1280
>    ```
>    Yani `/perception/buoys` bbox'ı **1280 genişlikli görüntünün pikselinde**
>    olmak ZORUNDA. Tespitler 416-uzayında yayınlanırsa:
>    ```
>    görüntü merkezi 208 → 208/1280 = 0.1625
>    bearing = (0.5 − 0.1625) × 1.2 = 0.405 rad = 23° SOLA
>    ```
>    → **her tespit sürekli 23° solda** raporlanır, saçılma üçte bire sıkışır,
>    eşleşme toleransı (0.15 rad = 8.6°) hepsini ya kaçırır ya YANLIŞ eşler.
>    **Hiçbir uyarı basılmaz** — sessiz arıza. Sonuç: sarı engel "kapı direği"
>    sınıfı alabilir, `planning_node` onu engel torbasından çıkarır, araç
>    engele sürer (Ç1/Ç2).
>
>    **Sorulacak (Eyüp):** `/perception/buoys` bbox'ı hangi uzayda —
>    **orijinal kare** (1280×720) mı, **model girdisi** (416×416) mı?
>    *Ultralytics normalde kutuları orijinal boyuta geri ölçekler; ama
>    `.pt → blob` zinciriyle DepthAI/Myriad X üzerinde koşturulunca bu
>    otomatik DEĞİLDİR* (bkz. commit `14efcef`).
>
> 2b. 🔴 **416 KARE → HFOV ARTIK 68.8° OLMAYABİLİR.** DepthAI kare (1:1) NN
>    girdisi üretirken iki yol var ve `camera_hfov_rad`'ı **1.63× değiştirir:**
>
>    | Yol | HFOV | `camera_hfov_rad` |
>    |---|---|---|
>    | **Kırp** (`setPreviewKeepAspectRatio(True)`, VARSAYILAN) | **42.1°** | **0.735** |
>    | **Ez/gerdir** (`False`) | 68.8° (korunur) | 1.2 (mevcut) |
>
>    Kırpma yapılıyorsa mevcut `1.2` bütün kerterizleri **1.63× şişirir** —
>    ayrıca yatay görüş alanının %44'ü kaybedilir (kenardaki kapı direkleri
>    hiç görünmez). Teyit edilmeli.
>
> 2c. 🟡 **416 tespit menzilini sınırlıyor.** Şartname dubası 30 cm çap.
>    Model girdisinde dubanın piksel genişliği `≈ D·W/(HFOV·d)`:
>
>    | mesafe | 416 (gerdirme) | 640 (gerdirme) |
>    |---|---|---|
>    | 10 m | 10.4 px | 16.0 px |
>    | 15 m | **6.9 px** | 10.7 px |
>    | 25 m | 4.2 px | 6.4 px |
>
>    YOLO'nun küçük nesne tabanı pratikte ~8-10 px. **416'da 15 m sınırda,
>    640'ta güvenli.** Yükseklikte durum iyi (duba 50 cm + BAYRAK → 15 m'de
>    ~20-40 px), yani ince-uzun bir hedef; renk de ayırt edici. Yani 416
>    çalışabilir ama menzil payı dar. P1 kapalı-döngü ölçümü "kamera 15 m"
>    ile koşuldu — o menzil bu çözünürlükte marjinal.
>    Ek not: 16:9'u **kareye letterbox**'lamak satırların **%44'ünü dolgu**
>    yapar; dikdörtgen `imgsz` (örn. 640×384) hem pikseli daha iyi kullanır
>    hem 640×640'tan hızlıdır. **→ Eyüp'ün kararı, karar ekibi sadece bildirir.**
> 3. ⚪ **Karar tarafının HSV eşikleri — YARIŞMA YOLUNDA KULLANILMIYOR, konu
>    değil.** `camera_buoys.py` docstring'i kırmızı/yeşil/kahverengi için
>    *"sahada doğrulanmadı, kör güvenilmemeli"* diyor ve bu doğru; ama o kod
>    `perception_camera_node`'da yaşıyor ve node yalnız
>    `use_onboard_camera:=true` ile açılıyor — **varsayılan `false`**
>    (`hardware.launch.py:488`). Yarışmada `/perception/buoys`'u **Eyüp'ün OAK
>    node'u** üretiyor (VPU'da YOLO11n), HSV yolu hiç koşmuyor. İkisi birden
>    açılırsa OAK-D USB cihazını iki süreç açamaz — varsayılanın `false`
>    olması bu yüzden **doğru ayar**, değiştirilmemeli.

> ✅ **Beklenmedik çapraz teyit:** `39.5 + 39.0 = 78.5 cm` — bu **tam olarak**
> §1'de ölçülen gövde genişliği. İki bağımsız ölçüm çakıştı; ayrıca metrenin
> gerçekten **dik** tutulduğunu kanıtlıyor (yatık olsaydı toplam 78.5'i
> **aşardı**).

> 🔴 **PITCH İŞARETİ — bu form 2026-08-09'a kadar TERS yazıyordu.** Eski hâli
> *"aşağı bakıyorsa −"* idi. **Yanlış.** `hardware.yaml` `tf:` değerleri
> `hardware.launch.py` → `_static_tf` üzerinden doğrudan
> `tf2_ros static_transform_publisher --pitch`'e gidiyor, yani standart REP-103
> kuralı geçerli: +y (iskele) etrafında sağ-el dönüşü,
> `R_y(θ)·x̂ = (cosθ, 0, −sinθ)` → küçük pozitif θ'da z bileşeni **negatif**.
>
> | | |
> |---|---|
> | `pitch` **pozitif** | kamera **AŞAĞI** bakar |
> | `pitch` **negatif** | kamera **YUKARI** bakar |
>
> Eski notu okuyup dolduran kişi açıyı **ters** yazardı → kamera TF'te yanlış
> yöne yatar, duba mesafe ve kerteriz projeksiyonu bozulur, ve **hata sessizdir**
> (mesafeler tutarlı biçimde yanlış çıkar, hiçbir uyarı basılmaz). Ölçümü
> **fiziksel olarak** not edin ("ufka göre yukarı/aşağı, şu kadar derece"),
> işaret çevrimini tek yerde yapın.

---

## §4 — ESC (bkz. `ardurover_bench.parm.md` → ESC KALİBRASYONU)

Tekne **2 motorlu** (sol/sağ). Her iki ESC de aynı model olmalı.

| | Değer |
|---|---|
| Marka / model | **markasız jenerik "Bidirectional ESC 50A"** (motorobit.com, su altı motoru uyumlu) ✅ 2026-08-06. Etiket: `50A` · `BEC 2A 5V` · `LIPO 2S-4S`. **Üretici manuali YOK** — bkz. `ardurover_bench.parm.md` ESC adım 3 |
| **Tek yönlü mü, çift yönlü (reversible) mi?** | ☑ **ÇİFT YÖNLÜ** ✅ (2026-08-04) |
| Akım değeri | **50 A** ✅ |
| PWM aralığı (canlı FC, 2026-08-06) | min **1000** / sıfır itki **1487** / max **2000** µs — ✅ su testinden geçti |
| **FC tarafı simetri** (`SERVOn_MIN/MAX/TRIM` iki kanalda aynı mı?) | ☑ **EVET** ✅ (2026-08-06, canlı param dökümünden) |
| Kalibrasyon yapıldı mı (ikisi de, aynı prosedürle)? | ☑ **YAPILMAYACAK** — bilinçli karar ✅ 2026-08-06. Çift yönlü ESC'de klasik gaz kalibrasyonu nötrü kaydırabilir; nötrümüz zaten doğru ve simetrik. Gerekçe + kalan risk: `ardurover_bench.parm.md` ESC adım 3 |
| **Thruster tepe akımı (tek motor)** | `____` A ⏳ |
| **2 motor toplam tepe akım** | `____` A ⏳ 🔴 **KONTAKTÖR SINIRI 50 A** — GRDNER HEV50-A12NS temin edildi (2026-08-06). Toplam tepe bu değeri aşarsa kontaktör görev sırasında yanar. Suda tam gazda ölç |

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

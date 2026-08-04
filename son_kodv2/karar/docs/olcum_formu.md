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

**Eksen kuralı (ROS REP-103, değiştirilemez):**
- **+x = PRUVA** (ileri, teknenin burnu)
- **+y = İSKELE** (sol taraf)
- **+z = YUKARI**
- Açı birimi: **derece**, saat yönünün TERSİ (+) — yani +yaw = sola dönüş

### 🔴 Önerilen konum: **Pixhawk'ın bulunduğu nokta**

Sezgisel seçim "iki gövdenin tam ortası"dır ama **teknik olarak doğru seçim
Pixhawk'tır.** Gerekçe:

`/mavros/local_position/odom` — yani konum kaynağımız — ArduPilot EKF'inin
çıktısıdır ve ArduPilot varsayılan olarak **IMU'yu (yani FC kartını) araç
orijini** kabul eder (`INS_POS_*` / `GPS_POS_*` offset'leri 0 ise). Yani odom
zaten "Pixhawk'ın konumu"nu söylüyor.

`base_link`'i başka bir yere koyarsak: odom Pixhawk'ın yerini bildirir, engel
koordinatları ise seçtiğimiz noktaya göre ölçülür → **aradaki mesafe kadar
sistematik kayma** oluşur. Pixhawk pruvaya doğru 30 cm ileride ise her engel
30 cm yanlış yerde görünür. Kapı net açıklığı ~1.35 m, tekne 0.78 m → boşluk
zaten dar; 30 cm'lik sabit hata pahalıya patlar.

**Uygulama:** Pixhawk'ın gövdesinin merkezini güverteye çekülle indir, o
noktayı işaretle. `base_link` orası.

> Alternatif isterseniz: geometrik merkezi `base_link` yapıp ArduPilot
> `INS_POS_X/Y/Z` parametrelerine Pixhawk'ın o merkeze göre offset'ini girin.
> İki iş yerine tek iş olduğu için önerimiz yukarıdaki.

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
| Gövde boyu | 1.04 m | `____` m | Burun hattı / min_forward |
| Şamandıra/fender vb. ile toplam genişlik | — | `____` m | Çarpma payı için |

---

## §2 — 🔴 LiDAR (Livox Mid-360) — EN KRİTİK ÖLÇÜM

> **Neden kritik:** `perception_lidar_node` şu an **hiçbir TF dönüşümü
> uygulamıyor** — çıktının `frame_id`'sini `base_link` diye etiketliyor, o
> kadar. Oysa `z_min=0.1` filtresi "base_link'e göre su üstü kesim" diye
> tanımlı. LiDAR `base_link`'in ÜSTÜNDEYSE dubalar LiDAR çerçevesinde
> **negatif z**'de kalır ve `z_min=0.1` filtresi **hepsini eler** → LiDAR
> hiçbir engel görmez. `h` ölçülmeden bu doğrulanamaz.

| Ölçü | Değer | Nasıl ölçülür |
|---|---|---|
| `x` — `base_link`'ten ileri (+) / geri (−) | `____` m | Yatay mesafe, cetvel |
| `y` — iskeleye (+) / sancağa (−) | `____` m | Merkezdeyse 0 |
| **`z` — `base_link`'ten yukarı (h)** | `____` m | 🔴 F5.1'i çözen sayı |
| `yaw` — pruvaya göre dönüklük | `____` ° | 0 = konnektör kıça bakıyor varsayımı DEĞİL, ölçün |
| `pitch` / `roll` — eğik monte edildi mi? | `____` / `____` ° | Düz monte ise 0/0 |
| **Su hattından yükseklik** (yüklü tekne, sakin su) | `____` m | Menzil/kör nokta hesabı |
| Gövdenin LiDAR görüş alanına giren kısmı var mı? | ☐ var ☐ yok | Varsa `min_range` filtresi gerekir (F5.1 ile birlikte) |

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
| Marka / model | `________________` |
| **Tek yönlü mü, çift yönlü (reversible) mi?** | ☐ tek yönlü ☐ çift yönlü |
| PWM aralığı (üretici verisi) | min `____` / nötr `____` / max `____` µs |
| Kalibrasyon yapıldı mı (ikisi de, aynı prosedürle)? | ☐ evet, tarih: `______` |
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

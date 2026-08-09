# Sensör Konumları — `base_link`'e göre

**GİRDAP İDA · Takım 989124 · Alt Alan B (FC / navigasyon / aktüasyon)**
**Derleme tarihi:** 2026-08-09
**Ölçüm tarihi:** 2026-08-04 (tekne masada, düz zemin, su terazisiyle teyitli)
**Kaynak:** `docs/olcum_formu.md` §0–§3 · yürürlükteki değerler
`ros2_ws/src/girdap_decision/config/hardware.yaml` → `tf:` bloğu

> Bu dosya kaptanın talebi üzerine derlendi. **Yeni ölçüm içermez** — mevcut
> ölçümleri tek sayfada, hepsi aynı referansa (`base_link`) çevrilmiş halde
> sunar. Değer değiştirmek gerekirse **`hardware.yaml` tek doğruluk kaynağıdır**;
> burası ondan türer, tersi değil.

---

## 🔴 ÖNCE BUNU OKU — `base_link` Pixhawk'ta DEĞİL

`base_link`, 2026-08-04'te **Pixhawk'tan teknenin geometrik merkezine
taşındı.** Daha eski dokümanlarda (05.08 öncesi) yazan sensör konumları
**Pixhawk'a göreliydi** ve artık **yanlış referanstadır**.

| | Eski (≤2026-08-05) | **Yürürlükteki (≥2026-08-06)** |
|---|---|---|
| `base_link` nerede | Pixhawk 6C gövdesinin merkezi | **Teknenin geometrik merkezi** |
| Livox `x/y/z` | +0.07 / −0.1375 / +0.255 | **+0.015 / 0.0 / +0.41** |

Eski sayıları bir yere kopyalamışsanız **değiştirin.** Karışıklık olursa
ölçüt: `hardware.yaml`'da ne yazıyorsa o doğrudur.

### `base_link` tam olarak nerede

| Eksen | Tanım | Ölçü |
|---|---|---|
| `x` | Gövde **boy ortası** | Ön uçtan **51.5 cm** (tekne boyu **103 cm**) |
| `y` | Gövde **merkez hattı** | Livox tam bu hat üzerinde |
| `z` | Gövde **tabanı** | Tekne masadayken **masa yüzeyi** |

### Neden taşındı (gerekçe — kaptanın sorabileceği)

İlk seçim Pixhawk'tı, gerekçe *"ArduPilot IMU'yu araç orijini sayar"* idi.
Bu eksikti, iki sebeple:

1. **Mutlak konumu GPS çiviliyor.** `GPS1_POS_*` sıfırken EKF anten ölçümünü
   olduğu gibi kabul eder — yani raporlanan konum pratikte **GPS anteninin**
   konumudur, Pixhawk'ın değil. Bu teknede anten sancakta bir kol üzerinde,
   yani orijin daha da kayık.
2. **Ölçüm gösterdi ki Pixhawk merkez hattının 13.75 cm iskelesinde.**
   `base_link` orada kalsaydı, kapı ortasına `base_link` sürüldüğünde tekne
   gövdesi ortadan **13.75 cm sancağa kaymış** geçerdi:

```
kapı açıklığı (md 5.5.2.1, ~1.35 m varsayımı)        1.350 m
tekne genişliği                                      0.780 m
→ ideal yan pay (her iki yanda)                      0.285 m
base_link ofseti yüzünden  sancak payı  0.285−0.1375 = 0.148 m
                           iskele payı               = 0.423 m
```

Sancak emniyet payı **yarıya iniyordu.** Şartname kapı genişliğinin alana göre
değişeceğini söylüyor (md 5.5.2.1 / 5.5.3.1) → dar kapıda çarpma cezası
(`Ç1`/`Ç2`) riski.

**Seçilen çözüm:** orijini biz seçeriz (gövde merkezi), ArduPilot'a da
"GPS şurada, IMU şurada" deriz → EKF gövde merkezini raporlar, kayma kökünden
biter. (Alternatif "base_link Pixhawk'ta kalsın, kapı hedefine 13.75 cm yanal
düzeltme uygulansın" yolu **reddedildi** — kod karmaşası, tek parametreyle
çözülebilecek şeyi mantığa gömmek.)

---

## Eksen kuralı (ROS REP-103 — değiştirilemez)

| | Yön |
|---|---|
| **+x** | **PRUVA** — ileri, teknenin burnu (kamera yuvasının olduğu taraf) |
| **+y** | **İSKELE** — sol taraf |
| **+z** | **YUKARI** |
| **+yaw** | Saat yönünün **TERSİ** = sola dönüş |

---

## Sensör konumları — `base_link` → sensör (ROS / TF çerçevesi)

Öteleme **metre**, dönüklük **radyan**. Hepsi `base_link` orijininden ölçülür.

| Sensör | TF çerçevesi | `x` | `y` | `z` | `yaw` | `pitch` | `roll` | Durum |
|---|---|---|---|---|---|---|---|---|
| **Livox Mid-360** (LiDAR) | `livox_frame` | **+0.015** | **0.000** | **+0.410** | ⏳ `____` | 0.0 | 0.0 | Öteleme ✅ ölçüldü · yaw ⏳ |
| **Pixhawk 6C** (IMU) | `imu_link` | **−0.055** | **+0.1375** | **+0.155** | 0.0 | 0.0 | 0.0 | ✅ ölçüldü |
| **GPS anteni** (Holybro H-RTK F9P) | — (TF yayınlanmıyor) | **−0.035** | **+0.160** | **+0.365** | — | — | — | ✅ ölçüldü |
| **OAK-D Lite** (kamera) | `oak_frame` | ⏳ `____` | ⏳ `____` | ⏳ `____` | ⏳ `____` | ⏳ `____` | 0.0 | 🔴 **HENÜZ TAKILMADI** |

### Aynı ölçümler fiziksel dille (işaret hatası yapmamak için)

| Sensör | ileri/geri | sağ/sol | yükseklik (gövde tabanından) |
|---|---|---|---|
| Livox Mid-360 | 1.5 cm **pruvada** | merkez hattında (0) | **41.0 cm** |
| Pixhawk 6C | 5.5 cm **kıçta** | 13.75 cm **iskelede** | **15.5 cm** |
| GPS anteni | 3.5 cm **kıçta** | 16 cm **iskelede** | **36.5 cm** |

> GPS ile Pixhawk **aynı tarafta** (ikisi de iskele) — bu tesadüf değil, aynı
> montaj bölgesinde.

### Livox `z` = 41.0 cm nasıl bulundu

Doğrudan masadan ölçüldü (masa yüzeyi = gövde tabanı = `base_link` z=0).
Ara adımlar, tutarlılık kontrolü için:

```
LiDAR masadan                            41.0 cm
kapak ağzı masadan                       24.5 cm
kapak ağzı tekne tabanından              15.0 cm
Pixhawk tekne tabanından (3D platform)    6.0 cm
→ Pixhawk kapak ağzından  15.0 − 6.0   =  9.0 cm aşağıda
→ Pixhawk masadan         24.5 − 9.0   = 15.5 cm   ← imu_link.z ile tutarlı ✅
→ LiDAR ile Pixhawk arası 41.0 − 15.5  = 25.5 cm
```

Bu `z` **kritik**: LiDAR su üstü kesim filtresini (`z_min`) o belirliyor.
Yanlışsa `obstacle_map` sessizce **boş** kalır (2026-08-06'da tam bu yüzden
boştu, bkz. `olcum_formu.md` §2 / F5.1).

---

## Gövde ölçüleri (referans için)

| Ölçü | Değer | Durum |
|---|---|---|
| Gövde **boyu** | **1.03 m** | ✅ ölçüldü 2026-08-04 (kodda 1.04 yazıyor, 1 cm fark tolerans içinde) |
| Gövde **genişliği** (en geniş yer) | 0.78 m | 🟡 **KAYNAKLAR ÇELİŞİYOR** — aşağıya bak |
| Fender/şamandıra dahil toplam genişlik | `____` m | ⏳ ölçülmedi |

> 🟡 **Genişlik çelişkisi (çözülmeli):** `hardware.yaml` satırı
> `hull_width_m: 0.78 # ÖLÇÜLMÜŞ (GIRDAP_DURUM §1)` diyor; ama
> `olcum_formu.md` §1 aynı sayıyı **"teyit edin"** diye işaretleyip alanı boş
> bırakmış (`____`). Yani 0.78 eski bir dokümandan geliyor, 04.08 ölçüm
> turunda **yeniden ölçülmedi** — boy ölçüldü (1.04 → gerçek **1.03**), en
> ölçülmedi. Boyda 1 cm sapma çıktığına göre ende de çıkabilir.
>
> Genişlik `gate_follower`'ın kapı geçilebilirlik testinde kullanılıyor —
> eşik ayarı değil, **fizik**: bundan dar açıklık "kapı" sayılmıyor. Gerçek en
> 0.78'den **büyükse** geçemeyeceği bir kapıyı geçilebilir sayar. **Şeritle
> 2 dakikada ölçülür, ölçün.**

---

## ⚠️ EKSEN TUZAĞI — ArduPilot ile ROS'un işaretleri TERS

Aynı fiziksel konum iki dosyada **farklı işaretle** yazılır:

| | ROS / TF (`hardware.yaml`) | ArduPilot `INS_POS*` / `GPS1_POS*` |
|---|---|---|
| X | ileri **+** | ileri **+** (aynı) |
| Y | **iskele (sol) +** | **sancak (sağ) +** ← **TERS** |
| Z | **yukarı +** | **aşağı +** ← **TERS** |

**Kural:** ölçümü daima fiziksel olarak not et ("şu kadar sağda", "şu kadar
yukarıda"), çevirmeyi **tek yerde** yap. Yukarıdaki "fiziksel dille" tablosu
bu yüzden var.

### FC'ye girilecek karşılıklar (Y ve Z işareti çevrilmiş)

```
INS_POS1_X   -0.055        GPS1_POS_X   -0.035
INS_POS1_Y   -0.1375       GPS1_POS_Y   -0.16
INS_POS1_Z   -0.155        GPS1_POS_Z   -0.365
```

> 🔴 **BU ALTI PARAMETRE FC'DE HÂLÂ 0** (2026-08-07 canlı dökümüyle teyitli:
> `docs/fc_mevcut_parametreler_2026-08-07.param`). Girilene kadar `base_link`
> tanımı ile odom'un raporladığı nokta **ÇAKIŞMIYOR** — sistemde bilinen sabit
> bir ofset var. Girilmesi A-3'te sıraya alındı.

> 🔴 **PARAMETRE ADI:** GPS ofsetleri bu firmware'de **`GPS1_POS_X/Y/Z`**.
> Bazı eski notlarda `GPS_POS1_*` yazıyor — **öyle bir parametre YOK.** Yanlış
> adı Mission Planner'da arayan kişi bulamaz, ofset **hiç uygulanmaz** ve
> herkes yapıldı sanır. `INS_POS1_*` doğru yazımdır.

> ⚠️ **Doğrulanacak varsayım:** ArduPilot dokümanı bu ofsetleri aracın
> **ağırlık merkezine** göre tanımlar; biz **geometrik merkezi** kullandık.
> Küçük teknede ikisi yakındır ve düzeltmeyi asıl belirleyen GPS↔IMU arasındaki
> **göreli** geometridir (o doğru). Ağırlık merkezi belirgin şekilde başka
> yerdeyse tüm değerler ötelenir.

---

## Kodda nerede yaşıyor

| Değer | Dosya | Not |
|---|---|---|
| Tüm TF ötelemeleri/dönüklükleri | `ros2_ws/src/girdap_decision/config/hardware.yaml` → `tf:` | **tek doğruluk kaynağı** |
| Static TF yayıncıları | `launch/hardware.launch.py` → `_static_tf("base_link", <child>)` | `tf:` bloğundan okur, elle argüman yazılmaz |
| LiDAR mount ofseti (nokta bulutu → base_link) | `perception_lidar_node` ← `tf.livox_frame` | `_mount_params("livox_frame", hw["tf"])` ile geçirilir |

> `hardware.yaml`'da `perception.lidar` altına **`mount_*` yazma** — launch
> onu `tf:`ten geçiriyor ve **EZER.**

---

## ⏳ AÇIK — ölçülmeyi bekleyen

| # | Ne | Neden önemli | Kim |
|---|---|---|---|
| 1 | **Livox `yaw`** (ampirik) | 10 m'de 5° hata → **0.87 m yanal kayma**; kapı ortası hesabını doğrudan bozar | ölçüm: mekanik + karar |
| 2 | **Kamera (OAK-D) x/y/z/yaw/pitch** | Kamera takılmadı. `oak_frame` şu an hiçbir node tarafından tüketilmiyor (füzyon kalibrasyonsuz) → **önceliği düşük** | mekanik, takıldıktan sonra |
| 3 | **Livox'un su hattından yüksekliği** (yüklü tekne, sakin su) | `z_min`'in doğru değerini bu belirler | ilk suya inişte |
| 4 | **Gövde genişliği** teyidi | `gate_follower` geçilebilirlik testi | mekanik |
| 5 | **6 FC parametresinin girilmesi + masa testinde doğrulanması** | Girilmeden odom ≠ base_link. ArduPilot'un ofseti **fiilen uyguladığı** gözle görülmeden "tamam" denmeyecek | Alt Alan B (A-3) |
| 6 | Gövdenin LiDAR görüş alanına giren kısmı var mı | Varsa `min_range` filtresi gerekir | ilk suya inişte |

### `yaw` nasıl ampirik ölçülür (iletkiyle uğraşma)

1. Mekanik olarak hizalı monte edildiyse `yaw = 0` yaz (başlangıç).
2. Merkez hattının **tam üzerine**, ~10 m ileriye bir duba/kova/direk koy.
   Yığın açıkken:
   ```bash
   ros2 topic echo /perception/obstacle_map --once
   ```
   Tek engelin `position.y`'si **≈ 0** olmalı. Değilse:
   ```
   yaw_hata(derece) = atan2(y, x) × 180/π
   ```
   Bulduğun açıyı `tf:` bloğuna **radyan olarak** gir ve tekrarla.
3. Aynı testi kamera için: duba görüntünün **tam ortasında** olmalı.

Bu yöntem mekanik ölçümden daha doğru, ayrıca montaj/optik eksen farklarını da
yakalar.

**Hassasiyet:** ±2 cm yeter (LiDAR cluster toleransı 0.5 m, duba yarıçapı
0.15 m — santimin altını kovalamayın). **İstisna: `z`** — su üstü kesim
filtresini o belirliyor, dikkatli ölçün.

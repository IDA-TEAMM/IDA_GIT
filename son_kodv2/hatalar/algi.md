# Algı hataları

Kamera (OAK-D), LiDAR (Livox Mid-360), sınıflandırma ve engel haritası katmanındaki
bulgular. Kaynak: 14 rosbag oturumu, bkz. [README](README.md).

**Sınıf sözleşmesi** (`duba_gecis_navigator.py:141-142, 219-223`):

```python
KENAR_CLASS = 0        # turuncu duba (RAL 2003) - parkur kenarı
ENGEL_CLASS = 1        # sarı duba   (RAL 1026) - engel
SINIF_ESLEME = {KENAR_CLASS: "0", ENGEL_CLASS: "1"}
```

`duba_gecis_navigator.py:918` → `hyp.hypothesis.class_id = self.sinif_esleme.get(d.cls, str(d.cls))`
Yani **`SINIF_ESLEME`'de olmayan her sınıf ham sayısıyla geçer**; bag'lerde görülen
`"99"` bu düşüş yolundan gelir — kamerayla eşleşmemiş ham LiDAR kümesi.

| # | Bulgu | Şiddet |
|---|---|---|
| [ALG-01](#alg-01) | Engellerin %99,96'sı sınıf `99` — sınıflandırma pratikte çalışmıyor | 🔴 |
| [ALG-02](#alg-02) | Engel bulutunun %27'si aracın arkasında, en yakını 1,3 mm | 🔴 |
| [ALG-03](#alg-03) | `/perception/buoys` oturumun %95,6'sında boş | 🔴 |
| [ALG-04](#alg-04) | Turuncu duba (sınıf 0) pratikte hiç tespit edilmiyor — 140:1 dengesizlik | 🟠 |
| [ALG-05](#alg-05) | LiDAR tamamen çöktü: 5 saatte 39 mesaj, 45 nokta/tarama | 🟠 |
| [ALG-06](#alg-06) | Algı `header.stamp`'leri 56 yıl bayat | 🟠 |
| [ALG-07](#alg-07) | Algı hattı füzyonla eşzamanlı ~1,1 s donuyor | 🟡 |

---

## ALG-01

### Engellerin %99,96'sı sınıf `99` — sınıflandırma pratikte çalışmıyor

**Şiddet:** 🔴 Kritik

**Kanıt** — `/perception/classified_obstacles` sınıf dağılımı:

| Oturum | sınıf `99` | sınıf `1` (sarı) | sınıf `0` (turuncu) | `99` oranı |
|---|---:|---:|---:|---:|
| `session_20260810_213017` (10 Ağu 21:30) | **1** | 148 | 146 | **%0,34** |
| `session_19700101_020119` | 101.431 | 189 | 288 | %99,53 |
| `session_20260811_143741` | 870.323 | 6.036 | 1.434 | %99,15 |
| `session_20260811_145923` | 476.210 | 4.084 | 21 | %99,15 |
| `session_20260811_163939` | **1.775.300** | 742 | 6 | **%99,96** |

**Bulgu:** 10 Ağustos 21:30 oturumunda sınıflandırma **çalışıyordu** (sınıf 99 = 1 adet).
11 Ağustos'un tüm oturumlarında sınıf 99 baskın hale geldi. Bu bir **regresyon**:
kırılma 2026-08-10 21:31 ile 2026-08-11 02:01 arasında.

Ham örnek (`session_20260811_145923`, `t=1786449771.526`):

```
/perception/classified_obstacles n=119 frame=base_link
  (-2.02, -1.94, 0.0, '99', 0.0)
  (-2.00, -0.04, 0.0, '99', 0.0)
  (-2.19,  0.48, 0.0, '99', 0.0)
  (-2.32,  1.84, 0.0, '99', 0.0)
  (-2.34,  2.33, 0.0, '99', 0.0)
```

Dikkat: **skor alanı `0.0`** ve **z = `0.0`** — yani bunlar kamera hipotezi almamış,
düzleme indirgenmiş ham LiDAR kümeleri.

**Kök neden:** LiDAR kümesi kamera tespitiyle eşleşemediğinde `d.cls` `SINIF_ESLEME`
dışında kalıyor ve `duba_gecis_navigator.py:918`'deki `.get(..., str(d.cls))`
varsayılanı ham sayıyı geçiriyor. Eşleşmenin neden koptuğu iki alt nedene bağlı:

1. Kamera zaten çok az tespit üretiyor (bkz. [ALG-03](#alg-03), [ALG-04](#alg-04)) —
   eşleşecek hipotez yok.
2. Kamera ve LiDAR zaman damgaları uyumsuz (bkz. [ALG-06](#alg-06)) — eşleştirme
   penceresi tutmuyor.

**Etki:** Duba renk mantığı (kenar/engel ayrımı) beslenmiyor. Karar katmanı
"engel var" bilgisini alıyor ama **hangi renk olduğunu bilmiyor** → parkur kenarı ile
engel ayırt edilemez, geçiş planlaması yapılamaz.

**Öneri:**
1. Sınıf `99`'u `/perception/classified_obstacles`'a **hiç yayma** ya da ayrı bir
   topic'e (`/perception/unclassified_clusters`) ayır — karar katmanı sınıfsız engeli
   sınıflıymış gibi işlemesin.
2. Eşleştirme penceresini ve başarısızlık sayacını `/diagnostics`'e bas: kaç LiDAR
   kümesi kamerayla eşleşti / eşleşemedi. Şu an bu sessizce kayboluyor.
3. 10 Ağustos 21:30 ile 11 Ağustos 02:01 arasındaki commit'leri tara — regresyon
   orada.

---

## ALG-02

### Engel bulutunun %27'si aracın arkasında, en yakını 1,3 mm

**Şiddet:** 🔴 Kritik

**Kanıt** — `/perception/classified_obstacles`, `frame=base_link`, `x < 0` (arka yarı düzlem):

| Oturum | toplam tespit | `x<0` | oran | en yakın engel |
|---|---:|---:|---:|---:|
| `session_20260810_213017` | 294 | **0** | %0 | 1,03 m |
| `session_19700101_020119` | 101.908 | 26.994 | %26,5 | 0,351 m |
| `session_20260811_143741` | 877.793 | 140.975 | %16,1 | **0,00134 m** |
| `session_20260811_145923` | 480.315 | 84.442 | %17,6 | 0,152 m |
| `session_20260811_163939` | 1.776.052 | **489.057** | **%27,5** | 0,152 m |

**Bulgu:** Mesaj başına ortalama **110-120 engel** üretiliyor (10 Ağustos'ta 2,06 idi).
Bunların dörtte biri aracın arkasında ve en yakını **1,3 milimetre** mesafede.
`base_link` orijininde duran bir engel fiziksel olarak imkânsızdır — bu, aracın kendi
gövdesi, güverte donanımı ve/veya sıfır-yarıçap gürültüsüdür.

`/perception/obstacle_map` aynı içeriği taşıyor (`session_20260811_143741`: 14.006
mesaj × 109,4 engel, 270.938'i arkada).

**Kök neden:** LiDAR nokta bulutunda **kendi gövdesini eleyen yarıçap/kutu filtresi
yok** (ya da devre dışı). Livox Mid-360 360° tarar; araca monte edildiğinde gövde,
direk ve kablolar sürekli dönüş üretir. 10 Ağustos oturumunda bu filtre etkiliydi
(arka nokta = 0, engel/mesaj = 2,06), sonra kayboldu — [ALG-01](#alg-01) ile **aynı
regresyon penceresinde**.

**Etki:**
- Engel kaçınma sürekli "0,15 m'de engel var" görüyor → kaçınma mantığı kilitlenir
  veya sürekli manevra üretir.
- Arkadaki dönüşler ileri planlamayı kirletir.
- Mesaj başına 120 engel, planlayıcıya gereksiz yük bindirir.
- [KAR-04](karar.md#kar-04)'ün (itki hep sıfır) olası tetikleyicilerinden biri:
  sürekli "çok yakın engel" durumu güvenlik kilidini açık tutuyor olabilir.

**Öneri:**
1. LiDAR sürücüsünden hemen sonra **öz-gövde filtresi** uygula: `r < r_min` (öneri:
   1,0-1,5 m, araç boyutuna göre) ve gerekiyorsa arka sektör maskesi.
2. `z` ekseninde kırpma ekle — bag'lerde tüm engellerin `z = 0.0` olması, z bilgisinin
   zaten düşürüldüğünü gösteriyor; kırpma bundan **önce** yapılmalı.
3. `/diagnostics`'e "filtre öncesi/sonrası nokta sayısı" ekle.

---

## ALG-03

### `/perception/buoys` oturumun %95,6'sında boş

**Şiddet:** 🔴 Kritik

**Kanıt** — `/perception/buoys` (Detection2DArray, `frame=oak_rgb`):

| Oturum | mesaj | boş mesaj | boş oranı | tespit/mesaj |
|---|---:|---:|---:|---:|
| `session_20260810_213017` | 143 | 44 | %30,8 | 2,056 |
| `session_19700101_020119` | 1.009 | 488 | %48,4 | 0,621 |
| `session_20260811_143741` | 9.760 | 2.904 | %29,8 | 0,902 |
| `session_20260811_145923` | 4.471 | 55 | %1,2 | 1,025 |
| `session_20260811_163939` | 18.381 | **17.563** | **%95,6** | **0,046** |

**Bulgu:** `session_20260811_163939` (16:39-17:07, 28 dakika) boyunca kamera
**neredeyse hiçbir şey görmedi** — 18.381 karede toplam 850 tespit. Kamera akışının
kendisi sağlıklıydı (11 Hz, kesintisiz), yani sorun görüntü değil **tespit**.

Aynı oturumda LiDAR normal çalışıyordu (16.704 tarama, ~20.000 nokta/tarama) — yani
sahada nesne vardı, kamera onları sınıflandıramadı.

**Kök neden:** İki aday, ikisi de aynı yöne işaret ediyor:
- HSV doygunluk/parlaklık eşiği çok yüksek — bkz. [PAR-06](parametre.md#par-06)
  (`hsv_*_lo = (H, 120, 120)`, yani `S ≥ 120` **ve** `V ≥ 120`). Akşam saatlerinde
  (16:39-17:07) ışık düştükçe `V` bu eşiğin altına iner.
- Güven eşiği: bag'de gözlenen en düşük skor **0,50** (tüm oturumlarda) → `conf=0.5`
  sabit. Düşük ışıkta YOLO skorları bu eşiğin altında kalır.

Saat korelasyonu bu okumayı destekliyor: 14:59 oturumunda boş oran %1,2 iken
16:39 oturumunda %95,6'ya çıkıyor — **ışık azaldıkça tespit çöküyor**.

**Etki:** Kamera-LiDAR eşleşmesi kalmıyor → [ALG-01](#alg-01) (sınıf 99 seli) doğrudan
bunun sonucu. Renk bilgisi olmadan parkur/geçiş mantığı çalışamaz.

**Öneri:**
1. HSV alt sınırlarını ölçüme dayalı düşür (bkz. [PAR-06](parametre.md#par-06)) ve
   `V` alt sınırını ışığa göre uyarlanabilir yap.
2. `/diagnostics`'e kare başına "ham tespit / eşik sonrası tespit" sayacı ekle —
   şu an eşiğin mi yoksa modelin mi elediği bag'den ayırt edilemiyor.
3. Akşam/alacakaranlık koşulunda ayrı bir eşik profili tut.

---

## ALG-04

### Turuncu duba (sınıf 0) pratikte hiç tespit edilmiyor — 140:1 dengesizlik

**Şiddet:** 🟠 Yüksek

**Kanıt** — `/perception/buoys` sınıf dağılımı:

| Oturum | sınıf `0` (turuncu/kenar) | sınıf `1` (sarı/engel) | oran |
|---|---:|---:|---|
| `session_20260810_213017` | 146 | 148 | 1,0 : 1 ✅ |
| `session_19700101_020215` | 126 | 89 | 1,4 : 1 ✅ |
| `session_19700101_020119` | 384 | 243 | 1,6 : 1 ✅ |
| `session_20260811_143741` | 1.762 | 7.046 | 1 : 4,0 |
| `session_20260811_145923` | **22** | 4.559 | **1 : 207** |
| `session_20260811_163939` | **6** | 844 | **1 : 140** |

**Bulgu:** Erken oturumlarda iki sınıf dengeliyken, 11 Ağustos öğleden sonraki
oturumlarda turuncu duba tespiti **pratikte sıfıra** indi. Parkur kenarı turuncu
dubalarla işaretlendiği için bu, aracın **parkur sınırlarını göremediği** anlamına gelir.

**Kök neden:** `camera_buoys.py:67-70`'teki HSV aralıkları:

```python
hsv_orange_lo = (5, 120, 120)     # RAL 2003 yakını
hsv_orange_hi = (20, 255, 255)
hsv_yellow_lo = (21, 120, 120)    # RAL 1026 yakını
hsv_yellow_hi = (35, 255, 255)
```

Turuncu ve sarı **H ekseninde bitişik** (20 | 21). Su yüzeyi yansıması ve düşük ışıkta
turuncu dubanın H değeri 21'in üstüne kayarsa **sarı olarak sınıflanır**. Sınıf 1'in
şişmesi ve sınıf 0'ın çökmesi tam olarak bu kaymanın imzasıdır.

Ayrıca `S ≥ 120` eşiği ([PAR-06](parametre.md#par-06)) her iki rengi de kırpıyor;
turuncu, sarıya göre daha düşük doygunlukta göründüğü için önce o eleniyor.

**Etki:** `/girdap/planning/edge_buoys` besleniyor ama içeriği güvenilmez —
`session_20260811_143741`'de mesaj başına **60,6** "kenar dubası" var ve 74.631'i
aracın arkasında ([ALG-02](#alg-02) kirliliği buraya taşınmış).

**Öneri:**
1. H sınırını turuncu/sarı arasında **boşluklu** yap (örn. turuncu 5-18, sarı 24-35)
   ve arada kalan tespitleri "kararsız" olarak işaretle.
2. Sadece H'ye değil, `a*`/`b*` (Lab) kanallarına da bak — su yansımasına H'den daha
   dayanıklıdır.
3. Sınıf dağılımını `/diagnostics`'e periyodik bas; 100:1'e giden dengesizlik alarm
   üretsin.

---

## ALG-05

### LiDAR tamamen çöktü: 5 saatte 39 mesaj, 45 nokta/tarama

**Şiddet:** 🟠 Yüksek

**Kanıt** — `session_19700101_020215`, `/livox/lidar`:

```
mesaj sayısı  : 39            (sağlıklı oturumlarda 16.000-34.000)
nokta/tarama  : min 2, maks 120, ortalama 45,7   (normal: ~20.000)
en büyük kesinti: 1.896,9 s  (31,6 dakika)
diğer kesintiler: 1.206,3 s · 673,2 s · 258,3 s
toplam kesinti : 4.035,6 s   (1 saat 7 dakika)
```

Karşılaştırma (sağlıklı): `session_20260811_163939` → 16.704 mesaj, ortalama
19.992 nokta, kesinti yok.

**Bulgu:** LiDAR bu oturumda fiilen ölüydü. Gelen az sayıdaki tarama da neredeyse boştu
(2-120 nokta). Aynı oturumda `/perception/classified_obstacles` yalnızca 50 mesaj
üretebildi — algı zinciri LiDAR'a bağımlı olduğu için tümüyle sustu.

**Kök neden:** Bag verisi tek başına ayırt etmiyor; iki aday:
- Ağ/arayüz: `livox_driver_node.py` sabitleri (`192.168.117.100`, port `56301`) ve
  host statik IP'si (`192.168.117.50/24`) — arayüz düşerse UDP akışı kesilir.
- Cihaz beslemesi/aşırı ısınma.

Ayırt edici kanıt: nokta sayısının **sıfır değil 2-120** olması, soketin açık kaldığını
ama akışın parçalandığını gösterir → ağ tarafı daha olası.

**Etki:** Engel algılama yok. Aynı oturumda `/mavros/setpoint_velocity/cmd_vel_unstamped`
5 saatte yalnızca 110 mesaj ([KAR-10](karar.md#kar-10)) — araç fiilen hareket etmedi.

**Öneri:**
1. `livox_driver_node`'a **tarama başına nokta sayısı** eşiği koy ve
   `/diagnostics`'e `ERROR` bas (şu an sessizce boş bulut yayıyor).
2. `sensor_node` timeout izlemesine "nokta sayısı çok düşük" koşulunu ekle — mesaj
   geliyor ama içi boş olduğunda mevcut timeout tetiklenmiyor.
3. Ethernet arayüzünü `nmcli`'de `autoconnect yes` + link izleme ile kalıcılaştır.

---

## ALG-06

### Algı `header.stamp`'leri 56 yıl bayat

**Şiddet:** 🟠 Yüksek

**Kanıt** — `session_19700101_020215`, `header.stamp` ile bag alım zamanı farkı:

| Topic | ölçülen mesaj | ortalama gecikme | `stamp = 0` |
|---|---:|---:|---:|
| `/livox/lidar` | 17 | **1.786.422.858 s** (≈56,6 yıl) | 22 |
| `/perception/obstacle_map` | 106 | 1.786.422.516 s | 0 |
| `/perception/classified_obstacles` | 50 | 1.786.422.847 s | 0 |
| `/perception/buoys` | 89 | 1.786.422.480 s | 37 |

Karşılaştırma — aynı oturumda sağlıklı olanlar:
`/mavros/local_position/pose` ortalama **0,0006 s**, `/girdap/fusion/pose` **0,0008 s**.

**Bulgu:** LiDAR ve ondan türeyen tüm algı mesajları, 1970 tabanlı bir saatten
damgalanmış; bag ise 2026 sistem saatiyle kaydediyor. Fark tam olarak
[PAR-02](parametre.md#par-02)'deki saat sıçraması kadar.

Ayrıca `/livox/lidar`'ın 22 mesajı ve `/perception/buoys`'un 37 mesajı **`stamp = 0`**
ile yayınlanmış — hiç damgalanmamış.

**Kök neden:** Livox sürücüsü noktaları **cihazın kendi iç saatiyle** damgalıyor
(açılıştan itibaren sayan, 1970 tabanlı); ROS saatine dönüştürülmüyor. Algı düğümleri
de gelen damgayı olduğu gibi taşıyor.

**Etki:**
- `tf2` dönüşümleri bu damgalarla **çalışmaz** — `lookupTransform` "extrapolation into
  the past" ile başarısız olur. Bag'lerde karar yığınının TF yayınlamaması bunu maskeliyor.
- Kamera (`oak_rgb`, doğru ROS damgası) ile LiDAR (1970 damgası) arasındaki
  **zaman eşleştirmesi imkânsız** → [ALG-01](#alg-01)'in ikinci kök nedeni.
- `message_filters.ApproximateTimeSynchronizer` kullanılıyorsa hiçbir çift eşleşmez.

**Öneri:**
1. `livox_driver_node`'da her buluta `self.get_clock().now().to_msg()` bas (cihaz
   damgasını ayrı bir alanda sakla, gerekiyorsa).
2. Sıfır damgalı mesajı **yayınlamayı reddet**, `/diagnostics`'e `WARN` bas.
3. Bir kere doğrula: `ros2 topic echo /livox/lidar --field header.stamp` ile
   `date +%s` yan yana tutmalı.

---

## ALG-07

### Algı hattı füzyonla eşzamanlı ~1,1 s donuyor

**Şiddet:** 🟡 Orta

**Kanıt** — `session_20260811_163939`, aynı zaman aralığında:

| Topic | nominal | kesinti sayısı | en büyük |
|---|---|---:|---:|
| `/girdap/fusion/odom` | 10 Hz | 6 | 1,537 s @ `t=1786456972,77` |
| `/girdap/fusion/pose` | 10 Hz | 6 | 1,530 s @ `t=1786456972,77` |
| `/perception/classified_obstacles` | 10 Hz | 15 | 1,136 s @ `t=1786456975,62` |
| `/perception/obstacle_map` | 10 Hz | 12 | 1,119 s @ `t=1786456976,16` |

Dört topic de `t≈1786456972-976` penceresinde (4 saniyelik bir aralık) birlikte donuyor.
`/livox/lidar` **aynı pencerede kesintisiz** (10 Hz, boşluk yok) — yani sensör akıyor,
işleyen düğümler duruyor.

**Bulgu:** Donma sensörde değil, işleme katmanında. LiDAR verisi gelmeye devam ederken
hem füzyon hem algı ~1,1-1,5 s duruyor.

**Kök neden:** Bag'den kesin ayırt edilemiyor; en olası aday **CPU/GC baskısı**:
mesaj başına 116 engel ([ALG-02](#alg-02)) × 10 Hz = saniyede ~1.160 `Detection3D`
nesnesi serileştiriliyor. Bu yük [ALG-02](#alg-02) düzeltilirse ~20 kat azalır.

**Etki:** 1,5 saniyelik kör nokta. Karar katmanının `odom_timeout_s = 1.0 s` eşiği bu
süreyi aşıyor → itki sıfırlanır, sonra geri döner (dur-kalk davranışı).

**Öneri:**
1. Önce [ALG-02](#alg-02)'yi düzelt; yükün büyük kısmı oradan geliyor.
2. `top -H` / `ros2 run` CPU profili ile donma anını yakala.
3. Donma kalıcı olursa algı ve füzyonu ayrı executor/process'e al.

---

## Not: arıza olmayan gözlemler

Bag'lerde dikkat çeken ama **arıza olmayan** üç gözlem — yanlış teşhis edilmesinler:

- **`/perception/gate_passed` hiç yayınlanmadı** (14 oturumun tamamında 0 mesaj).
  Bu kodda **bilerek** kapatılmış: `duba_gecis_navigator.py` → `GATE_PASSED_YAYINLA = False`.
  Gerekçe kaynakta yazılı: `fsm_node._on_gate_passed` gelen herhangi bir `True`'yu
  PARKUR2→PARKUR3 geçişine çeviriyor; algı hangi kapının sonuncusu olduğunu bilemediği
  için ilk kapıda parkur yarıda kesilirdi.

- **`/perception/gate_target` ve `/perception/gate_count` neredeyse hiç yayınlanmadı.**
  Beklenen davranış: `gate_target` yalnız `GECIT_MAX_MESAFE = 8.0 m`
  (`duba_gecis_navigator.py:287`) içindeki bir duba çifti bulunduğunda çıkar; `gate_count`
  ise geçişin **odometriyle doğrulanması** hâlinde artar. Kapı 8 m'den uzaksa sessizce
  hiçbir şey yayınlanmaz — bu tasarım gereği, ancak eşiğin sahaya uygunluğu
  [PAR-07](parametre.md#par-07)'de ayrıca ele alındı.

- **`/perception/buoys` `frame_id = oak_rgb`, `/perception/classified_obstacles`
  `frame_id = base_link`.** Farklı olmaları doğrudur (biri görüntü, diğeri araç çerçevesi).

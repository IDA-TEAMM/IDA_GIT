# Parametre hataları

Konfigürasyon, eşik, ortam ve donanım köprüsü katmanındaki bulgular. Kaynak:
14 rosbag oturumu, bkz. [README](README.md).

Bu kümedeki bulguların çoğu **kod hatası değil, ayar/ortam hatasıdır** — yani
yazılım tasarlandığı gibi çalışıyor ama yanlış sayıyla ya da yanlış ortamda
çalışıyor. Bu yüzden [algi.md](algi.md) ve [karar.md](karar.md)'deki birçok
semptomun ortak kök nedeni buradadır.

| # | Bulgu | Şiddet |
|---|---|---|
| [PAR-01](#par-01) | Test düğümleri canlı ROS domain'ine sızıyor — 24.430 sahte GPS mesajı | 🔴 |
| [PAR-02](#par-02) | Sistem saati senkronsuz başlıyor; oturum ortasında 20.676 günlük sıçrama | 🔴 |
| [PAR-03](#par-03) | Araç **hiçbir oturumda ARM edilmedi** — 41.524 mesajın tamamında `armed=false` | 🔴 |
| [PAR-04](#par-04) | MAVLink akış hızı heartbeat bütçesinin altında — `/mavros/state` 0,17 Hz → KILL | 🔴 |
| [PAR-05](#par-05) | MPPI `K`/`T` ayarları Jetson'ın 10 Hz bütçesini aşıyor | 🟠 |
| [PAR-06](#par-06) | HSV doygunluk eşiği (`S ≥ 120`) sahada ölçülen değerin çok üstünde | 🟠 |
| [PAR-07](#par-07) | Algı menzil sabitleri sahada hiç doğrulanmadı | 🟠 |
| [PAR-08](#par-08) | PID kazançları itki ölçeğiyle 10× tutarsız — saha yedeği çalışmaz | 🟠 |
| [PAR-09](#par-09) | `mission_source: fc` ile çoklu parkur görev dosyası çelişiyor | 🟠 |
| [PAR-10](#par-10) | Kayıt servisi düzgün kapatılmıyor — 13/14 bag sonlandırılmamış | 🟡 |

---

## PAR-01

### Test düğümleri canlı ROS domain'ine sızıyor

**Şiddet:** 🔴 Kritik

**Kanıt** — `session_19700101_020215`, `/mavros/global_position/global` üzerinde
164.251 mesajın **24.430'u sahte**:

| lat / lon | adet | kaynak |
|---|---:|---|
| 41,0 / 29,0 | 12.299 | `prototype/tests/test_mission_manager.py:251,253,282` |
| 40,8002 / 29,3 | 7.335 | `prototype/tests/test_mission_manager.py:199` ("gerçekçi göl koordinatı") |
| 0,0 / 0,0 | 4.796 | test kurulum varsayılanı |

Sahte mesajların imzası gerçek MAVROS GPS'inden **kesin olarak ayrılıyor**:

| Alan | Sahte | Gerçek MAVROS |
|---|---|---|
| `header.frame_id` | boş | `base_link` |
| `status.service` | 0 | 1 |
| `altitude` | 0 | ≈880 m |
| `position_covariance` | tamamen sıfır | ≈3,8 |
| `header.stamp` | 0 | geçerli |

**Kök neden:** `prototype/tests/` altındaki 16 test dosyası `rclpy.init()` çağırıp
gerçek düğüm/publisher oluşturuyor ve **hiçbirinde `ROS_DOMAIN_ID` izolasyonu yok.**
Araç servisleri ise `ROS_DOMAIN_ID=42` ile koşuyor (`scripts/girdap-karar.service:58`,
`scripts/girdap-rosbag.service:57`). Geliştiricinin kabuğunda da `~/.bashrc` aynı
domain'i export ediyor. Sonuç: araç sahada koşarken aynı makinede `pytest`
çalıştırmak, test publisher'larını **canlı sistemin içine** sokuyor.

Aynı testler `/girdap/mission/state`'e de yayın yapıyor — bkz. [KAR-01](karar.md#kar-01).

**Etki:** Füzyon zehirleniyor: iSAM2'ye gerçek RTK ölçümüyle 41,0/29,0 gibi yüzlerce
km uzaktaki sahte bir ölçüm aynı ağırlıkla giriyor → [KAR-06](karar.md#kar-06)
(odometri ışınlanması) ve [KAR-07](karar.md#kar-07) (sahte yol birikimi) doğrudan
bunun sonucu. **Bu düzeltilmeden hiçbir bag verisi güvenilir değildir.**

**Öneri:**
1. Test oturumlarını izole et: `prototype/tests/conftest.py`'ye
   `os.environ["ROS_DOMAIN_ID"] = "99"` (araçtan farklı) koy — tek satır, tüm testleri kapsar.
2. Daha güçlüsü: `ROS_LOCALHOST_ONLY=1` ekle; testler ağa hiç çıkmasın.
3. CI'da ve saha bilgisayarında `pytest`'i araç servisi koşarken çalıştırmayı
   **engelle** (servis aktifse test hedefi hata versin).
4. `fusion_node`'a kaynak doğrulaması ekle: `status.service == 0` **veya**
   `position_covariance` tamamen sıfır olan `NavSatFix` mesajlarını reddet. Bu,
   sızıntıyı gelecekte de zararsız kılar (savunma derinliği).

---

## PAR-02

### Sistem saati senkronsuz başlıyor; oturum ortasında 20.676 günlük sıçrama

**Şiddet:** 🔴 Kritik

**Kanıt** — 14 oturumun **3'ü** 1970 tarihiyle açılmış
(`session_19700101_020119`, `_020120`, `_020215`). `session_19700101_020215`'te
saat oturum **ortasında** sıçrıyor:

```
t =           771,09     (1970-01-01 ...)
t = 1786405459,54        (2026-08-11 ...)
fark ≈ 1.786.404.688 s ≈ 20.676 gün
```

**Kök neden:** Jetson'da RTC pili yok; boot'ta saat 1970'ten başlıyor ve kıyı
yordamı (`sudo date -s` ya da tethering gelince NTP adımı) saati **düğümler
çalışırken** düzeltiyor. Kaynakta bu risk zaten yazılı
(`duba_gecis_navigator.py:70-81`, "⏱️ SAAT KURALI"): süre/timeout için
`time.monotonic()`, mutlak an için `time.time()` kullanılmalı — çünkü duvar saati
ileri sıçrarsa geçiş penceresi (`pass_bitis_t`) anında dolar, geri sıçrarsa
`durum_log` susar ve Dosya-1 kaydı durur.

**Etki:**
- Tüm `header.stamp` değerleri geçersiz → TF sorguları, `ApproximateTimeSynchronizer`
  eşleşmesi ve iSAM2 zaman entegrasyonu bozuluyor.
- Şartname md 4.2 zorunlu çıktıları (Dosya-1 mp4, Dosya-2 CSV, Dosya-3 png)
  **zaman etiketli** olmak zorunda; 1970 damgalı teslim geçersizdir.
- Bu üç oturumun frekans/boşluk ölçümleri elle ayıklanmadan kullanılamaz —
  [algi.md ALG-06](algi.md#alg-06) doğrudan bunun sonucu.

**Öneri:**
1. `girdap-karar.service` ve `girdap-rosbag.service`'e
   `After=time-sync.target` + `ExecStartPre=/usr/bin/timeout 60 /bin/bash -c 'until timedatectl show -p NTPSynchronized --value | grep -q yes; do sleep 2; done'`
   ekle — saat oturmadan yığın açılmasın.
2. Donanım çözümü asıl çözümdür: **Jetson'a RTC pili tak** (birkaç TL, kalıcı).
3. `girdap_saat_kur.py` zaten var (`scripts/`); systemd zincirine **zorunlu**
   önkoşul olarak bağla, "best effort" bırakma.
4. Saat sıçraması olursa kayıt oturumunu **döndür** (yeni bag dosyası) ki tek bir
   dosyada iki farklı zaman tabanı bulunmasın.

---

## PAR-03

### Araç hiçbir oturumda ARM edilmedi

**Şiddet:** 🔴 Kritik

**Kanıt** — 14 oturumun tamamındaki **her** `/mavros/state` mesajı tarandı:

```
TOPLAM /mavros/state mesajı : 41.524
armed  = true olan          :      0
guided = true olan          :      0
```

| (connected, armed, guided) | adet |
|---|---:|
| `(true,  false, false)` | 26.884 |
| `(false, false, false)` | 14.640 |

Oturum bazında `armed=true` sayısı: `020119`:0/81 · `020120`:0/11.941 ·
`020215`:0/20.641 · `130029`:0/992 · `143741`:0/1.804 · `145923`:0/397 ·
`151706`:0/1.761 · `154109`:0/2.243 · `163939`:0/1.660 · kalan oturumlar 0/1.

**Bulgu:** Bu, [KAR-04](karar.md#kar-04)'ün (`thrust` hep `[0,0]`) **tek ve yeterli**
açıklamasıdır ve KAR-04'teki "her oturumda farklı bir kilit" tablosundan daha
basittir. `mavros_bridge.control_gate()` disarm hâlinde `zero_thrust=True` ve
`allow_cmd_vel=False` döndürür (`planning_node.py:1119,1129`); araç hiç ARM
edilmediği için kontrol katmanı **doğru davranarak** sıfır itki basmıştır.
Yani KAR-04 bir yazılım arızası değil, bu bulgunun semptomudur.

**Açık tutarsızlık (araştırılmalı):** `mission_fsm` `ARM → BEKLEMEDE` geçişini
`obs.kill_switch_off`'a bağlar ve bu alan `fsm_node.py:569`'da doğrudan
`self._mav_armed`'dan beslenir. `armed` hiç `true` olmadıysa FSM `ARM`'da kalmalıydı;
oysa `session_20260811_151706`'da `BEKLEMEDE` 15.965 örnekle baskın ve `ARM` yalnız
**1** örnek. Bu, [KAR-01](karar.md#kar-01)'in "`/girdap/mission/state`'e ikinci bir
yayıncı yazıyor" hipotezi için **bağımsız bir kanıttır** — çünkü kaydedilen MAVROS
verisiyle kaydedilen FSM durumu birbirini tutmuyor.

**Etki:** Hiçbir oturumda gerçek bir otonomi denemesi yapılmamış. Bag'lerdeki
"görev başlamadı" bulgularının tamamı bu tek gerçeğin türevidir; kontrol/planlama
katmanı hakkında bu veriden **performans sonucu çıkarılamaz**.

**Öneri:**
1. Ölçüm öncesi kontrol listesine "ARM teyidi" koy:
   `ros2 topic echo /mavros/state --once` çıktısında `armed: true` görülmeden
   koşu başlatılmasın.
2. `fsm_node`, `armed=false` iken 30 s'den fazla beklerse **pre-arm reddi sebebini**
   `/diagnostics`'e bassın — ArduPilot pre-arm mesajı `/mavros/statustext/recv`'de
   zaten geliyor, okunup ilişkilendirilmeli. (Repo geçmişinde `769f3c0` "Kotu AHRS =
   kalibrasyonlar GECERSIZ (arm reddediliyor)" commit'i tam bu sorunu işaret ediyor.)
3. `/mavros/statustext/recv` topic'ini **rosbag kayıt listesine ekle** — şu an
   kaydedilmiyor, bu yüzden arm reddinin sebebi bag'den okunamıyor.

---

## PAR-04

### MAVLink akış hızı heartbeat bütçesinin altında → KILL

**Şiddet:** 🔴 Kritik

**Kanıt** — `session_20260811_130029` (96 dakika, daha önce analiz edilmemiş oturum):

| Topic | Ölçülen | Beklenen |
|---|---:|---:|
| `/mavros/state` | **0,17 Hz** (≈6 s aralık) | 1 Hz |
| `/mavros/imu/data` | 1,27 Hz | ≥10 Hz |
| `/mavros/global_position/global` | 1,27 Hz | 1-10 Hz |
| `/girdap/fusion/odom` | 1,69 Hz | 10 Hz |
| `/girdap/mission/state` | 1,72 Hz | 10 Hz |
| `/livox/lidar` | 10,00 Hz | 10 Hz ✅ |

Aynı oturumun FSM dağılımı: **`KILL` 8.561 (%86)** · `BEKLEMEDE` 857 · `ARM` 497 ·
`BOOT` 31. `/girdap/control/thrust` topic'i **hiç yok**.

**Kök neden:** `mavros_bridge_node`'un `heartbeat_timeout_s` değeri **5,0 s**
(`hardware.yaml:26`). `/mavros/state` 6 saniyede bir geldiğinde bu eşik **her
aralıkta** aşılıyor → köprü KILL'e latch'liyor. Kaynakta bu risk açıkça yazılı
(`hardware.yaml:61-69`): ArduPilot taze bağlantıda `SR0_*` parametrelerine göre
~1 Hz yayınlıyor; bu yüzden `stream_rate_hz: 10` isteği eklenmiş ve
*"⚠ ALT SINIR: fusion_node pose_timeout_s=1.0 → 1-2 Hz'de odom yayını KESİLİR.
5 Hz'in altına İNME"* uyarısı düşülmüş.

Ölçüm bu uyarının **gerçekleştiğini** gösteriyor: LiDAR (ayrı sürücü, ethernet)
tertemiz 10 Hz koşarken MAVLink kaynaklı her şey ~1,3 Hz'de. Yani darboğaz
Jetson ya da yığın değil, **seri hat / akış hızı isteği**.

Bu ayrıca [KAR-02](karar.md#kar-02) (KILL'den çıkış yok) ile birleşiyor: köprü
KILL'e girdikten sonra kendi kendine toparlanamıyor, oturumun kalan 86'sı ölü geçiyor.

**Etki:** Oturumun tamamı kayıp. Yarışmada aynı durum görev iptali demektir.

**Öneri:**
1. `stream_rate_hz` isteğinin **gerçekten uygulandığını doğrula**: bağlantıdan sonra
   `/mavros/state` frekansını ölç, 5 Hz altındaysa gürültülü `ERROR` bas ve
   isteği tekrarla (şu an tek sefer isteniyor, teyit edilmiyor).
2. FC tarafında `SR0_EXTRA1`/`SR0_EXT_STAT` parametrelerini kalıcı yaz —
   oturumluk istek yerine EEPROM'a. Hat 57600 baud olduğu için toplam bütçeyi hesapla.
3. `heartbeat_timeout_s` ile gerçek akış hızı arasında **tutarlılık kontrolü** ekle:
   ölçülen `/mavros/state` periyodu timeout'un yarısını aşıyorsa başlangıçta uyar.
4. F-M.10 (`auto_recover`, çalışma ağacında commit edilmemiş) bu oturumu
   kurtarırdı — bkz. [PAR-10](#par-10) ve `karar.md` KAR-02.

---

## PAR-05

### MPPI `K`/`T` ayarları Jetson'ın 10 Hz bütçesini aşıyor

**Şiddet:** 🟠 Yüksek

**Kanıt** — kontrol döngüsü periyodu, `session_20260811_163939` (ayrıntı ve diğer
oturumlar: [KAR-11](karar.md#kar-11)):

| t (s) | ölçülen periyot | bütçe |
|---:|---:|---:|
| 0 | **117 ms** | 100 ms |
| 140 | 326 ms | 100 ms |
| 480 | 670 ms | 100 ms |
| 1665 | **1.062 ms** | 100 ms |

**İki ayrı sorun var:**

1. **Başlangıçta bile bütçe aşılıyor** (117 ms > 100 ms). `control_rate_hz: 10.0`
   (`params.yaml:40`) seçilirken dayanak CLAUDE.md'deki *"CPU'da K=1000 rollout
   ~100 ms"* ölçümüydü — ama o ölçüm **`n_ref≈114`'lük demo sahnesine** ait ve
   geliştirme makinesinde (Ryzen) yapılmış. Jetson Orin Nano'da, ~120 gerçek engelle,
   aynı ayar 117 ms'den başlıyor.
2. **Periyot zamanla 10 katına çıkıyor** — girdi yükü sabitken (kare başına
   `classified_obstacles` tespit sayısı 107-130 arasında sabit). Yani biriken bir
   iç durum var; en güçlü aday `_huni_payi`'nin O(n²) saf Python taraması
   (`planning_node.py`, "torba sınırsız büyüyor" notu).

**Etki:** MPPI'nin ürettiği komut, hesaplandığı ana ait değil. 1 saniyelik periyotta
1 m/s'lik araç **1 metre** yol almış oluyor. CLAUDE.md'nin kendi uyarısı
(*"executor birikir, cmd_vel gecikir/titrer → istemsiz hareket"*) 20 Hz için
yazılmıştı; ölçüm 10 Hz'de de geçerli olduğunu gösteriyor.

**Öneri:**
1. `K`'yı 1000 → 400 indirip Jetson'da yeniden ölç. CLAUDE.md'deki tüm K=1000
   ölçümleri geliştirme makinesine ait; **Jetson'da hiç doğrulanmamış.**
2. Kontrol döngüsüne **aşım sayacı** koy: adım süresi periyodu aşarsa
   `/diagnostics`'e bas. Şu an aşım tamamen görünmez.
3. Kök nedeni profille (`cProfile`, 60 s saha kaydı) — `_huni_papi` O(n²) taraması
   doğrulanırsa uzamsal indeks (KD-ağacı) ile O(n log n)'e indir.
4. `birlestirme_s` gibi bu da tarih ayracıdır: ölçüm regresyonunu CI'ya bağla.

---

## PAR-06

### HSV doygunluk eşiği sahada ölçülen değerin çok üstünde

**Şiddet:** 🟠 Yüksek

**Kanıt** — `params.yaml:233-236`:

```yaml
hsv_orange_lo: [5, 120, 120]     # H, S, V alt sınırı
hsv_yellow_lo: [21, 120, 120]
```

Kaynağın kendi notu (`params.yaml:231-232`, 2026-07-16 gerçek donanım testi):
akşamüstü ışığında gerçek turuncu/sarı dubanın ölçülen doygunluğu **S ≈ 29-83**.
Eşik **S ≥ 120** → hiçbiri tespit edilmedi.

Bag tarafındaki karşılığı: [ALG-01](algi.md#alg-01) (engellerin %99,96'sı sınıf 99)
ve [ALG-04](algi.md#alg-04) (turuncu duba pratikte hiç tespit edilmiyor).

**Kök neden:** Eşikler RAL renk kodlarından **teorik olarak** türetilmiş, sahada
ölçülerek değil. `equalize_saturation()` telafisi eklenmiş
(`saturation_clahe: true`) ama yalnız sahne **genelinde** düşük doygunsa devreye
giriyor — parlak su + mat duba sahnesinde sahne geneli doygun görünüp telafi
tetiklenmiyor.

⚠ Kırmızı/yeşil/kahverengi eşikleri (`hsv_red_*`, `hsv_green_*`, `hsv_brown_*`)
sahada **hiç doğrulanmadı** — kaynak bunu açıkça "ilk tahmin, kör güvenilmemeli"
diye işaretliyor. Parkur-3 hedef rengi bu üçünden seçileceği için risk doğrudan
puana bağlı.

**Öneri:**
1. Sahada duba fotoğrafı çek, gerçek H/S/V dağılımını histogramla ölç, alt sınırı
   ölçülen dağılımın **%5 yüzdeliğine** koy (tahmin değil, veri).
2. Kısa vadeli emniyet: `S` alt sınırını 120 → **60** indir (ölçülen 29-83 bandının
   ortası), yanlış pozitifleri `min_area_px` ve şekil filtresiyle ele.
3. `equalize_saturation()`'ı sahne geneline değil **ROI'ye** (bbox içi) uygula.
4. Asıl çözüm HSV değil: `camera_buoys.py`'deki `BuoyLocalizer` + `classify_roi_color()`
   deseni (YOLO sınıfsız duba bulur, rengi bbox içi HSV karar verir) —
   `hardware.yaml:214-219` bunu zaten öneriyor.

---

## PAR-07

### Algı menzil sabitleri sahada hiç doğrulanmadı

**Şiddet:** 🟠 Yüksek

**Kanıt** — `duba_gecis_navigator.py:287`:

```python
GECIT_MAX_MESAFE = 8.0   # 2026-08-04'te 15.0'dan İNDİRİLDİ
```

Gerekçe kaynakta iki bağımsız fizik sınırına dayanıyor: (a) HFOV 69°, NN girişi
416 px → 30 cm duba 15 m'de ~6 px (YOLO için pratikte yok), 8 m'de ~11 px;
(b) OAK-D Lite stereo baseline 7,5 cm → ~8 m ötesinde Z hatası **metrelerce**.
Kaynak notu: *"Sahada ölçülünce (gerçek duba, gerçek model) bu sayı GÜNCELLENECEK"*
— **güncellenmedi.**

Aynı durumdaki diğer sabitler:

| Sabit | Değer | Dosya | Durum |
|---|---|---|---|
| `GECIT_MAX_MESAFE` | 8,0 m | `duba_gecis_navigator.py:287` | ⏳ ölçülmedi |
| `MENZIL_BAGIL_TOL` | 0,35 | `:290` | ⏳ ölçülmedi |
| `GECIT_AYIRT_M` | 3,0 m | `:293` | ⏳ ölçülmedi |
| `PASS_TETIK_Z` / `PASS_KAYIP_Z` | 2,0 / 3,2 m | `:303-304` | ⏳ ölçülmedi |
| `bearing_tolerance_rad` | 0,15 (8,6°) | `params.yaml:257` | ⏳ ölçülmedi |
| `camera_hfov_rad` | 1,2 ("yaklaşık değer") | `params.yaml:258` | ⏳ ölçülmedi |

**Etki:** `/perception/gate_target` ve `/perception/gate_count`'ın neredeyse hiç
yayınlanmaması bu eşiklerin doğrudan sonucu (bkz. [algi.md — arıza olmayan
gözlemler](algi.md#not-arıza-olmayan-gözlemler)). Kapı 8 m'den uzaktaysa **hiçbir
uyarı basılmadan** sessizce hiçbir şey üretilmiyor.

**Öneri:**
1. Sahada tek bir kalibrasyon koşusu bu tablonun tamamını doldurur: bilinen
   mesafelere duba koy, `/perception/buoys_3d`'deki `position.x`'i şeritle ölçülen
   mesafeyle karşılaştır. Yarım saatlik iş.
2. `gecit_bul()`'un reddettiği çift sayaçlarını (`_tani["dar"]`, `["dizili"]`,
   `["arada_duba"]`) periyodik olarak **logla** — şu an yalnız bellekte, sahada
   "neden kapı bulunmuyor" sorusu cevapsız kalıyor.
3. `camera_hfov_rad`'ı OAK-D `CameraInfo`'sundan **oku**, sabit yazma.

---

## PAR-08

### PID kazançları itki ölçeğiyle 10× tutarsız — saha yedeği çalışmaz

**Şiddet:** 🟠 Yüksek

**Kanıt** — iki dosya arasındaki ölçek uyuşmazlığı:

| Parametre | Değer | Dosya |
|---|---:|---|
| `max_thrust` (tek motor doygunluğu) | **1,455 N** | `prototype/configs/dynamics.yaml:48` |
| `PidControllerConfig.cruise_thrust_n` | **15,0 N** | `prototype/planning/pid_controller.py:57` |
| `PidControllerConfig.max_diff_thrust_n` | **15,0 N** | `pid_controller.py:58` |

PID yolu, doygunluk sınırının **10 katı** komut üretiyor.

**Kök neden:** 2026-08-05'te log 58 sistem tanılamasıyla `max_thrust` 30,0 → 1,455 N
düşürüldü ve MPPI tarafı buna göre güncellendi (`sigma_u` 5,0 → 0,364 = 0,25×itki).
Aynı güncelleme **PID dosyasına uygulanmadı** — 15,0 N değerleri eski 30 N'lik
tekneden kalma (30/2 = 15).

**Neden bugün canlı bir arıza değil:** varsayılan `control_mode: "mppi"`
(`pipeline.py`), yani PID yolu koşmuyor. Bag'lerde de doğrulandı: hiçbir oturumda
PID kolu devrede değil.

**Neden yine de önemli:** F-S.10 bu yolu açıkça *"MPPI saha kalibrasyonu tamamlanana
kadar ya da beklenmedik davranışta düşme-güvenli yedek"* diye tanımlıyor. Yani
sahada MPPI bozulursa `planning.control_mode:=pid` verilmesi **öngörülmüş bir
kurtarma yolu** — ve o an bu değerlerle devreye girerse yedek çalışmaz, doygunlukta
takılı kalır (her komut ±1,455'e kırpılır, diferansiyel yetki kaybolur).
[PAR-05](#par-05) MPPI'nin gerçekten zorlandığını gösterdiği için bu yedeğe ihtiyaç
duyulma olasılığı **düşük değil**.

**Öneri:**
1. `cruise_thrust_n` ve `max_diff_thrust_n`'i `CatamaranParams.max_thrust`'tan
   **türet**, ayrı sabit tutma (ör. `cruise = 0,5 × max_thrust`,
   `max_diff = max_thrust`). Böylece itki değeri güncellenince PID kendiliğinden takip eder.
2. Bir **tutarlılık testi** ekle: `pid_cfg.cruise_thrust_n <= params.max_thrust`
   assert'i. Aynı sınıftaki gelecek sürüklenmeleri CI'da yakalar.
3. Saha yedeğine geçmeden önce yedeğin **masada bir kez koşturulduğunu** doğrula —
   hiç çalıştırılmamış bir yedek, yedek değildir.

---

## PAR-09

### `mission_source: fc` ile çoklu parkur görev dosyası çelişiyor

**Şiddet:** 🟠 Yüksek

**Kanıt** — `hardware.yaml`'da aynı anda tanımlı:

```yaml
mission:
  mission_source: "fc"                        # satır 194
  mission_file: "competition_mission.yaml"    # satır 183  → parkur etiketleri 1,1,2,2,3
```

Bag tarafındaki karşılığı: **14 oturumun tamamında `/girdap/parkur/state` yalnız
`PARKUR_1`** (s130029 dahil, hiç geçiş yok) ve `/girdap/mission/current_target`
**0 mesaj**.

**Kök neden:** `mission_source=fc` iken gerçek waypoint'ler FC'den (QGC yüklemesi)
geliyor ve `fc_items_to_waypoints_with_seqs` bunlara **her zaman `parkur=1`** veriyor
(FC görev formatı parkur etiketi taşımaz). Parkur **sınırları** ise hâlâ statik
`competition_mission.yaml`'dan okunuyor. İki kaynak senkron değil.

`fsm_node._build_parkur_logic():315-325` bu çelişkiyi tespit edip `ERROR` basıyor
ama **düzeltmiyor ve başlatmayı durdurmuyor** — yığın çelişkili konfigürasyonla
koşmaya devam ediyor.

İkinci sorun: `competition_mission.yaml`'daki **tüm koordinatlar `0.0` placeholder**:

```yaml
- {lat: 0.0, lon: 0.0, parkur: 1, name: "P1_WP1"}
```

`mission_manager_node`'un `max_target_distance_m: 10000.0` bekçisi bu yüzden
görevi reddediyor (gerçek konum ile (0,0) arası binlerce km) — `current_target`'ın
hiç yayınlanmamasının ikinci açıklaması budur.

**Etki:** Parkur geçişi hiç tetiklenemez → [KAR-08](karar.md#kar-08).

**Öneri:**
1. Çelişkiyi **başlangıçta reddet**: `mission_source=fc` **ve** `mission_file`
   birden fazla parkur içeriyorsa düğüm `RuntimeError` ile çıksın. `ERROR` loglayıp
   devam etmek, sahada fark edilmeyen sessiz arıza üretiyor.
2. Parkur etiketlerini FC görevine taşı: QGC'de her waypoint'in `param1` alanına
   parkur numarası yazılabilir; parser tek fonksiyonda izole
   (`parkur_fsm.load_parkur_labels`), yalnız orası değişir.
3. Placeholder koordinatları **reddet**: `lat == 0.0 and lon == 0.0` olan waypoint
   görev dosyasında varsa yükleme hata versin.
4. Ölçüm öncesi kontrol listesine "QGC görevi yüklendi + `/mavros/mission/waypoints`
   dolu" maddesi ekle.

---

## PAR-10

### Kayıt servisi düzgün kapatılmıyor — 13/14 bag sonlandırılmamış

**Şiddet:** 🟡 Orta

**Kanıt** — her `.mcap` dosyasının sonundaki MCAP kapanış imzası (magic + footer)
kontrol edildi:

| Durum | Oturum sayısı |
|---|---:|
| Düzgün kapatılmış (footer + özet + indeks var) | **1** (`session_20260811_130029`, `.mcap.zstd`) |
| Sonlandırılmamış (footer yok, indeks yok) | **13** |

`girdap-rosbag.service` `ros2 bag record ... --compression-mode file
--compression-format zstd` ile koşuyor: dosya **kapanışta** sıkıştırılıyor.
Yalnız `130029` `.mcap.zstd` olarak bitmiş; kalan 13'ü ham `.mcap` kalmış —
yani kayıt süreci **SIGKILL ile öldürülmüş**, düzgün durdurulmamış.

**Etki:**
1. **Özet/indeks bölümü yok** → her okuma tam dosya taraması gerektiriyor.
   Bu analizde 13,6 GB'lık oturum tek başına ~20 dakika sürdü; toplam maliyeti
   saatlerce. (Bu raporun ölçümleri, indeks yerine `MessageIndex` kayıtlarını okuyan
   [`araclar/mcapscan.py`](araclar/mcapscan.py) ile alındığı için 46 GB'ın tamamı
   **15 saniyede** tarandı — ama bu bir çözüm değil, geçici çare.)
2. Son chunk'taki mesajlar (birkaç yüz) kayıp.
3. `ros2 bag info` "no message indices found" uyarısı veriyor; standart araçlarla
   (Foxglove, PlotJuggler) açmak yavaş veya imkânsız.

**Öneri:**
1. `girdap-rosbag.service`'e `KillSignal=SIGINT` + `TimeoutStopSec=120` ekle —
   `ros2 bag record` SIGINT'te dosyayı düzgün kapatır, SIGKILL'de kapatmaz.
   (`systemd` varsayılanı SIGTERM/90 s; kapanış sıkıştırması 8,5 GB'da bunu aşıyor
   olabilir — süreyi cömert tut.)
2. Kapanış sıkıştırmasını **kapat** (`--compression-mode none`): 8,5 GB'lık dosyayı
   kapanışta sıkıştırmak dakikalar sürüyor ve tam da bu yüzden timeout'a takılıp
   SIGKILL yiyor. Sıkıştırma isteniyorsa `--compression-mode message` kullan
   (akış hâlinde, kapanışta iş kalmaz).
3. Kayıt dosyalarını **döndür** (`--max-bag-size`), tek 13,6 GB dosya yerine
   1 GB'lık parçalar: hem çökme dayanımı hem analiz hızı.

---

## Ek: bu kümedeki bulguların birbirine bağı

```
PAR-02 (saat yok)  ──→ ALG-06 (56 yıl bayat stamp) ──→ füzyon/TF geçersiz
PAR-01 (test sızıntısı) ──→ KAR-06 (ışınlanma) ──→ KAR-07 (sahte yol)
                        └─→ KAR-01 (çelişkili durum akışı)
PAR-04 (akış hızı) ──→ KILL latch ──→ KAR-02 ──→ KAR-04 (sıfır itki)
PAR-03 (hiç ARM yok) ──────────────────────────→ KAR-04 (sıfır itki)  ← asıl neden
PAR-09 (görev config) ──→ KAR-08 (PARKUR'a geçilemedi)
PAR-05 (MPPI bütçesi) ──→ KAR-11 (kontrol döngüsü 10× yavaş)
PAR-06/07 (eşikler) ──→ ALG-01/04 (sınıflandırma çalışmıyor)
```

**Düzeltme sırası önerisi** — üstteki üçü çözülmeden aşağıdakiler ölçülemez:

1. **PAR-01** (test izolasyonu) — bir satır, veri güvenilirliğinin önkoşulu.
2. **PAR-02** (saat) — RTC pili + systemd `time-sync.target`.
3. **PAR-03** (ARM) — arm reddi sebebini görünür kıl, `statustext/recv`'i kaydet.
4. **PAR-04** (akış hızı) — heartbeat/stream tutarlılığı.
5. Kalanlar (PAR-05…PAR-10) yukarıdakiler düzeldikten sonra **yeniden ölçülmeli**;
   bu bag setinden çıkarılan performans sonuçları o zamana kadar geçici sayılmalı.

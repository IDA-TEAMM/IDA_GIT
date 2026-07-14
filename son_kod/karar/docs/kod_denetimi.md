# Kod Denetimi — girdap-decision

> **Ne bu?** Algı ekibinin (girdap-ida-algi) yığın üzerinde yürüttüğü baştan sona
> kod denetimi. Karar yazılımcısı geçici olarak müsait olmadığı için depo
> `EyupEker1/girdap-decision` altına **fork**landı (git geçmişi + `upstream`
> remote korunuyor, geri merge edilebilir).
>
> **Başlangıç noktası:** `d4ce88b` (upstream/main ile birebir aynıydı).
> **Yöntem:** dosya dosya okuma, her iddia `dosya:satır` kanıtıyla.
> **Kural:** bitmiş ve test edilmiş kod yeniden yazılmaz; yalnız kanıtlanmış
> hatalar hedefli olarak düzeltilir.

**Şiddet ölçeği:** 🔴 yarışmayı/parkuru kırar · 🟠 kurulum/deploy kırar ·
🟡 gerçek hata, bugün etkisi sınırlı · ⚪ kozmetik / doküman

---

## Faz 18 — UYGULANAN DÜZELTMELER (video, test-doğrulamalı)

> Test ortamı: `source /opt/ros/humble/setup.bash` + saf-Python çekirdek testleri
> (numpy 2.2.6). `mavros_msgs`/gtsam kurulu değil → mavros/füzyon **node**'ları
> bu makinede kurulamaz; çekirdek mantıkları test edilir, node değişiklikleri
> `py_compile` + gerektiğinde saha/HIL doğrulaması ister.

| Bulgu | Düzeltme | Doğrulama | Commit |
|---|---|---|---|
| 🔴 F11.1/F9.1 MPPI warm-start kaybı | `pipeline._rebuild_mppi` kontrolcüyü koruyor (config aynıysa referans/engel günceller, değişince U_nominal taşır); `MPPIController.set_obstacles()` | 2 yeni test (TDD kırmızı→yeşil), 97 test | `aaf3f73` |
| 🔴 F12.2 video terminal durumu yok | `Observation.mission_complete` + PARKUR*→TAMAMLANDI; `mission_manager_node` `/girdap/mission/complete` yayınlar, `fsm_node` tüketir | 3 yeni FSM testi; `mission_manager_node` ROS testi 2 geçti | `ec7e1f5` |
| 🔴 F15.1 Dosya-2 göreli yol (5 ceza) | `telemetry_node` varsayılan `""` → `~/girdap_logs/telemetry` mutlak | `TelemetryNode` `/tmp`'den kuruldu, CSV home altında oluştu | `c2308a2` |
| 🟡 F4.2 control_rate 20→10 Hz | CPU MPPI ~100 ms; 10 Hz gerçekçi | config | `c2308a2`→ayrı |
| 🔴 F14.1 KILL disarm etmiyor | `_trigger_kill` `/mavros/cmd/arming False` çağırır | node py_compile; HIL gerek | `0c7e1b6` |
| 🔴 F14.2 kasıtlı disarm=failsafe | `MavrosBridge.note_command_disarm()`+`is_unexpected_disarm()`; node bağlandı | 4 yeni çekirdek testi (20 toplam) | `0c7e1b6` |
| 🔴 T0-f görev yalnız YAML'dan (md 3.3.1(2)/5.5.2.2 ihlali) | `mission_source: {file, fc}`; fc modu `/mavros/mission/waypoints` (WaypointList) okur → `fc_items_to_waypoints` çekirdeği | 7 çekirdek + 2 ROS node testi (mavros'suz duck-typed) | `3856fb0` |
| 🟠 T0-g thrust kaydedilmiyor (Ekran-2c) + 🟡 F15.4 hız yedeği | ayrı `grafik_<UTC>.csv` (10 Hz) + odom hız yedeği; Dosya-2 header donduruldu | 5 çekirdek + 1 ROS node testi | `08f3fb5` |
| 🟠 F14.3 auto_guided manuel dönüşte kavga | auto-GUIDED görev-aktife (PARKUR1/2/3) bağlandı — `MavrosBridge.set_mission_state()`, node `/girdap/mission/state` dinler | 3 yeni çekirdek testi (23 toplam); node py_compile | ↓ bu commit |

**Kalan video işleri:** KOD TARAFI BİTTİ; kalan saha/HIL: suda 4 nokta provası
(T0-h), gerçek FCU'da KILL/arm zinciri, QGC→MAVROS görev yükleme teyidi.
F12.1 (last_waypoint_xy — F12.2 kapsadı, düştü). **MPPI/FSM/telemetri çekirdek
matematiğine dokunulmadı** — tümü sağlamdı; düzeltmeler yaşam
döngüsü/config/yol. **T0-f/T0-g** yeni kod, çekirdek math değil.

---

## Faz 1 — Kod haritası ✅

| Katman | Satır |
|---|---|
| `prototype/` (Layer 0, saf NumPy çekirdek) | 5.137 |
| `ros2_ws/` (Layer 2, ROS sarmalayıcı) | 3.170 |
| `prototype/tests/` (161 test fonksiyonu, 20 dosya) | 3.186 |
| **Toplam** | **11.493** |

Ağaçta **sıfır** `TODO` / `FIXME` / `NotImplementedError` / `XXX`. Yığın yarım
değil; esasen bitmiş. Denetimin amacı bu yüzden "tamamlamak" değil, **sessiz
hataları bulmak**.

`CLAUDE.md`'nin klasör şemasında görünen `cpp/` (Layer 1) dizini **mevcut
değil** — şema özlem, gerçek değil. (bkz. Faz 17)

---

## Faz 2 — Bağımlılık & paketleme ✅

### 🔴 F2.1 — `package.xml` iki ROS bağımlılığını beyan etmiyor

**Dosya:** `ros2_ws/src/girdap_decision/package.xml:14-21`

Kodun gerçekte kullandığı ama `<depend>` listesinde **olmayan** paketler:

| Paket | Nerede kullanılıyor | package.xml |
|---|---|---|
| `vision_msgs` | `perception_camera_node.py:34` (`from vision_msgs.msg import ...`), `perception_fusion_node.py` (`Detection3DArray`) | ❌ yok |
| `message_filters` | `perception_fusion_node.py:33` (`import message_filters`) | ❌ yok |
| `python3-pillow` | `prototype/mapping/local_map.py:29` (`from PIL import Image`) → **Dosya-3 üretimi** | ❌ yok |

Beyan edilenler: `rclpy`, `std_msgs`, `std_srvs`, `geometry_msgs`, `nav_msgs`,
`sensor_msgs`, `mavros_msgs`, `visualization_msgs`.

**Etki:** Temiz bir Jetson'da `rosdep install --from-paths src --ignore-src -y`
bu üç paketi **kurmaz**. `perception_camera_node`, `perception_fusion_node` ve
`local_map_node` çalışma anında `ModuleNotFoundError` ile ölür. `local_map_node`
ölürse **şartname Dosya-3 üretilmez → 5 ceza puanı**.

Algı tarafı bu eksiği zaten fark etmişti (bizim README'de `sudo apt install
ros-humble-vision-msgs` elle kurulum notu var) — ama kaynağı burası.

**Düzeltme:**
```xml
<depend>vision_msgs</depend>
<depend>message_filters</depend>
<exec_depend>python3-pillow</exec_depend>
```

### 🟠 F2.2 — `numpy` üst sınırı yok

**Dosya:** `requirements.txt:1` → `numpy>=1.26`

Jetson'da temiz pip kurulumu `numpy 2.x` çeker; ROS Humble ve sistem `scipy`'si
numpy 1.x ABI'sine derli → çalışma anında `_ARRAY_API not found`.

Kendisi bu riski `docs/jetson_deployment.md`'de **yazmış**; yalnız pinlemeyi
atlamış. Ayrıca `perception_camera_node` docstring'i de aynı ABI sorununu
`cv_bridge` gerekçesi olarak anlatıyor — yani biliniyor.

**Düzeltme:** `numpy>=1.26,<2`

### 🟠 F2.3 — `prototype/` kurulmuyor; runtime `PYTHONPATH` hack'ine bağlı

**Dosyalar:** `ros2_ws/src/girdap_decision/setup.py:18`, `docs/jetson_deployment.md:141`

11 ROS node'unun tamamı `from prototype.<modül> import ...` yapıyor
(`planning_node.py:53-55`, `fsm_node.py:71`, `telemetry_node.py:47`,
`perception_fusion_node.py:45`, `local_map_node.py:29`, …).

Ama `setup.py`'daki `find_packages(exclude=["test"])` çağrısı
`ros2_ws/src/girdap_decision/` dizininden koşuyor → yalnız `girdap_decision`
paketini bulur. `prototype/` depo kökünde, dört dizin yukarıda; **kurulmuyor.**

`setup.py`'ın kendi docstring'i bunu kabul ediyor:
> "Prototip kodu (prototype/) bu pakete bağımlıdır; çalıştırmadan önce repo
> kökü PYTHONPATH'e eklenmeli"

ve `jetson_deployment.md:141` çözümü veriyor:
```bash
export PYTHONPATH=$HOME/girdap-decision:$PYTHONPATH
```

**Etki:** Bilinçli bir tasarım tercihi, hata değil — **ama kırılgan.** `systemd`
servisi kullanıcının `.bashrc`'sini source'lamaz; bu `export` orada olmazsa
node'lar boot'ta `ImportError` ile ölür. (Algı tarafında aynı tuzağa
`girdap-algi.service`'te düşmüştük.) Jetson'da `ros2 launch` bir servisten
koşacaksa `Environment=PYTHONPATH=...` satırı şart.

**Sağlam alternatif (sonraya):** `prototype`'ı ayrı bir `ament_python` paketi
yapmak veya `setup.py`'a `package_dir` ile dahil etmek.

### 🟡 F2.4 — `gtsam` sürümü dokümanla çelişiyor + ARM'de wheel yok

**Dosya:** `requirements.txt:4` → `gtsam>=4.3a0` (alpha sürüm tabanı)

`CLAUDE.md` iki yerde **GTSAM 4.2** diyor ("GTSAM 4.2+", "libgtsam-dev (4.2+)").
`requirements.txt` ise 4.3 alpha'yı taban alıyor. Hangisi doğru belirsiz.

Ayrıca `gtsam` PyPI'da ARM64 (Jetson) wheel'i genelde yayınlamaz → kaynaktan
derleme (saatler). `use_isam2: false` (video modu) bu yükü atlatıyor: `fusion_node.py:102`
GTSAM'ı **lazy import** ediyor, yani bypass modunda hiç yüklenmiyor. İyi tasarım.

**Aksiyon:** sürümü netleştir; Jetson'da `use_isam2: true` açılacaksa gtsam
derlemesi takvime girmeli.

### ⚪ F2.5 — `pyproject.toml` ruff yapılandırması eski şema

**Dosya:** `pyproject.toml:5-7` → `[tool.ruff] select = ["E","F","I"]`

Ruff 0.2+ bunu `[tool.ruff.lint] select = [...]` altına taşıdı; eski konum
uyarı veriyor (yakında hata). Kozmetik.

---

## Faz 3 — Launch kompozisyonu ✅

Dosya: `ros2_ws/src/girdap_decision/launch/hardware.launch.py` (361 satır)

### 🔴 F3.1 — OAK-D kamera sahipliği çakışıyor (iki tasarım aynı cihazı istiyor)

Bu, ilk sanıldığından daha derin. Zincir:

- `perception_camera_node.py:95-96` → **`/oak/rgb/image_raw`** topic'ine abone
  (`sensor_msgs/Image`, SensorDataQoS), üstünde HSV + mock YOLO koşturur,
  `/perception/buoys` yayınlar.
- `hardware.launch.py:314-317` → bu node **koşulsuz** başlatılıyor.
- `hardware.launch.py:353-358` → yorum, donanım ekibinden **OAK-D sürücüsünü
  (`depthai_ros`) bu launch'a eklemesini** istiyor: "OAK-D Lite (depthai_ros)
  → /oak/rgb/image_raw".

Algı tarafının (`girdap-ida-algi`) tasarımı ise farklı: YOLO **VPU üstünde**
koşuyor, `duba_gecis_navigator` OAK cihazını **doğrudan DepthAI ile açıyor** ve
`/perception/buoys`'u kendisi yayınlıyor. Ham görüntüyü ROS'a hiç basmıyor.

Bu iki tasarım **aynı USB cihazı üzerinde birlikte yaşayamaz**:

| Senaryo | Sonuç |
|---|---|
| `depthai_ros` sürücüsü eklenirse | Cihaz zaten bizim node tarafından açık → **sürücü OAK'ı açamaz** (USB exclusive). İki taraf da körleşir. |
| Sadece bizim node koşarsa (doğru kurulum) | `/oak/rgb/image_raw` yayınlayan yok → `perception_camera_node` callback'i **hiç tetiklenmez**, sessizce hiçbir şey yayınlamaz. Zararsız ama **zombi node**: loglarda canlı görünür, iş yapmaz. |

**Yani şu anki hâliyle "iki yayıncı çakışması" pratikte oluşmaz** (önceki
değerlendirmemiz burada fazla alarmcıydı) — ama tasarım niyeti çelişik ve
`depthai_ros` eklendiği anda **kamera tamamen ölür**.

**Düzeltme:**
1. `hardware.launch.py:314-317`'deki `perception_camera_node`'u kaldır ya da
   `use_mock_camera` LaunchArgument koşuluna bağla (varsayılan `false`).
2. `:353-358` yorumundaki "OAK-D sürücüsü buraya eklenecek" talimatını **sil** —
   OAK'ı algı node'u sahipleniyor. Livox sürücüsü talimatı geçerli kalır.

### 🟠 F3.2 — `use_mppi` ölü bir launch argümanı (yalan söyleyen bayrak)

`use_mppi` şurada geçiyor: `:49` (varsayılan), `:158` (`LaunchConfiguration`),
`:187` (`DeclareLaunchArgument`), `:349` (`LogInfo` ile **ekrana basılıyor**).

Ve **başka hiçbir yerde.** `girdap_decision/*.py` içinde `use_mppi` araması
**sıfır sonuç** veriyor — hiçbir node bu parametreyi almıyor.
`planning_params` (`:266-273`) yalnız `mode_name` ve `use_rrt` geçiriyor.

**Etki:** `hardware.yaml`'da `algorithm.use_mppi: false` yazmak **hiçbir şey
yapmaz**; üstelik `LogInfo` "mppi=false" diye basar, yani operatöre MPPI'nin
kapandığını **söyler**. `CLAUDE.md` de bunu gerçek bir bayrak olarak belgeliyor.
Yarışma günü MPPI'yi acil kapatmak gerekirse bu bayrağa güvenilemez.

**Düzeltme:** ya `use_mppi`'yi `planning_params`'a ekle ve `planning_node`'da
uygula, ya da argümanı ve log satırını tamamen kaldır. Yarısı olmaz.

### 🟠 F3.3 — `except Exception: pass` config hatasını yutuyor ve **tutarsız** moda düşüyor

```python
# :128-129
    except Exception:                       # paket kurulmadan --show-args vb.
        pass
```

Bu blok `hardware.yaml` okuma + **tüm `cast()` çağrılarını** (`:127`) kapsıyor.
Bozuk YAML, yanlış tipte bir değer, ya da eksik `config/` → hepsi sessizce
yutulur, **tek bir uyarı bile basılmaz**.

Geri düşülen varsayılanlar ise kendi içinde **tutarsız**:

| Varsayılan | Değer | Anlamı |
|---|---|---|
| `_ALGO_DEFAULTS` (`:49`) | `use_isam2=True, use_rrt=True` | **yarışma** modu (tam stack) |
| `_MISSION_DEFAULT` (`:51`) | `video_mission.yaml` | **video** görevi |

Yani config yüklenemezse araç, **video görev dosyasını yarışma algoritma
yığınıyla** koşar — ikisi bilerek birbirinden ayrılmış iki mod olmasına rağmen.
Video günü `hardware.yaml`'a girilen bir YAML yazım hatası, bypass edilmesi
istenen iSAM2 + RRT*'ı sessizce açar. Tam da `CLAUDE.md`'nin "video için
kırılgan katmanları bypass et" gerekçesinin tersi.

**Düzeltme:** `except Exception` yerine dar bir yakalama (`FileNotFoundError`,
`PackageNotFoundError`) + diğer her durumda `LogWarn` ya da yükselt. Ve iki
varsayılan bloğu aynı modu göstersin.

### 🟡 F3.4 — Static TF'ler 0,0,0 **ve hiçbir node TF okumuyor**

`:133-144` üç static transform yayıncısı kuruyor (`base_link` → `livox_frame`,
`oak_frame`, `imu_link`), hepsi identity (0,0,0,0,0,0), docstring'de "kalibre
EDİLMEMİŞ" diye kabul edilmiş.

Ama daha önemlisi: `girdap_decision/*.py` içinde `tf2_ros`, `TransformListener`
veya `lookup_transform` araması **sıfır sonuç**. Yani bu TF'ler yayınlanıyor,
**hiç kimse okumuyor**. Dekoratif.

**Etki:** Sensör extrinsic'i (kameranın direkteki yüksekliği/ofseti, LiDAR'ın
konumu) sisteme **hiçbir noktada girmiyor**. `fusion.associate()` kamera ve
LiDAR bearing'lerini, iki sensör de `base_link` orijinindeymiş gibi
karşılaştırıyor. Kısa mesafede (dubalar 5-20 m) açısal hata küçük kalır, ama
sensörler birbirinden ~0.5 m ayrıysa 5 m'deki bir dubada ~6° bearing farkı
doğar — `bearing_tolerance_rad = 0.15` (≈8.6°) eşiğinin yarısını tek başına
yer. Bearing işaret hatasıyla (F5.x) birlikte düşünülmeli.

### ⚪ F3.5 — Node sıralaması yorumu yanlış bir varsayıma dayanıyor

`:318-319`: "LiDAR+kamera node'larından SONRA gelmeli (mesajları tüketiyor)".
ROS 2'de `LaunchDescription` içindeki sıra node'ların **hazır olma** sırasını
garanti etmez; pub/sub keşfi asenkrondur ve `message_filters` zaten geç gelen
yayıncıyı bekler. Yorum zararsız ama yanıltıcı.

---

## Faz 4 — Config YAML ✅

Dosyalar: `config/hardware.yaml`, `config/params.yaml`

### 🔴 F4.1 — Dosya-2 (zorunlu telemetri CSV) **göreli yola** yazılıyor

**Zincir:**
- `telemetry_node.py:62` → `declare_parameter("csv_output_dir", "data/telemetry")`
- `params.yaml:92` → `csv_output_dir: "data/telemetry"` — **göreli yol**
- `prototype/telemetry/csv_logger.py:139-140` →
  `Path(output_dir).expanduser()` ardından `mkdir(parents=True, exist_ok=True)`

`expanduser()` göreli bir yolda **hiçbir şey yapmaz** (yalnız `~` açar). Sonuç:
CSV, sürecin **çalışma dizinine (cwd)** göre çözülür.

- `ros2 launch` ile: cwd = kullanıcının terminaldeki dizini → dosya rastgele bir
  yere düşer, 20 dakikada bulunması gerekiyor.
- `systemd` servisi ile (`WorkingDirectory=` verilmezse): **cwd = `/`** →
  `mkdir("/data/telemetry")` → root olmayan kullanıcı için **`PermissionError`**
  → telemetri node'u boot'ta ölür.

**Karşılaştırma — Dosya-3 bunu doğru yapıyor:** `prototype/mapping/local_map.py:66`
→ `Path.home() / "girdap_logs" / "local_map"` (mutlak, sağlam). İki zorunlu
çıktı iki farklı şekilde ele alınmış.

**Etki:** Şartname md 4.2 → Dosya-2 teslim edilemez → **5 ceza puanı**.

**Düzeltme:** `csv_output_dir: ""` varsayılanı + `local_map` ile aynı desen
(`Path.home() / "girdap_logs" / "telemetry"`), ya da doğrudan mutlak yol.

### 🟡 F4.2 — `control_rate_hz: 20.0` kendi MPPI ölçümüyle çelişiyor

`params.yaml:26-28`'in kendi yorumu:
> "CLAUDE.md 50 Hz MPPI HEDEFİ Jetson+CUDA içindir; CPU'da K=1000 rollout
> ~100 ms → 50 Hz gerçekçi değil"

Doğru tespit. Ama ardından `control_rate_hz: 20.0` yazılmış → **50 ms periyot**,
yani ölçülen 100 ms'nin hâlâ **iki katı hızlı**. CPU'da (CUDA portu henüz yok)
kontrol döngüsü her turda taşar; ROS zamanlayıcı geri kalır, `cmd_vel` gecikmeli
basılır. CPU için tutarlı değer ~`10.0` Hz olurdu.

Doğrulama Faz 11'de (`planning_node`'un MPPI'yi gerçekten bu timer içinde koşup
koşmadığı) tamamlanacak.

### 🟡 F4.3 — `topics:` bloğu tamamen ölü config

`hardware.yaml:81-86` bir `topics:` bloğu tanımlıyor (`livox`, `oak_rgb`,
`oak_depth`, `gps`, `imu`) ve yorumu "referans/remapping amaçlı" diyor.

`hardware.launch.py:_load_hardware_config()` (`:84-130`) yalnız şu anahtarları
okuyor: `fcu_url`, `gcs_url`, `mode_name`, `heartbeat_timeout_s`,
`arming_retry_max`, `algorithm`, `planning`, `mission`, `perception`.
**`topics` hiç okunmuyor.** Hiçbir remap üretilmiyor; node'lar topic adlarını
kodda sabit tutuyor (`perception_camera_node.py:96` → `"/oak/rgb/image_raw"`).

Yani bu blok değiştirilse hiçbir şey olmaz — config'in çalıştığı sanılan ama
çalışmayan bir parçası. (Ayrıca yorumunda şartnamede **var olmayan** "Dosya 1a/1b"
adlandırması geçiyor — bkz. Faz 17.)

### 🟡 F4.4 — RRT* örnekleme sınırları `[0, 200] × [0, 200]`

`params.yaml:29-30`. Eğer bu sınırlar araç-merkezli ya da odom çerçevesindeyse
(orijin = başlangıç pozu), araç **köşede** demektir ve RRT* yalnız `+x/+y`
çeyreğinde örnekler — başlangıcın gerisindeki ya da solundaki hiçbir hedefe
plan üretemez (video senaryosunda "başlangıca dönüş" var!). Çerçeve bağımlı;
kesin hüküm Faz 11'de `planning_node` okunduğunda verilecek. **Açık soru.**

### ⚪ F4.5 — HSV yorumlarındaki RAL kodları yanlış, ama **değerler tesadüfen doğru**

`params.yaml:76,78` yorumları "RAL 2008" (turuncu) ve "RAL 1003" (sarı) diyor;
şartname md 5.5.2.1 ise **RAL 2003** ve **RAL 1026**. Yorumlar yanlış.

Ancak değerler kontrol edildiğinde:

| Duba | Gerçek RAL | ≈Hex | OpenCV H | Config aralığı | Sonuç |
|---|---|---|---|---|---|
| Kenar (turuncu) | RAL 2003 | `#FA842B` | ≈13 | `hsv_orange` H 5–20 | ✅ ortada |
| Engel (sarı) | RAL 1026 | `#FFFF00` | ≈30 | `hsv_yellow` H 21–35 | ✅ ortada |

İlginç detay: yorumdaki **yanlış** RAL 1003 (`#F9A800`) H≈20'ye düşer — tam
turuncu/sarı sınırında (`orange_hi=20`, `yellow_lo=21`). Yani kod yanlış RAL'e
göre ayarlanmış olsaydı sınıflandırma sınırda kalırdı; **gerçek RAL 1026 daha
güvenli bir yerde**. Değerler sağlam, yalnız yorumlar düzeltilmeli.

### ✅ F4.6 — Doğrulanan: bbox piksel uzayı sözleşmesi **tutarlı**

`params.yaml:85-86` → `camera_image_width_px: 640`, `height: 480`.
Algı tarafı (`duba_gecis_navigator.py:154-155`) → `IMG_W, IMG_H = 640, 480`
ve yorumu: "fusion_node camera_image_width_px=640 varsayıyor — AYNI kal."

Sözleşme iki repo arasında **birebir uyumlu**. (Koddaki `requestOutput((640,400))`
yalnızca **mono/stereo** akışı içindir, `/perception/buoys` bbox uzayı değil —
bu ikisi karıştırılmamalı.)

### 📋 F4.7 — Operasyonel not (hata değil)

`hardware.yaml` şu an **video modunda**: `use_isam2: false`, `use_rrt: false`,
`mission_file: video_mission.yaml`. Otonomi videosu (son teslim **21.07.2026**)
için doğru. Yarışma günü üçünün de değişmesi gerekiyor — kontrol listesine.

---

## Faz 5 — Algı çekirdeği (Layer 0) ✅

Dosyalar: `prototype/perception/lidar_obstacles.py`, `camera_buoys.py`, `fusion.py`
(+ `perception_lidar_node.py`, `synthetic_lidar.py` çapraz doğrulama için)

### 🔴 F5.1 — LiDAR Z-filtresi **yanlış çerçevede** uygulanıyor → dubalar tamamen elenebilir

Denetimin en ciddi bulgusu. Üç dosyanın kesişiminde saklı.

**1. Filtrenin niyeti** (`lidar_obstacles.py:41`, `hardware.yaml:50`):
```python
z_min: float = 0.1   # "base_link'e göre su üstü kesim (m)"
z_max: float = 3.0
```
Yani `z`'nin **su yüzeyinden** ölçüldüğü varsayılıyor: 10 cm altı = su yansıması.

**2. Sentetik üretecin varsayımı** (`synthetic_lidar.py:39`, docstring `:9`):
```python
z = rng.uniform(0.0, spec.height)      # duba noktaları z ∈ [0, 0.5]
# "Koordinatlar base_link (araç merkezi) çerçevesinde"
```
Üreteç, orijini **su yüzeyine** koyuyor. Duba noktaları z ∈ [0, 0.5], su
gürültüsü (`:63`) z ∈ [−0.05, 0.08]. Filtre [0.1, 3.0] mükemmel çalışıyor:
gürültü elenir, dubanın %80'i kalır. **Testler yeşil.**

**3. Gerçekte olan** (`perception_lidar_node.py:89-96`):
```python
points = point_cloud2.read_points_numpy(msg, ("x","y","z"), ...)
obstacles = detect_obstacles(np.asarray(points, ...), self._cfg)   # ← DÖNÜŞÜM YOK
```
`/livox/lidar` noktaları **`livox_frame`** çerçevesinde gelir — orijin
**sensörün kendisi**. Hiçbir TF uygulanmıyor (Faz 3'te doğrulandı: yığında
`tf2_ros` / `lookup_transform` **sıfır kullanım**).

**Sonuç:** LiDAR su hattından `h` metre yukarı monteliyse, 50 cm'lik bir duba
sensör çerçevesinde `z ∈ [−h, −h+0.5]` aralığında görünür.

| Montaj yüksekliği `h` | Dubanın filtreden geçen kısmı |
|---|---|
| 0.0 m (su hattı — fiziksel olarak imkânsız) | %80 ✅ |
| 0.4 m | %20 |
| **> 0.4 m** (katamaran direk/güverte montajı) | **%0 — duba tamamen silinir** |

`/perception/obstacle_map` **boş yayınlanır**. `planning_node` engelleri buradan
alıyor (`:115-117`). **MPPI dubaların içinden geçer.** Parkur-2 biter.

**Neden 161 test bunu yakalamıyor:** sentetik üreteç ile gerçek sürücü **farklı
çerçevelerde** veri üretiyor, ve aradaki dönüşüm hiçbir yerde uygulanmıyor.
Testler yalnızca gerçek sistemin asla üretmediği bir çerçevede geçerli. Bu,
"yeşil test" güveninin tam olarak gizlediği hata sınıfı.

**Düzeltme (biri şart):**
- (a) Node, noktaları `livox_frame → base_link` static TF'i ile dönüştürsün
  (ve `base_link` su hattında tanımlansın); **veya**
- (b) `lidar_height_m` parametresi eklensin, filtre `z ∈ [z_min − h, z_max − h]`
  uygulasın. (a) doğru olan; (b) hızlı yama.

**Aksiyon:** mekanik ekipten Livox montaj yüksekliğini iste. `h` bilinmeden
Parkur-2 sahada koşturulmamalı.

### 🟠 F5.2 — `frame_id = "base_link"` sahte etiket

`perception_lidar_node.py:107`:
```python
out.header.frame_id = "base_link"     # yorum: "LiDAR → base_link static TF"
```
Yorum bir dönüşüm yapıldığını ima ediyor; **yapılmıyor**. Node, `livox_frame`
verisini alıp `base_link` diye **yeniden etiketliyor**.

`hardware.launch.py:237` gerçekten bir static TF yayınlıyor ama identity (0,0,0)
ve zaten **kimse okumuyor**. Yani mekanik ekip gerçek extrinsic'i ölçüp
launch'a girse bile **hiçbir şey değişmez** — kod TF'i hiç sormuyor. Sessiz.

> **✅ F5.3 DÜZELTİLDİ (Faz 18-T1, commit `a6aae64` — ortam kurulunca lokalde
> TDD ile):** Union-Find + `labels()` Python döngüleri →
> `scipy.sparse.csgraph.connected_components` + vektörize gruplama
> (argsort/split) + yeni `voxel_downsample` (config `voxel_size`; çekirdek
> varsayılan 0=kapalı → davranış birebir, üretim 0.1 m — node + params.yaml +
> hardware.yaml + hardware.launch kablolu). **Benchmark (x86, kıyı-duvarlı
> gerçekçi sahne): 20k nokta 524.5 ms → 53.6 ms (~10×); voxel'li tam detect
> 49.5 ms** — 10 Hz bütçesi x86'da rahat, Jetson gerçek ms'i D3'te ölçülecek.
> Doğrulama: O(n²) referans-eşdeğerlik teli (bileşenler birebir) + 3 voxel
> testi + üretim-config sahne testi (F6.2 dersi); suite 216 passed.
> ⚠ F5.4 etkileşimi: `point_count` artık voxel sayısı — duvar/iskele
> `max_cluster_size=500`'ü yine aşar. ~~"böl, atma" düzeltmesi (F5.4) AÇIK~~
> → ✅ F5.4 2026-07-11'de düzeltildi (`_split_oversized`, aşağıda).

### 🟠 F5.3 — Clustering, Jetson'da 10 Hz'i tutturamaz

`lidar_obstacles.py:112-114`:
```python
for i, j in tree.query_pairs(cfg.cluster_tolerance, output_type="ndarray"):
    uf.union(int(i), int(j))          # ← saf Python döngüsü, çift başına
```
ve `:95`:
```python
return np.array([self.find(i) for i in range(len(self._parent))])   # ← Python döngüsü
```

`output_type="ndarray"` yorumu "Python set yerine Kx2 array (hız)" diyor — ama
dizi yine **Python seviyesinde** dolaşılıyor, kazanç kayboluyor.

Livox Mid-360 ≈ 200k nokta/s → 10 Hz'de ~20k nokta/mesaj. Z+menzil filtresinden
sonra binlerce nokta kalır; `cluster_tolerance = 0.5` m yoğun bulutta
**10⁵–10⁶ komşu çifti** üretir. Saf Python `union` döngüsü bu ölçekte yüzlerce
milisaniye alır → 100 ms'lik LiDAR bütçesi aşılır, engel haritası geri kalır.

**Voxel downsample yok.** Eklenmesi gereken ilk şey o.

**Düzeltme:** `scipy.sparse.csgraph.connected_components` ile çiftleri seyrek
matris olarak ver (tamamı C tarafında), ya da `scipy.cluster.hierarchy`.
Öncesinde voxel grid downsample (5 cm) nokta sayısını 10×–50× düşürür.

### 🟡 F5.4 — `max_cluster_size = 500` en tehlikeli engeli **sessizce** düşürür

`lidar_obstacles.py:118`:
```python
if cfg.min_cluster_size <= len(member_idx) <= cfg.max_cluster_size:
    clusters.append(...)          # ← üst sınırı aşan cluster ATILIR
```
Yorum gerekçesi: "aşırı büyük = tekne kendisi / yanıltıcı".

Ama LiDAR'da **nokta sayısı mesafeyle ters orantılıdır**: bir engel ne kadar
yakın ve büyükse o kadar çok nokta döndürür. Yani bu filtre, **en yakın ve en
büyük engeli** — yani çarpma riski en yüksek olanı — sessizce siler ve MPPI
orayı boş alan sanar.

Parkur-3 hedef dubası 640 mm × 950 mm; yakın mesafede 500 noktayı rahat aşar.

**Düzeltme:** büyük cluster'ı atmak yerine **böl** (voxel/alt-clustering), ya da
en azından `WARN` bas. Sessiz düşürme kabul edilemez.

> ✅ **DÜZELTİLDİ (2026-07-11, F6.3 sahneleriyle aynı commit).**
> `cluster_points` artık üst sınırı aşan kümeyi atmıyor: `_split_oversized`
> kümeyi `split_cell_m=1.0` m'lik XY ızgara hücrelerine böler (kayıpsız;
> engel dairesi ≤ hücre yarı çaprazı ~0.71 m → duvar boyunca daire zinciri,
> tek dev daire serbest suyu kapatmaz). `min_cluster_size` alt-kümelere
> uygulanmaz — büyük katı cismin kenar hücresi gürültü değil. Yeni param
> `split_cell_m` node + hardware.yaml + params.yaml + launch'a kablolu.
> TDD: `scene_yakin_duvar` (voxel sonrası >500 hücre, ön koşul testli) önce
> KIRMIZI (duvar tamamen siliniyordu), düzeltmeyle YEŞİL; bölme kayıpsızlık
> + mekânsal sınır sözleşmesi ayrı testte. Performans x86: 20k noktalı duvar
> sahnesi ort. 34 ms (F5.3 bütçesi içinde, bölme maliyeti ihmal edilebilir).
> ⚠ NOT: max_cluster_size'ın eski "tekne kendisi" gerekçesi bölmeyle
> karşılanmıyor; bugün gövde dönüşleri sensör çerçevesinde z_min'e takılıyor
> (F5.1'in tersi yüzü). **F5.1 (lidar_height_m) düzeltmesi çerçeveyi
> kaydırdığında gövde görünür olursa `min_range` filtresi eklenmeli** —
> F5.1 paketinde değerlendirilecek.

### 🟡 F5.5 — HSV duba tespitinin etkin menzili ≈ **15 m**, LiDAR'ınki 25 m

`camera_buoys.py:98` → `if area < cfg.min_area_px: continue` (`min_area_px=150`).

30 cm çaplı, 50 cm yüksek duba; 640 px genişlik / 1.2 rad HFOV → ≈533 px/rad.
`d` metre mesafede: `w ≈ 160/d` px, `h ≈ 267/d` px, dolu alan ≈ `0.785·w·h`.
`area = 150` çözümü → **d ≈ 15 m**.

Yani 15 m'nin ötesindeki dubalar kameraya **görünmez**; LiDAR (25 m) görür.
Füzyonda eşleşme olmaz → `CLASS_UNKNOWN=99` (güvenli, engel olarak korunur) ama
**renk/sınıf bilgisi yok** → hangi çiftin "geçit" olduğu 15 m'den önce
bilinemez. Tasarım sınırı; sözleşmede yazılı değil, yazılmalı.

### 🟡 F5.6 — `score` bir güven skoru değil, **şekil** metriği — ve ters çalışıyor

`camera_buoys.py:108`:
```python
score=min(1.0, float(area) / float(w * h))     # kontur alanı / bbox alanı
```
Bu **doluluk oranı**. Dairesel bir nesne için ≈ **0.785**. Dikdörtgen bir nesne
(su üstündeki parlama şeridi, turuncu bir şamandıra halatı, tekne parçası) için
**1.0**.

Yani **gerçek yuvarlak duba, dikdörtgen bir yanlış-pozitiften daha DÜŞÜK skor
alır.** Skora göre sıralama/eşikleme yapan her downstream mantık tersine çalışır.

`/perception/buoys` sözleşmesi bu değeri `hypothesis.score` olarak taşıyor;
algı tarafı (bizim YOLO) buraya **gerçek bir güven skoru** koyuyor. İki repo
aynı alana **farklı anlamlar** yüklüyor.

**Düzeltme:** doluluk oranını dairesellik testi olarak **filtre** amaçlı kullan
(`0.6 < fill < 0.9` kabul), skoru ayrı üret.

### 🟡 F5.7 — `_infer_real` gerçek modelin sınıf sırasına kör bağımlı

`camera_buoys.py:170-172`:
```python
cx, cy, w, h = box.xywh[0].tolist()
detections.append(Detection(cx, cy, w, h, int(box.cls), float(box.conf)))
```
`int(box.cls)` YOLO'nun sınıf indeksini **doğrudan** `class_id` yapıyor.
Sözleşmede `2 = hedef` bekleniyor (`use_yolo` yalnız Parkur-3 hedefi için).

Ama algı tarafının eğittiği model **yalnız kenar/engel** sınıflarına sahip
(hedef sınıfı **yok**, `sartname-ida-2026` açık işi). O `.pt` verilirse
`box.cls ∈ {0,1}` döner → HSV'nin ürettiği 0/1 tespitlerinin **üstüne çift
tespit** basar, `class_id=2` hiç üretilmez.

**Düzeltme:** `_infer_real`'e açık bir sınıf eşleme tablosu koy
(`model_class → sözleşme class_id`), varsayılan olarak reddet.

### ✅ F5.8 — Düzeltme: mock YOLO **hiç çalışmıyor** (önceki alarm iki kez yanlıştı)

`camera_buoys.py:200`:
```python
if cfg.use_yolo and yolo is not None:
    detections += yolo.infer(frame_bgr)      # ← sabit merkez bbox, class_id=2
```
`use_yolo` hem `hardware.yaml:64` hem `params.yaml:72`'de **`false`**. Yani
mock'un sabit bbox'ı **hiçbir zaman** yayınlanmıyor.

Bu, "mock YOLO `/perception/buoys`'u kirletiyor" endişesinin **ikinci** çürütmesi
(birincisi F3.1: `/oak/rgb/image_raw` yayınlayan yok). Denetim, kendi erken
teşhisini iki kez düzeltti. F3.1'in **cihaz sahipliği** düzeltmesi yine de
geçerli — `depthai_ros` eklenirse HSV tespitleri (mock değil, gerçek) bizim
YOLO tespitlerimizle aynı topic'te çakışır.

### 🟡 F5.9 — Bearing işaret hatası (Faz 0'da bulunmuştu, burada bağlamına oturuyor)

`fusion.py:73` `atan2(y,x)` (sol +) ↔ `fusion.py:83` `(bbox_cx−0.5)·hfov` (sağ +).
Ters. `synthetic_fusion.py:27` `_cx_for_bearing` tam ters fonksiyon olduğu için
testler maskeliyor — **F5.1 ile aynı maskeleme deseni.**

Bugün etkisiz (çıktısı `classified_obstacles`'ın abonesi yok), ama F3.4
(extrinsic hiç uygulanmıyor) ile birlikte füzyon **iki bağımsız geometrik
hatayı** taşıyor. `classified_obstacles` planlayıcıya bağlanmadan önce ikisi de
düzeltilmeli.

---

## T0-a — MAVROS köprüsü + KILL zinciri ✅ (VİDEO + güvenlik)

Dosyalar: `mavros_bridge_node.py` (327), `prototype/control/mavros_bridge.py` (156),
`planning_node.py` (kontrol döngüsü + gate), `fsm_node.py` (KILL yayılımı)

**Doğrulanan KILL zinciri:** `mavros_bridge_node._trigger_kill()` →
`/girdap/mission/kill` → `fsm_node` FSM'i KILL durumuna alır →
`/girdap/mission/state = KILL` yayınlar → `planning_node.set_mission_state("KILL")`
→ `compute_control()` None döner (KILL, PARKUR1/2/3 değil) → `_publish_thrust(0)`.
Ek olarak `planning_node` kendi `control_gate`'inden `gate.zero_thrust` ile de
sıfırlıyor. **Zincir, `planning_node` canlı ve companion↔FCU hattı açıkken
çalışır.** Aşağıdaki bulgular bu iki koşulun kırıldığı yerler.

### 🔴 F14.1 — KILL, FCU'yu **disarm ETMİYOR**; en kritik failsafe'te motor sinyali FCU'ya ulaşamaz

`_trigger_kill()` (`mavros_bridge_node.py:281-290`) yalnız `/girdap/mission/kill`
çağırıyor — yani motoru durdurma yolu tamamen **companion tarafı sıfır-thrust**.
Köprünün elinde `_cli_arm` (`:103`, disarm yeteneği) **var ama KILL'de
kullanılmıyor.**

İki kopan senaryo:

**(a) Heartbeat-kaybı KILL'i — kendi kendini iptal ediyor.** KILL'in **bir
numaralı tetikleyicisi** heartbeat kaybı (`_on_monitor:152`, `control_gate:137`):
`/mavros/state` gelmiyor demek. Ama `/mavros/state`'i taşıyan MAVLink hattı
(companion↔FCU USB) koptuysa, aynı hat üzerinden gidecek **sıfır-thrust komutu
da FCU'ya ulaşamaz.** En çok fiziksel durdurma gereken failsafe, yazılım yolu
kopmuş olan failsafe. Doğru eylem: FCU'yu **disarm** etmek (ya da FCU'nun kendi
GCS-failsafe'ine güvenmek), companion-side sıfır-thrust değil.

**(b) KILL sonrası araç ARMED kalıyor.** Disarm çağrılmadığı için tekne canlı;
son bir `cmd_vel` ya da başka bir setpoint kaynağı onu hâlâ hareket ettirebilir.

**Düzeltme:** `_trigger_kill()` önce `/mavros/cmd/arming value=False` çağırsın
(companion↔FCU hattı canlıyken kesin durdurma), sonra FSM KILL'i yaysın.
Heartbeat/bağlantı kaybında FCU'nun kendi failsafe'i (ArduPilot GCS/throttle
failsafe → otomatik disarm/hold) devrede olmalı — bu FC parametresi, mekanik/FC
ekibiyle teyit edilmeli.

### 🔴 F14.2 (VİDEO) — Görev sonu **kasıtlı disarm'ı** FAILSAFE KILL sanıyor

Video akışı (md 3.3.1/3-4): 4. waypoint'ten sonra araç manuel döner, **sonra
operatör gücü/motoru güvenlik anahtarıyla keser** (bilerek disarm).

`_on_monitor` (`:160-166`):
```python
if self._was_armed and not armed:
    self.get_logger().error("FAILSAFE — beklenmedik disarm → KILL")
    self._trigger_kill()
```
Operatör `/girdap/bridge/disarm` ile ya da fiziksel anahtarla disarm ettiğinde
FCU `armed=False` yayınlar → köprü bunu **"beklenmedik disarm"** sanar, tam da
kameraya çekilen kasıtlı güç-kesme anında ekrana **"FAILSAFE — beklenmedik
disarm → KILL"** basar. `_request_disarm` (`:260`) hiçbir "beklenen disarm"
bayrağı set etmiyor — komutlu ve beklenmedik disarm **ayırt edilmiyor**.

Videoda bu, kasıtlı bir eylemi bir arıza gibi gösterir (QGC/telemetride hata
durumu). Fonksiyonel olarak da `killed` latch'lenir.

**Düzeltme:** `_request_disarm` bir `_expected_disarm=True` bayrağı set etsin;
`_on_monitor` bu bayrak varken disarm'ı failsafe saymasın (bir sonraki arm'da
temizle).

### 🟠 F14.3 (VİDEO) — `auto_guided` manuel dönüşte operatörle **kavga ediyor**

md 3.3.1/3: "Bu aşamadan sonra İDA başlangıç noktasına **manuel** olarak
döndürülebilir." Operatör RC'yi MANUAL/HOLD'a alır.

`_on_state` (`:135-140`): `auto_guided and needs_mode_change()` → her
`/mavros/state` GUIDED-dışı gördüğünde `set_mode GUIDED` gönderir. Yani manuel
dönüş boyunca companion, aracı **sürekli GUIDED'e geri çekmeye** çalışır;
operatörün manuel moduyla çatışır. Firmware'e göre RC mod-switch GCS set_mode'u
ezebilir ama bu **garantisiz** ve videoda mod titremesi/çatışma riski.

**Düzeltme:** `auto_guided`'i görev-aktif durumuna bağla (FSM PARKUR* dışında ya
da görev COMPLETE sonrası set_mode isteğini kes), ya da bir manuel mod bilinçli
seçilince devre dışı bırak.

**✅ DÜZELTİLDİ (Faz 18):** birinci seçenek uygulandı — auto-GUIDED **yalnız
görev aktifken** (FSM `PARKUR1/2/3`). Çekirdek: `MavrosBridge.set_mission_state()`
+ `MISSION_ACTIVE_STATES`; `needs_mode_change()` görev-aktif değilse False
(varsayılan False → FSM ölürse/`/girdap/mission/state` hiç gelmezse mod asla
zorlanmaz; görev FSM'siz koşamayacağı için güvenli taraf). Node
`/girdap/mission/state` dinler; PARKUR1'e girişte `/mavros/state`'i (~1 Hz)
beklemeden hemen dener (`_maybe_auto_guided`). Görev öncesi RC konumlama da
serbestleşti (önceden boot'tan itibaren zorluyordu). 3 yeni çekirdek testi
(TDD kırmızı→yeşil; mevcut 3 test doğru davranışa güncellendi), node
py_compile. Video akışı: start→GUIDED, TAMAMLANDI→manuel dönüş kavgasız,
güvenlik anahtarı→F14.2 beklenen disarm.

### 🟠 F14.4 — Heartbeat KILL'i **kalıcı**, geçici kopma = kurtarılamaz görev kaybı

`_trigger_kill` `self._killed = True` latch'liyor; `_on_monitor:146` bu bayrakta
erken dönüyor. **Kurtarma yok.** Geçici bir USB/mavros hıçkırığı (`/mavros/state`
5 sn boşluğu — CDC reset, mavros restart) → **kalıcı KILL**, süreç yeniden
başlatılmadan dönülemez. 2-5 dk'lık video ya da 20 dk'lık yarışma koşusunda tek
bir kısa boşluk her şeyi bitirir. `/mavros/state` ~1 Hz, eşik 5 sn (5 kaçan
mesaj) makul ama histerezissiz ve tek yönlü.

**Düzeltme:** heartbeat geri geldiğinde + araç güvenliyken (disarmed) KILL'den
çıkışa izin ver, ya da yalnız N ardışık kayıpta latch'le.

### 🟡 F14.5 — Üç ayrı `MavrosBridge` örneği, parçalı durdurma otoritesi

`MavrosBridge` üç yerde ayrı ayrı kuruluyor, her biri **kendi** `/mavros/state`
aboneliğiyle, bağımsız zamanlamayla:
1. `mavros_bridge_node` (`:82`) — izleme + KILL tetiği.
2. `planning_node` (`:96`) — `control_gate` ile adım-başı sıfır-thrust.
3. (çekirdek `planning_node`'da tekrar kullanılıyor — DRY notu.)

Heartbeat kaybında ikisi **farklı anlarda** ateşleyebilir/çelişebilir. Asıl
motor-durdurma `planning_node`'un canlı ve 20 Hz dönüyor olmasına bağlı — tek
gerçek yürütücü o.

### 🟡 F14.6 — `planning_node` ölürse motoru durduran **hiçbir şey yok**

Tüm yazılım motor-durdurma `planning_node`'un 20 Hz döngüsünün sıfır-thrust
yayınlamaya **devam etmesine** bağlı (`_on_control_step:228`). MPPI'de yakalanmamış
bir istisna `planning_node`'u düşürürse, düğüm yayın yapmayı keser; ESC'deki son
komut Layer-1 zaman aşımına kadar sürebilir. `planning_node` ölümünde disarm
eden bağımsız bir watchdog yok. (F14.1'in FCU-disarm gerekçesi de bu.)

### ✅ F14.7 — Doğru tasarlanmış yanlar (yeniden yazma, dokunma)

- Pre-arm reddinde (EKF/GPS fix) retry + **KILL tetiklememe** doğru karar
  (araç zaten disarm/hareketsiz).
- Node kendiliğinden **arm etmiyor** — arm bilinçli operatör eylemi.
- `control_gate` öncelik sırası (heartbeat > bağlantı > arm > mod) sağlam.
- Mock modda servis yokken `service_is_ready()` ile çökmeme.

**Not:** F14.1-14.4'ün hiçbiri simülasyonda görülmedi — mock `armed=True, GUIDED`
yayınladığından ne set_mode ne KILL ne disarm-failsafe yolu **hiç koşmadı**
(`mavros_bridge_node.py:42-44` docstring bunu kabul ediyor). Bu yollar ilk kez
gerçek FCU ile çalışacak.

## T0-b — Görev yönetimi ✅ (VİDEO 4 nokta)

Dosyalar: `prototype/mission/mission_manager.py` (147),
`mission_manager_node.py` (203), `planning_node._on_target`, `fsm_node` start yolu

### 🟠 F13.1 (VİDEO) — Görev sonu **temiz duruş yok**; son waypoint'te istasyon-tutma titremesi riski

Video akışı (md 3.3.1/3): 4. waypoint'e varınca **otonom görev biter**, sonra
manuel dönüş. Kodda "biter" karşılığı net değil:

- `MissionManager` 4. waypoint dwell'i dolunca `COMPLETE` olur, `update()` None
  döner (`mission_manager.py:114-116`), node `current_target` yayınını **keser**.
- Ama `planning_node._on_target` en son hedefi `set_reference_direct` ile tutuyor
  (`planning_node.py:195-197`); yeni hedef gelmese de MPPI **son hedefe sürmeye
  devam eder**. Araç zaten oradadır → MPPI **istasyon-tutma** yapar (sıfır değil,
  küçük düzeltmeler).
- `MissionFSM` ise 4. waypoint'e varınca `dist_to_last_wp_p1 < 1.5` görüp
  `PARKUR1 → PARKUR2`'ye geçebilir — ama video görevinde **parkur-2 waypoint'i
  yok** (hepsi parkur=1). Bu durumda planning hâlâ "aktif sürüş" durumunda.

⚠ md 3.3.1.1: "hareket eksenlerinde **istemsiz** ... dönüş/sürüş gözlemlenirse
video **BAŞARISIZ**." İstasyon-tutma titremesi tam da bu riski doğurur. Videoda
4. noktada aracın **temiz durması** (thrust sıfır / hold / disarm) gerekiyor;
şu an bunu yapan bir yol yok. **Tam çözüm Faz 12 (FSM) ile birlikte** — orada
`COMPLETE → TAMAMLANDI → planning None → sıfır thrust` zinciri kurulmalı.

### 🟠 F13.2 (VİDEO) — Görev başlatma **çok adımlı operatör dizisi**, tek komut değil

md 3.3.1/3 "YKİ/RC'den **bir komutla** görev başlayacak" diyor. Kodda başlatma
zinciri:
1. Operatör `/girdap/bridge/arm` → FCU arm.
2. FSM `BOOT → ARM → BEKLEMEDE` (`fsm_node:286` — armed + kill switch OFF).
3. Operatör `/girdap/mission/start` → FSM (`_on_start_srv:263` **yalnız
   BEKLEMEDE'de** kabul) → `PARKUR1`.
4. `mission_manager._on_state` PARKUR1 görür → `MissionManager.start()`.

Yani "bir komut" aslında **iki servis çağrısı + iki FSM geçişi**. Hata değil ama
video çekiminde bu dizi **prova edilip scriptlenmeli** (bir QGC MAVLink komutu ya
da `ros2 service call` sırası). Adım atlanırsa görev başlamaz ve sebebi loglarda
görünür (`start sadece BEKLEMEDE'de geçerli`).

### 🟡 F13.3 — `current_target` frame_id="base_link" **yanlış etiket** (ama canlı bug DEĞİL)

`mission_manager_node.py:185` hedefi `frame_id = "base_link"` ile yayınlıyor,
`position.{x,y} = (east, north)`. Ama east/north **dünya-ENU** ofseti (mutlak
yön), base_link ise **gövde** çerçevesi (x ileri, y sol). İkisi yalnız araç tam
doğuya bakarken çakışır.

**İlk şüphem "araç yanlış yöne sürer" idi — YANLIŞ.** `planning_node._on_target`
(`:195-197`) ofseti **dünya odom pozisyonuna ekliyor**:
`hedef_dünya = _last_xy + (east, north)`. `_last_xy` ENU odom pozisyonu olduğundan
sonuç doğru dünya hedefidir. Yani **tüketici tutarlı**, davranış doğru.

Kalan risk 🟡: etiket yanlış olduğu için başka bir tüketici (RViz, ikinci node)
`base_link`'i harfi harfine okursa yanılır; ayrıca görev hedefinin **GPS**'ten,
planning ofsetinin **fusion-odom**'dan gelmesini gizliyor (F13.4). Etiket
`odom`/dünya-hizalı bir çerçeve olmalı.

### 🟡 F13.4 — Hedef GPS'ten, taban odom'dan: **iki poz kaynağı**

Görev ofseti `mission_manager`'da **GPS lat/lon**'dan (`_on_gps`), planning'de
**fusion-odom** `_last_xy`'ye ekleniyor. İki farklı kaynak, iki farklı an.
Takip hedefi olarak çalışır (her tick yeniden hesaplanıyor) ama odom GPS'e göre
sürüklenirse hedef kayar. Video ölçeğinde (dakikalar, ~50 m) tolere edilebilir;
yarışmada (20 dk) izlenmeli.

### 🟡 F13.5 — `_started` latch'i → görev **yeniden başlatılamaz**

`mission_manager_node.py:146` `_started=True` kalıcı; FSM BEKLEMEDE'ye dönüp
tekrar PARKUR1'e geçse bile `MissionManager.start()` bir daha çağrılmaz.
Şartname 5.5.1 "yeniden başlama hakkı 1 kere" veriyor — yarışmada restart
görev yöneticisini yeniden kurmaz. Video tek atış olduğu için orada zararsız.

### 🟡 F13.6 — GPS bayatlık kontrolü yok

`_on_gps` (`:139-142`) yalnız `status >= 0` bakıyor (NO_FIX'i eler, iyi) ama
zaman aşımı yok. GPS düşerse `_lat/_lon` son değerde kalır, görev **bayat
pozisyondan** ofset hesaplar → araç yanlış yöne sürebilir. Bir `fix_timeout`
eklenmeli.

### ✅ F13.7 — Doğru: çekirdek durum makinesi temiz

`MissionManager` arrival/dwell/index mantığı (`mission_manager.py:92-123`)
sağlam: varışta DWELL, dwell dolunca index++, son waypoint'te COMPLETE, tek-atış
`waypoint_reached` DWELL girişinde. Bu çekirdek dokunulmamalı.

## Faz 12 (T0) — FSM çekirdeği + görev otomatı ✅

Dosyalar: `prototype/fsm/mission_fsm.py` (446), `fsm_node.py` (324),
`prototype/mission/parkur_fsm.py` (parkur katmanı)

Durumlar: `BOOT → ARM → BEKLEMEDE → PARKUR1 → PARKUR2 → PARKUR3 → TAMAMLANDI`,
her durumdan `KILL`. `/girdap/mission/state` = **MissionFSM.tick()** çıktısı
(`fsm_node:289,300`) — planning bunu MPPI ağırlık profili + sürüş izni için okur
(`pipeline.py:41` `_ACTIVE_STATES = PARKUR1/2/3`).

### 🔴 F12.1 — `dist_to_last_wp_p1` **odom orijinine** ölçülüyor, gerçek son waypoint'e değil

`fsm_node._on_odom:207-211`:
```python
last_wp = self.get_parameter("last_waypoint_xy").value   # varsayılan [0.0, 0.0]
dx = self._pose_xy[0] - last_wp[0]
self._obs.dist_to_last_wp_p1 = math.hypot(dx, dy)
```
`last_waypoint_xy` `params.yaml:42`'de `[0.0, 0.0]`, yorumu **"planning_node
tarafından override"**. Ama grep (`--include=*.py`, tüm ağaç): bu parametreyi
çalışma anında **yazan tek satır yok**. Kalıcı `[0,0]`. Yorum yalan.

Sonuç: `dist_to_last_wp_p1` = **aracın odom orijinine (0,0) uzaklığı**. PARKUR1→
PARKUR2 geçişi (`mission_fsm.py:215-217`, eşik 1.5 m) gerçek son waypoint'le
**alakasız** bir geometriyle tetikleniyor.

En kötüsü: video modunda `use_isam2=false` → odom, MAVROS yerel pozisyonunu
geçiriyor, o da EKF orijininde (0,0) başlıyor. Araç açılışta orijinin yanında →
`dist ≈ 0 ≤ 1.5` → **görev başlar başlamaz FSM PARKUR1'den PARKUR2'ye atlıyor.**

### 🔴 F12.2 (VİDEO) — Video senaryosunun **terminal durumu yok** (F13.1 kök nedeni)

Video görevi tek parkur (tüm waypoint `parkur=1`) ve kamikaze **yok**. FSM'in
tek TAMAMLANDI yolu (`mission_fsm.py:226`): `PARKUR3 + shock_detected_p3` (IMU
çarpması). Video hiç PARKUR3'e ulaşmaz, çarpma da olmaz → **video HİÇBİR ZAMAN
TAMAMLANDI'ya varamaz.**

Akış: BEKLEMEDE→PARKUR1→(F12.1 yüzünden hemen)→PARKUR2→**sonsuza kadar takılı**
(PARKUR2→PARKUR3 için duba geçişi lazım, videoda duba yok). Tüm bu süre boyunca
`mission_state ∈ PARKUR*` → `compute_control` aktif → MPPI son hedefe (bayat
`current_target`) sürmeye devam → **istasyon-tutma titremesi.**

⚠ md 3.3.1.1: "istemsiz ... dönüş/sürüş → video **BAŞARISIZ**." Bu, geçen
fazdaki F13.1'in kök nedeni. **Düzeltme:** `mission_manager` COMPLETE'i (ya da
son `waypoint_reached`) FSM'e bir sinyalle taşınmalı; FSM bunu parkur/kamikaze
yolundan **bağımsız** olarak `TAMAMLANDI`'ya geçiş kuralı yapmalı. TAMAMLANDI'da
`compute_control` None → sıfır thrust = temiz duruş (zincir zaten var, eksik olan
o duruma **varış**).

### 🟠 F12.3 — İki paralel FSM çelişiyor; **yanlış olanı** otoriter

Kod iki ayrı "parkur" kavramı taşıyor:
- **MissionFSM** — mesafe-tabanlı (F12.1'deki buggy geçiş), `/girdap/mission/state`'i
  **bu** yayınlıyor → planning bunu dinliyor. **Otoriter.**
- **ParkurTransitionLogic** (`parkur_fsm.py`) — waypoint-index tabanlı, "doğru"
  tasarım. Ama yalnız **log** basıyor (`fsm_node._emit_parkur_transition:238`),
  hiçbir davranışı yönetmiyor. Video'da (boş etiket) PARKUR_1'de kalıyor.

Yani sağlam olan katman dekoratif, buggy olan otoriter. `CLAUDE.md`'nin "parkur
katmanı MissionFSM'in üstüne oturur" anlatısı gerçekte tersine dönmüş.

### 🟡 F12.4 (P3) — Kamikaze şok bayrağı **erken latch'leniyor**

`_on_imu:213-220` `shock_detected_p3 = True`'yu **her durumda** set ediyor
(|a| > 5g), yalnız PARKUR3→TAMAMLANDI'da temizleniyor (`fsm_node:294`). Deniz
Durumu-2'de PARKUR1/2 sırasında bir dalga/sarsıntı 5g'yi geçerse bayrak
latch'lenir; FSM PARKUR3'e girer girmez **anında** TAMAMLANDI'ya atlar (kamikaze
hedefe varmadan). Şok algılama yalnız PARKUR3'te aktif olmalı.

### ✅ F12.5 — Doğru: çekirdek yapı sağlam

`mission_fsm.py` çekirdeği (geçiş tablosu, KILL önceliği, on_enter/exit/tick,
history) temiz ve test edilebilir. **Bug'lar çekirdekte değil, onu BESLEYEN
`fsm_node` gözlemlerinde** (F12.1 yanlış mesafe, F12.2 eksik terminal geçiş,
F12.4 erken şok). Çekirdeğe dokunma; düzeltmeler node'un gözlem üretiminde.

## T0-c — planning_node + pipeline ✅ (VİDEO bypass yolu)

Dosyalar: `planning_node.py` (295), `prototype/planning/pipeline.py` (335)

Video bypass zinciri **çalışıyor**: `_on_target` (`:187`) → `set_reference_direct`
(`pipeline.py:137`) `_ref_path=[poz→hedef]` kurar + `_rebuild_mppi` → MPPI hazır →
`compute_control` (`:235`) `_mppi.step` döndürür. Ama nasıl kurulduğu sorunlu.

### 🔴 F11.1 (VİDEO) — MPPI **her hedef güncellemesinde sıfırdan yaratılıyor** → warm-start kaybı → zikzak

`_on_target` (5 Hz, `mission_manager` yayını) → `set_reference_direct` →
`_rebuild_mppi` (`pipeline.py:224-231`):
```python
self._mppi = MPPIController(self._dyn, self._bounds, self._obstacles, ...)
self._mppi.set_reference(...)
```
Her çağrıda **yeni bir `MPPIController` nesnesi**. MPPI'nin kararlılığı önceki
adımın kontrol dizisini **warm-start** etmesine dayanır. `CLAUDE.md`'nin kendi
uyarısı: *"MPPI ilk iterasyon kararsız. Warm-start yoksa rastgele kontrol → araç
zikzak. Önceki kontrol dizisini kaydır."*

Nesne 5 Hz'te (200 ms'de bir) yeniden yaratıldığından warm-start **her 200 ms'de
sıfırlanıyor** → her hedef tazelemesinde MPPI soğuk başlıyor → titreme/zikzak.

⚠ Doğrudan md 3.3.1.1: "istemsiz ... dönüş/sürüş → video **BAŞARISIZ**." Bu, F12.2
ve F13.1 ile birlikte videonun **üçüncü** istemsiz-hareket kaynağı. (Warm-start'ın
gerçekten `MPPIController.__init__`'te sıfırlandığı T0-d'de teyit edilecek — ama
nesne yeniden yaratıldığı için per-controller durum kaçınılmaz kayboluyor.)

**Düzeltme:** referans/engel değişince MPPI'yi **yeniden yaratma**, mevcut
kontrolcünün `set_reference`/`set_obstacles`'ını güncelle (warm-start korunur).

### 🟠 F11.2 — Engel güncellemesi de MPPI'yi yeniden yaratıyor (F11.1'i katlıyor)

`set_obstacles` (`pipeline.py:154-160`): replan gerekmezse `else` dalı
`_rebuild_mppi()` çağırıyor. `/perception/obstacle_map` yayınlanıyorsa (~10 Hz)
MPPI **ayrıca** 10 Hz'te yeniden yaratılıyor. F11.1 ile birlikte warm-start
neredeyse hiç yaşamıyor.

### ✅ F4.4 çözüldü (VİDEO) — `bounds [0,200]` videoda kullanılmıyor

`bounds_x/y=[0,200]` (`params.yaml:29-30`) yalnız RRT*'ta (`_global_replan:195`)
kullanılıyor. Video modu `use_rrt=false` → `_on_waypoints` erken dönüyor
(`planning_node.py:176`), `_on_target` bypass yolu bounds'a **hiç dokunmuyor**.
Yani **video için sorun değil**.

🟡 **T1 (yarışma) açık soru kalıyor:** odom ENU (0,0)'da başlarsa ve arena
[0,200] haritalıysa, araç orijinin güney/batısına (negatif) geçtiğinde RRT*
örnekleme uzayı dışında kalır. Faz 10 (RRT*) + saha koordinat çerçevesiyle
kesinleşecek.

### 🟡 F4.2 ilerledi — `control_rate=20 Hz` timer, MPPI step'i **senkron** çağırıyor

`_on_control_step` (`:216-233`) 20 Hz timer'da `compute_control` → `_mppi.step`'i
**senkron** çalıştırıyor. Step ~100 ms (CPU, `params.yaml:26` kendi notu) sürerse
tek-thread executor 20 Hz'i (50 ms) tutamaz; callback'ler birikir, `cmd_vel`
gecikir/titrer. Kesin step maliyeti **T0-d (MPPI)**'de ölçülecek; oradan sonra
`control_rate` gerçekçi değere (~10 Hz) çekilmeli.

### 🟡 F11.3 — `_on_obstacles` PLACEHOLDER şemaya kilitli (F5.1/F5.4 ile zincir)

`_on_obstacles` (`:167-173`) `/perception/obstacle_map`'i `position.{x,y}`=merkez,
`orientation.z`=yarıçap hack'iyle okuyor — algı tarafıyla uyumlu. Ama bu engeller
F5.1 (LiDAR z-filtresi yanlış çerçeve → engel listesi boş olabilir) ve F5.4
(500+ nokta engeli sessiz düşürme) bug'larından besleniyor. Planning tarafında
hata yok; **girdi** zehirli. Video'da LiDAR şart değil (GPS dikdörtgen), yarışmada
kritik.

### ✅ F11.4 — Doğru: MAVROS geçidi + parkur profili ayrımı temiz

`compute_control` parkur-dışı None (motor stop), `control_gate` ile thrust
sıfırlama/cmd_vel susturma (T0-a'da doğrulandı), `_PARKUR_PROFILES` ağırlık
ayrımı (P1 takip / P2 engel agresif / P3 kamikaze) mantıklı. Bunlar sağlam.

## T0-d — MPPI ✅ (VİDEO geçme kriteri: md 3.3.1.1 istemsiz hareket)

Dosya: `prototype/planning/mppi.py` (569)

### 🔴 F9.1 — F11.1 KANITLANDI: warm-start `__init__`'te sıfırlanıyor, pipeline 5-10 Hz yeniden yaratıyor

MPPI'nin warm-start makinesi **doğru ve çalışıyor**: `_apply_warmstart` (`:356-362`)
`U_nominal`'i bir adım kaydırıp sonuna 0 koyuyor (`warm_start_enabled=True`). Bir
kontrolcünün **ömrü boyunca** warm-start birikiyor.

AMA `__init__:143`: `self.U_nominal = np.zeros((T, 2))`. Her yeni `MPPIController`
warm-start'ı **sıfırdan** başlatıyor. T0-c'de görüldü: pipeline `_rebuild_mppi`
ile bu nesneyi hedef güncellemesinde (5 Hz) **ve** engel güncellemesinde (~10 Hz)
yeniden yaratıyor → `U_nominal` sürekli sıfırlanıyor → warm-start **hiç birikmiyor**
→ MPPI her 100-200 ms'de soğuk başlıyor.

Cold-start MPPI'nin ilk iterasyonları rastgele kontrol üretir (`CLAUDE.md`'nin
kendi uyarısı: "warm-start yoksa araç zikzak"). **F11.1 artık "olası" değil,
kanıtlı** — kanıt `__init__`'in `U_nominal`'ı sıfırlaması. Videonun istemsiz-hareket
riskinin (md 3.3.1.1) **üç kaynağından biri** (F12.2, F13.1 ile birlikte).

**Düzeltme MPPI'de DEĞİL, pipeline'da:** kontrolcüyü yeniden yaratma; mevcut
kontrolcünün `set_reference`/engel dizisini güncelle, `U_nominal` yaşasın. (MPPI
zaten `set_reference`'ı ayrı metod olarak sunuyor — pipeline yanlış kullanıyor.)

### 🟡 F4.2 çözüldü — `control_rate=20 Hz` CPU'da imkânsız, ~10 Hz'e çekilmeli

`step` (`:315`) senkron: K=1000 rollout × T=50 adım RK4 + `_trajectory_cost`'ta
track maliyeti `(K, T+1, n_ref)` argmin — referans uzunluğuyla büyüyen baskın
terim. Yazarın ölçümü ~100 ms/iter CPU (`params.yaml:26`, `CLAUDE.md`). 20 Hz
(50 ms) tek-thread executor'da **tutulamaz**: callback'ler birikir, `cmd_vel`
gecikir/titrer — bu da istemsiz harekete katkı.

**Düzeltme:** CUDA portu yapılana kadar `control_rate_hz ≈ 10`. Jetson'da gerçek
step süresi **ölçülmeli** (henüz ölçülmedi — `bekleyen_girdiler.md` D3).

### 🟡 F9.2 — MPPI aracı **nokta** sayıyor; tekne genişliği yalnız `obstacle_margin`'de

Maliyet fonksiyonu araç pozunu tek `(xs, ys)` noktası alıyor; 0.75 m tekne
genişliği hesaba **girmiyor**. Tek pay `obstacle_margin=0.5 m` (`:88`,
`_obs_r = o.r + margin`). 0.15 m'lik dubada emniyet yarıçapı 0.65 m, tekne yarı
genişliği 0.375 m → emniyet sınırında **0.275 m** açıklık. Deniz Durumu-2
sürüklenmesiyle dar. `obstacle_margin ≥ yarı_genişlik + sürüklenme` olmalı.
(Video'da engel yok → yalnız yarışma/T1 riski, ama şimdi not edildi.)

### ✅ F9.3 — MPPI çekirdeği sağlam (Williams 2017 sadık uygulama, dokunma)

- **Softmax sayısal kararlı:** `S_min` çıkarımı (`:330-331`), `w_sum` sonlu/≥1e-12
  değilse nominal'i sürdürme guard'ı (`:333-339`) — taşma/NaN korumalı.
- dt=0.05 × T=50 = 2.5 s horizon tutarlı; K=1000.
- Heading `_wrap_angle` (atan2 sin/cos) ile sarılı — π sıçraması maliyeti bozmuyor.
- Engel quadratic barrier `max(0, r_safe−d)²` doğru; kamikaze Gaussian çekici
  sınırlı katkı (P3).
- Kontrol kırpma sonrası `eps_eff = V − U_nominal` ile etkin gürültü doğru.

**Sonuç:** MPPI kodu iyi. Videodaki zikzak riski MPPI'nin matematiğinden değil,
pipeline'ın kontrolcüyü sürekli yok etmesinden (F9.1) ve 20 Hz hedefinden (F4.2)
geliyor. İkisi de **pipeline/config düzeltmesi**, MPPI'ye dokunulmaz.

## T0-i — Telemetri (Ekran-2 + Dosya-2) ✅

Dosyalar: `prototype/telemetry/csv_logger.py` (173), `telemetry_node.py` (186)

### 🔴 F15.1 (F4.1 kesinleşti + yer belli) — Dosya-2 göreli yola; systemd'de **başlangıçta çöker**

`telemetry_node.py:62,68`: `csv_output_dir="data/telemetry"` (göreli) →
`TelemetryCsvLogger(out_dir)` → `csv_logger.py:139-140`
`Path("data/telemetry").expanduser().mkdir(parents=True)`. `expanduser` göreli
yolu **çözmez**.

- `ros2 launch` ile (video için olası): cwd = terminal dizini → dosya **rastgele
  yere** düşer, 20 dk içinde bulunması gereken Dosya-2. 🟠
- `systemd` servisi ile (`WorkingDirectory` yoksa cwd=`/`): `mkdir("/data/telemetry")`
  → **PermissionError**. Logger `__init__`'te patlar, `TelemetryNode.__init__`
  çöker, `main()` düşer → **telemetri node'u hiç açılmaz → Dosya-2 ÜRETİLMEZ →
  5 ceza puanı.** 🔴

`local_map.py:66` aynı işi `Path.home()/girdap_logs/...` ile **doğru** yapıyor;
telemetri yapmıyor. **Düzeltme:** `csv_output_dir` varsayılanı `""` + node'da
`Path.home()/girdap_logs/telemetry` fallback (Dosya-3 deseniyle aynı).

### 🟠 F15.2 (VİDEO Ekran-2c = T0-g) — thrust telemetriye girmiyor

`CSV_HEADER` (`csv_logger.py:28-39`): `hiz, hiz_setpoint, heading, yon_setpoint`
var — Ekran-2'nin (a) ve (b) sinyalleri ✅. Ama **thrust/kuvvet kolonu YOK**.
md 3.3.1.1 Ekran-2'nin üçüncü sinyali "**thrusterlardan kuvvet isteği**" eksik.

Sinyal **mevcut**: `planning_node` `/girdap/control/thrust`
(`Float32MultiArray [T_left,T_right]`) yayınlıyor ama `telemetry_node` buna
**abone değil**.

**Önemli ayrım:**
- **Dosya-2 (md 4.2):** thrust İSTEMİYOR. CSV_HEADER 4.2 ile birebir uyumlu —
  **değiştirme** (docstring de sabit tutulmasını istiyor).
- **Ekran-2 (video, md 3.3.1.1):** thrust İSTİYOR. Bu **ayrı** bir çıktı.

**Düzeltme (T0-g):** ya video için ayrı bir grafik-CSV'si (`/girdap/control/thrust`
+ hız/heading senkron), ya da video çekiminde `rosbag record` → sonradan çiz.
Dosya-2 CSV'sine dokunma.

### 🟡 F15.3 — heading kaynağı latch'li: IMU düşerse **donar**

`_heading_from_imu` (`:130`) bir kez IMU gelince kalıcı True; sonra `_on_odom`
(`:142`) heading'i güncellemez. IMU görev ortasında düşerse heading **son IMU
değerinde donar** (odom fallback latch yüzünden devre dışı). Değerlendirilen
telemetride donmuş heading yanıltıcı. Bayatlık kontrolü + odom'a geri düşüş gerek.

### 🟡 F15.4 (VİDEO) — hız kaynağının odom yedeği yok

`hiz` yalnız `/mavros/local_position/velocity_body`'den (`_on_vel_body:132`). Bu
topic yoksa (mavros local_position plugin kapalıysa) `hiz=None` → CSV'de boş →
**Ekran-2 "gerçek hız" grafiği boş → video zayıf/başarısız**. heading'in odom
yedeği var ama hız'ın **yok**; oysa `_on_odom` `twist.twist.linear`'ı görüyor,
oradan hız yedeklenebilirdi. Eklenmeli.

### 🟡 F15.5 — GPS/veri bayatlık kontrolü yok

GPS düşerse `_lat/_lon` son değerde kalır, her tick yazılır → CSV donmuş pozisyonu
geçerliymiş gibi gösterir (mission_manager F13.6 ile aynı desen).

### ✅ F15.6 — Doğru: dikkatli yazılmış

Satır-başı `os.fsync` (güç kesintisi güvenli, 20 dk teslim kısıtına uygun),
2 Hz ≥ 1 Hz garantisi, `CSV_HEADER` Dosya-2 ile birebir, mavros sensör topic'leri
için `BEST_EFFORT` QoS (gerçek bir tuzak doğru çözülmüş), `quat_to_rpy` doğru,
`self._csv` isimlendirmesiyle rclpy `self._logger` çakışması bilinçli önlenmiş.
Bu çekirdeğe dokunma; düzeltmeler yol (F15.1) ve eksik yedekler (F15.3/4).

## T0-f — Görevi FC'den oku ✅ (VİDEO + YARIŞMA, yeni kod)

**Şartname bağı:** md 3.3.1(2) "YKİ üzerinden 4 noktalı görev tanımlanacak ve bu
görev İDA'ya **gönderilecektir**" + md 5.5.2.2 "görev yükleme, alana girişe
müteakip İDA'ya güç verildikten sonra yapılacaktır." **Video eleme kapısının
2. zorunlu maddesi.**

### Sorun (denetim bulgusu, önceki fazlardan)

`mission_manager_node` görevi **yalnız araç üstü `config/video_mission.yaml`**'dan
okuyordu (`_load_mission`). QGC'den Pixhawk'a yüklenen waypoint'ler
`/mavros/mission/waypoints`'e düşer ama **hiçbir node onu okumuyordu** →
"YKİ'den yüklenen görev" ile "İDA'nın icra ettiği görev" **farklı**. Hem video
hem yarışma şartını ihlal ediyordu.

### Düzeltme (mekanizma, iki etkinliği de kapatır)

- **Çekirdek (Layer 0, `prototype/mission/mission_manager.py`):** `FcMissionItem`
  + `fc_items_to_waypoints(items, skip_home_seq0=True)`. mavros_msgs'e **bağımsız**
  (node alanları çıkarır) → pytest ile test edilir. Filtreler: (1) ArduPilot
  index 0 = home atla, (2) yalnız gezinme komutları NAV_WAYPOINT(16)/
  NAV_SPLINE_WAYPOINT(82), (3) lat==lon==0 tanımsız item atla. Parkur etiketi
  FC'den gelmez → hepsi parkur=1 (video tek parkur).
- **Node (Layer 2, `mission_manager_node.py`):** `mission_source` param
  (`"file"`|`"fc"`). fc modunda boş başlar, `/mavros/mission/waypoints`
  (`mavros_msgs/WaypointList`, **latched QoS** — geç abone son görevi alır)
  dinler; `WaypointList` gelince görevi (yeniden) kurar. `mavros_msgs` **lazy
  import** (file modu + pytest mavros'suz çalışsın). Görev yalnız **başlamadan**
  yüklenir (md 5.5.2.2); başladıktan sonra gelen liste yok sayılır. FSM aktif
  olsa da görev yüklenmemişse başlatma yapılmaz (`_started` latch kilidi önlendi).
- **cfg:** fc modunda arrival/dwell/cruise node parametrelerinden (`params.yaml`);
  file modunda YAML kazanır (geriye dönük uyumlu).
- **Wiring:** `hardware.yaml mission.{mission_source, skip_home_seq0}` +
  `hardware.launch` DeclareLaunchArgument (`mission_source:=fc` CLI override) +
  `params.yaml mission_manager_node` varsayılanları. **Varsayılan `file`**
  (offline/geliştirme bozulmasın); **sahada video/yarışma günü `fc`**.
- **QoS:** `qos_profiles.latched_qos()` (RELIABLE + TRANSIENT_LOCAL) — mavros
  waypoint plugin'i latch'li yayınlar, VOLATILE ile eşleşmezdi.

### Doğrulama

- **7 çekirdek testi** (`test_mission_manager.py`): home atlama, skip kapalı,
  gezinme-dışı komut filtresi, (0,0) atlama, parkur=1 etiketi, boş/yalnız-home,
  FC→MissionManager sürüşü. → **111 test** (104 + 7), <1 sn.
- **2 ROS node testi** (`test_mission_manager_node.py`, sourced ROS): fc callback
  görevi rebuild ediyor + başladıktan sonra reddediyor; görev yüklenmeden FSM
  aktifte başlamıyor. **mavros_msgs kurulu olmasa da** duck-typed mesajla çalışır
  (abonelik kurulmaz, dönüşüm+rebuild+kilit mantığı test edilir). → 4/4 geçti.

### Kalan (saha)

- **mavros_msgs Jetson'da kurulu olmalı** (`ros-humble-mavros-msgs`) — fc modu
  onsuz boş görevde kalır (hata loglanır, çökmez).
- **Gerçek QGC→Pixhawk→MAVROS zinciri sahada test edilmeli:** WaypointList'in
  gerçekten latched geldiği, ArduRover'da home item'ın index 0 olduğu, komut
  kodlarının 16 olduğu HIL/sahada teyit edilecek (`skip_home_seq0` param'ı ters
  çıkarsa tek anahtarla kapatılır).
- Yarışma modunda parkur etiketleri: FC görevi hepsini parkur=1 yapar; Parkur
  1/2/3 ayrımı gerekince ayrı ele alınacak (video için sorun değil, tek parkur).

---

## T0-g — Ekran-2 grafik telemetrisi ✅ (VİDEO, yeni kod)

**Şartname:** md 3.3.1.1 Ekran-2 — videoda İDA hareketiyle **senkron** üç
grafik zorunlu: (a) gerçek hız + hız setpoint, (b) heading/yaw + yaw setpoint,
(c) **thrusterlardan kuvvet isteği**. F15.2 tespiti: `/girdap/control/thrust`
yayınlanıyordu ama hiçbir yere kaydedilmiyordu → Ekran-2c çizilemezdi.

### Tasarım kararı

- **Dosya-2 CSV_HEADER'a DOKUNULMADI** (md 4.2 thrust istemiyor; sözleşme
  donmuş). Testle sabitlendi (`test_dosya2_header_frozen`).
- Ekran-2'nin üç sinyali tek zaman tabanında **ayrı grafik CSV'sine** gider:
  `~/girdap_logs/grafik/grafik_<UTC>.csv`, header
  `zaman,hiz,hiz_setpoint,heading,yon_setpoint,thrust_sol,thrust_sag`.
  Ayrı dizin: USB teslimine yalnız `telemetry/` kopyalanır, grafik dosyası
  yarışma çıktısı değil.
- **10 Hz** (`graph_rate_hz`) — MPPI control_rate (F4.2 sonrası 10 Hz) ile
  aynı; Dosya-2'nin 2 Hz'i thrust isteğini alias'lardı. Jetson yükü: satır
  başına format+fsync, ihmal edilebilir (sürekli CPU döngüsü yok).
- **F15.4 birlikte kapatıldı:** `velocity_body` hiç gelmezse hız
  `/girdap/fusion/odom` twist'inden yedeklenir (child-frame büyüklük) —
  Ekran-2a hız grafiği boş kalamaz. IMU-heading yedeğiyle aynı desen.

### Uygulama

- Layer 0 `prototype/telemetry/csv_logger.py`: `GRAPH_CSV_HEADER` +
  `GraphSample` (thrust 2 ondalık); `TelemetryCsvLogger`'a opsiyonel `header`
  parametresi (verilmezse eski davranış birebir — Dosya-2 etkilenmez).
- Layer 2 `telemetry_node.py`: `/girdap/control/thrust`
  (`Float32MultiArray [T_sol, T_sag]`, `len>=2` korumalı) aboneliği; ayrı
  `graph_rate_hz` timer'ı; `destroy_node` iki CSV'yi de kapatır;
  `**node_kwargs` passthrough (mission_manager_node ile aynı test deseni).

### Doğrulama

- **5 yeni çekirdek testi** (`test_telemetry_logger.py`): Dosya-2 header
  donmuş, grafik header sözleşmesi, satır formatı (thrust 2 ondalık), eksik
  alanlar boş, varsayılan header eski davranış. → **116 test** (111 + 5), <1 sn.
- **1 ROS node testi** (`test_telemetry_node.py`, sourced ROS, mavros'suz):
  thrust + odom yayınla → grafik CSV'de `12.30/-7.00` + hız odom yedeğinden
  `1.000`; Dosya-2 header'ı değişmemiş. → geçti. (Not: testte node birkaç kez
  spin edilmeli — 20 Hz timer tek `spin_once`'ı doyurur, wait-set'te timer
  abonelikten önce gelir.)

### Kalan (video günü)

- Grafik CSV'den üç paneli çizen basit matplotlib script'i video montajında
  lazım olacak (offline iş, araç üstünde koşmaz — istenince yazılır).
- `graph_rate_hz` sahada MPPI gerçek frekansına göre ayarlanabilir.

---

## Faz 6 — Sentetik üreteçler: test maskeleme avı ✅ (T1, salt okuma)

F5.1 ve F5.9 aynı deseni göstermişti: *sentetik üreteç, üretim kodunun hatalı
varsayımını aynen kodlar → test yeşil kalır, hata görünmez.* Üç üreteç
(`synthetic_lidar/camera/fusion.py`, 225 satır) + tüketen 6 test dosyası +
`viz/scenario.py` sistematik tarandı.

### 🟡 F6.1 — Bearing işaret maskesinin TAM mekanizması (F5.9 kesinleşti)

Fiziksel gerçek: base_link'te y **SOL**+ (`atan2`), görüntüde `bbox_cx=0`
**SOL** kenar. Soldaki nesne → LiDAR bearing **pozitif**, `bearing_from_camera`
(`fusion.py:83` `(cx-0.5)*hfov`) **negatif**. İşaretler TERS (bulgular #3).
Maskeleme ÜÇ katmanlı:
1. `synthetic_fusion.py:26-28` `_cx_for_bearing` ters fonksiyon → soldaki (y=+3,
   bearing +0.540) duba bbox'ı `cx=0.95`'e, yani görüntünün **SAĞINA** konur.
   Sahne fiziksel olarak imkânsız ama formülle tutarlı → eşleşme testleri yeşil.
2. `test_fusion.py:60-68` sol kenar→`−hfov/2` assert'i **yanlış kuralı
   donduruyor** — işaret düzeltilirse bu test kırmızı olur (test hatalı
   davranışın bekçisi).
3. Hiçbir test aynı fiziksel nesneyi İKİ sensöre bağımsız geometriyle
   yerleştirmiyor (kamera hep LiDAR'dan türetiliyor).

**Düzeltme reçetesi (T1, tek commit'te):** `bearing_from_camera` işareti +
`_cx_for_bearing` + kenar testi birlikte çevrilir; ayrıca **fiziksel tutarlılık
testi** eklenir: (5, +3)'teki nesne için iki sensör bearing'i aynı işarette
olmalı — üreteçten bağımsız, ham geometriyle. Bu test var olsaydı hata ilk
günden yakalanırdı. (Bugün ölü kol: `/perception/classified_obstacles`
abonesiz; yine de F3.4 extrinsic işiyle birlikte T1'de düzeltilmeli.)

**✅ DÜZELTİLDİ (Faz 18-T1, reçete aynen — TEK commit):**
`bearing_from_camera = (0.5 − cx)·hfov` (sol pozitif = atan2 ile aynı) +
`_cx_for_bearing = 0.5 − b/hfov` + `viz/scenario.py:145` aynı çevirme (üçüncü
kopya oradaydı) + kenar testi doğru kurala güncellendi. İki YENİ üreteçten
bağımsız test: fiziksel işaret tutarlılığı (sol/sağ iki sensörde aynı işaret)
+ ham geometriyle soldaki duba eşleşmesi (eski işaretle 1.08 rad ayrık düşer,
eşleşmezdi). TDD 3 kırmızı→yeşil; suite **131**. Kamera ters monte çıkarsa
değişim noktası yine tek: `bearing_from_camera`.

### 🟡 F6.2 — LiDAR üreteci F5.1'in çerçeve hatasını kodluyor (bilinen, sınırı çizildi)

`synthetic_lidar.py:39` `z=uniform(0, height)` + `:63` su gürültüsü
`z∈[−0.05,0.08]` — her ikisi **su yüzeyi orijinli**; gerçek Livox verisi sensör
çerçevesinde (`z≈−h` altında). `test_lidar_obstacles.py`'nin Z-filtre "negatif
örnek" testi yalnız bu hayali çerçevede anlamlı. **F5.1 düzeltmesi
(`lidar_height_m`) geldiğinde üreteç + testler AYNI commit'te güncellenmeli** —
yoksa ya boş yere yeşil kalırlar ya yanlış kırmızıya döner.

### 🟡 F6.3 — Küme boyutu uçları hiç tetiklenmiyor: F5.4 testlerde ATEŞLENEMEZ

`points_per_buoy=40` sabit — mesafeden bağımsız (gerçekte nokta sayısı ~1/d²).
Sonuç: `max_cluster_size=500` üst sınırı (F5.4 — **en yakın engeli sessizce
siler**) hiçbir sentetik sahnede aşılmıyor (40 ≪ 500); `min_cluster_size=5`
alt sınırı da uzak-seyrek dubayla hiç zorlanmıyor (menziller 4-20 m, sınır
25 m). İki filtre ucu da test edilmemiş durumda. T1: yakın-büyük küme (>500
nokta) + 24 m'de 4-5 noktalı duba sahneleri eklenmeli.

> ✅ **KAPANDI (2026-07-11, F5.4 düzeltmesiyle aynı commit).** İki yeni sahne
> `synthetic_lidar.py`'de: **`scene_yakin_duvar`** (8 m iskele duvarı x=6 +
> 2 duba; duvar voxel 0.1 sonrası >500 hücre — sahnenin üst sınırı gerçekten
> aştığı AYRI ön koşul testiyle doğrulanıyor, maskeleme avı dersi) ve
> **`scene_uzak_seyrek_duba`** (24 m'de 4 dönüş; `min_cluster_size=5` altında
> → görünmez — BİLİNEN sınırlama testle belgelendi; aynı sahne
> `min_cluster_size=3` ile görünür → sınırın menzilde değil eşikte olduğu
> kanıtlı). Kamera tarafı çeldiricileri (beyaz sosis, FOV kenarı, 15 m HSV
> sınırı) F6.5 kapsamında AYRI iş olarak açık.

### 🟡 F6.4 — Kamera testleri DÖNGÜSEL: renkler dedektörün kendi aralığından seçilmiş

`synthetic_camera.py:19-22` docstring'i itiraf ediyor: "HSV karşılıkları
camera_buoys varsayılan aralıklarının içinde". Test, dedektörün *tespit
edilebilir seçilmiş* renkleri tespit ettiğini doğruluyor — HSV aralıklarının
GERÇEK duba renkleriyle (RAL 2003/1026) kalibrasyonu hakkında hiçbir şey
söylemiyor. Yorumlardaki RAL kodları da yanlış (2008/1003 — F4.5 ile aynı;
değerler gerçek RAL'a tesadüfen uygun). Sentetik kamera testinin doğası gereği
tam çözümü yok; gerçek saha karesi fikstürü (masa testi fotoğrafları) eklenince
kapanır. Yorumlar T1'de düzeltilecek.

### ⚪ F6.5 — Negatif/çeldirici sahne yok

Beyaz sosis duba (şartname: parkur çevresi), her-iki-renk-bir-arada + doluluk
skoru (F5.6) ayrımı, FOV kenarı ve 15 m HSV menzil sınırı (F5.5) — hiçbiri
sentetik sahnelerde yok. `viz/scenario.py` da aynı üreteçleri kullandığından
görselleştirme bu kör noktaları miras alır.

> ✅ **KAPANDI (2026-07-11).** `synthetic_camera.py`'ye 4 çeldirici sahne +
> `test_camera_buoys.py`'ye 4 belgeleme testi (tüm sınır davranışları önce
> ampirik doğrulandı, sonra iddiaya bağlandı):
> - **`scene_camera_beyaz_sosis`**: sosis hattı (S≈0) parlama+gürültü altında
>   ateşlemiyor, yalnız 2 gerçek duba tespit ediliyor.
> - **`scene_camera_turuncu_serit`**: F5.6 tersliği testte GÖRÜNÜR — şerit
>   0.961 > daire duba 0.757 (test docstring'i: skor sözleşmesi düzeltilince
>   test doğru davranışa güncellenecek, buggy davranışı dondurmuyor).
> - **`scene_camera_fov_kenari`**: aynı boy duba ortada tespit, kenarda yarım
>   → görünmez (bilinen kör nokta); bbox'ın frame içinde kaldığı da doğrulanıyor.
> - **`scene_camera_menzil_siniri`**: F5.5 eşiği iki yanı — r=8 (~13-14 m)
>   tespit, r=6 (~17-18 m) görünmez ⇒ ≈15 m etkin menzil kanıtlı.
> F5.5 (sözleşmeye yazılması) ve F5.6 (skor semantiği kararı — iki repo
> arası) bulguları hâlâ AÇIK; bu kapama yalnız test-görünürlüğü sağlar.

### Özet

| Bulgu | Maskelediği | Eylem (T1) |
|---|---|---|
| F6.1 ters-fonksiyon + donmuş kenar testi | bearing işareti (bulgular #3) | işaret+üreteç+test tek commit + fiziksel tutarlılık testi |
| F6.2 su-yüzeyi orijinli z | F5.1 çerçeve hatası | F5.1 düzeltmesiyle aynı commit |
| F6.3 sabit 40 nokta/duba | F5.4 büyük-küme silme, min-küme ucu | 2 yeni sahne |
| F6.4 döngüsel renk seçimi | HSV↔gerçek RAL kalibrasyonu | saha karesi fikstürü + yorum |
| F6.5 çeldirici yok | F5.5/F5.6 | negatif sahneler |

---

## Faz 7 — Algı ROS node'ları ✅ (T1, salt okuma)

Layer 2 sarmalayıcıları: `perception_lidar_node` (138), `perception_camera_node`
(157), `perception_fusion_node` (181). Çekirdek bulguları (F5.x) tekrar
edilmedi; burada yalnız node/ROS katmanı.

### 🟠 F7.1 — Füzyon senkronizasyonu SESSİZCE hiç ateşlemeyebilir (stamp kaynağı uyumsuz)

`ApproximateTimeSynchronizer(slop=0.1 s)` iki topic'in stamp'lerini eşliyor ama
iki üretici FARKLI zaman tabanı kullanıyor:
- Bizim OAK node'u (`duba_gecis_navigator.py:512`) `/perception/buoys`'u
  **yayın anı** saatiyle damgalar (`get_clock().now()`, VPU çıkarımı ~85 ms
  gecikmiş görüntüye ait).
- `perception_lidar_node.py:106` `/perception/obstacle_map`'e **Livox sürücü
  stamp'ini** aynen taşır (tarama zamanı; livox_ros_driver2 config'ine göre
  sistem saati YA DA sensör zaman tabanı olabilir).

Sapma > 0.1 s ise eşleşme HİÇ oluşmaz ve **hiçbir log basılmaz** —
`_periodic_info` yalnız callback ATEŞLENİNCE çalışıyor; ateşlenmeyen sync =
sonsuz sessizlik, sahada "füzyon çalışıyor" sanılır. (Bugün ölü kol —
`classified_obstacles` abonesiz — ama T1'de bearing düzeltmesiyle canlanacak.)
**Düzeltme (T1):** (a) sözleşmeye stamp kuralı yaz (iki taraf da yayın-anı
`now()` kullansın ya da slop ≥0.3 s), (b) node'a "N saniyedir girdi var ama
sync ateşlemedi" watchdog WARN'u ekle.

**✅ DÜZELTİLDİ (Faz 18-T1, bekçi kısmı):** `sync_watchdog_s` (10 s) timer'ı —
pencerede iki girdi de aktı ama eşleşme sıfırsa WARN (sebep + çare mesajda);
STAMP SÖZLEŞMESİ node docstring'ine yazıldı. `py_compile` ✓ (vision_msgs yok).
Kalıcı stamp hizası (Livox saat kaynağı / slop ölçümü) SAHA işi — bekleyen
girdiler listesine eklenecek.

### 🟡 F7.2 — `int(hyp.class_id)` sayısal olmayan class_id'de node'u ÖLDÜRÜR

`perception_fusion_node.py:154` — sözleşme `"0"/"1"/"2"` diyor ama string alan
serbest metin taşıyabilir; `int("kenar")` → ValueError → rclpy callback
istisnası `spin()`'i sonlandırır, füzyon node'u ölür. Bizim OAK node'u uyumlu
(`"0"/"1"` basar) — risk üçüncü kaynak/ileride YOLO `"2"` dışı etikette.
try/except + WARN yeterli (T1, 3 satır).

**✅ DÜZELTİLDİ (Faz 18-T1):** try/except ValueError → WARN + tespit atlanır.
`py_compile` ✓ (vision_msgs bu makinede yok, node testi Jetson'da).

### 🟡 F7.3 — LiDAR callback'i ağır kümelemeyi SENKRON koşuyor; derinlik 10 = 1 s bayat kuyruk

`_on_cloud` → `detect_obstacles` (F5.3: voxel'siz saf Python, yüzlerce ms)
tek-thread executor'da. `sensor_data_qos()` depth=10 → işleme yetişemeyince
kuyrukta 10 tarama birikir; node sürekli **~1 s bayat** bulutları işler (drop-
to-latest yok). Livox 10 Hz'te engel haritası saniye mertebesi gecikir → MPPI
bayat engelle plan yapar. **Düzeltme (T1):** abonelikte `depth=1` (en yeni
tarama kalır) — F5.3'ün hız düzeltmesinden bağımsız, tek satır, ucuz sigorta.

**✅ DÜZELTİLDİ (Faz 18-T1):** `/livox/lidar` aboneliği `sensor_data_qos(
depth=1)` — her callback eldeki en yeni taramayı işler. `py_compile` ✓.

### 🟡 F7.4 — Bilinen launch sorunları hâlâ açık (F3.1/F3.2 teyit)

`hardware.launch.py:379-382` hâlâ donanım ekibinden `depthai_ros` OAK sürücüsü
eklemesini istiyor (F3.1: bizim node OAK'ı doğrudan DepthAI ile açıyor — sürücü
eklenirse USB exclusive çakışması, kamera ölür) ve `use_mppi` LogInfo'su hâlâ
ölü argümanı basıyor (F3.2). İkisi de Faz 18-T1 düzeltme listesinde.

**✅ DÜZELTİLDİ (Faz 18-T1, F3.1+F3.2):** `perception_camera_node` artık
`use_onboard_camera` (VARSAYILAN **false**) koşuluna bağlı — `/perception/
buoys`'un tek üreticisi algı ekibinin OAK node'u; HSV node yalnız bilinçli
yedek. SENSOR DRIVERS yorumundaki depthai_ros talimatı "EKLEME!" uyarısına
çevrildi. `use_mppi` LogInfo'dan çıkarıldı, argüman açıklaması "REZERVE —
hiçbir node okumuyor" yapıldı (yanlış beklenti bitirildi). `py_compile` ✓.

### ⚪ F7.5 — Girdi kesilince tüm algı node'ları sonsuz sessiz

Üç node'da da `_periodic_info` callback-güdümlü: sensör/sürücü ölürse log akışı
sadece DURUR — "girdi yok" uyarısı yok. Saha teşhisi için her node'a girdi
zaman aşımı WARN'u (basit timer) T1'de değerlendirilebilir. (F7.1'in genel hali.)

### ✅ Doğru yanlar (dokunma)

Çekirdek/node ayrımı temiz (üç node da yalnız kablolama); QoS seçimleri bilinçli
ve gerekçeli (füzyonda RELIABLE eşleşmesi, sensörde BEST_EFFORT); kaynak stamp
koruma niyeti doğru (F7.1 kuralı netleşince); `_periodic_info` log-seli önleme
deseni tutarlı; placeholder PoseArray şeması downstream'le birebir (F4.6 ✓).

---

## Faz 8 — Füzyon/iSAM2 ✅ (fusion_node 225 + bypass 52 + pipeline 184 + isam2_smoother 223)

Video zinciri de buradan geçiyor: bypass modunda (`use_isam2=false`)
`PosePassthrough` `/girdap/fusion/odom`'un TEK kaynağı.

### 🔴 F8.1 (VİDEO) — Odometry twist HİÇ doldurulmuyor; MPPI araç hep DURUYOR sanıyor

`fusion_node.py:187-195` yalnız `pose`'u dolduruyor; `twist` sıfır kalıyor.
Yorumdaki gerekçe **BAYAT/YANLIŞ**: "planning_node şu anda sadece pozu
kullanıyor" — hayır: `planning_node._on_odom` `v.x, v.y, w.z`'yi okuyup MPPI
durum vektörüne (`[x,y,ψ,u,v,r]`) basıyor. Sonuç: **MPPI her kontrol adımında
rollout'a u=v=r=0'dan başlıyor** — araç 1.5 m/s seyrederken dinamik model
momentum yokmuş gibi öngörür → 2.5 s ufukta ciddi sapma → aşırı düzeltme /
salınım. md 3.3.1.1 "istemsiz hareket" için F11.1/F12.1/F12.2'den sonra
**4. kök neden.** Etkileşim: F15.4'ün odom hız yedeği de bu yüzden hep
"0.000" yazar (boş yerine yanıltıcı sıfır) — asıl düzeltme burada.
**Düzeltme:** iki modda da `/mavros/local_position/velocity_body`'den son
(vx, vy, ωz) cache'lenip `od.twist`'e yazılır (body-frame = `child_frame_id`
semantiğiyle DOĞRU). iSAM2 modunda abonelik zaten var (callback genişler);
bypass'a aynı abonelik eklenir.

**✅ DÜZELTİLDİ (Faz 18):** `_on_vel_body` her iki modda twist'i cache'ler
(iSAM2'de ayrıca smoother'ı besler), `_on_publish_timer` `od.twist`'i doldurur;
bypass moduna velocity_body aboneliği eklendi; `**node_kwargs` passthrough.
Doğrulama: yeni `test_fusion_node.py` ROS smoke (bypass, gtsam'sız): EKF poz +
body hız yayınla → odom'da pose VE twist (1.2/-0.1/0.25) + `child_frame_id=
base_link` → geçti (TDD kırmızı→yeşil, suite 125). velocity_body hiç gelmezse
twist eskisi gibi 0 kalır — regresyon yok, F15.4 yedeği artık anlamlı.

### 🟠 F8.2 — Bypass pozu bayatlamaya karşı korumasız: EKF akışı ölürse 50 Hz DONMUŞ poz

`PosePassthrough` son pozu latch'ler (`bypass.py:33-48`); `_on_publish_timer`
50 Hz yayına devam eder. `/mavros/local_position/pose` kesilirse (EKF reset,
mavros hıçkırığı) downstream DONMUŞ pozla plan yapar — hiçbir bayatlık kontrolü
yok (F13.6/F15.5 ailesi; zincirin en üstü burası). T1: son güncelleme > ~1 s
ise yayını kes + WARN — planning'in F14 heartbeat geçidi zaten thrust'ı keser.

**✅ DÜZELTİLDİ (Faz 18-T1):** `pose_timeout_s` param (varsayılan 1.0, 0=kapalı);
girdi (bypass: EKF pozu, iSAM2: IMU) `pose_timeout_s`'den eskiyse
`_on_publish_timer` yayını keser + tek seferlik WARN, akış dönünce INFO +
devam. Doğrulama: `test_fusion_node.py`'ye bayatlık senaryosu eklendi (akış
kes → odom sayısı sabitleniyor), TDD kırmızı→yeşil, suite **129**.

### 🟡 F8.3 — iSAM2 grafı sınırsız büyüyor (20 dk ≈ 12k+ anahtar)

`add_odometry` 10 Hz → 20 dk görevde >12.000 Pose2 anahtarı + GPS prior'ları.
Sliding-window/marginalization yok (CLAUDE.md kendisi "düşün" diyor).
`update()` maliyeti büyümeyle artar; Jetson'da uzun-görev ölçümü şart (D3
listesine). Video modunu etkilemez (iSAM2 kapalı).

### 🟡 F8.4 — İlk GPS fix'i araç HAREKET EDERKEN gelirse origin kayar

`pipeline.on_gps:121-127`: ilk fix yalnız origin'i pinler, faktör eklemez.
Araç ilk fix'ten önce odometriyle x=5 m'e ilerlediyse, origin oradaki fiziksel
konuma (0,0) diye pinlenir → sonraki GPS prior'ları smoother'ı geri çeker
(tek seferlik sıçrama). Pratikte pre-arm GPS fix şartı bunu önler (boot'ta
sabit); yine de "arm'dan önce fix bekle" operasyon kuralı nota yazılmalı.

### ✅ Sağlam yanlar (dokunma)

`ISAM2Smoother` temiz: GPS'i heading-serbest Pose2 prior'u yapma hilesi doğru
(sigma=1e6 kanal), compose tabanlı ilk tahmin doğru, pending graph/values
yaşam döngüsü doğru; `pipeline` dt guard'ları (`0<dt≤0.5`) ve GPS öncesi flush
mantıklı; ENU eşit-dikdörtgen projeksiyon <1 km alanda yeterli; GTSAM lazy
import (video GTSAM'sız koşar) F2.4'ün doğru çözümü.

---

## Faz 10 — RRT* ✅ (rrt_star.py 477, ilk kez okundu; T1 — video `use_rrt=false`)

### 🔴 F10.1 — `plan()` ValueError'ı KİMSE yakalamıyor → planning_node görev ORTASINDA ölür

`rrt_star.py:155-156`: start veya goal `(engel_r + safety_margin 0.3 m)` içinde
ya da **bounds dışında** ise `ValueError` fırlatır. Çağrı zinciri:
`/perception/obstacle_map` callback (10 Hz) → `set_obstacles` →
`_needs_replan` → `_global_replan` (`pipeline.py:195-196`) → `plan()` →
istisna YAKALANMIYOR → rclpy callback istisnası `spin()`'i bitirir →
**planning_node ölür** → cmd_vel/thrust akışı kesilir, görev biter (F14.6:
bağımsız watchdog da yok).

Senaryo GERÇEKÇİ: Parkur-2 geçit net açıklığı ~1.35 m; tekne dubaya
~0.45-0.7 m yaklaşırken replan tetiklenirse start engel payı içinde kalır.
F5.4 (kümelerin birleşip yarıçapın şişmesi) olasılığı büyütür. Ayrıca hedef
waypoint bir dubanın payı içindeyse aynı ölüm.
**Düzeltme (T1):** `_global_replan` try/except → WARN + eski `ref_path`'i
koru (plan() zaten "çözüm yok"ta None dönüyor; istisna yerine None dönmesi de
kabul). Test: start'ı engel payının içine koyan senaryo.

**✅ DÜZELTİLDİ (Faz 18-T1):** `_global_replan` `ValueError`'ı yakalar, WARN
loglar (`logging`, pipeline'a eklendi), eski `ref_path` korunur; "çözüm yok"
(None) da WARN'lanır. Doğrulama: yeni `prototype/tests/test_planning_replan.py`
(gtsam'sız — `test_planning_pipeline.py` FusionPipeline importu yüzünden bu
ortamda koşamıyor): start payın içinde + goal payın içinde senaryoları, TDD
kırmızı→yeşil.

### 🟠 F10.2 — F4.4 KESİNLEŞTİ: `bounds [0,200]²` odom-merkezli çalışmaz

`planning_node.py:76-78` bounds'u `params.yaml`'dan alıyor; ENU origin = boot
konumu → araç köşede (0,0). Batı/güneye (negatif x/y) görev: goal bounds
dışı → **ValueError → F10.1 ölümü**; goal içeride ama araç dışarı sürüklendi
→ start için aynı. Örnekleme yalnız KD çeyrekte. **Düzeltme (T1):** bounds'u
start+goal zarfından dinamik kur (± pay, örn. 50 m) ya da origin-merkezli
`[-100,100]²` yap; F10.1'in try/except'i her durumda şart.

**✅ DÜZELTİLDİ (Faz 18-T1):** örnekleme alanı artık `statik bounds ∪
(start/goal ± bounds_margin_m)` (yeni config alanı, varsayılan 30 m) —
start/goal daima alan içinde; statik `[0,200]` param davranışı üst-küme olarak
korunur (ölü config yok). Doğrulama: negatif çeyrek senaryosu (araç (-20,-10),
hedef (-40,-30)) plan üretiyor. Suite **128**.

### 🟡 F10.3 — `_nearest_idx` her iterasyonda diziyi Python döngüsüyle yeniden kuruyor

`rrt_star.py:283-289`: O(n) Python fill × 1500 iter ≈ 1-2 M döngü adımı →
Jetson CPU'da plan başına ~1-2 s; replan **kontrol thread'inde senkron**
koştuğundan 10 Hz control step'leri bloklar (MPPI ~100 ms/step'e ek). Görev
sırasında görünür duraksama = istemsiz hareket algısı riski. D3 ölçümüne ekle;
düzeltme: diziyi artımlı büyüt (basit) — algoritmaya dokunmadan.

### ✅ Sağlam yanlar (dokunma)

- **Rewire döngü (cycle) riski YOK — doğrulandı:** ata düğümün torununa
  rewire'ı maliyet tutarlılığı imkânsız kılar (cand = new.cost + d > ata.cost,
  üçgen eşitsizliği; `_reattach` propagation'ı tutarlılığı koruyor).
- Informed elips örneklemesi (Gammell 2014) doğru: a=c_best/2,
  b=√(c²−c_min²)/2, rotasyon+öteleme doğru; `use_informed` yalnız çözümden
  sonra devrede.
- `_reattach` iteratif cost-propagation (recursion patlaması yok), çarpışma
  kontrolü broadcast-vektörize, matplotlib tembel import (runtime'a sızmıyor),
  `_near_indices` Karaman yarıçapı step×4 ile sınırlı. Çekirdek matematik
  SAĞLAM — sorunlar çağrı sözleşmesi (istisna) ve config (bounds) katmanında.

---

## Faz 16 — Test paketi denetimi ✅ (24 dosya, 195 test fonksiyonu)

> Kapsam: `prototype/tests/` içeriği + test ALTYAPISI (toplama, ortam
> kapılaması, CI). Faz 6 üreteç maskelemesini kapsamıştı; bu faz testlerin
> kendisini ve koşulabilirliğini denetler. Şartname bağı: video düzeltmelerinin
> (F11.1, F12.2, F8.1, …) tek regresyon teli bu suite — suite sessizce
> koşamazsa düzeltmeler telsiz kalır (md 3.3.1.1 istemsiz-hareket kriterinin
> güvencesi bu testlerdir).

### 🔴 F16.1 — ROS ortamında `pytest prototype/tests/` KOŞAMAZ; ignore'larla "yeşil görünen SIFIR koşu"

ROS source'lanınca `launch_testing` pytest plugin'i otomatik yüklenir
(`/opt/ros/humble/.../launch_testing/pytest/hooks.py:193`) ve toplama
sırasında HER dosyayı hevesle import eder (`find_launch_test_entrypoint` →
`path.pyimport()`). Sonuç (bu oturumda birebir yeniden üretildi):
- Tek kırık transitive import (gtsam / scipy-ABI / vision_msgs) `Interrupted:
  1 error during collection` ile **TÜM suite'i** öldürür — sağlam 145 test
  dahil hiçbir şey koşmaz.
- Kırık dosyalar `--ignore` edilirse dizin koşusu **"1 skipped, exit 0"**a
  düşer: sıfır test koşar, çıkış kodu yeşildir. CI/operatör "testler geçti"
  sanır. **Yanlış-yeşil tuzağı.**
- `pyproject.toml testpaths=["prototype/tests"]` yüzünden çıplak `pytest` de
  aynı tuzağa düşer.

Repo'da launch testi YOK (`generate_test_description`: 0 sonuç) → plugin'i
kapatmak güvenli. **Düzeltme:** `pyproject.toml`
`addopts = "-p no:launch_testing -p no:launch_ros"`.
(Plugin kayıt adları: `launch_testing` ve `launch_ros` — egg-info
entry_points'ten; `no:launch_testing_ros` YANLIŞ ad, tutmaz.)

### 🟠 F16.2 — Import kapılaması tutarsız: tek dosya tüm suite'i öldürüyor

Node testleri `pytest.importorskip("rclpy")` ile kapılı (doğru desen) ama:
- (a) `test_perception_camera_node.py:21` + `test_perception_fusion_node.py:23`
  `vision_msgs`'i ÇIPLAK import ediyor → rclpy VAR ama vision_msgs YOK olan
  ortamda (bu makine; Jetson'da da kurulum unutulursa) toplama HATASI.
- (b) scipy×numpy2 ABI kırığı `ValueError` fırlatır (`numpy.dtype size
  changed`); `pytest.importorskip` yalnız `ImportError` yakalar →
  `test_lidar_obstacles.py` + `test_perception_lidar_node.py` toplama HATASI
  (skip değil).
- (c) `test_parkur_fsm_node.py:29` skip nedeni YANILTICI: "girdap_decision
  source'lanmamış" der; gerçek neden `fsm_node.py:66`'nın `mavros_msgs`
  import'u. Operatör yanlış şeyi kurmaya çalışır.

F16.1 ile birleşince: bu dosyalardan HERHANGİ biri tüm koşuyu öldürüyor.
**Düzeltme:** vision_msgs'e importorskip; scipy'ye modül-düzeyi
`try/except (ImportError, ValueError) → pytest.skip(allow_module_level=True)`;
parkur_fsm_node skip nedenini gerçekçi yaz.

### 🟠 F16.3 — `test_planning_pipeline` modül-düzeyi FusionPipeline import'u 7 kapalı-döngü testini rehin tutuyor

gtsam yalnız 1 testte gerekli (`test_e2e_fusion_to_planning_chain`;
`prototype/fusion/pipeline.py:34` modül düzeyinde `import gtsam`). Import
modül düzeyinde olduğu için dosyanın TAMAMI — `test_closed_loop_reaches_goal`,
`test_closed_loop_avoids_obstacle` dahil, yani **MPPI kararlılığının en
doğrudan davranış telleri (md 3.3.1.1)** — gtsam'sız makinede koşamıyor.
Faz 18'in MPPI warm-start düzeltmeleri bu teller olmadan gitti (bypass kapalı
döngüsü kısmi teminattı). `prototype.planning.pipeline`'ın kendisi gtsam'sız
temiz import oluyor (doğrulandı) — rehinenin tek nedeni test dosyasındaki
import yeri. **Düzeltme:** FusionPipeline import'unu o tek testin içine taşı +
`pytest.importorskip("gtsam")`.

### 🟠 F16.4 — CI YOK; CLAUDE.md "var" diyor

`.github/workflows/` mevcut değil. `CLAUDE.md` Test Stratejisi bölümü "CI:
GitHub Actions, pytest + cmake build + ctest. Push'ta otomatik" iddia ediyor.
Testler yalnız elle koşuluyor; F16.1 yüzünden ROS'lu bir CI kurulsaydı da
sessiz sıfır-koşuya düşerdi. CLAUDE.md düzeltmesi Faz 17'ye; CI kurulumu
(ROS'suz çekirdek job bile yeter: 145 test) ekip kararı.

> **✅ F16.4 DÜZELTİLDİ (2026-07-11):** `.github/workflows/ci.yml` — ROS'suz
> çekirdek job (ubuntu-latest, Python 3.10 = Jetson paritesi, requirements.txt,
> `pytest prototype/tests/ -q`). Yerel simülasyon (ROS source'suz, temiz env):
> 210 passed / 9 gerekçeli skip / 0 error. Yanlış-yeşil koruması: pytest 0
> test toplarsa exit 5 → job kırmızı (F16.1 dersi). CLAUDE.md CI satırı
> güncellendi.
> **Gerçek Actions koşuları:** ilk koşu 209/10 (runner'da ffmpeg yok → Ekran-2
> MP4 testi skip); ffmpeg adımı eklendi (`c78f95a`) → **210 passed / 9 skip,
> lokalle birebir, YEŞİL.** Kalan 9 skip: 8 rclpy-kapılı node modülü (ROS'suz
> ortamda tasarım gereği) + 1 cupy paritesi (Jetson'da koşacak).

> **✅ F16.5/F12.1 DÜZELTİLDİ (Faz 18-T1):** ortam tamamlandıktan sonra
> (mavros_msgs kaynaktan derlendi) yeni `prototype/tests/test_fsm_node.py`
> yazıldı — 3 test F12.1'i KIRMIZI yakaladı (sahte P1→P2 video senaryosunda
> bile tetikleniyordu). Düzeltme (fsm_node, çekirdek FSM'e DOKUNULMADI):
> (a) `last_waypoint_xy` [0,0] varsayılanı "ayarlanmamış" sayılır → mesafe
> hesaplanmaz (inf kalır); (b) gerçek tetik: `/girdap/mission/waypoint_reached`
> index'i parkur-1'in SON index'ine eşitse `dist_to_last_wp_p1=0` beslenir
> (waypoint-index + parkur etiketi tabanlı — CLAUDE.md FSM ilkesi; yeni topic
> yok). Elle gerçek koordinat verilirse odom-mesafe yolu yedek olarak çalışır
> (testli). params.yaml'ın yalan yorumu ("planning override eder") düzeltildi.
> Ayrıca F4.5'in beklettiği yanlış RAL yorum etiketleri (params.yaml +
> camera_buoys + plotter + synthetic_camera, 10 yer) 2003/1026'ya çekildi.
> Doğrulama: 4 yeni test + TÜM suite **206 passed** (ortam kurulumu sonrası
> taban 202 + 4).

### 🟡 F16.5 — fsm_node HİÇBİR ortamda test edilmiyor + F12.1 hâlâ AÇIK

Tek fsm_node-düzeyi test dosyası (`test_parkur_fsm_node.py`) mavros_msgs'siz
her makinede sessizce skip (F16.2c). **F12.1 kodda duruyor:** `fsm_node.py:89`
`last_waypoint_xy=[0.0,0.0]` parametresini HİÇBİR ŞEY yazmıyor (grep: 2
kullanım, ikisi de fsm_node içinde) → görev başlar başlamaz odom origin'e
0 m = sahte P1→P2. Önceki denetim "video düzeltme paketi"nde saymıştı ama
uygulanmadı ve kalan-listeden düştü. Video etkisi SINIRLI: F12.2 sayesinde
`mission_complete` her PARKUR*'dan TAMAMLANDI'ya götürür → video kurtulur;
ama MPPI ağırlık profili sahte P2'ye geçer ve FSM durumu yanlış yayınlanır.
**YARIŞMADA gerçek P1→P2 kapısı bozuk** — Faz 18-T1 listesine GERİ ALINDI.
Düzeltme adayı: mission_manager parkur-1 son wp'sini `/girdap/mission/...`
kanalıyla yayınlar ya da fsm_node mission_file'dan kendisi okur.

> **✅ F16.6 DÜZELTİLDİ (Faz 18-T1):** yeni `prototype/tests/test_rrt_star.py`
> — 6 deterministik tel: boş harita start→goal bağlantısı + uzunluk üst sınırı,
> engel açıklığı (yol örnekleme), F10.1 ValueError sözleşmesi (start/goal pay
> içinde + bounds dışı — pipeline try/except'i buna dayanır), seed
> determinizmi, `best_cost` = gerçek yol uzunluğu (rewire maliyet tutarlılığı
> teli), ulaşılamaz goal → None. 6/6 geçti; TÜM suite 212 passed.

### 🟡 F16.6 — rrt_star çekirdeğinin (477 satır) doğrudan birim testi yok

`test_rrt_star.py` yok; kapsam yalnız pipeline üzerinden dolaylı
(`test_planning_pipeline` — F16.3 yüzünden lokalde koşamıyordu;
`test_planning_replan` yalnız replan yolunu test eder). Faz 10 matematiği
elle doğruladı (rewire/informed elips sağlam) ama regresyon teli yok —
rrt_star'a dokunacak ilk kişi telsiz çalışır. T1 işi: collision check +
rewire maliyet tutarlılığı + informed elips + bounds∪zarf için ~5
deterministik birim test.

### ✅ F16.7 — Pozitif: test paketi NİTELİKLİ, yeni maskeleme deseni YOK

Faz 6'nın üreteç bulguları (F6.1-F6.5) dışında maskeleme bulunamadı. Güçlü
desenler: white-box deterministik MPPI maliyet testleri (stokastik kırılganlık
bilinçli atlanmış), kapalı-döngü yakınsama + engel açıklığı, warm-start
invariant'ları, Dosya-2 header'ı donduran sözleşme testi, fsync dayanıklılık,
bearing fiziksel tutarlılık (F6.1 reçetesi), FSM tek-yönlülük + determinizm,
`test_yolo_mock_never_imports_ultralytics` gibi lazy-import telleri.
Çekirdek testler (`mission_fsm` vb.) buggy node-besleme davranışını
DONDURMUYOR (F12.1 testlerde yok, sadece kapsam dışı — doğru taraf).

### 📊 Bu makinedeki gerçek taban çizgisi (2026-07-11)

Ortam kırıkları: gtsam yok (1 dosya), scipy+matplotlib×numpy2 ABI (3 dosya),
vision_msgs yok (2 dosya), mavros_msgs yok (parkur_fsm_node skip).
Belgelenmiş 131'lik dosya-listesi komutu gerçek setin ALT KÜMESİydi
(test_camera_buoys + test_fusion_bypass dışarıda kalıyordu).

**✅ F16.1 + F16.2 + F16.3 DÜZELTİLDİ (Faz 18-T1 devamı):**
- `pyproject.toml` `addopts="-p no:launch_testing -p no:launch_ros"` (F16.1)
- vision_msgs importorskip (2 dosya) · scipy/matplotlib ABI hataları modül
  düzeyi try/except→skip (3 dosya: ValueError/AttributeError importorskip'i
  deler) · gtsam importorskip (fusion_pipeline) · parkur_fsm_node skip nedeni
  gerçekçi (F16.2)
- FusionPipeline import'u e2e testin içine taşındı + gtsam importorskip
  (F16.3) → 7 kapalı-döngü testi serbest kaldı
- **Doğrulama:** ROS source'lu, ignore'suz `pytest prototype/tests/` =
  **156 passed, 8 skipped (hepsi gerekçeli), 0 error, 17 sn.**
  `test_closed_loop_reaches_goal` + `test_closed_loop_avoids_obstacle`
  bu makinede İLK KEZ koştu ve GEÇTİ (MPPI warm-start düzeltmesinin
  kapalı-döngü teli artık canlı). Jetson'da (gtsam+scipy+vision_msgs+
  mavros_msgs kurulu) skip'ler kendiliğinden açılır.

## Faz 17 — Doküman denetimi ✅ (CLAUDE.md 595 satır + README + docs/)

> Kapsam: CLAUDE.md, README.md, docs/*.md. `docs/KTR/` TESLİM EDİLMİŞ rapor
> (20.05) — tarihî belge, DOKUNULMAZ (50 Hz oradaki tablo zaten "~120 ms CPU
> → <20 ms CUDA hedefi" diye dürüst). `docs/jetson_deployment.md` Faz 2'de
> incelendi, sağlam. `docs/kod_denetimi.md` bu belge.

### 🔴 F17.1 — CLAUDE.md RAL kodları YANLIŞ (şartname md 5.5.2.1)

`CLAUDE.md:321-322`: "turuncu RAL 2008, sarı RAL 1003" — şartname: kenar
**RAL 2003**, engel **RAL 1026** (bulgular#4/F4.5'in doküman ayağı; HSV
değerleri tesadüfen gerçek RAL'lere uygundu, yalnız etiket yanlış). Bu
dosyayı okuyan her AI asistanı yanlış rengi "doğru" belleyecek. ✅ DÜZELTİLDİ.

### 🟠 F17.2 — CI iddiası boş (F16.4'ün doküman ayağı)

`CLAUDE.md:402` "CI: GitHub Actions … Push'ta otomatik" — `.github/` yok.
✅ DÜZELTİLDİ ("henüz kurulmadı" gerçeğine çevrildi).

### 🟠 F17.3 — "MPPI 50 Hz, GPU" bayat (gerçek: 10 Hz CPU)

`CLAUDE.md:125` (mimari diyagram) + `:185` ("20 ms döngü (50 Hz)") — gerçek:
`params.yaml control_rate_hz=10.0` (F4.2 düzeltmesi), CUDA portu YOK (CPU
~100 ms/iter). Video günü operatör CLAUDE.md'ye bakıp 50 Hz beklerse yanılır.
✅ DÜZELTİLDİ (10 Hz CPU bugünü + 50 Hz CUDA hedefi ayrıştırıldı).

### 🟡 F17.4 — Klasör şemasında `cpp/` hayaleti

`CLAUDE.md:382` — Layer 1 (C++) hiç yazılmadı (bilinçli: Python Layer 0→2
yeterli çıktı). ✅ DÜZELTİLDİ ("planlandı, yazılmadı" işaretlendi).

### 🟡 F17.5 — `configs/*.yaml` yolu yanlış

`CLAUDE.md:447` — gerçek yol `ros2_ws/src/girdap_decision/config/*.yaml`.
✅ DÜZELTİLDİ.

### 🟡 F17.6 — "Dosya 1a/1b" adlandırması şartnamede YOK

`CLAUDE.md` Çıktı Formatları tablosu 1a/1b kullanıyor; şartname md 4.2 tek
"Dosya 1" sayar (içinde kamera + diğer sensör alt maddeleri). İç adlandırma
zararsız ama takımlar arası konuşmada karışıklık yarattı ("1a olmayacakmış"
söylentisi). ✅ Dipnot eklendi.

### ⚪ F17.7 — GTSAM sürüm çelişkisi (F2.4'ün doküman ayağı)

`CLAUDE.md:17,142` "GTSAM 4.2" vs `requirements.txt:4` `gtsam>=4.3a0`.
✅ CLAUDE.md'ye requirements gerçeği not edildi. (Hangi sürümün Jetson'a
kurulacağı Faz 2 F2.4 kararına bağlı — kod lazy-import, video modunu
etkilemez.)

### ⚪ F17.8 — FSM şeması F12.2 sonrası eksik

`CLAUDE.md:207-208` TAMAMLANDI'ya tek yol "IMU ani ivme" gösteriyor; F12.2
`mission_complete` yolunu ekledi (video terminali). ✅ Şemaya not eklendi.
README test bölümüne ROS node testleri için source notu eklendi (F16
düzeltmeleri sonrası `pytest prototype/tests/` artık her ortamda güvenli).

## Sonraki fazlar ·
**DENETİM FAZLARI 1-17 TAMAM.** Kalan = Faz 18-T1 düzeltmeleri:
F12.1 (F16.5 — geri alındı) + F5.1 (mekanik `h` bekliyor) + F5.3 +
F6.3 sahneleri + F8.3 ölçümü + F16.6 rrt_star birim testleri + F16.4 CI.
(Tamamlananlar: Faz 1-17 + T0-a/b/c/d/f/g/i + video düzeltme paketi +
Faz 18-T1 ilk 11 düzeltme.)

**CUDA Faz 0 ✅ (2026-07-11):** mppi.py xp-backend soyutlaması (`9257fb4`) +
`scripts/bench_mppi.py` D3 ölçüm aracı (3 testli). x86 numpy ort ~99 ms/iter
(K=1000, T=50) — F4.2/D3'ün "~100 ms CPU" taban değeri bağımsız teyit edildi;
cupy ölçümü Jetson'da (plan: docs/mppi_cuda_plani.md §5).

**T0-j ✅ (2026-07-11) — md 3.3.1(3) başlatma boşluğu kapandı:** video günü
runbook'u yazılırken tespit edildi: başlatma yalnız ROS servisiydi, RC
dinleyen kod yok, WiFi yasak → YKİ'den başlatma İMKÂNSIZDI. Düzeltme
(Eyüp onayı, Seçenek A): `fsm_node` `start_on_mode` parametresi (varsayılan
GUIDED) — BEKLEMEDE'de operatör QGC'den modu GUIDED'a çevirince
`request_start()` (kenar tetikli; boot'ta-zaten-GUIDED başlatmaz, F14.3
sayesinde görev-öncesi GUIDED kesin operatör komutu). TDD: 4 yeni test
(çekirdek tetik kırmızı→yeşil + 3 guard), params.yaml kablolu, runbook §1
güncel. SAHA teyidi: masa testinde QGC→mod değişimi→log satırı.

**CUDA Faz A ✅ (2026-07-11, Jetson üzerinde):** cupy-cuda12x 13.6.0 kuruldu
(numpy 1.26.4 pini aynı komutta — altın kural), import + parite testi GEÇTİ,
CUDA context RAM ~50 MB.

- 🔴 **F-A.1 (düzeltildi, TDD):** `cupy.random.Generator`'da `.normal()` YOK
  (yalnız `standard_normal`) → `MPPIController._sample_noise` cupy
  backend'inde ilk GERÇEK koşuda `AttributeError` ile ölüyordu. Parite testi
  yakalayamadı çünkü tam da `_sample_noise`'u monkeypatch'liyor (Faz 6
  maskeleme deseninin yeni örneği). Düzeltme: iki backend'de de var olan
  `standard_normal(size) * sigma_u` ortak yolu — numpy'de `normal(0, σ)` ile
  bit-birebir (bu makinede doğrulandı, mevcut determinizm/bit-eşitlik
  testleri yeşil kaldı). Yeni test: `test_cupy_real_sample_noise_step`
  (monkeypatch'siz gerçek RNG yolu, GPU'suz makinede gerekçeli skip).
- 🟠 **F-A.2 (düzeltildi):** test_mppi.py'deki 11 white-box test
  (`_trajectory_cost`/`U_nominal`/`_apply_warmstart`'a doğrudan dokunanlar)
  `MPPIConfig`'i backend'siz kuruyordu → GPU'lu makinede `auto`→cupy seçince
  9'u "Implicit conversion to a NumPy array" ile kırıldı (0-skip taban
  yalnız CPU makinelerde doğrulanmıştı). Düzeltme: white-box matematik
  testleri `backend="numpy"`'a sabitlendi — amaçları backend seçimi değil.
  Host sözleşmesi (step giriş/çıkışı) testleri auto'da kalıyor, GPU'da da
  geçiyor.
- **Yeni suite tabanı: 246 passed / 2 gerekçeli skip** — GPU'lu makinede
  cupy'siz-fallback 2 testi, GPU'suz makinede parite + gerçek-RNG 2 testi
  skip olur; her iki makine tipinde de 246 geçer.
- 📊 **Benchmark (MAXN_SUPER, jetson_clocks kilitsiz):** numpy 208.9 ms
  (~4.8 Hz) · cupy 301.9 ms (~3.3 Hz). Ayrıştırma: maliyet GPU'da 18× hızlı
  (162→9 ms), rollout launch-overhead'e takıldı (34→266 ms) → **Faz B
  tetiklendi** (docs/mppi_cuda_plani.md). ⚠ `control_rate_hz: 10` Jetson
  CPU'sunda da tutmuyor — karar Faz B sonrasına.

**CUDA Faz B ✅ (2026-07-11, Jetson'da, Faz A ile aynı gün):** rollout tek
CUDA kernel'i (`_ROLLOUT_KERNEL_SRC` / `rollout_rk4`, RawKernel) —
`MPPIConfig.fused_rollout=True` varsayılan, yalnız cupy yolunda etkili,
derlenemezse WARN + jenerik yola sessiz düşüş. Fizik batch yoluyla
işlem-sırası düzeyinde aynı; TDD 3 test (fused≡generic 1e-4, fused≡numpy
1e-3, numpy bit-etkisiz). **Ölçüm: step 301.9→9.0 ms (~33×), tavan ~112 Hz,
20 ve 50 Hz kriterleri GEÇTİ; 600 adımda sürüklenme −1.0%.** Suite 249/2.
MPPI çekirdek matematiğine yine dokunulmadı — füzyon yalnız yürütme katmanı.

**LiDAR canlı test — aşama 1 ✅ (2026-07-12, Jetson + gerçek Livox Mid-360):**
veri akışı 10.00 Hz / ~20k nokta/mesaj / IMU 200 Hz (config 117.x senkron,
`rviz_MID360_launch` yerine düz node, xfer_format=0 → PointCloud2).

- 🔴 **F-L.1 (düzeltildi, TDD):** `perception_lidar_node._on_cloud`
  `read_points_numpy` kullanıyordu; gerçek Livox bulutu KARIŞIK dtype'lı
  (x/y/z/intensity float32 + tag/line uint8 + timestamp float64,
  point_step=26) ve `read_points_numpy` `field_names`'ten BAĞIMSIZ tüm
  alanların aynı tipte olmasını assert eder → node İLK gerçek mesajda
  `AssertionError` ile ölüyordu (`/perception/obstacle_map` hiç üretilmez —
  Parkur-2 biterdi). Testler yakalamadı çünkü hepsi `create_cloud_xyz32`
  kullanıyor (tüm alanlar float32) — Faz 6 maskeleme deseninin yeni örneği.
  Düzeltme: `read_points` (yapılandırılmış) + `structured_to_unstructured`.
  Yeni test: `test_real_livox_mixed_dtype_cloud_is_processed` —
  `_make_livox_cloud` üreteci gerçek sürücü şemasını birebir taklit eder
  (canlı mesajdan okunan düzenle). Kırmızı→yeşil kanıtlı.
  **Suite yeni taban: 250 passed / 2 gerekçeli skip.**
- 📊 **F5.3 Jetson teyidi:** `detect_obstacles` canlı 20k nokta/mesajda
  üretim config 38.6 ms / geniş-Z 52.3 ms medyan (x86 53.6 ms'e karşı) →
  10 Hz bütçesi RAHAT. Node canlıda 9.98 Hz `/perception/obstacle_map`
  yayınladı (frame=base_link, atölye engelleri makul).
- 🟠 **F-L.2 (AÇIK — F7.1'in ölçülmüş hali):** Livox mesaj damgası ile
  Jetson saati farkı sabit **+0.20 s** (medyan; 0.19-0.21 bandı) >
  `sync_slop_s=0.1` → kamera-LiDAR füzyon sync'i bu haliyle HİÇ ateşlemez
  (sync_watchdog WARN basar, F7.1 bekçisi). T1 işi (videoda füzyon yok).
  Karar seçenekleri: (a) node'da `now()` ile restamp (obstacle_map zaten
  base_link'e yeniden etiketleniyor), (b) `sync_slop_s`'i 0.3'e çek,
  (c) Livox saat senkronu (PTP/gPTP). Sahada duba testiyle birlikte karar.

**F8.3 ✅ KAPANDI (2026-07-12, Jetson ölçümü):** iSAM2 graf büyümesi 20 dk
yarışma simülasyonuyla ölçüldü (gerçek fusion hızları: IMU 50 Hz → flush
10 Hz + GPS 1 Hz, dairesel rota; 11.381 anahtar).
- **RAM: sorun DEĞİL** — maxRSS 61→91 MB (+30 MB / 20 dk); 8 GB kutuda ihmal.
- **Flush süresi LİNEER büyüyor:** medyan 0.3 ms (1. dk) → 12.7 ms (19. dk),
  p95 20.6 ms, maks 23.1 ms. 10 Hz bütçesi 100 ms → **20 dk görev sonunda
  bile ~4× pay var. Yarışma süresi için sliding-window/marginalization
  GEREKMİYOR** (CLAUDE.md "20 dk ~kabul edilebilir" öngörüsü sayıyla teyit).
- Sınır notu: büyüme lineer olduğundan >60 dk kesintisiz koşuda p95 bütçeyi
  zorlamaya başlar — yarışma dışı senaryo, önlem alınmadı (bilinçli).
- Doğruluk: smoother 20 dk sonunda dairesel rotayı doğru takip etti
  (son poz beklenen konumda). jetson_clocks kilitsiz ölçüldü; kilitliyse
  sayılar bir miktar daha iyi olur.

**F-L.2 REVİZE (2026-07-12 öğleden sonra — canlı deneyle):** "sync HİÇ
ateşlemez" teşhisi YANLIŞTI, deney çürüttü (erken-teşhis-geri-alma kuralı):
gerçek Livox + sahte 5 Hz buoys (stamp=now) ile sync **20 sn'de 90 çıkış**
verdi — obstacle_map 10 Hz yoğun akış olduğundan ApproximateTimeSynchronizer
+0.2 s ofsete rağmen slop içinde daima aday buluyor. **Gerçek etki:** çiftler
~0.2 s ZAMAN KAYMALI (kamera karesi T ↔ lidar taraması ≈T+0.2) + çıktı stamp'i
Livox tabanında. Tekne dinamiğinde bearing etkisi ~0.06 rad @0.3 rad/s dönüş
(< bearing_tol 0.15) → tolere edilebilir; karar (restamp en temizi) T1'de,
aciliyet DÜŞÜK.
**Eşleştirme matematiği CANLI KANITLANDI:** gerçek kümenin bearing'ine
(−0.312 rad, 1.5 m) nişanlanmış bbox → 99/99 mesajda class_id "0", log
"1 eşleşti"; eşleşmeyen ~100 küme class 99 ile korundu (güvenlik davranışı ✓).
Bearing işaret düzeltmesi (`e66cb40`) gerçek LiDAR verisiyle ilk kez doğrulandı.

**🔴 F-M.1 ✅ DÜZELTİLDİ (2026-07-12, PC, TDD):** masa OOM olayının (görev
başlar başlamaz `_trajectory_cost` 92 GB cupy alloc → planning ölümü) üç
katmanlı guard'ı — kök: fix'siz (0,0) "null island" konum + gerçek koordinatlı
görev = ~4400 km referans (0.5 m aralık → 8.8M nokta → (K,T+1,n_ref) tensörü).
1. `mission_manager_node._on_gps`: status=FIX görünse bile lat==lon==0.0
   konum GEÇERSİZ sayılır (ArduPilot'un fix'siz çıktısı) — cache'lenmez.
2. `_on_state` başlatma yolu: geçerli fix yoksa WARN + başlatmama (F8.4'ün
   kod karşılığı); `farthest_waypoint_m` (yeni Layer-0 fonksiyonu) >
   `max_target_distance_m` (yeni param, varsayılan 10 km, params.yaml
   kablolu) ise ERROR + görev REDDİ. İki yolda da `_started` latch'lenmez →
   düzeltilince yeniden denenebilir.
3. `MPPIConfig.max_ref_points=2048` + `set_reference` tavanı: aşan yol
   kabalaştırılır (uçlar korunur) + WARN — ValueError DEĞİL (F10.1 dersi:
   planlama yolunda hata fırlatmak node öldürür). Tavan, tam yarışma rotasını
   (≤1 km @ 0.5 m) kırpmaz; K=1000/T=50'de en kötü tensör ~0.4 GB.
TDD: 6 yeni test (2 çekirdek `farthest_waypoint_m` + 2 MPPI tavan/tavan-altı
+ 2 node guard'ı — null-island'la başlamama, 111 km hedefte red) ÖNCE kırmızı
koşuldu (mevcut kod "görev başlatıldı" bastı — masa hatası birebir), sonra
yeşil. Mevcut node testleri (0,0) fix kullanıyordu → gerçek koordinata (41°K/
29°D) güncellendi (davranış değiştiren düzeltmede test doğru davranışa çekilir).
**Suite: PC 254 passed / 4 skip (GPU'suz: 2 cupy + 2 fused skip); Jetson
beklenen 256/2.** Şartname bağı: md 3.3.1.1 istemsiz hareket (planning ölümü =
kontrolsüz araç) + md 5.5.2.2 görev yükleme sağlamlığı.

**🟠 F-M.2 ✅ DÜZELTİLDİ (2026-07-12, PC, TDD):** kasıtlı disarm'ın gerçek
FCU'da yine "FAILSAFE — beklenmedik disarm → KILL" üretmesi (masa günlüğü
§10) YARIŞ KOŞULU DEĞİL, deterministik latch hatasıymış:
`mavros_bridge_node._on_monitor` sonundaki `_was_armed = _was_armed or armed`
bir kez arm olunca sonsuza dek True kalıyor → disarm KENARI her monitor
tick'inde yeniden "görülüyor"; F14.2'nin `_expected_disarm` bayrağı TEK
ATIMLIK olduğundan ilk tick'te tükeniyor, İKİNCİ tick sahte KILL basıyordu.
Mock testler tek tick kontrol ettiği için yakalayamadı (maskeleme deseninin
zamansal örneği). Düzeltme: `_was_armed = armed` (önceki-tick kenar takibi).
TDD: yeni `test_mavros_bridge_node.py` (rclpy+mavros_msgs kapılı, F16.2
deseni) 3 test — (1) kasıtlı disarm sonrası tick'ler KILL üretmez (ÖNCE
kırmızı: masa logu birebir üredi), (2) regresyon bekçisi: komutsuz disarm
HÂLÂ KILL, (3) yeniden arm→komutsuz disarm kenarı yine yakalanır.
**Suite: PC 257 passed / 4 skip.** Şartname bağı: md 3.3.1(4) video güç-kesme
gösterimi — çekim anında sahte FAILSAFE/KILL logu artık basılmaz. SAHA teyidi:
M6'da gerçek FCU'yla kasıtlı disarm→temiz log akışı doğrulanacak.

## F-V.1 🔴 yon_setpoint yaw HIZI değil AÇI olmalı ✅ DÜZELTİLDİ (2026-07-13, video derin denetimi)

- **Şart:** md 3.3.1.1 Ekran-2b — "Gerçek heading/yaw açısı, heading/yaw **açısı**
  isteği (setpoint)" (2026-07-13'te birinci kaynaktan yeniden okundu, s.12).
  md 4.2 Dosya-2 `yon_setpoint` alanı da aynı anlamı taşır.
- **Bulgu:** `telemetry_node._on_setpoint` yon_setpoint'e cmd_vel `angular.z`
  yazıyordu = `planning_node`'un yaw HIZI komutu (rad/s, `(u_r−u_l)/I_z`).
  `ekran2.py` bu değeri açı sanıp dereceye çevirip heading paneline çiziyordu →
  panelde heading (°) ile hız (rad/s→°) karışık = grafik yanıltıcı, video
  değerlendirme riski; Dosya-2 alanı da md 4.2'ye aykırı.
- **Düzeltme:** `/girdap/mission/current_target` (PoseStamped, araç-göreli ENU
  ofset) aboneliği eklendi; `yon_setpoint = atan2(y, x)` — heading ile aynı ENU
  açı konvansiyonu, bypass'ta kontrolcünün fiilen gitmeye çalıştığı yön.
  `_on_setpoint` yalnız `hiz_setpoint`'i günceller. `ekran2.py` DEĞİŞMEDİ
  (zaten açı bekliyordu).
- **Doğrulama (TDD):** `test_yon_setpoint_is_angle_from_current_target` —
  PoseStamped(3,4) → grafik CSV `yon_setpoint == "0.927"` (atan2(4,3)); cmd_vel
  `angular.z=0.5` yayınlanırken değerin 0.500 OLMADIĞI da asserte edildi.
  Önce kırmızı (0.500 geldi), düzeltme sonrası yeşil. TAM suite: **258 passed /
  4 skipped** (yeni taban).

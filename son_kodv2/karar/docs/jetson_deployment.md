# Jetson Deployment Guide — Girdap İDA Karar Yığını

> **Kime:** Sahada Jetson'a deploy yapan ekip (hand-off).
> **Hedef donanım:** NVIDIA Jetson Orin Nano 8GB Super — JetPack 6 (Ubuntu 22.04) + ROS 2 Humble.
> **İlgili dokümanlar:** [`bench_mavlink_runbook.md`](bench_mavlink_runbook.md), [`ardurover_bench.parm.md`](ardurover_bench.parm.md)

---

## İKİ PYTHON BAĞLAMI (önce bunu anla)

| Bağlam | Ne için | Nasıl |
|---|---|---|
| **`.venv`** (proje kökü) | Layer-0 prototip **testleri** (`pytest`) + KTR görselleştirme. ROS node **çalıştırmaz**. | `python3 -m venv .venv` + `requirements.txt` |
| **Sistem python3.10 + ROS Humble** | **Runtime ROS node'ları** (rclpy). Yarışma modunda `gtsam` **buraya** kurulur. | `apt` ROS + `pip install --user gtsam` |

> 🔑 **Kritik:** ROS node'ları `.venv`'de değil, **sistem python3.10**'da koşar
> (rclpy Humble ile derli). `gtsam` yarışma modunda (`use_isam2=true`) rclpy ile
> **aynı yorumlayıcıda** olmalı → sisteme kurulur. **Video modunda
> (`use_isam2=false`) gtsam gerekmez.**

---

## ÖN KOŞULLAR

- [ ] JetPack 6 flash'lı (Ubuntu 22.04, CUDA 12)
- [ ] ROS 2 Humble kurulu (`ros-humble-desktop` veya `ros-humble-ros-base`)
- [ ] `ros-humble-mavros` + `ros-humble-mavros-extras`
- [ ] `ros-humble-vision-msgs` (perception `/perception/buoys` mesajı;
      desktop'ta var, ros-base'de ayrıca kur)
- [ ] GeographicLib veri seti:
      ```bash
      sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
      ```
- [ ] Git + ağ (ilk clone için; saha WiFi'siz → ilk kurulum ofiste yapılır)
- [ ] Kullanıcı `dialout` grubunda (Pixhawk seri erişimi):
      `sudo usermod -a -G dialout $USER` (logout/login şart)

---

## ADIM 1 — SSH ile İlk Clone

Geliştirme makinesinden Jetson'a bağlan (aynı ağ / Ethernet):

```bash
ssh usv@<jetson-ip>          # örn. ssh usv@192.168.1.50
```

Depoyu klonla (ev dizinine):

```bash
cd ~
git clone https://github.com/vistastris/girdap-decision.git
cd girdap-decision
```

> **Not:** VSCode **Remote-SSH** ile bağlanıp `~/girdap-decision`'ı açman
> önerilir (CLAUDE.md geliştirme akışı) — kod Jetson'da yaşar, düzenleme
> geliştirme makinesinden.

---

## ADIM 2 — Python venv (Layer-0 Test Katmanı)

```bash
cd ~/girdap-decision
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate
```

> `requirements.txt`: numpy, scipy, matplotlib, **gtsam>=4.3a0**, **pillow**,
> pytest, pyyaml, tqdm, black, ruff, mypy. JetPack 6 python3.10 ile gtsam
> `cp310` wheel'i uyumludur.

**Doğrula:**
```bash
PYTHONPATH= .venv/bin/python -m pytest prototype/tests/ -q
# beklenen: 98 passed, 1 skipped (node testi — ROS ortamında ayrıca koşar)
```

> `PYTHONPATH=` öneki önemli: bashrc ROS ortamını source'luyorsa py3.10 ament
> pytest plugin'leri py3.11 venv'e sızar ve test koleksiyonunu bozar.

---

## ADIM 3 — Runtime Bağımlılığı (Yarışma Modu, gtsam)

ROS node'ları sistem python3.10'da koşar. **Yarışma modunda** (`use_isam2=true`)
`fusion_node` gtsam'a muhtaç → sisteme kur:

```bash
python3 -m pip install --user --pre gtsam
python3 -c "import gtsam; print('gtsam OK')"      # doğrula
```

> **Video modunda gerekmez** — `use_isam2=false` iken `fusion_node` GTSAM'ı hiç
> yüklemez (MAVROS EKF pass-through). Video günü bu adım atlanabilir.

**scipy (perception, HER MODDA gerekli):** `perception_lidar_node` cKDTree
clustering için scipy kullanır. pip numpy 2.x kuruluysa apt scipy'si ABI
uyumsuzluğuyla çöker (`numpy.dtype size changed`) → pip sürümünü kur:

```bash
python3 -m pip install --user --upgrade --ignore-installed scipy
python3 -c "from scipy.spatial import cKDTree; print('scipy OK')"   # doğrula
```

**opencv (perception kamera, HER MODDA gerekli):** apt `python3-opencv` aynı
ABI tuzağına düşer; pip headless sürümünü kur. `cv_bridge` KULLANILMIYOR
(aynı ABI sorunu — yerine `girdap_decision/image_codec.py`):

```bash
python3 -m pip install --user --upgrade --ignore-installed opencv-python-headless
python3 -c "import cv2; print('cv2', cv2.__version__)"              # doğrula
```

---

## ADIM 4 — colcon build

```bash
source /opt/ros/humble/setup.bash
cd ~/girdap-decision/ros2_ws
colcon build --packages-select girdap_decision
```

> Build **build/**, **install/**, **log/** üretir (git-ignore'lu). Temiz
> bitmeli: `Summary: 1 package finished`.

---

## ADIM 5 — Ortam Source + Doğrulama

Her yeni terminalde (ve `~/.bashrc`'ye eklenmesi önerilir):

```bash
source /opt/ros/humble/setup.bash
source ~/girdap-decision/ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/girdap-decision:$PYTHONPATH   # prototype/ runtime importları
```

**Doğrula:**
```bash
ros2 launch girdap_decision hardware.launch.py --show-args
# beklenen: use_isam2/use_rrt/use_mppi + fcu_url argümanları, hata yok
```

> `~/.bashrc`'ye üç `source/export` satırını ekle → her terminal hazır gelir.

---

## GÜNCELLEME PROSEDÜRÜ

Kodda değişiklik geldiğinde (ofiste, ağ varken):

```bash
cd ~/girdap-decision
git pull origin main

# requirements değiştiyse:
.venv/bin/pip install -r requirements.txt

# her zaman yeniden derle:
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select girdap_decision

# hızlı sağlık kontrolü:
cd ~/girdap-decision && .venv/bin/python -m pytest prototype/tests/ -q
```

> ⚠️ `git pull` sonrası **colcon build ŞART** — install/ eski kalırsa node'lar
> eski kodu koşar. Config (`params.yaml`, `hardware.yaml`, `video_mission.yaml`)
> değişiklikleri de rebuild ile share'e kopyalanır.

---

## AĞ İZOLASYONU — geliştirme ↔ yarışma (şartname md 4.1)

> ## 🔴 md 4.1 FREKANS/WiFi DURUMU — 2026-08-07 ölçümü
>
> Şartname md 4.1: *"Tüm bilgisayarlarda dahili WiFi kapalı"* + 2.4-2.8 GHz ve
> 5.15-5.85 GHz **yasak**.
>
> **Jetson: ✅ yapısal olarak uygun — hiç telsiz donanımı YOK.**
> ```
> rfkill list        → boş (hiç telsiz cihaz yok)
> Bluetooth adaptörü → yok    (bluetooth.service çalışıyor ama adaptör yok, yayın yapmıyor)
> PCI kablosuz kart  → yok    (M.2 WiFi/BT kartı takılı değil)
> arayüzler          → can0 enP8p1s0 l4tbr0 lo usb0 usb1  (kablosuz arayüz YOK)
> ```
> `nmcli radio` "WIFI enabled" der ama bu NetworkManager'ın yazılım bayrağı,
> arkasında donanım yok. Kapatacak bir şey yok.
>
> **🔴 LAPTOP (YKİ): AÇIK — yarışmada KAPATILMALI.**
> Ölçüm sırasında laptopun `wlo1` arayüzü bir WiFi ağına bağlıydı. Laptop
> yarışmada YKİ olarak kullanılacağı için md 4.1 kapsamındadır.
> ```bash
> nmcli radio wifi off        # yarışma sabahı
> nmcli radio wifi on         # sonra geri
> ```
> ⚠️ Bu adım daha önce **hiçbir dokümanda yoktu**; md 4.1 doğrudan ihlal
> edilebilirdi. Kontrol listesine eklendi.

İki ayrı mod var; **karıştırma**:

| | Geliştirme / masa testi | Yarışma / video günü |
|---|---|---|
| `ROS_DOMAIN_ID` | 42 (serviste + bashrc) | 42 (aynı) |
| `ROS_LOCALHOST_ONLY` | **yok** (laptop'tan `ros2 topic echo` çalışsın) | **1** (drop-in ile) |
| ROS trafiği | ethernet'e çıkar | Jetson içinde kalır |

**Yarışma günü kurulumu** — iki yol:

*(a) Repo Jetson'da varsa:*
```bash
sudo mkdir -p /etc/systemd/system/girdap-karar.service.d
sudo cp <REPO_KOKU>/karar/scripts/girdap-karar-yarisma.conf \
        /etc/systemd/system/girdap-karar.service.d/
sudo systemctl daemon-reload && sudo systemctl restart girdap-karar
```
> 🔴 **`<REPO_KOKU>` sabit yazılMIYOR, çünkü Jetson'da değişebiliyor.**
> 2026-08-07'de ölçüldü: Jetson'daki tek repo `EyupEker1/girdap-video`
> (`~/girdap-video`), **son_kodv2 orada YOK** — algı katmanı Eyüp'te olduğu ve
> son_kodv2 henüz son hâline gelmediği için. Yani "repodan kopyala" o Jetson'da
> **işlemez**. Kurulumdan önce `find ~ -name girdap-karar-yarisma.conf` ile yeri
> doğrula; yoksa (b)'yi kullan.

*(b) Repo yoksa — dosyayı doğrudan yaz (2 satır, bağımlılık yok):*
```bash
sudo mkdir -p /etc/systemd/system/girdap-karar.service.d
sudo tee /etc/systemd/system/girdap-karar.service.d/girdap-karar-yarisma.conf >/dev/null <<'EOF'
[Service]
Environment=ROS_LOCALHOST_ONLY=1
EOF
sudo systemctl daemon-reload && sudo systemctl restart girdap-karar
```

**Doğrulama:**
```bash
# 1) Bayrak systemd'ye geçti mi
systemctl show girdap-karar -p Environment | grep LOCALHOST     # =1 görmeli

# 2) YIĞIN SAĞLIKLI MI — Jetson'da (aynı bayrakla!)
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
for t in /mavros/state /mavros/imu/data /girdap/fusion/odom \
         /girdap/control/thrust /girdap/mission/state; do
  echo -n "$t "; timeout 10 ros2 topic hz $t 2>/dev/null | grep -m1 "average rate"
done

# 3) İZOLASYON — laptop'tan (bayrak OLMADAN)
ros2 topic list | grep -cE "girdap|mavros"                      # 0 görmeli
```

> 🔴 **`ros2 topic list` ile yığın sağlığını KONTROL ETME — YANLIŞ SONUÇ VERİR.**
> Bu dokümanın önceki hâli *"`ros2 topic list` → Jetson'da DOLU"* diyordu.
> **2026-08-07'de ölçüldü:** bayrak açıkken yığın tamamen sağlıklıyken
> (5 kritik topic'in hepsi akıyor) Jetson'da `ros2 topic list` yalnız **2**,
> `ros2 node list` **0** gösterdi. Sebep: 72 katılımcı varken yeni başlayan
> kısa ömürlü bir CLI süreci graf keşfini tamamlayamıyor.
> Bu adımı uygulayan operatör **"sistem çökmüş" diye yanlış alarma kapılır.**
> Sağlık için **`ros2 topic hz`** kullan — güvenilir çalıştı.
>
> ⚠️ Ayrıca Jetson'da ölçüm yaparken kabuğunda da `ROS_LOCALHOST_ONLY=1`
> **olmalı**: bayraklı yığın ile bayraksız CLI birbirini göremez.

### ✅ ÖLÇÜLDÜ — 2026-08-07 (A/B/A, gerçek Jetson)

Jetson `192.168.117.60`, laptop `192.168.117.50`, doğrudan ethernet, `DOMAIN_ID=42`
her iki tarafta. Yığın canlı (72 node, MAVROS FC'ye bağlı).

| Laptop'tan görünen | Drop-in YOK | **Drop-in VAR** | Drop-in kaldırıldı |
|---|---|---|---|
| `girdap`/`mavros` topic'i | 150 | **0** | 150 |
| node | 72 | **0** | 72 |

→ **Sızıntı gerçek, izolasyon çalışıyor**, kaynağı kesin olarak bayrak (A/B/A).

Bayrak açıkken yığın sağlığı (Jetson'da, aynı bayrakla ölçüldü):

| Topic | Hz |
|---|---|
| `/mavros/state` | 1.001 |
| `/mavros/imu/data` | 7.386 ← MAVROS FC'den veri alıyor, **seri hat etkilenmiyor** |
| `/girdap/fusion/odom` | 9.998 |
| `/girdap/control/thrust` | 10.005 |
| `/girdap/mission/state` | 10.026 |

→ **Bayrak yığını BOZMUYOR.** Tahmin doğruydu: MAVROS seri porttan, Livox ham
UDP soketinden konuşuyor, DDS izolasyonu ikisini de etkilemiyor.

> 🔎 **SAHA TUZAĞI (2026-08-07'de yaşandı):** Jetson'ın ethernet arayüzünde
> **IP yoktu** ve yığın öyle başlamıştı. DDS ağ arayüzlerini **katılımcı
> oluşturulurken** bağlar; sonradan `ip addr add` ile eklenen adresi görmez.
> Sonuç: "topic'ler görünmüyor" → izolasyon sanılabilir, oysa DDS o arayüzü hiç
> tanımıyordur. **Ölçümden önce IP'nin var olduğunu ve yığının ONDAN SONRA
> başladığını doğrula** (`systemctl restart girdap-karar`).

**Neden:**
- md 4.1 *"YKİ'lerde görüntü işleme, sensör işleme ya da otonomi kabiliyeti
  olmayacaktır"* — ROS topic'leri yer tarafından erişilemez olunca bu yapısal
  olarak garanti altına alınır. Görev telemetrisi ayrı hattan (MAVLink/RFD868 →
  Mission Planner) gittiği için etkilenmez.
- Livox Mid-360 aynı ethernet üzerinde 10 Hz'te ~20k nokta basıyor; DDS keşif
  multicast'ini o hattan çekmek LiDAR trafiğini rahatlatır.

> ⚠️ Yarışma sonrası **geri al** — açık kalırsa bir sonraki masa testinde
> "servisi göremiyorum" diye saatler kaybedilir (2026-07-13'te DOMAIN_ID
> yüzünden tam bu yaşandı).

---

## SU GÜNÜ — 3 TERMİNAL LAUNCH SIRASI

> Her terminalde önce ADIM 5'teki üç `source/export` satırı (bashrc'de yoksa).
> **Sıra önemlidir.** Her adımda beklenen çıktıyı gör, sonra ilerle.
> 🔴 Yarışma günüyse önce yukarıdaki **AĞ İZOLASYONU** drop-in'ini kur.

### Terminal 1 — Sensör Sürücüleri (donanım ekibi)
```bash
# Livox Mid-360 + OAK-D Lite sürücüleri (arkadaşın launch'ı)
ros2 launch <sensor_bringup> sensors.launch.py
```
**Beklenen:** `/livox/lidar`, `/oak/rgb/image_raw` yayında.
> Sensörler önce ısınsın; Dosya-1a/1b (kamera/LiDAR video) bunlara bağlı.

### Terminal 2 — Karar Yığını + MAVROS
```bash
ros2 launch girdap_decision hardware.launch.py
# yarışma modu için:  ros2 launch girdap_decision hardware.launch.py use_isam2:=true use_rrt:=true
```
**Beklenen loglar:**
```
[mavros_bridge] mavros_bridge aktif (heartbeat=5.0s, hedef mod=GUIDED, ...)
[mission_manager_node] ... 5 waypoint, arrival=2.0 m, dwell=2.0 s
[fusion_node] fusion_node aktif [...]
[planning_node] planning_node aktif [...]
```
> MAVROS bu launch'ın **içinde** başlar (apm.launch include). Ayrı MAVROS başlatma.

### Terminal 3 — Operatör Kontrolü (arm + görev başlat + monitör)
```bash
# 1) FCU bağlantısı ve mod:
ros2 topic echo /mavros/state --once           # connected=true, mode görünür

# 2) ARM (retry'li köprü servisi):
ros2 service call /girdap/bridge/arm std_srvs/srv/Trigger {}
ros2 topic echo /mavros/state --once           # armed=true doğrula

# 3) GÖREVİ BAŞLAT (FSM → PARKUR1 → mission_manager ACTIVE):
ros2 service call /girdap/mission/start std_srvs/srv/Trigger {}

# 4) Canlı izleme:
ros2 topic echo /girdap/mission/state          # PARKUR geçişleri
ros2 topic hz /mavros/setpoint_velocity/cmd_vel_unstamped   # ~20 Hz kontrol
```

> 🔴 **ACİL DURDURMA her an:** fiziksel kill switch **veya**
> `ros2 service call /girdap/mission/kill std_srvs/srv/Trigger {}`
> → FSM KILL → planning sıfır thrust → motor stop.

### Görev sonrası — Çıktı Teslimi (Şartname 4.2, 20 dk içinde)
- **Dosya-2** (telemetri CSV): `~/girdap-decision/data/telemetry/telemetri_*.csv`
- **Dosya-3** (yerel harita PNG): `~/girdap_logs/local_map/session_*/frame_*.png`
- Dosya-1a/1b (kamera/LiDAR video): sensör ekibinden.

---

## SORUN GİDERME

| Belirti | Kontrol |
|---|---|
| `No module named 'prototype'` | `PYTHONPATH`'e repo kökü eklendi mi (ADIM 5)? |
| `No module named 'gtsam'` (yarışma) | ADIM 3 sisteme kurulum yapıldı mı? Video modunda gtsam gerekmez. |
| Node eski davranıyor | `git pull` sonrası `colcon build` yapıldı mı? |
| `/mavros/state connected=false` | Runbook ADIM 1 (dmesg, dialout, fcu_url). |
| QoS uyarısı / mesaj gelmiyor | sensör topic'leri SensorDataQoS (BEST_EFFORT) — normalde tek kaynaktan. |
| Orphan node (çift yayım) | `pkill -9 -f "lib/girdap_decision"` ile temizle, tek launch koştur. |

> Takılırsan **adım numarasıyla** karar/algoritma sorumlusuna bildir; suya inme
> kararı [`ardurover_bench.parm.md`](ardurover_bench.parm.md) checklist'i tamamlanmadan alınmaz.

# GİRDAP İDA — Algı & Görev Yazılımı

TEKNOFEST 2026 İnsansız Deniz Aracı (İDA) yarışması için Team GİRDAP'ın
duba algılama ve geçit görev mantığı. YOLOv11n, OAK-D Lite kameranın
**içinde** (Myriad X VPU) çalışır; Jetson tarafındaki node geçit seçer,
sayar ve seçilen moda göre çıkış üretir.

**Algı gerçekleri (saha ölçümü):**
- Model: YOLOv11n @ **416×416** (NN Archive içinde tanımlı, RVC2, 6 shave)
- Tespit hızı: **10–14 FPS bandı, tipik ~11.6** (YOLO + stereo birlikte, VPU sınırı)
- API: **DepthAI v3** (3.7.1'e karşı doğrulandı; v2 kodu bu repoda ÇALIŞMAZ)
- Node çalışırken ölçülen FPS loglanır, 8'in altında uyarı basılır

## Mimari

Karar/sürüş tarafı takım arkadaşının
[girdap-decision](https://github.com/vistastris/girdap-decision) reposunda
(kendi RRT* + MPPI + iSAM2 + FSM yığını — **Nav2 değil**). Bu repo onun
perception sözleşmesini besler.

```
OAK-D Lite (VPU: YOLO11n 416x416 + StereoDepth, 10-14 FPS)
        │  tespitler (sınıf + bbox + X/Y/Z)
        ▼
duba_gecis_navigator ─► MOD="algi_yayin" (PLAN A, varsayılan):
 (geçit seç + say)        /perception/buoys        Detection2DArray (sözleşme)
                          /perception/gate_passed  Bool (odom-doğrulamalı geçiş)
                          /perception/buoys_3d     PoseArray (bonus: stereo 3D)
                          → girdap-decision füzyon/planlama/FSM sürer
                     ─► MOD="mppi_hedef"     : /goal_pose (Nav2 arşiv modu)
                     ─► MOD="dogrudan_surus" : /mavros/.../cmd_vel (Plan B, yedek)
```

| MOD | Ne yapar | Durum |
|---|---|---|
| `algi_yayin` | girdap-decision sözleşme topic'leri; komut basmaz | **Plan A (varsayılan)** |
| `mppi_hedef` | Nav2'ye `/goal_pose` | Arşiv — ancak Nav2 kurulursa |
| `dogrudan_surus` | Dümeni kendisi tutar (MAVROS) | Plan B saha yedeği |

- **Sözleşme detayı, kurulum, test merdiveni, sorun giderme:**
  [`docs/mppi_entegrasyon_notu.md`](docs/mppi_entegrasyon_notu.md)
- **TEK DÜMEN KURALI:** Hız komutu tek kaynaktan — `algi_yayin`/`mppi_hedef`
  asla komut basmaz; Plan B çalışırken girdap-decision planning_node kapalı.
- `algi_yayin` gereksinimi: `sudo apt install ros-humble-vision-msgs` +
  girdap-decision stack'inin `/girdap/fusion/odom` yayını.

## Depo yapısı

```
girdap_ida_algi/   ROS 2 (ament_python) paketi — ana görev node'u + launch
scripts/           duba_kamera_test.py   — ROS'suz masa testi (kutu çizer)
                   jetson_kur.sh         — sıfır Jetson'a kurulum (+ --servis)
                   jetson_kontrol.sh     — ortam denetimi (sürümler, OAK, model)
                   girdap-algi.service   — açılışta otomatik başlatma (systemd)
docs/              girdap-decision entegrasyon notu (sözleşme + test planı)
models/            NN Archive dosyaları (git dışı, HubAI'den indirilir)
```

## Kurulum (Jetson Orin Nano, JetPack 6.2 / ROS 2 Humble)

**Hedef yığın (sürüm kilidi — takım genelinde aynı):**

| Katman | Sürüm | Neden |
|---|---|---|
| JetPack | 6.x (Ubuntu 22.04, CUDA 12) | Orin Nano resmi hattı |
| Python | 3.10 (sistem) | ROS Humble buna derli |
| ROS 2 | **Humble** (Jazzy'ye geçme) | girdap-decision de Humble; TYF raporuyla tutarlı |
| numpy | **1.26.4** (`>=1.26,<2`) | 2.x, apt scipy/matplotlib/ROS ABI'sini kırar (`_ARRAY_API not found`) |
| depthai | **>=3.6** (v3 API) | v2 kodu bu repoda çalışmaz |
| vision-msgs | apt `ros-humble-vision-msgs` | `algi_yayin` sözleşme mesajı |

**Tek komutla:** `bash scripts/jetson_kur.sh` — apt+pip bağımlılıkları (sürüm
kilitli), OAK udev kuralı, iki reponun klonu ve colcon derlemesi. `--servis`
bayrağıyla açılışta otomatik başlatma da kurulur. Ardından doğrula:
`bash scripts/jetson_kontrol.sh` (çıktıyı paylaşılabilir PASS/FAIL listesi basar).

El ile kurulum:

```bash
# Bağımlılıklar
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-vision-msgs
pip install "depthai>=3.6" "numpy>=1.26,<2" --break-system-packages   # DepthAI v3 API şart

# Derleme
cd ~/ros2_ws/src && git clone <REPO_URL>
cd ~/ros2_ws && colcon build --packages-select girdap_ida_algi
source install/setup.bash
```

## Kullanım

```bash
# 1) Masa testi (ROS'suz) — sınıf/mesafe/FPS doğrulama
python3 scripts/duba_kamera_test.py

# 2) Görev node'u (MOD bayrağını dosya başından seç)
ros2 launch girdap_ida_algi algi.launch.py      # respawn'lı (önerilen)
#   veya: ros2 run girdap_ida_algi duba_gecis_navigator
#   veya paket kurmadan: python3 girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py

# 3) Açılışta otomatik başlatma (yarışma günü)
bash scripts/jetson_kur.sh --servis
journalctl -fu girdap-algi                       # canlı log
sudo systemctl stop girdap-algi                  # elle müdahale / Plan B'ye geçiş
```

Plan B için MAVROS hazırlığı:

```bash
ros2 param set /mavros setpoint_velocity.mav_frame BODY_NED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

## Suya inmeden kontrol listesi

- [ ] `MODEL_NNARCHIVE` yolu gerçek tar.xz'yi gösteriyor
- [ ] `KENAR_CLASS` / `ENGEL_CLASS` data.yaml sırasıyla doğrulandı (masa testi)
- [ ] Kamera orta hatta, suya ~paralel monte; `KAMERA_KIC_MESAFE` ve
      `KAMERA_OFSET_ILERI` gerçek ölçülerle güncellendi
- [ ] Plan B ilk denemede `YAW_ISARET` yönü doğrulandı
- [ ] Plan A için: girdap-decision stack'i açık, `/girdap/fusion/odom` yayında,
      `ros2 topic echo /perception/buoys --once` sözleşmeye uygun
- [ ] Letterbox dikey düzeltmesi masa testinde doğrulandı (`_LB_PAY`, bkz. not §2)

## Araç

Katamaran, 1050 × 750 mm. Görev hızı 1 m/s. Dubalar: turuncu RAL 2003
(parkur kenarı) ve sarı RAL 1026 (engel), 30 cm çap / 50 cm yükseklik.

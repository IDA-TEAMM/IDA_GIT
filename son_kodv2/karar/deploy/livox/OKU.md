# Livox MID-360 — GİRDAP'a özel düzeltmeler (kalıcı kopya)

> Bu dizin **livox_ros_driver2 üçüncü parti deposunun** iki dosyasının
> GİRDAP sürümünü tutar. Amaç: bu iki düzeltme sürücü deposu yeniden
> kurulduğunda / `git checkout` yendiğinde / Jetson yeniden kurulduğunda
> **kaybolmasın**.

## Neden ayrı bir kopya gerekiyor

Düzeltmeler `~/livox_ws/src/livox_ros_driver2/` altında yaşıyor ve
**12.08.2026 itibarıyla o depoda COMMIT EDİLMEMİŞ** durumdaydılar
(`git status` → `M config/MID360_config.json`, `M launch_ROS2/msg_MID360_launch.py`).
Yani tek bir `git checkout .` ya da `git stash` ikisini birden siler ve
LiDAR **hiçbir hata vermeden** susar.

Üstelik ikisi de daha önce **yalnız `install/` kopyasında** düzeltilmişti;
bir `colcon build` arızayı geri getiriyordu. `src/` tarafı 11.08'de
düzeltildi (kaptan §0.40 "livox src mayını temizlendi"), ama repoya hiç
girmedi.

## İki düzeltme

### 1 · `MID360_config.json` — ağ adresleri

Fabrika varsayılanı `192.168.1.x`. Bu ekipte **doğrulanmış gerçek değerler**:

| | fabrika | GİRDAP |
|---|---|---|
| LiDAR | `192.168.1.12` | **`192.168.117.100`** |
| Host (Jetson) | `192.168.1.5` | **`192.168.117.60`** |

⚠ Jetson'ın ethernet arayüzü de aynı alt ağda statik IP olmalı
(`192.168.117.60/24`, `nmcli` ile kalıcı yapılandırıldı).

### 2 · `msg_MID360_launch.py` — `xfer_format` **1 → 0**

`perception_lidar_node` `sensor_msgs/PointCloud2` dinliyor. `1` (CustomMsg)
kalırsa topic *"contains more than one type: [CustomMsg, PointCloud2]"* olur
ve `/perception/obstacle_map` **tamamen boş** kalır — **hiçbir hata mesajı
üretmeden**. Parkur-2 engel kaçınması sessizce çöker.

## Geri yükleme / doğrulama

```bash
# geri yükle (sürücü deposu yeniden kurulduysa)
cp deploy/livox/MID360_config.json      ~/livox_ws/src/livox_ros_driver2/config/
cp deploy/livox/msg_MID360_launch.py    ~/livox_ws/src/livox_ros_driver2/launch_ROS2/
cd ~/livox_ws && colcon build --packages-select livox_ros_driver2

# DOĞRULA — dosya değil, ETKİN sonuç kanıttır (kaptanın §0.43 kuralı)
grep -c 192.168.117 ~/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json   # 0 OLMAMALI
grep "^xfer_format" ~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py                        # = 0
ros2 topic hz /livox/lidar        # ~10 Hz
ros2 topic hz /perception/obstacle_map
```

🔑 `systemctl is-active girdap-livox` **sağlık kanıtı DEĞİLDİR** — kaptan
§0.40'ta ölçtü: sürücü açılışta `bind failed` ile ölmüştü, servis yine
`active` görünüyordu ve `NRestarts=0` idi. Tek kanıt `topic hz`.

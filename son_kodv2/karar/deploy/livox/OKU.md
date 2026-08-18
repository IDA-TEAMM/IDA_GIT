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

---

## 3 · 🔴 `src/` C++ YAMASI — IMU dağıtıcısı boş kuyruğu SPIN ediyordu

> **17.08.2026.** Yukarıdaki iki düzeltmeden farklı: bu bir **config değil,
> KAYNAK KODU** yaması. Bu yüzden yanında `.patch` dosyası duruyor.

**Yama:** `0001-imu-semafor-spin-duzeltmesi.patch` (28 ekleme, 1 silme,
3 dosya: `src/lds.h`, `src/lds.cpp`, `src/lddc.cpp`)

### Ne bulundu

`Lddc::DistributeImuData` **semaforsuz** `while(true)` dönüyordu. Nokta
bulutu yolunda `lds_->semaphore_.Wait()` var, IMU yolunda **yoktu**:
kuyruk boş → anında dön → tekrar sor.

Ölçüm (Jetson Orin Nano, MAXN_SUPER, `gdb` yığın izi):
```
tek thread durum 'R', wchan=0, %97 ÇEKİRDEK KESİNTİSİZ
yığın: PollingLidarImuData -> LidarImuDataQueue::Empty() -> mutex spin
/livox/imu abone sayısı: 0        ← IMU'muz MAVROS'tan geliyor
```

🔑 **Bu yukarı akışın ZATEN DÜZELTTİĞİ bir hata** — bizim vendored kopyamız
düzeltmeden önceki sürüm. Upstream `master`'da `lds_->imu_semaphore_.Wait();`
satırı var. Yama o çözümün birebir geri portu.

### Neden CPU değil PUAN meselesi

Makine doyunca `planning_node`'un odom callback'i **aç kalıyor**, F-P.1
bekçisi *"poz 1,0 s'dir gelmiyor"* deyip **thrust'ı SIFIRLIYOR** — poz
kaynağı 8 Hz akarken, EKF sağlıklıyken, eşik 1,0 s iken. Bekçinin mesajı
operatörü **sağlıklı olan** poz kaynağına yönlendiriyor.

| | önce | sonra |
|---|---|---|
| GIRDAP CPU | %74,3 (446%/600%) | **%55,6** (333%) |
| yük ortalaması | 8,4 | **4,9** |
| POZ-BAYAT | 34 / 60 sn | **7 / 60 sn** |
| `/livox/lidar` · `/livox/imu` | 10 Hz · 200 Hz | **10 Hz · 200 Hz** (bozulmadı) |

### Ek: kapanma asılması da kapatıldı

`Lds::RequestExit()` yalnız bayrak koyuyordu. Tüketici thread'ler artık
semaforda **BLOKE** beklediği için, LiDAR susmuşsa (kablo çıktı, cihaz
kapandı) sinyal hiç gelmez ve `join()` **sonsuza kadar asılır** — servis
durdurulamaz. `RequestExit()` artık iki semaforu da uyandırıyor.
**Upstream bunu yapmıyor**, bizim eklememiz.

### Yeniden kurulumda NE YAPILACAK (pazarlıksız)

```bash
cd ~/livox_ws/src/livox_ros_driver2
git am < ~/IDA_GIT/son_kodv2/karar/deploy/livox/0001-imu-semafor-spin-duzeltmesi.patch
#   (git am tutmazsa:  git apply --3way <aynı dosya>)
cd ~/livox_ws && colcon build --packages-select livox_ros_driver2 \
    --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
sudo systemctl restart girdap-livox
```

**Yamanın uygulandığını DOĞRULA** (tek satır — "kurdum sanmak" yetmez):
```bash
PID=$(pgrep -f livox_ros_driver2_node | head -1)
for t in /proc/$PID/task/*; do awk '{print $3}' $t/stat; done | grep -c R
#   0 dönmeli.  1 dönüyorsa yama YOK: bir çekirdek boş döngüde.
```

⚠️ `src/lds.cpp` ve `src/lddc.cpp` **CRLF** satır sonlu; düzenlerken
koruyun, yoksa diff 1800 satır görünür ve gerçek değişiklik kaybolur.

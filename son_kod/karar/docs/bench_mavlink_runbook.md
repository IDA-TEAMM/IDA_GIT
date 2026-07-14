# Bench MAVLink Runbook — Kuru Zemin Canlı Doğrulama

> **Kime:** Donanım entegrasyonu ekibi (hand-off).
> **Ne zaman:** Suya inmeden önce, her saha gününün başında.
> **İlgili doküman:** [`ardurover_bench.parm.md`](ardurover_bench.parm.md) — Pixhawk parametre önerisi.

---

## AMAÇ

Pixhawk 6C USB ile Jetson Orin Nano'ya bağlı, araç **kuru zeminde, pervanesiz**.
Suya inmeden MAVLink hattının ve karar node'larının (fusion → planning →
mavros_bridge → fsm → mission_manager) canlı, uçtan uca doğrulanması. Amaç:
su testinde sürpriz yaşamamak; arm/kill/failsafe zincirini güvenli ortamda görmek.

---

## ÖN KOŞULLAR

- [ ] JetPack 6 (Ubuntu 22.04) + ROS 2 Humble kurulu
- [ ] `ros-humble-mavros` + `ros-humble-mavros-extras` apt paketi kurulu
- [ ] GeographicLib veri seti çalıştırılmış:
      `sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh`
- [ ] `girdap_decision` colcon build edilmiş, `install/setup.bash` source'lanmış
- [ ] ArduRover firmware Pixhawk'ta flash'lı (**Rover-4.5+** önerilir)
- [ ] `config/ardurover_bench.parm` yüklendi (bkz. parametre dokümanı)
- [ ] 🔴 **PERVANELER SÖKÜLMÜŞ VEYA MOTOR KABLOLARI (ESC) DISCONNECTED**
- [ ] 🔴 **Fiziksel kill switch takılı ve operatörün elinde**
- [ ] Kullanıcı `dialout` grubunda (seri porta erişim):
      `groups | grep dialout` boşsa aşağıdaki ADIM 1 düzeltmesine bak

> **Terminoloji:** Aşağıdaki her komut için ROS ortamı source'lanmış olmalı:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/girdap-decision/ros2_ws/install/setup.bash
> export PYTHONPATH=$HOME/girdap-decision:$PYTHONPATH
> ```

---

## ADIM 1 — FCU Bağlantısı

MAVROS'u tek başına başlat (karar node'ları olmadan, izole test):

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:57600
```

Ayrı terminalde:
```bash
ros2 topic echo /mavros/state --once
```

**Beklenen:** `/mavros/state` düzenli HEARTBEAT geliyor:
```
connected: true
armed: false
mode: "MANUAL"        # veya HOLD
```

**Başarısızlık → çözüm:**
| Belirti | Kontrol / Çözüm |
|---|---|
| `connected: false` | `dmesg \| grep tty` → cihaz (`ttyACM0`) görünüyor mu? Kablo/port |
| Port yok | Başka port dene: `ls /dev/ttyACM* /dev/ttyUSB*` |
| `Permission denied` | `sudo usermod -a -G dialout $USER` → **logout/login şart** |
| Sürekli reconnect | Baud uyuşmazlığı; ArduRover `SERIAL0_BAUD` ile eşleştir |

> Bağlantı kurulunca ADIM 2'ye geç. Bu launch'ı **açık bırak** (ADIM 3'te
> kendi launch'ımız MAVROS'u kendi include'uyla başlatacağı için önce bunu
> Ctrl-C ile kapat — aynı fcu_url'e iki bağlantı olmaz).

---

## ADIM 2 — EKF / Extended State

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
```

**Beklenen:** `mode` adı ve `vtol_state`/`landed_state` alanları görünür;
`/mavros/state`'te `system_status` dolu.

> **Not (bench GPS):** Kapalı ortamda GPS fix olmayabilir → **EKF healthy=false
> normaldir**. Bench'te arming'i geçirmek için `ARMING_CHECK=0` politikası
> kullanılır (parametre dosyasında ayarlı).
>
> ⚠️ **SU'DA `ARMING_CHECK=1` YAP** — pre-arm güvenlik kontrolleri açık olmalı.
> Bkz. parametre dokümanı "Suya İnerken Checklist".

---

## ADIM 3 — hardware.launch (Tam Karar Yığını)

Önce ADIM 1'deki tekil MAVROS'u kapat, sonra:

```bash
ros2 launch girdap_decision hardware.launch.py
```

Bu launch MAVROS'u kendi include'uyla başlatır + fusion, planning,
mavros_bridge, fsm, telemetry, local_map, mission_manager node'larını açar.

**Beklenen loglar (gerçek çıktı):**
```
[mavros_bridge] mavros_bridge aktif (heartbeat=5.0s, hedef mod=GUIDED, auto_guided=True)
[mavros_bridge] GUIDED mod isteği gönderildi          # bağlıysa + mod GUIDED değilse
[mission_manager_node] mission_manager_node aktif: 5 waypoint, arrival=2.0 m, dwell=2.0 s, yayım=5.0 Hz
[planning_node] planning_node aktif [düz hedef+MPPI (video)] ...
[fusion_node] fusion_node aktif [MAVROS EKF geçişi (video)] ...
```

> **Notlar:**
> - **ARM otomatik DEĞİL:** `mavros_bridge` kendiliğinden arm etmez (bilinçli
>   operatör eylemi). `GUIDED` moduna geçişi dener; arming'i ADIM 4'te sen
>   tetiklersin.
> - **GPS yoksa `mission_manager` IDLE'da kalır** — GPS fix + FSM start
>   olmadan `current_target` yayınlamaz. Bench'te bu **normaldir**, sorun değil.
> - `algorithm.use_isam2=false`, `use_rrt=false` (video modu) hardware.yaml'dan
>   gelir; yarışma modu için `use_isam2:=true use_rrt:=true` override et.

---

## ADIM 4 — Manuel Arm Testi

Araç GUIDED moduna geçtikten sonra (bench'te `ARMING_CHECK=0` sayesinde):

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Beklenen:** `success: true`

```bash
ros2 topic echo /mavros/state --once      # armed: true görmeli
```

> **Alternatif (retry'li):** `ros2 service call /girdap/bridge/arm std_srvs/srv/Trigger {}`
> → `mavros_bridge` arming'i çağırır; **pre-arm reddinde `arming_retry_max`
> (3) kez, 2 s aralıkla** yeniden dener. Loglar:
> ```
> [mavros_bridge] ARM reddedildi (result=...) — pre-arm bekleniyor, 2s sonra yeniden dene (1/3)
> [mavros_bridge] ARM başarılı (2. deneme)
> ```
> Su'da `ARMING_CHECK=1` iken pre-arm sağlanmazsa 3 denemede vazgeçer,
> KILL **tetiklemez** (araç zaten disarm).

---

## ADIM 5 — cmd_vel → Motor Sinyali

Araç **ARMED** ve **GUIDED** modda. İleri hız setpoint'i bas:

```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10
```

**Beklenen:** Sağ + sol thruster çıkışında PWM ~**1600** (ileri).
Doğrulama: multimetre (servo sinyali) **veya** ESC LED/ses.

> 🔴 **DİKKAT:** Pervaneler **sökük** olmalı, motor mount sağlam sabitlenmiş
> olmalı. Motor dönebilir; ani tork için hazır ol.

**Durdur:**
```bash
# cmd_vel terminalinde Ctrl-C, ardından:
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

---

## ADIM 6 — Kill Switch Testi

Araç **ARMED**, cmd_vel akıyor (ADIM 5 tekrar).

**Eylem:** Fiziksel kill switch'e bas.

**Beklenen:**
- Motor PWM anında **minimum (1000)** (`MOT_SAFE_DISARM=1` → disarm'da PWM min)
- `/mavros/state` → `armed: false`

**Ayrıca — RC failsafe:**
- Eylem: RC vericiyi **kapat**
- Beklenen: `FS_THR_ENABLE=1` / `FS_ACTION=2` (Hold) tetiklenir, motorlar durur

---

## ADIM 7 — Heartbeat Kaybı Simülasyonu

`hardware.launch` açık, araç bağlıyken:

**Eylem:** Pixhawk USB kablosunu **çek**.

**Beklenen (5 sn içinde):**
```
[mavros_bridge] FAILSAFE — heartbeat kaybı (5.5s) → KILL
[fsm_node] *** KILL — motorlar durduruluyor ***
[planning_node] ...   # cmd_vel yayını durur (gate KILL → sıfır thrust)
```

**Eylem:** USB'yi **tekrar tak**.

**Beklenen:** MAVROS yeniden bağlanır; `mavros_bridge` KILL'i **latch'lidir**
→ görev otomatik devam ETMEZ, operatörden **restart** beklenir (soft restart:
node'ları yeniden başlat veya araç güç döngüsü).

---

## YEŞİL KRİTERLER

| Adım | OK? | Not |
|---|---|---|
| 1. FCU bağlantısı (HEARTBEAT) | ☐ | |
| 2. EKF / extended_state | ☐ | GPS yoksa healthy=false normal |
| 3. hardware.launch (7 node) | ☐ | |
| 4. Manuel arm (success=true) | ☐ | |
| 5. cmd_vel → PWM ~1600 | ☐ | pervane sökük |
| 6. Kill switch + RC failsafe | ☐ | PWM→1000, armed=false |
| 7. Heartbeat kaybı → KILL | ☐ | 5 sn içinde |

**7 adım da OK ise araç su testine hazır.**

---

## HATA DURUMUNDA

- Takılınan **adım numarasıyla** karar/algoritma sorumlusuna (Kao) bildir.
- Log çıktısını (tam, kırpılmamış) ve `ros2 topic echo /mavros/state` anlık
  değerini paylaş.
- Karar birlikte alınır — **suya inme kararı tek kişiye bırakılmaz**.
- ⚠️ Herhangi bir adımda motor beklenmedik davranırsa: **kill switch + arming
  false**, sonra teşhis.

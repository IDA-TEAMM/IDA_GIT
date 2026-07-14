# ArduRover Bench Parametre Önerisi — Pixhawk 6C

> **Kime:** Donanım entegrasyonu ekibi (hand-off).
> **İlgili doküman:** [`bench_mavlink_runbook.md`](bench_mavlink_runbook.md) — kuru zemin doğrulama adımları.

---

## AMAÇ

**SITL değil**, gerçek **Pixhawk 6C** üzerinde ArduRover'ı **bench (kuru zemin,
pervanesiz)** modunda güvenli koşturmak. Değerler Mission Planner →
**Config → Full Parameter List** üzerinden girilir (veya `.parm` dosyası olarak
yüklenir). Suya inerken bazı parametreler değişecek — **⚠️ işaretlilere dikkat**.

> **Firmware:** Rover-4.5+ (Boat frame desteği olgun). Katamaran = skid/diferansiyel tahrik.

---

## FRAME (Gövde / Motor Karışımı)

| Parametre | Değer | Neden |
|---|---|---|
| `FRAME_CLASS` | `2` | Boat sınıfı — su aracı dinamiği + skid steering karışımı. |
| `SERVO1_FUNCTION` | `73` | ThrottleLeft — sol thruster çıkışı (diferansiyel tahrik sol). |
| `SERVO3_FUNCTION` | `74` | ThrottleRight — sağ thruster çıkışı (diferansiyel tahrik sağ). |
| `MOT_PWM_MIN` | `1100` | ESC ölü bölgesi üstü minimum; motorun güvenli "durur" PWM'i. |
| `MOT_PWM_MAX` | `1900` | ESC tam gaz üst sınırı; donanım ESC aralığıyla eşleşmeli. |
| `MOT_SAFE_DISARM` | `1` | Disarm'da PWM min'e döner — kill/disarm anında motor kesin durur. |

> **Doğrulama:** Runbook ADIM 5'te sol/sağ PWM ~1600; ADIM 6 disarm'da 1000/min.

---

## ARMING (BENCH — ⚠️ SU'DA DEĞİŞECEK)

| Parametre | Değer | Neden |
|---|---|---|
| `ARMING_CHECK` | `0` | ⚠️ **Bench:** GPS/EKF fix yokken arming'e izin ver (kuru test). **SU'DA `1`** — tüm pre-arm kontrolleri açık. |
| `ARMING_REQUIRE` | `0` | ⚠️ **Bench:** arming zorunluluğunu gevşet. **SU'DA `1`** — motor komutları yalnız armed iken. |

> 🔴 Bu iki parametre **su öncesi checklist'in ilk maddesi**. `ARMING_CHECK=0`
> ile suya inmek = EKF sağlıksızken hareket riski. Asla unutma.

---

## FAILSAFE

| Parametre | Değer | Neden |
|---|---|---|
| `FS_ACTION` | `2` | Hold — failsafe'te motorları durdur (aracı olduğu yerde tut). |
| `FS_TIMEOUT` | `1.5` | 1.5 s sinyal kaybı failsafe eşiği — hızlı tepki, yanlış tetik dengesi. |
| `FS_GCS_ENABLE` | `1` | GCS (MAVROS) heartbeat kaybında failsafe — Jetson/mavros düşerse araç durur. |
| `FS_THR_ENABLE` | `1` | RC throttle failsafe — RC verici kapanınca/menzil dışında failsafe. |

> **Karar yığını ile ilişki:** `mavros_bridge` yazılım tarafı heartbeat KILL'i
> (5 s) yayınlar; `FS_GCS_ENABLE=1` ise **firmware tarafı** bağımsız ikinci
> katman. İki katman = tek nokta arıza yok (Runbook ADIM 7).

---

## MODE / GCS / Telemetri Rate

| Parametre | Değer | Neden |
|---|---|---|
| `SYSID_MYGCS` | `255` | MAVROS varsayılan GCS ID'si — bridge komutlarının kabulü için eşleşmeli. |
| `SR0_EXTRA1` | `10` | Attitude (roll/pitch/yaw) 10 Hz — Dosya-2 telemetri + video log için taze poz. |
| `SR0_POSITION` | `10` | Local/global position 10 Hz — fusion/planning + Dosya-2 lat/lon akışı. |
| `SR0_EXT_STAT` | `2` | Extended status (mod, arm, sistem) 2 Hz — `/mavros/state` bant genişliği dengesi. |

> **Not:** `SR0_*` = SERIAL0 (USB) mesaj akış hızları. Jetson'a giden USB
> portu; telemetri (MicoAir) ayrı port/rate ister. Dosya-2 ≥1 Hz şartını
> `SR0_POSITION=10` fazlasıyla karşılar.

---

## EKF

| Parametre | Değer | Neden |
|---|---|---|
| `EK3_ENABLE` | `1` | EKF3 füzyon motoru açık — poz/hız kestirimi (yarışma modu iSAM2 bunu tamamlar). |
| `EK3_GPS_TYPE` | `0` | 3D hız + konum — RTK GPS'ten tam ölçüm kullan. |
| `AHRS_EKF_TYPE` | `3` | EKF3'ü ana AHRS kaynağı yap (EKF2 değil) — daha olgun, RTK uyumlu. |

> **Video modu ilişkisi:** `use_isam2=false` iken karar yığını doğrudan bu
> EKF3'ün `/mavros/local_position/pose` çıktısını kullanır (pass-through).
> Yarışma modunda iSAM2 aynı ölçümleri GPS+IMU füzyonuyla pürüzsüzleştirir.

---

## `.parm` Dosyası Üretimi

Yukarıdaki değerler Mission Planner'da tek tek girilebilir veya bir
`config/ardurover_bench.parm` metin dosyası olarak (her satır `PARAM,VALUE`)
**Full Parameter List → Load** ile yüklenebilir. Örnek satır formatı:
```
FRAME_CLASS,2
SERVO1_FUNCTION,73
...
ARMING_CHECK,0
```
> Yükleme sonrası **Write Params** + Pixhawk **reboot** şart (frame/servo
> fonksiyonları reboot'ta oturur).

---

## 🔴 SUYA İNERKEN CHECKLIST

Bench yeşil olsa bile **suya inmeden önce** bu listeyi operatör yüksek sesle okur:

- [ ] `ARMING_CHECK = 1` (pre-arm kontrolleri AÇIK)
- [ ] `ARMING_REQUIRE = 1`
- [ ] Kill switch bağlı **ve test edildi** (Runbook ADIM 6)
- [ ] RC failsafe test edildi (verici kapatma → Hold)
- [ ] Motor mount sıkı, pervaneler **takılı ve sağlam**
- [ ] Batarya voltajı **> 15 V** (4S için; hücre başına > 3.75 V)
- [ ] Write Params + reboot yapıldı, değişiklikler oturdu
- [ ] GPS fix alındı (`/mavros/state` + EKF healthy=true)

> Bu checklist'te **tek bir madde bile ☐ ise araç suya İNMEZ.**

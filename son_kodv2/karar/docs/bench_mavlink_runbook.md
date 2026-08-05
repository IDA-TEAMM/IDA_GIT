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

> ⚠️ Bu adım yalnız **düz ileri**yi doğrular. Sol/sağ kanallar ters bağlıysa
> düz gidişte HİÇBİR fark görünmez — dönüş yönü test edilmeden mixing
> doğrulanmış sayılmaz → **ADIM 5B**.

---

## ADIM 5B — Diferansiyel Mixing / Dönüş Yönü

**Neden ayrı adım:** ADIM 5 iki motora da aynı komutu verir; `SERVO1`(sol) ve
`SERVO3`(sağ) yer değiştirmiş olsa bile sonuç aynı görünür. Yanlış bağlantı
ancak **dönüşte** ortaya çıkar — ve suda ortaya çıkarsa tekne her kapıda ters
tarafa kırar (md 5.5.4.2 geçiş puanı sıfırlanır).

Test **ikiye bölünmüştür**; ikisi ayrı şeyi doğrular ve **5B-1 ROS'suz koşar**:

| | Ne doğrular | Neye ihtiyaç var |
|---|---|---|
| **5B-1** | FC'nin karıştırması: `FRAME_CLASS=2`, `SERVO1/3_FUNCTION`, ESC yönleri | **Yalnız Mission Planner** (Motor Test — RC bile gerekmedi) |
| **5B-2** | `cmd_vel` → yaw işaret sözleşmesi (MAVROS üzerinden) | ROS ortamı |

> Karıştırmayı **FC yapıyor**, bizim yazılımımız değil. Bu yüzden 5B-1 karar
> yığını hiç çalışmadan yapılabilir — ROS ortamı hazır değilken de ilerler.

**Ortak ön koşul:** ESC kalibrasyonu yapılmış (`ardurover_bench.parm.md` → ESC
KALİBRASYONU) · 🔴 pervaneler **sökük**.

**Okuma yeri:** Mission Planner → **Setup → Optional Hardware → Servo Output**
(ya da Status ekranında `ch1out` / `ch3out`).

---

### 5B-1 — Mixing testi — ✅ **KOŞULDU 2026-08-05, GEÇTİ**

> **RC'ye gerek kalmadı.** Test Mission Planner **Motor Test** sayfasıyla
> yapıldı (`KURULUM → Opsiyonel ekipmanlar → Motor testi`), araç disarm,
> pervaneler sökülü, `Guc %=20`, `Sure=2`.

#### 🔑 ArduRover motor-test numaralandırması (bulundu, Copter'dan FARKLI)

| Buton | Örnek | Rover'da karşılığı | Sonuç |
|---|---|---|---|
| A | 1 | yön (steering) — skid steer'de doğrudan çıkış yok | hiçbir şey dönmez (normal) |
| B | 2 | gaz (throttle) — aynı şekilde | hiçbir şey dönmez (normal) |
| **C** | 3 | **ThrottleLeft** | 🔵 **SOL motor** |
| **D** | 4 | **ThrottleRight** | 🔵 **SAĞ motor** |

> ⚠️ A ve B'nin dönmemesi ARIZA DEĞİLDİR. İlk denemede "motor testi çalışmıyor"
> sanıldı; gerçek motorlar C ve D'dir.
>
> Ayrıca: **parametreler tam inmeden** sayfaya girilirse butonlar kilitli
> görünür (`Class: unknown`). Telemetride (57600) indirme ~1 dk sürer, bekle.

#### Ölçülen sonuç (2026-08-05)

```
C (ThrottleLeft)  → SOL motor,  saat yönü        ✓ eşleme doğru
D (ThrottleRight) → SAĞ motor,  saat tersi       ✓ eşleme doğru
Pervaneler        → AYNA ÇİFT (counter-rotating)
```

**Yorum:** Ayna pervanede zıt dönüş **doğrudur** — ikisi de ileri iter.
Dolayısıyla `SERVO1_REVERSED=1` / `SERVO3_REVERSED=0` asimetrisi bilinçli ve
yerindedir, **dokunulmayacak**.

**Elenen iki tehlikeli arıza:**
- sol/sağ kanal takası (tekne kapıda ters tarafa kırardı) → **YOK**
- aynı-pervane + zıt dönüş (tekne ileri gitmez, yerinde dönerdi) → **YOK**

#### ⏳ Doğrulanmayan tek şey: MUTLAK yön

Test iki motorun **birbirine göre** doğru olduğunu kanıtladı. "İleri komutu
gerçekten ileri itiyor mu, yoksa ikisi de geri mi itiyor" pervanesiz
anlaşılamaz. Bu **düşük riskli**: ikisi birden ters olsaydı tekne sadece geri
giderdi — ilk suya inişte anında görülür ve simetrik olduğu için tehlikeli
değil. Asıl tehlikeli olan iki mod (takas / yerinde dönme) yukarıda elendi.
**İlk su testinde teyit edilecek.**

---

### (arşiv) RC ile alternatif yöntem — kullanılmadı

1. Mod: **MANUAL** · araç **ARM** edilmiş
2. Sağ çubuğu (steering) **sola** it, MP'de Servo Output'u izle

> 🔴 **ÖNCE ŞUNU BİL — `SERVO1_REVERSED=1`, `SERVO3_REVERSED=0`** (2026-08-04
> param dökümü). Kanallardan biri ters çevrilmiş, yani **ekrandaki PWM'lere
> bakarak mixing'i yorumlayamazsın.** Ters çevrilmiş kanalda "ileri" komutu
> ekranda 1500'ün ALTINDA görünür.
>
> Bu asimetri **fiziksel motor yönü farkını telafi ediyor olabilir** (doğru) ya
> da hata olabilir. **Bu testin asıl amacı bunu ayırt etmek.**

**Beklenen (mevcut REVERSED ayarlarına göre):**

| Komut | `SERVO1` (Sol, fn 73, **ters**) | `SERVO3` (Sağ, fn 74, düz) |
|---|---|---|
| İleri (throttle) | **< 1500** | **> 1500** |
| Sola dönüş | 1500'e yaklaşır/geçer | **> 1500**, artar |
| Sağa dönüş | **daha da < 1500** | 1500'e yaklaşır/geçer |

**Simetri kontrolü (ESC kalibrasyonunu sınar):** düz ileri komutta iki çıkışın
**1500'e uzaklıkları eşit** olmalı:
```
|SERVO1 − 1500|  ≈  |SERVO3 − 1500|
örn. SERVO1=1400, SERVO3=1600  → ikisi de 100 → simetrik ✅
     SERVO1=1400, SERVO3=1560  → 100 vs 60   → asimetrik ❌ tekne düz gitmez
```

> ⚠️ **PWM yeterli değil — motor yönünü GÖZLE doğrula.** ESC'lere güç verip
> (pervaneler SÖKÜK) düz ileri komutunda **iki motorun da aynı yöne, ileri
> itecek şekilde** döndüğünü gör. `REVERSED` ayarları doğruysa öyle olur;
> yanlışsa motorlar zıt yöne döner ve tekne ileri gitmek yerine yerinde döner.

### Sonuç yorumu (5B-1)

| Gözlem | Anlamı | Ne yapılacak |
|---|---|---|
| Beklendiği gibi | Mixing doğru | ✅ geç |
| Sol/sağ **tam ters** | `SERVO1`/`SERVO3` fonksiyonları veya motor kabloları yer değişmiş | `SERVO1_FUNCTION=73` / `SERVO3_FUNCTION=74` teyit et; doğruysa fiziksel ESC çıkışlarını takas et |
| Bir taraf hiç değişmiyor | O kanal atanmamış / ESC ölü | `SERVOx_FUNCTION` ve besleme kontrol |
| İkisi de aynı yöne gidiyor | Skid mixing devre dışı | `FRAME_CLASS=2` teyit + reboot |
| İleri komutta iki PWM farklı | ESC'ler farklı kalibre | ESC kalibrasyonunu ikisine birden tekrarla |

---

### 5B-2 — cmd_vel işaret sözleşmesi (ROS ortamı gerekir)

5B-1 geçtikten sonra. Araç **ARMED + GUIDED**:

```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{angular: {z: 0.3}}" -r 10
```

`planning_node` sözleşmesi: `angular.z = (sağ_itki − sol_itki)` → **pozitif
angular.z = sola (CCW) dönüş**, yani sağ motor daha hızlı. Yani beklenen çıktı
5B-1'in "sola" satırıyla **aynı** olmalı. `z: -0.3` ile tersi.

> 🔴 **AÇIK SORU — bu test cevaplayacak:** Eski `ida_topics/decision_node.py`'de
> `cmd.angular.z = -angular` şeklinde **bilinçli bir işaret çevirmesi** vardı
> ("ArduPilot yaw yönü uyumu" gerekçesiyle). `girdap_decision/planning_node.py`'de
> böyle bir çevirme **YOK**. İkisi aynı anda doğru olamaz.
>
> **Ayrım kritik:** 5B-1 geçip 5B-2 ters çıkarsa sorun **kodda**
> (`_publish_cmd_vel`) — FC'ye dokunma, karar ekibine bildir. İkisi birden ters
> çıkarsa sorun **FC/kabloda**.

**Durdur:** komut terminalinde Ctrl-C.

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
- `/mavros/state` → `armed: false`
- Motor durur. 🔴 **ESC'ler ÇİFT YÖNLÜ** → "PWM min (1100)" duruş DEĞİL,
  **tam geri** demektir. Servo çıkışında kabul edilebilir tek iki sonuç:

| Gözlenen | Anlamı |
|---|---|
| Sinyal yok (pals kesik) | ESC failsafe'iyle durur ✅ |
| **1500** | nötr = duruş ✅ |
| **1100** | 🔴 **TAM GERİ** — `MOT_SAFE_DISARM`/`SERVOx_TRIM` yanlış, **suya inilmez** |

> Ayrıntı: [`ardurover_bench.parm.md`](ardurover_bench.parm.md) → "ÇİFT YÖNLÜ ESC".

**Ayrıca — RC failsafe:**
- Eylem: RC vericiyi **kapat**
- Beklenen: `FS_THR_ENABLE=1` / `FS_ACTION=2` (Hold) tetiklenir, motorlar durur

> ⚠️ Bu adım yalnız **sinyal** kesmeyi doğrular. Şartname md 4.2 ayrıca gücün
> kesilmesini şart koşuyor → ADIM 6B.

---

## ADIM 6B — Uzaktan GÜÇ Kesme (şartname md 4.2, minimum gereksinim)

> Şartname: *"motorlara gönderilen sinyallerin akışını kesmek yeterli değildir,
> **motorların gücünün kesilmesi şarttır**."* Donanım reçetesi:
> [`fc_parametre_onerileri.md` §4.5](fc_parametre_onerileri.md).

**Ön koşul:** Kontaktör/röle takılı (ESC kolunda, FC/Jetson kolunda DEĞİL) ·
pervaneler **sökük** · multimetre hazır.

Araç **ARMED**, cmd_vel akıyor (ADIM 5 tekrar).

**Ölçüm noktası:** ESC besleme ucu (kontaktörün motor tarafı), DC volt kademesi.

**Eylem:** RC kanal 8 kill anahtarına bas.

**Beklenen — üçü birden:**

| # | Beklenen | Nasıl görülür |
|---|---|---|
| 1 | ESC besleme **0 V** | Multimetre — **bu maddenin asıl kanıtı** |
| 2 | `armed: false`, PWM 1000 | `ros2 topic echo /mavros/state` (üçüncü katman hâlâ çalışıyor) |
| 3 | **Telemetri CSV yazmaya DEVAM ediyor** | `tail -f ~/girdap_logs/telemetri/.../*.csv` satır eklemeye devam etmeli |

> 🔴 3. madde geçmezse röle **yanlış kola** takılmıştır: Jetson da güç kaybediyor
> → Dosya-1/2/3 yarım kalır, her biri için 5 ceza puanı (md 5.5.4.3.5).

**Eylem 2 (fail-safe yönü):** Anahtar basılıyken röle bobin kablosunu ayır.
**Beklenen:** Motorlar güçsüz KALIR (NO/energize-to-run seçimi doğruysa). Kablo
koptuğunda motorların çalışır hale gelmesi = ters emniyet, kontaktör yanlış tipte.

**Geri alma:** Anahtarı bırak → ESC beslemesi geri gelir; araç **disarm** kalır
(`mavros_bridge` KILL latch'lidir), yeniden arm operatörden beklenir.

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
| 5B-1. **Mixing / kanal + yön** | ✅ | 2026-08-05 GEÇTİ — MP Motor Test C=sol/saat yönü, D=sağ/saat tersi, pervaneler ayna → REVERSED asimetrisi DOĞRU |
| 5B-2. **cmd_vel işaret sözleşmesi** | ☐ | 5B-1 geçtikten sonra; ters çıkarsa düzeltme KODDA |
| 6. Kill switch + RC failsafe | ☐ | PWM→1000, armed=false |
| 6B. **Uzaktan GÜÇ kesme** | ☐ | 🔴 ESC ucunda **0 V** + CSV yazmaya devam ediyor — md 4.2 minimum gereksinimi |
| 7. Heartbeat kaybı → KILL | ☐ | 5 sn içinde |

**9 adım da OK ise araç su testine hazır.**

> 🔴 **ADIM 6B geçmeden yarışmaya gidilmez** — teknik kontrolde bakılan bir
> minimum gereksinimdir (md 4), eksikliği yazılımla telafi edilemez.

---

## HATA DURUMUNDA

- Takılınan **adım numarasıyla** karar/algoritma sorumlusuna (Kao) bildir.
- Log çıktısını (tam, kırpılmamış) ve `ros2 topic echo /mavros/state` anlık
  değerini paylaş.
- Karar birlikte alınır — **suya inme kararı tek kişiye bırakılmaz**.
- ⚠️ Herhangi bir adımda motor beklenmedik davranırsa: **kill switch + arming
  false**, sonra teşhis.

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

## ADIM 5 — Motor Sinyali / PWM aralığı

> ## 🔴 GUIDED + `cmd_vel` BENCH'TE KULLANILAMAZ (2026-08-06'da öğrenildi)
>
> Bu adımın önceki hâli *"ARMED + GUIDED, `cmd_vel` 0.1 m/s bas, ~1600 gör"*
> diyordu. **Denendi ve motorlar anında %100 gaza gitti** (`ch1out=1994`,
> `ch3out=2000`).
>
> **Neden:** GUIDED'da ArduRover **kapalı döngü hız kontrolü** yapar. Tekne
> masada olduğu için ölçülen hız **daima 0**; kontrolcü hatayı kapatmaya
> çalışıp **integralini doyurur** → tam gaz. Arıza değil, kontrolcünün doğal
> davranışı. Hangi hızı istersen iste (0.1 m/s de olsa) sonuç aynı.
>
> ⚠️ Yani **"~1600 göreceksin" beklentisi bench'te fiziksel olarak imkânsız.**
> Suda geçerli, karada değil.
>
> **Doğru bench yöntemi: MANUAL mod.** Gaz orada açık döngü — çubuk ne kadar
> itilirse çıkış o kadar olur, doyum yok.

**Yöntem (MANUAL):**

```bash
# 1) Mod MANUAL (kumanda gaz çubuğu NÖTRDE olmalı — geçişte çıkış çubuğa atlar)
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: \"MANUAL\"}"

# 2) Çıkışları kaydet (kumandadan gaz verirken)
ros2 topic echo /mavros/rc/out --field channels   # channels[0]=SERVO1, channels[2]=SERVO3
ros2 topic echo /mavros/rc/in  --field channels   # karşılaştırma için
```

Kumandadan gazı yavaşça ileri it, birkaç saniye tut, nötre bırak.

**Beklenen:** iki çıkış da nötrden (`SERVOn_TRIM`) **birlikte** yükselir,
aralarındaki fark ~0 kalır, ileri komutta 1600'ü geçer.

> 🔴 **DİKKAT:** Pervaneler **sökük** olmalı, motor mount sağlam sabitlenmiş
> olmalı. Motor dönebilir; ani tork için hazır ol.

### ✅ ÖLÇÜLDÜ — 2026-08-06, GEÇTİ

MAVROS `/dev/ttyACM0` (Pixhawk USB) üzerinden bağlıyken, Mission Planner
telemetri radyosunda (`ttyUSB0`) kalarak. İki ayrı kayıt, toplam 125 örnek:

| Kayıt | `ch1out` (SAĞ) | `ch3out` (SOL) | RC gaz |
|---|---|---|---|
| 1 (kısmi gaz) | 1430 → 1552 | 1430 → 1552 | 1438 → 1520 |
| 2 (yüksek gaz) | 1556 → **1976** | 1556 → **1976** | 1523 → 1884 |

```
|ch1out − ch3out|  =  0      125 örneğin HEPSİNDE
1600 üstü          :  30/48 örnek (2. kayıt)
tepe               :  1976  (maks 2000)
```

**Doğrulananlar:**
- ✅ **Simetri mükemmel** — 1430 (geri) → 1487 (nötr) → 1976 (ileri) aralığının
  tamamında tek µs ayrışma yok. "Aynı PWM farklı itki" riski FC tarafında YOK.
- ✅ **Mixing doğru** — düz gaz komutunda iki kanal birlikte hareket ediyor.
- ✅ **Yön doğru** — ileri çubuk → nötrün ÜSTÜ, geri çubuk → nötrün ALTI.
- ✅ **Çift yönlü çalışma** — hem ileri hem geri sinyal üretiliyor, simetrik.
- ✅ Belgenin (`Document 11.pdf` A-1) istediği **"≈1600"** aşıldı.

**⏳ Hâlâ açık (bu adımın kapsamı DIŞINDA):** `cmd_vel` → yaw işaret sözleşmesi.
Bu ADIM **5B-2**'nin işi ve suda/hareket halinde ölçülmeli — bench'te yukarıdaki
doyum sorunu yüzünden ölçülemez. MAVROS→FC komut yolunun **çalıştığı** ayrıca
kanıtlandı (MAVROS'tan verilen mod değişimlerine FC anında yanıt verdi).

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

> 🔴 **2026-08-06 DÜZELTMESİ.** Buraya önce *"`SERVO1_REVERSED=1` /
> `SERVO3_REVERSED=0` asimetrisi bilinçli"* yazılmıştı — **YANLIŞ**. Canlı FC'de
> **ikisi de `REVERSED=0`**. O iddia `parametrelerDefMP.param` adlı, bu tekneden
> alınmamış bir dosyadan geliyordu (146 parametrede canlıdan farklı). Yani
> testin geçmesini "asimetri telafi ediyor" diye açıklamak gereksizdi: zıt
> dönüş **doğrudan aynalı pervaneden** geliyor, FC tarafında ters çevirme yok.
> Gerçek değerler: `docs/fc_mevcut_parametreler_2026-08-06.param`.
>
> Ayrıca kanal eşlemesi: **`SERVO1` = fn 74 = SAĞ**, **`SERVO3` = fn 73 = SOL**.

Sonuç değişmiyor — mixing, kanal eşlemesi ve dönüş yönü **doğru**,
**dokunulmayacak**.

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

> 🔴 **2026-08-06'da geçersiz kılındı.** Buradaki `SERVO1_REVERSED=1` varsayımı
> yanlış kaynaktan geliyordu; canlı FC'de **ikisi de 0** ve kanallar
> `SERVO1`=SAĞ(74) / `SERVO3`=SOL(73). Ters çevirme olmadığı için aşağıdaki
> "ters kanal" uyarısı ve tablo **artık geçerli değil** — ileri komutta iki
> çıkış da 1500'ün ÜSTÜNDE görünür.

**~~Beklenen (mevcut REVERSED ayarlarına göre)~~ — ARŞİV, güncel değil:**

| Komut | `SERVO1` (Sol, fn 73, **ters**) | `SERVO3` (Sağ, fn 74, düz) |
|---|---|---|
| İleri (throttle) | **< 1500** | **> 1500** |
| Sola dönüş | 1500'e yaklaşır/geçer | **> 1500**, artar |
| Sağa dönüş | **daha da < 1500** | 1500'e yaklaşır/geçer |

**Simetri kontrolü (FC KARIŞTIRMASINI sınar):** düz ileri komutta iki çıkışın
**1500'e uzaklıkları eşit** olmalı:
```
|SERVO1 − 1500|  ≈  |SERVO3 − 1500|
örn. SERVO1=1400, SERVO3=1600  → ikisi de 100 → simetrik ✅
     SERVO1=1400, SERVO3=1560  → 100 vs 60   → asimetrik ❌ tekne düz gitmez
```

> 🔴 **BU KONTROL ESC KALİBRASYONUNU *SINAMAZ*** (2026-08-06 düzeltmesi — eski
> başlık "ESC kalibrasyonunu sınar" diyordu, YANLIŞ). Okunan PWM **FC'nin
> ürettiği sinyaldir**. `SERVO1/3_MIN/MAX/TRIM` iki kanalda birebir aynıysa
> (bizde öyle) bu değerler **tanım gereği** simetrik çıkar — ESC'lerin *iç*
> davranışı hakkında hiçbir şey söylemez. Yani buradaki ✅ yalnız
> **FC → ESC sinyal yolunu** doğrular, **ESC → itki** eşleşmesini DEĞİL.
>
> ESC iç asimetrisi ancak **suda** görünür: `angular.z=0` verilirken sabit
> heading kayması. Bkz. `ardurover_bench.parm.md` → ESC adım 3 "KALAN RİSK".

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
| İleri komutta iki PWM farklı | **FC parametreleri** farklı (ESC değil — PWM'i FC üretir) | `SERVO1/3_MIN`, `_MAX`, `_TRIM` altı değerini yan yana oku, eşitle |
| Suda düz komutta tekne bir yana kayıyor | ESC'lerin **iç** trim'i farklı (bench'te görünmez) | `SERVOn_TRIM`'de ±birkaç µs telafi; kalibrasyon DEĞİL |

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
- Motor durur. 🔴 **ESC'ler ÇİFT YÖNLÜ** → "PWM min (1000)" duruş DEĞİL,
  **tam geri** demektir. Servo çıkışında kabul edilebilir tek iki sonuç:

| Gözlenen | Anlamı |
|---|---|
| Sinyal yok (pals kesik) | ESC failsafe'iyle durur ✅ |
| **1487-1500** | nötr = duruş ✅ (canlı `SERVOn_TRIM=1487`) |
| **1000** | 🔴 **TAM GERİ** — `MOT_SAFE_DISARM`/`SERVOx_TRIM` yanlış, **suya inilmez** |

> Ayrıntı: [`ardurover_bench.parm.md`](ardurover_bench.parm.md) → "ÇİFT YÖNLÜ ESC".

### ✅ ÖLÇÜLDÜ ve GEÇTİ — 2026-08-07

| Durum | `ch1out` (SAĞ) | `ch3out` (SOL) | Sonuç |
|---|---|---|---|
| ARMED, sıfır komut | 1487 | 1487 | ✅ |
| **DISARMED** | **1487** | **1487** | ✅ **nötr — 1000 (tam geri) DEĞİL → suya inilebilir** |

Koşullar: Pixhawk USB'den besleniyor (batarya yok, ESC'ler beslenmiyor) → motorlar
dönemez, **sıfır riskli ölçüm**. `MOT_SAFE_DISARM=0` olduğu için çıkış kesilmiyor,
`SERVOn_TRIM`'de (1487) kalıyor — beklenen davranış, doğrulandı.

> 🔴 **BU ÖLÇÜM ÖNCE ALINAMIYORDU — SEBEBİ BULUNDU: `ARMING_REQUIRE=0`**
>
> 2026-08-06'da disarm **hiçbir yolla** çalışmadı: Mission Planner Arm/Disarm,
> MP Force Disarm, kumandadan rudder disarm, MAVROS `cmd/arming` — MANUAL'da da
> HOLD'da da reddedildi (`result=4`), FC hiç açıklama mesajı basmadı.
>
> **Sebep:** `ARMING_REQUIRE=0`. ArduPilot'un kendi parametre açıklaması:
> *"Arming disabled until some requirements are met. **If 0, there are no
> requirements (arm immediately)**"* → araç sürekli armed sayılır, disarm
> kavramı yoktur.
>
> **🔴 EN ÖNEMLİ SONUÇ:** Yazılımımızın KILL yolu (`mavros_bridge` → disarm +
> sıfır thrust) `ARMING_REQUIRE=0` iken **HİÇ ÇALIŞMIYORDU**. Acil durdurma
> sessizce etkisizdi ve kimse fark etmemişti çünkü disarm hiç test edilmemişti.
>
> **Düzeltme:** `ARMING_REQUIRE = 1` (2026-08-07). Disarm ilk denemede çalıştı.
> ⚠️ Bu ayar **geri alınmamalı** — 0 yapılırsa acil durdurma tekrar ölür.
> ⚠️ Yan etki: araç artık kendiliğinden armed gelmez, arm edilmesi gerekir
> (masa testi akışlarını etkileyebilir — takıma haber verilmeli).

### (arşiv) 2026-08-06 — ölçüm alınamadı

Tekneye bağlıyken MP → `Durum` sekmesi → `ch1out`/`ch3out` okundu:

| Durum | `ch1out` (SAĞ) | `ch3out` (SOL) | Sonuç |
|---|---|---|---|
| **ARMED**, sıfır komut | **1487** | **1487** | ✅ ikisi de `SERVOn_TRIM`'de, **1000 DEĞİL** → tam geri komutu yok |
| DISARM | — | — | ⏳ **ölçülemedi** |

**Neden ölçülemedi:** Normal disarm **reddedildi** — FC aracı hareketli sanıyordu
(`GS 2,2 m/s`, tekne masada duruyorken). Force Disarm da kabul etmedi. Sonunda
**güç kesilerek** durduruldu, o da disarm değil.

> ⚠️ **Güç kesmek ≠ disarm.** Güç kesilince FC hiç sinyal üretmez (ESC'ler kendi
> failsafe'iyle durur — güvenli), ama bu adımın asıl sorusu *"FC disarm ettiğinde
> servo çıkışına NE koyuyor"* hâlâ cevapsız. `MOT_SAFE_DISARM=0` olduğu için
> çıkışın kesilmeyip 1487'de kalması bekleniyor — **doğrulanmalı**.

🔴 **Ölçüm sırasında bulunan üç anomali** (suya inmeden çözülmeli):
1. **`GS 2,2 m/s`** tekne dururken → sahte hız tahmini. Disarm'ı engelledi;
   suda MPPI'yi de yanıltır (araç "ilerliyorum" sanıp yanlış düzeltir).
   GPS suçlu değil: 21-25 uydu, 3D dgps, `prearmstatus True`.
2. **`Kotu AHRS`** kalıcı (açılış artığı değil — 1.5 saat sonra hâlâ duruyordu),
   oysa EKF Status penceresinde beş varyans da sıfıra yakın ve bayraklar temiz
   (`const pos mode Off`). Çelişki çözülmedi.
3. **`Bat1 57,7 V / 115,4 A`** — paket 4S (dolu 16.8 V), motorlar dururken 115 A
   çekilmiyor. `BATT_VOLT_MULT`/`BATT_AMP_PERVLT` kalibrasyonu şüpheli.
   Gerilim failsafe'i (13.2/12.4 V) bu okumayla **hiç tetiklenmez**.

**Ayrıca — RC failsafe:**
- Eylem: RC vericiyi **kapat**
- Beklenen: `FS_THR_ENABLE=1` / `FS_ACTION=2` (Hold) tetiklenir, motorlar durur
- ⚠️ Canlı FC'de **`FS_THR_ENABLE=0`** → bu test şu an **geçersiz**, önce 1 yapılmalı

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
| 5. Motor sinyali / PWM aralığı | ✅ | 2026-08-06 GEÇTİ — MANUAL modda 1430→1976, iki kanalda **125 örnekte 0 fark**. ⚠️ GUIDED+cmd_vel bench'te KULLANILAMAZ (integral doyumu → %100 gaz) |
| 5B-1. **Mixing / kanal + yön** | ✅ | 2026-08-05 GEÇTİ — MP Motor Test C=sol/saat yönü, D=sağ/saat tersi, pervaneler ayna → zıt dönüş DOĞRU (FC'de ters çevirme yok, ikisi de `REVERSED=0`) |
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

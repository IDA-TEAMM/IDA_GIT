# Karar hataları

Görev durum makinesi (FSM), füzyon/odometri, planlama ve kontrol/aktüasyon
katmanındaki bulgular. Kaynak: 14 rosbag oturumu, bkz. [README](README.md).

**Katman sözleşmesi** — iki ayrı durum akışı vardır ve string biçimleri **farklıdır**:

| Topic | Yayıncı | Biçim | Örnek değerler |
|---|---|---|---|
| `/girdap/mission/state` | `fsm_node.py:219` | alt çizgisiz | `BOOT` `ARM` `BEKLEMEDE` `PARKUR1` `PARKUR2` `KILL` `TAMAMLANDI` |
| `/girdap/parkur/state` | `fsm_node.py:225` | alt çizgili | `PARKUR_1` `PARKUR_2` `PARKUR_3` |

| # | Bulgu | Şiddet |
|---|---|---|
| [KAR-01](#kar-01) | `/girdap/mission/state` üzerinde iki çelişkili durum akışı | 🔴 |
| [KAR-02](#kar-02) | KILL durumundan çıkış yok — oturum sonuna kadar kalıyor | 🔴 |
| [KAR-03](#kar-03) | BOOT kilitlenmesi — 25 dakika BOOT'ta, ama 10 Hz komut yayınlamaya devam | 🔴 |
| [KAR-04](#kar-04) | `/girdap/control/thrust` hiçbir oturumda sıfırdan farklı olmadı | 🔴 |
| [KAR-05](#kar-05) | Füzyon geçersiz veriyi geçerli gibi yayınlıyor — 16.974 mesaj (0,0,0) | 🔴 |
| [KAR-06](#kar-06) | Odometri ışınlanması — 25 ms'de 6,54 m (257 m/s) | 🟠 |
| [KAR-07](#kar-07) | Füzyon 32 m'lik alanda 4.766 m "yol" biriktirdi | 🟠 |
| [KAR-08](#kar-08) | Görev hiç PARKUR'a geçemedi; `mission/complete` hep `False` | 🟠 |
| [KAR-09](#kar-09) | Tüm hat 8-12 saniye donuyor | 🟠 |
| [KAR-10](#kar-10) | FCU'ya hız komutu neredeyse hiç gitmedi — 5 saatte 110 mesaj | 🟠 |
| [KAR-11](#kar-11) | Kontrol döngüsü 10 Hz bütçesini tutturamıyor; periyot oturum boyunca 10 katına çıkıyor | 🔴 |

---

## KAR-01

### `/girdap/mission/state` üzerinde iki çelişkili durum akışı

**Şiddet:** 🔴 Kritik

**Kanıt** — `session_19700101_020215`, `t≈1786420314` civarı, ardışık geçişler:

```
ARM      -> PARKUR2   dt=  21 ms
PARKUR2  -> ARM       dt=  21 ms
ARM      -> PARKUR2   dt=  80 ms
PARKUR2  -> ARM       dt=  18 ms
ARM      -> PARKUR2   dt=  82 ms
PARKUR2  -> ARM       dt=  18 ms
...
```

Oturum boyunca **1.190 durum geçişi**; geçişlerin **%92,7'si 0,5 saniyeden kısa**,
medyan **47,5 ms**.

**Bulgu:** Desen rastgele değil — `20 ms / 80 ms` **sabit faz farkıyla** tekrar ediyor,
yani toplam periyot tam **100 ms = 10 Hz**. Tek bir FSM'in salınımı bunu üretmez
(o durumda 100/100 ms simetrik olurdu). Bu, **iki bağımsız 10 Hz yayıncının** aynı
topic'e farklı durum yazmasının imzasıdır.

Üretim kodunda `/girdap/mission/state`'in **tek** yayıncısı var (`fsm_node.py:219`).
Dolayısıyla ya iki `fsm_node` örneği aynı anda koştu, ya da yayıncı dışarıdan geldi.
İkinci ihtimali destekleyen kanıt: `karar/prototype/tests/` altındaki testler bu
topic'e yayın yapıyor (`test_mission_manager_node.py:85,122,287,323,474,549` ve
`test_telemetry_node.py:124,180`) ve **testlerde `ROS_DOMAIN_ID` izolasyonu yok** —
bkz. [PAR-01](parametre.md#par-01).

Testlerin yaydığı değerler `PARKUR1` (13 kez) ve `BEKLEMEDE`'dir; `PARKUR2` yaymazlar.
Aynı oturumun durum dağılımında `PARKUR1` **16.982** örnekle bulunuyor — bu kısım
büyük olasılıkla test sızıntısı. `ARM ↔ PARKUR2` salınımının kaynağı ise ayrıca
doğrulanmalı (en olası: ikinci bir `fsm_node` örneği).

Aynı oturumda `tf_static` **28 mesaj** taşıyor (sağlıklı oturumlarda 4) ve MAVROS
tanılamasında `Node starting up` 5-7 kez tekrarlıyor — ikisi de **düğümlerin
tekrar tekrar başlatıldığını** gösteriyor, çift örnek hipoteziyle tutarlı.

**Etki:** FSM'i dinleyen her düğüm (`mission_manager_node`, `planning_node`,
`mavros_bridge_node`, `telemetry_node`, `kamikaze_param`) saniyede 10 kez birbiriyle
çelişen durum görüyor. Görev-aktif geçidi (`mavros_bridge_node.py:220`) sürekli açılıp
kapanıyor → aktüasyon kararsız. Telemetri kaydı ([şartname zorunlu](../README.md))
anlamsız.

**Öneri:**
1. `fsm_node`'a **tekil örnek koruması** ekle: başlangıçta aynı isimde düğüm varsa
   `RuntimeError` ile çık (`ros2 node list` kontrolü ya da bir kilit dosyası).
2. `sistem_baslat.sh`'e başlatmadan önce `pkill -f fsm_node` ekle.
3. Test izolasyonu — bkz. [PAR-01](parametre.md#par-01). Bu düzeltilmeden bag verisi
   güvenilir değil.
4. FSM geçişlerine **minimum bekleme süresi** (dwell time, öneri 0,5 s) koy; meşru bir
   geçiş 20 ms sonra geri dönmemeli. Bu, kök nedeni çözmez ama semptomu görünür kılar.

---

## KAR-02

### KILL durumundan çıkış yok — oturum sonuna kadar kalıyor

**Şiddet:** 🔴 Kritik

**Kanıt:**

| Oturum | `KILL` örnek | toplam örnek | oran | KILL'e giriş | çıkış |
|---|---:|---:|---:|---|---|
| `session_20260811_163939` | 13.624 | 16.652 | **%81,8** | `t=1786455889,54` | **yok** |
| `session_19700101_020215` | 156.957 | 227.125 | **%69,1** | — | **yok** |

`session_20260811_163939` durum geçiş dizisi — oturumun **tamamı** 4 geçişten ibaret:

```
t=1786455586,752  BOOT
t=1786455593,234  ARM
t=1786455593,335  BEKLEMEDE
t=1786455889,543  KILL        <- 28 dakikanın kalanı burada
```

**Bulgu:** KILL'e girildikten sonra **hiçbir oturumda** çıkış gözlenmedi. Araç
KILL'e düştüğü an, kalan test süresi (bu oturumda 23 dakika) tamamen ölü geçti.

Dikkat çekici: aynı oturumda RC kanalları **sağlıklı** okunuyor ve
**ch8 (kill-switch) tüm oturum boyunca sabit `1000`** — yani kill-switch hiç
tetiklenmedi. Demek ki KILL'e geçiş RC'den değil **yazılım içinden** geldi.

**Kök neden:** KILL'den çıkış yolu ya yok ya da tetiklenemiyor. Bag verisi hangisi
olduğunu ayırt etmiyor; `fsm_node.py`'deki KILL geçişleri incelenmeli. Şu anki
davranış "tek yönlü kapı".

**Etki:** Bir kez KILL → oturum bitti. Yarışmada bu, turun kaybedilmesi demektir.

**Öneri:**
1. KILL'e **neden** girildiğini `/diagnostics`'e ve durum string'ine yaz
   (`KILL:odom_timeout`, `KILL:rc`, `KILL:watchdog` gibi). Şu an bag'den ayırt
   edilemiyor — en büyük teşhis boşluğu bu.
2. Yazılım kaynaklı KILL için **kurtarma yolu** tanımla: koşul ortadan kalkınca
   `KILL → BEKLEMEDE`. RC kaynaklı KILL tek yönlü kalmalı (güvenlik).
3. KILL'e giriş anını ayrı bir topic'e (`/girdap/mission/kill_reason`) bas.

---

## KAR-03

### BOOT kilitlenmesi — 25 dakika BOOT'ta, ama 10 Hz komut yayınlamaya devam

**Şiddet:** 🔴 Kritik

**Kanıt** — `session_20260811_171943` (17:19-17:44, 25 dakika):

```
/girdap/mission/state   : 16.967 mesaj, TAMAMI "BOOT", 0 geçiş
/girdap/parkur/state    : 16.967 mesaj, TAMAMI "PARKUR_1", 0 geçiş
/girdap/control/thrust  : 16.952 mesaj @ 9,999 Hz, TAMAMI [0.0, 0.0]
/girdap/fusion/odom     : 16.974 mesaj @ 10,001 Hz, TAMAMI (0,0,0)
/mavros/state           : 1 mesaj (connected=false)
mavros tanılama         : "disconnected" ×1.695, endpoint "closed" ×1.699
```

Aynı desen `session_20260811_022231` (14,8 s), `session_20260811_022259` (111,7 s),
`session_20260811_022452` (142,0 s) oturumlarında da var — hepsinde `mission/state`
tamamı `BOOT`, `mavros/state` tek mesaj, füzyon (0,0,0).

**Bulgu:** MAVROS Pixhawk'a hiç bağlanamadığında FSM `BOOT`'ta kalıyor — bu **doğru**
davranış. Sorun, sistemin geri kalanının **bunu umursamadan tam hızda çalışmaya devam
etmesi**: füzyon 10 Hz sahte odometri, kontrol 10 Hz sıfır itki üretiyor, kayıt
katmanı 25 dakika boyunca anlamsız veri yazıyor.

**Kök neden:** `BOOT` durumu alt katmanları **kapıya almıyor**. Düğümler FSM durumuna
bakmadan kendi timer'larında koşuyor.

**Etki:**
- 25 dakikalık test oturumu boşa gitti; operatör "sistem çalışıyor" görüntüsü aldı
  (topic'ler akıyor, Hz normal) ama hiçbir şey gerçek değildi. **Sahte yeşil.**
- 18,5 MB anlamsız bag kaydı.
- Gerçek arıza (MAVROS bağlantısı) topic akışının altında kayboldu.

**Öneri:**
1. `BOOT` durumunda füzyon ve kontrol **yayın yapmasın** (ya da açıkça `STALE`
   işaretli yayınlasın). Sessizlik, sahte veriden iyidir.
2. `sistem_baslat.sh` MAVROS bağlantısını başlangıçta doğrulasın: N saniye içinde
   `/mavros/state.connected == true` gelmezse **başlatmayı durdur ve ekrana bas**.
3. `BOOT`'ta 60 s'den fazla kalınırsa `/diagnostics`'e `ERROR` bas.

---

## KAR-04

### `/girdap/control/thrust` hiçbir oturumda sıfırdan farklı olmadı

**Şiddet:** 🔴 Kritik

**Kanıt** — incelenen tüm oturumlarda `min = max = 0.0`, doygunluk sayacı 0:

| Oturum | thrust mesajı | sıfır olan | min | maks | frekans |
|---|---:|---:|---:|---:|---|
| `session_20260811_171943` | 16.952 | **16.952** | 0,0 | 0,0 | 9,999 Hz |
| `session_20260811_143741` | 10.084 | **10.084** | 0,0 | 0,0 | — |
| `session_20260811_163939` | 2.415 | **2.415** | 0,0 | 0,0 | — |
| `session_20260811_145923` | 1.043 | **1.043** | 0,0 | 0,0 | 2,606 Hz |
| `session_19700101_020119` | 380 | **380** | 0,0 | 0,0 | 4,484 Hz |

Ham örnek: `t=1786457989,038 data=[0.0, 0.0]` — mesaj hep 2 elemanlı (sol/sağ motor),
hep sıfır.

**Bulgu:** Araç **hiçbir oturumda tahrik komutu almadı**. Bu, bulguların en doğrudan
sonucudur: 30.874 itki mesajı, sıfır hareket komutu.

**Kök neden:** Tek bir kök neden yok; her oturumda farklı bir kilit devrede:

| Oturum | Muhtemel kilit | Dayanak |
|---|---|---|
| `session_20260811_171943` | MAVROS bağlı değil, FSM `BOOT` | [KAR-03](#kar-03) |
| `session_20260811_163939` | FSM `KILL` (%82) | [KAR-02](#kar-02) |
| `session_20260811_143741` | FSM `ARM`/`BEKLEMEDE`, hiç PARKUR yok | [KAR-08](#kar-08) |
| `session_20260811_145923` | FSM `BEKLEMEDE` (%98,9) | [KAR-08](#kar-08) |

Ortak payda: **FSM hiçbir zaman "görev yürüyor" durumuna geçmedi**, dolayısıyla
kontrol katmanı hep güvenli varsayılana (sıfır itki) düştü. Bu davranış **doğrudur**;
asıl hata FSM'in ilerleyememesidir.

İkinci bir gözlem: `session_20260811_145923`'te thrust **2,606 Hz** ile yayınlanıyor ve
`/girdap/planning/edge_buoys` ile **birebir aynı zaman damgalarında** seyrekleşiyor.
Buna karşılık `session_20260811_171943`'te (algı hiç yok) thrust düzgün 10 Hz.

> 🔧 **DÜZELTME (2026-08-11, ikinci tur):** Bu gözlemden önce *"kontrol döngüsü
> sabit bir timer'a değil algı çıktısına bağlı koşuyor"* sonucu çıkarılmıştı.
> **Nedensellik ters yönde.** Kontrol döngüsü kendi timer'ında koşuyor
> (`planning_node.py:443`, `create_timer(1/control_rate_hz)`); `edge_buoys` ise bir
> **abonelik callback'inden** yayınlanıyor (`planning_node.py:716`, `_on_classified`).
> İkisi aynı **tek iş parçacıklı executor'ı** paylaşıyor. Kontrol adımı bütçeyi
> aştığında executor bloke oluyor ve algı callback'i **aç kalıyor** — yani kontrol
> algıyı yavaşlatıyor, tersi değil.
>
> Kesin kanıt `session_20260811_143741`'de, tek oturum içinde doğal bir A/B olarak
> duruyor: algı hattı ayağa kalkmadan önce (`classified_obstacles` = 0,00 Hz) thrust
> **9,86 Hz** koşuyor; algı akmaya başladığı anda thrust **9,09 → 3,07 → 1,03 Hz**'e
> çöküyor. Kontrol algıya *bağlı* olsaydı, algı yokken thrust hiç çıkmazdı.
> Ayrıntılı ölçüm ve mekanizma: [KAR-11](#kar-11).
>
> Öneri #2 bu yüzden değişti: sorun "döngüyü algıdan ayırmak" değil, **kontrol
> adımının bütçeye sığması** (ve/veya çok iş parçacıklı executor).

**Öneri:**
1. Önce [KAR-02](#kar-02), [KAR-03](#kar-03), [KAR-08](#kar-08)'i çöz — thrust'ın
   sıfır olması onların semptomu.
2. ~~Kontrol döngüsünü algı çıktısından ayır~~ → **düzeltildi:** döngü zaten kendi
   timer'ında; asıl iş kontrol adımını 100 ms bütçesine sığdırmak, bkz. [KAR-11](#kar-11).
3. `/girdap/control/thrust` yanında **neden sıfır olduğunu** yayınla
   (`/girdap/control/inhibit_reason`). Şu an "komut sıfır" ile "komut yok" ayırt
   edilemiyor.

---

## KAR-05

### Füzyon geçersiz veriyi geçerli gibi yayınlıyor — 16.974 mesaj (0,0,0)

**Şiddet:** 🔴 Kritik

**Kanıt** — `session_20260811_171943`, `/girdap/fusion/odom`:

```
mesaj      : 16.974 @ 10,001 Hz (dt = 100,0 ± 1,45 ms — kusursuz düzenli)
pozisyon   : x ∈ [0,00, 0,00]  y ∈ [0,00, 0,00]  z ∈ [0,00, 0,00]
sıfır poz  : 16.974 / 16.974  (%100)
NaN        : 0
kovaryans NaN: 0
hız        : maks 0,00 m/s
```

Aynı desen: `session_20260811_022231` (131 mesaj), `session_20260811_022259`
(1.115), `session_20260811_022452` (1.419) — hepsi %100 sıfır poz.
`session_19700101_020215`'te 10.559 mesaj sıfır poz (karışık oturum).

**Bulgu:** MAVROS bağlı değilken füzyon düğümü **durmuyor**; mükemmel düzenlilikte
(±1,45 ms) sıfır odometri yayınlıyor. NaN yok, kovaryans işaretlenmemiş, `stamp`
geçerli — yani **aşağı akıştaki hiçbir düğüm bunun geçersiz olduğunu anlayamaz.**

Karşılaştırma: `session_20260810_213017`'de `/girdap/fusion/odom` **143 mesajın
143'ünde `header.stamp = 0`** — orada en azından damga bozuktu; burada o ipucu da yok.

**Kök neden:** Füzyon düğümü, girdi yokluğunda son/varsayılan durumu yayınlamaya devam
ediyor. Geçerlilik sinyali (kovaryans şişirme, `child_frame_id` değişimi, ayrı bir
sağlık topic'i) yok.

**Etki:** Bu, [KAR-03](#kar-03)'teki "sahte yeşil"in en tehlikeli biçimi. Bir
operatör `ros2 topic hz /girdap/fusion/odom` çalıştırıp "10 Hz, sağlıklı" sonucunu
görür. Gerçek donanımda bu, aracın "ben orijindeyim ve duruyorum" diye ısrar etmesi
demektir — kontrol katmanı buna göre komut üretirse sonuç öngörülemez.

**Öneri:**
1. Girdi yoksa **yayınlama.** Tüketiciler timeout'la doğru davranışı seçer.
2. Yayınlamak zorunluysa **kovaryansı şişir** (örn. 1e9) — `robot_localization` ve
   `nav2` bu sözleşmeyi zaten anlar.
3. `/diagnostics`'e füzyon sağlık durumu bas: kaç saniyedir girdi yok.
4. Bir birim testi ekle: "girdi olmadan `fusion_node` odom yayınlamamalı".

---

## KAR-06

### Odometri ışınlanması — 25 ms'de 6,54 m (257 m/s)

**Şiddet:** 🟠 Yüksek

**Kanıt** — `session_19700101_020215`, `/girdap/fusion/odom` sıçramaları
(eşik: >1 m yer değiştirme **ve** >10 m/s örtük hız):

| t | mesafe | dt | örtük hız |
|---|---:|---:|---:|
| 1786421559,276 | 2,45 m | 0,035 s | 69,4 m/s |
| 1786421561,795 | 6,54 m | 0,067 s | 97,3 m/s |
| 1786421561,861 | 6,54 m | 0,066 s | 99,0 m/s |
| **1786421561,900** | **6,54 m** | **0,025 s** | **257,2 m/s** |

Toplam **60+** sıçrama (sayaç 60'ta dolduruldu). Eşzamanlı **yaw sıçramaları**:
55,4° / 35 ms, yine 60+ kez.

`session_20260811_143741`'de de 9 sıçrama var, maksimum hız 6,89 m/s.

**Bulgu:** Su üstü aracı için 257 m/s fiziksel olarak imkânsız. Poz tahmini
ışınlanıyor ve **yaw ile birlikte** ışınlanıyor (aynı zaman damgalarında) — yani
tam bir poz sıçraması.

**Kök neden:** Doğrudan [PAR-01](parametre.md#par-01) — aynı oturumda GPS'e
24.430 sahte mesaj enjekte edilmiş. Sıçrama zamanları (`t≈1786421559-1786421580`)
sahte GPS enjeksiyon penceresiyle (`t=1786421576,8` başlangıç) **çakışıyor**.
Füzyon, (41,0/29,0) ve (0,0) koordinatlarını gerçek ölçüm sanıp poz tahminini
oraya çekiyor, sonra gerçek GPS geri çekiyor — salınım.

Sahte mesajların **kovaryansı tamamen sıfır** olduğu için füzyon onlara
**sonsuz güven** atfediyor; bu, sıçramanın neden bu kadar sert olduğunu açıklıyor.

**Etki:** Konum tahmini kullanılamaz. Waypoint navigasyonu, geçiş doğrulaması
(`gate_count`) ve engel kaçınma hep bu poza bağlı.

**Öneri:**
1. [PAR-01](parametre.md#par-01)'i düzelt — kök neden orada.
2. Füzyon girişine **makullük kapısı** ekle: kovaryansı sıfır olan `NavSatFix`'i
   **reddet** (geçerli bir GPS asla sıfır kovaryans bildirmez).
3. Yenilik (innovation) kapısı: son poza göre `> v_max · dt` sıçratan ölçümü ele.
4. `frame_id` boş olan sensör mesajını reddet.

---

## KAR-07

### Füzyon 32 m'lik alanda 4.766 m "yol" biriktirdi

**Şiddet:** 🟠 Yüksek

**Kanıt** — `session_19700101_020215`:

```
kat edilen yol (ardışık poz farkları toplamı) : 4.765,8 m
gezinilen alan  : x ∈ [-15,55, 17,00]  →  32,6 m
                  y ∈ [-10,09, 16,96]  →  27,1 m
maksimum hız    : 2,50 m/s
```

Karşılaştırma: `session_20260811_163939` → 627 m yol, 13,0 × 13,6 m alan (oran ~46);
`session_19700101_020215` oranı **~146**.

**Bulgu:** Araç 33 × 27 metrelik bir kutunun içinde kalırken poz tahmini 4,77 km yol
biriktirmiş. Bu, gerçek hareketin değil **poz gürültüsünün** integralidir. Sinyal-gürültü
oranı o kadar düşük ki, tahmin edilen konumdan hız/yön türetmek anlamsız.

**Kök neden:** [KAR-06](#kar-06) ile aynı (sahte GPS enjeksiyonu), artı sıfır kovaryanslı
ölçümlerin filtreyi sürekli dürtmesi. `session_20260811_163939`'da (enjeksiyon yok)
oran 3 kat daha iyi ama hâlâ yüksek — yani **enjeksiyon dışında bir gürültü kaynağı
daha var**, muhtemelen GPS'in kendi çok yollu (multipath) hatası; alt 880 m ve
kovaryans ~3,8 m bunu destekliyor.

**Etki:** Türev alan her şey bozulur: hız tahmini, yön (heading), geçiş doğrulaması.
`gate_count`'un hiç artmaması bununla tutarlı.

**Öneri:**
1. [PAR-01](parametre.md#par-01) → [KAR-06](#kar-06) zincirini düzelt.
2. Düzeltme sonrası bu metriği **regresyon testi** olarak kullan: sabit dururken
   biriken yol < 5 m/dakika olmalı. Ölçmesi ucuz, çok şey yakalar.
3. GPS'i tek başına değil, IMU + hız ile sıkı bağlı (tightly-coupled) kullan.

---

## KAR-08

### Görev hiç PARKUR'a geçemedi; `mission/complete` hep `False`

**Şiddet:** 🟠 Yüksek

**Kanıt** — `/girdap/mission/state` dağılımı:

| Oturum | durum dağılımı | PARKUR'a geçti mi? |
|---|---|---|
| `session_20260811_143741` | `ARM` 14.644 · `BEKLEMEDE` 3.281 · `BOOT` 68 | ❌ |
| `session_20260811_145923` | `BEKLEMEDE` 3.950 · `BOOT` 44 · `ARM` 1 | ❌ |
| `session_20260811_163939` | `KILL` 13.624 · `BEKLEMEDE` 2.962 · `BOOT` 65 · `ARM` 1 | ❌ |
| `session_20260811_171943` | `BOOT` 16.967 | ❌ |

`/girdap/mission/complete`: `session_20260811_143741` → **8.943 mesajın tamamı `False`**;
`session_20260811_145923` → 1.977 `False`; `session_20260811_163939` → 8.300 `False`.
Hiçbir oturumda `True` görülmedi.

`/girdap/parkur/state` ise **her oturumda sabit `PARKUR_1`** (16.652 / 17.994 / 3.995 /
16.967 örnek, hepsinde 0 geçiş).

**Bulgu:** İki katman da ilerlemiyor. `mission/state` `BEKLEMEDE`'de takılıyor,
`parkur/state` `PARKUR_1`'de.

`session_20260811_145923` özellikle net: `BOOT → ARM → BEKLEMEDE` geçişi 4 saniyede
tamamlanmış (`t=1786449571,5` → `1786449575,9`), sonra **6,5 dakika boyunca
`BEKLEMEDE`**.

> 🔧 **DÜZELTME (2026-08-11, ikinci tur):** Bu paragraf önce *"aynı anda
> `/mavros/state` `armed=true` bildiriyor — yani araç kuruluydu"* diyordu.
> **Bu yanlıştı.** 14 oturumun tamamındaki 41.524 `/mavros/state` mesajı tarandı:
> `armed=true` olan **sıfır**, `guided=true` olan **sıfır** — bkz.
> [PAR-03](parametre.md#par-03). Araç bu oturumların hiçbirinde ARM edilmemiş.
>
> Bu, aşağıdaki "mod kenarı" kök nedenini geçersiz kılmaz ama **önüne bir adım
> ekler**: mod kenarı tartışması ancak araç ARM edildikten sonra anlamlıdır.
> Ayrıca bir tutarsızlık açığa çıkarır — `mission_fsm` `ARM → BEKLEMEDE` geçişini
> `armed`'a bağlar (`fsm_node.py:569`), dolayısıyla `armed` hiç `true` olmadıysa
> FSM `BEKLEMEDE`'ye **hiç geçmemeliydi**. Geçtiğine göre kaydedilen MAVROS verisi
> ile kaydedilen FSM durumu birbirini tutmuyor; bu da [KAR-01](#kar-01)'in
> "ikinci bir yayıncı" hipotezi için bağımsız kanıttır.

**Kök neden (iki ayrı):**
- **`BEKLEMEDE` takılması:** Görev başlatma **mod kenarı** (edge) tetikli, seviye
  değil. ARM'dan **önce** GUIDED'a geçilmişse kenar hiç oluşmaz ve görev hiç başlamaz.
  Bag imzası bunu doğruluyor: `state = BEKLEMEDE` sabit **+** `armed = true`.
- **`PARKUR_1` takılması:** `hardware.yaml`'da `mission_source: "fc"` ile çoklu parkur
  içeren `mission_file: competition_mission.yaml` birlikte tanımlı; FC waypoint'leri
  her zaman `parkur=1` döndürüyor. `fsm_node` bu çelişkiyi `ERROR` olarak logluyor ama
  düzeltemiyor — bkz. [PAR-09](parametre.md#par-09).

**Etki:** Görev hiç başlamadığı için tüm alt katman (planlama, kontrol, aktüasyon)
boşta. [KAR-04](#kar-04)'ün doğrudan nedeni.

**Öneri:**
1. Başlatmayı **seviye** tetikli yap (`armed && guided` → görev aktif), kenar değil.
   Kenar tetikleme sıralamaya duyarlı ve sahada iki kez kaybedildi.
2. `BEKLEMEDE`'de 30 s'den fazla kalınırsa **neden beklendiğini** `/diagnostics`'e bas
   (`arm yok` / `guided yok` / `waypoint yok` / `odom yok`).
3. `mission_source` ile `mission_file` çelişkisini **başlangıçta reddet** — `ERROR`
   loglayıp devam etmek yerine başlatmayı durdur.

---

## KAR-09

### Tüm hat 8-12 saniye donuyor

**Şiddet:** 🟠 Yüksek

**Kanıt** — `session_19700101_020215`, saat sıçraması ayıklandıktan sonra kalan
gerçek kesintiler:

| Topic | nominal | en büyük kesintiler |
|---|---|---|
| `/mavros/global_position/global` | ~7 Hz | **12,359 s** · 11,373 s · 11,307 s |
| `/mavros/imu/data` | ~7 Hz | 12,353 s · 11,371 s · 11,306 s |
| `/mavros/local_position/pose` | ~7 Hz | 12,229 s · 11,533 s · 11,453 s |
| `/girdap/mission/state` | 10 Hz | **9,050 s** · 7,713 s · 7,605 s |
| `/girdap/fusion/odom` | 10 Hz | 8,152 s · 8,075 s · 7,784 s |

Donmalar `t≈1786417125`, `t≈1786420246`, `t≈1786421291` civarında **kümeleniyor** —
yani tüm topic'ler aynı anda duruyor.

**Bulgu:** Bu, tek bir düğümün değil **tüm sistemin** donmasıdır (MAVROS dahil).
8-12 saniye, bir USV için çok uzun; 2 m/s hızda 24 metre kör uçuş demektir.

**Kök neden:** Bag verisi tek başına ayırt etmiyor. En olası adaylar:
- Sistem geneli kaynak baskısı (CPU/bellek/disk G/Ç). Bu oturumda 14 GB'lık kardeş
  bag (`session_19700101_020120`, 4,9 milyon mesaj) **eşzamanlı** kaydediliyordu —
  disk G/Ç doygunluğu güçlü aday.
- Jetson termal kısıtlama.

Not: `session_20260811_163939`'daki 1,5 s'lik donmalar ([ALG-07](algi.md#alg-07))
daha küçük ve LiDAR'ı etkilemiyor — farklı bir olay, muhtemelen algı yükü.

**Öneri:**
1. Kayıt yükünü düşür: `/livox/lidar` ham bulutunu **her karede** kaydetme (14 GB'ın
   3,6 milyon mesajı bu topic'ten) — ya da ayrı diske yaz.
2. Oturum sırasında `tegrastats` kaydı al; termal/CPU korelasyonu ancak böyle kurulur.
3. Kritik düğümleri (`fsm_node`, `mavros`) gerçek zamanlı önceliğe al.

---

## KAR-10

### FCU'ya hız komutu neredeyse hiç gitmedi — 5 saatte 110 mesaj

**Şiddet:** 🟠 Yüksek

**Kanıt** — `/mavros/setpoint_velocity/cmd_vel_unstamped`:

| Oturum | mesaj | süre | ortalama | en büyük sessizlik |
|---|---:|---|---:|---:|
| `session_19700101_020215` | 110 | ~5 saat | 0,027 Hz | **1.805,6 s** (30 dk) |
| `session_19700101_020120` | 40 | — | — | — |
| `session_20260811_154109` | 855 | 3.457,8 s | 0,247 Hz | — |

Değer aralığı (`session_19700101_020215`): `linear.x ∈ [0,0, 0,8]`,
`angular.z ∈ [0,0, 0,5]`, **110 mesajın 53'ü tamamen sıfır**.

**Bulgu:** Aktüasyon köprüsü fiilen sessiz. MAVROS setpoint akışının **sürekli**
olması gerekir (ArduPilot GUIDED modunda setpoint akışı kesilirse failsafe devreye
girer); 30 dakikalık boşluklar bunun hiç kurulmadığını gösteriyor.

Diğer oturumlarda (`session_20260811_143741`, `145923`, `163939`, `171943`) bu topic
**hiç yok** — yani köprü o oturumlarda hiç yayın yapmadı.

**Kök neden:** [KAR-04](#kar-04) ile aynı zincir: FSM görev-aktif olmadığı için
`mavros_bridge_node`'un görev-aktif geçidi (`mavros_bridge_node.py:220`) hiç açılmıyor.

**Etki:** Araç hareket etmiyor. Hareket etse bile setpoint akışı seyrek olduğu için
ArduPilot failsafe'e düşerdi.

**Öneri:**
1. Zincirin başını çöz ([KAR-08](#kar-08)).
2. Görev aktifken setpoint'i **sabit frekansta** (≥ 5 Hz, tercihen 10 Hz) yayınla —
   komut değişmese bile. ArduPilot bunu bekler.
3. Setpoint akışı 0,5 s'den fazla kesilirse `/diagnostics`'e `ERROR` bas.

---

## KAR-11

### Kontrol döngüsü 10 Hz bütçesini tutturamıyor; periyot oturum boyunca 10 katına çıkıyor

**Şiddet:** 🔴 Kritik

**Kanıt** — `/girdap/control/thrust` yayın periyodu (kontrol döngüsünün gerçek adım
süresi), oturum boyunca 10 dilimde ölçüldü. Hedef periyot **100 ms**
(`control_rate_hz: 10.0`, `params.yaml:40`):

| Oturum | başlangıç | orta | son | bozulma |
|---|---:|---:|---:|---:|
| `session_20260811_163939` | 117 ms | 522 ms | **1.062 ms** | 9,1× |
| `session_20260811_151706` | 111 ms | 639 ms | **979 ms** | 8,8× |
| `session_20260811_145923` | 145 ms | 341 ms | **554 ms** | 3,8× |
| `session_19700101_020119` | 141 ms | 235 ms | **326 ms** | 2,3× |

Hz cinsinden aynı veri (`session_20260811_163939`): 3,94 → 1,94 → 1,54 → 1,25 → 1,30
→ 1,01 → 1,10 → 1,07 → 0,73 → **0,61 Hz**.

**Girdi yükü sabit.** Aynı pencerelerde `classified_obstacles` kare başına tespit
sayısı **107-130 arasında** dalgalanıyor, artmıyor; `/livox/lidar` tertemiz 10,00 Hz.
Yani yavaşlama girdi artışından değil, **biriken bir iç durumdan** geliyor.

**Doğal A/B deneyi** — `session_20260811_143741`, tek oturum, aynı donanım:

| dilim | thrust | `classified_obstacles` |
|---|---:|---:|
| 1-5/10 | **9,86 → 9,09 Hz** | **0,00 Hz** (algı hattı yok) |
| 6/10 | 3,07 Hz | 8,00 Hz (algı ayağa kalktı) |
| 7/10 | **1,03 Hz** | 6,90 Hz |
| 8-10/10 | 1,19-1,28 Hz | 9,64-9,91 Hz |

Algı hattı yokken kontrol döngüsü bütçeyi **tutturuyor**; algı akmaya başladığı anda
çöküyor. `session_20260811_171943` (algı hiç yok, 50 dk) bunu doğruluyor:
thrust **10,00 Hz, sıfır boşluk**.

**Kök neden:** İki katmanlı.

1. **Taban maliyet zaten bütçe üstünde.** Döngü daha ilk dilimde 111-145 ms sürüyor.
   `control_rate_hz: 10.0` seçilirken dayanak CLAUDE.md'deki *"CPU'da K=1000 rollout
   ~100 ms"* ölçümüydü; ama o ölçüm `n_ref≈114`'lük demo sahnesinde ve geliştirme
   makinesinde yapılmış. **Jetson'da hiç doğrulanmamış.** Bkz. [PAR-05](parametre.md#par-05).

2. **Zamanla biriken maliyet.** Periyot monoton büyüyor. En güçlü aday, kaynağın
   kendi uyarısı: `EdgeBuoyMemory`'de **unutma yok** (kaptan kararı 09.08) ve
   *"konum sıçraması çakışma bandının üçte birini geçince aynı duba ikinci kayıt
   açıyor ve torba sınırsız büyüyor; bedeli `_huni_payi`'nin O(n²) saf Python
   taramasında"* (`planning_node.py`).

   ⚠ **Bu mekanizma henüz kesinleşmedi.** `/girdap/planning/edge_buoys` dizi
   uzunluğu ölçüldü ve **açıklamaya yetmiyor**: `session_20260811_163939`'da
   dizi tüm oturum boyunca **1 elemanda sabit** kalırken periyot yine de 9 katına
   çıkıyor. Muhtemel açıklama, torbada asıl büyüyenin **yayınlanmayan** kısım
   olması (`UNKNOWN` sınıflı hatırlanan cisimler; `edge_buoys` yalnız sınıf 0
   olanları taşır ve sınıflandırma zaten çalışmıyor — [ALG-01](algi.md#alg-01)).
   Doğrulama için düğüm içi sayaç gerekiyor, bag'den okunamıyor.

**Neden tek iş parçacığı da suçlu:** `edge_buoys` bir abonelik callback'inden
(`_on_classified`, `planning_node.py:716`), `thrust` ise timer callback'inden
yayınlanıyor. İkisi aynı **tek iş parçacıklı executor'ı** paylaştığı için
kontrol adımı uzadıkça algı callback'i de aç kalıyor — ölçümde ikisinin frekansı
birebir örtüşüyor. Bkz. [KAR-04](#kar-04) düzeltme notu.

**Etki:**
- MPPI'nin ürettiği komut hesaplandığı ana ait değil: 1 saniyelik periyotta 1 m/s'lik
  araç **1 metre** yol almış olur. Engel kaçınma payı (`obstacle_margin` 1,0 m) bu
  gecikmenin altında kalıyor.
- CLAUDE.md'nin kendi uyarısı (*"executor birikir, cmd_vel gecikir/titrer → istemsiz
  hareket"*) 20 Hz için yazılmıştı; ölçüm **10 Hz'de de** geçerli olduğunu gösteriyor.
- ArduPilot GUIDED setpoint akışının sürekliliği bekleniyor; 0,6 Hz bu şartı ihlal
  eder ([KAR-10](#kar-10) ile aynı sonuç).
- Bugün araç hiç ARM edilmediği için ([PAR-03](parametre.md#par-03)) bu bulgu **henüz
  can yakmadı** — ama ilk gerçek otonom koşuda doğrudan çarpışma riskidir.

**Öneri:**
1. Kontrol adımına **süre ölçümü + aşım sayacı** ekle; periyot aşılırsa
   `/diagnostics`'e bas. Şu an aşım tamamen görünmez — bu bulgu ancak bag'den
   dolaylı olarak çıkarılabildi.
2. Jetson'da profille (`cProfile`, 60 s gerçek kayıt) ve `_huni_payi` O(n²)
   taraması doğrulanırsa uzamsal indekse (KD-ağacı) geçir.
3. `EdgeBuoyMemory`'ye **sınır** koy: ya kapasite tavanı (LRU), ya yaş tabanlı
   budama. "Unutma yok" kararı kapı direklerini kaybetmemek içindi; yerel harita
   penceresi dışındaki kayıtlar için bu gerekçe geçerli değil.
4. `K`'yı 1000 → 400 indirip Jetson'da yeniden ölç ([PAR-05](parametre.md#par-05)).
5. Orta vadede `MultiThreadedExecutor` + ayrı callback group: kontrol adımı algı
   callback'ini aç bırakmasın.

# PARKUR-1 koşu kontrol listesi (su testi / yarışma P1 bacağı)

> Kapsam: `son_kodv2` yığınının Parkur-1'i koşabilir hâle gelmesi.
> Son güncelleme: **2026-08-08**. Ölçümler bu tarihte, repo `ea99edf` üzerinde
> alındı (kapalı-döngü simülasyon; koşum betiği ve sayılar §4'te).
>
> P1 puanı **GPS noktasına basmaktan değil, iki kenar dubasının ARASINDAN
> geçmekten** gelir (md 5.5.2.2 / 5.5.4.2) ve **P1 tamamlama şartı geçiş
> puanı ≥ 5**'tir. Yani "waypoint'lere vardık" P1'i bitirmez.

---

## 1. Zincir — P1'de puanı üreten yol

```
OAK-D (VPU'da YOLO)          Livox Mid-360
   │ /perception/buoys          │ /livox/lidar
   │ (2D bbox + SINIF)          ▼
   │                    perception_lidar_node
   │                       │ /perception/obstacle_map (3D, SINIFSIZ)
   └────────┬───────────────┘
            ▼  perception_fusion_node (bearing eşleştirme)
      /perception/classified_obstacles
            ▼  planning_node._on_classified
   class 0 (KENAR) → engel torbasından ÇIKAR → GateFollower
   diğer/eşleşmeyen → CircleObstacle (engel)
            ▼  _refine_target: ham GN → KAPI NİŞANI
      RRT* → MPPI → cmd_vel → mavros → FC (GUIDED)
```

🔑 **Kamera olmadan P1 puanı YOKTUR — dahası, P1 KOŞUSU da yoktur.**
Kenar ve engel dubası aynı geometridedir, tek fark renktir (§14.2) → LiDAR
ayıramaz. Sınıf gelmezse `_classified_seen` False kalır ve **tüm dubalar
engel** olur. Ölçüldü (08.08, `test_parkur1_kapali_dongu` negatif kontrolü,
3 kapılı sahne): kapı takibiyle **3/3 GN + 3/3 kapı, sapma 0,04 m**;
sınıf olmadan **0/3 GN** — çünkü kapı ortasından kaçık GN, iki direğin ceza
halkasının içinde kalıyor ve RRT* "goal engel içinde" diye planı reddediyor
(§18 A1'in aynı kökü). Yani model gelmeden su testi yapılırsa tekne
kıpırdamaz ve bunun belirtisi yalnız log'daki tek satırdır.

---

## 2. Koşu öncesi — SIRAYLA

### 2.1 Model (bu adım olmadan devamı anlamsız)

- [ ] Eğitilmiş `.pt` elde → **tek komut**:
      `cd son_kodv2/algi && ./scripts/model_uret.sh /yol/best.pt`
      (4 shave · 416×416 · sınıf isimleri doğrulanır; DUR derse tekneye taşıma)
- [ ] `blob` + `config.json` → Jetson `/home/girdap/models/` (**USB ile**, WiFi yok)
- [ ] Masa teyidi: `python3 scripts/duba_kamera_test.py`
      → log satırı: `Model sınıf sırası: [...] → kenar=…, engel=…`
      **turuncu dubaya `kenar` demeli** (isimden çözülür, indeksten değil)
- [ ] `systemctl status girdap-algi` → **active** (T6; yığın bunu başlatmaz,
      `hardware.launch.py` içinde OAK sürücüsü BİLEREK yok — tek OAK, tek süreç)
- [ ] `ros2 topic hz /perception/buoys` → akıyor (≥8 Hz; ölçülen tavan 12,2)

### 2.2 Karar yığını config'i

- [ ] P1'i **tek başına** koşacaksan:
      `sudo cp scripts/girdap-karar-parkur1.conf /etc/systemd/system/girdap-karar.service.d/`
      (yarışma drop-in'i varsa ÖNCE kaldır — ikisi aynı anda kurulmaz)
- [ ] `journalctl -u girdap-karar | grep 'config overlay UYGULANDI'`
      → **İKİ satır**: `yarisma.yaml`, sonra `parkur1.yaml`.
      🔴 Satır yoksa yığın **VİDEO MODUNDA** koşuyor demektir (AUTO bekler,
      RRT* kapalı, parkur etiketi yok) — 08.08'e kadar servisin sessiz hâliydi.
- [ ] Görev noktalarını Mission Planner'dan FC'ye yükle (`mission_source: fc`).
      Etiketler `parkur1_mission.yaml`'dan gelir ve **hepsi 1**'dir → yanlışlıkla
      PARKUR_3 (kamikaze) moduna geçilmez.
- [ ] `ros2 topic echo /girdap/mission/state` → `PARKUR1` (koşu boyunca değişmemeli)

### 2.3 FC (suya girmeden, pervane sökük)

- [ ] `ARMING_REQUIRE=1` (🔴 0 iken yazılım KILL yolu ölü — §0.14b)
- [ ] `ARMING_CHECK=1` · `FS_THR_ENABLE` (RC alıcısı sökülüyse 0)
- [ ] Buzzer geri yazılı (`NTF_BUZZ_TYPES=5`, `NTF_BUZZ_VOLUME=100`)
- [ ] `GPS1_POS_*` / `INS_POS1_*` girildi mi (olcum_formu §0)

### 2.4 Teslim zinciri (md 4.2 — eksik dosya = 5 ceza/dosya)

- [ ] `ls -R ~/girdap_logs/` → dört dizin de doluyor mu (kamera · telemetri ·
      yerel harita · LiDAR)
- [ ] Jetson saati doğru mu (damga kareye yazılıyor — T3)

---

## 3. Koşu sırasında izlenecek üç kanal

| topic | ne söyler | beklenen |
|---|---|---|
| `/girdap/planning/gate_count` | **geçilen FARKLI kapı** (puan kanıtı) | her kapıda +1 |
| `/girdap/planning/gate` | kilitli kapının NİŞAN noktası | kapıya yaklaşırken dolu |
| `/girdap/mission/state` | parkur katmanı | hep `PARKUR1` |

`KAPI SEÇİLEMEDİ: …` WARN'ı görüyorsan sebep **algıdadır** (kapı seçiminde
ayarlanabilir eşik yok): dubanın biri görünmüyor ya da sınıfı kaçıyordur.

---

## 4. 🔴 ÖLÇÜM (2026-08-08) — sevk edilen MPPI ayarı P1'i BİTİREMİYOR

Kapalı-döngü koşum, `planning_node` zincirinin aynası (algı 10 Hz → kapı →
RRT* → MPPI 10 Hz → `step_rk4`), gerçek dinamik (log 58), 6 kapı + 6 GN,
kapı genişliği 4 m, GN kapı ortasından 1,2 m kaçık (md 5.5.2.2 hâli),
kamera 15 m / 69° (FOV dışındaki duba **engel** sayılır — füzyon sözleşmesi):

| `T` | `terminal_lookahead_m` | tohum 0 | tohum 1 |
|---|---|---|---|
| **50** | **15 (SEVK EDİLEN)** | **VARAMADI** 3/6 GN | **VARAMADI** 1/6 GN |
| 50 | 3 | BİTTİ 252 s, 5 kapı | BİTTİ 140 s, 3 kapı |
| 100 | 15 | BİTTİ 107 s, 3 kapı | **VARAMADI** 4/6 GN |
| 75 | 5 | BİTTİ 127 s, 2 kapı | BİTTİ 129 s, 2 kapı |

**Baskın değişken `terminal_lookahead_m`.** Kodun kendi kuralı
(`lookahead ≥ seyir_hızı × ufuk`) gerçek teknede **1,05 × 2,5 = 2,62 m** diyor;
sevk edilen 15 m, 7,5 m/s'lik HAYALİ tekneden kalmadır (§0.9c). Kontrol
noktası: kamera 360°/25 m yapılıp algı MÜKEMMELLEŞTİRİLDİĞİNDE de sevk edilen
ayar bir tohumda varamadı → sorun algıda değil, **ufuk/lookahead ayarında**.

⚠️ İkinci bulgu: koşumların çoğunda gövde payı **negatif** (direğe temas).
P1'de çarpma cezası kenar dubalarını da sayar (Ç1, 16 puan) → ayar seçilirken
yalnız "bitirdi mi" değil **pay** da bakılmalı.

⚠️ Üçüncü bulgu: `passed_gate_count` ile fiilen kirişten geçme sayısı iki
yönde de ayrışıyor (bir koşumda sayaç 3 / fiili 1). Sayaç **teşhis** kanalı,
puanı hakem sayıyor — ama saha teşhisinde buna güvenilemez. (§18 G1 maddesi.)

👉 **Karar kaptanın:** `planning.mppi_terminal_lookahead_m` sahada CLI'dan da
verilebilir (`ros2 launch … planning.mppi_terminal_lookahead_m:=3.0`), yani
suya girmeden A/B denenebilir.

---

## 5. Bu koşuda AÇIK kalan bilinen riskler

1. **Kapı direği yakın mesafede ENGELE dönüşüyor.** Kamera 69°; 4 m kapıya
   ~3 m'den yakınken iki direk de FOV dışına çıkar → sınıf kaybolur →
   `CLASS_UNKNOWN` → engel torbası → MPPI tam kapı ağzında dışarı iter.
   Kilit (B5 onayı) hedefi korur ama itme kuvveti kalır. Ölçüldü, kapatılmadı.
2. **`plan()` hâlâ kontrol thread'inde** (§18 P1) — A3 sıklığı düşürdü,
   bloklamayı kaldırmadı.
3. **Geri dinamiği doğrulanmadı** — model geri itkiyi ileriyle aynı sayıyor.
4. **iSAM2 kapalı** (`use_isam2: false`) — poz MAVROS EKF'inden geliyor. Bu
   bilinçli; iSAM2 gerçek GPS/IMU ile doğrulanmadı.

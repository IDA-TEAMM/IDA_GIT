# Karar tarafına bulgular — algı ekibinden

**09.08.2026** · Kaynak: `EyupEker1/girdap-ida-algi` → ayna `IDA-TEAMM/IDA_GIT`
`son_kodv2/algi/`. Bu dosya **bildirim**dir: karar tarafının dosyalarına
DOKUNMADIK ([[ida-git-ortak-alan-kurali]] gereği hata bulunursa sahibine
kanıtıyla bildirilir, düzeltmeyi sahibi yapar).

Aşağıda **üç grup** var: 🔴 açık bulgular · ✅ zaten sağlam olanlar (tekrar
kontrol etmeyin diye, gerekçesiyle) · 🔧 bizim tarafta düzelttiklerimiz.

---

## 🔴 1. Bench'te kanıtlanmış `sync_queue_size` düzeltmesi `son_kodv2`'de YOK

**Kaptanın kendi notu** (`son_kod/memory/ida-perception-canli.md`), gerçek
Livox + OAK ile koşulmuş:

> *"Kapalı alanda ~20k yoğun nokta → clustering YAVAŞ (~1-3.3 s/kare),
> `obstacle_map` stamp'i doğru ama GEÇ varıyor; `buoys` anlık → aynı-stamp'li
> kamera karesi varsayılan queue_size=10 (~0.6 s) penceresinden düşünce sync hiç
> tutmuyor, **`classified_obstacles` ÜRETİLMİYOR**."*
> Çözüm: `sync_queue_size` parametresi eklendi; queue=50 **kıl payı yetmedi**,
> **queue=100 ile füzyon üretmeye başladı**.

**Ama canlı hatta yok** (09.08, IDA_GIT@6552fc5 üzerinde doğrulandı):
```
grep -rn "sync_queue_size"  ->  IDA_GIT'in TAMAMINDA 0 sonuc
son_kodv2/karar/.../perception_fusion_node.py:104-108
    ApproximateTimeSynchronizer([lidar, camera],
                                queue_size=10,          # <-- SABIT
                                slop=float(p("sync_slop_s").value))
```
`slop` config'den geliyor, **`queue_size` gelmiyor** — parametre bile değil.
⇒ Düzeltme kaptanın kanonik deposunda (`~/Desktop/son_kod`) kalmış, ortak
alandaki canlı hatta taşınmamış görünüyor.

**Neden bildiriyoruz:** kırılırsa **belirti vermiyor** —
`classified_obstacles` hiç üretilmez → `planning_node`'un `_edge_buoys`'u boş
kalır → `gate_follower` ham GPS'e düşer → P1 (G1/KD1 ≥ 0,5) ve P2 (≥2 ikili)
şartları sağlanamaz.

⚖️ **Dürüst pay:** kaptanın notu *"bu gecikme KAPALI-ALAN artefaktı — açık suda
seyrek nokta → clustering ms'ler → default queue yeter"* diyor. Yani suda sorun
çıkmayabilir. Ama bu **ölçülmemiş bir varsayım** ve karşılığı tüm P1+P2.
Karar sizin; biz yalnız "fix var ama burada yok" durumunu raporluyoruz.

## 🟠 2. `ida-oak-d-lite.md` notu bugünkü mimariyle çelişiyor (bayat)

Not diyor ki: OAK'ı **`depthai_ros_driver`** ile aç · `i_pipeline_type: RGB` ·
*"bu config'te depth YOK, sadece RGB"* · *"kurulu python depthai **3.7.1**"*.

Bugünkü gerçek:
- Kamerayı **algı node'u doğrudan** açıyor (DepthAI **2.30.0.0**, v2 API);
  `depthai_ros_driver` **kullanılmıyor**.
- **Stereo derinlik ŞART** (menzil oradan geliyor). depthai **v3'te bu
  OAK-D Lite'ta stereo %0 üretiyor** — v2'ye inmemizin tek sebebi buydu.
- Tek OAK var: `depthai_ros_driver` koşarsa cihazı kapar → **algı node'u hiç
  açılamaz**, `/perception/buoys` hiç akmaz.

⇒ Kod riski değil **bilgi** riski: yarışma günü o nota bakıp sürücü açılırsa
algı ölür. Notun güncellenmesi ya da "ARTIK GEÇERLİ DEĞİL" başlığı yeterli.

## 🟡 3. `usbfs_memory_mb` uyarısı artık bizi bağlıyor

Aynı notta: *"`usbfs_memory_mb=16` (varsayılan, düşük) — **depth/yüksek
çözünürlük açılırsa 256'ya çıkarmak gerekebilir**."*
Biz artık **depth AÇIK + 12MP** (ispScale ile 1352×1014) kullanıyoruz.
Bu değer bizim repomuzda ve notlarımızda **hiç geçmiyordu**; PC'de ölçtük: **16**.
🔎 Jetson'da bakılacak (tek satır, cihaz gerekmez):
`cat /sys/module/usbcore/parameters/usbfs_memory_mb`

---

## ✅ ZATEN SAĞLAM — tekrar kontrol etmenize gerek yok (gerekçeleriyle)

| Konu | Durum | Kanıt (09.08, IDA_GIT@6552fc5) |
|---|---|---|
| `/perception/buoys` **tek yayıncı** | ✅ | Üretimde başka publisher yok; `create_publisher(... "/perception/buoys")` yalnız `prototype/tests/` içinde (2 yer) |
| Kamera sahipliği | ✅ | `hardware.launch.py:473` `use_onboard_camera` **default false** · `:592` `with_oak_driver` **default false** |
| Sözleşme değerleri | ✅ | `bearing_tolerance_rad: 0.15` (~8,6°) ve `sync_slop_s: 0.1` — `hardware.yaml:209,216` ↔ `params.yaml:245,253` **tutarlı** |
| Piksel uzayı | ✅ | Biz **1280×720** uzayında yayınlıyoruz; tüketicinin `camera_image_width_px` değeriyle uyumlu (E-1 04.08'de kapandı) |
| Sessiz ölüm koruması | ✅ | Füzyonda `_sync_watchdog` var: iki girdi akarken sync hiç ateşlemezse WARN basıyor — doğru tasarım, sahada tek görünürlük kanalı |
| **Saat tabanı** | ✅ **sorun DEĞİL** | Kaptanın bench ölçümü: *"Ham sensör stamp'leri aynı tabanda (fark ~27 ms), yani saat sorunu YOK — tek sorun gecikme."* ⇒ §1 bir **gecikme** sorunu, saat sorunu değil; slop 0.1 s bu sapma için yeterli |
| Eşleşmeyen LiDAR kümesi | ✅ | Güvenlik gereği atılmıyor, `class_id=99` ile geçiyor — doğru davranış (kamera düşerse çarpma önleme ayakta kalır) |

## 🔧 BİZİM TARAFTA DÜZELTİLENLER (bilginiz olsun)

| # | Neydi | Durum |
|---|---|---|
| 1 | **Algı node'u `durum_log`'da çöküyordu** — rclpy logger'a 3 konumsal argüman verilmişti; `TypeError` timer callback'inden `rclpy.spin()`'e çıkıp node'u öldürüyordu, hem de tam *"duba görüyorum ama kapı kuramıyorum"* anında | ✅ `girdap-ida-algi@9dbf83b` ↔ ayna `IDA_GIT@6552fc5`; AST regresyon testi eklendi, 129 test |
| 2 | **Stereo ölçemeyince tespit komple atılıyordu** (`z<=0.05 → continue`); su dokusuz/aynasal olduğu için `/perception/buoys` boş kalabilirdi | ✅ bbox genişliğinden pinhole yedeği (`menzil_coz`), `girdap-ida-algi@3018654`. Tanı sayaçları: `mono_menzil`, `menzil_yok` — sahada `mono_menzil` yüksekse **stereo suda ölçemiyor** demektir |
| 3 | **Başlatma zinciri 4 yerden kırıktı** (servis `WorkingDirectory` yok → boot'ta node hiç açılmıyordu; kurulum betiği depthai'yi v3'e yükseltiyordu) | ✅ `@605d713`; 🔴 Jetson'da **reboot testi hâlâ yapılmadı** |
| 4 | Yayınlanan bbox piksel uzayı 640 ↔ 1280 uyuşmazlığı | ✅ 04.08'de kapandı |

## 🔧 5. BİZDE DE BİR SAAT SORUNU VARDI — ✅ 09.08'de DÜZELTİLDİ

**Algı node'u süre ölçümlerinin hepsini duvar saatiyle (`time.time()`) yapıyordu.**
Jetson **RTC pilsiz** ve boot'ta saat geride açılıyor (iki kez ölçüldü: bir kez
~15 saat, bir kez bir gün). Kıyı yordamımız *"`date` ile doğrula, yanlışsa
`sudo date -s`"* diyor — yani **saat, node çalışırken düzeltiliyor**.
Duvar saati **ileri** sıçrarsa geçiş penceresi (`pass_bitis_t`) anında dolar
(G puanı), **geri** sıçrarsa `durum_log` tamamen susar (sahadaki tek görünürlük
kanalı) ve Dosya-1 kaydı durur (md 4.2 → geçersiz dosya = 5 ceza puanı).

✅ **Düzeltildi:** süre ölçümleri `time.monotonic()`'e taşındı (16 çağrı yeri),
mutlak anlar duvar saatinde bırakıldı. İki incelik:
- **Dosya-1'in görünen zaman etiketi** aynı değişkeni paylaşıyordu; ayrıldı
  (`t_duvar` = etiket · `time.monotonic()` = segment süresi). Körü körüne
  dönüştürseydik md 4.2'nin istediği etiketi bozacaktık.
- Sentinel'ler `0.0` → `-math.inf`: `monotonic()` boot'ta küçük olduğu için
  `0.0` açılışta **yanlış "taze tespit var"** üretiyordu (duvar saatinde,
  epoch ~1,7e9 iken bu hata görünmüyordu).

Regresyon testi: `test/test_saat_kaynagi.py` (AST; ROS/kamera gerektirmez) —
süre ölçümünde duvar saati kullanılırsa **ve** Dosya-1 etiketi yanlışlıkla
monotonic'e çevrilirse testler kırmızıya döner. 129 → **133 test**.
📌 Not: toplayıcımız (`scripts/oak_veriseti_topla.py` — 16.08'de repodan
kaldırıldı, git geçmişinde) bu ayrımı **zaten doğru** yapıyordu; hata yalnız
deploy node'undaydı.
⇒ **Sizden bir şey gerekmiyor**, yalnız haberiniz olsun.

---
<sub>Hazırlayan: algı ekibi (Eyüp). Kanıtların tamamı bu repoda ve
`girdap-ida-algi` commit geçmişinde. Sorular için: bulguların her biri
dosya:satır ile verildi, tekrar üretilebilir.</sub>

---

## `models/best.pt` KALDIRILDI (2026-08-10) — Sude'nin onayıyla

**Ne:** `son_kodv2/karar/ros2_ws/src/girdap_decision/models/best.pt`
(6.242.993 bayt · sha256 `047017d657ea50db…`) depodan çıkarıldı.
Eyüp bildirdi, Sude *"sen bilirsin"* dedi.

**Neden:** dosya **tek sınıflı** (`names={0:'duba'}`, yolov8n, imgsz 640,
2026-04-11). Bizim algı node'u sınıfları **isimden** çözüyor
(`_sinif_indeksleri_coz`, "kenar"/"engel" alt dizgisi). `{0:'duba'}` ile çözüm
tutmaz → yedek sabite düşer → **her duba "kenar" sayılır** → sarı engel dubası
geçit direği sanılır → yanlış kapı → **Ç2 çarpma cezası**, hata basılmadan.
Yani depoda duran bir tuzaktı.

**Silmeden önce doğrulandı — hiçbir şey kırılmıyor (4 kanıt):**
1. `setup.py` → `glob("models/*.pt")` boş dönerse setuptools **hata vermiyor**
   (test edildi). `setup.py`'ye **dokunulmadı**; ileride model eklenirse yine kurar.
2. `hardware.launch.py:45-58` `_default_localizer_model()` → dosya yoksa `""`
   döndürüyor. Kendi yorumu birebir: *"Model kurulu değilse … BuoyLocalizer
   güvenli mock'a düşer, **davranış bozulmaz**"*.
3. `perception_camera_node` modeli **yüklemiyor** — HSV kullanıyor, YOLO katmanı
   zaten **MOCK** (dosyanın kendi yorumu, `:12`).
4. `ida_topics/perception_node.py:13` → `YOLO('/root/best.pt')` — **mutlak yol**,
   depodaki dosya değil. Ondan etkilenmiyor.

**🔴 DİKKAT — asıl risk bu DEĞİL, hâlâ açık:**
İkinci yayıncı riskini kapatan şey bu dosya değil, **`use_onboard_camera`
bayrağı**. `hardware.launch.py:488` varsayılanı `"false"` ve
`perception_camera_node` `IfCondition` ile sarılı ✅ — ama bayrak `true` gelirse
o node `/perception/buoys`'a **HSV ile** basar (`:30`, `:153`) ve bizim
YOLO+stereo yayınımızla çakışır. `best.pt` olsun olmasın bu risk aynı.
⇒ **`use_onboard_camera` her zaman `false` kalmalı.** Sahada doğrulama:
```bash
ros2 topic info /perception/buoys      # Publisher count: 1 OLMALI
```

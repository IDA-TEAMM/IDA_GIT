# Hatalar — rosbag tabanlı arıza analizi

Bu bölüm, `~/girdap_logs/rosbag/` altındaki **14 ROS2 oturumunun (~45,7 GB, mcap)**
tek tek taranmasıyla çıkarılmış arıza kataloğudur. Her bulgu bag'den gelen sayısal
kanıta dayanır; kök neden `son_kodv2` kaynak koduna referansla verilmiştir.

Analiz tarihi: **2026-08-11**
Kapsam: 2026-08-10 21:30 → 2026-08-11 17:44 arası saha/masa testleri.

## Kümeler

| Dosya | Küme | Bulgu | Kritik |
|---|---|---|---|
| [algi.md](algi.md) | **Algı** — kamera, LiDAR, sınıflandırma, engel haritası | 7 | 3 |
| [karar.md](karar.md) | **Karar** — FSM, füzyon/odometri, kontrol, aktüasyon | 11 | 6 |
| [parametre.md](parametre.md) | **Parametre** — konfigürasyon, eşik, ortam, donanım köprüsü | 10 | 4 |

Toplam **28 bulgu**, bunlardan **13'ü kritik**.

## Yönetici özeti — en önemli 8 bulgu

| # | Bulgu | Küme | Etki |
|---|---|---|---|
| [PAR-03](parametre.md#par-03) | **Araç hiçbir oturumda ARM edilmedi** — 41.524 `/mavros/state` mesajının tamamında `armed=false`, `guided=false` | Parametre | KAR-04'ün tek ve yeterli açıklaması; bu veriden otonomi performansı çıkarılamaz |
| [PAR-01](parametre.md#par-01) | **Test düğümleri canlı ROS domain'ine sızıyor**; `/mavros/global_position/global`'a 24.430 sahte GPS mesajı enjekte edilmiş (41.0/29.0, 40.8002/29.3, 0/0) | Parametre | Füzyon zehirleniyor → odometri ışınlanıyor |
| [KAR-11](karar.md#kar-11) | **Kontrol döngüsü 10 Hz bütçesini tutturamıyor**; periyot 117 ms → 1.062 ms'e çıkıyor (9×), algı hattı ayağa kalkınca çöküyor | Karar | MPPI komutu hesaplandığı ana ait değil → çarpışma riski |
| [PAR-04](parametre.md#par-04) | **MAVLink akış hızı heartbeat bütçesinin altında** — `/mavros/state` 0,17 Hz (eşik 5 s) → oturumun %86'sı KILL | Parametre | Oturum tamamen kayıp |
| [KAR-04](karar.md#kar-04) | **`/girdap/control/thrust` incelenen hiçbir oturumda sıfırdan farklı olmadı** — 21.000+ mesajın tamamı `[0.0, 0.0]` | Karar | Araç hiç tahrik komutu almadı |
| [ALG-01](algi.md#alg-01) | Engellerin **%99,96'sı sınıf `99` (sınıflandırılamayan)** — 1.775.300 karşı 748 | Algı | Duba renk mantığı beslenmiyor |
| [ALG-02](algi.md#alg-02) | Engel bulutunun **%27'si aracın arkasında** (x<0), en yakını 0,15 m | Algı | Kendi gövdesini engel sanıyor |
| [KAR-02](karar.md#kar-02) | **KILL durumundan çıkış yok** — girildikten sonra oturum sonuna kadar kalıyor (%82) | Karar | Test oturumu ölü geçiyor |
| [PAR-02](parametre.md#par-02) | Sistem saati senkronsuz başlıyor; oturum ortasında **20.676 günlük** sıçrama | Parametre | Tüm zaman damgaları ve TF geçersiz |

## İncelenen oturumlar

| Oturum | Boyut | Süre | Mesaj | Durum |
|---|---|---|---|---|
| `session_19700101_020119` | 370 MB | 91,6 s | 10.631 | ✅ |
| `session_19700101_020120` | 13,6 GB | (saat sıçramalı) | 4.903.577 | ✅ |
| `session_19700101_020215` | 332 MB | (saat sıçramalı) | 1.344.971 | ✅ |
| `session_20260810_213017` | 503 KB | 25,0 s | 885 | ✅ |
| `session_20260811_022231` | 28 KB | 14,8 s | 440 | ✅ |
| `session_20260811_022259` | 83 KB | 111,7 s | 3.574 | ✅ |
| `session_20260811_022452` | 1,6 MB | 142,0 s | 4.545 | ✅ |
| `session_20260811_130029` | 1,1 GB | 5.784,7 s | 68.002 | ✅ (zstd — açılıp tarandı, bkz. [PAR-04](parametre.md#par-04)) |
| `session_20260811_143741` | 6,6 GB | 1.808,6 s | 219.056 | ✅ |
| `session_20260811_145923` | 1,6 GB | 406,3 s | 50.485 | ✅ |
| `session_20260811_151706` | 7,0 GB | 1.772,1 s | 239.498 | ✅ |
| `session_20260811_154109` | 8,5 GB | 3.457,8 s | 293.195 | ✅ |
| `session_20260811_163939` | 6,5 GB | 1.671,9 s | 223.534 | ✅ |
| `session_20260811_171943` | 18,5 MB | 1.504,1 s | 78.047 | ✅ |

## Yöntem

Analiz üç aşamada yapıldı:

1. **Envanter** — her bag `ros2 bag info -s mcap` ile tarandı; topic listesi, mesaj
   sayısı, süre çıkarıldı. Sayılmayan (`Count: 0`) topic'ler ilk şüpheli grup.
2. **Tam tarama** — [`araclar/analyze_bag.py`](araclar/analyze_bag.py) her mesajı
   deserialize edip akış halinde özetledi (bellekte mesaj tutmadan): frekans/boşluk,
   NaN/Inf, durum geçişleri, poz sıçraması, doygunluk, sınıf dağılımı, `header.stamp`
   ile bag alım zamanı farkı, saat sıçraması.
3. **Kanıt toplama** — şüpheli her bulgu için [`araclar/probe.py`](araclar/probe.py) ve
   [`araclar/gps_cluster.py`](araclar/gps_cluster.py) ile ham mesaj dizisi çıkarıldı;
   ardından kök neden `son_kodv2` kaynağında doğrulandı.

4. **Hızlı tarama** — bag'lerin 13'ü sonlandırılmamış olduğu için indeks/özet
   bölümleri yok ([PAR-10](parametre.md#par-10)) ve her okuma tam dosya taraması
   gerektiriyor (13,6 GB'lık oturum tek başına ~20 dk).
   [`araclar/mcapscan.py`](araclar/mcapscan.py) bunu aşmak için yazıldı: chunk
   gövdelerini **hiç okumadan**, yalnız `MessageIndex` kayıtlarından tam zaman
   serisini çıkarır (bu bag'lerde chunk'lar sıkıştırılmamış). 46 GB'ın tamamı
   **15 saniyede** taranıyor. Frekans/boşluk/durum sayımları bu araçla alındı;
   `analyze_bag.py` ile çakışan oturumlarda mesaj sayıları **birebir doğrulandı**.

Tekrar üretmek için:

```bash
source /opt/ros/humble/setup.bash
python3 araclar/analyze_bag.py <bag.mcap> -o ozet.json
python3 araclar/gps_cluster.py <bag.mcap> /mavros/global_position/global
python3 araclar/probe.py <bag.mcap> --topic /girdap/mission/state --n 40

# hızlı yol (ROS gerektirmez):
python3 -c "from araclar.mcapscan import Scan; s=Scan('<bag.mcap>'); \
  [print(t, len(ts)) for t,(ty,ts) in s.scan().items()]"
```

⚠ **Kayıt canlıyken ölçüm alma.** `session_20260811_171943` bu analiz sırasında
hâlâ kaydediliyordu (`ros2 bag record` PID 4994, `girdap-rosbag.service` aktif);
o oturumun sayıları ölçüm anına göre değişir. Tablodaki değeri bir anlık görüntüdür.

## Okuma notları

- **Şiddet:** 🔴 kritik (yarışmayı/aracı riske atar) · 🟠 yüksek (görev başarısız olur)
  · 🟡 orta (performans/teşhis kaybı).
- **Kanıt** satırlarındaki `t=` değerleri bag alım zamanıdır (Unix epoch, saniye).
- Bulguların bir kısmı **tek kök nedene** bağlanıyor; bunlar ilgili yerde çapraz
  referanslandı. Özellikle [PAR-01](parametre.md#par-01) (test sızıntısı) ve
  [PAR-02](parametre.md#par-02) (saat) birden çok semptomun ortak kaynağı.
- `/perception/gate_passed`'in hiç yayınlanmaması **arıza değildir** — kodda bilerek
  kapatılmış (`GATE_PASSED_YAYINLA = False`, `duba_gecis_navigator.py`). Bu yüzden
  katalogda yer almıyor; ayrıntı [algi.md](algi.md#not-arıza-olmayan-gözlemler).

## Değişiklik geçmişi

| Tarih | Değişiklik |
|---|---|
| 2026-08-11 | İlk sürüm — 14 oturum tarandı, 27 bulgu. |
| 2026-08-11 (ikinci tur) | `parametre.md` yazıldı (10 bulgu). `session_20260811_130029` (zstd) açılıp tarandı → [PAR-04](parametre.md#par-04). Yeni bulgu [KAR-11](karar.md#kar-11) (kontrol döngüsü bütçe aşımı). **İki düzeltme:** KAR-08'deki `armed=true` iddiası ölçümle çürütüldü ([PAR-03](parametre.md#par-03)), KAR-04'teki "kontrol algıya bağlı koşuyor" nedenselliği ters çevrildi ([KAR-11](karar.md#kar-11)). `araclar/` dolduruldu. |

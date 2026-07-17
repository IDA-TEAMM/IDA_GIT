# jetsongeneltest1_grafLog — F5.1 LiDAR çerçeve düzeltmesi grafikleri

`index.html` — tarayıcıda aç (self-contained, bağımlılık yok). Grafikler
**gerçek çekirdek çıktılarından** üretildi (uydurma veri yok):
`prototype/perception/lidar_obstacles.detect_obstacles` + `fusion.associate`,
sentetik sahneler `scene_orta` (5 duba + 200 su gürültüsü) ve
`scene_fusion_matched`.

## 4 grafik
1. **Yükseklik taraması** — `lidar_height_m` 0→0.6 taranınca tespit edilen engel
   sayısı. h=0 (düzeltme yok) 1 duba kaybeder; ≥0.02'de 5/5 stabil (geniş plato =
   ±cm ölçüm toleransı). Ölçülen değer 0.31 işaretli.
2. **base_link z dağılımı** — su yüzeyi gürültüsü `z<z_min`'de toplanıp elenir,
   duba gövdeleri `[z_min, z_max]` bandında korunur.
3. **Nokta bulutu → engel haritası** (üstten) — base_link'e taşınmış ham noktalar
   → clustering → 5 daire engel.
4. **Bearing füzyonu** — LiDAR+kamera → renk sınıfı (turuncu/sarı/bilinmiyor),
   renk + şekil + etiket üçlü kodlama.

## F5.1 özeti
Ham Livox noktaları SENSÖR (livox_frame) çerçevesinde gelir; z_min/z_max ise
base_link (su datumu) çerçevesinde. Dönüşüm olmadan su hizası dubaları z_min ile
yanlış eleniyordu. Yeni `to_base_link()` noktaları filtrelemeden önce base_link'e
taşır (`lidar_height_m` yükseklik + opsiyonel `lidar_pitch_rad` eğim).

**Ölçüm:** `lidar_height_m = 0.31 m` (2026-07-17, kuru montaj: optik merkez → tekne
tabanı). **Doğrulama:** tam suite 340 passed / 2 skipped (+5 TDD).

Kod: kardeş klasör `jetsonvideokod_geneltest1_log/` (güncel main, F5.1 dahil).

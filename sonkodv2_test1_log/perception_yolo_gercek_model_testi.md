# perception_camera_node + Gerçek YOLO Modeli Testi (2026-07-17)

**Amaç:** F-P.21/F-P.22 düzeltmelerinden sonra (`use_yolo_localizer` varsayılan
açık, yeni renk sınıfları) `perception_camera_node`'u **gerçek eğitilmiş bir
YOLO modeliyle** ve **gerçekçi (Gazebo-render) test görüntüleriyle** uçtan uca
doğrulamak — birim testlerin ötesinde, gerçek model dosyası + gerçek
görüntülerle canlı çalıştırma.

## Girdi

- **Model:** `~/Masaüstü/duba_dataset/runs/detect/train8/weights/best.pt`
  (8 eğitim koşusu arasından en iyi metrikler: precision=0.995,
  recall=0.976, mAP50=0.984, mAP50-95=0.822). Tek sınıf (`nc: 1,
  names: ["duba"]`) — girdap'ın `BuoyLocalizer` deseniyle birebir uyumlu
  (model yalnız KUTU bulur, sınıf/renk HSV ile belirlenir).
- **Görüntüler:** `~/Masaüstü/duba_dataset/images/val/` — 8 tutulan
  (held-out) doğrulama görüntüsü, Gazebo'da render edilmiş, sahnede
  turuncu + 2× sarı/zeytin-sarı + yeşil + kırmızı + kahverengi (büyük,
  bayraklı) dubalar bir arada.

## Kurulum

`perception_camera_node` gerçek modelle başlatıldı:
```bash
ros2 run girdap_decision perception_camera_node --ros-args \
  --params-file .../params.yaml \
  -p yolo_localizer_model_path:=/root/duba_best.pt
```
8 görüntü sırayla `/oak/rgb/image_raw`'a (gerçek `bgr8` `Image` mesajı olarak)
basıldı, `/perception/buoys` cevabı yakalandı.

## Sonuç — ÇALIŞIYOR

Tüm 8 görüntüde tutarlı, gerçek tespitler alındı (örnek, img_0018):
```
7 tespit:
  class_id=5 (kahverengi) score=0.96 bbox=(1400,451) 150x356
  class_id=1 (sarı)       score=0.89 bbox=(53,463)   64x145
  class_id=0 (turuncu)    score=0.88 bbox=(1305,463) 50x115
  class_id=1 (sarı)       score=0.85 bbox=(695,465)  46x109
  class_id=4 (yeşil)      score=0.82 bbox=(318,466)  36x79
  class_id=3 (kırmızı)    score=0.81 bbox=(864,467)  32x72
  class_id=2 (hedef)      score=0.90 bbox=(818,436)  48x48   ← MOCK, aşağıya bak
```

**Doğrulanan:**
- Gerçek eğitilmiş model + girdap'ın `BuoyLocalizer`/`classify_roi_color`
  hibrit yolu (F-S.9) uçtan uca çalışıyor — mock değil, gerçek `.pt` dosyası
  yüklendi ve gerçek kutu tahminleri üretti.
- **Yeni renk sınıfları (kırmızı=3, yeşil=4, kahverengi=5) gerçek, karmaşık
  bir sahnede doğru atandı** — bu oturumda eklenen F-P.21 kod değişikliğinin
  ilk gerçek-veri doğrulaması.
- 8 görüntü arasında bbox konumları tutarlı şekilde kayıyor (kamera/sahne
  hareketi) — aynı sonucun tekrarlanmadığı, her karenin gerçekten ayrı
  işlendiği doğrulandı (ilk denemede script hatası yüzünden 8 kare de aynı
  sonucu veriyordu — subscription/callback hatası düzeltildi, bkz. not).

**⚠ Not — class_id=2 (hedef) MOCK'tur:** `use_yolo=true` ama gerçek bir
hedef-sınıfı modeli (`yolo_model_path`) verilmedi, bu yüzden her karede
sabit, anlamsız bir merkez-kutu (818,436 48x48, score=0.90) dönüyor —
`YoloInference` mock modunun tasarım gereği davranışı. Gerçek hedef modeli
gelince bu satır gerçek tespite dönüşecek, şimdilik yok sayılmalı.

**⚠ Not — dataset/model repoya EKLENMEDİ:** `duba_dataset/` (görüntüler +
`best.pt`) `~/Masaüstü/`'de kalıyor, git'e commit edilmedi (büyük binary
dosyalar, ayrıca bu paket `son_kodv2`'nin kendi eğitim verisi değil). Gerçek
kullanım için `best.pt`'nin `perception.camera.yolo_localizer_model_path`
launch-arg'ıyla (ya da `params.yaml`'da) gösterilmesi yeterli — kod
değişikliği gerekmez.

## Script hatası ve düzeltmesi (öğrenilen ders)

İlk denemede 8 görüntü için 8 ayrı ROS2 subscription açılmıştı (döngüde);
bu, `/perception/buoys`'a gelen HER mesajın 8 callback'in TAMAMINI
tetiklemesine yol açtı — ilk cevap gelir gelmez tüm 8 sonuç slotu aynı
(yanlış) veriyle doldu. Düzeltme: TEK subscription, her görüntüden önce
`latest.clear()` ile önceki cevap atılıp yeni cevap beklendi. Bu, ROS2 test
script'i yazarken önemli bir genel tuzak — çoklu-mesaj döngülerinde
subscription'ı döngü İÇİNDE değil, döngü ÖNCESİNDE bir kez açıp durum
sıfırlama ile ilerlemek gerekiyor.

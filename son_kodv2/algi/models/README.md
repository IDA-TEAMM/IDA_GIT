# Modeller

Bu klasöre HubAI'den çevrilen NN Archive (`.tar.xz`) dosyaları konur.
Büyük oldukları için git'e dahil edilmez (`.gitignore`).

> ⚠️ **2026-07-10 durumu:** Bu klasör **boş** ve `MODEL_NNARCHIVE`'ın işaret
> ettiği `yolo11n_duba_rvc2.tar.xz` bu makinede **bulunamadı**. Ayrıntı ve
> yapılacaklar: [`../docs/bekleyen_girdiler.md`](../docs/bekleyen_girdiler.md) §B1.

## Kodun beklediği

`duba_gecis_navigator.py`:
```python
MODEL_NNARCHIVE = "/home/girdap/models/yolo11n_duba_rvc2.tar.xz"   # Jetson yolu
```
Giriş boyutu (416×416) arşivin **içinde** tanımlıdır, kodda ayarlanmaz.
Saha benchmark'ı: 10–14 FPS bandı, tipik ~11.6 (416×416 + stereo birlikte, VPU sınırı).

## Eldeki artefaktlar

| Dosya | Ne | Sınıflar | Not |
|---|---|---|---|
| `~/girdap_yolo/Gazebonew.pt` | **Gerçek eğitilmiş duba modeli** | `{0: Engel Dubasi, 1: Kenar Dubasi}` | 5.478.490 B, 06.04.2026. **Tek kopya, yedeksiz.** Adı simülasyon verisi şüphesi doğuruyor. |
| `~/Desktop/oakdlite/416x416yolov11n.tar.xz` | ❌ **stok COCO YOLO11n** — duba modeli DEĞİL | 80 COCO sınıfı (`person`, `bicycle`, …) | SHA256 `a87b573b…7318`. Jetson'a **atmayın**. |

## Sınıf sırası — dikkat

Eğitilmiş modelin sırası (`0=Engel, 1=Kenar`) koddaki yedek sabitlerin
(`KENAR_CLASS=0, ENGEL_CLASS=1`) **tersidir**. Bu yüzden kod artık indeksleri
sabitten değil, `sdn.getClasses()`'tan gelen **isimlerden** çözüyor
(`_sinif_indeksleri_coz`). Yeni bir model konurken:

1. Node'u başlat, `Model sınıf sırası: [...] → kenar=..., engel=...` logunu **oku**.
2. `ERROR: Sınıf isimleri çözülemedi` görürsen sınıf adlarında `kenar`/`engel`
   geçmiyordur — model yanlış ya da isimler değişmiş demektir.

## Yeni model koyarken

1. `.tar.xz`'yi bu klasöre (ya da Jetson'da `/home/girdap/models/`) koy.
2. SHA256'sını hesapla ve yukarıdaki tabloya işle.
3. `tar -xJf <dosya> -O config.json` ile `heads[0].metadata.classes` sırasını doğrula.
4. `MODEL_NNARCHIVE` yolunu güncelle.

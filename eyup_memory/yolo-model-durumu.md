---
name: yolo-model-durumu
description: "GİRDAP YOLO modelinin gerçek durumu — sınıf sırası TERS, dağıtılabilir NN Archive lokalde YOK, Desktop'taki arşiv stok COCO"
metadata: 
  node_type: memory
  type: project
  originSessionId: 7f762d05-ebe8-4ea3-9b25-133521740824
---

2026-07-10'da **birinci kaynaktan (dosyaların içinden) doğrulandı.** Önceki varsayımların çoğu yanlıştı. İlgili: [[girdap-ida-proje-durumu]], [[girdap-decision-entegrasyon]], [[sartname-ida-2026]].

## 🔴 1. SINIF SIRASI TERS

Gerçek eğitilmiş model: **`/home/eyup/girdap_yolo/Gazebonew.pt`** (5.478.490 B, 6 Nisan 2026).
`data.pkl` içindeki ham `names` sözlüğü:

```
{0: 'Engel Dubasi', 1: 'Kenar Dubasi'}
```

Kod (`duba_gecis_navigator.py:100-101`) ise:
```python
KENAR_CLASS = 0        # turuncu duba - parkur kenarı
ENGEL_CLASS = 1        # sarı duba - engel
```

**TERS.** Bu sırayla dağıtılırsa: geçit tespiti (karşılıklı iki TURUNCU kenar çifti) aslında iki SARI ENGEL'i çift sanar; `SINIF_ESLEME` karar yığınına ters `class_id` yayınlar. **Parkur-2 tamamen bozulur.**

`sdn.getClasses()` çağrılıyor (`:281`) ama **yalnızca loglanıyor** (`:327-331`), remap için KULLANILMIYOR. Log satırı hatayı ekrana basacak durumda — ama kimse gerçek modelle koşturup okumamış.

**Kesinlik notu:** Dağıtılan `.tar.xz` lokalde olmadığı için o arşivin sırası doğrulanamadı. Elimizdeki tek eğitilmiş duba modeli bu sırayı veriyor. Sahaya çıkmadan `getClasses` logu MUTLAKA okunmalı.

## 🔴 2. Dağıtılabilir duba NN Archive lokalde YOK

- Kod `MODEL_NNARCHIVE = "/home/girdap/models/yolo11n_duba_rvc2.tar.xz"` bekliyor — bu **Jetson** yolu (`/home/girdap`), bu makinede değil.
- `girdap-ida-algi/models/` **boş** (yalnız `.gitkeep` + README).
- `models/README.md` `yolo11n_duba_rvc2.tar.xz` / HubAI `ida-buoy-yolo11n` diyor — **bu makinede böyle bir dosya yok.**
- Jetson'da var mı → **DOĞRULANMADI, kontrol edilecek.**

## 🔴 3. Desktop'taki arşiv STOK COCO — duba modeli DEĞİL

`/home/eyup/Desktop/oakdlite/416x416yolov11n.tar.xz` (4.800.688 B, SHA256 `a87b573b764e76fb3168114a057f5c134a3139b99dd9c287aa6f351da4377318`)

`config.json` → `n_classes: 80`, `classes: [person, bicycle, car, ... toothbrush]`. Giriş 416×416 ✓, subtype yolov8, 3 çıkış head.

Yanındaki `yolo11n.pt` stok Ultralytics modeliyle **byte-byte aynı** (SHA256 `0ebbc80d...` = `/home/eyup/yolo11n.pt`).

Trash'teki 3 arşiv (`yolo11n.rvc2.tar.xz`, `yolo11n_s6.tar.xz`, `3605.tar.xz`) de **80 sınıf COCO**.

⚠️ Bu arşiv yanlışlıkla Jetson'a atılırsa tekne `person/boat/orange` tespit eder; `class 0 = person`, `class 1 = bicycle`.

## ✅ Yapılanlar (2026-07-10, commit `bd62d7c`, push edildi)

- **Kod düzeltildi:** `duba_gecis_navigator.py`'ye `_sinif_indeksleri_coz()` eklendi — indeksler artık `getClasses()`'tan gelen **isimlerden** çözülüyor (`"kenar"`/`"engel"` alt dizgi). Sabitler yalnız isimler okunamazsa yedek, o durumda gürültülü `ERROR`; çözülen sıra sabitlerden farklıysa `WARN`. Tüm kullanım yerleri `self.kenar_cls`/`self.engel_cls`/`self.sinif_esleme`'ye geçti. **5 senaryo doğrulandı** (gerçek sıra→kenar=1/engel=0, ters sıra, stok COCO→yedek+ERROR, boş, tek sınıf). `py_compile` ✓
- **`Gazebonew.pt` yedeklendi:** `/home/eyup/model_yedek/Gazebonew.pt`. SHA256 `64d9a0605bf7f04e29a35c96ed40471c09c212b6b488ed05caaa2cf58d48d079`. ⚠️ **AYNI DİSK** — harici disk/bulut yedeği HÂLÂ gerekli.
- **`models/README.md`** gerçek durumla yeniden yazıldı (var olmayan dosyayı tarif ediyordu).
- **`docs/bekleyen_girdiler.md`** oluşturuldu (commit `9fe8ae1` + `bd62d7c`).

## Kalan yapılacaklar

**⏳ ZAMANLAMA (Eyüp, 2026-07-11): NN Archive üretimi VİDEO SONRASINA ertelendi — birlikte yapılacak.** Gerekçe doğru: otonomi videosunda algı yok (md 3.3), tar.xz 21.07 teslimini bloke etmiyor. T1 işi.

1. 🔴 **Jetson'da `/home/girdap/models/yolo11n_duba_rvc2.tar.xz` var mı?** Varsa `tar -xJf … -O config.json` ile `heads[0].metadata.classes` sırasını oku. Yoksa `Gazebonew.pt`'den RVC2 arşivi üret (416×416, 6 shave) — video sonrası, Eyüp'le birlikte.
2. 🔴 `Gazebonew.pt`'yi **harici disk + buluta** yedekle (şu anki yedek aynı diskte).
3. 🟠 `Gazebonew.pt` neyle eğitildi? Adındaki "Gazebo" **simülasyon verisi** şüphesi doğuruyor — gerçek saha görüntüsüyle eğitilmediyse saha performansı bilinmiyor. Eğitim setini bul.
4. Sahada `Model sınıf sırası: [...]` logunu **oku ve teyit et**.

Detaylı liste: `girdap-ida-algi/docs/bekleyen_girdiler.md` (§B).

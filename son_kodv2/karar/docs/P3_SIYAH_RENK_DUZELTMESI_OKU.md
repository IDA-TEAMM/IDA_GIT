# PARKUR-3 HEDEF RENGİ: kahverengi → SİYAH (18.08.2026)

> Kural 10 kartı: **NE · NEDEN · GERİ ALINIRSA NE KIRILIR**.
> Değişiklik P3 kapsamındadır (13.08 Eyüp kararı). `gate_follower`, MPPI,
> P1/P2 yollarına **dokunulmadı**; renk boşken davranış bit birebir eskisi.

## 1. NE

| # | Değişiklik | Dosya |
|---|---|---|
| 1 | `CLASS_KAHVERENGI` → `CLASS_SIYAH` (**değer 5 aynı**) | `prototype/perception/camera_buoys.py` |
| 2 | `RENK_SINIFLARI`: `kahverengi/kahve/brown` → `siyah/black` | `prototype/mission/kamikaze_hedef.py` |
| 3 | `SECILEBILIR_SINIFLAR` = {kırmızı, yeşil, **siyah**} | aynı dosya |
| 4 | **YENİ** `kanonik_ad(sinif)` — yayınlanacak kanonik renk adı | aynı dosya |
| 5 | `_renk_yayinla` artık **kanonik** adı basıyor, ham metni değil | `girdap_decision/kamikaze_param.py` |
| 6 | mp4 overlay etiketi `KAHVERENGI` → `SIYAH` | `ida_topics/kamera_kayit_node.py` |
| 7 | 5 yeni koruma testi (mutasyonla doğrulandı) | `prototype/tests/test_kamikaze_hedef.py` |

## 2. NEDEN — iki ayrı kusur, ikisi de P3'ü sessizce sıfırlıyordu

Şartname **s.18**: hedef duba renkleri **RAL 3026 kırmızı · RAL 6037 yeşil ·
RAL 9005 SİYAH**. "Kahverengi" hiçbir maddede geçmiyor.

**Kusur A — "siyah" hiç kabul edilmiyordu.** Ölçüldü (çalıştırılarak):

    renk_to_class("siyah") -> HedefRengiHatasi: bilinmeyen hedef rengi

Zincir: `ros2 param set … kamikaze_target_color siyah` **reddedilir** →
`_renk_yayinla` hiç çağrılmaz → `/girdap/mission/hedef_rengi` **boş** →
`fsm_node.py:600` `p3_bekleniyor=False` → `mission_fsm.py:307` **PARKUR3'e
hiç geçilmez**. Hakem üç renkten birini söylediğinde P3 = 0.

**Kusur B — takma adlar aşağı akışta düşüyordu.** `kamikaze_param`
operatörün YAZDIĞI metni yayınlıyordu, `planning_node._on_hedef_rengi` ise
adı `renk_kodu.RENK_KOD`'da arıyor. Ölçüldü:

    RENK_KOD.get("kahverengi", 0) -> 0      # = "hedef atanmamış"
    RENK_KOD.get("red", 0)        -> 0
    RENK_KOD.get("black", 0)      -> 0

Yani kahverengi **kabul edilse bile** nişanı açmıyordu (ölü yol), ve
`red`/`green` yazan operatör de aynı sessiz deliğe düşüyordu.

### Düzeltme sonrası uçtan uca (ölçüldü)

    hakem der   -> class -> yayınlanan ad -> planning_node kodu -> p3_bekleniyor
    kirmizi/kırmızı/red   3    'kirmizi'         1  KIRMIZI-RAL3026   True
    yesil/green           4    'yesil'           2  YESIL-RAL6037     True
    siyah/black           5    'siyah'           3  SIYAH-RAL9005     True
    kahverengi/mor             REDDEDİLİR (açık hata)                 —
    turuncu/sarı               REDDEDİLİR (kenar/engel gerekçesiyle)  —

## 3. Neden yeni tablo EKLENMEDİ

`kanonik_ad`, `RENK_SINIFLARI` ile `RENK_KOD`'un **kesişiminden** türetiliyor.
Elle yazılan üçüncü bir tablo, bu arızanın kaynağının ta kendisiydi (iki tablo
tek başına doğru görünüyordu, kırık olan **aralarındaki bağdı**).

## 4. GERİ ALINIRSA NE KIRILIR

- Hakem **"siyah"** derse (1/3 ihtimal) renk hiç yüklenemez ⇒ FSM PARKUR3'e
  geçmez ⇒ **P3 = 0 (145 puan, toplamın ~%48'i)**, tek satır hata basılmadan.
- Operatör **"red"/"green"/"black"** yazarsa renk kabul edilmiş görünür ama
  nişan kapalı kalır — belirtisi yok, yalnız tekne hedefe gitmez.

## 5. Kalan açık madde (bu değişikliğin KAPSAMI DIŞI)

`hardware.yaml`'ın kendi uyarısı geçerliliğini koruyor: **YOLO 3/4/5 sınıfları
için eğitilmedi**, yani akışta `class_id 3/4/5` yoksa bu parametre tek başına
hiçbir tespiti hedefe taşımaz. P3'ün gerçek üreticisi bugün algı tarafındaki
**saf OpenCV** yolu (`/perception/targets`, `p3_hedef_bul.py` — siyahı RAL 9005
eşikleriyle destekliyor). Bu düzeltme o yolun **kapısını** açar (renk yüklenebilir
hâle gelir); tespitin kendisi oradan gelir.

## 6. `hsv_brown_lo/hi` (params.yaml) — bilerek DOKUNULMADI

HSV tespit hattı 16.08'de kaldırıldığında bu iki parametreyi okuyan kod da
gitti (`grep hsv_brown` → yalnız yaml). Ölü config; silmek bu değişikliğin
kapsamı değil, ayrı bir temizlik kalemi.

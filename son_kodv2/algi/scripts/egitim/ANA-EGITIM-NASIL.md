# ANA EĞİTİM — nasıl yapacağız (kart)

> Bu dosyanın **gerçeği** `~/girdap_EGITIM_HATTI/` içinde durur.
> `~/Desktop/ANA-EGITIM.md` ona **sembolik bağdır** — iki kopya ayrışmasın diye.
> Son güncelleme: **09.08.2026 22:10 (NİHAİ)** · Yarışma **20.08.2026**

---

## 0. TEK CÜMLE
Kareleri **416×416'ya EZ** → `yolo11n.pt`'den **300 epoch / patience 50** ile eğit
(**`hsv_s=0.35`**, `hsv_h=0.015`, gerisi varsayılan, `workers=2`) → `.pt` →
**4-shave düz blob** + `config.json` → **3 pazarlıksız kabul testi**.

> ✅ **09.08 22:05 — asıl eğitim BAŞLADI.** Log `~/girdap_EGITIM_HATTI/egitim.log`,
> çıktı `runs/detect/girdap_final_20260809_2205/weights/best.pt`.
> train 7.546 kare (%8 negatif) · valid 2.087 · YOLO11n 2,59 M parametre.

---

## 1. 🔴 ÖN ADIM — KARELERİ EZ (STRETCH). ATLANMASI PUAN KAYBI

Deploy kareyi **eziyor**: `setPreviewKeepAspectRatio(False)`
(`duba_gecis_navigator.py:412`). Ultralytics ise varsayılan olarak
**letterbox** yapıyor — kaynaktan doğrulandı:
`base.py:240 load_image(rect_mode=True) → r = imgsz / max(h0,w0)` (en-boy korur)
ve `rect_mode=False`'u yalnız RT-DETR kullanıyor.

⇒ Eğitim ile saha **ayrışıyor**: model eğitimde yuvarlak, sahada **1,33× dikey
uzamış** duba görüyor.

🔬 **09.08'de ÖLÇÜLDÜ** (v3 modeli, 400 insan-onaylı kare, IoU≥0,5):

| girdi geometrisi | KÜÇÜK kutu (<12 px) recall | BÜYÜK kutu |
|---|---|---|
| letterbox (eğitimdeki hâli) | **%79,1** | %99,4 |
| stretch (sahadaki hâli) | **%75,7** | %99,4 |

⇒ **3,4 puan**, tam da uzak dubada = puanı taşıyan bant.

**Çözüm:** kareleri eğitimden önce `cv2.resize(im,(416,416))` ile ez
(`INTER_AREA`). Kare girdide Ultralytics'in letterbox'ı hiçbir şey yapmaz
(`r=1.000`, dolgu yok). YOLO etiketleri normalize (0-1) ⇒ **etiket dosyalarına
DOKUNULMAZ**.

📌 Bu adımın sahibi eskiden Roboflow'du (*Preprocessing → Stretch to 416×416*).
Roboflow'dan çıkınca **adım sahipsiz kaldı** — bu yüzden kart yazıldı.
🎁 Yan fayda: 1352×1014 yerine 416 okunuyor ⇒ epoch **~2,5× hızlı** ve RAM
baskısı düşük (09.08'deki OOM donmasının sebebi tam buydu).

```bash
python3 ~/girdap_EGITIM_HATTI/1_veri_hazirla.py
# 416'ya ezer + sizintisiz blok bolme + guard band + data.yaml (kanonik isimler)
```

---

## 2. AUGMENTATION — ölçüldü, tahmin değil

**09.08 ablasyonu:** 3.806 eğitim / 600 val karesi, 30 epoch, tek değişken,
`seed=0`. Ölçüt: **küçük kutu (<12 px) recall'ü** = uzak duba = darboğazımız.
Val'in 4 versiyonu: `temiz` · `sari42` (yarışma dubası, ölçülen 42° tondan) ·
`soluk` (UV/pus/parlama, doygunluk ×0,55) · `sicak` (öğle güneşi, −6°).

| konfigürasyon | hsv_s | scale | persp | ORT | karar |
|---|---|---|---|---|---|
| **B (kazanan)** | **0.35** | 0.5 | 0 | **%84,3** | ✅ **AL** |
| A (v3'ün ayarı) | 0.7 | 0.5 | 0 | %83,5 | temel |
| D | 0.7 | 0.5 | 0.0005 | %83,4 | ⚪ nötr — ALMA (bedava değil) |
| E (üçü birlikte) | 0.35 | 0.30 | 0.0005 | %83,1 | ❌ temelin altında |
| C | 0.7 | **0.30** | 0 | %82,8 | ❌ **ZARARLI** |

### Sonuç: 3 öneriden **1'i doğru**
- ✅ **`hsv_s: 0.7 → 0.35`** — dört val setinin **dördünde de** kazandı.
  ⚠️ Bedeli var: yanlış kutu 887 → 952 (**+%7**) = Ç2 ceza kalemine dokunur.
- ❌ **`scale: 0.5 → 0.30`** — *"kritik"* denen madde en zararlısı çıktı (−0,7).
  Beklenti tersineydi (stride-8'in altına düşen hedefler gürültü olur denmişti);
  **ölçüm çürüttü** — ölçek jitter'ı aynı zamanda **1,5×'e büyütüyor** ve
  mozaikle birlikte güçlü düzenleyici.
- ⚪ **`perspective: 0.0005`** — fark yok. Fiziksel gerekçesi de yoktu: kamera
  tekneye sabit; yalpa **dönme**, baş-kıç **öteleme** — ikisi de keystone değil.
  Ultralytics rehberi: *"if the camera's point of view is consistent… you can
  likely skip geometric transformations such as perspective"*.

### DEĞİŞMEYECEKLER (memory'de gerekçesi var)
`hsv_h=0.015` (±5,4°; **h>7,5° olursa turuncu↔sarı kuyrukları çakışır**) ·
`flipud=0` · `degrees=0` (duba hep dik yüzer) · `fliplr=0.5` · `mosaic=1.0` ·
`close_mosaic=10` · `hsv_v=0.4`.
📌 `erasing=0.4` **ölü ayar** — kurulu pakette yalnız sınıflandırma hattında
(`dataset.py:753 → classify_augmentations`), tespit eğitimine girmiyor.

---

## 3. EPOCH — 300, patience 50

| dayanak | ne diyor |
|---|---|
| Kurulu paket varsayılanı | `epochs=100 · patience=100` |
| Ultralytics eğitim tavsiyesi | *"Start with **300 epochs**… `--patience 50`"* |
| Veri seti eşikleri | ≥1500 görüntü/sınıf · ≥10.000 örnek/sınıf → **ikisini de geçiyoruz** (12.754 kare, ~38.000 kutu ≈ 19.000/sınıf) |
| Kendi eğrimiz | 30 epoch **DOYMAMIŞ** (`box_loss` e20 0,794 → e30 0,641, hâlâ düşüyor) |

**Süre (gerçek ölçümden):** ablasyonda 3.806 kare × 30 epoch = 8,0 dk ⇒
**13,5 sn/epoch**. Final sette 10.152 kare ⇒ **~36 sn/epoch**:
100 → ~1,0 sa · 150 → ~1,5 sa · **300 → ~3,0 sa** (val açıkken ~3,6 sa).
⇒ Süre kısıt değil; **üst sınırı bol tut, erken durdurma karar versin.**

---

## 4. KOMUT

```bash
HSV_H=0.015 HSV_S=0.35 ~/girdap_EGITIM_HATTI/2_egit.sh
# icinde: yolo detect train model=yolo11n.pt data=.../data.yaml imgsz=416
#         epochs=300 patience=50 batch=16 device=0 workers=2 cache=False
# Betik once IKI KAPIYI kontrol eder:
#   1) kareler gercekten 416x416 mi (stretch adimi atlanmis olabilir)
#   2) bos RAM >= 2500 MB
```

### ✅ NİHAİ AUGMENTATION KARARI (09.08 gece, olculdu)
| ayar | deger | gerekce |
|---|---|---|
| `hsv_s` | **0.35** | sinif hatasi 62-66 ↔ 0.7'de 74-93; recall esit |
| `hsv_h` | **0.015** (varsayilan) | KAPATMAK kotu (h=0 → 76 hata); 0.007 ile fark yok |
| `scale` | 0.5 (varsayilan) | 0.30 zararli cikti |
| `perspective` | 0 | fark yok + fiziksel gerekce yok (kamera sabit) |
| negatif kare | **%8** | Ultralytics onerisi; 19,8'den indirildi, valid'e dokunulmadi |

🔴 **`workers=2` PAZARLIKSIZ.** 09.08'de `workers=8` iki koşuda 18 işçiye çıktı,
~22 GB istedi, 15 GB RAM + 0 swap ⇒ OOM → PC dondu, etiketleme kesildi.
Swap artık 8 GB ama kural yine `workers=2`.
🔴 Etiketleme ile eğitim **aynı anda koşturulmaz**.

---

## 5. EXPORT + 3 PAZARLIKSIZ KABUL TESTİ

```bash
# .pt -> ONNX -> DUZ blob, 4 SHAVE (superblob DEGIL - depthai 2.30'da yok)
python3 -m tools --imgsz 416 --use-rvc2 ...    # luxonis/tools
blobconverter ... shaves=4
```
1. `dai.OpenVINO.Blob(yol).numShaves == 4` — fazlası model **hiç yüklenmez**
2. `config.json` içinde sınıf **İSİMLERİ** olmalı: `['kenar_dubasi','engel_dubasi']`
   🔴 Yoksa node `KENAR_CLASS=0, ENGEL_CLASS=1` sabitine düşer; Roboflow export'u
   isim listesini **alfabetik** yazıyordu ⇒ turuncu↔sarı **sessizce takas** → Ç2
3. **Passthrough sınıf kabul testi** — cihazda kare al, PC'de `.pt` ile 1:1
   kıyasla; blob sıkıştırması sınıfları çökertmiş olabilir (mimariden bağımsız)

---

## 6. AÇIK — 09.08 gece durumu

**Kapananlar:** tohum tekrarları (5'er tohum: recall'de fark yok, sınıf hatasında
var) · G hedefli ton (fark yok, alınmadı) · epoch eğrisi (150'de bile plato yok,
%99'a epoch 106'da) · negatif oran %8'e indirildi.

**🔴 AÇIK KALANLAR — önem sırasıyla:**
- [ ] **Uçtan uca zincir testi** — algı → füzyon → `gate_follower` hiç birlikte
      koşmadı (model yoktu, artık var). Renk hatası puan düşürür; zincirin
      kopması **puanı sıfırlar**.
- [ ] **Eğitim verisi TEK OTURUMDAN** (`20260807_150930`). %88,9 recall bir
      **üst sınır**, saha değeri değil. Çözüm: **farklı noktada bir göl oturumu,
      tamamı holdout** — tek kare bile eğitime girmeyecek.
- [ ] Jetson: autostart doğrulaması · **reboot testi** · `girdap-veriseti`
      **KALINTI unit'i silinmiş olmalı** (toplayıcı 16.08'de repodan kaldırıldı;
      denetim: `systemctl list-unit-files | grep veriseti` → BOŞ). Her biri tek
      başına P1+P2'yi sıfırlar, belirti vermeden
- [ ] Kameraya **yağmur siperliği** (şartname md 4.1 sorumluluğu bizde;
      sette 0 yağmur karesi var)
- [ ] **Yarışma yeri** — md 5.1 "deniz kenarı **veya** göl/gölet"; deniz çıkarsa
      su rengi/parlama/dalga sette yok
- [ ] `luxonis/tools` klonlanmadı (export ön koşulu, internet gerekir)
- [ ] 🔴 Dubalarımızın gerçek **ÇAPI** ölçülmedi (mezura, 1 dk) — `DUBA_CAP_M`
      pinhole menzilini besliyor

## İlgili
`~/girdap_EGITIM_HATTI/1_veri_hazirla.py` · memory: `pc-memory/ana-egitim-recetesi`,
`pc-memory/yolo-mimari-secimi-rvc2`, `pc-memory/yolo-menzil-ve-giris-boyutu`,
`pc-memory/shave-butcesi-arastirmasi`, `veriseti-etiketleme-plani`

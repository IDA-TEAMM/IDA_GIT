# Eğitim hattı — repo kopyası (12.08.2026)

Bu dört dosya **12.08'e kadar yalnız Eyüp'ün PC'sinde** (`~/girdap_EGITIM_HATTI/`)
duruyordu, git'te değildi: PC ölse model üretme reçetesi giderdi. Kod olduğu için
repoya alındı. **Veri seti alınmadı** — 135-592 MB'lık kare klasörleri git'e
girmez (ham kareler `~/girdap_HAM_KARELER/` + USB'de, iki kopya).

| dosya | ne yapar |
|---|---|
| `1_veri_hazirla.py` | kareleri `--boyut`×`--boyut`'a **EZER** (stretch) + böler |
| `2_egit.sh` | Ultralytics eğitimi (varsayılan `IMGSZ=512`) |
| `3_kabul_testi.py` | blob + config + `.pt` kabul denetimi |
| `ANA-EGITIM-NASIL.md` | reçetenin gerekçeleri (tek doğru kaynak) |

## 🔴 Koşum yeri
Betikler **`~/girdap_EGITIM_HATTI/` altında** koşar (veri orada). Buradaki kopya
**referans ve yedek**; ikisi ayrışırsa `ANA-EGITIM-NASIL.md` hangisinin güncel
olduğunu söyler. Değişiklik yaparken **ikisini birden** güncelle.

## Ölçülmüş, pazarlıksız kısıtlar
- **Ön işleme = SIKIŞTIRMA (stretch), letterbox DEĞİL** — deploy
  `setPreviewKeepAspectRatio(False)` kullanıyor. Ayrışırsa uzak dubada **3,4 puan**
  recall kaybı. Eğitim ve deploy **ASLA ayrı değişmez**.
- **`IMGSZ` = deploy `NN_GIRIS`** (bugün **512**). `2_egit.sh`'in kare boyutu
  koruması artık `IMGSZ`'den **türetiliyor** — eskiden `"416x416"` diye sabit
  yazılmıştı ve 512'ye geçince **yeni eğitimi haksız yere durduruyordu**.
  🔑 Ders: *koruma, koruduğu değerden türetilmeli.*
- **`workers=2`** — 09.08'de `workers=8` iki koşuda 18 işçiye çıktı, ~22 GB istedi,
  15 GB RAM + 0 swap ⇒ OOM ⇒ PC dondu, etiketleme kesildi.
- **Export'ta `--reverse_input_channels`** — model RGB bekler, kamera BGR gönderir.
  Ölçüldü: eksikse recall **%96,8 → %43,0**, hata basılmadan.
  Cihazda doğrulaması: `scripts/kontrol3_kanal_sirasi.py`.
- **Etiketleme ile eğitim aynı anda koşturulmaz.**

## Varsayılan veri
`veri512_ton_full/` = 7.546 orijinal + 6.942 **dikişsiz ton kopyası** = 14.488 kare
(ep87 modelini üreten set). Ton kopyaları yarışma paletindeki 28-34° turuncu
boşluğunu doldurdu: video sınıf hatası **%18,8 → %3,3**.
Ton kopyasız: `VERI=~/girdap_EGITIM_HATTI/veri512/data.yaml`.

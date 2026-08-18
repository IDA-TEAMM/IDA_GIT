# ✅ ÇÖZÜLDÜ (17.08.2026) — ŞÜPHE KAPANDI, AMA BAŞKA BİR KUSUR ÇIKTI

> Bu dosya 17.08 gecesi *"dağıtılan blob'un kanal sırası doğrulanmadı"* şüphesiyle
> yazılmıştı. Şüphe **aynı gün ölçümle kapatıldı** ve sonuç **beklenenin tersi**:
> kanal sırası **DOĞRUYDU**, düşen bayrak **`--scale_values`**'tı.
> Dosya silinmedi — yanlış tarafa çekmesin diye **düzeltildi**.

## 🎯 SONUÇ — tek cümle

Eski blob (`6df2d644…`, ep135) **yalnız `--reverse_input_channels`** ile
derlenmiş; **`--scale_values`/`--mean_values` verilmemiş** ⇒ ağa 0..1 yerine
**0..255** giriyordu. **Bu blob artık dağıtımda değil.** Yerine aynı ONNX'ten
doğru parametrelerle derlenen **`c4d69ec7…`** kondu (bkz. `PROVENANS.json`).

## 🔬 NASIL KANITLANDI (cihaz gerekmedi, iki bağımsız yol)

**1) Derleme parametreleri geri getirildi.** `blobconverter` her isteğin
sha256'sını `~/.cache/blobconverter/.config.json`'a yazıyor; hash =
`parametre JSON + config yaml + ONNX baytları` (`blobconverter/__init__.py:253`).
Hash kaba kuvvetle yeniden üretildi. Yöntem **önce 4 eski blob'da doğrulandı**
(hepsi beklenen parametreleri verdi), sonra dağıtılana uygulandı:

| blob | derlemede verilen `optimizer_params` |
|---|---|
| `5726819a…` (416, 10.08) | scale + mean + **reverse** ✅ |
| `31fb0348…` (512 ep87, 11.08) | scale + mean + **reverse** ✅ |
| `6df2d644…` (512 ep135, 16.08) | **yalnız reverse** 🔴 |

**2) Yeniden derleme.** Aynı ONNX doğru parametrelerle derlendi →
**5.697.816 B**, ep87 blob'uyla **birebir aynı boyut**. Eski blob **5.682.840 B**
(14.976 B eksik = katlanmamış ölçek katmanı).

## 📉 BEDELİ — ÖLÇÜLDÜ (PC, ep135 `.pt`, 120 valid karesi)

ONNX'te normalizasyon **yok** (grafiğin ilk düğümü doğrudan `Conv`), kodda da
yok (`ColorCamera` U8 BGR → `setBlobPath`), `config.json`'daki `scale: 255`
**NNArchive metadata'sı** ve depthai 2.30 onu **okumuyor**:

| giriş | tespit | tespit olan kare | medyan güven |
|---|---|---|---|
| doğru (x/255) | 226 | 88/120 | 0,735 |
| eski blob (x, 0..255) | **36.000** | **120/120** | **0,906** |

36.000 = 120 × 300 = **`max_det` doyumu**. Yani belirti *"duba kaçırma"* değil,
**her karede yüzlerce yüksek güvenli uydurma kutu** ⇒ `/perception/buoys` çöple
dolar ⇒ füzyon + `EdgeBuoyMemory` zehirlenir ⇒ **P1+P2 = 0**.

## 🔴🔴 ARTIK YAPILMAMASI GEREKEN ŞEY

**`setColorOrder(RGB)` saha kurtarmasını UYGULAMA.** Bu dosyanın eski sürümü
onu öneriyordu. Dağıtımdaki blob'da kanal takası **zaten var**; RGB'ye çevirmek
**çift çevirme** olur ve recall'ü **%96,8 → %43** yapar. Kural değişmedi:
**(A) ve (B) birlikte uygulanmaz** — bugün geçerli olan **(A)**'dır.

## 🪤 NEDEN HİÇBİR KAPI YAKALAMADI (ve ne değişti)

- `3_kabul_testi.py` *"reverse uygulandı"* cümlesini **sabit metin** basıyordu
  → doğru ve bozuk blob **ikisi de testi geçti**. ✅ Artık **KONTROL 0**
  (`PROVENANS.json` ↔ blob sha256) ve blob giriş boyutu ↔ `NN_GIRIS` kontrolü var.
- `model_dogrula.py` shave/giriş/sınıf adına bakıyor; ölçek de takas da
  **ağırlıkların içinde**, çıktıdan görünmez.
- `config.json` yanıltıcı: **doğru** ep87 blob'unda da `reverse_channels: null`
  ve `scale: 255` yazıyor — o dosya derlemeyi değil ONNX'in *beklentisini* anlatır.
- 17.08'de `model_uret.sh`'nin reverse'ü hiç geçmediği bulunmuştu (`a4ca1e3`,
  doğru ve değerli düzeltme) — ama **bu blob o betikle üretilmemiş**: betik
  reverse'süz + ölçekliydi, blob tam tersi. Şüphenin **yönü** oradan şaştı.

🔑 **Ders:** *"bayrak listesi"* tek parça değil. Biri düşünce diğerinin varlığı
**yanlış güven** verdi. Tek gerçek koruma: parametreleri üreten betiğe kilitlemek
(`test_model_uretim_bayraklari.py`) **ve** üretilen blob'un sha256'sını
`PROVENANS.json`'a yazıp kabul testinde karşılaştırmak.

## 🔬 KONTROL 3 HÂLÂ YAPILACAK (bu bulgu onun yerine geçmez)

Kanal sırası artık **derleme kaydından** biliniyor, ama blob sıkıştırmasının
sınıfları çökertmediği (turuncu↔sarı) **hâlâ cihazda** doğrulanmalı:

```bash
sudo systemctl stop girdap-algi          # tek OAK — algı serbest bırakılmalı
python3 scripts/kontrol3_kanal_sirasi.py --kare 8 --kaydet /tmp/kontrol3
sudo systemctl start girdap-algi
```
Çıkış kodu: **0 = GEÇTİ** · **1 = TERS (🔴 DAĞITMA)** · **2 = KARARSIZ**
⚠️ **Kuru ortam tuzağı:** kadrajda duba yoksa iki taraf da sıfır tespit verir,
betik *"KARARSIZ"* der — bu **başarı değildir**.

🆕 Bu bulgudan sonra KONTROL 3'ün ikinci bir işlevi var: **ölçek kusurunun
belirtisi** de orada görünür — cihaz karede `.pt`'den **kat kat fazla** kutu
üretiyorsa (yüzlerce), sorun kanal değil **ölçektir**.

## 📌 DURUM

- Dağıtımdaki blob: **`c4d69ec7…`** (`PROVENANS.json`'da tam kayıt).
- 🔴 **CİHAZDAKİ blob hâlâ eski olabilir** — Jetson'a bu dosyayla birlikte
  `yolo11n_duba_rvc2.blob` + `config.json` + `PROVENANS.json` yeniden kopyalanmalı
  ve `sha256sum` ile teyit edilmeli.
- Yedekler: `~/models_yedek_512_ep87/` (`31fb0348…`) ve `~/models_yedek_416/`
  (`5726819a…`) — ikisi de bayraklı ve **ölçekli** derlenmiş, geri dönüş noktası.

## Ayrıntı
`GIRDAP_DURUM.md` §1.34 · `a4ca1e3` (bayrak düzeltmesi + testler) ·
memory: `pc-memory/blob-olcek-bayragi-eksik.md`

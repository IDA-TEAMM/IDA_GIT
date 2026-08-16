# 🔴 OKU ÖNCE — DAĞITILAN BLOB'UN KANAL SIRASI **DOĞRULANMADI**

> Yazan: 17.08.2026 · İlgili: `yolo11n_duba_rvc2.blob` (`6df2d644…`, ep135)
> Bu dosya blob'un YANINDA duruyor ki blob'a dokunan herkes önce bunu görsün.

## ⚠️ ÖNCE NE **KANITLANDI**, NE **KANITLANMADI** — karıştırma

**KANITLANAN (üçü de doğrulandı):**

1. `scripts/model_uret.sh` blobconverter'a `--reverse_input_channels` bayrağını
   **hiç geçmiyordu**. `git log -S "reverse_input_channels" -- scripts/model_uret.sh`
   → **boş**; bayrak o betiğe hiç girmemiş. *(17.08'de düzeltildi + 4 testle
   donduruldu, bkz. `test_model_uretim_bayraklari.py`.)*
2. Bu blob'un (`6df2d644…`, ep135) **nasıl üretildiği hiçbir yerde kayıtlı
   değil.** Önceki iki blob'un ikisi de bayrakla derlendiği **belgeli**:
   `5726819a…` (416) ve `31fb0348…` (512 ep87 — `README.md` export bloğunda
   bayrak açıkça yazılı).
3. Zincirde bunu yakalayacak **hiçbir otomatik kapı yoktu**: `model_dogrula.py`
   shave/giriş/sınıf adına bakıyor, kanal sırasını **göremiyor** (takas
   ağırlıkların içinde); `config.json`'daki `reverse_channels` alanı da işe
   yaramıyor — **bilinen İYİ** ep87 blob'unda da `None` (iki config yan yana
   konup doğrulandı).

**KANITLANMAYAN:**

> ❌ *"Bu blob yanlış."* — **Bilmiyoruz.** Betikle üretildiyse takassızdır;
> elle, bayrak verilerek üretildiyse doğrudur. Kayıt olmadığı için ayırt
> edilemiyor. **Şüpheli, kesin değil.**

## 🔬 NEDEN ÖNEMLİ

Model **RGB** bekler (ultralytics `augment.py` → `img[::-1]`), `ColorCamera`
**BGR** gönderir (`setColorOrder(BGR)`), blob'un içinde çeviren yoktur. Takas
derleme anında ilk konvolüsyon ağırlıklarına gömülür (çalışma maliyeti **0 ms**).

**Bayrak düşerse ölçülmüş bedel (600 kare, PC, `.pt` ile):**

| | recall |
|---|---|
| doğru kanal sırası | **%96,8** |
| ters kanal sırası | **%43,0** |

🔴 Ve **hiçbir hata basılmaz**: node açılır, FPS normaldir, tespitler gelir —
yalnız dubaların çoğu görünmez olur. Sahada teşhis edilemez.

## ✅ NASIL KARAR VERİLİR (tek yol)

Cihazda, **kamera takılıyken**, kadrajda **gerçek duba varken**:

```bash
sudo systemctl stop girdap-algi          # tek OAK — algı serbest bırakılmalı
python3 scripts/kontrol3_kanal_sirasi.py --kare 8 --kaydet /tmp/kontrol3
sudo systemctl start girdap-algi
```

Çıkış kodu: **0 = GEÇTİ** · **1 = KANALLAR TERS (🔴 DAĞITMA)** · **2 = KARARSIZ**

⚠️ **Kuru ortam tuzağı:** kadrajda duba yoksa iki taraf da sıfır tespit verir ve
betik "KARARSIZ" der — bu **başarı değildir**, tekrar koşulmalıdır.

## 🔧 YANLIŞ ÇIKARSA — İKİ ÇÖZÜM, **BİRLİKTE UYGULANMAZ**

### (A) Blob'u yeniden üret — *tercih edilen*

Betik **artık bayrağı geçiyor**, yani düzeltilmiş `model_uret.sh` ile üretilen
blob doğru olur. Ama **bu Jetson'da yapılamaz**:

- 🔴 **`.pt` bu makinede YOK** (17.08'de arandı, hiçbir yerde yok) — eğitim
  ağırlıkları PC'de/Roboflow'da.
- 🔴 **İnternet gerekir** — `blobconverter` bulut derleyicisidir. Yarışma
  alanında internet **yok** (md 4.1), yani bu **alana gitmeden önce** yapılmalı.

Üretim sonrası doğrulama: `dai.OpenVINO.Blob(yol).numShaves == 4` +
`config.json` sınıf isimleri `kenar_dubasi`/`engel_dubasi` + **KONTROL 3**.

### (B) Saha kurtarması — *internetsiz, tek satır*

Blob yanlış ve yeniden üretilemiyorsa:
`duba_gecis_navigator.py` → `cam_rgb.setColorOrder(...BGR)` **→ `RGB`**

⚠️ **Bedeli var:** Dosya-1 mp4 (şartname md 4.2, kutu çizimli kayıt) o zaman
**yanlış renkte** kaydedilir. Tespit doğru çalışır, teslim edilen video renk
olarak terstir. Bu yüzden (A) tercih edilir.

🚨 **(A) ve (B) ASLA BİRLİKTE UYGULANMAZ** — çift çevirme takası geri alır ve
tam başladığın yere dönersin (recall yine %43).

## 📌 DURUM ÖZETİ

- Cihazdaki blob ile repodaki blob **birebir aynı** (`6df2d644…`, config de aynı)
  ⇒ repo/cihaz ayrışması **yok**, bu taraf temiz.
- Algı node'u `/home/girdap/models/yolo11n_duba_rvc2.blob` yolunu okuyor.
- Yedekler duruyor: `~/models_yedek_512_ep87/` (`31fb0348…`, **bayraklı olduğu
  belgeli**) ve `~/models_yedek_416/` (`5726819a…`, aynı şekilde belgeli).
  🔑 **KONTROL 3 bu blob'da "TERS" derse**, ep87 yedeği bilinen-iyi bir geri
  dönüş noktasıdır — ama ep135'ten daha zayıf bir model (uzak recall farkı için
  `GIRDAP_DURUM.md` §1.23 öncesi kayıtlara bak). Karar kaptanın.

## Ayrıntı

`GIRDAP_DURUM.md` **§1.34** · commit `a4ca1e3` (bayrak düzeltmesi + testler)

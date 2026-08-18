# PARKUR-3 ALGI ENTEGRASYONU — geri almadan önce oku (17.08.2026)

> Kural 10 gereği: her değişiklik **NE · NEDEN · GERİ ALINIRSA NE KIRILIR**.
> Bu dosya `karar/` tarafına **dokunmuyor**; anlatılan iş algı ayağındadır.

## 1. NE değişti

| # | Değişiklik | Dosya |
|---|---|---|
| 1 | Saf OpenCV hedef tespiti algı node'una **bağlandı** | `duba_gecis_navigator._p3_opencv_adaylari` |
| 2 | Dedektör paketin içine **kopyalandı** (kanonik: `girdap-ida-p3`) | `girdap_ida_algi/p3_hedef_bul.py` |
| 3 | YOLO **vetosu**: yayınlanmış duba kutularıyla örtüşen aday elenir | aynı metot |
| 4 | İki tanı sayacı: `p3_opencv` (üretilen) · `p3_veto` (elenen) | `_tani` |
| 5 | 12 test (mutasyonla doğrulandı) | `test/test_p3_opencv_hatti.py` |

Çıktı **yalnız** `/perception/targets`'a gider. `/perception/buoys`
sözleşmesine **dokunulmadı**.

## 2. NEDEN — P3'ün gözü yoktu

Hedef adayları bugüne kadar **YOLO kutularından** türüyordu
(`buyuk_cisim_mi` / `_mono_hedef_mi`). Ama model **2 sınıflı** ve **P3
hedefini görmemeyi öğrendi** (eğitim setinde etiketsiz P3 negatifleri var).
YOLO hedefi kutulamazsa o iki süzgeç hiç tetiklenmez ⇒ `/perception/targets`
boş ⇒ `hedef_sec` seçim yapamaz ⇒ **P3 = 0**, hiçbir hata basılmadan.
Ayrıca **kırmızı** hedef mono yolundan bilerek geçmiyor
(`MONO_HEDEF_RENKLERI = yeşil/siyah`, ölçüme dayalı) ⇒ hakem *"kırmızı"*
derse OpenCV yolu **tek üretici**.

## 3. Sözleşmeler (kod okunarak doğrulandı, varsayımla değil)

- `MissionState.PARKUR3 = "PARKUR3"` → `/girdap/mission/state` (String,
  `new_state.value`) → algı node `.upper()` → `p3_hedef_bul.P3_DURUMU`.
  Node bu topic'e **zaten aboneydi** (yeniden başlama için).
- Renk kodu **1=kırmızı · 2=yeşil · 3=siyah**, `gecit_mantik.HEDEF_RENK_KODU`
  ile birebir; testle donduruldu. (16.08'de bir kopyada bu tablo **TERSTİ**.)
- `planning_node._on_targets` → `Hedef(x, y, kod, cap, skor)`.
  **`cap = 0,0` basıyoruz**: menzil Ø0,64 *varsayarak* kurulduğu için ondan
  türetilen çap **dairesel** olurdu (ölçüm gibi görünen sahte kanıt).
  `hedef_secim.cap_makul_mu` 0'ı açıkça *"iddia yok"* sayıyor ⇒ kör eleme de
  olmuyor. `skor = 0,0`: saf OpenCV'nin model güveni yoktur.

## 4. ÖLÇÜMLER (17.08, PC)

- **Maliyet**, 512×512 (deploy passthrough boyutu): **7,61 ms/kare** ort,
  p95 8,55. Yalnız **2 Hz**'te ve **ayrı süreçte** koşar ⇒ karar tarafının
  kontrol döngüsünü **bloklayamaz**.
- **240 gerçek göl karesi, kadrajda hedef YOK** — kare başına yanlış aday:
  vetosuz **0,267** → YOLO vetolu **0,212**.
  Renk kırılımı: `kırmızı 3 → 0` · `siyah 8 → 2` · **`yeşil 53 → 49`**.

### 🔴 Bilinen risk: YEŞİL
Yeşil yanlış adayların kaynağı **kıyı bitkisinin su yüzeyindeki yansıması**
(gözle doğrulandı). **Veto kesmiyor**, konum ayırmıyor (medyan cy 0,84 =
suyun içi), boyut ayırmıyor (medyan alan 503 px ↔ gerçek hedef 8 m'de 513 px).
⇒ Hakem **yeşil** derse yanlış kilit riski en yüksektir.
Eşikler **bilerek değiştirilmedi**: kanonik süpürme yeşilde
`doluluk 0,55 → 0,62`'nin yanlış alarmı 3 kat kestiğini ama **bulmayı
%88 → %73** düşürdüğünü ölçmüş. Elimizde tek bir **gerçek hedef karesi**
yokken bu takas tek taraflı verilmez (16.08 dersi: İHA plakasının eşiklerini
kopyalayan ilk sürüm hedefin **%81'ini elemişti**). Karar Eyüp'ün.

## 5. Emniyet — P1/P2 etkisi SIFIR

1. Ana şalter `GIRDAP_P3_HEDEF` **varsayılan KAPALI**; kapalıyken
   `hedef_adimi` ilk satırda dönüyor ⇒ OpenCV **hiç** koşmaz.
2. İkinci kapı modülün içinde: `PARKUR3` dışında boş liste.
3. Veto listesi **yayınlanan** kutulardan; hedef adayları dışarıda — yoksa
   OpenCV kendi bulduğu gerçek hedefi elerdi (kırmızı hedef mono'da tam bu
   yola düşer). Kaynak sırası testle donduruldu.

**Yarışma günü açmak için** (kalkıştan önce, servis dosyasına):
```
Environment=GIRDAP_P3_HEDEF=1
```

## 6. GERİ ALINIRSA NE KIRILIR

P3'te hedef **yalnız YOLO onu kutularsa** görünür; model bunu görmemeye
eğitildiği için pratikte hedef **hiç görünmez** ve `/perception/targets` boş
akar. Belirtisi yoktur — ne hata, ne log; yalnız tekne hedefe gitmez
(**145 puan**, toplamın %48'i).

## 7. Kopya ↔ kanonik

`p3_hedef_bul.py` **kopyadır**; kanonik dosya `girdap-ida-p3/p3_hedef/hedef_bul.py`
ve sha256'sı kopyanın başlığında yazılı. Değişiklik **kanonikte** yapılır,
sonra kopya yenilenir. `test_kopya_kanonik_kaynakla_AYNI` ayrışmayı yakalar;
kanonik repo makinede yoksa **SKIP** eder (sessizce yeşil vermez).
Neden kopya: Jetson'da `girdap-ida-p3` kurulu bir paket değil ve sahada
internet/pip yok (md 4.1) ⇒ tek `git pull` ile gelen kod çalışmak zorunda.

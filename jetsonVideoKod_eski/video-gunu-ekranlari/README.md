# Video günü ekranlarına nasıl ulaşacağız (Ekran-1/2/3) + Ekran-2 önizleme

> Tek başlık, tek klasör: videonun 3 bölmesine çekim günü nasıl ulaşılır,
> Ekran-2 neye benzeyecek (SAHTE veriyle önizleme) ve montaj akışı.
> Ayrıntılı operatör listesi: `../karar/docs/video_gunu_runbook.md` §0-A.
> Web sürümü (aynı içerik): https://claude.ai/code/artifact/af17ad52-7431-407c-80ee-2f1a105eb2ea

## ⚠️ Bu klasördeki her şey SAHTE VERİ

Önizleme dosyaları gerçek `run_ekran2.py` aracıyla üretildi ama girdi
**sentetik** (60×40 m dikdörtgen, 4 GPS köşe, AUTO, WP_SPEED=2.0, köşe
dönüşleri, görev sonu temiz duruş, manuel dönüş). Amaç: çekimden ÖNCE
Ekran-2'nin neye benzeyeceğini görmek. Gerçek kayıtlarla karışmasın diye
`~/girdap_logs`'a hiç yazılmadı.

| Dosya | Ne |
|---|---|
| `ekran2_SAHTE_onizleme.png` | Ekran-2 paneli, statik (3 grafik) |
| `ekran2_SAHTE_onizleme.mp4` | Aynı panel, akan zaman imleçli (montajlık formatın aynısı) |
| `grafik_SAHTE_onizleme.csv` | Sahte 10 Hz görev verisi (GRAPH_CSV_HEADER formatı) |
| `sahte_grafik_uret.py` | CSV'yi üreten betik (yeniden üretmek için) |
| `ekran2_onizleme_SAHTE.pdf` | Hepsinin açıklamalı sayfa hâli (grafik okuma tablosu dahil) |

Yeniden üretmek: `python3 sahte_grafik_uret.py` →
`python3 ../karar/scripts/run_ekran2.py --csv grafik_SAHTE_onizleme.csv --thrust-birim %` (`--mp4` ile video).

## Üç ekran, çekim günü kim/nasıl

| Bölme | İçerik | Nasıl elde edilir |
|---|---|---|
| **Ekran-1** | YKİ (QGC) ekranı — harita, görev, telemetri | Laptop'ta **ekran kaydı** — operatör çekimden önce BAŞLATIR |
| **Ekran-2** | Senkron grafikler: hız+setpoint, heading+yön setpoint, thrust sol/sağ | **Canlı değil** — CSV teknede otomatik birikir, panel çekimden SONRA üretilir (aşağıda) |
| **Ekran-3** | Dış kamera — tekne suda | Kıyıdan telefon/kamera — operatör BAŞLATIR |

## Ekran-2'ye ulaşma akışı (çekim günü)

1. **Çekim öncesi:** Jetson açılınca `girdap-karar.service` kendiliğinden
   kalkar, kayıt **otomatik başlar** (16.07 düzeni: her koşu kendi numaralı
   klasöründe — `~/girdap_logs/kayit/<N>/grafik.csv` 10 Hz + `telemetri.csv`
   2 Hz; FC ARM olunca YENİ kayıt klasörü açılır, eski kayıtlar 20 adedi
   aşınca en eskiden silinir) — teknede hiçbir şey başlatılmaz. QGC ekran
   kaydı + dış kamera operatör işi.
2. **Çekim sırasında:** Ekran-2 izlenmez/izlenemez; veri sessizce birikir.
   CSV her satırda diske işlenir (fsync) — güç kesilse bile kayıp olmaz.
3. **Tekne karaya alınınca** Jetson'da hızlı PNG kontrolü (kontrol-listesi §0b):
   ```bash
   cd ~/ros2_ws/src/girdap-decision
   python3 scripts/run_ekran2.py --thrust-birim %          # hızlı PNG kontrolü
   ```
   **Montajlık MP4 ise PC'DE üretilir (KARAR 16.07):** Jetson render'ı ~3 kare/sn
   (3-5 dk görev @30 fps = 30-50 dk), PC ~5-15 dk. CSV'yi USB/scp ile PC'ye al,
   adım adım: **`../pc-render-yonergesi.md`** (tek seferlik kurulum + iki geçişli
   yöntem: PNG'den pencereyi oku → `--t0/--t1` ile MP4). Jetson yolu yedek olarak
   çalışır durumda (aynı komut `--mp4` ile, sadece yavaş).
   Çıktı: `~/girdap_logs/viz/ekran2_*.png|.mp4`.
   - `--thrust-birim %` UNUTMA: video AUTO+FC modunda — thrust MPPI newtonu
     değil, FC servo çıkışı yüzdesi. Unutulursa eksen yalan söyler.
   - **Acelesi yok:** CSV diskte durduğu sürece bu adım eve dönünce de olur;
     CSV PC'ye kopyalanıp orada da üretilebilir (araç ROS gerektirmez,
     numpy+matplotlib yeter).
   - **Jetson'u kapatıp açmak sorun değil** — eski kayıtlar silinmez.
     **⚠ Tek tuzak:** yeni boot'ta servis YENİ (boş) CSV açar ve komut
     varsayılan olarak EN YENİSİNİ seçer → reboot'tan sonra üretiyorsan görev
     CSV'sini `--csv grafik_<görev-saati>.csv` ile ELLE seç (adında UTC saat var).
4. **Montaj:** MP4'ü USB/scp ile montaj bilgisayarına al; 3 bölmeyi yerleştir,
   **görev başlangıcını hizala** — MP4'teki akan zaman imleci senkron kanıtı
   (md 3.3.1.1 "senkron" şartı), gerisi kendiliğinden hizalı kalır.
   ≥720p, 2–5 dk, kesintisiz tek gösterim → YouTube liste dışı → link KYS'ye.

## Sahte grafikte ne görülüyor (kısa okuma)

- 0–8 s: disarm — setpoint sütunları BOŞ, thrust hücresi BOŞ (PWM=0 → boş hücre kuralı).
- 8–15 s: ARM rölanti — thrust %0 çizgisi başlar.
- 15 s: AUTO → görev başlar — hız setpoint 2.0 belirir, yön setpoint ilk noktaya kilitlenir.
- ~21/~50/~73 s: köşeler — yön setpoint basamak, heading yumuşak takip, hızda
  çukur, thrust sol/sağ zıt makaslama (diferansiyel dönüş imzası).
- 73–104 s: batı bacağı — heading ±180° sarımı, çizgideki kopukluk bilinçli NaN (hata değil).
- ~104 s: 4. nokta — görev otonom TAMAMLANIR: setpoint kesilir, thrust 0, temiz duruş.
- 108+ s: manuel dönüş — thrust operatörden, setpoint YOK (manuel dönüşte
  setpoint görünüyorsa bir şey ters — runbook kontrolü).

## Yarışma günü notu

Ekran-2 yalnız VİDEO şartı (md 3.3.1.1). Yarışmada zorunlu olan Dosya-1/2/3
teslimi (20 dk, USB — md 4.2); canlı izleme zaten yasak (md 4.1 görüntü/veri
aktarımı + md 5.5.3.1 YKİ'de yalnız YKİ arayüzü, çadırdakiler İDA'yı göremez).
Sahada canlı tek ekran QGC. İstenirse her koşudan SONRA aynı komutla analiz
paneli üretilir — CSV her koşuda zaten yazılıyor.

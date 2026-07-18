# Ekran-2 MP4'ünü PC'de üretme yönergesi (KARAR: render PC'de — 2026-07-16)

> **Neden PC:** Jetson ölçümü (16.07): ~3 kare/sn render → 3-5 dk'lık görev
> @30 fps = **30-50 dk bekleme**. PC'de aynı iş ~5-15 dk; montaj zaten PC'de.
> Jetson yolu ÇALIŞIR durumda ve YEDEK olarak kalır (aynı komut, sadece yavaş).
> Çıktı iki yolda da birebir aynıdır (aynı kod, aynı CSV).

## 0. Tek seferlik PC hazırlığı (video gününden ÖNCE dene!)

```bash
# 1) Repo (yoksa klonla, varsa güncelle)
git clone https://github.com/EyupEker1/girdap-video ~/girdap-video   # ilk kez
cd ~/girdap-video && git pull                                        # sonrası

# 2) Python bağımlılıkları (araç: saf numpy+matplotlib, ROS GEREKMEZ)
pip install matplotlib numpy

# 3) ffmpeg (MP4 yazımı için ŞART — araç PATH'te ffmpeg arar)
ffmpeg -version          # varsa tamam
sudo apt install ffmpeg  # yoksa ve sudo varsa
# sudo YOKSA (bu PC'de gh de ~/.local/bin'deydi) — statik ffmpeg, sudo'suz:
pip install imageio-ffmpeg
mkdir -p ~/.local/bin
ln -sf "$(python3 -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')" ~/.local/bin/ffmpeg
# ~/.local/bin PATH'te olmalı (genelde öyledir): ffmpeg -version ile doğrula
```

**Hazırlık testi:** bu repodaki sahte CSV ile bir MP4 çıkar — çıkıyorsa PC hazır:
```bash
cd ~/girdap-video
python3 karar/scripts/run_ekran2.py \
  --csv video-gunu-ekranlari/grafik_SAHTE_onizleme.csv --thrust-birim % --mp4
# çıktı: ~/girdap_logs/viz/ekran2_grafik_SAHTE_onizleme.mp4 (klasörü kendisi açar)
```

## 1. Koşu günü: CSV'yi Jetson'dan al

CSV küçüktür (~2-5 MB). İki yol:

- **Kayıt düzeni (16.07): her koşu kendi numaralı klasöründe** —
  `~/girdap_logs/kayit/<N>/` içinde `grafik.csv` + `telemetri.csv` (Dosya-2).
  Boot bir kayıt açar, **FC ARM olduğu anda yenisi açılır** (journal:
  "ARM algılandı → yeni kayıt: …/kayit/N") → **görev = ARM anında açılan
  klasör**. Numara silinen numarayı yeniden kullanır → "en büyük numara ≠
  en yeni" olabilir; şüphede setpoint sütunları dolu olana bak.
- **Sahada (ağ yok): USB bellek** — görev klasörünü komple kopyala
  (`kayit/<N>/`, iki CSV bir arada).
- **Aynı ağda: scp.** IP her ağda değişir — Jetson'da `hostname -I` ile bak:
  ```bash
  scp -r girdap@<JETSON_IP>:girdap_logs/kayit/<N> .
  ```

## 2. PC'de render — İKİ GEÇİŞLİ yöntem (önerilen)

MP4 yavaş, PNG saniyelik → pencereyi PNG ile bul, MP4'ü tek seferde bas:

```bash
cd ~/girdap-video

# GEÇİŞ 1 — hızlı PNG, pencereyi gözle bul:
python3 karar/scripts/run_ekran2.py --csv <dosya>.csv --thrust-birim %
# PNG'yi aç; x-ekseni "görev süresi (s)". Setpoint eğrilerinin dolu olduğu
# aralık = görev penceresi. Başı/sonu oku (ör. 1210 ile 1465).

# GEÇİŞ 2 — MP4'ü o pencereyle bas:
python3 karar/scripts/run_ekran2.py --csv <dosya>.csv --thrust-birim % \
  --mp4 --fps 30 --t0 1210 --t1 1465
# çıktı: ~/girdap_logs/viz/ekran2_<dosya>.mp4  (--out ile başka yere yazılır)
```

Unutma:
- **`--thrust-birim %` her zaman** (AUTO/fc modunda eksen % olmalı; yoksa "N" yazar = yanlış).
- `--t0/--t1` birimi: CSV'nin İLK satırından itibaren saniye (PNG ekseninin aynısı).
- Saat sıçraması tuzağı: Jetson saati koşudan önce senkronlanmadıysa eksende
  saatlik delik görünebilir — pencereyi yine PNG'den gözle seç, sorun çıkmaz.

## 3. Montaj (PC'de, değişmedi)

Şekil 1 düzeni: sol-üst QGC ekran kaydı · sol-alt bu MP4 · sağ dış kamera.
Senkron referansı: ARM/görev başlama anı; kaba test = köşe dönüşü üç bölmede
aynı anda. Ayrıntı: `video-gunu-ekranlari/README.md` + runbook §0-A.

## Sorun giderme

| Belirti | Sebep/çözüm |
|---|---|
| `ffmpeg bulunamadı` | §0 adım 3 — PATH'te ffmpeg yok |
| `ModuleNotFoundError: prototype` | repo kökünden değil de kopyalanmış tek dosyayla çalıştırıldı — repo içinden `karar/scripts/run_ekran2.py` yolunu kullan |
| PNG'de her şey boş | yanlış CSV (boş boot dosyası) — görev dosyasını seç |
| thrust paneli boş, gerisi dolu | koşuda kanal 1/3'e PWM düşmemiş — Jetson'da `fc_thrust_left_ch/right_ch` FC'nin sürdüğü kanalla eşleşmeli (FC ekibiyle) |

# REBOOT SONRASI YAPILACAKLAR — 17.08.2026

> Bugün değişen **her şey boot'ta yükleniyor** ve son reboot 04:20'deydi,
> yani hepsinden önce. Bu liste o sınavın kontrol listesi.
>
> ⚠️ Claude bu Jetson'da koşuyor — reboot oturumu da kapatır. Claude'u
> yeniden açtığında **bu dosyayı ona okut**, kaldığı yerden devam eder.

---

## 0 · REBOOT'TAN ÖNCE (bitti ✅)

- [x] Benim beş commit'im push'landı → `2a4a8c9..f8e8910`
- [x] Reboot öncesi kanıt: `~/girdap_logs/reboot_oncesi_1256.txt`
- [ ] **Diğer oturum kendi işini push etsin** (plant aracı · pusula aracı ·
      `gol_hazir_mi.sh`) — kaptan "ayrı ayrı" dedi
- [ ] Reboot'u **kaptan** atacak

---

## 1 · İLK KOMUT — tek satır, 7 bölüm (~2 dk)

```bash
bash ~/reboot_sonrasi_teyit.sh 2>&1 | tee ~/reboot_sonrasi.txt
```

Bugünün dört değişikliğini boot'ta sınar:

| # | ne sınıyor | neden |
|---|---|---|
| 1 | servisler kendi başına kalktı mı | autostart |
| **2** | **`girdap-ff-ayar` boot'ta ölüyor mu** | 🔑 **asıl sınav** — kusur tam olarak *boot'tan 29 sn sonra ölmek*ti; `restart` ile doğrulandı ama **restart ≠ boot** (MAVROS'un parametre indirme zamanlaması boot'ta farklı, yarış durumu oradan çıkmıştı) |
| 3 | Livox spin yaması yürürlükte mi | bir çekirdek geri kaybedilmesin |
| 4 | blob + repo taze mi | §1.41a'nın tekrarı olmasın |
| 5 | algı ↔ karar aynı keşif dünyasında mı · `/dev/pixhawk` | §1.25 · udev |
| 6 | FC parametreleri sağ çıktı mı | `FF=0,52` · `ACCEL_MAX=0,30` |
| 7 | POZ-BAYAT | bugünün asıl kazanımı (34/60 sn → 0) |

**Beklenen:** `TEYİT GEÇTİ`. Kırmızı varsa çıktı hangi maddede olduğunu yazar.

---

## 2 · SONRA — mevcut iki betik (~5 dk)

```bash
bash ~/IDA_GIT/son_kodv2/karar/scripts/reboot_teyit.sh | tee ~/reboot_teyit.txt
bash ~/IDA_GIT/son_kodv2/karar/scripts/gol_hazir_mi.sh          # diğer oturumun
```

Üçü birbirini tamamlıyor: benimki *"bugünün değişiklikleri sağ çıktı mı"*,
`reboot_teyit.sh` *"autostart kanıtı"*, `gol_hazir_mi.sh` *"suya inmeye
hazır mıyım"*.

---

## 3 · BOZUKSA — nereye bakılacak

| belirti | sebep | çözüm |
|---|---|---|
| `ff-ayar` inactive + *"beklenmedik tip (0)"* | düzeltme tutmadı | `journalctl -b -u girdap-ff-ayar`; kod `scripts/otomatik_ff_ayar.py` |
| livox thread'i %90+ | yama boot'ta yok | `deploy/livox/OKU.md` §3 — `git am` + `colcon build` + restart |
| `/dev/pixhawk` yok | udev yüklenmedi | `sudo udevadm control --reload-rules && sudo udevadm trigger` |
| `/perception/buoys` abonesi 0 | §1.25 izolasyon | `ROS_LOCALHOST_ONLY` dört serviste de olmalı |
| blob hash tutmuyor | dağıtım bayat | `cp ~/IDA_GIT/son_kodv2/algi/models/yolo11n_duba_rvc2.blob ~/models/` |
| `girdap-livox` `activating` | LiDAR bağlı değil | **NORMAL**, hata değil |

---

## 4 · BOOT'TAN SONRA KOŞULACAK KURU TEST ADIMLARI

Reboot testi (ADIM 3) bununla kapanmış olur. **Elle iş gerektiren, hâlâ
açık** olanlar:

- **ADIM 4** — kamerayı çıkar → tak (F-A.4'ün ikinci yarısı, 3 dk).
  Beklenen: *"OAK kamera USB'de göründü (N s bekledi)"*
- **ADIM 6** — motor testi, `MOT_THR_MIN` (⚠️ pervaneler sökük)
- **ADIM 7** — ileri/geri itki asimetrisi, `MOT_THST_ASYM`
- **ADIM 8** — pusula-motor girişimi
- **ADIM 9** — motor yönü / kanal
- **ADIM 14** — kumanda anahtarı (🔑 **yeni bilgi:** `MODE6 = 15 = GUIDED`,
  `MODE4 = 4 = HOLD` — hafızadaki *"altı konumun altısı da MANUAL"* tespiti
  ÇÜRÜDÜ; kumandayı açıp doğrula)

---

## 5 · 🔴 SUYA GİRMEDEN ÖNCE — UNUTULMAYACAK

1. **Üç ayar servisi AÇIK kalsın** (`girdap-ff-ayar`, `girdap-plant-ayar`,
   `girdap-pusula-ayar`) — ölçümleri onlar alacak, sahada elle
   başlatılamıyorlar (md 4.1: SSH yok).
2. **Yarışma günü** (koşumdan önce) kapatılacak:
   ```bash
   sudo systemctl disable --now girdap-ff-ayar girdap-plant-ayar girdap-pusula-ayar
   systemctl is-enabled girdap-ff-ayar girdap-plant-ayar girdap-pusula-ayar  # KANIT
   ```
3. **`COMPASS_LEARN` kararı VERİLMEDİ** — dosyada 3 (Sude, `2a4a8c9`),
   canlı FC'de 0. Sude'nin sabit öğrenmesi ile diğer oturumun sabit-yön
   kalibrasyonu **aynı ofsetleri** yazıyor; ikisi birden açılmamalı.
   Kaptan Sude ile konuşmalı. **O karar gelmeden tekneye indirilmeyecek.**

---

## 6 · GÜNÜN ASIL ÖLÇÜTÜ (kod değil, su)

> **5 dakika kesintisiz, müdahalesiz GUIDED+ARMED.**

Şimdiye kadarki en uzun pencere **345 sn**. Başarı ölçütü tek sayı:
`inhibit_reason`'daki **PIVOT oranı** — şu an **%71**, düşerse zincir
doğrulanmış olur.

Bugünkü iş bunu *çözmedi*, **ölçülebilir hale getirdi**: tekne daha önce
zamanın %22'sinde POZ-BAYAT yüzünden itkisiz kalıyordu (şimdi 0), o katman
kalkmadan hiçbir A/B temiz okunamazdı.

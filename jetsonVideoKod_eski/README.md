# 📦 jetsonVideoKod — ARŞİV

> Bu klasör, **GİRDAP** ekibinin TEKNOFEST 2026 İnsansız Deniz Aracı yarışması
> **otonomi kabiliyet videosunda** (21.07.2026 eleme kapısı) Jetson üzerinde koşan
> **karar/otonomi yazılımının dondurulmuş bir kopyasıdır**. Gömülü ekip (IDA-TEAMM)
> deposunda **referans/arşiv** olarak durur — adındaki "eski", bu kodun canlı
> geliştirme dalı değil, o ana ait **dondurulmuş sürüm** olduğunu belirtir.

| | |
|---|---|
| **Kaynak repo** | `github.com/EyupEker1/girdap-video` @ `14a0be5` |
| **İçerik** | Kaynak reponun izlenen **149 dosyasının tamamı** (`git archive` ile birebir kopya; `__pycache__`/derleme çıktısı yok) |
| **Arşivlenme** | 2026-07-18, EyupEker1 (GİRDAP) |
| **Durum** | Dondurulmuş — güncel geliştirme başka repolarda sürebilir; buradaki kod **değişmez arşivdir** |

### Klasör haritası
- **`karar/`** — asıl karar (decision) yazılımı — ROS2 otonomi yığını:
  - `prototype/` — saf Python algoritma modülleri: **perception** (kamera+LiDAR engel/duba algısı), **fusion** (sensör füzyonu/durum kestirimi), **planning** (MPPI yol planlayıcı), **mission** (waypoint/görev akışı), **fsm** (ARM→görev→dönüş durum makinesi), **control** (hız/thrust setpoint), **mapping/dynamics/telemetry/viz/configs**
  - `ros2_ws/src/girdap_decision` — bunu saran ROS2 paketi (node'lar, launch)
  - `scripts/` — `run_ekran2.py` (Ekran-2 grafik üretici), `fc_param_turu.sh`, `girdap-karar.service`, udev kuralları
  - `docs/` — video günü runbook'u, `hata_defteri.md`, FC parametre önerileri
- **`testler/`** — bileşen bileşen kanıt (`video_testleri.sh`)
- **Video günü materyalleri** — `kontrol-listesi.md`, `test-plani.md`, `pc-render-yonergesi.md`, `qgc-video-ayarlari.*`, `video-gunu-ekranlari/`

### Videodaki senaryo (özet)
Tekne **ARM** → mod **AUTO/GUIDED** → görev başlar → 4 noktayı **otonom** takip eder
(algı engelden kaçar, MPPI planlar, setpoint'ler Pixhawk uçuş kontrolcüsüne MAVROS
üzerinden gider) → sıfır thrust → manuel dönüş → güç kesme. Ekran-2 üç sinyali
gösterir: hız↔hız-setpoint, heading↔yaw-setpoint, thrust isteği.

> ℹ️ Aşağıdaki bölüm, kaynak `girdap-video` reposunun **orijinal README'sidir** (olduğu
> gibi korunmuştur). Metindeki "bu repo" ifadeleri o kaynak repoyu kasteder.

---

# GİRDAP — Otonomi Kabiliyeti Videosu Reposu

**Son teslim: 21.07.2026 saat 17:00 (KYS'ye YouTube linki) — ELEME KAPISI.**
Video gönderilmezse/geçmezse yarışmaya katılım yok (şartname md 3, 3.3); P1+P2+P3 = 0.

## Bu repo ne?

1. **`karar/` = videoda koşacak kodun DONDURULMUŞ kopyası** (girdap-decision @ `15dc238`
   + 2026-07-14 yamaları: **F-M.3** (servis-KILL → FCU disarm) + **B1/B2 AUTO dönüşü +
   F-M.6** (10 Hz akış, canlı 9.99 Hz teyitli) + **AUTO denetimi F-V.6/7/8 + F-P.1** +
   **F-M.7** (restart/boot'ta FC hiç bağlanmadan heartbeat-KILL latch'i — bekçi artık
   ilk `connected=true`'dan sonra kurulur). Jetson runtime klonunda son lokal commit
   `c2d7a10`, suite yeni taban **302/2**. Ayrıntı: `karar/docs/hata_defteri.md`.
   Geliştirme başka repolarda devam etse bile videonun kodu burada sabit kalır.
   Güncelleme yalnız BİLİNÇLİ kararla: `git subtree pull --prefix=karar
   /home/eyup/girdap-decision main` + bu README'deki commit'i güncelle.
2. **Video günü planı ve kontrol listeleri** (aşağıda + `kontrol-listesi.md`).
3. **`testler/` = bileşen bileşen kod kanıtı** — çekimden önce her bileşenin
   ayrı ayrı çalıştığını tek komutla gösterir: `bash testler/video_testleri.sh`
   (Jetson'da `GIRDAP_KOD=~/ros2_ws/src/girdap-decision` ile ayrı klona karşı;
   ayrıntı `testler/README.md`). PC'de 12/12 bileşen ✅ teyitli (2026-07-13).

## Video günü — tek bakışta akış

Ayrıntılı prosedür: **`karar/docs/video_gunu_runbook.md`** (bölüm 0-8, şartname eşlemeli).

```
GÜN ÖNCESİ  atölye hazırlığı (runbook §2) + FC güvenlik aksiyonları (aşağıda 🔴)
SAHADA      kurulum (§3) → QGC'den 4 nokta yükle → prova
ÇEKİM       ARM → mod GUIDED = görev başlar → 4 nokta → sıfır thrust
            → manuel dönüş → güvenlik anahtarıyla güç kesme + RC'de motor dönmüyor
            → kapaklar açık/su yok — HEPSİ TEK KESİNTİSİZ ÇEKİMDE (md 3.3.1/6)
MONTAJ      3 bölme: Ekran-1 QGC kaydı · Ekran-2 grafikler (run_ekran2.py)
            · Ekran-3 dış kamera (§6)
YAYIN       ≥720p, 2-5 dk, YouTube LİSTE DIŞI → linki KYS'ye (§7; link sorunlu = eleme)
```

- Başlatma tetiği: önce ARM, sonra QGC'den mod → **GUIDED** (kenar tetikli `start_on_mode`).
- Ekran-2 zorunlu üç sinyal: hız+hız_setpoint · heading+yaw_setpoint · **thrust isteği**.
  Kayıtlar `~/girdap_logs/grafik/`; montaj: `python karar/scripts/run_ekran2.py --mp4`.
- ⚠️ md 3.3.1.1: **istemsiz dönüş/sürüş görünürse video BAŞARISIZ** — prova şart.

## 🔴 İLK GÜÇ VERİŞTE (PERVANESİZ!) — pazarlıksız

FC'nin hafızasında M4 testinden kalma **4400 km'lik sahte görev duruyor** — RC/AUTO ile
kendi kendine tam güç koştuğu OLAY yaşandı. Sırasıyla:
1. `ros2 service call /mavros/mission/clear mavros_msgs/srv/WaypointClear`
2. `BRD_SAFETY_DEFLT=1` geri yaz (QGC parametreler)
3. RC mod kanalı / FLTMODE düzeni FC ekibiyle (öneriler: `karar/docs/fc_parametre_onerileri.md`)
4. Batarya takılı bırakılmaz.

## Jetson kurulumu (sıfırlama sonrası)

Videoda Jetson **test edilmiş ayrı-klon düzeniyle** koşar (bu repo Jetson'a kurulmaz):
Masaüstündeki `JETSON_REHBERI.md` (= girdap-ida-algi `docs/BURADAN_BASLA.md`) uçtan uca
rehber. Kontrol noktaları: numpy 1.26.4+opencv 4.11.0.86 TEK komutta · cupy-cuda12x ·
suite 259/2 · `jetson_kontrol.sh` PASS.

## İlgili diğer dokümanlar (hepsi `karar/docs/` içinde)

`masa_testi_runbook.md` (M0-M8) · `fc_parametre_onerileri.md` · `donanim_gercekleri_gomulu_ekip.md`
· dış ihtiyaç listesi: girdap-ida-algi `docs/kod_disi_ihtiyaclar.md` (RFD çifti, QGC laptop,
OBS, RC bandı 2.4 GHz OLMAMALI, güvenlik anahtarı, dış kamera).

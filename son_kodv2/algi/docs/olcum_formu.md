# Ölçüm İstek Formu — Mekanik + FC Ekibine

> Bu formu olduğu gibi mekanik/FC ekibine gönder; boşlukları doldurup geri
> yollasınlar. Her satırda NEDEN gerektiği ve NASIL ölçüleceği yazıyor.
> Teknik arka plan: [`bekleyen_girdiler.md`](bekleyen_girdiler.md) §A.
>
> ⚠️ Su hattı ölçüleri tekne **yarışma yüküyle** (bataryalar takılı) suda
> yüzerken alınır; suya giremiyorsanız CAD + fribord hesabından tahmin yazın
> ve "tahmin" diye işaretleyin.

## 1. 🔴 Livox Mid-360 yüksekliği `h` — EN ACİL

Su yüzeyinden LiDAR'ın optik merkezine (kubbenin ortası) DÜŞEY mesafe.
Bu sayı bilinmeden **Parkur-2 suda denenmeyecek** (yanlışsa LiDAR dubaları
tamamen siler, tekne dubaların içinden sürer).

- Nasıl: tekne suda yüklüyken şerit metreyle su yüzeyi → kubbe ortası. ±5 cm yeter.
- `h` (m): **______**
- LiDAR'ın tekne merkezinden ileri/geri ofseti x (m, ileri +): **______**
- Sağ/sol ofseti y (m, sol +): **______**

## 2. 🔴 `base_link` kararı (ölçüm değil, TEK CÜMLELİK karar)

Yazılımın "teknenin sıfır noktası" tanımı. Önerimiz: **su hattı
yüksekliğinde, teknenin geometrik merkezi**. Farklı bir tercih varsa yazın:

- base_link = **______________________** (öneri kabulse "öneri kabul" yazın)

## 3. 🟠 OAK-D Lite kamera montajı

- Su hattından yüksekliği (m): **______**
- Tekne merkezinden ileri ofset (m): **______**
- Pitch açısı (derece; aşağı bakıyorsa −, telefon su terazisi uygulaması yeter): **______**
- Yaw (tam ileri bakmıyorsa, derece): **______**

## 4. 🟠 Tekne + thruster geometrisi

- Toplam genişlik / beam (m, gövdeden gövdeye dış-dış): **______**
- Sol-sağ thruster hatları arası mesafe (m): **______**
- Thrusterların tekne merkezinden geri ofseti (m): **______**

## 5. 🟡 Pixhawk/IMU konumu (acele değil — Parkur-3 işi)

- base_link'e göre x/y/z (m): **______ / ______ / ______**

## 6. 🔴 FC ekibinden — ArduPilot failsafe parametreleri

Telemetri koparsa motorların FCU tarafında da durduğunu garanti etmemiz
gerekiyor (yazılım KILL'i hat koptuğunda FCU'ya ulaşamaz — bilinen sınır).

- QGC → Vehicle Setup → Parameters'tan şu değerleri yazın/ekran görüntüsü alın:
  `FS_ACTION`, `FS_TIMEOUT`, `FS_GCS_ENABLE`, `FS_THR_ENABLE`, `FS_CRASH_CHECK`:
  **______________________**
- RC kumandanın çalıştığı frekans bandı (etiketten; 2.4 GHz YASAK — md 4.1):
  **______**

## 7. ℹ️ CAD/URDF referans değerleri — DOĞRULAYIN (ölçümün yerini tutmaz)

Gömülü ekibin SolidWorks→URDF modelinden (IDA_GIT `girdap_yenimodel`, 2026-07-13
okundu) çıkan değerler. Eksen yorumumuz: x=yanal, y=boyuna (kıç +), z=yukarı,
origin ≈ pruva/güverte. **Önce şu soruya cevap verin: bu eksen yorumu doğru mu?**
Doğruysa aşağıdakiler §1/§3/§4 için ön-dolgu olarak kullanılabilir:

- Thruster eksenleri arası yanal mesafe: **0.594 m** (CAD'de kesin; §4 sorusu)
- LiDAR: tekne boyunca ortada (~0.52 m), **güverteden ~0.16 m yukarıda** —
  ⚠️ §1'in istediği SU HATTINDAN yükseklik; fribord (su→güverte) EKLENMEDEN
  kullanılamaz. Fribord (yüklü, m): **______**
- Kamera: pruvaya yakın (boyuna ~0.26 m), güverteden ~0.08 m
- GPS direği: kıça yakın (~0.90 m), güverteden ~0.17 m
- Thruster pervaneleri: güverteden ~0.21 m aşağıda, kıçta (~0.99 m)
- Eksen yorumu doğru mu / base_link (URDF origin) tam neresi: **______**

---

Form dolunca Eyüp'e geri gönderin; değerler `hardware.launch.py` static
TF'lerine, LiDAR filtresine (F5.1) ve MPPI emniyet payına işlenecek.

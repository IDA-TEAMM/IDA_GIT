# Pixhawk Parametre Karışıklığı — Bulgu ve Düzeltme Raporu

*Hazırlanma tarihi: 2026-07-19 16:22 — İDA/Girdap USV, Takım 989124*

## Özet

Yeni bir test öncesi Pixhawk parametreleri 18 Temmuz'daki (temiz/doğru) durumla karşılaştırıldı. **118 parametre farklı bulundu** — tek tük kayıp değil, sistematik bir "parametreler fabrika ayarlarına dönmüş" paterni: pusula/ivmeölçer kalibrasyonu, RC kalibrasyonu, batarya monitörü ve failsafe ayarları aynı anda sıfırlanmış. `STAT_BOOTCNT` sayacının 29'dan 21'e (normalde sadece artar) düşmüş olması da bunun bir "parametreleri sıfırlama" olayından geçtiğini doğruluyor. Ekip içinde birinin (muhtemelen Mission Planner'da "Reset to Default" gibi bir işlemle) kazara tetiklediği düşünülüyor.

**Aynı zamanda bu, önceki oturumda bulduğumuz "AUTO modda tekne kendi etrafında dönüyor / rastgele gidiyor" sorununun da kök nedeniydi** — pusula hiç kalibre edilmemişti (`COMPASS_OFS`=0,0,0 / `COMPASS_DIA`=1,1,1, tam fabrika değerleri).

## Durum

| Grup | Açıklama | Durum |
|---|---|---|
| **A** — Kalibrasyon (pusula, ivmeölçer, trim) | 26 parametre | ✅ **Düzeltildi ve reboot sonrası doğrulandı** |
| **B** — Güvenlik/failsafe (batarya, failsafe, açılış modu) | 6 ana + 19 batarya alt-parametresi | ✅ **Düzeltildi ve reboot sonrası doğrulandı** |
| **C** — RC kalibrasyonu + kanal haritası (`RCMAP_THROTTLE/PITCH`, `MODE_CH`) | ~35 parametre | ⏳ **BEKLİYOR — fiziksel kumanda testi + takım onayı gerekiyor** |
| **D** — Servo fonksiyonları | 4 parametre | ℹ️ Kısmen zaten doğru (dokunulmadı), `SERVO1_REVERSED` alan testi gerektiriyor |
| **E** — Seri port/telemetri stream-rate | ~17 parametre | 🟡 Opsiyonel, kritik değil |
| **F** — Bilgi amaçlı/ortama bağlı sayaçlar | ~13 parametre | ➖ Düzeltmeye gerek yok |

## Jiroskop notu

`INS_GYROFFS_*`, `INS_GYR2OFFS_*` ve `INS_GYR*_CALTEMP` de listede görünüyor ama bunlar bir sorun DEĞİL — ArduPilot jiroskopu her açılışta (tekne sabit dururken) otomatik yeniden kalibre eder, pusula/ivmeölçer gibi elle kalibrasyon gerektirmez.

## Tam parametre listesi (118 fark + 19 batarya alt-parametresi)

| Grup | Parametre | Eski (doğru) | Karışıklık sonrası | Şimdi | Açıklama |
|---|---|---|---|---|---|
| A · Kalibrasyon | `AHRS_TRIM_X` | -0.00494463 | 0 | **-0.00494463** (düzeltildi) | Seviye/trim kalibrasyonu (tekne düz dururken yapılmıştı). |
| A · Kalibrasyon | `AHRS_TRIM_Y` | -0.0147192 | 0 | **-0.0147192** (düzeltildi) | Seviye/trim kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_DIA_X` | 1.02148 | 1 | **1.02148** (düzeltildi) | Pusula soft-iron ölçek kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_DIA_Y` | 1.00394 | 1 | **1.00394** (düzeltildi) | Pusula soft-iron ölçek kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_DIA_Z` | 1.05863 | 1 | **1.05863** (düzeltildi) | Pusula soft-iron ölçek kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_ODI_X` | 0.00675183 | 0 | **0.00675183** (düzeltildi) | Pusula soft-iron çapraz terim. |
| A · Kalibrasyon | `COMPASS_ODI_Y` | 0.099924 | 0 | **0.099924** (düzeltildi) | Pusula soft-iron çapraz terim. |
| A · Kalibrasyon | `COMPASS_ODI_Z` | 0.0754121 | 0 | **0.0754121** (düzeltildi) | Pusula soft-iron çapraz terim. |
| A · Kalibrasyon | `COMPASS_OFS_X` | -51.8937 | 0 | **-51.8937** (düzeltildi) | Pusula hard-iron offset kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_OFS_Y` | 30.4748 | 0 | **30.4748** (düzeltildi) | Pusula hard-iron offset kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_OFS_Z` | -2.77444 | 0 | **-2.77444** (düzeltildi) | Pusula hard-iron offset kalibrasyonu. |
| A · Kalibrasyon | `COMPASS_ORIENT` | 6 | 0 | **6** (düzeltildi) | Pusulanın fiziksel montaj yönü — olmadan heading direkt yanlış okunur. |
| A · Kalibrasyon | `INS_ACC1_CALTEMP` | 44.8068 | -300 | **44.8068** (düzeltildi) | İvmeölçer sıcaklık-kompanzasyon referansı (IMU1). |
| A · Kalibrasyon | `INS_ACC2OFFS_X` | 0.0455435 | 0 | **0.0455435** (düzeltildi) | İvmeölçer (IMU2) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2OFFS_Y` | 0.00206468 | 0 | **0.00206468** (düzeltildi) | İvmeölçer (IMU2) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2OFFS_Z` | 0.0755522 | 0 | **0.0755522** (düzeltildi) | İvmeölçer (IMU2) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2SCAL_X` | 0.993165 | 1 | **0.993165** (düzeltildi) | İvmeölçer (IMU2) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2SCAL_Y` | 0.994851 | 1 | **0.994851** (düzeltildi) | İvmeölçer (IMU2) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2SCAL_Z` | 0.993066 | 1 | **0.993066** (düzeltildi) | İvmeölçer (IMU2) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_ACC2_CALTEMP` | 41.25 | -300 | **41.25** (düzeltildi) | İvmeölçer sıcaklık-kompanzasyon referansı (IMU2). |
| A · Kalibrasyon | `INS_ACCOFFS_X` | 0.0225652 | 0 | **0.0225652** (düzeltildi) | İvmeölçer (IMU1) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACCOFFS_Y` | 0.0175615 | 0 | **0.0175615** (düzeltildi) | İvmeölçer (IMU1) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACCOFFS_Z` | 0.0489043 | 0 | **0.0489043** (düzeltildi) | İvmeölçer (IMU1) offset kalibrasyonu. |
| A · Kalibrasyon | `INS_ACCSCAL_X` | 0.999449 | 1 | **0.999449** (düzeltildi) | İvmeölçer (IMU1) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_ACCSCAL_Y` | 0.998178 | 1 | **0.998178** (düzeltildi) | İvmeölçer (IMU1) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_ACCSCAL_Z` | 0.999613 | 1 | **0.999613** (düzeltildi) | İvmeölçer (IMU1) ölçek kalibrasyonu. |
| A · Kalibrasyon | `INS_GYR1_CALTEMP` | 42.3913 | 27.4155 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYR2OFFS_X` | -0.00131365 | -0.000106914 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYR2OFFS_Y` | 0.00496642 | -0.00396401 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYR2OFFS_Z` | -0.000366553 | 0.00370668 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYR2_CALTEMP` | 18.8125 | 12.5 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYROFFS_X` | 0.00656938 | -0.00584416 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYROFFS_Y` | -0.0268651 | -0.0150338 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| A · Kalibrasyon | `INS_GYROFFS_Z` | -0.00595699 | -0.0015541 | otomatik (boot'ta yeniden hesaplanır) | Jiroskop offseti — her boot'ta ArduPilot otomatik yeniden kalibre eder, normal. |
| B · Güvenlik | `BATT_MONITOR` | 4 | 0 | **4** (düzeltildi) | Batarya voltaj/akım izleme tamamen kapanmıştı (0=Disabled). |
| B · Güvenlik | `COMPASS_USE2` | 0 | 1 | **0** (düzeltildi) | İkinci (kalibre olmayan) pusula EKF'ye dahil ediliyordu. |
| B · Güvenlik | `FS_ACTION` | 2 | 0 | **2** (düzeltildi) | Genel failsafe aksiyonu kapalıydı. |
| B · Güvenlik | `FS_EKF_ACTION` | 1 | 0 | **1** (düzeltildi) | EKF (konum/yön) hata failsafe aksiyonu kapalıydı. |
| B · Güvenlik | `FS_THR_ENABLE` | 1 | 0 | **1** (düzeltildi) | Throttle/RC sinyal kaybı failsafe'i kapalıydı. |
| B · Güvenlik | `INITIAL_MODE` | 4 | 0 | **4** (düzeltildi) | Açılışta güvenli HOLD yerine direkt MANUAL'a giriyordu. |
| C · RC (bekliyor) | `MODE_CH` | 6 | 8 | 8 (değiştirilmedi) | Mod anahtarının fiziksel kanalı değişmiş — RC'den mod değiştirilemeyebilir. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC10_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC10_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC10_OPTION` | 55 | 0 | 0 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC11_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC11_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC12_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC12_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC13_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC13_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC14_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC14_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC15_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC15_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC16_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC16_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC1_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC1_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC1_TRIM` | 1503 | 1500 | 1500 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC2_DZ` | 30 | 0 | 0 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC2_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC2_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC2_TRIM` | 1000 | 1500 | 1500 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC3_DZ` | 0 | 30 | 30 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC3_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC3_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC3_TRIM` | 1596 | 1500 | 1500 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC4_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC4_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC4_TRIM` | 1496 | 1500 | 1500 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC5_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC5_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC6_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC6_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC7_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC7_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC8_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC8_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC9_MAX` | 2000 | 1900 | 1900 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RC9_MIN` | 1000 | 1100 | 1100 (değiştirilmedi) | RC kanal kalibrasyonu. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RCMAP_PITCH` | 3 | 2 | 2 (değiştirilmedi) | Pitch kanalı haritası değişmiş (boat için kritik değil ama THROTTLE ile birlikte değişti, dikkat). RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| C · RC (bekliyor) | `RCMAP_THROTTLE` | 2 | 3 | 3 (değiştirilmedi) | Gaz kolunun hangi kanala haritalandığı değişmiş. RC kalibrasyonu/kanal haritası — fiziksel kumanda testi yapılmadan yazılmadı, TAKIMIN ONAYI BEKLENİYOR. |
| D · Servo | `SERVO15_REVERSED` | 0 | 1 | 1 (değiştirilmedi) | İlgili kanal zaten devre dışı (FUNCTION=0), zararsız. |
| D · Servo | `SERVO1_FUNCTION` | 74 | 73 | 73 (değiştirilmedi) | DİKKAT: burada "şimdi" (73) olan DOĞRU — masa testiyle teyitli. "Eski" (74) aslında yanlıştı, buna DOKUNMAYIN. |
| D · Servo | `SERVO1_REVERSED` | 0 | 1 | 1 (değiştirilmedi) | Kim/ne zaman değiştirdiği belirsiz — ana motoru etkiliyor, fiziksel test (pervanesiz kısa gaz, yön kontrolü) olmadan değiştirilmedi. |
| D · Servo | `SERVO3_FUNCTION` | 73 | 74 | 74 (değiştirilmedi) | DİKKAT: burada "şimdi" (74) olan DOĞRU — masa testiyle teyitli. "Eski" (73) aslında yanlıştı, buna DOKUNMAYIN. |
| E · Seri/telemetri | `SERIAL1_BAUD` | 19 | 57 | 57 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SERIAL2_OPTIONS` | 8 | 0 | 0 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_ADSB` | 0 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_EXTRA3` | 2 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_EXT_STAT` | 2 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_POSITION` | 2 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_RAW_CTRL` | 1 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_RAW_SENS` | 2 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR0_RC_CHAN` | 2 | 4 | 4 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_EXTRA1` | 4 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_EXTRA2` | 4 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_EXTRA3` | 2 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_EXT_STAT` | 2 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_PARAMS` | 1 | 10 | 10 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_POSITION` | 2 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_RAW_SENS` | 2 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR1_RC_CHAN` | 2 | 1 | 1 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| E · Seri/telemetri | `SR2_PARAMS` | 1 | 10 | 10 (değiştirilmedi) | Seri port / telemetri stream-rate — donanıma bağlı, kritik değil, isteğe bağlı. |
| F · Bilgi | `BARO1_GND_PRESS` | 92070.8 | 92003.7 | 92003.7 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `COMPASS_DEV_ID` | 0 | 658433 | 658433 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `COMPASS_DEV_ID2` | 658433 | 0 | 0 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `COMPASS_PRIO1_ID` | 658953 | 658433 | 658433 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `COMPASS_PRIO2_ID` | 658433 | 0 | 0 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `INS_ACC2_ID` | 2.81857e+06 | 2.6875e+06 | 2.6875e+06 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `INS_ACC3SCAL_X` | 0 | 1 | 1 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `INS_ACC3SCAL_Y` | 0 | 1 | 1 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `INS_ACC3SCAL_Z` | 0 | 1 | 1 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `INS_GYR2_ID` | 2.81831e+06 | 2.68724e+06 | 2.68724e+06 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `MIS_TOTAL` | 2 | 5 | 5 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `STAT_BOOTCNT` | 29 | 21 | 21 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `STAT_FLTTIME` | 344 | 475 | 475 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |
| F · Bilgi | `STAT_RUNTIME` | 16202 | 37567 | 37567 (değiştirilmedi) | Bilgi amaçlı / ortama bağlı / kendiliğinden değişir — düzeltmeye gerek YOK. |

### B grubu — Batarya monitör alt-parametreleri (BATT_MONITOR=0 iken tamamen kaybolmuşlardı)

| Parametre | Değer | Not |
|---|---|---|
| `BATT_AMP_OFFSET` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_AMP_PERVLT` | 36.36 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_ARM_MAH` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_ARM_VOLT` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_CAPACITY` | 19900 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_CRT_MAH` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_CRT_VOLT` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_CURR_PIN` | 4 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_FS_CRT_ACT` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_FS_LOW_ACT` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_FS_VOLTSRC` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_LOW_MAH` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_LOW_TIMER` | 10 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_LOW_VOLT` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_OPTIONS` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_SERIAL_NUM` | -1 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_VLT_OFFSET` | 0 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_VOLT_MULT` | 18.18 | ✅ Geri yazıldı, reboot sonrası doğrulandı |
| `BATT_VOLT_PIN` | 8 | ✅ Geri yazıldı, reboot sonrası doğrulandı |

## Takımdan beklenen: Grup C (RC kalibrasyonu)

Bu parametrelere kasıtlı olarak dokunulmadı çünkü fiziksel kumanda ile doğrulama gerektiriyor:

1. **`RCMAP_THROTTLE`** şu an `3`, olması gereken `2` — gaz kolunun hangi kanalda olduğunu fiilen test edin.
2. **`RCMAP_PITCH`** şu an `2`, olması gereken `3`.
3. **`MODE_CH`** şu an `8`, olması gereken `6` — mod anahtarınız fiziksel olarak hangi kanaldaysa onu doğrulayın.
4. **`RC1-16_MIN/MAX/TRIM/DZ`** — tamamı generic (1100/1900/1500) değerlere dönmüş, gerçek kalibre edilmiş uç değerler kayboldu.
5. **`SERVO1_REVERSED`** (Grup D) — ana motorun yönü, pervanesiz kısa gaz testiyle doğrulanmalı.

**Önerilen:** Mission Planner → Setup → Mandatory Hardware → Radio Calibration ile RC'yi yeniden kalibre edin (bu otomatik olarak MIN/MAX/TRIM/DZ'yi doğru dolduracaktır), sonrasında `RCMAP_*` ve `MODE_CH`'i gerçek kanal atamanıza göre elle doğrulayın.

## Kaynak veriler

- "Eski/doğru" değerler: 18 Temmuz 2026 saat 00:08'de (o günkü değişikliklerden önce) çekilen tam parametre dökümü.
- "Şimdi" değerler: bu oturumda (19 Temmuz 2026) düzeltme sonrası, reboot sonrası tekrar okunarak doğrulandı.
- Düzeltme MAVLink `PARAM_SET` ile canlı yapıldı, Mission Planner'dan da aynı ekranlardan (Full Parameter List) teyit edilebilir.
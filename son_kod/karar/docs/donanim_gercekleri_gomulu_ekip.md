# Gömülü Ekip Yığınından Devralınan Donanım Gerçekleri

Kaynak: IDA-TEAMM/IDA_GIT `CLAUDE.md` + kod (2026-07-13 denetimi, GİRDAP kopyası
EyupEker1/IDA_GIT `docs/kod_denetimi.md` Faz 14). Bu değerler gömülü ekipçe **gerçek
donanımda** doğrulanmış; bizim yığının kurulum/konfigürasyonuna girdi.

## Livox Mid-360 ağ değerleri (tcpdump ile doğrulanmış)

- Cihaz IP: **192.168.117.100** (genel dokümandaki 192.168.1.1xx DEĞİL)
- Nokta verisi hostun **56301** portuna gelir (doküman varsayılanı 56100 değil);
  IMU verisi 56401'e.
- Host ethernet: **192.168.117.50/24 statik** (nmcli ile kalıcı yapılmıştı — Jetson'da
  livox_ros_driver2 kurulurken aynı ayar gerekecek).
- ⇒ livox_ros_driver2 `MID360_config.json` alanlarına bu IP'ler yazılacak.
- ⚠️ Gömülü ekibin kendi UDP driver'ı paket ayrıştırmasını protokole aykırı yapıyor
  (başlık 28≠36 bayt, float32≠int32 — IDA_GIT denetimi V25); köprü olarak
  kullanılmadan ÖNCE bizim kopyada düzeltildi. livox_ros_driver2 gelene kadar geçici
  köprü: `ros2 run ida_topics livox_driver_node --ros-args -r /lidar/points:=/livox/lidar`.

## F9P GPS mimarisi

- Holybro H-RTK F9P Rover (IST8310 kompas) **tek birleşik konnektörle yalnız Pixhawk
  GPS1'e** bağlı; bağımsız ikinci UART/USB çıkışı YOK (FTDI ile pin tapping denenmiş,
  tanımlanamayan ikili protokol gelmiş).
- ⇒ Companion tarafında GPS/IMU'nun TEK yolu MAVROS (`/mavros/global_position/global`,
  `/mavros/imu/data`) — bizim fusion'ın zaten varsaydığı yol ✓. Ayrı GPS seri portu
  ARAMAYIN.

## Pixhawk bağlantı portları — ⚠️ ÇELİŞKİ, çapraz teyit gerekli

- Gömülü ekip: hem telemetri radyo (`/dev/ttyUSB0`, CP2102) hem **Pixhawk'ın kendi
  doğrudan USB'si (`/dev/ttyACM0`, "Pixhawk6C"/Holybro) ÇALIŞIYOR**, ikisi de
  `fcu_url=serial://<port>:57600`.
- Bizim donanım günlüğünde ise "Pixhawk USB-C soketi arızalı şüphesi" (descriptor -32)
  kayıtlı. İki gözlem çelişiyor — muhtemel açıklamalar: farklı kablo, farklı port
  (USB-C vs micro-USB), soket sonradan bozuldu, ya da bizim gördüğümüz USB hub/kablo
  sorunuydu. **Masa testinde çapraz teyit: aynı Pixhawk + gömülü ekibin kablosu.**
- Bir oturumda MAVROS heartbeat'te bir kez "PX4 Autopilot" görülmüş (beklenen
  "ArduPilot") — tekrarlarsa araştırılacak.

## RC kanal düzeni (gömülü ekip kullanımı)

- Kanal 8 (idx 7): kill-switch (<1500 aktif) · Kanal 5 (idx 4): mod seçimi (>1700 manuel).
- `docs/fc_parametre_onerileri.md`'deki "RC mod kanalı / FLTMODE düzeni" sorusuna girdi —
  FC ekibi RC düzenini netleştirirken bu mevcut atamayı bilsin (FC sahte-görev OLAYI'nda
  şüpheli CH5 atlaması bu kanal!).
- T1 işi (IDA_GIT denetimi E3): RC ch8 kill izlemesini bizim mavros_bridge kill zincirine
  (kill servis + disarm) bağlama reçetesi IDA_GIT `docs/kod_denetimi.md` Faz 9'da.

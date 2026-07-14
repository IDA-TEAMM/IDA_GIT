# son_kod — GİRDAP İDA Kanonik Karar Yazılımı (Yahya)

**Son teslim: 21.07.2026 17:00 — Otonomi Kabiliyeti videosu (ELEME KAPISI).**
Bu repo, TEKNOFEST 2026 İDA yarışması (Takım GİRDAP, ID 989124) karar/otonomi
yazılımının **Yahya Seha Danış tarafındaki kanonik kopyasıdır**: girdap-video
(Eyüp) + girdap-decision fix'leri + sensör sürücüleri (ida_topics) tek repoda,
tam bağımsız çalışır halde.

## Repo haritası

```
son_kod/
├── karar/                        ← karar yığınının tamamı
│   ├── prototype/                ← ROS-bağımsız çekirdekler (pytest ile)
│   ├── ros2_ws/src/girdap_decision/   ← ROS 2 node'ları + config + launch
│   ├── ros2_ws/src/ida_topics_yeni/   ← sürücüler: Livox UDP, OAK-D,
│   │                                    kamera_kayit (Dosya-1), gps_imu
│   ├── scripts/                  ← girdap-karar.service, run_ekran2.py
│   ├── docs/                     ← hata_defteri.md (canlı bug kaydı), runbook'lar
│   └── testler/                  ← bileşen kanıtı: video_testleri.sh
├── memory/                       ← Claude hafıza anlık görüntüleri
├── KOD_DEGISIKLIKLERI.txt        ← insan-okur değişiklik günlüğü (bölüm bölüm)
├── fake_mavros_publisher.py      ← masa testi sahte veri üreticisi (repo-dışı yardımcı)
├── ground_speed_publisher.py     ← WP_SPEED setpoint yayıncısı (yardımcı)
└── plotjuggler_girdap.xml        ← canlı grafik düzeni (laptop)
```

## Video günü — tek bakışta (AUTO modu)

Ayrıntı: `karar/docs/video_gunu_runbook.md` **§0-A** (AUTO dönüşü — eski GUIDED
adımlarını ezer). Özet akış:

1. Jetson'a güç ver → `girdap-karar.service` yığını kendisi kaldırır
   (boot sonrası **~1 dk bekle** — ttyACM/ttyUSB kısa süre meşgul kalabiliyor).
2. QGC: Plan → **Remove All** → gerçek 4 nokta → **Upload** (md 3.3.1(2);
   FC hafızasında eski görev bırakma — OPS-1 kuralı!).
3. ARM → mod **AUTO** (sıra fark etmez; `start_on_arm_in_mode: true`).
4. Görev biter → manuel dönüş → güvenlik anahtarıyla güç kesme gösterimi.
5. Montaj: `python karar/scripts/run_ekran2.py --mp4 --thrust-birim %`
   (fc modunda thrust YÜZDE'dir — birimi unutma).

Kritik config: `karar/ros2_ws/src/girdap_decision/config/hardware.yaml`
(video değerleri) ↔ `config/params.yaml` (yarışma varsayılanları).
`fcu_url` artık **ttyUSB0** (TELEM2 FTDI — USB-C soketi güvenilmez, F-M.9).

## Build + test (DOĞRU sırayla — yoksa testler sessizce atlanır!)

```bash
cd karar/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..
export PYTHONPATH="$PWD:$PYTHONPATH"        # BAŞA ekle, üzerine YAZMA
source ros2_ws/install/setup.bash           # ← BUNU UNUTMA
python3 -m pytest prototype/tests -q
```

⚠️ `ros2_ws/install/setup.bash` source'lanmazsa TÜM ROS node testleri
"girdap_decision source'lanmamış" gerekçesiyle SKIP olur — suite küçülmüş
görünür (ör. 227 passed/15 skipped). Bu bir gerileme DEĞİLDİR; ortamı düzelt.
Referans taban: **bkz. KOD_DEGISIKLIKLERI.txt son bölüm** (her değişiklikte
güncellenir). Node testleri izole DDS'te koşmak için: `ROS_DOMAIN_ID=77`.
Not: bu makinede colcon install'ı KOPYA üretir — node kodu değişince testten
önce `colcon build --packages-select girdap_decision` şarttır.

## Sahte veriyle masa testi (Pixhawk'sız)

```bash
# Terminal 1 — yığın (sürücüsüz, dosyadan görev):
ros2 launch girdap_decision hardware.launch.py mission_source:=file
# Terminal 2 — sahte MAVROS verisi (dikdörtgen tur):
python3 fake_mavros_publisher.py       # GERÇEK MAVROS açıkken ASLA çalıştırma
# Bileşen kanıtları tek komutla:
bash karar/testler/video_testleri.sh
```

## Senkron kuralları

- **IDA_GIT içinde yalnız `son_kod/` düzenlenir**; dış klasör değişimleri
  taranır ve buraya UYARLANIR (tek yön). SessionStart hook'u otomatik tarar.
- Push sırası: `girdap-kaptan-video` (origin) → `IDA_GIT/son_kod/` (rebase'li).
- Eyüp'ün hattı: github.com/EyupEker1/girdap-video — **Jetson'da KURULU olan
  odur**; iki hat bulguları `karar/docs/hata_defteri.md` üzerinden paylaşır.
- Her kod değişikliği: TDD + tam suite + `KOD_DEGISIKLIKLERI.txt` + memory.

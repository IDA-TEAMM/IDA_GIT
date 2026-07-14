# Memory Index

## Kullanıcı
- [Kullanıcı rolü](user-role.md) — Yahya Seha Danış: yazılım kaptanı + FC ekibi üyesi (FC parametreleri kendi yetkisinde)
- [Büyük işleri kaydet](feedback-buyuk-isleri-kaydet.md) — her büyük değişiklik/iş sonunda sormadan hafızaya kaydet

## İDA Projesi
- [Eyüp memory senkron](eyup-memory-senkron.md) — 🔴 JETSON'DA ONUN girdap-video'su kurulu (taban 302/2); F-M.9: fcu_url=ttyUSB0 (USB-C arızalı, udev DU0EFEA7); son_kodv2 F-S bulguları; P0=video-eleme listesi
- [Şartname 2026 uygunluk](ida-sartname-2026.md) — V1.2 denetimi: uyumlular + 5 kritik risk (RC/MicoAir frekans, Wi-Fi, güç-kesme rölesi, Dosya-1); değişimler buna göre
- [İDA proje genel](ida-project.md) — GİRDAP İDA, TEKNOFEST 2026, katamaran USV + İHA, 3 parkur
- [Yazılım durumu](ida-software-status.md) — 5 katman + boşluk analizi; planlama/kontrol darboğazı
- [Sim workspace](ida-sim-workspace.md) — ~/Desktop/girdap_ida_ws, Gazebo Harmonic kurulumu + bilinen yol/build sorunları
- [IDA_GIT + decision depoları](ida-git-decision-repos.md) — masaüstü 2 kod deposu + derleme tuzakları (IDA_GIT'te tüm ROS2 kaynağı tuzağı)
- [Donanım/komponentler](ida-hardware.md) — İDA+İHA komponent listesi ve şüpheli girdiler
- [Telemetri](ida-e32-lora.md) — GÜNCEL: araçta MicoAir telemetri modülü (SERIAL1 57600/MAVLink2). E32 433MHz artık kullanılmıyor (eski kayıt dosyada)
- [OAK-D Lite topic](ida-oak-d-lite.md) — depthai_ros_driver; udev + USB2/RGB-only config şart, yoksa X_LINK_ERROR
- [Livox Mid-360 topic](ida-livox-mid360.md) — livox_ros_driver2; config IP + SDK build sırası + PointCloud2 varyantı
- [Perception canlı](ida-perception-canli.md) — read_points fix + füzyon sync_queue_size gecikme çözümü (Option 2)
- [Jetson ethernet](ida-jetson-ethernet.md) — laptop eno1 .1 ↔ Jetson .50; systemd-networkd kalıcı + NM split-brain (.50) tuzağı çözümü
- [Video AUTO kararı](ida-video-auto.md) — video (21.07) AUTO modla; B1/B2 kod fixleri, FC param planı, 20×30 m + upload→ARM→AUTO sıra kuralı, Jetson pasif kayıtçı
- [Batarya izleme PM06](ida-batarya-pm06.md) — Daly BMS CAN çıkmazı; PM06→POWER1 planı, 6C param değerleri, kalan iş: voltage_battery teşhisi + kalibrasyon
- [Decision repo inceleme](ida-decision-repo-review.md) — tam mimari/test/risk incelemesi; KANONİK: ~/Desktop/son_kod (birleşik repo + memory kopyası)
- [Kod düzeltme günlüğü](kod-duzeltme.md) — fix günlüğü; son_kod VİDEOYA HAZIR (14 Tem gece: suite 287/4 + canlı sahte-veri turu TAMAMLANDI); "testler kötü"=source eksikliği tuzağı; ROS_DOMAIN_ID=42/77

---
name: fm9-telem2-cozuldu
description: "F-M.9 (USB→mavros ölümü) TELEM2 hattıyla çözüldü; respawn yazılım fix'i bilinçli istenmiyor; F-L.2 video işi değil"
metadata:
  node_type: memory
  type: project
---

F-M.9 (Pixhawk USB düşünce mavros ölüp geri gelmiyor → yığın kalıcı KILL) **çözüldü**:
MAVLink hattı titrek USB-C soketinden **TELEM2 FTDI kalıcı hattına** taşındı.
`hardware.yaml` → `fcu_url: serial:///dev/pixhawk:57600` (udev symlink
`/dev/pixhawk`, 2026-07-15 kuruldu + canlı doğrulandı). USB-C bypass edildiği
için F-M.9'un arıza senaryosu (USB hıçkırığı) artık oluşamaz.

**Yazılım `respawn_mavros` fix'i BİLİNÇLİ İSTENMİYOR** — kalan iş DEĞİL. Hata
defterinin F-M.9 kaydının kendi sonucu: "yazılım respawn/histerezis bu arızayı
ÖRTME aracı değil — kırılgan güvenlik hattını yazılımla otomatik kurtarmak suda
tanımsız duruma dönebilir; kök çözüm donanım." KILL'in latch olması doğru
güvenlik davranışı.

**Why:** 2026-07-17'de kalan hataları özetlerken "F-M.9 respawn = kalan tek
yazılım işi" dedim; kullanıcı düzeltti — TELEM2 ile zaten çözülmüştü ve respawn
istenmeyen bir yaklaşım.

**How to apply:** Kalan-iş/hata özetlerinde F-M.9 respawn'ı YAZILIM işi olarak
listeleme. F-M.9 = kapalı (donanım/TELEM2 ile). Video-yolu yazılımında
gerçek anlamda kalan iş YOK (testler 340/2 yeşil). Kalan blokerler saha/FC
(FC-OLAY-2 gaz kanalı + RC kalibrasyonu) ve video-sonrası (NN model).
Not: hata_defteri F-M.9 durumu "🔴 AÇIK donanım blokeri" olarak BAYAT kalmış
olabilir (TELEM2 aksiyonu tamamlandı) — özetlerken defterin ham durumuna değil
bu gerçeğe göre konuş. İlgili: [[grafik-formati-ekran2]].

**İş bölümü (2026-07-17):** Bu ekip (karar-yazılım) T1 hatalarına yoğunlaşıyor;
**FC-OLAY-2 FC ekibinin işi, bu ekip YAPMAYACAK.**

**F-L.2 VİDEO İŞİ DEĞİL — Parkur-2/yarışma işi.** F-L.2 = kamera-LiDAR bearing
FÜZYON sync'i (`/perception/classified_obstacles`); ama `planning_node`
`/perception/obstacle_map` (ham LiDAR) dinliyor, füzyon çıktısını DEĞİL → füzyon
sürüşü hiç beslemiyor. Video (4-nokta dikdörtgen, engel yok) füzyonu çalıştırmaz.
Defterin kendi notu: "video bypass'ında sürüşü etkilemez, Parkur-2 işi". F-L.2'yi
video kalan-iş listesine KOYMA.

**Video T1 işi = SADECE F-M.8 (boot busy) + F-M.5 (seri kopma); ikisi de yalnız
Pixhawk ister.** Yani video T1 için Pixhawk'ı TELEM2/FTDI (`/dev/pixhawk`)
hattından bağlamak YETERLİ (LiDAR/kamera video amacıyla gerekmez — onlar Parkur-2
+ Dosya-1/3 teslimleri için). F-M.8 kapanış kriteri = Pixhawk takılıyken boot →
`/dev/pixhawk` oluşuyor + ~30 sn busy gecikmesi kalkıyor.

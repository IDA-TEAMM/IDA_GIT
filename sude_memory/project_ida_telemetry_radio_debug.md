---
name: project-ida-telemetry-radio-debug
description: IDA USV telemetry radio <-> QGroundControl connection debugging session state
metadata: 
  node_type: memory
  type: project
  originSessionId: 0177bfd4-f55e-432e-8505-9f5599a8723d
---

**Sorun:** QGroundControl telemetri radyosu (ground-side, `/dev/ttyUSB0`, CP210x çipli, 57600 baud) ile Pixhawk'a bağlanamıyor, sürekli "Disconnected - Click to manually connect" gösteriyor.

**2026-07-12 oturumunda yapılanlar / bulunanlar:**
- Fiziksel katman muhtemelen sağlıklı: iki telsizin LED'i de sabit yanıyor (RF link kurulu görünüyor), Pixhawk USB (wall charger) ile güçlü.
- QGC uygulaması "already running" diyip açılmıyordu → eski process kill edilip (`kill <pid>`) temiz yeniden başlatıldı, pencere göründü.
- QGC'de hiç Comm Link tanımlı değildi ("No Links Configured") → Application Settings > Comm Links'ten elle "telemetri radyo" adında Serial link eklendi: port `ttyUSB0`, baud `57600`, Automatically Connect on Start açık. Kaydedildi.
- Manuel bağlanmayı denedi, yine "Disconnected" kaldı.
- **Kök neden bulundu:** `/dev/ttyUSB0` izinleri `root:dialout 660`. Kullanıcı `sudenaz` diskte (`getent group dialout`) dialout üyesi görünüyor, AMA o an açık olan masaüstü oturumunun (ve QGC dahil tüm süreçlerinin) CANLI grup listesinde dialout yoktu (`/proc/<pid>/status` Groups satırında sadece `10 36 969 1000` vardı, 18/dialout eksikti). Bu, dialout grubuna eklendikten sonra oturumun yenilenmemiş olmasından kaynaklanıyor — klasik Linux davranışı, `usermod -aG` sadece yeni login'de etkili olur.
- **Verilen çözüm (henüz doğrulanmadı):** Kullanıcının log out/login yapması (ya da reboot) gerekiyor, ardından `id` çıktısında `dialout` görünmeli ve QGC portu açabilmeli.

**ÇÖZÜLDÜ (2026-07-13):** Kullanıcı logout/login yaptı, `id` çıktısında `dialout` artık canlı oturumda göründü (`uid=1000(sudenaz) gruplar=...,18(dialout),...`). QGC açıldı, kayıtlı "telemetri radyo" Comm Link ile bağlandı ve başarılı oldu — ekran görüntüsünde PX4 Autopilot tanınmış, GPS 16 uydu kilitli (HDOP 0.7), pil "Ok", harita üzerinde canlı konum akıyor (Abant İzzet Baysal Üniversitesi civarı). Flight mode "Hold", durum "Not Ready" — bu normal, pre-arm check'ler tamamlanmadan/arm edilmeden beklenen durum, sorun değil.

**How to apply:** Bu sorun kapandı, tekrar açılırsa (örn. yeni bir oturumda dialout grubu yine "canlı degil" hatası verirse) aynı kök nedeni kontrol et: `id` çıktısında `dialout` var mı.

**"Not Ready" nedeni bulundu (2026-07-13):** QGC Vehicle Configuration ekranında görüldü — Sensors ve GPS yeşil/saglikli (17 uydu, 3D Lock, HDOP 0.7), ama **Radio (RC kalibrasyonu yapılmamış), Flight Modes (switch atamaları yok), Power (batarya hücre sayısı 0)** kırmızı/"Setup required" durumda. Bu bir hata degil, ilk kurulumun tamamlanmamış olması — normal. Arm etmeden once bu 3 adımın QGC'de tamamlanmasi gerekiyor (Radio kalibrasyonu icin RC kumanda acik/bagli olmali). Kullanıcı 2026-07-13'te "şu an arm etmeyeceğiz, donanım üzerinden node/driver testine devam" dedi — yani bu 3 kurulum adımı simdilik ERTELENDİ, bir sonraki oturumda "RC kalibre ettik mi" diye kullanıcıya sorulabilir.

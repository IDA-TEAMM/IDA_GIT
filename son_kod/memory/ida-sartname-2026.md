---
name: ida-sartname-2026
description: 2026 İDA şartnamesi (V1.2) uygunluk denetimi — kod/QGC uyumlu maddeler + kritik riskler; TÜM gelecek değişimler buna göre
metadata: 
  node_type: memory
  type: project
  originSessionId: 97fbf033-cf13-4546-afbf-c6b146529521
---

**2026 İDA Şartnamesi V1.2 (18.05.2026) tam okundu ve son_kod + QGC akışıyla denetlendi (2026-07-14).** Kaynak: `~/Downloads/2026_İnsansız_Deniz_Araci_Şartnamesi_TR_18_05_Mh9Vx.pdf`. TÜM gelecek kod/prosedür değişimleri bu maddelere göre yapılmalı.

**✓ UYUMLU (kod/QGC):**
- md 3.3.1 video akışı: YKİ'den 4 nokta dikdörtgen upload → YKİ/RC komutla başlat → son noktada otonom görev biter → manuel dönüş = son_kod (mission_source=fc, start_on_mode=AUTO kenar tetik, F-V.4 dönüş manuel) + runbook birebir. Güç kesme (md 3.3.1(4)) ve su almazlık (md 3.3.1(5)) sahneleri video_gunu_runbook §5.2-5.4'te var.
- md 3.3.1.1 Ekran-1/2/3 + Ekran-2 üç sinyal (hız+istek, heading+istek, thruster kuvvet İSTEĞİ) = telemetry_node grafik CSV + run_ekran2. "İstemsiz hareket = başarısız" → B1 auto_guided=false fix'i tam bu riski kapatır.
- Video: kesintisiz tek çekim, ≥720p, 2-5 dk, YouTube liste dışı, KYS'ye link, son 21.07 17:00. 20×30 m @1 m/s görev ~110 s; upload+güç kesme+kapak dahil toplam süre 2 dk'yı rahat aşar — 5 dk ÜST sınırına dikkat.
- md 4.2 Dosya-2 (CSV ≥1Hz: lat/lon, yer hızı, roll/pitch/heading, hız sp, yön sp, header) = telemetry_node 2 Hz ✓. Dosya-3 (lokal harita ≥1Hz) = local_map_node ✓. Teslim: karaya alımdan 20 dk içinde takımın kendi USB'siyle; dosya başı 5 ceza.
- md 4.1 "YKİ'de görüntü işleme/otonomi OLMAYACAK, yazılım araç üstünde" = mimari zaten böyle (her şey Jetson'da). md 5.5.3.1 "YKİ'de kod başlatma YOK, sadece YKİ arayüzü açık" = girdap-karar.service otomatik başlatma bu kuralı KARŞILAR (operatör kod başlatmaz).
- Görev başladıktan sonra YKİ/RC komut YASAK (acil motor kesme hariç); görev/hareket başlat komutu KABLOLU verilemez (kablosuz upload ise operatör çadıra geçmez) ✓ akışımız uyumlu.
- md 5.5.2.2: parkurlar arası geçiş kullanıcı girişi OLMADAN otomatik = FSM ✓; önceden haritalama yasak, engel konumu verilmez = onboard RRT*/MPPI ✓. Görev noktaları dd.ddddddd dosya olarak verilecek (USB ile alınır).

**⚠ KRİTİK RİSKLER (yapılacak):**
1. **Frekans (md 4.1):** 2.4-2.8 GHz ve 5.15-5.85 GHz YASAK — telemetri VE RC dahil. **MicoAir = 868 MHz UYGUN ✓** (14.07). **RC = Radiolink AT9S Pro = 2.4 GHz → FİNAL İÇİN YASAK** (video için pratikte engel değil, denetim finalde). Dahili 2.4 GHz DONANIMSAL — menüden/yazılımla banda müdahale EDİLEMEZ. Tek çözüm: AT9S Pro CRSF moduna alınır → harici Crossfire/ELRS 868 MHz TX modülü + 868 alıcı (kumanda kalır; CRSF modunda dahili RF'in sustuğu teknik kontrolden önce doğrulanmalı); geçişte QGC'de yeniden RC kalibrasyonu + FS_THR_VALUE yeniden + (CRSF serial ise SERIALx_PROTOCOL=23, RSSI_TYPE=3). Kalan: MicoAir kanal seçilebilirlik teyidi. İhlal: 55 ceza / teknik kontrolden kalma.
2. **Wi-Fi kapatma (md 4.1):** Jetson + YKİ laptop dahil TÜM bilgisayarların dahili Wi-Fi'si KAPALI olacak. Prosedüre eklenecek (Jetson'da rfkill).
3. **Uzaktan GÜÇ kesme (md 4.2):** "Motor sinyalini kesmek YETMEZ, motorların GÜCÜ kesilmeli" — YKİ yazılımı ya da RC'den tetiklenen fiziksel röle ŞART. Yazılım KILL'imiz (sıfır thrust) şartnameye GÖRE YETERSİZ; donanım ekibiyle röle durumu teyit edilmeli. Ayrıca araç üstünde KIRMIZI fiziksel anahtar şart (var).
4. **Dosya-1 üreticisi kodda YOK (md 4.2):** işlenmiş kamera mp4 (≥1Hz, zaman etiketli, tespit çerçeveli) + lidar vb. her sensör tipi için ayrı mp4 (kümeleme görünür). Final öncesi yazılmalı (video için gerekmez).
5. Video çekiminde Ekran-1 = YKİ ekranı: QGC dışında pencere (PlotJuggler vb.) GÖRÜNMESİN — "YKİ'de otonomi yok" algısı ve 5.5.3.1 kuralıyla tutarlılık. PlotJuggler ayrı monitör/makine.

**Yarışma günü notları:** süre 20 dk (P3 sonrası dönüş hariç); 1 kez yeniden başlama (puanlar sıfırlanır) + 1 pas hakkı (10 ceza); alanda ≤6 kişi, çadırda ≤2 (yarış alanını göremez); denize bırakan (≤4 kişi) çadıra giremez; dubaya 30 sn temas = 2 çarpma; parkur dışında 40 sn = 2 çıkış. Puan: KTR 15 + P1 55 + P2 100 + P3 145. Ödül sıralaması için P1+P2 tamamlanmalı; P1 tamamlama şartı geçiş puanı ≥5. Yarışma sırasında yarışan takım dışındakiler haberleşme modüllerine güç veremez (150 ceza).

İlgili: [[ida-video-auto]] · [[ida-decision-repo-review]] · [[kod-duzeltme]] · [[ida-e32-lora]] · [[ida-hardware]]

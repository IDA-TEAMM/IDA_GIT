# Yol Haritası — Kalan Hatalar (karar-yazılım ekibi)

> Güncel: 2026-07-17. Kaynak: `docs/hata_defteri.md` + kod incelemesi.
> Kapsam: bu ekip (karar-yazılım) T1'e yoğunlaşıyor; FC-OLAY-2 FC ekibinde.
> Video son teslim **21.07.2026**. Testler: **340 passed / 2 skipped.**

## Durum tablosu

| Kod | Öncelik | Video? | Neye ihtiyaç | Kalan iş |
|---|---|---|---|---|
| **F-M.8** | 🟡 T1 | ✅ evet | Pixhawk (TELEM2) | Boot doğrulaması |
| **F-M.5** | 🟡 T1 | dolaylı | Pixhawk (TELEM2) | Karar: respawn mı, KILL-latch mı |
| **F-L.2** | 🟡 T1 | ❌ Parkur-2 | Livox + OAK-D | Suda sync teyidi |
| FC-OLAY-2 | 🔴 | ✅ evet | — | **FC ekibi** (bu ekip değil) |
| MODEL (NN) | 🔴 | ❌ | — | Video sonrası |

---

## F-M.8 — boot'ta mavros ~30 sn geç bağlanıyor

**Ne:** Taze boot'ta `ttyACM0 busy` → mavros_router 30 sn sabit retry → bağlantı
~44 sn'de kuruluyor. Kök neden: ModemManager ACM portlarını yokluyor + router'ın
parametresiz 30 sn retry'ı gecikmeyi büyütüyor. **Zararsız** (F-M.7 sayesinde KILL
yok), sadece boot yavaş.

**Yapılan:** `scripts/udev/99-girdap-fc.rules` deploy edildi (2026-07-15) —
(a) ModemManager-ignore (Pixhawk idVendor 3162 + FTDI), (b) `/dev/pixhawk` FTDI
symlink. TELEM2/FTDI hattı ACM sorununu zaten bypass ediyor.

**Kalan (Pixhawk TELEM2'den bağlı, tek boot):**
1. `ls -l /dev/pixhawk` → ttyUSBx'e işaret ediyor mu?
2. `journalctl -u girdap-karar -b` → `busy` satırı var mı? `Got HEARTBEAT`'e kaç sn?
3. Busy gecikmesi kalktıysa **F-M.8 KAPANIR**. Kalırsa: router retry süresini
   düşürmeyi değerlendir (mavros_router param — upstream, dikkatli).

**Çıkar:** yalnız Pixhawk yeterli.

---

## F-M.5 — seri kopunca mavros SIGABRT (respawn yok)

**Ne:** Seri hat koparsa (`serial0: End of file`) mavros `std::system_error` →
exit -6. Hiçbir node'a respawn yok. **Video'yu bloke etmez** (F14.4 KILL-latch
zaten kalıcı → bilinçli stack restart gerekir). TELEM2 kalıcı hattında bu senaryo
pratikte oluşmaz.

**Karar gereken (kod DEĞİL, tasarım):** respawn `true` yapmak KILL-latch varken
işe yaramaz (mavros geri gelse de KILL temizlenmez). İki seçenek:
- **(A) Dokunma** — mevcut davranış güvenli (kopma = görev bitti, elle restart).
  Hata defteri F-M.9 sonucu bunu savunuyor ("yazılımla otomatik kurtarma suda
  tanımsız duruma dönebilir").
- **(B) respawn + KILL histerezis** — mavros geri gelince KILL temizlensin.
  Riskli; yalnız kapsamlı suda test sonrası.

**Öneri:** video için **(A)** — dokunma. (B) yarışma sonrası araştırma.

---

## F-L.2 — kamera-LiDAR füzyon sync (~0.2 s kayma)

**Ne:** Livox stamp'i Jetson saatinden ~0.2 s geride → bearing eşleşmesi kaymalı.
**VİDEO İŞİ DEĞİL** — `planning_node` `/perception/obstacle_map` (ham LiDAR)
dinliyor, füzyon çıktısını (`classified_obstacles`) değil → füzyon sürüşü
beslemiyor. Parkur-2 (renk sınıflı engel kaçınma) işi.

**Yapılan:** `sync_slop_s` 0.1→0.3 (4 config kaynağı, TDD 4 test). Eşleşme-kaybı
riski kapandı.

**Kalan (Livox + OAK-D bağlı, video amaçlı DEĞİL):**
1. İki topic akışında sync `_watchdog` WARN basıyor mu (basmıyorsa eşleşme sağlam)?
2. Ölçülen kayma < slop (0.3) mu? Değilse slop büyüt veya restamp/PTP (T1).
3. Bearing işaret kuralı sahada doğru mu (sol/sağ ters çıkarsa `bearing_from_camera`
   işaretini çevir — modül docstring'i).

---

## Öncelik sırası (bu ekip)

1. **F-M.8 boot doğrulaması** — Pixhawk TELEM2'den bağla, tek boot, `/dev/pixhawk`
   + busy kontrolü. (Hızlı, video-ilgili.)
2. **F-M.5 kararı** — (A) dokunma önerisi onaylanırsa defterde T1→kapalı-not.
3. **F-L.2** — yarışma hazırlığında Livox+OAK ile suda teyit (video sonrası).

**Video için karar-yazılım blokeri YOK.** Tek video blokeri FC-OLAY-2 (FC ekibi).

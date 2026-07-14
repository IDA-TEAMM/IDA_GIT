# PC Günlüğü — 2026-07-12 (masa raporlarının incelenmesi + F-M düzeltmeleri)

> PC oturumu: Jetson'daki 14 commit'lik masa/donanım raporları incelendi,
> iki açık bulgu (F-M.1, F-M.2) TDD ile düzeltildi. Bu dosya "ne yaptık +
> sırada ne var" özetidir — ayrıntı için işaret edilen dokümanlara bak.

## ✅ Bu oturumda yapılanlar

1. **Raporlar incelendi, repolar senkronlandı.** İki repo da pull'landı;
   yedek repo zaten günceldi, "c742284 push'lanmamış" notu bayattı (origin'de
   varmış). Kod diff'leri doğrulandı: CUDA Faz B kernel'i ve F-L.1 düzeltmesi
   sağlam.

2. **F-M.1 DÜZELTİLDİ (`dff52af`)** — masadaki 92 GB çökmesinin sigortası:
   - GPS `(0,0)` verirken (fix yokken ArduPilot'un bastığı sahte konum)
     artık konum SAYILMAZ.
   - Geçerli fix yokken görev başlatılmaz ("GPS fix yok" uyarısı basar,
     fix gelince kendiliğinden başlar — kilitlenme yok).
   - Hedef 10 km'den uzaksa görev REDDEDİLİR ("koordinatları kontrol et"
     hatası) — yanlış koordinat sahada da girilse tekne beyinsiz kalmaz.
   - MPPI referans nokta tavanı (2048) — en kötü durumda bile bir daha o
     boyutta bellek isteyemez; normal yarışma rotası tavana değmez.
   - Yeni parametre: `max_target_distance_m` (params.yaml, 10 km).

3. **F-M.2 DÜZELTİLDİ (`3931220`)** — kasıtlı disarm'ın "FAILSAFE → KILL"
   sanılması. Kök neden yarış koşulu değil, tek satırlık latch hatasıymış:
   kod "araç arm oldu mu"yu hiç sıfırlamıyordu → disarm anı her kontrol
   turunda YENİDEN görülüyordu; "beklenen disarm" izni tek kullanımlık
   olduğundan ikinci turda sahte alarm basıyordu. Düzeltme: kenar takibi
   (`_was_armed = armed`). Testin kırmızı hali masadaki log satırını birebir
   üretti — doğru şey düzeltildi. Videodaki güç-kesme gösterimini (md
   3.3.1/4) doğrudan ilgilendiriyordu.

4. **Runbook güncellendi (`3e0f7ae`):**
   - Masa runbook'unun EN BAŞINA **"M0-ÖNCESİ: FC'ye güç vermeden zorunlu
     kontrol"** bloğu eklendi — dünkü kaçak motor OLAY'ının 4 aksiyonu
     (RC mod anahtarı konumu · FC görev hafızasını sil · BRD_SAFETY_DEFLT=1
     geri · batarya takılı bırakma). **Bir dahaki güç verişte önce bunu aç.**
   - M6/2'ye F-M.2 teyit notu (disarm SONRASI tick'leri de izle).
   - Doğrulama matrisine F-M.1/F-M.2 satırları.

5. **Doğrulama:** her düzeltme TDD (önce kırmızı, sonra yeşil). Suite yeni
   taban: **Jetson 259 passed / 2 skip** (önce `git pull`!), PC/GPU'suz
   karşılığı 257/4. CI (GitHub Actions) her iki fix commit'inde YEŞİL.

## 📋 SIRADA NE VAR (öncelik sırasıyla)

**Kod tarafında acil iş YOK.** Kalanlar insan/donanım işi:

1. **Eyüp (bugün, PC/telefon):**
   - [ ] `docs/olcum_formu.md`'yi mekanik + FC ekibine GÖNDER (Livox `h`
     gelmeden Parkur-2 sahaya çıkamaz — en eski bekleyen iş).
   - [ ] `Gazebonew.pt`'yi buluta/harici diske yedekle (tek eğitilmiş model;
     mevcut yedek aynı diskte).
   - [ ] FC ekibine: OLAY raporu (donanim_gunlugu_2026-07-12.md §OLAY) +
     Pixhawk USB-C soket şüphesi + RC kalibrasyon notları.

2. **Bir dahaki Jetson oturumu:**
   - [ ] `git pull` (taban 259/2 olmalı).
   - [ ] Runbook "M0-ÖNCESİ" güvenlik bloğunu uygula (pervanesiz!).

3. **Açık alan günü (GPS gören park/bahçe — su gerekmez):**
   - [ ] M5 tam (QGC'den GUIDED tetiği) → **M6 KILL zinciri (F-M.2'nin
     gerçek FCU teyidi)** → M7 dolu kayıtlar + run_ekran2 → M8 gerçek D3.
   - [ ] QGC laptop ayarlanırsa M2 (RFD kablosuz hat) + gerçek Plan Upload.

4. **Sonra:** suda 4 nokta provası → **VİDEO ÇEKİMİ** (son teslim
   **21.07 17:00 — ELEME KAPISI**; açık alan bu hafta biterse videoya
   2-3 gün pay kalır). Prosedür: `docs/video_gunu_runbook.md`.

5. **Video sonrası (T1):** NN Archive üretimi (birlikte) · F5.1 lidar_height
   (`h` gelince) · F-L.2 restamp kararı · arkadaş dönünce upstream PR.

# TEST PLANI — videoya giden yol (yazım: 2026-07-14)

> Video son teslim: **21.07.2026 17:00 (ELEME)** — bugünden 7 gün.
> Bu plan mevcut durumdan çekime kadar olan HER test oturumunu sıralar.
> Ayrıntılı prosedürler: `karar/docs/masa_testi_runbook.md` (M0-M8) +
> `karar/docs/video_gunu_runbook.md`. Bu dosya "hangi gün ne yapılacak" katmanı.
>
> **Mevcut durum (14.07):** M0 ✅ M1 ✅ M3 ✅ M4 ✅ · M5 yarım (GPS şart) ·
> M6 HİÇ koşmadı (en kritik) · M2 bekliyor (RFD+laptop) · M7/M8 kısmi ·
> 🔴 FC-OLAY temizliği yapılmadı — her şeyin ÖN ŞARTI.

---

## GENEL KURALLAR (her oturumda geçerli)

1. **Suya kadar tüm oturumlar PERVANESİZ.** İstisna yok.
2. Elle test öncesi `sudo systemctl stop girdap-karar` (çift node olmasın);
   oturum bitince `start`.
3. Her FAIL → `karar/docs/hata_defteri.md`'ye şablonla işlenir (log yolu dahil).
4. Ham loglar `~/girdap_logs/` altında oturum klasörüne.
5. **🔴 OPS-1: Görev yüklenen HER oturumun SON işi = FC görev hafızasını sil**
   (`/mavros/mission/clear` + `waypoints: []` teyidi). Neden: FC parametreleri
   (ekip kararı, 2026-07-14) pil+RC açıkken kendiliğinden ARM+AUTO'ya izin
   veriyor; hafızada görev kalırsa güç verilince KENDİ KOŞAR (12.07 olayı).

---

## OTURUM 1 — FC GÜVENLİK + TEŞHİS (kapalı alan OLUR, ~1,5 saat)
**Ne zaman: EN KISA SÜREDE. Gerekenler: tekne/FC + batarya + RC kumanda + Jetson. QGC şart değil.**

Sıra önemli — güç verir vermez hiçbir şeye dokunmadan önce teşhis alınır
(olayın kanıtı bozulmasın):

| # | Adım | PASS kriteri |
|---|---|---|
| 1.1 | Pervane SÖKÜLÜ teyidi (fotoğraf çek) | görsel |
| 1.2 | Güç ver, MAVROS bağlan, DOKUNMA | `connected: true` |
| 1.3 | **Teşhis dökümü** (verici AÇIK, eller cepte): `/mavros/rc/in` kanal PWM'leri (özellikle mod + gaz kanalı boşta nerede) + aktif mod + armed durumu | döküm kaydedildi → FC-OLAY (a)/(c)/(d) hangisi, KESİNLEŞİR |
| 1.4 | Parametre dökümü: `MODE_CH, MODE1-6, RCMAP_*, INITIAL_MODE, ARMING_REQUIRE, FS_THR_ENABLE, FS_ACTION, FS_TIMEOUT, FS_GCS_ENABLE, FS_CRASH_CHECK, BRD_SAFETY_DEFLT, SERVOx_FUNCTION (hepsi), SERIAL2_BAUD` | dosyaya kaydedildi |
| 1.5 | **Görev sil:** `/mavros/mission/clear` + geri okuyup 0 item teyidi | waypoint listesi BOŞ |
| 1.6 | **`BRD_SAFETY_DEFLT=1`** geri + FCU reboot + teyit | param=1 okundu |
| 1.7 | Güç döngüsü provası: kapat-aç, DOKUNMADAN 2 dk izle | motor DÖNMÜYOR, mod güvenli (HOLD/MANUAL), arm YOK |
| 1.8 | SERVO eşleşmesi: pervanesiz kısa gaz → hangi motor dönüyor (73=sol mu?) | sol komut=sol motor |
| 1.9 | (fırsat) `SERIAL2_BAUD=921600` dene → IMU Hz ölç | ≥25 Hz olursa kalıcı yap; olmazsa 57600 geri |
| 1.10 | (tamir edildiyse) Pixhawk USB-C → Jetson: `ttyACM0` çıkıyor mu | çıkarsa `fcu_url` USB'ye döner (~50 Hz IMU) |

**Çıktı:** FC-OLAY kök nedeni kesinleşir → defterde kapanışa yazılır;
1.7 geçmeden BAŞKA HİÇBİR OTURUM YAPILMAZ.

### ⚡ 30 DAKİKA PİL KISITI DÜZENİ (2026-07-14: pil sıkıntılı — sıcak süre ≤30 dk)

Jetson kendi adaptöründen beslenir (pil yemez). İlke: **pil takılı değilken
her şey hazırlanır; pil yalnız "sıcak" dakikalarda takılı kalır.**

**Pilsiz hazırlık (süre sınırsız):** pervane sökümü+fotoğraf · Jetson açık,
`systemctl stop girdap-karar` · 3 terminal hazır: (1) `ros2 launch mavros
apm.launch fcu_url:=serial:///dev/ttyUSB0:57600` yazılı, (2)
`bash ~/girdap-video/testler/fc_teshis.sh` yazılı, (3) temizlik komutları
yazılı (mission clear + BRD_SAFETY_DEFLT=1 — oturum günü verilecek) ·
RC verici açık, masada sabit.

**💡 Pil tasarrufu fırsatı:** Pixhawk USB'den de beslenir. USB-C soketi
gerçekten tamir edildiyse ÖNCE pilsiz, sadece USB ile dene (`ttyACM0`):
çalışırsa teşhis+temizlik (aşağıda T+2…T+8) TAMAMEN PİLSİZ yapılır; pil
yalnız güç-döngüsü provası + SERVO testine kalır (~10 dk).

**Sıcak koşu (pil takılı, hedef ≤25 dk):**

| Zaman | İş |
|---|---|
| T+0 | pil tak → FC boot → terminal-1 enter (mavros) → `connected: true` bekle |
| T+2 | terminal-2: `fc_teshis.sh` (salt okuma, ~3 dk — RC PWM + mod + tüm parametreler dosyaya) |
| T+5 | görev sil + boş teyidi |
| T+6 | `BRD_SAFETY_DEFLT=1` + FCU reboot + teyit |
| T+8 | **güç döngüsü provası:** pili çek-tak, DOKUNMADAN 2 dk izle → motor dönmüyor + mod güvenli teyidi |
| T+13 | SERVO eşleşmesi: arm (artık emniyet butonuyla) → her motora KISA düşük gaz → hangisi dönüyor not et → disarm |
| T+18 | **pil ÇEK — zorunlu kısım bitti** |
| kalan | yedek: sorun çıktıysa onarım; sorunsuzsa M6'nın 2 kritik adımı (kill→disarm + kasıtlı disarm≠failsafe — motorsuz, akım düşük) |

Bu oturumdan ÇIKARILANLAR (pil yemesin): `SERIAL2_BAUD` denemesi (sonraki
oturuma) · USB-C testi (pilsiz ayrı zamanda yapılabilir, yukarıdaki 💡).
Teşhis betiği: `testler/fc_teshis.sh` (çıktı `~/girdap_logs/fc_teshis/`).

---

## OTURUM 2 — MASA REGRESYONU + M6 KILL ZİNCİRİ (kapalı alan OLUR, ~2 saat)
**Oturum 1 ile AYNI GÜN yapılabilir. Gerekenler: aynı + pervanesiz teyidi.**

| # | Adım | PASS kriteri |
|---|---|---|
| 2.1 | M0 hızlı: suite `pytest prototype/tests/ -q` | 265 passed / 2 skip (taban) |
| 2.2 | M1 tekrar: tam stack + gerçek FCU | connected, heartbeat-KILL YOK |
| 2.3 | M4 tekrar: `/girdap/bridge/arm` → BOOT→ARM→BEKLEMEDE | FSM BEKLEMEDE |
| 2.4 | **F-M.1 fix doğrulaması (kapalı alan = tam fırsat):** GPS fix YOKKEN görev başlatmayı dene | görev REDDEDİLMELİ (OOM değil!) — 12.07'deki 92 GB patlamasının fix'i ilk kez gerçek FCU'da |
| 2.5 | **M6a:** `/girdap/mission/kill` → sıfır thrust + FCU DISARM | ikisi de gerçekleşir (F14.1) |
| 2.6 | **M6b — F-M.2 fix doğrulaması:** kasıtlı `/girdap/bridge/disarm` | "FAILSAFE" alarmı BASMAMALI |
| 2.7 | **M6c:** manuel moda al → auto_guided susuyor mu | set_mode savaşı YOK (F14.3) |
| 2.8 | **M6d:** TELEM2 kablosunu çek → heartbeat kaybı | KILL latch + motorlara sıfır komut |
| 2.9 | M7 ön: `~/girdap_logs/{telemetry,grafik,local_map}` dosyalar üretiliyor | CSV/PNG akıyor (içerik fix'sizken kısmen boş, normal) |

**Çıktı:** güvenlik zinciri UÇTAN UCA ilk kez gerçek FCU'da kanıtlı.
M6 geçmeden suya çıkılmaz — videonun 4. maddesi (güç kesme gösterimi) buna dayanıyor.

---

## OTURUM 3 — AÇIK ALAN (GPS'li; su GEREKMEZ, otopark/bahçe yeter, ~2 saat)
**Gerekenler: GPS gören yer + (ideali) x86 QGC laptop + RFD çifti. Pervanesiz!**

| # | Adım | PASS kriteri |
|---|---|---|
| 3.1 | GPS fix bekle (ARM'dan önce — F8.4 kuralı) | fix + `fusion/odom` akıyor (ilk kez!) |
| 3.2 | **M2:** QGC ↔ RFD868x ↔ Pixhawk kablosuz hat | QGC telemetri görüyor; MAVROS'la AYNI ANDA yaşıyor |
| 3.3 | **M3-QGC:** gerçek QGC Plan Upload (4 köşe) → FC → bizim yığın okudu mu | "FC görevi alındı: N item" logu (M3'ün MAVROS'la değil QGC'yle ilk provası) |
| 3.4 | **M5 TAM:** ARM → QGC'den mod→GUIDED | "YKİ mod komutu … görev başlatıldı" logu |
| 3.5 | Görev akışı (pervanesiz, tekne masada): thrust komutları akıyor, FSM PARKUR1→TAMAMLANDI | son noktada SIFIR thrust (F12.2/F13.1) |
| 3.6 | **M7 DOLU:** Dosya-2 CSV + grafik CSV + Dosya-3 PNG gerçek verili | `run_ekran2.py` PNG/MP4 üretti |
| 3.7 | **M8:** canlı MPPI Hz + `tegrastats` termal | ≥9 Hz sürekli, throttling yok |

**Çıktı:** video akışının %90'ı karada kanıtlı. Kalan tek bilinmeyen: suda gerçek hareket.

---

## OTURUM 4 — SUDA PROVA (T0-h; hedef 17-19.07 arası bir gün)
**Gerekenler: göl/sahil + PERVANELER TAKILIR (ilk kez) + can güvenliği + halat.**

1. **Bollard pull fırsatı:** tekne halatla bağlıyken ölçüm listesi §8 (el kantarı) — 20 dk.
2. **Kalibrasyon sürüşü:** 10-15 dk düz gaz adımları + dönüşler (log kaydı — Xu/Yv/Nr çıkarımı için; ölçüm listesi §10).
3. **Video senaryosu provası (çekimsiz):** 4 köşe dikdörtgen, tam akış:
   QGC Upload → ARM → GUIDED → 4 nokta → son noktada dur → manuel dönüş →
   güvenlik anahtarı → RC'de motor dönmüyor gösterimi.
   - İzlenecek: **istemsiz hareket VAR MI** (4 kök de kapandı ama gerçek suda İLK KEZ — md 3.3.1.1 doğrudan eleme kriteri).
4. Kayıt kontrolü: Ekran-2 CSV'leri dolu, `run_ekran2.py --mp4` çalışıyor.
5. Sorun çıkarsa → hata defteri → düzeltme → ertesi gün tekrar (bu yüzden çekimden ÖNCE ayrı prova günü).

## OTURUM 5 — ÇEKİM GÜNÜ (hedef 19-20.07; 21'i YEDEK gün bırak)

`karar/docs/video_gunu_runbook.md` harfiyen + `kontrol-listesi.md`.
Kısa hatırlatmalar: WiFi `rfkill block` çekimden önce · RC bandı 2,4 GHz
OLMAMALI · Ekran-1 QGC kaydı + Ekran-2 `run_ekran2.py --mp4` + Ekran-3 dış
kamera · ≥720p, 2-5 dk, TEK KESİNTİSİZ çekim · YouTube liste dışı → KYS'ye
link (yüklemeyi son güne BIRAKMA).

---

## DONANIM KURULUMU — oturum oturum NE TAKILIR, NEREYE

> Referans şema: girdap-ida-algi `docs/BURADAN_BASLA.md` §1. Buradaki liste
> o şemanın oturumlara uyarlanmış hâli. **Videoda algı YOK → OAK kamera ve
> Livox LiDAR HİÇBİR video oturumunda TAKILMAZ** (sadelik = daha az arıza).

### Oturum 1 + 2 (masa) — bağlantı listesi

| Ne | Nereye | Not |
|---|---|---|
| Pervaneler | ❌ SÖKÜLÜ | fotoğrafla belgele; motorlar/ESC'ler BAĞLI kalır (SERVO testinde dönecekler) |
| Batarya → PM07 → Pixhawk | güç modülü üzerinden | **EN SON bağlanır** — önce her şey kablolanır, herkes hazırken güç |
| RC alıcısı | Pixhawk RC IN portu | verici AÇIK ve masada sabit; teşhis sırasında KİMSE dokunmaz |
| GPS (H-RTK F9P) | Pixhawk GPS1 | takılı olsun — kapalı alanda fix gelmemesi F-M.1 testi için zaten isteniyor |
| Güvenlik anahtarı/buton | Pixhawk'a bağlıysa yerinde | 1.6'da BRD_SAFETY_DEFLT=1 olunca ARM için buna basmak gerekecek |
| TELEM2 → FTDI çevirici → Jetson USB | `ttyUSB0` | ⚠️ ÇAPRAZLAMA: Pixhawk TX→çevirici RX, RX→TX, GND-GND, VCC BOŞ (12.07'de hattı bu düzeltti) |
| Jetson güç | kendi DC adaptörü, prize | masada şebeke; teknedeki regülatör aranmaz |
| Jetson monitör + klavye | HDMI/DP + USB | WiFi kapalı, SSH yok → masada en pratiği ekran başında çalışmak. Alternatif: PC ile ethernet kablosu |
| OAK kamera / Livox | ❌ TAKILMAZ | videoda algı yok |
| (1.10 için) USB-C kablo | Pixhawk USB-C → Jetson | sadece soket testi anında; OAK'ın sağlam SS kablosu kullanılabilir |

Masa düzeni: tekne/iskelet masada sabit, motorlar boşta dönebilsin (etrafta
kablo/parmak yok), batarya şarjlı + yedek batarya.

### Oturum 3 (açık alan) — masaya EK olarak

| Ne | Nereye | Not |
|---|---|---|
| RFD868x #1 | Pixhawk **TELEM1** | TELEM2 bizde (MAVROS) — iki port karışmasın |
| RFD868x #2 | x86 laptop USB | QGC bağlantı hızı RFD varsayılanı (57600) |
| x86 laptop + QGC | — | ARM64 QGC yok, Jetson'a kurulamaz |
| GPS anteni | gökyüzünü görecek | bina/araç gölgesinden uzak; fix beklemeden ARM yok (F8.4) |
| Jetson güç | teknenin güç sistemi (regülatör) ya da uzatma+adaptör | alan seçimini prize göre yap ya da mekanik ekipten tekne içi besleme iste |
| Pervaneler | ❌ HÂLÂ SÖKÜLÜ | görev akışı thrust komutlarıyla izlenir, motor boşta döner |

### Oturum 4 + 5 (su) — tam kurulum + yanına alınacaklar çantası

**Teknede takılı:** Pixhawk+PM07+batarya · RC alıcı · GPS direkte · RFD #1
(TELEM1) · Jetson tekne içinde, tekne beslemesinden, TELEM2 bağlı ·
**PERVANELER TAKILI (ilk kez)** · güvenlik anahtarı erişilebilir (md 3.3.1/4
kameraya gösterilecek!) · kapaklar kapalı/sızdırmaz (md 3.3.1/5 gösterimi).
Jetson'a monitör YOK — boot'ta `girdap-karar.service` kendiliğinden kalkar;
görev QGC'den yürür.

**Kıyıda:** x86 laptop (QGC + ekran kaydı programı test edilmiş, disk boş) ·
RFD #2 · RC verici (bandı 2,4 GHz OLMAYAN) · dış çekim kamerası + tripod +
dolu pil/kart (≥720p) · yedek batarya + şarj cihazı.

**Çantaya:** şerit metre (ölçüm listesi §A suya girmişken!) · el kantarı
20-50 kg (bollard pull §8) · sağlam halat (bollard + emniyet) · USB bellek
(log yedeği) · TELEM kablo yedeği + FTDI yedeği varsa · bant/kablo bağı ·
havlu/poşet (elektronik ıslanmasın) · güneş gölgeliği (laptop ekranı).

**Çekim öncesi son kontroller (Oturum 5):** `rfkill block wifi bluetooth` ·
`getUsbSpeed` KONTROLÜ GEREKMİYOR (OAK yok) · Jetson saat/tarih doğru mu
(log damgaları) · disk boş alanı ≥5 GB · `kontrol-listesi.md` maddeleri.

---

## BLOKER / TEDARİK LİSTESİ (oturumları bekletenler)

| İhtiyaç | Hangi oturum | Durum |
|---|---|---|
| x86 laptop + QGC kurulu | 3, 4, 5 | ✅ arkadaşın laptopu, QGC çalışıyor (2026-07-14) — saha öncesi kontrol: ekran kaydı programı + disk alanı + şarj/powerbank + USB seri izni |
| RFD868x çifti kurulumu | 3, 4, 5 | ⛔ hiç kurulmadı — KALAN TEK DONANIM BLOKERI |
| FC ekibi/FC başında biri | 1 | mesaj atıldı, gün bekleniyor |
| GPS gören açık alan | 3 | kolay |
| Göl/su erişimi + pervaneler | 4, 5 | planlanacak |
| Ölçüm listesi C bölümü (SERVO/failsafe) | 1'de bizzat alınıyor | — |

## ZAMAN ÇİZELGESİ ÖNERİSİ (bugün 14.07 Salı)

- **15.07:** Oturum 1 + 2 (aynı gün, masa)
- **16.07:** Oturum 3 (açık alan) — laptop/RFD hazırsa; değilse laptop tedariki
- **17-18.07:** Oturum 4 suda prova (+1 gün düzeltme payı)
- **19-20.07:** Oturum 5 çekim + montaj + YouTube/KYS yükleme
- **21.07:** YEDEK gün (hava/arıza) — son teslim 17:00

> Kural: bir oturum FAIL verirse sonrakine GEÇME; düzelt, tekrarla.
> Takvim 2 gün kayarsa bile yedek gün korunur.

# Su Testi 1 ve 2 — Prosedür (şartname + ArduPilot resmi kaynak)

> §0.25'in "5 TESTLİK SU PLANI"nda TEST 1/2 için taslak vardı; bu dosya onu
> şartname maddeleri + ArduPilot resmi dokümanlarıyla (2026-08-10 araştırması)
> çapraz doğrulanmış, uygulanabilir bir prosedüre çeviriyor. Bench testleri
> (`bench_mavlink_runbook.md`, `ardurover_bench.parm.md`) TAMAMLANMADAN bu
> dosyaya geçilmez.

---

## TEST 1 — Manuel düz git / geri gel

**Amaç:** Pervane/motor/yön doğrulaması + M3'ün bedava ölçümü (ileri/geri
itki oranı asimetrisi, `catamaran.py:130` şu an simetrik varsayıyor, PWM
yetkisi %5,3 asimetrik ölçülmüştü — §0.14c).

### Neden MANUAL ile başlanır (ArduPilot resmi gerekçe)
[ArduPilot Rover — Manual mode](https://ardupilot.org/rover/docs/manual-mode.html):
*"This mode does not require a position estimate (GPS is not required)"* —
GPS/DGPS henüz doğrulanmamış olsa bile yürütülebilir, en düşük riskli ilk
adım budur.

### Ön koşullar (bench'ten devralınan, tekrar doğrula)
- [ ] `INITIAL_MODE=0` (açılışta MANUAL) — §0.24c⑤
- [ ] `ARMING_REQUIRE=1` — yazılım KILL yolu canlı (§0.14b, GERİ ALINMAZ)
- [ ] `ARMING_CHECK=1` × `ARMING_RUDDER` — kazara ARM reçetesi kapatılmış (§6.5 OPS-1)
- [ ] `FS_THR_ENABLE=0` doğru mu (RC alıcı VARSA bu satır gözden geçirilir —
      alıcı sökükken pre-arm reddi olmasın diye 0'dı, alıcı takılınca anlamı
      değişir)
- [ ] Kill switch fiziksel test edilmiş (`bench_mavlink_runbook.md` ADIM 6) — **BUGÜN TEKRAR DENE**, su ayrı bir elektriksel ortam
- [ ] Uzaktan güç kesme (röle) kurulu VEYA en azından yazılım disarm doğrulanmış (§0.24c②'nin md 4.2 gerekliliği)
- [ ] Can yeleği + tekneyi tutacak ip/çekme halatı (şartname md "çeki demiri" gereksinimi, 5.2)

### Adımlar
1. Sığ/kontrollü suda, pervaneler suya değecek ama tekne sabit tutulacak
   şekilde ARM et. `ros2 topic echo /mavros/state --once` (varsa) veya
   Mission Planner'dan `armed=true` doğrula.
2. Tekneyi serbest bırak, **düşük sabit gaz** ile ~10 m düz ileri sür.
   Durdur (gaz nötr), 3-5 sn bekle.
3. **Aynı gaz seviyesinde** ~10 m geri sür. Durdur.
4. Kill switch'i **hareket hâlindeyken** bir kez test et (motor anında durmalı).
5. DGPS/telemetri logunu (Dosya-2 formatı zaten üretiliyor) kaydet — mesafe/
   süre/hız hesaplanacak.

### Ölçülecek (M3'ü kapatan bedava veri)
| | ileri | geri |
|---|---|---|
| PWM (SERVO1/3) | | |
| ölçülen hız (m/s, DGPS'ten) | | |
| X metrede geçen süre | | |

→ oran = geri_hız / ileri_hız (aynı PWM'de) → `catamaran.py`'nin
`max_thrust`/`Xu` geri yönü için ayrı katsayı gerekip gerekmediğine karar verir.

### Acil çıkış
ArduPilot dokümanının kendi tavsiyesi: *"if you notice any odd behaviour, use
the transmitter's mode switch at any time to change back to MANUAL or HOLD"*
— MANUAL zaten bu testin modu; anormal davranışta **kill switch** birincil,
RC nötr ikincil.

---

## TEST 2 — AUTO Parkur-1 (4 waypoint)

**Amaç:** FC'nin kendi native navigasyonunu doğrulamak. 🔑 **Bu test
Jetson'ı hiç kullanmıyor** — "TEST 2 geçti" = FC sağlıklı demektir, otonomi
zincirinin (karar yığını) çalıştığı anlamına GELMEZ (§0.25c'nin uyarısı,
ArduPilot dokümanıyla da tutarlı: AUTO modda navigasyonu tamamen ArduPilot
yapar).

### ArduPilot resmi prosedürü (research doğrulaması)
[ArduPilot Rover — Auto mode](https://ardupilot.org/rover/docs/auto-mode.html) +
[Arming your Rover](https://github.com/ArduPilot/ardupilot_wiki/blob/master/rover/source/docs/arming-your-rover.rst):

> *"Arm the vehicle in MANUAL or HOLD mode, then change the mode to AUTO"*
> **AUTO, GUIDED, LOITER, RTL, SMARTRTL, FOLLOW, DOCK modları RC anahtarından
> ARM edilemez.**

🔴 **Bizim durumumuzla kesişim:** `MODE_CH=8` ama `MODE1..6` hepsi 0
(§0.25c) → RC'den zaten AUTO'ya geçilemiyordu, bu artık ArduPilot'un kendi
kısıtıyla da örtüşüyor — **tek yol Mission Planner'dan mod değişimi.**
Sıra: **①** MANUAL/HOLD'da ARM (Test 1'deki gibi) **②** Mission Planner'dan
mod → AUTO.

### 🔴 Şartname çakışması — KABLOLU mod değişimi/görev başlatma YASAK
`eyup_memory/sartname-ida-2026.md` (md 5.5.3.1, birinci kaynaktan okunmuş):
*"Görev/Hareket **Başlat komutu KABLOLU verilmeyecek**"* — yani Mission
Planner'ın FC'ye AUTO komutunu **USB/SSH üzerinden DEĞİL, telemetri
radyosundan (868 MHz)** göndermesi gerekir. Bench'te USB ile denemek
alışkanlık yaratır, sahada yasak olan yolu pratik etmiş oluruz.
👉 **Test 2'yi telemetri radyosu (MicoAir LR868) üzerinden Mission Planner
ile koş — USB debug bağlantısını mod değişiminde KULLANMA.**

### Ön koşullar
- [ ] Test 1 tamamlanmış (motor/yön doğrulanmış)
- [ ] `WP_RADIUS` — mevcut **2,5 m** (10.08 dökümü); RTK varsa 0,5'e indir (§0.25c)
- [ ] `MIS_DONE_BEHAVE` mevcut **0** (Hold — 4. noktada durur) — kontrol et,
      şartnamenin *"4. noktaya ulaşınca otonom görev tamamlanır, sonrası
      manuel"* (md 3.3.1.3) beklentisiyle **uyumlu**, değiştirme.
- [ ] Görev dosyası (4 nokta, dd.ddddddd) Mission Planner'a **güç
      verildikten SONRA** yüklenmiş (md 5.5.2.2 — yarışma günü kuralı, bench'te de aynı sırayla pratik et)
- [ ] Telemetri hattı (868 MHz) ayakta ve o an ölçülüyor

### Adımlar
1. MANUAL/HOLD'da ARM (Test 1 adım 1 aynısı).
2. Mission Planner → Flight Plan'dan 4 waypoint'i yükle (`WP_LOAD` veya elle).
3. Mission Planner'dan mod → **AUTO** (telemetri üzerinden — yukarıdaki kural).
4. Aracı izle: log58 emsali **4/4 WP, 99 s, "Mission Complete"** (§6.0) —
   bu turda süre/nokta sayısı bunula kıyaslanacak.
5. Anormallikte Mission Planner'dan **MANUAL/HOLD**'a geri al (ArduPilot'un
   kendi önerdiği acil çıkış — RC anahtarı da MANUAL'e alabilir, AUTO'dan
   ÇIKARKEN RC kısıtı yok, yalnız AUTO'ya GİRERKEN var).

### Ölçülecek
- Tamamlanan waypoint sayısı / 4
- Toplam süre (log58 = 99 s referans)
- `WP_RADIUS` içinde mi geçti yoksa dışından mı döndü
- Telemetri kesintisi oldu mu (868 MHz bant testi de örtük olarak burada yapılmış olur — §9.4 frekans maddesi)

---

## İki testin ortak notu — video günü provası

Şartname md 3.3.1'in otonomi videosu da bu iki testin bir kombinasyonu
(4 nokta AUTO görevi + manuel dönüş + kill switch gösterimi). Test 1+2 GEÇERSE
video günü senaryosunun donanım/FC ayağı da fiilen provalanmış olur — ayrı
bir "video provası" gerekmez, kayıt (Ekran 1/2/3) eklenmesi yeter.

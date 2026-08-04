# Ölçüm Formu — GİRDAP İDA

> **Durum:** Bu dosya 2026-08-04'te oluşturuldu. Daha önce **7 ayrı dokümandan
> referans veriliyordu ama hiç yazılmamıştı** — bu yüzden ölçümler hiç
> toplanmadı ve `hata_defteri.md` F5.1 maddesi "BLOKE, mekanik `h` ölçüsü
> bekliyor" durumunda kaldı. Referans veren yerler: `fc_parametre_onerileri.md`
> (×2), `ardurover_bench.parm.md`, `hata_defteri.md`, `dogrulama_matrisi.md`
> (×2), `donanim_gunlugu_2026-07-12.md`, `pc_gunlugu_2026-07-12.md`.
>
> **Kime:** mekanik + donanım + FC ekipleri.
> **Nasıl:** `____` alanlarını doldurup commit'leyin. Tahmin YAZMAYIN — ölçün.
> Ölçemediğiniz bir satırı boş bırakın, "yaklaşık" yazmayın (yaklaşık değer
> sessizce doğru sanılır; boşluk en azından görünür kalır).

---

## §0 — `base_link` NEREDE? (önce bu karara varılmalı)

Diğer tüm ölçümler bu noktaya göre alınacak. **Bir kez seçilir, bir daha
değişmez**, ve araç üzerinde fiziksel olarak işaretlenir (bant/kalem).

**Eksen kuralı (ROS REP-103, değiştirilemez):**
- **+x = PRUVA** (ileri, teknenin burnu)
- **+y = İSKELE** (sol taraf)
- **+z = YUKARI**
- Açı birimi: **derece**, saat yönünün TERSİ (+) — yani +yaw = sola dönüş

**Önerilen `base_link` konumu:** iki gövdenin tam ortası, güverte düzlemi,
boy ekseninde ağırlık merkezi hizası. Farklı bir yer seçilirse buraya yazın:

| | Değer |
|---|---|
| `base_link` seçilen konum (tarif) | `________________________________` |
| Araç üzerinde işaretlendi mi? | ☐ evet |

---

## §1 — Tekne gövde ölçüleri

`gate_follower` bu iki sayıyı kullanıyor (kapı geçilebilirlik testi — eşik
ayarı değil, fizik). Şu an kodda `hull_width_m=0.78`, `hull_length_m=1.04`
yazıyor (kaynak: GIRDAP_DURUM §1). **Teyit edin:**

| Ölçü | Kodda | Ölçülen | Not |
|---|---|---|---|
| Gövde genişliği (uçtan uca, en geniş yer) | 0.78 m | `____` m | Kapıdan sığma hesabı |
| Gövde boyu | 1.04 m | `____` m | Burun hattı / min_forward |
| Şamandıra/fender vb. ile toplam genişlik | — | `____` m | Çarpma payı için |

---

## §2 — 🔴 LiDAR (Livox Mid-360) — EN KRİTİK ÖLÇÜM

> **Neden kritik:** `perception_lidar_node` şu an **hiçbir TF dönüşümü
> uygulamıyor** — çıktının `frame_id`'sini `base_link` diye etiketliyor, o
> kadar. Oysa `z_min=0.1` filtresi "base_link'e göre su üstü kesim" diye
> tanımlı. LiDAR `base_link`'in ÜSTÜNDEYSE dubalar LiDAR çerçevesinde
> **negatif z**'de kalır ve `z_min=0.1` filtresi **hepsini eler** → LiDAR
> hiçbir engel görmez. `h` ölçülmeden bu doğrulanamaz.

| Ölçü | Değer | Nasıl ölçülür |
|---|---|---|
| `x` — `base_link`'ten ileri (+) / geri (−) | `____` m | Yatay mesafe, cetvel |
| `y` — iskeleye (+) / sancağa (−) | `____` m | Merkezdeyse 0 |
| **`z` — `base_link`'ten yukarı (h)** | `____` m | 🔴 F5.1'i çözen sayı |
| `yaw` — pruvaya göre dönüklük | `____` ° | 0 = konnektör kıça bakıyor varsayımı DEĞİL, ölçün |
| `pitch` / `roll` — eğik monte edildi mi? | `____` / `____` ° | Düz monte ise 0/0 |
| **Su hattından yükseklik** (yüklü tekne, sakin su) | `____` m | Menzil/kör nokta hesabı |
| Gövdenin LiDAR görüş alanına giren kısmı var mı? | ☐ var ☐ yok | Varsa `min_range` filtresi gerekir (F5.1 ile birlikte) |

---

## §3 — Kamera (OAK-D Lite)

| Ölçü | Değer | Not |
|---|---|---|
| `x` — ileri (+) | `____` m | |
| `y` — iskele (+) | `____` m | |
| `z` — yukarı | `____` m | |
| `yaw` — pruvaya göre | `____` ° | |
| `pitch` — ufka göre (aşağı bakıyorsa −) | `____` ° | Duba mesafesi tahminini doğrudan etkiler |
| Lens ekseni su hattından yükseklik | `____` m | |

---

## §4 — ESC (bkz. `ardurover_bench.parm.md` → ESC KALİBRASYONU)

Tekne **2 motorlu** (sol/sağ). Her iki ESC de aynı model olmalı.

| | Değer |
|---|---|
| Marka / model | `________________` |
| **Tek yönlü mü, çift yönlü (reversible) mi?** | ☐ tek yönlü ☐ çift yönlü |
| PWM aralığı (üretici verisi) | min `____` / nötr `____` / max `____` µs |
| Kalibrasyon yapıldı mı (ikisi de, aynı prosedürle)? | ☐ evet, tarih: `______` |
| **Thruster tepe akımı (tek motor)** | `____` A |
| **2 motor toplam tepe akım** | `____` A |

> Son iki satır uzaktan güç kesme kontaktörünün boyutlandırılması için gerekli
> (`fc_parametre_onerileri.md` §4.5).

---

## §5 — FC parametre okuması

`fc_parametre_onerileri.md` tablolarındaki **"Mevcut" sütunları boş.** Tek tek
yazmak yerine tüm listeyi dosyaya dökün:

```
Mission Planner → Config → Full Parameter List → Save to file
→ docs/fc_mevcut_parametreler_<YYYY-MM-DD>.param  (repoya commit)
```

| | Değer |
|---|---|
| Dosya alındı mı? | ☐ evet, tarih: `______` |
| ArduRover firmware sürümü | `________` |
| RC kalibrasyonu baştan yapıldı mı? | ☐ evet (CH2/CH3 uçlarda dinliyordu — `RCMAP_*` teyidi şart) |

---

## §6 — Güç sistemi

| | Değer | Not |
|---|---|---|
| Batarya konfigürasyonu | `________` | 4S7P bekleniyor, teyit |
| Nominal / dolu / boş voltaj | `____` / `____` / `____` V | `BATT_LOW_VOLT` için |
| Ana sigorta değeri | `____` A | |
| Motor kolu ile FC/Jetson kolu **ayrı mı?** | ☐ evet ☐ hayır | 🔴 Kontaktör için ŞART (`fc_parametre_onerileri.md` §4.5) |

---

## Ölçüm sonrası — bu değerler nereye girilecek

| Ölçüm | Gideceği yer |
|---|---|
| §2 LiDAR x/y/z/yaw | `hardware.launch.py` → `_static_tf("base_link", "livox_frame")` argümanları |
| §2 `h` (z) | F5.1 — `lidar_height_m`; **perception ekibine bildir** (`perception_lidar_node` bu bölgede çalışıyor) |
| §3 Kamera x/y/z/yaw/pitch | `_static_tf("base_link", "oak_frame")` |
| §1 gövde | `gate_follower` `hull_width_m` / `hull_length_m` teyidi |
| §4 ESC PWM | `MOT_PWM_MIN` / `MOT_PWM_MAX` tutarlılığı |
| §4 akım | Kontaktör seçimi |
| §5 | `fc_parametre_onerileri.md` "Mevcut" sütunları |
| §6 | `BATT_*` failsafe parametreleri |

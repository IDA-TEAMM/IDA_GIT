# PARKUR-3 değişiklikleri — ÖNCE BUNU OKU (13.08.2026)

> Bu dosya, algı ekibinin (Eyüp) **Parkur-3 kapsamında** karar tarafına giren
> 7 commit'inin gerekçesidir. Amaç: bir maddeyi geri almadan önce **neden
> orada olduğunu** bilmek. Sorusu olan doğrudan bize yazsın.

## Ortak desen — P1/P2'ye dokunulmadı

Hepsi **renge koşullu**: `kamikaze_target_color` **boşken** davranış **bit
birebir** eskisi gibidir. Renk yoksa / İHA başarısızsa kamikaze **hiç açılmaz**,
araç temiz durur.

**Doğrulama: 682 test geçiyor, 0 regresyon** (taban 632'ydi; artan testler
yeni davranışın bekçileri). Her davranış ayrıca **mutasyonla** sınandı:
koruma kaldırılınca ilgili testler kırmızıya dönüyor.

---

## 1. `49a61f7` — SİYAH hedef rengi eklendi

**Ne:** `kamikaze_hedef.RENK_SINIFLARI`'na `siyah`/`black` → `CLASS_SIYAH = 6`;
`camera_buoys`'a aynı kimlik (**dedektörü yok**, yalnız sözleşme kimliği).

**Neden:** şartname **s.18** hedef renklerini **RAL 9005 (siyah)** · RAL 3026
(kırmızı) · RAL 6037 (yeşil) diyor. Sözlükte siyah **yoktu** ⇒ hakem "siyah"
derse `renk_to_class` hata atıyor, `ros2 param set` **reddediliyor**, hedef
atanmamış kalıyordu = **3 renkten 1'inde Parkur-3 tamamen sıfır**.

Kök neden (suçlama değil, boşluk): sınıf 3/4/5 şartnameden değil, **16.07
saha testinde görülen** renklerden türetilmişti (commit `166352b`: *"parkurda
bu renklerin de bulunduğu bulundu"*). 12.08'de P3 mekanizması o hazır listeye
eşlenince siyah dışarıda kaldı.

**Yanında:** `"SİYAH"`/`"YEŞİL"` gibi **büyük harf** yazımlar da reddediliyordu —
Python `.lower()` Türkçe `İ`yi `i`+U+0307 yapıyor. `_anahtarla()` bunu düzeltir.
⚠️ Bu hata **yeşili de** vuruyordu, yani yalnız yeni renkle ilgili değildi.

---

## 2-3. `f920ef5`, `c10b2ba` — sayısal renk kodu + FC parametresi köprüsü

**Ne:** `prototype/mission/renk_kodu.py` (**0**=karar yok · **1**=kırmızı ·
**2**=yeşil · **3**=siyah) ve `renk_kodu_koprusu.py`:
`Mission Planner → SCR_USER1 → MAVROS → hedef node parametresi`.

**Neden:** şartname **s.21**: *"Görev yükleme aşamasında … YKİ'de **sadece YKİ
arayüzü** açık olacak."* Terminalden `ros2 param set` bu maddeyle sürtüşüyor.
Mission Planner'ın parametre ekranı bir YKİ arayüzüdür — **ama parametreler
float**, içine "siyah" yazılamaz. Bu yüzden sayısal kod.

**Kapı tekrarlanmadı:** köprü değeri kendi uygulamıyor, **hedef node'un
parametresini set ediyor**; md 5.5.3.1 zamanlama kapısı orada zaten var ⇒
kural **tek yerde** kalıyor.

`c10b2ba`: **hareket başlayınca yoklama tamamen duruyor.** *"Zaten
reddediliyor"* savunması *"hiç denemiyor"*dan zayıftır; ayrıca journal'da koşu
boyunca parametre okuma trafiği görünmesin.

**🔓 SÖKÜLEBİLİR:** köprü **hiçbir launch'tan otomatik başlamıyor**. Gerek
kalmazsa tek dosya silinir, gerisi çalışmaya devam eder.

---

## 4-5. `a85b1f7`, `2a8ce3d` — köprü sağlamlaştırma

- **Geçici hatada yeniden deneme:** köprü okunan kodu *uygulamadan önce*
  önbelleğe yazıyordu; hedef node'un servisi o an hazır değilse (**açılışta çok
  muhtemel**) renk **bir daha hiç uygulanmıyordu** — logsuz, belirtisiz.
  Artık bir kod ancak **başarıyla uygulandıktan sonra** işlenmiş sayılıyor.
- **Yanıltıcı teşhis:** siyahta *"karede hiç görülmedi — HSV eşiği / ışık /
  renk adı kontrol edilmeli"* deniyordu. Gerçek sebep eşik değil, **bu node'da
  siyah dedektörünün olmaması**. `DEDEKTORU_OLAN_SINIFLAR` ile mesaj ayrıldı.
- **Uçuşta-istek koruması:** servis var ama cevap vermiyorsa (FC susmuş)
  istekler birikiyordu.

---

## 6. `3470d7f` — PARKUR-3 GİRİŞ + ÇIKIŞ 🔴 en kritik

**Ne:** `mission_complete` kuralı ikiye ayrıldı:
- `p3_bekleniyor` **VE** PARKUR1/2 → **PARKUR3**  *(yeni)*
- aksi hâlde → **TAMAMLANDI**  *(eski davranış, aynen)*

Çıkış: `şok` **VEYA** `ilerleme-yok` **VEYA** `süre aşımı`.

**Neden (giriş):** PARKUR3'e giden tek yol `/perception/gate_passed` idi ve o
**bilerek kapalı** (ilk kapıda Parkur-2'yi kırıyordu). Yani **145 puan
ulaşılamaz** durumdaydı. Tetik olarak *"son görev noktasına varmak"* seçildi:
PARKUR1→PARKUR2 geçişiyle **simetrik** ve Şekil 3'ün P3'e ayrı görev noktası
verip vermemesinden **etkilenmiyor** (şartname o konuda sessiz).

**Neden (çıkış):** şok eşiği **3,0 g**, ama bu reponun **kendi ölçümü**
(`pipeline.py:100-104`) kamikaze temas hızını **0,134-0,154 m/s** veriyor.
O hızdan duruş **0,03-0,14 g** üretir; IMU durağanken zaten **1,0 g** okur ⇒
tepe 1,03-1,14 g. **Şok kanalı P3'ü hiç bitiremez.**

> 🔴🔴 **`mission_complete` LATCH'lidir** (bir kez True olunca sıfırlanmaz).
> Eski kural PARKUR3'ü de kapsıyordu; kapsamaya devam etseydi P3'e giren araç
> **bir sonraki tick'te** TAMAMLANDI'ya düşerdi ⇒ **kamikaze tek tick yaşar,
> hedefe hiç gidilmez.** Bu istisna kaldırılmamalı.

---

## 7. `9397948`, `8c528e6` — kablolama + iki kritik düzeltme

- `P3CikisIzleyici` (ROS'suz, testli): *ilerleme-yok* + *süre aşımı*.
  Durgunluk sayacı **aracın fiilen durduğu andan** başlar; **tek hızlı örnek
  sayacı sıfırlar** — dalgada hız salınır, gerçek temasta salınmaz.
- `fsm_node` hızı `/girdap/fusion/odom` **twist**'inden alıyor.
- `KamikazeHedefKapisi` rengi **`/girdap/mission/hedef_rengi`**'ne ilan ediyor;
  `fsm_node` oradan öğreniyor. **Rengin sahibi ilan eder** — parametreyi ikinci
  bir node'dan okumak iki kaynak, yani sessiz sürüklenme demekti.

> 🔴 **QoS `TRANSIENT_LOCAL` şart.** VOLATILE ile ilan `__init__`'te bir kez
> yapıldığı için `fsm_node` sonra açılırsa **kaçırır** ⇒ P3 hiç açılmaz.
>
> 🔴 **Bayat odom koruması şart.** Odom susunca hız son değerinde donar; araç
> o sırada duruyorsa **sahte temas** üretir ve görev temassız biter. Artık
> bayat odomda *"durdu"* sonucu çıkarılmıyor (aynı kural `menzil_tutarli`da da var).

---

## ⚠️ Değiştirecekseniz dikkat

1. **`p3_bekleniyor` kapısını kaldırmayın** — P1/P2'nin eski davranışını o koruyor.
2. **PARKUR3'ün `mission_complete` istisnasını kaldırmayın** (tek tick tuzağı).
3. **`hedef_rengi` QoS'unu VOLATILE'a çevirmeyin.**
4. **Şoku tek çıkış olarak bırakmayın** — o eşik hiç ateşlenmiyor.
5. **Renk kodu tablosu iki repoda** (`renk_kodu.py` ↔ `girdap-iha-plaka/plaka/cikis.py`)
   — birini değiştiren ötekini de değiştirmeli; ikisinde de sabit değerli test var.
6. **Eşikler (0,08 m/s · 3 sn · 120 sn) suda ölçülecek** — şu an makul tahmin.

📌 Bilinen tutarlılık borcu: `qos_profiles.py`'de latch profili yardımcısı var;
satır içi `QoSProfile` yerine oraya taşınmalı.

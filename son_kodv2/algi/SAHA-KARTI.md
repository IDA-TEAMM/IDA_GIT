# 🚤 SAHA KARTI — göl günü, tek sayfa

> Ayrıntılı gerekçeler: `VERISETI-2-PLANI.md` · Jetson kurulumu: `VERISETI-JETSON-KARTI.md`
> **Yarışma: 20-23 Ağustos, Gölcük Tersanesi.** Biz Bolu'dayız.

---

## ⏰ SAAT — pazarlıksız

| pencere | ne çekilir | neden |
|---|---|---|
| **11:30 – 14:30** 🥇 | Ana çekim | Yarışmanın **11:00-15:00** güneş açısı (55-61°) **verimizde %0**. Bolu'da o açı tam bu saatlerde. Tepe 13:00 (63,9°) |
| **08:30 – 09:30** 🥈 | İkinci çekim | Yarışmanın 09:00'ı = burada 08:45. Alçak güneş, en sert su parlaması |
| **17:00 – 18:00** 🥉 | Varsa | Yarışmanın 17:00'ı = burada 17:00 |
| ~~15:00-17:00~~ | ❌ **ÇEKME** | Elimizdeki 12.754 karenin **tamamı** bu bantta. Bir kare daha katkı yapmaz |

🔴 **Öğle atlanırsa oturum boşa gider.** Tek şey seçilecekse: **11:30-14:30.**

---

## 🎒 YANINDA GÖTÜR

- [ ] OAK kamera + **yedek USB kablo** · Jetson · powerbank/akü
- [ ] 4 duba (turuncu + sarı) + ip/ballast
- [ ] 🔴 **KIRMIZI** büyük cisim — bidon / varil / can yeleği *(en kritik)*
- [ ] 🔴 **BEYAZ** silindir — usturmaça / PVC boru / beyaz bidon
- [ ] **YEŞİL** ve **SİYAH** cisim (bidon olur)
- [ ] Turuncu/sarı ama duba OLMAYAN şey (can yeleği, boya kovası)
- [ ] Mezura *(kamera ofseti için)* · **gölgelik/şemsiye** *(kamera 69 °C'ye çıkıyor)*

💡 Hedef dubasının aslını bulmak şart değil — **renkli plastik bidon** boyut ve
siluet olarak yeterince benziyor.

---

## 📋 SUYA GİRMEDEN (kıyıda, 5 dk)

```bash
date                                   # 🔴 GÖZLE BAK — saat yanlışsa veri bölmesi bozulur
lsusb | grep 03e7                      # kamera görünüyor mu
sudo systemctl start girdap-veriseti
journalctl -fu girdap-veriseti         # "[+] N kare" AKMALI
ls ~/girdap_veriseti/images | wc -l    # sayı ARTMALI
```
Kapatırken: `sudo systemctl stop girdap-veriseti` → **fişi doğrudan çekme.**

---

## 🎬 ÇEKİM SIRASI

### 1️⃣ NEGATİF — ÖNCE BU (~20 dk) 🔴🔴 en kritik
Cisimleri suya bırak, **etrafında normal dolaş**. Duba **yok**, sadece bu cisimler.

| cisim | süre |
|---|---|
| **Kırmızı** bidon/can yeleği | **7 dk** |
| **Beyaz** silindir | **5 dk** |
| Yeşil + siyah | 4 dk |
| Turuncu/sarı ama duba olmayan | 3 dk |

> **Neden birinci sırada:** model şu an kırmızı cismi **%97,4 oranında "turuncu
> kenar dubası"** sanıyor (güven 0,84). Yarışmada 3 hedef dubasından biri kırmızı
> ve parkurun çevresi **beyaz** dubayla sarılı ⇒ sahte kapı ⇒ P1/P2 gider.
> **Bunlar etiketlenmeyecek** — model "duba değil" demeyi ancak görerek öğrenir.

### 2️⃣ UZAK MESAFE (~35 dk) 🥇
```
--interval 1.0   --zorunlu-aralik 5      ← bu manevrada AYARI DEĞİŞTİR
```
- **40-50 m'den yavaş (≤1 m/s) düz yaklaş — en az 10 tekrar**
- **30 m / 25 m / 20 m / 15 m'de 30'ar saniye SABİT DUR**

> Elimizdeki kutuların **%93,6'sı 13 m içinde**; 25+ m'de sadece **23 kutu** var.
> Durma şart: uzakta filtre kareleri eliyor, kalp atışı tek kurtarıcı.

### 3️⃣ KALABALIK + KAPI (~15 dk)
```
--interval 2.0   --zorunlu-aralik 10     ← varsayılana DÖN
```
- Dubaları **arka arkaya 3 kapı** olacak şekilde diz, içinden geç
- Kapıya **30-60° çapraz** yaklaş

> Model 5+ dubayı **hiç görmedi** (8 kare). Kapıdan geçerken dubalar kadraj
> **kenarında** olur — o anı çekmemiz lazım.

### 4️⃣ IŞIK + SU HATTI (~15 dk)
- **Güneşe karşı** seyret, su yansıması kadrajda olsun
- Dubaların **ipini/ballastını değiştir** → suya batma derinliği değişsin
- **Kendi teknenin gövdesi kadrajda** görünen kareler al

---

## ⚠️ ÜÇ KURAL

1. **Aynı yerde durup kare biriktirme.** Sorunumuz hacim değil, çeşitlilik.
2. **Etiketleme:** gövdenin **yarısı görünüyorsa** etiketle; yalnız bayrak/kulp
   görünüyorsa **geç**. Yansıma etiketlenmez. **Negatif cisimler etiketlenmez.**
3. **Mümkünse ikinci bir güne yay** (öğle + sabah ayrı gün). Aynı ışıkta 3 saat,
   1 saatten fazla bilgi taşımıyor.

---

## 🌊 BONUS: DENİZ — Akçakoca (~65 km)

Bir gün ayırabilirsen **en değerli tek oturum**. Yarışma denizde, verimiz göl:
su parlaklığımız 140, yarışma videosunda 85. Gölcük 150 km, Akçakoca 65 km.
Orada **1️⃣ + 2️⃣** yeter (negatif + uzak mesafe).

---

## ✅ DÖNÜŞTE

```bash
sudo systemctl stop girdap-veriseti
ls ~/girdap_veriseti/images | wc -l     # toplam kare
tail -3 ~/girdap_veriseti/manifest.csv  # son satır yazılmış olmalı
```
🔴 Oturumlardan **biri tamamen VALID'e** ayrılacak — eğitimde o oturumdan tek kare
olmayacak. Yoksa doğrulama yine kör kalır (%99 recall veriyor ama alan dışında çöküyor).

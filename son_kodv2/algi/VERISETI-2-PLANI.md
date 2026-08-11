# 🎯 VERİ SETİ-2 PLANI — yarışma alanında her şeyi görmek için

> **Amaç:** modelin TEKNOFEST parkurunda, yarışma saatinde, yarışma suyunda
> dubaların **tamamını** görmesi.
> **Yöntem:** önce mevcut veri setinin **neyi görmediğini ölçtük**, sonra planı
> yalnız o boşluklara kurduk. Her madde bir ölçüme ya da şartname maddesine bağlı.
> Kaynak: 11.08.2026 PC oturumu · şartname tam taraması (doğrulanmış PDF
> `sha256 09116afe…`, 29 sayfa) · 12.754 karelik mevcut setin ölçümü.

---

## 1. 🔴 EN BÜYÜK BULGU: her şeyi TEK bir öğleden sonradan öğrendik

Mevcut 12.754 karenin **tamamı** tek oturumdan geliyor:

| ölçüt | değer |
|---|---|
| farklı **gün** | **1** (07.08.2026) |
| farklı **oturum** | **1** (`20260807_150930`) |
| saat aralığı | **15:00-17:00** (15:00 %65,5 · 16:00 %26,6 · 17:00 %7,9) |
| **öğle bandı 10:00-14:00** | **%0** |
| su | tek göl, tek kıyı |

Manifest'te 28 oturum / 3 gün var ama **elemeden sonra tek oturum kaldı**.
⇒ Modelin "dünya" bilgisi = *bir ikindi vakti, bir gölde, bir kıyı şeridi.*

🔑 **Memory'de aylardır "alan farkı (domain gap), sebebi bulunamadı" diye duran
şeyin cevabı bu.** Tek tek denenen 9 etkenin hiçbiri hatayı açıklayamamıştı;
açıklama etkenlerde değil, **çeşitlilik yokluğunda.**

---

## 2. Ölçülen boşluklar — öncelik sırasıyla

### 🥇 A. UZAK MESAFE neredeyse hiç yok
Etiketli 27.849 kutunun mesafe dağılımı (bbox genişliği @1352 px, 30 cm duba):

| mesafe | kutu | oran |
|---|---|---|
| < 3,5 m | 7.806 | %28,0 |
| 3,5-7 m | 8.941 | %32,1 |
| 7-13 m | 9.320 | %33,5 |
| **13-25 m** | **1.759** | **%6,3** |
| **25+ m** | **23** | **%0,1** |

⇒ **%93,6'sı 13 m'nin içinde.** Modelin 20 m'de %62,7 recall vermesi şaşırtıcı
değil — o mesafeyi neredeyse hiç görmemiş. Bu, 512'ye geçişin de asıl gerekçesiydi
([[cozunurluk-512-ve-gercek-fps]]) ama **çözünürlük veriyi yerine koymaz.**

### 🥈 B. Sahne hiç KALABALIK olmuyor
Kare başına duba: 0→%20,5 · 1→%17,2 · 2→%10,5 · 3→%13,2 · **4→%38,7** ·
5+ → **8 kare (%0,07)**.
⇒ Model **5 ve üzeri dubayı pratikte hiç görmedi**. Gerçek parkurda ileride
birden fazla kapı aynı anda görünür. Şartname: *"parkurlarda kullanılan duba
sayıları ve kenar dubaları arasındaki mesafeler yarışma alanına göre
değişkenlik gösterecektir."*

### 🥉 C. SU ve ARKA PLAN yarışmadakine benzemiyor
Bizim göl ↔ TEKNOFEST yarışma videosu (600 kare) ölçümü:

| | su tonu | doygunluk | **parlaklık** |
|---|---|---|---|
| bizim (göl, ikindi) | 206° | 87 | **140** |
| yarışma videosu | 198° | 110 | **85** |

Ton yakın (8°) ama **parlaklık farkı 55** — bizimki çok daha aydınlık.
Arka plan ise tamamen başka: bizde **%86,2 karede ağaç/tepe/kıyı**, yarışmada
**liman/marina** — binalar, vinçler, tekneler, dalgakıran, insanlar.

### 🏅 D. TURUNCUMUZ ŞARTNAMEDEN KIRMIZIYA KAÇIK
Etiketli kutuların gerçek tonu (dairesel istatistik):

| sınıf | medyan | %5-95 | şartname |
|---|---|---|---|
| kenar/turuncu | **11,5°** | −16…28° | RAL 2003 = **24,8°** |
| engel/sarı | 57,0° | 38…93° | RAL 1026 = **60,0°** |

Sarı isabetli. **Turuncu ~13° kırmızıya kaçık.** Yarışma turuncusu (24,8°) bizim
dağılımımızın **%95'lik sınırında** (28,2°) duruyor — yani karar sınırının tam
kenarında. Ton kopyalarının neden bu kadar işe yaradığı ve videoda neden turuncuyu
sarı sandığı buradan.
✅ İyi haber: iki sınıf **ayrık**, aralarında 10,2° boşluk var; medyan farkı 45,5°
(şartname 35,2°) — sınıf ayrımı sağlam, sorun turuncunun **konumu**.

### E. Hiç görmediğimiz, ama parkurda KESİN olacak şeyler
Şartname md 5.5.2.1'den:

| nesne | durum | risk |
|---|---|---|
| **Beyaz sosis tip duba** — *"parkur dışında alanı çevrelemek ve güvenliği sağlamak için beyaz renk sosis tip dubalar bulunacaktır"* | 🔴 verimizde **YOK** | Parkurun **her kenarında** olacak; model hiç görmedi |
| **Hedef dubaları** Ø640×950 mm, RAL 9005 (siyah) / **RAL 3026 (kırmızı)** / RAL 6037 (yeşil) | 🔴 **YOK** | **RAL 3026 turuncuya çok yakın** → hayalet kenar dubası → sahte kapı → P1/P2 |
| Kendi teknemizin gövdesi kadrajda | 🔴 **YOK** (verimizde tekne parçası görünmüyor) | Yarışmada kamera teknede; alt bant sürekli gövde görecek |
| Diğer tekneler, iskele, vinç, şamandıra | 🔴 **YOK** | Limanda turuncu/sarı cisim bol |

### F. ✅ İyi durumda olanlar — bozma
- **Sınıf dengesi mükemmel:** kenar %50,5 / engel %49,5
- **Boş kare (negatif) oranı %20,5** — sağlıklı
- **Duba tipi ARMUT** ✅ — şartname *"parkurda sadece Armut tip dubalar kullanılacaktır"*
- Güneş parlaması olan kare %7,6 (az ama var)

---

## 3. Şartnamenin plana koyduğu kısıtlar

1. 🔴 *"Burada verilen şamandıraların boyut ve renk bilgileri ile yarışma
   alanındaki duba ve engellerin **boyut ve renklerinde farklılıklar
   olabilecektir**."*
   ⇒ **Tam RAL tonuna ezberletme.** Ton çeşitliliği bir süs değil, şartname
   gereği. Dar bir ton bandına oturan model sahada patlar.
2. 🔴 *"dubaların suyun içinde olduğu ve **suyun üzerinde kalan yüksekliğin o
   anki şartlara bağlı olduğu** unutulmamalıdır."*
   ⇒ **En/boy oranı güvenilir ipucu DEĞİL.** Modelimiz bunu ikincil ipucu olarak
   öğrenmiş (kenar h/w 1,397 ↔ engel 1,324) — şartname bunun değişeceğini
   söylüyor. Bağı kırmak gerek.
3. *"Deniz Durumu-2'ye kadar görev yapabilecektir."* ⇒ dalga var: duba yalpalar,
   kısmen suya gömülür, su yüzeyi kırışık.
4. *"Parkurların uzunlukları, duba sayıları ve kenar dubaları arasındaki
   mesafeler yarışma alanına göre değişkenlik gösterecektir."*
5. *"Yarışma parkurlarının önceden haritalanmasına izin verilmeyecektir."*

---

## 4. 🎯 PLAN — ne çekilecek, ne kadar

**Temel ilke: ÇEŞİTLİLİK > HACİM.** 4 farklı oturumdan 3.000'er kare, tek
oturumdan 12.000 kareden **kat kat** iyidir. Elimizdeki 12.754 kare bunun kanıtı:
hacim var, çeşitlilik yok.

### Hedef: **≥4 ayrı oturum**, toplam ~10-12 bin YENİ kare

| # | oturum | ne zaman | neden (ölçüm) |
|---|---|---|---|
| 1 | **Öğle** | 11:00-14:00 | Öğle bandı **%0**; en sert güneş, en çok parlama |
| 2 | **Sabah** | 08:00-10:00 | Alçak güneş, uzun gölge, farklı renk sıcaklığı |
| 3 | **İkindi** | 15:00-17:00 | Elimizde var ama **yeni koşullarla** (yeni yer/dizilim) |
| 4 | 🔴 **Deniz/kıyı** | herhangi | Yarışma **denizde**; suyumuz göl. Ulaşılabiliyorsa **en yüksek değerli tek oturum** |
| 5 | Bulutlu / rüzgârlı gün | fırsat | Parlaklık 140→85 aralığını doldurur |

⚠️ Tek gün mümkünse **iki farklı saat** dilimine bölünmeli; aynı ışıkta 3 saat
çekim, 1 saat çekimden fazla bilgi taşımıyor (elimizdeki set bunu gösteriyor).

### Her oturumda mutlaka yapılacak manevralar

| manevra | süre | neyi doldurur |
|---|---|---|
| 🥇 **Uzun yaklaşma**: 40-50 m'den dubaya doğru **yavaş** (≤1 m/s) düz seyir, en az **10 tekrar** | ~25 dk | 13-25 m bandı %6,3 → hedef **%20+**; 25+ m %0,1 → **%5+** |
| 🥇 **Mesafede DURMA**: 30 / 25 / 20 / 15 m'de **30'ar sn sabit dur** | ~10 dk | Uzak banda yoğun kare; `--interval` uzakta az kare üretiyor |
| 🥈 **Kalabalık dizilim**: tüm dubaları **arka arkaya 3 kapı** olacak şekilde diz, kapılardan geç | ~15 dk | 5+ duba/kare **%0,07** → hedef **%15+** |
| 🥈 **Güneşe karşı seyir**: güneş tam karşıda ve suda yansırken | ~10 dk | Parlama %7,6 → hedef %20 |
| 🥉 **Yan/çapraz açı**: dubaya 30-60° açıyla yaklaş | ~10 dk | Kapıdan geçerken dubalar kadraj kenarında olur |
| 🥉 **Su hattı değiştir**: dubaların ballastını/ipini değiştirip **suya batma derinliğini** oynat | ~5 dk | Şartname md 5.5.2.1 (yükseklik değişken); en/boy bağını kırar |

### 🔴 NEGATİF çekim (etiketsiz kare) — yeni ve şart

Modelin **görüp de etiketlemeyeceği** şeyler. Boş kare oranı %20,5'te kalsın ama
**içeriği zenginleşsin**:

| çekilecek | neden |
|---|---|
| **Beyaz sosis/silindir cisim** (beyaz usturmaça, PVC boru, beyaz bidon) suda | Parkur çevresi beyaz sosis dubayla çevrili olacak |
| 🔴 **Büyük KIRMIZI cisim** (kırmızı bidon/varil/can yeleği) suda | RAL 3026 hedef dubası turuncuya çok yakın; hayalet kenar dubası = sahte kapı = P1/P2 |
| Büyük **yeşil** ve **siyah** cisim | Diğer iki hedef duba rengi |
| Turuncu/sarı **can yeleği, şamandıra, bidon** (duba OLMAYAN) | Limanda bol; yanlış pozitif kaynağı |
| Tekne, iskele, insan, direk | Yarışma limanı kalabalık |
| **Kendi teknemizin gövdesi kadrajda** | Yarışmada kamera teknede — alt bant sürekli gövde görecek |

📌 Bunlar **etiketlenmez**; modelin "bunlar duba değil" demeyi öğrenmesi için var.

---

## 5. Toplama ayarları — bu oturuma özel

Varsayılanlar `VERISETI-JETSON-KARTI.md`'de; **uzak mesafe için iki değişiklik**:

```bash
# UZAK MESAFE oturumu (yavaş yaklaşma + durma manevraları)
--interval 1.0        # 2.0 yerine: yavaş giderken 2 sn'de 1 kare çok seyrek
--zorunlu-aralik 5    # 10 yerine: DURURKEN kalp atışı kare üretmeye devam etsin
```

🔑 **Neden:** `--min-fark 0.5` filtresi **tüm karenin** ortalama farkına bakıyor;
25 m'deki duba ortalamayı neredeyse hiç değiştirmiyor ⇒ tam ihtiyacımız olan
kareler eleniyor. Durma manevralarında kalp atışı tek kurtarıcı.
⚠️ Kalabalık/yakın manevralarda **varsayılana dön** (`2.0` / `10`), yoksa yine
yakın mesafe kare yığını üretiriz — zaten fazlasıyla var.

Çözünürlük **1352×1014 kalır** (512 değil) — gerekçesi kartta.

---

## 6. Etiketleme kuralları

Değişmeyenler:
- **Sudaki yansıma etiketlenmez**
- **Sarının iki tonu da `engel_dubasi`**
- **train/valid AYRI yüklenir** (Roboflow)

Yeni / netleşen:
- 🔴 **Gövdenin en az yarısı görünüyorsa etiketle; yalnız bayrak/kulp görünüyorsa GEÇ.**
  (Eski sette 16 kutu bu yüzden sorunluydu — etiket hatası değil, tanım boşluğuydu.)
- 🔴 **Beyaz sosis duba, hedef dubaları, bidon/can yeleği — HİÇBİRİ etiketlenmez.**
  İki sınıfımız var: `kenar_dubasi` (turuncu) ve `engel_dubasi` (sarı). Başka sınıf
  **açılmaz** — model 4 shave'e sığıyor, sınıf eklemek bütçeyi ve kararlılığı bozar.
- Çok uzak, 3 pikselden küçük, insan gözüyle **duba olduğu seçilemeyen** cisim
  etiketlenmez (model onu zaten öğrenemez, gürültü katar).

---

## 7. Eğitim tarafı — yeni veriyle ne değişecek

1. **Bölme yine zaman-bloğu + guard band** (sızıntısız). Ama artık **oturum
   bazlı** da ayrılabilir: bir oturumun tamamını valid'e koymak, "hiç görmediğim
   koşulda ne yapıyorum" sorusunun en dürüst ölçümüdür. **Öneri: yeni oturumlardan
   biri tamamen VALID.**
2. **Ton kopyaları yine üretilecek** (`arac/ton_kopyasi_uret_v3.py`) — turuncumuz
   13° kırmızıya kaçık olduğu sürece şart. Yeni veri turuncuyu 24,8°'e yaklaştırırsa
   kopya sayısı azaltılabilir; **ölçmeden azaltma**.
3. 🔴 **En/boy bağını kır:** hafif `shear` (2-3°) ya da çevrimdışı dikey ±%10
   esnetme. Şartname yüksekliğin değişeceğini söylüyor; modelimiz en/boy'u ipucu
   olarak öğrenmiş durumda (ölçüldü: %17 etkisi).
4. 512 girişte eğit, **4 shave** blob, `--reverse_input_channels`.
5. **Kabul kapıları değişmiyor** — yeni model bunları geçmeden dağıtıma girmez:
   ① 20+ m recall mevcut **%62,7'yi geçecek** ② genel recall %97,0'dan 0,5 puandan
   fazla düşmeyecek ③ yarışma videosunda turuncuya "engel" oranı ≤%5.
   **Ek kapı (yeni):** ④ negatif çekimlerde (beyaz/kırmızı/yeşil cisim) yanlış
   pozitif oranı ölçülecek ve raporlanacak.

---

## 8. Ne YAPILMAYACAK

- ❌ **Aynı yerde, aynı saatte 12.000 kare daha.** Elimizdeki setin sorunu bu.
- ❌ **Sınıf eklemek** (beyaz duba / hedef duba sınıfı). 4 shave bütçesi ve
  turuncu↔sarı ayrımının kararlılığı buna değmez; negatif örnek yeterli.
- ❌ **512'de toplamak.** Gerekçe kartta — geri dönüşü yok, elle etiketlemede
  1 px hata mesafede %18 sapma demek.
- ❌ **Kenar tespitlerini süzmek.** Ölçüldü ve reddedildi: kapıdan geçerken
  dubalar tam kenarlarda olur, tam o anda körleşiriz.
- ❌ **Yarışmadan önce dağıtımı yeni modelle değiştirmek — kabul kapılarını
  geçmeden.** Elimizde çalışan, ölçülmüş bir model var.

---

## 9. Özet — tek cümlelik öncelik sırası

1. 🥇 **Farklı gün + farklı saat** (özellikle **öğle 11:00-14:00**) — tek oturumluk körlüğü kırar
2. 🥇 **Uzak mesafe manevraları** (40-50 m'den yavaş yaklaşma + mesafede durma)
3. 🥈 **Deniz/kıyı oturumu** — ulaşılabiliyorsa tek başına en değerli
4. 🥈 **Kalabalık dizilim** (5+ duba/kare) + **negatif cisimler** (beyaz sosis, **kırmızı**, yeşil)
5. 🥉 Güneşe karşı seyir, yan açı, su hattı değişimi

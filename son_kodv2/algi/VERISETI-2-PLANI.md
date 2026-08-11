# 🎯 VERİ SETİ-2 PLANI — Gölcük'te her dubayı görmek için

> **Soru:** yarışma alanında dubaların **tamamını** görecek eğitim setini nasıl kurarız?
> **Yöntem:** hiçbir madde tahmine dayanmıyor. Önce mevcut 12.754 karenin
> **neyi görmediği ölçüldü**, sonra plan yalnız o boşluklara kuruldu.
> **Kaynaklar:** şartname tam taraması (doğrulanmış PDF `sha256 09116afe…`, 29 sayfa) ·
> mevcut veri setinin ölçümü · gerçek modelle (`girdap_512_ep87.pt`) yapılan testler ·
> Gölcük için hesaplanmış güneş geometrisi · web'den doğrulanmış yarışma yeri.
> **Tarih:** 11.08.2026 · **Yarışmaya:** 9 gün

---

## 0. ÖNCE: yarışma yeri artık VARSAYIM DEĞİL

Memory'de *"yarışma YERİ 🔴 hâlâ açık"* diye duruyordu. Şartname de söylemiyor
(yalnız *"Ağustos-Eylül 2026 YARIŞMA TARİHİ-YERİ"*). Web'den doğrulandı:

**TEKNOFEST Mavi Vatan 2026 · 20-23 Ağustos 2026 · Kocaeli, Gölcük Tersanesi Komutanlığı**

Bunun teknik sonuçları büyük:
- **İzmit Körfezi** — kapalı koy. Açık deniz değil ⇒ dalga düşük (şartname zaten
  Deniz Durumu-2 diyor), ama su **endüstriyel körfez suyu**: bulanık, yeşil-kahve,
  Bolu gölünden de açık denizden de farklı.
- **Donanma tersanesi** — arka planda gri savaş gemileri, vinçler, rıhtım,
  dalgakıran, konteyner. Bizim verimizde arka plan **%86,2 ağaç/tepe/kıyı**.
- Enlem **40,72°K** — Bolu ile neredeyse aynı (40,73°K) ⇒ **güneş açısı aynı**,
  yani saat farkı doğrudan karşılaştırılabilir (aşağıda kullanıldı).

📌 **Biz Bolu'dayız.** Gölcük'e ~150 km. Ama **Karadeniz kıyısı (Akçakoca) ~60-70 km** —
deniz suyu görmek için gerçekçi tek seçenek bu (§4'te).

---

## 1. 🔴🔴 EN AĞIR BULGU: model dubayı RENGİNDEN DEĞİL, BİÇİMİNDEN tanıyor

Bu, bu oturumun en önemli ölçümü. Gerçek karelerdeki duba piksellerini şartnamedeki
diğer renklere **boyadım** (biçim, boyut, su, ışık aynen kaldı — yalnız renk değişti),
sonra gerçek modeli koşturdum. 1.749 duba kutusu, 868 kare, `conf ≥ 0,5`:

| renk — parkurda VAR, verimizde YOK | yanlış pozitif | model ne diyor |
|---|---|---|
| **RAL 3026 KIRMIZI** (hedef dubası) | 🔴🔴 **%97,4** | **kenar_dubasi 1641** / engel 63 — güven **0,84** |
| **RAL 6037 YEŞİL** (hedef dubası) | 🔴🔴 **%86,4** | engel_dubasi 1498 / kenar 13 — güven 0,76 |
| **BEYAZ** (sosis sınır dubası) | 🔴🔴 **%70,2** | engel 877 / kenar 348 — güven 0,73 |
| **RAL 9005 SİYAH** (hedef dubası) | 🔴 **%54,1** | engel 611 / kenar 333 — güven 0,72 |
| *(karşılaştırma)* gerçek turuncu/sarı | %99,5 | — |

🔑 **Model, duba biçimindeki HER ŞEYİ duba sanıyor** ve rengine en yakın sınıfı
yapıştırıyor. Sebep basit: eğitim setinde **duba biçiminde ama başka renkte tek bir
örnek yok**. Modelin "bu duba değil" demeyi öğrenmesi için hiç fırsatı olmamış.

⚠️ **Testin dürüst sınırı:** boyama biçimi korudu. Gerçek **sosis** duba yatık
silindirdir, armut dubadan farklı görünür ⇒ **%70,2 beyaz için ÜST SINIRDIR**,
gerçek oran daha düşük olabilir. Ama kırmızı/yeşil/siyah hedef dubaları **armut
benzeri** olduğu için o rakamlar doğrudan geçerli.

### Peki savunmamız tutuyor mu? — kısmen

`buyuk_cisim_mi()` filtresi (Ø0,64 hedef dubasını boyutundan eleyen kural):

| durum | 5-25 m arası hedef dubası |
|---|---|
| **stereo derinlik ÇALIŞIYORSA** | ✅ **elendi** (her mesafede) |
| **stereo YOKSA** (pinhole yedeğine düşülmüş) | 🔴 **GEÇTİ — filtre kör** |

🔴 Ve **beyaz sosis sınır dubasını boyut filtresi AYIRAMAZ** — çapı kenar dubasıyla
aynı (Ø0,30-0,50 m), her mesafede geçiyor.

⚠️ Memory'de kayıtlı: *"derinlik kalitesi iç mekânda ✅, **suda ❌**"* — pinhole
yedeğini zaten bu yüzden yazdık. Yani **kırmızı hedef dubası savunması, tam da
çalışmadığını bildiğimiz sensöre bağlı.**

### Sonucu ne olur
- **Kırmızı hedef dubası → "turuncu kenar dubası"** ⇒ sahte kapı ⇒ İDA olmayan bir
  geçitten geçmeye çalışır ⇒ **P1 (G1/KD1 ≥ 0,5) ve P2 (≥2 ikili) gider.**
- **Beyaz sosis dubalar parkurun HER kenarında** (şartname: *"parkur dışında alanı
  çevrelemek ve güvenliği sağlamak için beyaz renk sosis tip dubalar bulunacaktır"*)
  ⇒ hayalet **engel** ⇒ maliyet haritası kirlenir, MPPI yok yere kaçınır.

⇒ **Bu, veri setiyle çözülür. Ayarla, eşikle, filtreyle çözülmez.** (§4-C)

---

## 2. 🔴 İKİNCİ AĞIR BULGU: doğrulama setimiz KÖR

| bant (bbox genişliği @1352) | eğitim örneği | **valid recall** |
|---|---|---|
| 17-25 m (8-12 px) | 133 | **%100,0** (12/12) |
| 13-17 m (12-16 px) | 1.350 | **%94,1** (127/135) |
| 9-13 m (16-24 px) | 4.149 | %99,1 |
| 5-9 m (24-40 px) | 5.631 | %99,7 |
| < 5 m (40+ px) | 10.275 | %99,7 |

Valid'de her bantta **%94-100**. Ama aynı model TEKNOFEST videosunda turuncuyu
sarı sanıyordu (%18,8 → ton kopyalarıyla %3,3) ve yukarıdaki renk testinde
**%97,4 yanlış pozitif** veriyor.

🔑 **Sebep: valid setimiz eğitim setiyle AYNI OTURUMDAN.** Zaman-bloğu bölmesi +
guard band sızıntıyı engelliyor ama **alan değişimini ölçmüyor** — aynı göl, aynı
ışık, aynı dubalar, aynı iki saat. ⇒ **%99 recall "ezberledim" demek, "genelliyorum"
demek değil.**

⚠️ Not: memory'de 512 modeli için *"20+ m recall %62,7"* kayıtlı; buradaki ölçüm
aynı bantta çok daha yüksek çıkıyor. İki ölçüm farklı bant tanımı/tezgâh
kullanıyor olabilir — **bu çelişki yeni veriyle birlikte tekrar ölçülmeli**,
şimdilik ikisinden biri "doğru" ilan edilmiyor. Değişmeyen sonuç: **kendi
valid'imiz yarışma başarısını öngöremiyor.**

### ⇒ Plana giren zorunlu kural
**Yeni oturumlardan en az biri, tamamı VALID olacak** — eğitimde o oturumdan tek
kare bulunmayacak. "Hiç görmediğim koşulda ne yapıyorum" sorusunun tek dürüst ölçümü budur.

---

## 3. Ölçülen kapsama boşlukları

### A. Her şeyi TEK bir ikindiden öğrendik
| ölçüt | değer |
|---|---|
| farklı **gün** | **1** (07.08.2026) |
| farklı **oturum** | **1** (`20260807_150930`) |
| saat | **15:00-17:00** (15:00 %65,5 · 16:00 %26,6 · 17:00 %7,9) |
| öğle bandı 10:00-14:00 | **%0** |

Manifest'te 28 oturum / 3 gün var; elemeden sonra **tek oturum kaldı**.

🔑 Memory'de *"alan farkı — 9 etken denendi, hiçbiri açıklamıyor, AÇIK SORU"*
diye duran şeyin cevabı bu. Açıklama etkenlerde değil, **çeşitlilik yokluğunda**.

### B. Güneş açısı — hesaplandı, çakışma yarım
Gölcük (40,72°K) 21.08.2026 ile Bolu verimizin (07.08, 15:00-17:00) karşılaştırması:

| yarışma saati | güneş yüksekliği | verimizde var mı |
|---|---|---|
| 09:00 | 29,7° | 🔴 YOK |
| 10:00 | 40,7° | ✅ var |
| 11:00 | 50,5° | ✅ var |
| **12:00** | **58,1°** | 🔴 **YOK** |
| **13:00** | **61,3°** (tepe) | 🔴 **YOK** |
| **14:00** | **58,8°** | 🔴 **YOK** |
| 15:00 | 51,6° | ✅ var |
| 16:00 | 42,0° | ✅ var |
| 17:00 | 31,1° | 🔴 YOK |

**Kapsadığımız aralık 34°-55°. Yarışma saatlerinin 5/9'u dışarıda.**
Eksik bant **55°-61° = 11:00-15:00** — yani muhtemel koşu saatlerinin göbeği.

📌 Parlama geometrisi: güneş **<30°** iken su aynası yatay bakan kameraya **doğrudan**
vurur (07:00-08:00 ve 18:00-19:00). Yarışma saatlerinde en riskli uçlar **09:00** ve
**17:00**; 11:00-15:00'te parlama teknenin yakınına düşer ama **su yüzeyi en parlak**
olduğu için kontrast düşer.

### C. Uzak mesafe neredeyse yok
27.849 etiketli kutunun dağılımı:

| mesafe | kutu | oran |
|---|---|---|
| < 3,5 m | 7.806 | %28,0 |
| 3,5-7 m | 8.941 | %32,1 |
| 7-13 m | 9.320 | %33,5 |
| **13-25 m** | **1.759** | **%6,3** |
| **25+ m** | **23** | **%0,1** |

**%93,6'sı 13 m'nin içinde.** Çözünürlüğü 416→512 yapmak yardımcı oldu ama
**çözünürlük veriyi yerine koymaz.**

### D. Sahne hiç kalabalık olmuyor
Kare başına duba: 0→%20,5 · 1→%17,2 · 2→%10,5 · 3→%13,2 · **4→%38,7** ·
**5+ → 8 kare (%0,07)**.
⇒ Model 5+ dubayı **pratikte hiç görmedi**. Gerçek parkurda ileride birden fazla
kapı aynı anda görünür. Şartname: *"duba sayıları ve kenar dubaları arasındaki
mesafeler yarışma alanına göre değişkenlik gösterecektir."*

### E. Su ve arka plan
| | su tonu | doygunluk | **parlaklık** |
|---|---|---|---|
| bizim (Bolu gölü, ikindi) | 206° | 87 | **140** |
| TEKNOFEST yarışma videosu | 198° | 110 | **85** |

Ton yakın (8°) ama **parlaklık farkı 55**. Arka plan bambaşka: bizde %86,2 karede
ağaç/tepe, yarışmada liman/tersane.

### F. Turuncumuz şartnameden kırmızıya kaçık
| sınıf | bizim medyan | %5-95 | şartname |
|---|---|---|---|
| kenar/turuncu | **11,5°** | −16…28° | RAL 2003 = **24,8°** |
| engel/sarı | 57,0° | 38…93° | RAL 1026 = **60,0°** |

Sarı isabetli, **turuncu ~13° kırmızıya kaçık**. Yarışma turuncusu (24,8°) bizim
dağılımımızın **%95 sınırında** (28,2°) — karar sınırının tam kenarında.
Ton kopyalarının neden bu kadar işe yaradığı buradan.
✅ İyi haber: iki sınıf **ayrık** (10,2° boşluk), medyan farkı 45,5° (RAL 35,2°).

### G. ✅ Bozulmayacak olanlar
- Sınıf dengesi **%50,5 / %49,5** — mükemmel
- Boş kare (negatif) oranı **%20,5** — sağlıklı
- Duba tipi **ARMUT** ✅ (şartname: *"parkurda sadece Armut tip dubalar kullanılacaktır"*)
- Kamera/ISP geometrisi dağıtımla birebir (1352×1014, tam 4:3, kırpma yok)

---

## 4. 🎯 PLAN

### Temel ilke
> **ÇEŞİTLİLİK > HACİM.** 4 oturum × 3.000 kare, tek oturumdan 12.000 kareden
> kat kat iyidir. Elimizdeki set bunun kanıtı: hacim var (12.754), çeşitlilik yok (1 oturum).

### Hedef: **≥4 ayrı oturum**, ~10-12 bin YENİ kare

| # | oturum | ne zaman | gerekçe (ölçüm) | öncelik |
|---|---|---|---|---|
| 1 | **ÖĞLE** | **11:00-14:00** | Güneş 55-61° bandı **%0**; yarışmanın göbeği | 🥇 |
| 2 | **Uzak mesafe** | herhangi | 13-25 m %6,3 · 25+ m %0,1 | 🥇 |
| 3 | 🌊 **DENİZ** — Akçakoca (~65 km) | herhangi | Yarışma denizde; suyumuz göl. Tek başına en yüksek değerli | 🥈 |
| 4 | **Sabah/geç ikindi** | 08:00-10:00 ve 17:00+ | Güneş <34° bandı yok; parlama en sert | 🥈 |
| 5 | Bulutlu/rüzgârlı gün | fırsat | Parlaklık 140→85 aralığını doldurur | 🥉 |

🔴 **Oturumlardan biri (tercihen DENİZ ya da ÖĞLE) tamamen VALID'e ayrılacak** — §2.

### A. Her oturumda yapılacak manevralar

| manevra | süre | neyi doldurur |
|---|---|---|
| 🥇 **Uzun yaklaşma**: 40-50 m'den **yavaş** (≤1 m/s) düz seyir, **≥10 tekrar** | ~25 dk | 13-25 m: %6,3 → **%20+** · 25+ m: %0,1 → **%5+** |
| 🥇 **Mesafede DURMA**: 30/25/20/15 m'de **30'ar sn sabit** | ~10 dk | Uzak banda yoğun kare (yaklaşma tek başına yetmiyor) |
| 🥈 **Kalabalık dizilim**: dubaları **arka arkaya 3 kapı** yap, geç | ~15 dk | 5+ duba/kare: %0,07 → **%15+** |
| 🥈 **Güneşe karşı seyir** (suda yansıma kadrajda) | ~10 dk | Parlamalı kare %7,6 → **%20** |
| 🥉 **Yan/çapraz açı** (30-60°) | ~10 dk | Kapıdan geçerken dubalar kadraj **kenarında** olur |
| 🥉 **Su hattı değiştir** (ballast/ip boyu) | ~5 dk | Şartname: yükseklik değişken; en/boy bağını kırar |

### B. Toplama ayarları — manevraya göre

```bash
# UZAK MESAFE ve DURMA manevraları
--interval 1.0        # 2.0 yerine
--zorunlu-aralik 5    # 10 yerine

# YAKIN / KALABALIK manevralar
--interval 2.0  --zorunlu-aralik 10      # varsayılan
```

🔑 **Neden:** `--min-fark 0.5` filtresi **tüm karenin** ortalama farkına bakıyor;
25 m'deki 30 cm duba ortalamayı neredeyse hiç değiştirmiyor ⇒ **tam ihtiyacımız
olan kareler eleniyor**. Dururken kalp atışı (`--zorunlu-aralik`) tek kurtarıcı.
Çözünürlük **1352×1014** kalır (gerekçe: `VERISETI-JETSON-KARTI.md` §3).

### C. 🔴 NEGATİF ÇEKİM — planın en kritik yeni parçası

§1'deki %97,4 doğrudan buradan çözülür. Boş kare oranı %20,5'te kalsın, **içeriği
zenginleşsin**. Hedef: **≥1.500 kare**, içinde duba biçiminde ama duba OLMAYAN cisim.

| çekilecek | neden | ne kadar |
|---|---|---|
| 🔴🔴 **Büyük KIRMIZI cisim** suda (kırmızı bidon/varil/can yeleği/top) | RAL 3026 hedef dubası → **%97,4 "turuncu kenar dubası"** ⇒ sahte kapı ⇒ P1/P2 | **≥500 kare** |
| 🔴 **BEYAZ silindir/sosis** suda (beyaz usturmaça, PVC boru, beyaz bidon) | Parkuru **çevreleyecek**; %70,2 "engel" ⇒ hayalet engel | **≥400 kare** |
| 🔴 **YEŞİL** ve **SİYAH** büyük cisim | Diğer iki hedef duba rengi (%86,4 / %54,1) | **≥300 kare** |
| Turuncu/sarı ama duba OLMAYAN (can yeleği, şamandıra, bidon, boya kovası) | Limanda bol; en sinsi yanlış pozitif | ~200 kare |
| Tekne, iskele, direk, insan, kuş | Tersane kalabalık | ~100 kare |
| 🔴 **Kendi teknemizin gövdesi kadrajda** | Yarışmada kamera teknede; alt bant sürekli gövde görecek — verimizde **hiç yok** | her oturumda |

📌 Bunların **hiçbiri etiketlenmez.** Model "bunlar duba değil" demeyi ancak
görerek öğrenir.
💡 Ucuz kaynak: kırmızı/beyaz/yeşil **plastik bidon** ve **can yeleği** — armut
dubaya boyut ve siluet olarak yeterince benzer; gerçek hedef dubası bulmak şart değil.

---

## 5. Etiketleme kuralları

**Değişmeyenler**
- Sudaki **yansıma etiketlenmez**
- Sarının **iki tonu da** `engel_dubasi`
- **train/valid AYRI** yüklenir (Roboflow)

**Yeni / netleşen**
- 🔴 **Gövdenin en az yarısı görünüyorsa etiketle; yalnız bayrak/kulp görünüyorsa GEÇ.**
  (Eski sette 16 kutu bu yüzden sorunluydu — etiket hatası değil, tanım boşluğu.)
- 🔴 **Beyaz sosis · hedef dubaları · bidon · can yeleği — HİÇBİRİ etiketlenmez.**
- **Sınıf SAYISI ARTMAYACAK.** İki sınıf: `kenar_dubasi`, `engel_dubasi`. Model
  4 shave'e sığıyor; sınıf eklemek bütçeyi ve turuncu↔sarı ayrımının kararlılığını bozar.
- İnsan gözüyle **duba olduğu seçilemeyen** (≲3 px) cisim etiketlenmez — model
  onu öğrenemez, yalnız gürültü katar.

---

## 6. Eğitim tarafı

1. 🔴 **Bölme oturum bazlı olacak:** bir oturumun **tamamı** valid. Zaman-bloğu +
   guard band korunur ama tek başına yeterli değil (§2).
2. **Ton kopyaları yine üretilecek** (`arac/ton_kopyasi_uret_v3.py`) — turuncumuz
   13° kırmızıya kaçık olduğu sürece şart. Yeni veri turuncuyu 24,8°'e yaklaştırırsa
   kopya sayısı azaltılabilir; **ölçmeden azaltma.**
3. 🔴 **En/boy bağını kır:** hafif `shear` (2-3°) ya da çevrimdışı dikey ±%10 esnetme.
   Şartname *"suyun üzerinde kalan yükseklik o anki şartlara bağlı"* diyor; modelimiz
   en/boy'u ikincil ipucu olarak öğrenmiş (ölçüldü: %17 etki).
4. 512 giriş · **4 shave** blob · `--reverse_input_channels`.

### Kabul kapıları — yeni model bunları geçmeden dağıtıma GİRMEZ
| # | kapı | eşik |
|---|---|---|
| ① | **Tutulan oturumda** (hiç görmediği) recall | ölçülecek ve **raporlanacak** — asıl ölçüt bu |
| ② | 20+ m recall | mevcut değeri **geçecek** |
| ③ | Genel recall | mevcuttan 0,5 puandan fazla düşmeyecek |
| ④ | Yarışma videosunda turuncuya "engel" | **≤ %5** |
| ⑤ | 🔴 **YENİ — kırmızı/beyaz/yeşil/siyah yanlış pozitif** | **%97,4 → ≤ %20** hedef |

⑤'in tezgâhı hazır: aynı boyama testi yeniden koşturulur, sayı doğrudan karşılaştırılır.

---

## 7. Ne YAPILMAYACAK

- ❌ **Aynı yerde, aynı saatte 12.000 kare daha.** Elimizdeki setin sorunu tam bu.
- ❌ **Sınıf eklemek** (beyaz duba / hedef duba sınıfı). 4 shave bütçesi ve sınıf
  ayrımının kararlılığı buna değmez; **negatif örnek yeterli ve ucuz.**
- ❌ **512'de toplamak.** Geri dönüşü yok; elle etiketlemede 1 px hata → %18 mesafe
  sapması (`VERISETI-JETSON-KARTI.md` §3).
- ❌ **Kenar tespitlerini süzmek.** Ölçüldü ve reddedildi: kapıdan geçerken dubalar
  tam kenarlarda olur — en kritik anda körleşiriz.
- ❌ **Kabul kapılarını geçmeden dağıtımı değiştirmek.** Elimizde çalışan, ölçülmüş
  bir model var.
- ❌ **Yanlış pozitifi conf eşiğini yükselterek çözmeye çalışmak.** Kırmızıdaki
  güven **0,84** — gerçek dubalarınkiyle aynı bantta. Eşiği oraya çekmek gerçek
  dubaları da keser.

---

## 8. Özet — öncelik sırası

| sıra | iş | çözdüğü ölçülmüş sorun |
|---|---|---|
| 1 | 🔴🔴 **Negatif çekim** (kırmızı · beyaz · yeşil · siyah cisim, ≥1.500 kare) | **%97,4 yanlış pozitif** → sahte kapı → P1/P2 |
| 2 | 🥇 **Farklı gün + ÖĞLE (11:00-14:00)** | Güneş 55-61° bandı %0; yarışma saatlerinin 5/9'u dışarıda |
| 3 | 🥇 **Uzak mesafe manevraları** (40-50 m yaklaşma + mesafede durma) | 13-25 m %6,3 · 25+ m %0,1 |
| 4 | 🔴 **Bir oturumun tamamını VALID'e ayır** | Doğrulama setimiz kör (%99 recall ama videoda hata) |
| 5 | 🌊 **Deniz oturumu** (Akçakoca ~65 km) | Su + arka plan; parlaklık farkı 55 |
| 6 | 🥈 **Kalabalık dizilim** (5+ duba/kare) | %0,07 |
| 7 | 🥉 Güneşe karşı · yan açı · su hattı · **kendi gövdemiz kadrajda** | Parlama %7,6 · en/boy bağı · gövde hiç yok |

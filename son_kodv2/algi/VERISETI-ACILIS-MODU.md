# 🔌 AÇILIŞTA HANGİSİ KALKAR — ölçülmüş kök neden ve kalıcı düzeltme

> **12.08.2026, öğleden sonra — JETSON'DA CANLI ÖLÇÜLDÜ. Tahmin yok.**
>
> Kaptanın şikâyeti: *"kapatıp açtım ama veri seti hâlâ açılmıyor."*
> Bu dosya o arızanın kök nedenini, neden önceki iki çözümün de işe
> yaramadığını ve uygulanan kalıcı düzeltmeyi kaydeder.
>
> İlgili kartlar: `VERISETI-JETSON-KARTI.md` (bölüm 2 ve 8 **bu dosya
> yüzünden geçersiz** — aşağıda 7. bölüm) · `JETSON-KURULUM.md`
> Ölçüm anındaki depo başı: `ce91734`.

---

## 0. Bir cümlede

`girdap-karar.service` içindeki `Wants=girdap-algi.service` satırı, algıyı
açılışta **kendi başına** ayağa kaldırıyordu; algı kalkınca kendi birim
dosyasındaki `Conflicts=girdap-veriseti.service` toplayıcının başlatma işini
**sessizce** düşürüyordu. Toplayıcıyı `enable` etmek de, algıyı `disable`
etmek de bunu **engellemiyor**. Düzeltme: o satır temel birim dosyasında
kapatıldı — ek ayar dosyasıyla (systemd'nin *drop-in* dizini) **yapılamıyor**,
gerekçesi 3. bölümde.

---

## 1. Belirti — açılış sonrası ölçüm

Açılış **16:04:08**. Hemen ardından:

```
systemctl is-active girdap-veriseti girdap-algi
  inactive
  active                       ← ters olması gerekiyordu

journalctl -b -u girdap-veriseti
  -- No entries --             ← TEK SATIR bile yok: hata değil, iş hiç kurulmadı
```

🔑 **En önemli ayrıntı bu:** toplayıcı `failed` olmuyor, `journal`'e hiçbir şey
yazmıyor. Yani "servis çöktü" diye aranırsa **hiçbir iz bulunamaz** — arıza
kamera arızası gibi görünür. `VERISETI-JETSON-KARTI.md` bölüm 0'daki
*"servis açılışta başlamaz → tek kare toplanmaz"* riski tam olarak budur.

Durum, iddia edilenin aksine **doğru kurulmuştu**:

| ölçüm | değer |
|---|---|
| `systemctl is-enabled girdap-veriseti` | `enabled` |
| `systemctl is-enabled girdap-algi` | `disabled` |
| `girdap-algi` `multi-user.target.wants` içinde mi | **HAYIR** |
| `girdap-veriseti` `multi-user.target.wants` içinde mi | evet |

Yani kâğıt üzerinde her şey veri seti moduna ayarlıydı ve yine de algı kalktı.

---

## 2. Kök neden — çekme sırası ölçüldü

```
girdap-algi.service   etkin oldu:  16:04:25
girdap-karar.service  etkin oldu:  16:05:11
```

Algı, karardan **46 saniye ÖNCE** kalkmış. Bu tesadüf değil, `girdap-karar`'ın
kendi birim dosyasındaki iki satırın birlikte ürettiği sonuç:

```ini
After=girdap-algi.service      # sıra kurar: algı ÖNCE
Wants=girdap-algi.service      # çekme kurar: algı BAŞLATILIR
```

`girdap-karar` `enabled` olduğu için açılışta kalkar, kalkarken de `Wants=`
ile algıyı **beraberinde çeker**; `After=` yüzünden algı ondan önce başlar.
Algı ayağa kalkar kalkmaz kendi dosyasındaki

```ini
Conflicts=girdap-veriseti.service
```

satırı toplayıcının aynı açılış işlemindeki başlatma işini iptal eder.

🔑 **`Wants=` çekmesi `disable`'ı EZER.** `systemctl disable girdap-algi`
yalnızca `multi-user.target.wants` sembolik bağını siler; başka bir birimin
`Wants=` ile açıkça çağırmasını engellemez. Kaptanın *"algıyı kapattım ama
yine açılıyor"* gözlemi doğruydu ve sebebi buydu.

**Algıyı açılışta çeken TEK şey buydu** — sistem genelinde arandı:
`girdap-karar.service.wants/` dizini **yok**, `/run/systemd/system` **boş**,
`girdap-algi.service`'e giden başka hiçbir sembolik bağ **yok**.

---

## 3. 🔴 Neden ek ayar dosyası (drop-in) ile ÇÖZÜLEMEZ — systemd 249'da ölçüldü

12.08 saat 15:47'de `99-veriseti-modu.conf` adlı bir ek ayar dosyası kurulmuştu.
Dayandığı varsayım şuydu:

> *"Boş `Wants=` ataması listeyi SIFIRLAR (`man systemd.unit`), sonra saat ve
> livox geri yazılır; yalnızca algı düşürülür."*

**Bu varsayım bu makinede GEÇERSİZ.** Sıfırdan, daha önce hiç yüklenmemiş bir
test birimiyle ölçüldü (`/run/systemd/system` altında, ölçümden sonra silindi):

```ini
# zz-test-a.service
[Unit]
Wants=zz-test-b.service
After=zz-test-b.service

# zz-test-a.service.d/99-reset.conf
[Unit]
Wants=
Wants=zz-test-c.service
```

```
systemctl daemon-reload
systemctl show zz-test-a -p Wants
  Wants=zz-test-c.service zz-test-b.service
                          ^^^^^^^^^^^^^^^^ sıfırlamaya rağmen DURUYOR
```

Sürüm: **systemd 249 (249.11-0ubuntu3.22)**.

🔑 **Kural olarak yaz: ek ayar dosyaları bağımlılık EKLER, KALDIRAMAZ.**
`Wants=` / `After=` gibi `[Unit]` bağımlılık listeleri **toplamalıdır**; boş
atamayla sıfırlanmaz. Bir bağımlılığı kaldırmanın **tek yolu temel birim
dosyasını değiştirmektir.**

Bu, §0.43'te konan kuralın bir başka örneği: *bir servis ayarının kanıtı
repodaki dosya ya da kurulan ek dosya değil, `systemctl show <birim> -p <ayar>`
çıktısıdır.* Ek ayar dosyası **kuruluydu, doğru yazılmıştı, `DropInPaths`
listesinde görünüyordu** — ve hiçbir şey yapmıyordu.

### Neden `mask` da kullanılmadı

`systemctl mask girdap-algi` bu makinede çalışmaz: `mask`, `/dev/null`'a
sembolik bağ koymak ister, ama `/etc/systemd/system/girdap-algi.service`
**gerçek dosyadır** → `Failed to mask unit: File ... already exists`.
Gerçek `mask` için birim dosyasını taşımak gerekirdi; bunun bedeli ağır:
yarışma sabahı geri almayı unutmak = **algı hiç başlamaz**.

---

## 4. ✅ Uygulanan düzeltme

`/etc/systemd/system/girdap-karar.service` — `Wants=girdap-algi.service`
satırı **kapatıldı**, `After=` **duruyor** (sıra korunur, çekme kalkar):

```ini
After=girdap-livox.service
Wants=girdap-livox.service
After=girdap-algi.service
#Wants=girdap-algi.service      ← gerekçesi dosyanın içine yazıldı
```

Ölü ek ayar dosyası, dizinin **dışına** taşındı (silinmedi):

```
/etc/systemd/system/girdap-karar.service.d/99-veriseti-modu.conf
  → /etc/systemd/system/girdap-karar.service.d.99-veriseti-modu.conf.OLU_20260812
```

🔑 **Neden taşındı, neden yerinde bırakılmadı:** masaüstü kartı *"algı çekmesi
kesik mi, DOĞRULA: `ls .../99-veriseti-modu.conf`"* diyordu. Dosya durup hiçbir
şey yapmasaydı **bu doğrulama yalan söyleyecekti** — kıyıda "kesik" yazan bir
kontrolle denize girilirdi. Şimdi kontrol açıkça başarısız olur.

Temel birim dosyasının yedeği: `girdap-karar.service.yedek_20260812`.

**Düzeltme sonrası ölçüm** (16:18):

```
systemctl show girdap-karar -p Wants
  Wants=girdap-saat.service girdap-livox.service     ← algı YOK
```

⚠️ Bu sefer `daemon-reload` bağı **anında** düşürdü, çünkü algı o an
`inactive`'di. **Algı açıkken** aynı işlem yapılırsa bağ açılışa kadar listede
kalabilir. Bu yüzden `show -p Wants` tek başına yeterli kanıt değildir —
açılış davranışının kanıtı **yeniden başlatmadır** (6. bölüm).

---

## 5. Yarışma günü geri alma — İKİSİNDEN BİRİ YETER

```bash
# (a) satırı geri aç
sudo sed -i 's/^#Wants=girdap-algi.service/Wants=girdap-algi.service/' \
    /etc/systemd/system/girdap-karar.service
sudo systemctl daemon-reload

# (b) algıyı kendi ayakları üstünde açılışa bağla  — ZATEN prosedürde var
sudo systemctl enable --now girdap-algi
```

🔑 **Unutma bedeli YOK.** (b) `girdap-algi`'yi `multi-user.target.wants`
içine koyar; `[Install] WantedBy=multi-user.target` zaten dosyasında var.
Yani karar servisinin çekmesine hiç ihtiyaç kalmadan açılışta kalkar — ve (b)
yarışma günü prosedüründe **zaten yazılı**. Gerçek `mask` seçilseydi bu
güvenlik payı olmayacaktı; seçilmeme sebebi budur.

⚠️ İkisi birden yapılırsa da sorun çıkmaz (aynı çekme iki yoldan gelir).

---

## 6. Doğrulama — tek geçerli kanıt yeniden başlatmadır

Dosyaya bakmak, `is-enabled` okumak ve `show -p Wants` **yetmez**; üçü de
12.08'de yanıltıcı çıktı verdi. Kıyıda, denize girmeden:

```bash
sudo systemctl stop girdap-veriseti      # temiz kapat — fişi ÇEKME
sudo reboot

# açılış sonrası:
systemctl is-active girdap-veriseti girdap-algi   # active / inactive OLMALI
journalctl -b -u girdap-veriseti | head           # satır AKMALI
ls ~/girdap_veriseti/images | wc -l               # KALDIĞI YERDEN artmalı
systemctl show girdap-karar -p Wants              # girdap-algi GEÇMEMELİ
```

🔴 **Bu ölçüm bu dosya yazılırken HENÜZ YAPILMADI.** Düzeltme uygulandı ve
`show -p Wants` temiz çıktı, ama yeniden başlatma testi koşulmadı — toplayıcı
o sırada kare topluyordu. **Denize girmeden önce koşulacak.**

---

## 7. 🔴 `VERISETI-JETSON-KARTI.md`'nin GEÇERSİZ KALAN İDDİALARI

| yer | iddia | durum |
|---|---|---|
| bölüm 2 | `bash scripts/jetson_kur.sh --veriseti-servis` servisi kurar + açılışta başlatır | **ÇALIŞMIYOR** — §0.44d / §0.45d: kurulum betiğinin her iki servis yolu da kırık, var olmayan bir yoldan birim kopyalamaya çalışıyor |
| bölüm 2/2 | `girdap-algi`'yi `disable --now` etmek "tek OAK'ı iki süreç açamaz"ı garanti eder | **YETMİYOR** — `girdap-karar`'ın `Wants=` çekmesi `disable`'ı ezer (2. bölüm) |
| bölüm 8 | *"BOOT'ta hangisi açılsın"* → `jetson_kur.sh --veriseti-servis` / `--servis` | **İKİSİ DE ÇALIŞMIYOR**; açılış seçimi artık `girdap-karar.service`'teki `Wants=` satırıyla yapılır (4. ve 5. bölüm) |

Bölüm 8'in **o anı** değiştiren komutları (`start`/`stop girdap-veriseti`,
`Conflicts=` ve `OnSuccess=` devri) **geçerli** — 12.08'de canlı doğrulandı.
Geçersiz olan yalnızca **kalıcı açılış seçimi** kısmı.

---

## 8. Aynı oturumda çıkan İKİ AYRI VERİ BULGUSU

### 8a. 🔴 1.392 kare hiçbir yerde yok

`manifest.csv` kayıtlı ama diskte de, çöp kutusunda da bulunamayan kareler:

| küme | aralık | adet | durum |
|---|---|---|---|
| `images/` içinde duran | `kare_02433`+ | — | sağlam |
| çöp kutusunda | `kare_01393`–`kare_02432` | 1.045 | **kaptan bilerek sildiğini söyledi** — dokunulmadı |
| **hiçbir yerde yok** | `kare_00001`–`kare_01392` | **1.392** | ❓ **AÇIK SORU** |

Kayıp karelerin oturumları ve zaman aralığı (`manifest.csv`'den):

```
  16 kare  20260812_112200
 771 kare  20260812_112919
 327 kare  20260812_115714
 278 kare  20260812_120922
zaman aralığı: 11:22:06 → 12:18:59
```

Tüm dosya sistemi tarandı (`find / -name "kare_0*.jpg"`): yalnızca çöp kutusu
ve `images/` çıktı. Bu 1.392 kare **çöp kutusundan da geçmemiş** — yani
çöpe atılan 1.045 kareyle **aynı işlem değil**. Kaptanın başka bir yere
(dizüstü/USB bellek) aktarıp aktarmadığı **sorulmuş, cevap beklenmektedir**.

### 8b. 🔴 `manifest.csv`'de 330 baytlık NUL deliği — temiz olmayan kapanma izi

```
dosya boyu     : 166.975 bayt
NUL başlangıcı : 133.645. bayt
NUL uzunluğu   : 330 bayt  (≈ 5–6 satır okunamaz)

önce  : kare_02024.jpg, 20260812_150851, 2026-08-12T15:13:45, saat_guvenilir=0
sonra : kare_02025.jpg, 20260812_152618, 2026-08-12T15:26:22, saat_guvenilir=1
```

Bu, `ext4` **gecikmeli ayırma** (delayed allocation) izidir: dosyanın boyutu
güncellenmiş ama veri blokları diske inmeden güç kesilmiştir. Yani
**15:13:45 ile 15:26:22 arasında temiz olmayan bir kapanma yaşanmıştır.**

`VERISETI-JETSON-KARTI.md` bölüm 0'daki *"güç kesilir / kirli kapanış"* riski
**gerçekleşmiştir**. `.mp4` tarafında segment yazımı kaybı 2 dakikayla sınırlar
ama `manifest.csv` **tek ve sürekli** bir dosyadır — orada segment koruması
yoktur, delik kalıcıdır.

📌 **Öneri (henüz uygulanmadı, kaptan kararı):** toplayıcı her kare satırından
sonra `flush()` + `os.fsync()` çağırsın, ya da `manifest.csv` de oturum başına
ayrı dosyaya yazılsın. İkincisi diski daha az yorar.

### 8c. 🔴 Toplayıcının kapanış mesajı KLASÖRÜ SAYMIYOR — kaybı gizler

Temiz kapanışta basılan satır:

```
[✓] Bitti. Bu oturumda 227 kare kaydedildi, 4 benzer kare atlandı.
    Toplam klasörde ~2667 kare: /home/girdap/girdap_veriseti/images
```

Ölçüm:

```
toplayıcının dediği : ~2667
gerçek dosya sayısı :   235      ← ls images/*.jpg | wc -l
manifest satırı     :  2667
```

🔑 Mesaj *"klasörde"* diyor ama aslında **sayacı/`manifest.csv`'yi** okuyor;
klasörü hiç saymıyor. Yani 8a'daki 1.392 karelik kayıp bu satıra **hiç
yansımadı** — kaptan her kapanışta "2667 kare var" yazısını görüp içi rahat
edecekti. **Kaybı gizleyen tam da bu satır.**

📌 **Öneri (kaptan kararı):** satır ya gerçekten klasörü saysın
(`len(os.listdir(...))`), ya da ikisini birden yazsın ve **fark varsa uyarsın**:
`manifest 2667 / diskte 235 — 2432 KARE EKSİK`. İkincisi daha iyi: sessiz kayıp
kıyıda görünür hâle gelir (bölüm 0'daki "telafisiz" listesinin ruhu).

---

## 9. Bu turdan kalan

1. 🔴 **Yeniden başlatma testi koşulmadı** (6. bölüm) — denize girmeden önce.
2. 🔴 **1.392 karenin akıbeti belirsiz** (8a) — kaptan cevabı bekleniyor.
3. 🟡 **`manifest.csv` dayanıklılığı** (8b) — `fsync` ya da oturum başına dosya.
3b. 🔴 **Kapanış mesajı klasörü saymıyor** (8c) — sessiz kaybı gizliyor,
   düzeltilmesi ucuz ve etkisi büyük.
4. 🟡 **`jetson_kur.sh`'ın iki servis yolu hâlâ kırık** (§0.44d/§0.45d) —
   bu dosya onu çözmez, yalnızca kartın ona güvenmemesi gerektiğini söyler.
5. 🟡 Masaüstündeki `girdap_komutlar.txt` kartının bölüm 1'i (satır 39–62,
   *"NEDEN DROP-IN"*) bu dosyayla **çelişiyor** — güncellenecek.

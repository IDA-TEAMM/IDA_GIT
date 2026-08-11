# 📸 VERİ TOPLAMA KARTI — sıfırlanmış Jetson'ı göl/deniz oturumuna hazırlama

> Jetson **sıfırlandı**. Bu dosya, kartı sıfırdan veri toplamaya hazır hâle
> getirmenin tek doğru sırasıdır. Yarışma/algı kurulumu **ayrı**:
> `JETSON-KURULUM.md`. İkisi aynı anda kurulu **kalamaz** (tek OAK var).
>
> Kaynak: 2026-08-11 PC oturumu — kapanış sözleşmesi ölçümü + arşiv çözünürlüğü
> ölçümü. Her adımın yanında **neden** var.

---

## 0. Önce bil: bu oturumda neyin yanlış gitmesi telafisiz

| risk | sonuç |
|---|---|
| Servis boot'ta başlamaz | Tekne suya girer, **tek kare toplanmaz**, oturum bir daha kurulamaz |
| OAK'ı başka süreç tutar | Toplayıcı hiç kare alamaz — belirti "kamera arızası" gibi görünür |
| Güç kesilir / kirli kapanış | OAK kirli kapanır ⇒ sonraki açılışta USB takılır, **sahada fiziksel erişim yok** |
| Yanlış çözünürlük | Veri seti dağıtım geometrisine uymaz, eğitilen model sahada başka FOV görür |
| Saat yanlış | Kareler düne yazılır, zaman-bloğu bölmesi bozulur ⇒ **eğitim/valid sızıntısı** |

Sahada **PC yok, ekran yok, SSH yok**. Her şey kıyıda, denize girmeden doğrulanır.

---

## 1. Kodu getir ve kur

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/EyupEker1/girdap-ida-algi.git
cd ~/ros2_ws/src/girdap-ida-algi && git pull
bash scripts/jetson_kur.sh          # depthai'yi 2.30.0.0'a PİNLER
```

🔴 **depthai sürümü pazarlıksız 2.30.0.0.** v3 bu OAK-D Lite'ta stereo
üretmiyor (v3 %0 ↔ v2 **29,7 FPS**, 5/5 tekrarlanabilir). Kurucu sürümü kontrol
edip gerekirse düşürür; çıktıda `depthai 2.30.0.0 — doğru sürüm` görmelisin.

📌 Veri toplama için **ROS gerekmez** — toplayıcı düz Python. Ama `colcon build`
zaten kuruluysa zararı yok; algı aşamasına geçerken lazım olacak.

---

## 2. Toplayıcı servisini kur (açılışta OTOMATİK başlar)

```bash
bash scripts/jetson_kur.sh --veriseti-servis
```

Bu bayrak (11.08'de eklendi) üç şeyi birden yapar:
1. `girdap-veriseti.service`'i kurar ve **enable** eder → boot'ta kendi başlar,
2. `girdap-algi.service`'i **disable --now** eder → tek OAK'ı iki süreç açamaz,
3. yer tutucuları (`__USER__`, `__WS__`) doldurur.

🔑 **Neden servis, neden elle değil:** denizde dizüstüyle bağlanamayacağız.
Elle komut çalıştıramayacağımıza göre toplayıcı **açılışta kendi başlamalı**.

🔴 **"enable ettim" ≠ "boot'ta başladı".** Kanıtı adım 5'te.

### Açma/kapama sözleşmesi — nasıl çalışıyor

| olay | ne olur |
|---|---|
| **Güç ver / boot** | systemd `girdap-veriseti`'yi başlatır. USB geç enumere olursa `--cihaz-bekle 3600` ile **1 saat bekler**, pes etmez |
| Kamera sonradan takılır | Aynı bekleme yakalar, toplama başlar |
| Kamera kopar / süreç çöker | `Restart=on-failure` + `RestartSec=10` → yeniden dener; **indeks kaldığı yerden devam eder**, manifest eklemeli yazılır |
| Disk 10 GB'a iner | Temiz durur (`exit 0`) — systemd yeniden **başlatmaz**, sonsuz döngü olmaz |
| **Güç kes / kapat / reboot** | systemd SIGTERM → kod bunu `KeyboardInterrupt`'a çevirir → `finally` çalışır → **`dev.close()`** ile OAK temiz kapanır |

### 🔴 Kapanış neden ayrı bir iş — 11.08'de ÖLÇÜLDÜ

Python'un **SIGTERM varsayılanı süreci ANINDA öldürür, `finally` bloğu
ÇALIŞMAZ**. (Gerçek süreçle doğrulandı: SIGINT'te `finally` çalışıyordu,
SIGTERM'de çalışmıyordu. `rclpy` de yalnız SIGINT'e işleyici kuruyor.)
systemd `stop`/`reboot`/güç düğmesi **SIGTERM** gönderir.

⇒ Düzeltmeden önce her kapanışta `dev.close()` **hiç çağrılmıyordu**: OAK kirli
kapanıyordu — bu, [USB kilitlenmesinin](docs/) tam zemini ve **teknede fiziksel
erişim yok**. (Algı tarafında sonucu daha ağırdı: mp4'ün moov atomu yazılmıyordu
⇒ Dosya-1 oynatılamaz ⇒ **5 ceza puanı**, md 5.5.4.3.5.)

Düzeltme **iki kat**: koddaki SIGTERM işleyicisi + servis dosyasındaki
`KillSignal=SIGINT` / `TimeoutStopSec=20` / `SendSIGKILL=yes`.
Gerçek systemd unit'iyle üçü de ölçüldü — düzeltme öncesi kapanış kayboluyordu,
sonrasında her iki sinyalde de temiz kapanıyor.

---

## 3. 🔴 ÇÖZÜNÜRLÜK — `1352x1014` (512 DEĞİL)

Servis zaten böyle kuruluyor, **elle değiştirme.**

Eyüp'ün sorusu haklıydı: *"512 eğitiyorsak kamera zaten 512 görmüyor mu?"*
Cevap: **kamera aynı anda İKİ ayrı çıkış veriyor.**

```
IMX214 sensör 4056×3040 (12 MP)
        │
        └─ ISP  ── ispScale 1/3 ──►  1352×1014  ─┬─►  isp çıkışı   → TOPLAYICI BUNU KAYDEDER
                                                 │
                                                 └─►  preview 512×512 (EZİLEREK) → NN BUNU GÖRÜR
```

Model gerçekten 512 görüyor — ama arşiv `isp` dalından geliyor ve **oradan 512
her zaman türetilebilir; tersi asla.**

### Ölçüldü (kendi verimiz, 1451 kare, 3.084 kutu)

| mesafe bandı | genişlik px (1352 → 512) | kutu içi piksel | ton kayması |
|---|---|---|---|
| <10 m | 71,3 → 27,0 | 4920 → 952 | 0,1° |
| 10-20 m | 25,6 → 9,7 | 702 → 140 | 0,5° |
| **20+ m** | **14,8 → 5,6** | **240 → 48** | 0,9° |

**Renk kaybolmuyor** (medyan kayma 0,3°, %95'te 3,1°; sınıf karar sınırı 34-38°,
pay ~21°). Sorun renk değil, **bilgi ve etiketleme hassasiyeti**:

1. **Geri dönüşsüz.** 1352'den 512 üretiliyor (`arac/veri_512_uret.py` bunu
   yapıyor, etiketler normalize olduğu için aynen taşınıyor). 512'den 1352
   **üretilemez**. Bugün 416→512'ye geçtik; yarın 640 gerekirse 512 arşiv ölü.
2. **Elle etiketleme.** 20+ m'deki duba 512'de **5,6 px** — Eyüp bunları kendi
   çiziyor. 1 piksellik el hatası kutu genişliğinin **%18'i**. 1352'de aynı hata
   **%6,8**. Ve menzil yedeğimiz **bbox genişliğinden** hesaplıyor
   (`d = f·D/w`) ⇒ %18 genişlik hatası **doğrudan %18 mesafe hatası**:
   20 m'de **3,6 m sapma**.
3. **Yer sorunu yok.** ~0,25 MB/kare ⇒ 4.000 kare ≈ **0,92 GB**; diskte 418 GB.
   Kazanç sıfıra yakın, kayıp kalıcı.

📌 `--res 1440x1080` verilirse toplayıcı sessizce **1352x1014'e düşürür** —
`THE_1440X1080` sensör modu bu cihazda **hiç kare üretmiyor** (sessizce, hata
bile vermeden). Eski servis satırı bu yüzden zararsız, ama yeni kurulumda
doğrudan `1352x1014` yazılı.

---

## 4. Kıyıda, DENİZE GİRMEDEN — zorunlu kontrol listesi

```bash
# (a) OAK boşta mı — başka süreç tutuyorsa toplayıcı tek kare alamaz
lsusb | grep 03e7                                    # 03e7:2485 = bootloader, BOŞTA
ps aux | grep -iE "oakd|perception_camera|depthai" | grep -v grep    # BOŞ olmalı
systemctl is-enabled girdap-algi 2>/dev/null         # "disabled" ya da hata

# (b) 🔴 SAAT — bayat saat kareleri düne yazar, zaman-bloğu bölmesi bozulur
date                                                  # GÖZLE DOĞRULA, tarih+saat doğru mu
timedatectl set-time "2026-08-12 09:00:00"           # yanlışsa ELLE düzelt (sudo)

# (c) Servisi başlat ve kare aktığını GÖR
sudo systemctl start girdap-veriseti
journalctl -fu girdap-veriseti                        # "[+] N kare" satırları AKMALI
ls ~/girdap_veriseti/images | wc -l                   # sayı ARTMALI

# (d) 🔴 REBOOT TESTİ — atlanamaz
sudo reboot
# boot sonrası:
systemctl is-active girdap-veriseti                   # "active"
ls ~/girdap_veriseti/images | wc -l                   # sayı KALDIĞI YERDEN ARTMALI

# (e) Kapanış testi — mp4/manifest bütünlüğü
sudo systemctl stop girdap-veriseti
journalctl -u girdap-veriseti | tail -5               # "[✓] Bitti. N kare" GÖRMELİ
```

🔴 **(d) ve (e) görülmeden denize girilmez.** `enable` etmek boot'ta başladığının
kanıtı değildir; bu tam olarak 05.08'de yaşandı (servis `disabled`'dı ve o gün
denize girilse tek kare toplanmayacaktı).

---

## 5. Toplama ayarları — neden böyle (değiştirme, gerekçeleri ölçülü)

| ayar | değer | gerekçe |
|---|---|---|
| `--res` | `1352x1014` | Bölüm 3. Tam 4:3, kırpma yok, deploy FOV'u ile birebir |
| `--fps` | `10` | USB2'ye **bilerek** zorluyoruz (USB3 bu Jetson'da kararsız). 10 fps = 20,6 MB/s; 15 = 35,0 (tavanın kenarı), 20 = 46,7 **çöktüğü ölçüldü** |
| `--interval` | `2.0` | 2 sn'de en fazla 1 kare ⇒ 1,5 m/s'te kare başına ~3 m yol. Çeşitlilik kare **sayısından** değil kareler arası **mesafeden** gelir. 3.0 yapılırsa 8-15 m bandı temsilsiz kalır |
| `--min-fark` | `0.5` | Filtre **tüm karenin** ortalama farkına bakıyor; 10 m'deki 30 cm duba ortalamaya yalnız 0,11 katıyor. Eşik 2,0 iken **uzak duba kadraja girince kare eleniyordu** |
| `--zorunlu-aralik` | `10` | Kalp atışı: 10 sn'dir kayıt yoksa filtreyi aş. Filtre uzak dubaya kör olduğu için bu bir **sigorta**. 15-20 sn yapılırsa sigorta işlevini kaybeder |
| `--min-bos-gb` | `10` | Disk dolmadan temiz dur |
| `--cihaz-bekle` | `3600` | Sahada monitör/SSH yok; "bulamadım, pes ettim" kabul edilemez |

**Bilerek bol topla:** fazla kareyi atmak kolay, kaçan oturum geri gelmez.

---

## 6. Bu oturumda özellikle NE çekilecek

Modelin bilinen zayıf noktaları — hedefli çekim:

- 🔴 **15-25 m bandı** — en zayıf yerimiz (20+ m recall %62,7). Uzaktan yavaş yaklaş.
- 🔴 **Turuncunun sarıya yaklaştığı ışıklar** — asıl hata kaynağı bir **alan
  farkı**; tek tek denenen 9 etkenin hiçbiri tek başına açıklamıyor, çözüm
  **koşul çeşitliliği**. Aynı dubayı: güneşe karşı / gölgede / ıslak / kuru.
- **Turuncu ve sarıyı AYNI açı, AYNI mesafede** yan yana — sınıf ayrımı buradan öğrenilir.
- **Yarışma saati bandı** — 12:00-14:00 ışığı elimizde eksikti.

### 🔴 Yeni etiketleme kuralı (11.08'de netleşti)
> **Gövdenin en az yarısı görünüyorsa etiketle; yalnız bayrak/kulp görünüyorsa GEÇ.**

Eski veride 16 kutu sadece bayrak/parçaydı (kaçanların bir kısmı buradan).
Etiketleme hatası değil, **tanım boşluğu**ydu — artık yazılı.

Değişmeyen kurallar: **sudaki yansıma etiketlenmez** · **sarının iki tonu da
`engel_dubasi`** · **train/valid AYRI yüklenir**.

---

## 7. Oturumdan sonra

```bash
sudo systemctl stop girdap-veriseti
ls ~/girdap_veriseti/images | wc -l          # toplam kare
tail -3 ~/girdap_veriseti/manifest.csv       # son satır yazılmış olmalı
```

Kareleri PC'ye aktar → etiketle → `arac/veri_512_uret.py` ile 512'ye türet →
512 modelini yeniden eğit → blob üret (**`--reverse_input_channels` + 4 shave**)
→ kabul testi → `JETSON-KURULUM.md` ile yarışma kurulumuna geç.

🔴 **Yarışma günü toplayıcıyı geri al** (md 4.1, tek OAK):
```bash
bash scripts/jetson_kur.sh --servis     # veriseti'ni disable eder, algıyı enable eder
```

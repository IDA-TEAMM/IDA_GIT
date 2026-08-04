# Deniz oturumu — YOLO veri seti toplama (PC YOK, EKRAN YOK)

**Kısıt:** İDA suya girdiğinde dizüstüyle bağlanamayacağız, ekran yok, klavye yok.
Veri setini orada toplayacağız, dosyaları sonradan alacağız.
**Sonuç:** her şey kıyıda, denize girmeden ayarlanır ve DOĞRULANIR. Denizde tek
müdahale yok; sessiz başarısızlık = kaybedilmiş oturum (bir daha kurulamaz).

**Neden bu iş kritik:** algı katmanı şu an model dosyası olmadığı için hiç
açılmıyor (A-1). Model = veri seti. Veri seti = bu oturum.

---

## 0. Önkoşul: kamera TAKILI mı?

🔴 2026-08-04 itibarıyla ekip ölçüm formunda **kamera henüz takılı değil**
(`son_kodv2/karar/docs/olcum_formu.md §3` boş, `hardware.yaml` `oak_frame`
{0,0,0}, not: *"pruvadaki mavi kutu onun yuvası"*). Toplama oturumundan önce:

1. OAK-D Lite pruvadaki yuvasına **sabit** monte edilir (gevşek montaj = her
   karede farklı açı = veri seti tutarsız).
2. **Aynı gün ölçülür** (bu fırsat bir daha gelmeyebilir) — `olcum_formu.md §3`:
   `x` (ileri), `y` (iskele), `z` (yukarı), `yaw`, **`pitch`**.
   Pitch özellikle: duba mesafe tahminini doğrudan etkiler.
3. Ölçüm bize de lazım: `duba_gecis_navigator.py` `KAMERA_OFSET_ILERI` hâlâ
   ölçülmemiş bir tahmin (0,50 m). base_link 04.08'de gövde merkezine taşındı
   (ön uçtan 51,5 cm) → kamera pruvadaysa gerçek değer ≈ 0,515 m, ama **ölç**.
4. Lens temiz + tuz/su damlası yok (damla tüm oturumu bulanıklaştırır).

---

## 1. Kıyıda kurulum (denize girmeden — sırayla)

```bash
# 1) Saat doğru mu? (RTC pilsiz Jetson boot'ta ~2 ay geride açılabilir)
date                       # yanlışsa: sudo date -s "2026-08-04 15:30:00"
                           # yanlış saat -> manifest'e saat_guvenilir=0 yazılır,
                           # ışık/saat çeşitliliği analizi yapılamaz

# 2) Servisi kur (bir kez yeterli)
sudo cp ~/ros2_ws/src/girdap-ida-algi/scripts/girdap-veriseti.service /etc/systemd/system/
sudo sed -i "s|__USER__|$USER|g; s|__WS__|$HOME/ros2_ws|g" /etc/systemd/system/girdap-veriseti.service
sudo systemctl daemon-reload

# 3) 🔴 KAMERA DEVRİ: tek OAK var, iki süreç açamaz
sudo systemctl disable --now girdap-algi
sudo systemctl enable  --now girdap-veriseti

# 4) DOĞRULA (bu adımı ATLAMA — denizde bir daha şansın yok)
journalctl -fu girdap-veriseti      # "[+] N kare -> kare_000NN.jpg" AKMALI
ls ~/girdap_veriseti/images | wc -l # sayı ARTMALI (30 sn bekle, tekrar bak)
tail -3 ~/girdap_veriseti/manifest.csv   # saat + oturum doğru mu?
df -h ~                             # ≥ 20 GB boş olsun (~1,5 GB/saat)

# 5) Yeniden başlatma testi: gerçekten AÇILIŞTA başlıyor mu?
sudo reboot                         # boot sonrası (4) tekrar kontrol
```

⚠️ (5) atlanırsa "servis enable edildi ama boot'ta açılmadı" hatası denizde
ortaya çıkar ve fark edilmez.

---

## 2. Denizde ne toplanmalı (çeşitlilik = modelin sahada çalışması)

Toplayıcı zaten benzer kareleri eler (`--min-fark`), ama **çeşitliliği manevra
üretir** — dümen sizde:

| Boyut | Ne yapılmalı | Neden |
|---|---|---|
| **Mesafe** | Dubaya 1-2 m'den ~15 m'ye kadar yaklaş/uzaklaş | Deploy menzili ~6-8 m (hesap); model uzak/küçük dubayı da görmeli |
| **Açı** | Dubanın etrafında dolan; geçide düz, çapraz ve yandan yanaş | Sahada duba her açıdan görünür |
| **Işık** | **Farklı saatlerde** koş (öğle + akşamüstü şart) | Ekip ölçümü: akşamüstü ışığında gerçek dubanın doygunluğu S≈29-83 — sabit eşikli HSV hiç görmedi. YOLO da bu ışığı eğitimde görmezse zorlanır |
| **Yön** | Güneşe karşı ve güneş arkada | Su parlaması (glare) en zor kare |
| **Deniz** | Dalgalı ve sakin | Duba batıp çıkar, kısmen görünür |
| **Negatif** | Beyaz sosis duba, **kırmızı/yeşil/kahverengi** nesneler, kıyı, tekne, kuş, şamandıra | Ekip 17.07'de parkurda kırmızı/yeşil/kahverengi nesne olduğunu tespit etti. Model bunları "kenar dubası" sanarsa yanlış geçit kurulur |
| **Kısmi** | Duba karenin kenarında yarısı kesik | Geçide girerken tam bu görüntü oluşur |

**Duba yoksa oturum yarım kalır:** en az bir turuncu (RAL 2003) ve bir sarı
(RAL 1026) armut duba suda olmalı. Yoksa toplanan kareler yalnız negatif örnek
olur — işe yarar ama model eğitilemez.

**Bonus ölçüm (aynı gün, bedava):** dubayı bilinen mesafelere koyup kare çek →
`GECIT_MAX_MESAFE=8.0 m` şu an **hesap, ölçüm değil**; bu kareler onu doğrular.

---

## 3. Dönüşte

```bash
sudo systemctl disable --now girdap-veriseti     # kamerayı algıya geri ver
sudo systemctl enable  --now girdap-algi

# Veriyi al (Jetson -> PC)
rsync -av girdap@<jetson-ip>:~/girdap_veriseti/ ./girdap_veriseti/
```

Kontrol:
- `manifest.csv` satır sayısı ≈ `images/` dosya sayısı olmalı.
- `saat_guvenilir` sütunu 1 mi? 0 ise o oturumun zaman etiketleri kullanılamaz.
- Oturum sütunundan hangi koşunun hangi ışıkta olduğu çıkarılır.

Sonra: etiketleme (`README.txt` — Roboflow/CVAT/labelImg), **sınıf sırası
KİLİTLİ**: `0=kenar_dubasi`, `1=engel_dubasi`. Bu sıra `/perception/buoys`
sözleşmesiyle aynı ve algı kodu sınıfı isimden çözüyor — bozulursa model
sessizce yanlış sınıf yayınlar.

---

## 4. Bilinen sınırlar (dürüst)

- **Denizde doğrulama yok.** Ekran/PC olmadığı için kayıt akıyor mu göremeyiz;
  tek güvence kıyıdaki (4)+(5) adımları.
- `--min-fark 2.0` **saha ölçümüyle ayarlanmadı**. Dalga kıpırtısı bu eşiği tek
  başına aşabilir (her kare kaydedilir) ya da tekne dururken her şey elenebilir.
  İlk oturumdan sonra `manifest.csv` + atlanan sayısına bakıp ayarlanacak.
  Şimdilik bilerek BOL toplama tarafına ayarlı.
- Toplayıcı ile algı node'u **aynı anda çalışamaz** (tek OAK). Servis dosyasında
  `Conflicts=` var ama asıl güvence yukarıdaki devir sırası.
- Toplanan kareler **4:3** (1440×1080), deploy da 4:3 (640×480 → 416×416
  letterbox). 16:9 istenirse sensör kırpılır ve veri seti deploy'a uymaz —
  `--res` değiştirilmemeli.

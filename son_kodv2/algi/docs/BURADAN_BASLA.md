# BURADAN BAŞLA — Jetson Günü Tam Rehberi

> Tek dosyada: **neyi nereye takacağın** + **terminale sırayla ne yazacağın**
> (her komutun ne işe yaradığıyla) + hangi rehber ne zaman açılır.
> Derin detay gerektiğinde ilgili rehbere bağlantı var; ama bu dosyayı
> baştan sona takip etmek masa gününü çıkarmaya yeter.
>
> ⚠️ ALTIN KURAL: masa testlerinde **PERVANELER SÖKÜLÜ**. Arm/KILL testleri
> motorlara gerçek komut basar.

---

## 1. NEYİ NEREYE TAKIYORUZ (fiziksel bağlantı)

```
   [monitör]──HDMI──┐                          ┌──USB──[RFD868x #2]
   [klavye ]──USB───┤                          │
                    │                    [YKİ LAPTOPU]
   [ethernet]───────┤                     (QGC + OBS)
   (kurulum için)   │                          ~~~~ 868 MHz hava ~~~~
                    │                          │
   [güç adaptörü]───JETSON ORIN NANO      [RFD868x #1]
                    │        │                 │ (JST-GH kablo)
        USB3 (MAVİ) │        │ USB          TELEM1
                    │        │                 │
              [OAK-D Lite] [PIXHAWK 6C]────────┘
                             │
                       (ESC/thruster — FC ekibi; masa testinde pervane YOK)
```

| Ne | Nereye | Nasıl / dikkat | Doğrulama |
|---|---|---|---|
| Jetson güç | Kutusundan çıkan adaptör → DC girişi | Önce her şeyi tak, gücü EN SON ver | Yeşil led, açılış ekranı |
| Monitör+klavye | HDMI/DP + USB | İlk kurulumda şart; sonra SSH yeter | Masaüstü görünür |
| Ethernet | Jetson RJ45 → modem/router | **Kurulum interneti için önerilen** (WiFi, kur scripti sonunda kapanır) | `ping 8.8.8.8` |
| **OAK-D Lite** | Jetson **USB3 = MAVİ port** | Data hatlı USB3 kablo (ince şarj kablosu OLMAZ — görüntü gelmez/FPS düşer) | `lsusb \| grep 03e7` |
| **Pixhawk 6C** | Jetson herhangi USB | Pixhawk'ın USB-C portu → Jetson | `ls /dev/ttyACM*` → ACM0 |
| **RFD868x #1** | Pixhawk **TELEM1** portu | JST-GH kablo (FC ekibi bağlamış olabilir — sor) | RFD led'leri yanar |
| **RFD868x #2** | YKİ laptopu USB | QGC bu porttan bağlanır (57600) | QGC'de telemetri |
| Livox Mid-360 | — **BUGÜN TAKMA** — | Yarışma fazı (T1) işi: ethernet + 9-27V güç + sürücü kurulumu ayrı rehberle gelecek | — |

Sıra önerisi: kabloları tak → Jetson'a güç ver → oturum aç → §2'ye geç.

---

## 2. TERMİNALE NE YAZACAKSIN (sırayla, açıklamalı)

Terminal = klavyeden `Ctrl+Alt+T`. Her bloğu sırayla kopyala-yapıştır;
üstündeki cümle o bloğun NE İŞE YARADIĞI.

### 2.1 GitHub kimliği — repolarımız private, kimliksiz klon çalışmaz

gh aracını kur (GitHub'ın resmî terminal aracı):
```bash
sudo mkdir -p -m 755 /etc/apt/keyrings
wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update && sudo apt install gh -y
```
Giriş yap — 8 haneli kodu **telefonda** `github.com/login/device`'a gireceksin:
```bash
gh auth login        # GitHub.com → HTTPS → Login with a web browser
gh auth setup-git    # bundan sonra git clone'lar otomatik kimlikli
```
Kontrol: `gh repo view EyupEker1/girdap-ida-algi` repo özetini basmalı.

### 2.2 ROS 2 Humble — tüm sistemin iskeleti (tek seferlik, ~15 dk)

⚠️ ÖNCE KONTROL: bazı Jetson imajlarında ROS deposu ZATEN ekli.
```bash
grep -rl packages.ros.org /etc/apt/sources.list.d/ 2>/dev/null
```
Bir dosya listeliyorsa aşağıdaki bloğun ilk 3 satırını (depo ekleme) ATLA,
yalnız son satırı (`sudo apt update && sudo apt install ...`) çalıştır —
yoksa "Conflicting values set for option Signed-By" hatası alırsın (§5/6).

```bash
sudo apt install software-properties-common curl -y && sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y
```
Kontrol: `source /opt/ros/humble/setup.bash && ros2 --help` hata vermemeli.

### 2.3 Bizim yazılımlar — klonla + kur scripti gerisini halleder

Kur scripti: doğru sürümleri kurar (numpy<2 ŞART!), OAK'a USB izni yazar,
İKİ repoyu da indirir (kamera + karar), derler, terminal ayarlarını yapar:
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
gh repo clone EyupEker1/girdap-ida-algi
bash girdap-ida-algi/scripts/jetson_kur.sh
```
Kontrol: sonda `KURULUM TAMAM`; **yeni terminal açıp** `ros2 pkg list | grep girdap`
→ iki paket görünmeli. (Script en son WiFi'yi kapatır — şartname gereği;
internet lazımsa geçici `sudo rfkill unblock wifi`.)

### 2.4 Karar yığını ekleri — Pixhawk köprüsü + matematik kütüphaneleri

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ffmpeg
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
python3 -m pip install --user --pre gtsam "numpy>=1.26,<2"
python3 -m pip install --user "scipy>=1.11,<1.14" "matplotlib>=3.8" pillow "numpy>=1.26,<2"
sudo usermod -aG dialout $USER      # Pixhawk USB izni — sonra ÇIKIŞ yap, gir
```

### 2.5 Her şey yerinde mi? — iki kontrol komutu

Ortam denetimi (yeşil/kırmızı liste basar; model satırı hariç hepsi YEŞİL olmalı):
```bash
bash ~/ros2_ws/src/girdap-ida-algi/scripts/jetson_kontrol.sh
```
Karar yazılımının 246 otomatik testi (5 dk; YEŞİL olmadan donanıma geçme):
```bash
cd ~/ros2_ws/src/girdap-decision && python3 -m pytest prototype/tests/ -q
```
Beklenen: `246 passed, 1 skipped`.

### 2.6 Kamera ilk görüntü — model olmadan (GEÇİCİ test)

OAK takılıyken (udev kuralı yeni yazıldıysa bir kez ÇIKAR-TAK):
```bash
python3 ~/ros2_ws/src/girdap-ida-algi/scripts/kamera_goruntu_test.py
```
Pencerede canlı görüntü + ~11 FPS görmelisin; `q` ile çık. (Bu script
geçici — model gelince silinecek; tespitli asıl test o gün `duba_kamera_test.py`.)

### 2.7 Ana yazılımı çalıştırmak

Karar yığını (MAVROS + görev + kontrol — Pixhawk takılıyken):
```bash
ros2 launch girdap_decision hardware.launch.py mission_source:=fc
```
Kamera yığını (model geldikten sonra, ayrı terminalde):
```bash
ros2 launch girdap_ida_algi algi.launch.py
```
Durdurmak: o terminalde `Ctrl+C`.
Görev nasıl yüklenir/başlatılır/durdurulur → **§2.9** (uçtan uca akış).

### 2.7b İki yığın birlikte çalışır — çakışma yok (neden?)

Kamera ve karar (MPPI) yazılımları **aynı anda** koşar; tasarım bu.
Başlatma sırası da önemli değil — ROS2'de kanallar birbirini bekler.

- **İşlemci çakışmaz:** YOLO Jetson'da DEĞİL, kameranın kendi çipinde (VPU)
  koşar; MPPI Jetson CPU'sunda. Kamera açıkken MPPI yavaşlamaz, tersi de olmaz.
- **Cihaz çakışmaz:** OAK'ı açan TEK süreç bizim düğümümüz (karar reposundaki
  eski HSV kamera düğümü bilerek kapalı — denetim F3.1 düzeltmesi `eb9ff58`).
  Pixhawk ayrı USB'de, RFD Pixhawk'ta — herkes kendi kapısında.
- **Konuşma anlaşmalı:** kamera `/perception/buoys` + `/perception/gate_passed`
  yayınlar, karar dinler; bbox uzayı (**1280×720** — 04.08'de 640×480'den
  çıkarıldı, E-1) ve sınıf kimlikleri iki tarafta birebir doğrulandı (F4.6).
- **Biri çökerse öbürü yaşar:** ayrı süreçler. Kamera düğümü çökerse launch
  onu 3 sn'de kendisi yeniden başlatır (respawn); karar yığını etkilenmez.
- **Video gününde kamera yığını HİÇ açılmaz** (videoda algı yok) — çakışma
  ihtimali sıfır.
- Açık kalan masa/saha teyidi (kod değil): kamera-LiDAR zaman damgası hizası
  (bozuksa yazılım kendisi WARN basar — F7.1 bekçisi). Model sonrası dönemin işi.
- Yarışma günü otomatik başlatma: kamera için systemd hazır
  (`jetson_kur.sh --servis`); karar yığınına benzer servis masa testlerinden
  sonra yazılacak (test döneminde elle başlatmak daha iyi — loglar görünür).

### 2.8 Mini ROS2 sözlüğü — "içeride ne oluyor"a bakmak

| Komut | Ne işe yarar |
|---|---|
| `ros2 topic list` | Akan tüm veri kanallarını listeler |
| `ros2 topic echo /mavros/state --once` | Bir kanaldan TEK mesaj göster (bağlı mı, armed mı) |
| `ros2 topic echo /girdap/mission/state` | FSM durumunu canlı izle (BOOT/ARM/BEKLEMEDE/PARKUR1…) |
| `ros2 topic hz /perception/buoys` | Kanalın saniyedeki mesaj sayısı (FPS gibi) |
| `ros2 service call /girdap/mission/kill std_srvs/srv/Trigger {}` | Acil durdurma komutu gönder |
| `ros2 node list` | Çalışan yazılım parçalarını listeler |

### 2.9 GÖREV KOŞUSU — uçtan uca (görev yükle → başlat → durdur)

> Kısa sürüm; her adımın PASS/FAIL kriterli hâli masa runbook'unda (M4-M6).
> ⚠️ Masada pervaneler SÖKÜLÜ; arm/başlat komutları motorlara gerçek komut basar.

**1) Yığını fc modunda başlat** (görevi QGC'den alacak — şartname md 3.3.1(2)):
```bash
ros2 launch girdap_decision hardware.launch.py mission_source:=fc
```
Logda `mission_source=fc — /mavros/mission/waypoints bekleniyor` görünmeli.

**2) İkinci terminalde durumu izle** (tüm koşu boyunca açık kalsın):
```bash
ros2 topic echo /girdap/mission/state
```

**3) QGC'den görevi yükle:** Plan ekranı → 4 köşe + kapanış noktası → **Upload**.
Jetson logunda: `FC görevi alındı: N item → M waypoint`. Görev, başlatmadan
ÖNCE yüklenmeli — sonra yüklenirse reddedilir (bilerek, md 5.5.2.2).

**4) ARM et:** QGC'den Arm (araç MANUAL/HOLD modundayken).
İzlediğin durum: `BOOT` → `ARM` → `BEKLEMEDE`. Arm etmek BAŞLATMAZ — güvenlik.

**5) BAŞLAT: QGC'den modu GUIDED'a çevir.** Tetik bu (md 3.3.1(3), `start_on_mode`).
- Logda: `YKİ mod komutu (…→GUIDED) — görev başlatıldı` · durum `PARKUR1` ·
  `ros2 topic hz /girdap/control/thrust` ~10 Hz.
- ⚠️ KENAR TETİKLİ: araç boot'ta ZATEN GUIDED'daysa çalışmaz — modu bir kez
  HOLD/MANUAL'a alıp GUIDED'a geri dön. Sıra hep: önce ARM, sonra GUIDED.

**6) Görev sonu:** son noktaya varınca durum `TAMAMLANDI`, thrust sıfırlanır,
araç süzülerek durur. Sonrası manuel dönüş serbest (mod savaşı yapmaz).

**7) Durdurma yolları:**

| Ne istiyorsun | Komut | Not |
|---|---|---|
| ACİL DURDUR | `ros2 service call /girdap/mission/kill std_srvs/srv/Trigger {}` | motor durur + FCU disarm; KALICI — devam için yığını yeniden başlat |
| Kontrollü disarm | `ros2 service call /girdap/bridge/disarm std_srvs/srv/Trigger {}` | görev sonu güç-kesme provası; failsafe sayılmaz |
| Yığını kapat | launch terminalinde `Ctrl+C` | normal kapanış |

### 2.10 KAYIT DOSYALARI + Ekran-2 grafiği — koşudan sonra

Koşu bittiğinde her şey `~/girdap_logs/` altında birikmiş olur:

| Dizin | İçerik | Kime lazım |
|---|---|---|
| `~/girdap_logs/telemetry/` | Dosya-2 telemetri CSV (yarışma zorunlu teslimi) | hakem USB'si |
| `~/girdap_logs/grafik/` | Ekran-2 ham verisi (10 Hz: hız+sp, heading+sp, thrust) | video montajı |
| `~/girdap_logs/local_map/` | Dosya-3 harita PNG serisi (yarışma zorunlu teslimi) | hakem USB'si |
| `~/girdap_logs/viz/` | `run_ekran2` çıktıları (PNG/MP4) | video montajı |

Ekran-2 panelini üret (en yeni grafik CSV'sini kendisi bulur):
```bash
python3 ~/ros2_ws/src/girdap-decision/scripts/run_ekran2.py            # hızlı bakış: PNG
python3 ~/ros2_ws/src/girdap-decision/scripts/run_ekran2.py --mp4 --t0 30 --t1 150   # video için: zaman imleçli MP4, 30-150 sn arası kırpılmış
```
PNG'de üç panel de DOLU olmalı (hız, heading, thrust) — thrust paneli boşsa
koşuda MPPI hiç thrust basmamış demektir, logu kontrol et.

---

## 3. SIRADA NE VAR — test günü akışı

Kurulum bittiyse artık test protokolüne geç (komutları + PASS kriterleri hazır):

**`~/ros2_ws/src/girdap-decision/docs/masa_testi_runbook.md`** → M0'dan M8'e
sırayla. Kabaca: M0 yazılım ✓ → M1 Pixhawk ✓ → M2 QGC/radyo ✓ → M3 FSM ✓ →
M4 görev yükleme ✓ → **M5 QGC'den GUIDED'a alınca görev başlıyor mu** ✓ →
M6 acil durdurma ✓ → M7 kayıt dosyaları ✓ → M8 CUDA hız ölçümü (bonus).

Hepsi PASS → suda prova → video çekimi (`video_gunu_runbook.md`).

---

## 4. REHBER HARİTASI — hangi dosya ne zaman

| Dosya | Ne zaman açılır |
|---|---|
| **BU DOSYA** | Jetson'u ilk kurarken / kabloları takarken |
| `jetson_kurulum_rehberi.md` | Kurulumda takıldığında (ayrıntı + sorun giderme tablosu) |
| girdap-decision `masa_testi_runbook.md` | Kurulumdan sonra, masa test günü (M0-M8) |
| girdap-decision `video_gunu_runbook.md` | Suda prova + çekim günü |
| `hubai_model_rehberi.md` | Video SONRASI — YOLO modelini üretirken (PC'de) |
| `olcum_formu.md` | Mekanik/FC ekibine GÖNDER (Livox yüksekliği vs.) |
| `kod_disi_ihtiyaclar.md` | Alışveriş/eksik kontrolü — kod hariç ne lazım |
| `bekleyen_girdiler.md` | Tüm açık işlerin ana defteri |

## 5. En sık 5 hata (30 saniyelik çözümler)

1. **`git clone` şifre soruyor** → §2.1 atlanmış: `gh auth login`.
2. **Kamera penceresi açılmıyor / 03e7 yok** → mavi USB3 porta tak, data
   kablosu kullan, çıkar-tak.
3. **`ros2: command not found`** → yeni terminal aç (ayarlar bashrc'de).
4. **Pixhawk `/dev/ttyACM0` izin hatası** → §2.4'teki dialout + oturumu kapat-aç.
5. **scipy/numpy `_ARRAY_API` hatası** → biri numpy 2 kurmuş:
   `python3 -m pip install --user --force-reinstall "numpy>=1.26,<2"`.
6. **`apt update`: "Conflicting values set for option Signed-By ... ros2"**
   → ROS deposu imajda zaten ekliydi, §2.2 ikinci kayıt oluşturdu (Jetson'da
   2026-07-11'de yaşandı): `sudo rm /etc/apt/sources.list.d/ros2.list &&
   sudo apt update` — sonra kuruluma devam.
7. **`gh auth login` tarayıcı akışı "context deadline exceeded"** → telefonla
   uğraşma; PC'de `gh auth token` çıktısını (BAŞINDAKİ `gho_` DAHİL) kopyala,
   Jetson'da: `echo "gho_..." | gh auth login --with-token` (başarıda sessizdir;
   `gh auth status` ile teyit). Ardından `gh auth setup-git` ŞART — yoksa
   `git pull` private repoya "Repository not found" der.
8. **Testlerde `TypeError: ... unexpected keyword argument` / eksik test sayısı**
   → Jetson'da ESKİ bir kurulum (örn. `~/girdap_ws`) bizim workspace'i
   gölgeliyor olabilir (2026-07-11'de yaşandı: arkadaşın denetim-ÖNCESİ kodu
   import ediliyordu). Teşhis: `python3 -c "import girdap_decision.fsm_node
   as m; print(m.__file__)"` — yol `~/ros2_ws` DEĞİLSE: bashrc'den eski
   satırları sil (`sed -i.yedek '/girdap_ws/d' ~/.bashrc`), eski workspace'i
   kenara al (`mv ~/girdap_ws ~/girdap_ws.eski`), YENİ terminalde tekrar dene.
   ⚠ Eski kod düzeltilmiş bugları içermez — teknede asla o koşmamalı.
9. **pip: opencv-python-headless numpy 2'yi geri getiriyor** → 4.12+ ve 5.x
   numpy>=2 dayatır; numpy<2'ye izin veren SON sürüm **4.11.0.86** (PyPI
   metadata'dan teyitli; Jetson'da 2026-07-11 iki kez yaşandı — `<5` pini
   yetmedi, 4.13 de numpy'ı 2.2.6'ya yükseltti). İkisini TEK komutta sabitle:
   `python3 -m pip install --user --force-reinstall opencv-python-headless==4.11.0.86 numpy==1.26.4`
   → kontrol: `python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"`
   → `4.11.0 | 1.26.4` görülmeli.
10. **OAK çalışıyor ama USB2'de (UsbSpeed.HIGH) — YOLO+stereo yükü için
    USB3 (SUPER) şart** → kabloyu OAK tarafından çıkar, **USB-C ucunu 180°
    ÇEVİRİP** tak (Jetson'da 2026-07-11'de yaşandı: SS-işaretli sağlam
    kablo + USB3.2 port olmasına rağmen HIGH kaldı; C-ucu ters çevrilince
    SUPER oldu — SuperSpeed hatları tek yönde temas kuruyordu). Teyit:
    pipeline start sonrası `pipeline.getDefaultDevice().getUsbSpeed()` →
    `SUPER` görülmeli. Suya çıkmadan/muhafazaya almadan önce MUTLAKA
    kontrol et: yanlış yönde takılı kablo sessizce USB2'de çalışır,
    belirti yalnız FPS düşüşü/kuyruk gecikmesi olur.

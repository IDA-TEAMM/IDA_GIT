# Jetson Kurulum Rehberi — Sıfırdan Çalışır Yığına

> Hedef: JetPack 6 yüklü Jetson Orin Nano Super'a HEM algı (bu repo) HEM
> karar (girdap-decision fork) yığınını kurmak, doğrulamak ve kamera kodunu
> çalıştırmak. Sırayla git; her adımın "doğrulama" satırı var.
>
> Süre tahmini: ~1 saat (ROS apt indirmesi hariç). İnternet: kurulum
> sırasında **ethernet önerilir** (kur scripti sonda WiFi'yi şartname 4.1
> gereği kapatır; WiFi ile kurduysan o adım seni düşürmez ama sonrasında
> internet için tekrar `sudo rfkill unblock wifi` gerekir — yarışma sabahı
> tekrar KAPAT).
>
> Yazım: 2026-07-11. Komutlar jetson_kur.sh/jetson_kontrol.sh güncel haliyle uyumlu.

## 0. Başlamadan: elinde ne olmalı

- Jetson Orin Nano Super, **JetPack 6** (Ubuntu 22.04) flash'lı, açılıyor.
- Monitör+klavye YA DA aynı ağda SSH (`ssh <kullanıcı>@<jetson-ip>`).
- Ethernet kablosu (önerilen) ya da WiFi ile internet.
- GitHub hesabın (EyupEker1) — telefon yeter (cihaz doğrulaması için).
- OAK-D Lite + USB3 kablosu (kamera testi için).
- ⚠️ Kod, model dosyasını `/home/girdap/models/` altında arar
  (`duba_gecis_navigator.py:97`). Jetson kullanıcı adını **`girdap`**
  yapmadıysan ya kullanıcıyı öyle aç ya da o satırı kendi yoluna güncelle
  (tek satır). Model zaten şimdilik YOK (aşağıda §6) — acil değil.

## 1. GitHub erişimi (repolar PRIVATE — bu adım atlanamaz)

`github.com/EyupEker1/girdap-ida-algi` ve `github.com/EyupEker1/girdap-decision`
private; kimlik doğrulamadan `git clone` ÇALIŞMAZ. En kolayı gh CLI:

```bash
sudo mkdir -p -m 755 /etc/apt/keyrings
wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg \
  | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
  | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update && sudo apt install gh -y

gh auth login
#  → GitHub.com  → HTTPS  → "Login with a web browser"
#  → ekrandaki 8 haneli kodu TELEFONDA github.com/login/device adresine gir.
gh auth setup-git        # git clone'lar artık otomatik kimlikli
```

**Doğrulama:** `gh repo view EyupEker1/girdap-ida-algi` repo özetini basmalı.

> Alternatif (gh istemezsen): github.com → Settings → Developer settings →
> Personal access token (repo yetkili) üret; `git clone` şifre sorduğunda
> token'ı yapıştır.

## 2. ROS 2 Humble (tek seferlik, ~15 dk)

```bash
sudo apt install software-properties-common curl -y && sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y
```

**Doğrulama:** `source /opt/ros/humble/setup.bash && ros2 --help` çalışır.

## 3. Algı + karar yığını kurulumu (scriptli)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
gh repo clone EyupEker1/girdap-ida-algi
bash girdap-ida-algi/scripts/jetson_kur.sh
```

Script sırayla: apt paketleri → **numpy<2 + depthai>=3.6** (sürüm kilitli —
numpy 2.x KURMA, ROS/scipy ABI'sini kırar) → OAK udev kuralı → iki repoyu
`~/ros2_ws/src/` altına klonlar (karar yığını **bizim fork'tan** gelir,
tüm düzeltmelerle) → `colcon build` → `~/.bashrc`'ye source+PYTHONPATH
satırları → WiFi/BT'yi kapatır (şartname 4.1).

**Doğrulama:** script sonunda `KURULUM TAMAM` + yeni terminal açınca
`ros2 pkg list | grep girdap` iki paketi (girdap_ida_algi, girdap_decision)
göstermeli.

## 4. Karar yığını bağımlılıkları (video için ŞART)

Kur scripti algı tarafını kilitler; karar yığınının ek bağımlılıkları:

```bash
# MAVROS (Pixhawk köprüsü) + mesaj paketleri:
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Python (numpy<2 ile uyumlu sürümler — numpy'yi YÜKSELTME):
python3 -m pip install --user --pre gtsam "numpy>=1.26,<2"
python3 -m pip install --user "scipy>=1.11,<1.14" "matplotlib>=3.8" pillow "numpy>=1.26,<2"

# Ekran-2 MP4 üretimi (video montaj aracı) için:
sudo apt install -y ffmpeg

# Seri port izni (Pixhawk USB):
sudo usermod -aG dialout $USER    # sonra ÇIKIŞ yapıp tekrar gir
```

**Doğrulama (karar yığını test paketi):**
```bash
cd ~/ros2_ws/src/girdap-decision
python3 -m pytest prototype/tests/ -q
```
Beklenen: **246 passed, 1 skipped** (skip = cupy; CUDA Faz A'da koşacak).
Kırık import görürsen eksik paket o satırda yazar.

## 5. Ortam denetimi (tek komut)

```bash
bash ~/ros2_ws/src/girdap-ida-algi/scripts/jetson_kontrol.sh
```
Yeşil/kırmızı PASS-FAIL listesi basar (L4T, ROS, numpy<2, depthai, udev,
WiFi kapalı, model dosyası...). Çıktıyı fotoğrafla — uyuşmazlıkta bana
olduğu gibi gönder. Model satırı hariç her şey YEŞİL olmalı (model §6).

## 6. Kamera kodunu çalıştırma (senin repo)

### Bugünkü gerçek: model dosyası olmadan YOLO düğümü AÇILMAZ

Kod `MODEL_BLOB = /home/girdap/models/yolo11n_duba_rvc2.blob` bekler
(🔴 05.08.2026 v2 geçişi: NNArchive değil **blob + yanında config.json**;
NNArchive tar.xz'den `tar -xJf` ile çıkarılır — `models/README.md`);
bu dosya git'te YOK (bilinçli) ve **henüz üretilmedi** — adım adım:
`docs/hubai_model_rehberi.md`. Video için algıya gerek yok.

**Modelsiz dönemde kamerayı YİNE DE test edebilirsin (GEÇİCİ script):**
```bash
python3 ~/ros2_ws/src/girdap-ida-algi/scripts/kamera_goruntu_test.py
```
Yalnız görüntü + FPS (YOLO yok) — kamera/USB/udev zincirini bugün doğrular.
🗑️ Model geldiği gün bu script SİLİNECEK (bekleyen_girdiler §B/5) —
yarışma yazılımının parçası DEĞİL, başlığında da yazıyor.

Model elinde olduğunda:
```bash
mkdir -p /home/girdap/models
# USB bellekle İKİ dosyayı taşı (WiFi yok):
#   /home/girdap/models/yolo11n_duba_rvc2.blob
#   /home/girdap/models/config.json          <- sınıf isimleri buradan okunuyor
```
🔴 **`.tar.xz` (NN Archive) taşıma** — Jetson'daki depthai **2.30.0.0** NN Archive
ve superblob **okuyamaz**. Arşiv geldiyse PC'de aç, içinden düz `.blob` +
`config.json` çıkar. Blob **4 shave**'e derlenmiş olmalı; taşımadan önce
PC'de doğrula: `dai.OpenVINO.Blob(yol).numShaves` → **4**.
Ayrıntı: [`hubai_model_rehberi.md`](hubai_model_rehberi.md) §4.

### 6a. Masa testi (ROS'suz, görüntülü) — İLK BUNU KOŞ

OAK-D Lite'ı USB3 porta tak (udev kuralı yeni yazıldıysa ÇIKAR-TAK):
```bash
lsusb | grep 03e7          # Movidius görünmeli
python3 ~/ros2_ws/src/girdap-ida-algi/scripts/duba_kamera_test.py
```
Ekranda canlı görüntü + tespit kutuları + sınıf + mesafe + bearing.
**Burada iki şeyi MUTLAKA doğrula (sahaya çıkmadan zorunlu):**
1. Terminaldeki `Model sınıf sırası: [...]` logu — kod sınıfları isimden
   çözer ama sırayı GÖZLE teyit et (Gazebonew.pt'de sıra TERSTİ!).
2. FPS 10-14 bandında mı (VPU doymuş normali ~11.6).

### 6b. ROS düğümü (yarışma kipi)

```bash
ros2 launch girdap_ida_algi algi.launch.py     # respawn'lı, önerilen
```
Varsayılan `MOD="algi_yayin"`: hedef/hız komutu BASMAZ, yalnız yayınlar:
```bash
ros2 topic echo /perception/buoys --once       # tespitler (1280×720 bbox uzayı)
ros2 topic echo /perception/gate_passed        # geçit sayacı
ros2 topic hz   /perception/buoys              # ~10-14 Hz beklenir
```
Karar yığınıyla birlikte: önce onun launch'ı, sonra algı — ikisi aynı
`~/ros2_ws`'ten source'lanır, ek ayar gerekmez (topic sözleşmesi hazır).

### 6c. Açılışta otomatik başlatma (yarışma günü, masa testinde GEREKMEZ)

```bash
bash ~/ros2_ws/src/girdap-ida-algi/scripts/jetson_kur.sh --servis
sudo systemctl start girdap-algi && journalctl -fu girdap-algi
```

## 7. Sonraki adım: masa testleri

Kurulum bitti → karar yığını masa testlerine geç:
**`~/ros2_ws/src/girdap-decision/docs/masa_testi_runbook.md`** (M0-M8:
MAVROS, QGC/RFD, görev yükleme, GUIDED başlatma tetiği, KILL, telemetri,
CUDA). Terminal hazırlığı artık otomatik (`~/.bashrc` kur scriptince dolduruldu).

## 8. Sorun giderme

| Belirti | Neden / Çözüm |
|---|---|
| `git clone` şifre soruyor / 403 | §1 atlanmış — `gh auth login` + `gh auth setup-git` |
| `lsusb`'de 03e7 yok | Kabloyu USB3 (mavi) porta tak; udev sonrası çıkar-tak; kabloyu değiştir (data hatlı olmalı) |
| depthai `X_LINK_DEVICE_NOT_FOUND` | udev kuralı + çıkar-tak; `dmesg | tail` ile USB düşmesine bak |
| `_ARRAY_API not found` / scipy ImportError | Birisi numpy 2.x kurmuş: `python3 -m pip install --user --force-reinstall "numpy>=1.26,<2"` |
| `ros2 pkg list`'te girdap yok | Yeni terminal aç (bashrc) ya da `source ~/ros2_ws/install/setup.bash` |
| mavros `/dev/ttyACM0` izin hatası | `dialout` grubu (§4) + oturumu kapat-aç |
| pytest'te mavros_msgs ImportError | `sudo apt install ros-humble-mavros-msgs` + yeni terminal |
| İnternet gitti (kurulum ortası) | Kur scripti WiFi'yi kapattı: `sudo rfkill unblock wifi` — işin bitince TEKRAR `block` |

# 🚀 JETSON KURULUM / DAĞITIM KARTI — algı

> Bu dosya **teknedeki Jetson'a** algı katmanını kurmanın tek doğru sırasıdır.
> Her adımın yanında **neden** var; gerekçesi olmayan adım yok.
> Kaynak: 2026-08-10 PC oturumunda yapılan tam dağıtım denetimi + web araştırması.

## 0. Önce bil: neyin yanlış gitmesi puanı götürür
Algı düşerse `/perception/buoys` akmaz → füzyon tüm LiDAR kümelerini
`CLASS_UNKNOWN` bırakır → `gate_follower` **ham GPS'e düşer** →
**P1 (G1/KD1 ≥ 0,5) ve P2 (≥2 ikili) gider** — ve bunların **hiçbiri hata basmaz**.
Aşağıdaki adımların tamamı bu tek sonucu engellemek içindir.

---

## 1. Kodu ve modeli getir
```bash
cd ~/ros2_ws/src/girdap-ida-algi
git pull                       # blob + config.json ARTIK REPODA (cb1773d)
```
🔑 Model dosyaları `.gitignore` istisnasıyla repoda: yarışma alanında **internet
yok** (md 4.1) ve `blobconverter` **bulut** ⇒ blob sahada üretilemez.

## 2. Modeli koddaki sabit yola kopyala
```bash
mkdir -p ~/models
cp models/yolo11n_duba_rvc2.blob models/config.json ~/models/
sha256sum ~/models/yolo11n_duba_rvc2.blob
#   beklenen: 5726819a101eb4f62dd8ad65cdd302f980b349c0fa448190264d412031871b9c
```
`duba_gecis_navigator.py:132` yolu **sabit** (`/home/girdap/models/...`).
`config.json` **yanında olmak zorunda** — sınıflar oradan **isimle** çözülüyor;
yoksa yedek sabite düşer ve turuncu↔sarı **sessizce takas** olabilir (⇒ Ç2).
📌 `jetson_kur.sh` bu kopyalamayı artık kendisi de yapıyor (adım 4).

## 3. Paketi derle
```bash
cd ~/ros2_ws && colcon build --packages-select girdap_ida_algi
```
Servis `ros2 launch girdap_ida_algi algi.launch.py` ile kalkıyor ⇒ paket
`install/` altında **derlenmiş olmalı**, yoksa servis başlamaz.

## 4. Kurulum + servis
```bash
bash src/girdap-ida-algi/scripts/jetson_kur.sh --servis
bash src/girdap-ida-algi/scripts/jetson_kontrol.sh      # HEPSİ YEŞİL olmalı
```
`jetson_kur.sh` depthai'yi **2.30.0.0'a pinler** (v3 bu OAK-D Lite'ta stereo
üretmiyor: v3 %0 ↔ v2 29,7 FPS).

## 5. 🔴🔴 DDS DOMAIN — en sinsi hata
```bash
grep ROS_DOMAIN_ID /etc/systemd/system/girdap-algi.service   # => 42 görmeli
grep ROS_DOMAIN_ID /etc/systemd/system/girdap-karar.service  # => 42 (karar tarafı)
```
Karar yığını `ROS_DOMAIN_ID=42` ile koşuyor. Bizde bu satır **yoktu** (10.08'de
eklendi) ⇒ domain 0'da kalıyorduk: iki taraf da "çalışıyor" görünür ama
**birbirini hiç görmez**. `bash -lc` kurtarmaz — Ubuntu `~/.bashrc`'si
etkileşimsiz kabukta ilk satırda `return` eder ("systemd .bashrc okumaz",
karar tarafının 2026-07-13 bulgusu).
⚠️ **Değer iki tarafta AYNI olmalı.** Onlar değiştirirse burası da değişir.

## 6. Tek OAK — veri seti servisi KALINTISINI temizle
Toplayıcı 2026-08-16'da repodan kaldırıldı (bkz. `KAYNAK.md`), ama **eski
kurulumlarda unit hâlâ diskte olabilir**. Boot'ta toplayıcı kamerayı önce
kaparsa algı node'u **hiç açılamaz** ⇒ P1+P2 = 0, belirtisiz (md 4.1).

```bash
systemctl list-unit-files | grep veriseti      # 🔴 BOŞ dönmeli
# doluysa:
sudo systemctl disable --now girdap-veriseti
sudo rm /etc/systemd/system/girdap-veriseti.service
sudo systemctl daemon-reload
systemctl list-unit-files | grep veriseti      # tekrar bak — artık boş
```
🔴 **Yarışma günü pazarlıksız.** `disable` boot davranışını, `stop` çalışan
süreci değiştirir — karıştırma; `rm` ise ikisini de kalıcı olarak bitirir.

## 7. Journal'ı kalıcı yap (sahada tek teşhis kanalımız)
```bash
sudo mkdir -p /var/log/journal && sudo systemctl restart systemd-journald
```
06.08 göl oturumunda toplayıcının bastığı hata **reboot'ta buhar oldu**; sahada
SSH yok, `durum_log` tek görünürlük kanalı.

## 8. USB tamponu (kaptanın uyarısı + NVIDIA forumu)
```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb    # varsayılan 16
```
Derinlik AÇIK + 12MP kullanıyoruz. 16 MB düşük kalırsa USB write timeout gelir.
Gerekirse: `sudo sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'`
(kalıcı için kernel komut satırı). ⚠️ Ölçmeden değiştirme — önce sorun var mı bak.

## 9. 🔴 REBOOT TESTİ (atlanamaz)
```bash
sudo reboot
# geldikten sonra:
systemctl is-active girdap-algi
journalctl -u girdap-algi -b | grep -E "hazır|isimle|FPS"
```
Beklenen satırlar:
```
OAK-D Lite hazır — YOLO VPU'da. MOD = algi_yayin, USB = HIGH
Model sınıf sırası: [...] → kenar=0, engel=1 — isimle çözüldü
[algi_yayin|ARAMA] ... NN ~11 FPS
```
*"enable ettim" ≠ "boot'ta başladı"* — 05.08 dersi.

## 10. Akış doğrulaması (iki taraf birlikte)
```bash
ROS_DOMAIN_ID=42 ros2 topic hz /perception/buoys                 # ~11 Hz
ROS_DOMAIN_ID=42 ros2 topic hz /perception/classified_obstacles  # ← ASIL KANIT
```
İkincisi akıyorsa algı **gerçekten** karar tarafına ulaşıyor demektir.
PC'de ölçülen: `/perception/buoys` 10,6-10,9 Hz · `buoys_3d` 10,9-11,2 Hz.

## 10b. 🔴 TEK YAYINCI — `/perception/buoys`'a başka kimse basmıyor mu
```bash
ROS_DOMAIN_ID=42 ros2 topic info /perception/buoys     # Publisher count: 1 OLMALI
grep -n use_onboard_camera .../hardware.launch.py      # default_value="false"
grep -n with_oak_driver    .../hardware.launch.py      # default_value="false"
```
**Neden:** karar tarafında `perception_camera_node` da `/perception/buoys`'a
basabiliyor (HSV + eskiden `best.pt`). İki yayıncı olursa sarı engel "kenar
dubası" sanılabilir → yanlış kapı → **Ç2 çarpma**, belirti vermeden.

✅ **Bugün ölçüldü, risk ÇİFT KİLİTLİ:** (1) `girdap-karar.service` yalnız
`mission_source:=fc` geçiyor ⇒ `use_onboard_camera` varsayılan **false**, node
açılmıyor; (2) açılsa bile `/oak/rgb/image_raw`'a abone ve o topic'i yalnız
`oakd_driver_node` yayınlar — o da `with_oak_driver` **false** arkasında, üstelik
kamerayı biz tuttuğumuz için sürücü açamaz.
📌 `best.pt` 10.08'de depodan kaldırıldı (Sude onayı) ama **asıl koruma bayraklar**.
`Publisher count: 1` çıktısı bunun **çalışırken** kanıtıdır — tek gerçek kontrol bu.

## 11. Dosya-1 (md 4.2) — GERÇEKTEN oynuyor mu
```bash
ls -la ~/girdap_logs/kamera/session_*/
python3 -c "import cv2,sys; c=cv2.VideoCapture(sys.argv[1]); \
print('kare', int(c.get(7)), 'acilabiliyor', c.read()[0])" ~/girdap_logs/kamera/session_*/seg_0001.mp4
```
🟡 **Neden şart:** JetPack'in OpenCV derlemesi mp4v kodeğinde eksik olabiliyor —
dosya oluşur, `isOpened()` bile True döner, ama **oynatılamaz**. Geçersiz Dosya-1
= **5 ceza puanı** (md 5.5.4.3.5). PC'de doğrulandı (226 kare @2 fps); **Jetson'da
ayrıca bakılacak.**

---

## ⚠️ Bilinen ve KAPANMAMIŞ riskler
| risk | durum |
|---|---|
| **X_LINK hatası çalışma anında** → node ölüyor | `dayanikli_ac()` yalnız **açılışta**; döngüde kurtarma **yok**. systemd 10 sn'de kaldırır ⇒ ~10 sn körlük |
| **Cihaz sert kilidi** (USB'den kaybolur) | Yazılımla kurtarma **yok** → güç kes-ver. 10.08'de PC'de yaşandı |
| Kamera ileri ofseti | `KAMERA_OFSET_ILERI=0,50` **şüpheli**, CAD ~0,26 ⇒ mezurayla ölçülecek |
| Duba çapı | Hiç ölçülmedi (şartname 0,30 varsayılıyor) |

## 📌 Deploy öncesi altın kural
**Cihazı gereksiz aç/kapa yapma.** 10.08'de ~10 kez açılıp kapandıktan sonra OAK
sert kilide girdi ve yalnız fiziksel çıkar-tak ile döndü. Sahaya çıkmadan önce
**tek temiz açılış** yeterlidir.

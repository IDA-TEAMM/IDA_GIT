# girdap-decision — Kod İnceleme Bulguları (algı tarafından)

> **Kapsam:** [vistastris/girdap-decision](https://github.com/vistastris/girdap-decision)
> deposu **salt okunur** klonlanıp incelendi. İncelenen commit: `d4ce88b`
> ("feat(viz): offline 2D matplotlib gorsellestici (Sprint 4.5)") — bu, kontrol
> anında `origin/main` ile birebir aynıydı, yani bulgular günceldir.
>
> **Biz o depoya HİÇBİR ŞEY push etmiyoruz.** Aşağıdaki düzeltmelerin tamamı
> karar yazılımının sahibinin (takım arkadaşı) kendi ağacında, kendi eliyle
> yapacağı değişikliklerdir. Bu doküman yalnızca bulguların kaydı ve ona
> iletilecek metnin kaynağıdır.
>
> **Neden bizim depoda düzeltemiyoruz:** Beş bulgunun beşi de onun ağacındaki
> dosyalarda. Bizim `girdap-ida-algi` paketimizde karşılığı olan bir satır yok.
> Bkz. §6 — "bizim tarafta telafi etme" seçeneği neden zararlı.

Öncelik sırası, mevcut odağımıza (**Parkur-1 + Parkur-2**) göre verilmiştir.

---

## 1. 🔴 P2-KRİTİK — `/perception/buoys` konusunda çift yayıncı

**Dosyalar:**
- `ros2_ws/src/girdap_decision/girdap_decision/perception_camera_node.py:93`
  → `Detection2DArray` yayıncısı, topic `"/perception/buoys"`
- `ros2_ws/src/girdap_decision/launch/hardware.launch.py:314-315`
  → `perception_camera_node` **gerçek donanım launch'ında aktif**

**Kanıt:** `perception_camera_node` kendi docstring'ine göre HSV segmentasyon +
**MOCK YOLO** (`perception_camera_node.py:12` — "YOLO katmanı MOCK modda (gerçek
.pt yok — sabit test bbox'ı döner)"; `:62` — `yolo_model_path` boşsa mock).
Bu node `hardware.launch.py`'de koşulsuz başlatılıyor.

**Etki:** Gerçek teknede bizim `duba_gecis_navigator` (`MOD="algi_yayin"`) ile
onun mock kamera node'u **aynı topic'e birlikte yayın yapar**. ROS 2'de aynı
topic'te iki yayıncı olması hata değildir — abone (`perception_fusion_node.py:87`)
ikisinin mesajlarını da alır, araya karışmış hâlde. Sonuç: sabit mock bbox'ları
gerçek duba tespitleriyle iç içe geçer. Parkur-2'de duba sınıflandırması
güvenilmez hâle gelir.

**Düzeltme (onun tarafında, tek satır):** gerçek donanım launch'ında mock kamera
node'unu başlatmamak — ya `hardware.launch.py:314` satırındaki `Node(...)`'u
bir `use_mock_camera` LaunchArgument koşuluna bağlamak, ya da OAK node'u
koştuğunda o node'u hiç açmamak.

**Bu, P1+P2 için onun ağacındaki en kritik entegrasyon riskidir.**

---

## 2. 🟠 KURULUM — `numpy` üst sınırı yok

**Dosya:** `requirements.txt:1` → `numpy>=1.26`

**Etki:** Jetson'da temiz kurulumda pip `numpy 2.x` çeker. ROS Humble'ın ve
sistem `scipy`'sinin derlendiği ABI numpy 1.x'tir → çalışma anında
`_ARRAY_API not found` ile patlar.

**Düzeltme:** `numpy>=1.26,<2`

**Not:** Kendisi bu riski `docs/jetson_deployment.md` içinde **yazmış** —
yani farkında; sadece `requirements.txt`'e pinlemeyi atlamış. Bizim tarafta
pin zaten uygulanmış durumda (`README` sürüm tablosu + `scripts/jetson_kur.sh`).

---

## 3. 🟡 GERÇEK AMA ÖLÜ KOL — kamera bearing işaret hatası

**Dosya:** `prototype/perception/fusion.py`

```python
# :73  LiDAR — REP-103 (x ileri, y SOL pozitif)
def bearing_from_lidar(det):  return math.atan2(det.y, det.x)
#     → SOLDAKİ nesne  →  POZİTİF bearing

# :83  Kamera — bbox_cx normalize [0=sol kenar, 1=sağ kenar]
def bearing_from_camera(det, hfov):  return (det.bbox_cx - 0.5) * hfov
#     → SAĞDAKİ nesne  →  POZİTİF bearing     ❌ ters
```

İki sensörün işaret kuralı zıt. `associate()` (`fusion.py:110`) bearing farkına
`bearing_tolerance_rad = 0.15` (≈8.6°) eşiği uyguladığı için, merkezden birkaç
derece sapan her duba **eşleşmez**.

**Neden 92 saf-NumPy testi bunu yakalamıyor:** `prototype/perception/synthetic_fusion.py:27`
içindeki `_cx_for_bearing()` = `0.5 + bearing/hfov`, yani `bearing_from_camera`'nın
**tam tersi**. Sentetik kamera tespitleri aynı (hatalı) kuralla üretildiği için
hata sadeleşiyor ve testler yeşil kalıyor. Hata ancak gerçek kamerayla görünür.

**Bugünkü etkisi: YOK.** Çünkü füzyon çıktısı hiçbir yere gitmiyor:
- `perception_fusion_node.py:79` → `/perception/classified_obstacles` yayınlar
- Bu topic'in **hiçbir abonesi yok** (tüm `ros2_ws` tarandı)
- `planning_node.py:115-117` engel girdisini **ham LiDAR'dan** alıyor:
  `/perception/obstacle_map` (`PoseArray`)

Yani füzyon şu an boşluğa yayınlıyor. Bearing hatası, `classified_obstacles`
planlayıcıya bağlanana kadar hiçbir davranışı bozmuyor. **P1/P2 için acil değil**
— ama bağlandığı gün sessizce bozacağı için şimdiden düzeltilmeli.

**Düzeltme (tek satır):**
```python
return (0.5 - det.bbox_cx) * hfov
```
`synthetic_fusion.py:27`'deki ters fonksiyon da birlikte güncellenmeli
(`0.5 - bearing/hfov`), aksi hâlde testler yine hatayı maskeler.

Kendi docstring'i (`fusion.py:8-14`) zaten "gerçek donanımda sol/sağ ters
çıkarsa `bearing_from_camera` içindeki işareti çevirmek yeterli" diye uyarıyor.
Biz sahaya çıkmadan, masa başında yakaladık.

---

## 4. 🟡 DOKÜMAN — CLAUDE.md'deki RAL kodları şartnameye aykırı

**Dosya:** `CLAUDE.md:321-322`

| | Onun CLAUDE.md'si | Şartname (md 5.5.2.1, s.18) |
|---|---|---|
| Parkur kenarı (turuncu) | RAL **2008** ❌ | RAL **2003** ✅ |
| Engel (sarı) | RAL **1003** ❌ | RAL **1026** ✅ |

**Etki:** Doğrudan kod etkisi yok (renk eşiği HSV'de, RAL kodu yalnız dokümanda).
Ancak HSV eşikleri bu yanlış renklere göre elle ayarlandıysa dolaylı etkisi olur.
Bizim koddaki RAL kodları şartnameyle uyumlu.

---

## 5. ⚪ BİLGİ — MPPI `obstacle_margin` teknemize dar olabilir

**Dosya:** `prototype/planning/mppi.py:88` → `obstacle_margin: float = 0.5  # m`
(YAML'de override edilmiyor; `config/*.yaml` içinde geçmiyor.)

Tekne genişliği 0.75 m. Duba yarıçapı + 0.5 m marj, geçit açıklığında
teknenin yarı genişliğinden (0.375 m) sonra ~0.125 m paya bırakıyor. Deniz
Durumu-2'de sürüklenme payıyla birlikte temas riski var — çarpma cezası
P2 puan formülünde doğrudan yer alıyor. Saha testinde ölçülmeli.

---

## 6. Neden bu düzeltmeleri "bizim tarafta telafi" ETMİYORUZ

Bearing işaret hatası için akla gelen kestirme yol, bizim yayınladığımız
`bbox_cx`'i baştan aynalamak (`640 - cx`) — böylece onun hatalı formülü doğru
bearing üretirdi. **Bu aktif olarak zararlı:**

1. Bizim `bbox`'ımız **piksel uzayında** (`1280×720`, E-1 sonrası) ve **Dosya-1 zorunlu
   overlay videosunu** besliyor. Aynalanmış kutular görüntüdeki dubaların
   üstüne oturmaz → şartname md 4.2'ye aykırı çıktı, eksik dosya başına
   **5 ceza puanı**.
2. `/perception/buoys_3d` stereo konumlarını da bozar.
3. Yazdığımız sözleşme dokümanıyla (`mppi_entegrasyon_notu.md`) çelişir.
4. O işaretini düzelttiği gün, çifte çevirme yüzünden her şey **tekrar** bozulur
   — üstelik sessizce.

Bir başkasının hatasını sessizce telafi etmek, klasik entegrasyon felaketidir.
Hata sahibinin ağacında, açıkça düzeltilir.

Aynı şekilde, onun ~11.5k satırlık yığınını bizim depoya **kopyalamıyoruz**:
`rrt_star.py` ve `isam2_smoother.py` gibi hiç okumadığımız dosyaların bakımını
üstlenmek, o aktif geliştirme yaparken (son commit'i Sprint 4.5) iki ayrık
kopya oluşturmak ve bunu otonomi videosu teslimine **11 gün kala** yapmak
kabul edilebilir bir risk değil.

---

## 7. Arkadaşa iletilecek özet

Öncelik sırasıyla:

1. **`hardware.launch.py:314` — mock kamera node'unu gerçek donanımda kapat.**
   OAK node'umuzla aynı `/perception/buoys` topic'ine yayın yapıyor. P2'yi bozar.
2. **`requirements.txt:1` — `numpy>=1.26,<2`.** Jetson'da ROS/scipy ABI kırılması.
3. **`fusion.py:83` — `(0.5 - det.bbox_cx) * hfov`** (+ `synthetic_fusion.py:27`
   ters fonksiyonu). Bugün ölü kol ama `classified_obstacles` bağlanınca bozar.
4. **`CLAUDE.md:321-322` — RAL 2003 (turuncu) / RAL 1026 (sarı).**
5. **`mppi.py:88` — `obstacle_margin=0.5`** teknemize (0.75 m en) dar olabilir,
   saha testinde ölçelim.

Ayrıca §6'daki sorular (odom frekans/çerçeve, hfov teyidi, `buoys_3d` isteniyor mu)
`mppi_entegrasyon_notu.md` §6'da duruyor.

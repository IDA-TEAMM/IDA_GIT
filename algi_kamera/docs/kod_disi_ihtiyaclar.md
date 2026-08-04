# Kod Dışında Ne Lazım? — Tek Sayfa

> Yazılım hazır; bu liste "kod hariç" eksikleri toplar. Ayrıntılı takip:
> [`bekleyen_girdiler.md`](bekleyen_girdiler.md). Öncelik: 🔴 video (21.07)
> · 🟠 yarışma (30 Eylül) · 🟡 rahatlık.

## 1. Dosyalar & hesaplar

| Ne | Neden | Durum |
|---|---|---|
| 🟠 NN Archive (`yolo11n_duba_rvc2.tar.xz`) | YOLO'nun VPU'da koşan hali — kamera tespiti onsuz YOK | Üretilecek: [`hubai_model_rehberi.md`](hubai_model_rehberi.md) (video sonrası) |
| 🟠 `Gazebonew.pt` YEDEĞİ (harici disk + bulut) | Tek eğitilmiş model, tek kopya — kaybı telafisiz | ⚠️ hâlâ aynı diskte |
| 🔴 GitHub erişimi Jetson'da | Repolar private | `gh auth login` (kurulum rehberi §1) |
| 🟠 HubAI hesabı + API key | Model çevirme | 5 dk, ücretsiz |
| 🔴 YouTube hesabı (liste dışı video) + KYS erişimi | Video teslimi — link sorunu = ELEME | teyit et |

## 2. Donanım — video günü (21.07 öncesi)

| Ne | Not |
|---|---|
| 🔴 Jetson + Pixhawk 6C + USB kablo | masa testi M1 |
| 🔴 RFD868x ÇİFTİ (araç + YKİ laptop) | tek kablosuz kanal — WiFi yasak |
| 🔴 YKİ laptopu: QGC + ekran kayıt (OBS) kurulu | Ekran-1 zorunlu |
| 🔴 RC kumanda — **bandı 2.4 GHz OLMAMALI** (md 4.1) | bandı ETİKETİNDEN teyit et |
| 🔴 Fiziksel güvenlik anahtarı (kırmızı, güç kesen) | videoda gösterilecek (md 3.3.1/4) |
| 🔴 Dış kamera ≥720p + tripod + dolu pil/kart | Ekran-3 |
| 🔴 Tekne: kapaklar su almıyor | videoda gösterilecek (md 3.3.1/5) |
| 🟡 Yedek pervane/sigorta, USB hub, uzatma | saha kanunu: bozulur |

## 3. Donanım — yarışma tarafı (video'dan sonra)

| Ne | Not |
|---|---|
| 🟠 OAK-D Lite + USB3 kablosu (data hatlı) | kamera; USB2 kablo FPS düşürür |
| 🟠 Livox Mid-360 + montaj | LiDAR engel haritası |
| 🟠 Kamera yağmur muhafazası + arkasında ODAK testi | mekanik ekip (bekleyen A4) |
| 🟠 USB bellek (boş, FAT32) | yarışma çıktı teslimi: 20 dk / 5 ceza |

## 4. Ölçümler & bilgiler (birinden İSTENECEK)

| Ne | Kimden | Bloke ettiği |
|---|---|---|
| 🟠 Livox montaj yüksekliği `h` (su hattından) | mekanik | F5.1 — bilinmeden Parkur-2 sahaya çıkmaz |
| 🟠 `base_link` orijini (su hattı/güverte/IMU?) | mekanik+FC | tüm geometri (bekleyen A2) |
| 🔴 ArduPilot failsafe paramları (FS_GCS/FS_THR/FS_ACTION) | FC ekibi | KILL zincirinin FCU ayağı (masa M6) |
| 🟠 `Gazebonew.pt` neyle eğitildi (Gazebo mu saha mı?) | modeli eğiten | saha performansı bilinmiyor |
| 🔴 Video çekim yeri (göl/havuz izni) + 2. kişi (kamera) | takım | tek kişiyle çekim zor |

## 5. Sahaya çıkmadan 3 zorunlu soru (değişmedi)

1. Livox yüksekliği `h`? 2. Jetson'daki arşivin sınıf sırası? 3. base_link nerede?
Cevapsızsa Parkur-2 denenmez — ayrıntı [`bekleyen_girdiler.md`](bekleyen_girdiler.md).

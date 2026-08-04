# algi_kamera — GİRDAP algı (görüntü işleme) katmanı

**Kaynak repo:** github.com/EyupEker1/girdap-ida-algi — commit `333f903`
**Kopya tarihi:** 2026-08-04
**Sorumlu:** Eyüp (görüntü işleme)

## Bu katman ne yapar
OAK-D Lite'ın **içindeki** Myriad X VPU'da YOLOv11n (416×416) koşturur; Jetson
CPU'suna tespit maliyeti ~sıfırdır. Aynı pipeline'da stereo derinlik gelir.
Yayınlar:

| Topic | Tip | Anlamı |
|---|---|---|
| `/perception/buoys` | `vision_msgs/Detection2DArray` | bbox (640×480 piksel uzayı), class_id "0"=kenar "1"=engel |
| `/perception/buoys_3d` | `geometry_msgs/PoseArray` | stereo 3D duba konumu (bonus) |
| `/perception/gate_count` | `std_msgs/Int32` | geçilen **farklı** geçit sayısı |
| `/perception/gate_target` | `geometry_msgs/PoseStamped` | geçidin ötesindeki hedef (karar tarafı isterse kullanır) |

Ayrıca **Dosya-1** (md 4.2) mp4 kaydını üretir: bbox + sınıf overlay'li,
her karesi zaman etiketli, ≥1 Hz.

## ⚠️ `/perception/gate_passed` bilerek KAPALI
`GATE_PASSED_YAYINLA = False`. Sebep: `fsm_node._on_gate_passed` gelen
**herhangi** bir True'yu `last_gate_passed_p2` yapıyor, `mission_fsm` bunu
görünce PARKUR2 → PARKUR3 (kamikaze) geçiyor. Yani her geçitte basılan sinyal
Parkur-2'yi **ilk geçitte** bitirir: P2 tamamlanmaz (md 5.5.2.4 "en az 2 duba
ikilisi + son görev noktası"), (G2/KD2)×40 gider, ödül sıralaması (en az P1+P2)
kaybedilir. Algı hangi geçidin **sonuncu** olduğunu bilemez (KD çalışma anında
bilinmiyor; şartname "duba sayılarına göre akış tasarlanmaması" diyor).
**Karar tarafında P2→P3 geçişi waypoint ilerlemesinden sürülürse** bu bayrak
True yapılabilir. Yerine dürüst sinyal: `/perception/gate_count`.

## Testler (donanım GEREKMEZ)
```bash
cd algi_kamera && python3 -m pytest girdap_ida_algi/test/ -q   # 39 passed
```

## Çalıştırma
```bash
ros2 launch girdap_ida_algi algi.launch.py
```
Gereken: `~/models/yolo11n_duba_rvc2.tar.xz` (NN Archive) — **henüz üretilmedi**,
veri seti toplama + eğitim sürüyor.

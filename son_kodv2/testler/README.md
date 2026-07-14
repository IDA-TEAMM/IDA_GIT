# testler/ — video zinciri bileşen doğrulaması

Çekimden önce her bileşenin **ayrı ayrı** çalıştığını kanıtlamak için tek komut:

```bash
bash testler/video_testleri.sh          # bu repodaki karar/ kopyasına karşı
bash testler/video_testleri.sh --tam    # + sonda tam suite
```

**Jetson'da** (video kurulum düzeni AYRI klon kullanır — bu repo Jetson'a
kurulmaz; betiği USB/scp ile taşı ya da repoyu sadece bunun için çek):

```bash
GIRDAP_KOD=~/ros2_ws/src/girdap-decision bash testler/video_testleri.sh --tam
```

## Bileşen sırası = video veri akışı

| # | Bileşen | Neyi kesinleştirir |
|---|---|---|
| 1 | csv_logger çekirdeği | Dosya-2 (md 4.2) + grafik CSV header/format sözleşmeleri |
| 2 | telemetry_node | yon_setpoint = AÇI (F-V.1, md 3.3.1.1 Ekran-2b) + görev-dışı setpoint boş (F-V.2) + thrust/hız yedeği |
| 3 | görev çekirdeği | 4-nokta DİKDÖRTGEN senaryosu (md 3.3.1(2)+(3)) + fc dönüşüm filtreleri + F-M.1 mesafe tavanı |
| 4 | mission_manager_node | QGC fc yolu: latched WaypointList, skip_home, başladıktan-sonra-red (md 5.5.2.2), fix'siz başlamama |
| 5 | FSM çekirdeği | mission_complete → TAMAMLANDI terminali (F12.2), parkur katmanı |
| 6 | fsm_node | GUIDED kenar tetiği (md 3.3.1(3), T0-j), sahte P1→P2 yok (F12.1) |
| 7 | MAVROS köprüsü | KILL→disarm (F14.1), kasıtlı disarm ≠ FAILSAFE (F-M.2), auto_guided görev-aktif (F14.3) |
| 8 | MPPI | warm-start korunumu (F11.1/F9.1 — istemsiz hareket kriteri), backend parite |
| 9 | planning | bypass düz-hedef, replan çökmezliği (F10.1/F10.2) |
| 10 | launch config | hardware.yaml okunamazsa GÜRÜLTÜLÜ uyarı (F-V.5 — video modunun sessizce kaybolmaması) |
| 11 | Ekran-2 aracı | grafik CSV → PNG/MP4 üretimi (montaj zinciri) |
| 12 | UÇTAN UCA | sahte QGC görevi → GUIDED tetiği → 4 varış → TAMAMLANDI → thrust [0,0] → CSV'ler |

## Beklenen sonuçlar

- **PC (GPU'suz, deps ws source'lu):** tüm bileşenler ✅; tam suite
  **262 passed / 5 skipped** (2026-07-13 tabanı — skip'ler gerekçeli:
  cupy paritesi GPU ister, share-dizini testi colcon install ister, vb.).
- **Jetson (kurulum rehberi sonrası):** tüm bileşenler ✅; skip sayısı
  PC'den KÜÇÜK olmalı (cupy + share testleri orada gerçekten koşar).
  `failed/error = 0` şart — kırmızı varken çekime çıkılmaz.
- rclpy/mavros_msgs yoksa node testleri "skipped" düşer — bu YEŞİL sayılmaz
  ortam eksiğidir: Jetson'da `ros-humble-mavros-msgs` kurulu olmalı
  (BURADAN_BASLA.md §2.4).

## Jetson hatırlatmaları

- numpy ALTIN KURALI: pip kurulumlarında `numpy==1.26.4` aynı komutta
  sabitlenir (jetson-surum-pinleri); testlerden önce
  `python3 -c "import numpy; print(numpy.__version__)"` → `1.26.4`.
- Bu koşu güvenlidir: hiçbir test FCU'ya komut göndermez (sahte
  yayıncılar), ARM/motor riski yok. Yine de alışkanlık: pervanesiz masa.

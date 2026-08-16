"""SIGTERM'i düzgün kapanışa çevir — TESLİM DOSYALARINI KURTARAN KAPI.

🔴 **Çözdüğü arıza (15.08.2026, kaptan: *"local map videosu açılmıyor"*).**
Üç teslim dosyasının ÜÇÜ de oynatılamıyordu:

```
local_map/oturum_20260814_153300/Dosya3_lokal_harita.mp4  → moov atom not found
lidar/oturum_20260814_153300/lidar_kumeleme.mp4           → moov atom not found
kamera/session_20260814_154548/seg_0009.mp4               → moov atom not found
```

**Kod suçsuzdu:** `Mp4Yazici.kapat()` `release()`'i çağırıyor, `destroy_node()`
`kapat()`'ı çağırıyor, `main()` `finally`'de `destroy_node()`'u çağırıyor —
zincir eksiksiz. Eksik olan, zincirin hiç TETİKLENMEMESİYDİ.

`systemctl stop/restart` **SIGTERM** gönderir. Python'un SIGTERM için
varsayılan davranışı süreci **anında sonlandırmaktır**: yığın çözülmez,
`finally` çalışmaz, `release()` çağrılmaz → mp4'ün `moov` atomu (indeks)
hiç yazılmaz → dosya **oynatılamaz**. rclpy yalnız SIGINT (Ctrl+C) için
işleyici kurar; SIGTERM'e dokunmaz.

Kanıt: `girdap-karar.service` 14.08 14:53:19'da durduruldu (*"Stopping GIRDAP
IDA karar yigini… Deactivated successfully"*) ve o oturumun mp4'ü bozuk;
15:32 oturumunun dosyası da servis 16:03'te durunca bozuldu. Servis birimi
zaten `TimeoutStopSec=30` veriyor — yani süre BOLDU, düzgün kapanmak için
hiçbir engel yoktu.

Şartname md 5.5.4.3.5: **teslim edilemeyen her dosya 5 ceza puanı** — üç
dosya = 15 puan. Bu yüzden bu kapı "iyileştirme" değil, teslim şartı.

Kullanım (kaydedici node'ların `main()`'inde, `rclpy.init`'ten SONRA):

    from girdap_decision.sigterm_kapanis import sigterm_kapanisi_kur
    ...
    rclpy.init(args=args)
    sigterm_kapanisi_kur()
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()      # ← artık SIGTERM'de de çalışır

⚠ **Neden `KeyboardInterrupt` fırlatılıyor da bayrak kurulmuyor:** node'ların
kapanış yolu ZATEN `except KeyboardInterrupt` + `finally` üzerine kurulu
(SIGINT için). Aynı yolu kullanmak, her node'a ikinci bir kapanış mantığı
eklememizi önler — iki kopya kapanış yolu, bu projenin daha önce iki kez
yediği "kopyalar ayrıştı" hatasıdır (§0.0b).
"""

from __future__ import annotations

import signal
from typing import Optional


def sigterm_kapanisi_kur(logger: Optional[object] = None) -> None:
    """SIGTERM alındığında `KeyboardInterrupt` fırlat → düzgün kapanış.

    `logger`: verilirse (rclpy logger) sinyal geldiğinde bir satır basar —
    saha günlüğünde *"kapanış düzgün müydü"* sorusunun cevabı kalsın diye.

    Aynı süreçte birden çok kez çağrılması zararsızdır (aynı işleyici kurulur).
    SIGTERM'in kurulamadığı ortamda (ör. ana iş parçacığı değilse) sessizce
    geçer — kaydedici asla bu yüzden düşmemeli.
    """

    def _isleyici(signum, frame):        # noqa: ANN001 - signal API imzası
        if logger is not None:
            try:
                logger.info(
                    "SIGTERM alindi → duzgun kapanis; mp4/teslim dosyalari "
                    "kapatiliyor (moov atomu yaziliyor)."
                )
            except Exception:            # günlükçü ölse bile kapanış sürsün
                pass
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _isleyici)
    except (ValueError, OSError):        # ana iş parçacığı değil / desteklenmiyor
        pass

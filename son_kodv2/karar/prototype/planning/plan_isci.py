"""
Girdap İDA — RRT*'ı KONTROL DÖNGÜSÜNDEN AYRI SÜREÇTE koşturan işçi (F-P.10).

🔴 NEYİ ÇÖZÜYOR (13.08.2026 canlı arıza, §0.66/§0.68/§0.69):
`planning_node` tek thread'de koşuyor (`rclpy.spin`) ve RRT* aynı thread'de.
Jetson'da ölçüldü: `plan()` 20 duba ile 331 ms, 100 engel ile ortanca 510 ms,
en kötü **1491 ms** — kontrol bütçesi **100 ms**. Blokladığı sürece:

  (a) `/mavros/setpoint_velocity/cmd_vel_unstamped` **susuyor**. ArduPilot
      GUIDED'da 3 s komut gelmezse aracı durdurur; **öncesinde SON komutu
      sürdürür** → tekne kör ilerler. Sahada ölçülen boşluklar 0,50-2,74 s.
  (b) Düğüm **kendi aboneliklerini** işleyemiyor → `_last_odom_t` yaşlanıyor →
      **kendi bekçisi** (F-P.1) "poz bayat" deyip thrust'ı sıfırlıyor. Kanıt:
      "poz 2,4 s bayat" satırı yazılırken füzyon kesintisiz **50 Hz** yayındaydı.

🔑 **NEDEN THREAD DEĞİL, SÜREÇ.** İlk plan `MultiThreadedExecutor` + ayrı
callback grubuydu; araştırma çürüttü: **Python GIL** yüzünden çok thread'li
executor yalnız G/Ç bağımlı işi çakıştırır, **işlemci bağımlı** işte paralellik
vermez. `rrt_star.py`'nin ana döngüsü saf Python (`for i in range(max_iter)` +
`math.hypot`); GIL bırakan ağır bir numpy bloğu yok → RRT* işçi THREAD'de de
kontrol timer'ını bekletirdi. Ayrı **süreç** = ayrı yorumlayıcı = ayrı GIL.

🔑 **NEDEN `spawn`, `fork` DEĞİL.** Bu düğüm MPPI'yi `cupy` ile GPU'da
koşturabiliyor; CUDA bağlamı yüklü bir süreci `fork` etmek tanımsız davranıştır
(çocukta CUDA çağrıları kilitlenir/çöker) — DDS soketleri için de aynı risk.
`spawn` taze yorumlayıcı başlatır: çocuk yalnız `rrt_star`'ı içe aktarır,
ne CUDA ne ROS yüklenir.

TASARIM SINIRLARI
-----------------
* **İlk plan İŞÇİYE GİTMEZ** (`PlanningPipeline` kararı): görev başında araç
  duruyor, `cmd_vel` akışı yok; orada bloklamak zararsız, referanssız kalmak
  zararlıdır (A1: `_ref_path` None → MPPI hiç kurulmaz → araç kıpırdamaz).
* **Tek istek uçuşta**: işçi meşgulken yeni istek GÖNDERİLMEZ (kuyruk şişmesin;
  bayat istek zaten değersizdir — sahne değişmiştir).
* **İşçi ölürse/yanıt vermezse** boru hattı sessizce **senkron** kola döner:
  planlama yavaşlar ama DURMAZ. Hiçbir güvenlik kararı işçiye bağlı değildir.
* Bu modül **ROS'suz** ve **cupy'siz** — çekirdek pytest koşumunda da yüklenir.
"""

from __future__ import annotations

import logging
import multiprocessing as mp
import queue
import time
from typing import List, Optional, Sequence, Tuple

from prototype.planning.rrt_star import (
    Bounds,
    CircleObstacle,
    RRTStar,
    RRTStarConfig,
)

__all__ = ["PlanIscisi", "VARSAYILAN_ZAMAN_ASIMI_S"]

logger = logging.getLogger(__name__)

#: İşçiden yanıt beklenecek üst süre (s). Ölçüm: en kötü `plan()` 1,49 s
#: (100 engel) / 1,18 s (1 km hedef, çözümsüz). 5,0 s ≈ 3× en kötü — bu süreyi
#: aşan işçi takılmış sayılır, öldürülüp yenisi kurulur (sessiz donma yok).
VARSAYILAN_ZAMAN_ASIMI_S: float = 5.0


#: İstek demeti: (no, sınır4, engel3'lü liste, RRTStarConfig, start, goal).
#: Sınıf yerine düz demet: pickle ucuz ve çocuk tarafında sürüm-bağımsız.


def _isci_dongusu(istek_q, sonuc_q) -> None:  # noqa: ANN001
    """Çocuk süreç: istek al → RRT* koş → sonuç yolla. ASLA istisna sızdırmaz.

    Çocukta ROS/cupy YOKTUR (spawn ile taze yorumlayıcı). `None` isteği
    düzgün kapanma işaretidir.
    """
    while True:
        try:
            istek = istek_q.get()
        except (EOFError, OSError):          # ebeveyn gitti
            return
        if istek is None:
            return
        no, sinir, engeller, cfg, start, goal = istek
        try:
            rrt = RRTStar(
                Bounds(*sinir),
                [CircleObstacle(cx, cy, r) for (cx, cy, r) in engeller],
                cfg,
            )
            yol = rrt.plan(start, goal)
            # 🔴 F-F.29: TEŞHİS DE DÖNER. Plan ayrı süreçte koştuğu için
            # `hedef_kurtarildi` / `kismi_plan` bayrakları ÇOCUKTA kalıyordu;
            # ebeveyn onları hiç göremiyor, sayaç 0 okunuyordu. Ölçüldü
            # (17.08 gecesi): kurtarma 37 vakayı kapattığı hâlde sayaç 0
            # görünüyordu — yani düzeltme çalışıyor ama KANITI yoktu.
            # Bu, deponun en pahalı hata sınıfının (arıza vardı, kod
            # biliyordu, söylemiyordu) planlayıcı süreç sınırındaki hâli.
            teshis = {
                "hedef_kurtarildi": getattr(rrt, "hedef_kurtarildi", None),
                "kismi_plan": getattr(rrt, "kismi_plan", None),
            }
            sonuc_q.put((no, yol, None, teshis))
        except Exception as exc:             # ValueError (pay içinde) dahil
            sonuc_q.put((no, None, repr(exc), {}))


class PlanIscisi:
    """RRT*'ı ayrı süreçte koşturan asenkron planlayıcı kolu.

    Kullanım (boru hattı):
        if isci.gonder(...):     # meşgulse False döner, çağıran senkron kalır
            ...
        sonuc = isci.sonuc_al()  # bloklamaz; None = henüz yok
    """

    def __init__(self, zaman_asimi_s: float = VARSAYILAN_ZAMAN_ASIMI_S) -> None:
        self._zaman_asimi_s = float(zaman_asimi_s)
        self._ctx = mp.get_context("spawn")      # ⚠ fork DEĞİL (CUDA/DDS)
        self._proc: Optional[mp.process.BaseProcess] = None
        self._istek_q = None
        self._sonuc_q = None
        self._bekleyen_no: Optional[int] = None
        self._gonderim_t: float = 0.0
        self._sayac = 0
        self._kullanilabilir = True              # spawn başarısızsa kapanır
        #: teşhis sayaçları
        self.gonderilen = 0
        self.tamamlanan = 0
        self.zaman_asimi = 0
        #: F-F.29 — son planın teşhis bayrakları (hedef kurtarma / kısmi plan)
        self.son_teshis: dict = {}

    # ----- yaşam döngüsü -----

    def _kur(self) -> bool:
        """İşçiyi (gerekiyorsa) başlat. Başarısızlıkta kalıcı olarak kapanır."""
        if not self._kullanilabilir:
            return False
        if self._proc is not None and self._proc.is_alive():
            return True
        try:
            self._istek_q = self._ctx.Queue()
            self._sonuc_q = self._ctx.Queue()
            self._proc = self._ctx.Process(
                target=_isci_dongusu,
                args=(self._istek_q, self._sonuc_q),
                daemon=True,                     # ebeveyn ölürse çocuk da ölsün
                name="girdap_plan_iscisi",
            )
            self._proc.start()
            return True
        except Exception as exc:                 # spawn yok/izin yok/kaynak yok
            logger.warning(
                "plan işçisi başlatılamadı (%r) → senkron kola dönülüyor", exc
            )
            self._kullanilabilir = False
            self._proc = None
            return False

    def kapat(self) -> None:
        """İşçiyi düzgünce durdur (düğüm kapanışında)."""
        if self._proc is None:
            return
        try:
            self._istek_q.put(None)
            self._proc.join(timeout=1.0)
        except Exception:                        # kapanışta gürültü çıkarma
            pass
        finally:
            if self._proc.is_alive():
                self._proc.terminate()
            self._proc = None
            self._bekleyen_no = None

    def _oldur_ve_sifirla(self) -> None:
        """Takılan işçiyi öldür; bir sonraki gönderimde yenisi kurulur."""
        try:
            if self._proc is not None:
                self._proc.terminate()
                self._proc.join(timeout=0.5)
        except Exception:
            pass
        self._proc = None
        self._bekleyen_no = None

    # ----- sorgu -----

    @property
    def mesgul(self) -> bool:
        """Uçuşta bir istek var mı."""
        return self._bekleyen_no is not None

    @property
    def kullanilabilir(self) -> bool:
        """İşçi kolu açık mı (spawn başarısızsa kalıcı False)."""
        return self._kullanilabilir

    # ----- iş -----

    def gonder(
        self,
        sinir: Bounds,
        engeller: Sequence[CircleObstacle],
        cfg: RRTStarConfig,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        simdi: Optional[float] = None,
    ) -> bool:
        """İsteği işçiye yolla. Meşgulse/işçi yoksa **False** (çağıran karar verir).

        Engeller düz demete çevrilir: `CircleObstacle` dataclass'ı da picklenir
        ama demet hem ucuz hem sürüm-bağımsızdır (çocukta yeniden kurulur).
        """
        if self.mesgul or not self._kur():
            return False
        self._sayac += 1
        try:
            self._istek_q.put((
                self._sayac,
                (sinir.x_min, sinir.x_max, sinir.y_min, sinir.y_max),
                [(o.cx, o.cy, o.r) for o in engeller],
                cfg,
                (float(start[0]), float(start[1])),
                (float(goal[0]), float(goal[1])),
            ))
        except Exception as exc:
            logger.warning("plan isteği yollanamadı (%r)", exc)
            self._oldur_ve_sifirla()
            return False
        self._bekleyen_no = self._sayac
        self._gonderim_t = time.monotonic() if simdi is None else simdi
        self.gonderilen += 1
        return True

    def sonuc_al(
        self, simdi: Optional[float] = None
    ) -> Optional[Tuple[Optional[List[Tuple[float, float]]], Optional[str]]]:
        """Bloklamadan sonucu al: `(yol, hata)` ya da henüz yoksa **None**.

        Zaman aşımına uğrayan işçi öldürülür ve `(None, "zaman aşımı")` döner —
        çağıran bunu "plan üretilemedi" gibi ele alır (mevcut referans korunur).
        """
        if self._bekleyen_no is None:
            return None
        try:
            # F-F.29: çocuk artık 4. eleman (teşhis) yolluyor. Eski 3'lü
            # biçim de kabul edilir — sürüm karışırsa düğüm ÖLMESİN.
            paket = self._sonuc_q.get_nowait()
            if len(paket) == 4:
                no, yol, hata, teshis = paket
            else:
                no, yol, hata = paket
                teshis = {}
        except (queue.Empty, AttributeError, OSError):
            an = time.monotonic() if simdi is None else simdi
            if (an - self._gonderim_t) > self._zaman_asimi_s:
                self.zaman_asimi += 1
                logger.warning(
                    "plan işçisi %.1f s'dir yanıt vermedi → yeniden kuruluyor",
                    an - self._gonderim_t,
                )
                self._oldur_ve_sifirla()
                return (None, "zaman aşımı")
            return None
        if no != self._bekleyen_no:              # bayat yanıt (yeniden kurulum)
            return None
        self._bekleyen_no = None
        self.tamamlanan += 1
        self.son_teshis = teshis            # F-F.29
        return (yol, hata)

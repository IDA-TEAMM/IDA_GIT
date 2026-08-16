"""Girdap İDA — PIVOT KAPISI: hedef arkadayken önce DÖN, sonra git (F-F.20).

14.08.2026, GIRDAP_DURUM §1.01. **Ölçülen arızadan doğdu.**

🔴 ÖLÇÜLEN ARIZA (14.08 su koşumu, `session_20260814_153256`)
--------------------------------------------------------------
15:54:17'de waypoint-0'a varıldı ve yeni hedef **101° yanda / 52 m arkada**
kaldı. Araç bundan sonra:

    8 metre ilerlemek icin  45,4 saniye  ·  21,2 metre yol  ·  verim 0,38
    (en kotu 20 s dilimi: 8,3 m yol, 0,8 m net)

Saniye saniye komut, dönmek yerine **ileri-geri saldırdığını** gösteriyor:

    +1296s  yon 171°   komut u = -1,013     +1299s  yon 170°   komut u = +0,600
    +1302s  yon -173°  komut u = -0,952     +1304s  yon -177°  komut u = +0,672

Görev penceresinde komut edilen ileri hızın **%36,7'si geri** (867/2362 mesaj,
ortalama −0,639, en büyüğü **−1,173 = yazılım tavanının tamamı ters yönde**);
işaret **139 kez** değişti.

🌐 ARAŞTIRMA — BU BİLİNEN BİR SINIF HATA, ÜÇ YIĞINDA DA ÇÖZÜMÜ VAR
-------------------------------------------------------------------
* **ArduPilot Rover — pivot dönüşü.** Teknemizde **`WP_PIVOT_ANGLE = 60`** ve
  **`WP_PIVOT_RATE = 60`** ZATEN KURULU: *"açı hatası bu değeri aşarsa araç
  durur, hedefe döner, sonra devam eder; başı hedefe **10 derece** yaklaşınca
  yola devam eder."* 🔑 **Ama bizde çalışmıyor** — GUIDED'da uçuş kontrolcüsüne
  hedef noktası değil **hız setpoint'i** gönderdiğimiz için ArduPilot'un kendi
  seyir kontrolcüsü (ve pivot mantığı) devre dışı kalıyor.
* **Nav2 MPPI** — `PreferForwardCritic` (ağırlık 5,0) + `PathAngleCritic`
  (2,2; azami açı 45°). Belgesi: *"hedef aracın arkasındaysa MPPI geri yörünge
  seçebilir; iki kritiğin etkileşimi aracın **ileri-geri salınıp
  salınmayacağını** belirler."*
* **Nav2 Pure Pursuit / MATLAB MPPI** — `use_rotate_to_heading`: *"başlangıç
  yönü yoldan saparsa araç hareket etmeden ÖNCE yerinde döner."*

Bizim MPPI maliyetimizde **ileri tercihi terimi YOK** ve `w_heading`'in ölçülmüş
ayırt edicilik payı **%0,1** → MPPI'yi geri gitmekten alıkoyan hiçbir şey yok.

🎛️ NEDEN SERT KAPI, YUMUŞAK TERİM DEĞİL
----------------------------------------
Yumuşak maliyet terimi (Nav2'nin `PreferForwardCritic` karşılığı) da eklenebilir
ve **eklenmeli** — ama önce bu. Sebep: MPPI'nin softmax'ı bu depoda **dejenere
olma geçmişine sahip** (λ bölümü: ESS 2,6/1000, "en iyi rastgele örneği seç"e
düşme). Dejenere softmax'ta yumuşak terimler güvenilmez. Sert kapı
deterministiktir, test edilebilir ve teknenin **kendi ayarlanmış sayılarını**
kullanır.

🛟 GÜVENLİK
-----------
Bu kapı `planning_node`'da **bekçi zincirinden ÖNCE** uygulanır. Yani
`DISARM-VEYA-KILL`, `POZ-SACMA`, `POZ-BAYAT`, `ENGEL-BAYAT` sıfırları pivotu
**her zaman ezer** — kapı asla bir duruşu geciktiremez.

⚙️ HİSTEREZİS ZORUNLU
---------------------
Tetik (60°) ve bırakma (10°) **farklı** olmalı; tek eşik kullanılsaydı araç
sınırda gidip gelirken kapı 10 Hz'te açılıp kapanır ve düzeltmek istediğimiz
salınımın **aynısını** üretirdik. İki sayı da uydurma değil: ArduPilot'un
`WP_PIVOT_ANGLE` varsayılanı ve "10° içinde devam et" kuralı.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PivotKapisiConfig:
    """Eşikler ArduPilot Rover'ın kendi pivot ayarlarından gelir."""

    #: `WP_PIVOT_ANGLE` — bu açıdan büyük yön hatasında pivot BAŞLAR.
    #: `<= 0` → kapı tamamen kapalı (eski davranış, A/B ölçümü için).
    tetik_derece: float = 60.0
    #: ArduPilot: "başı hedefe 10 derece yaklaşınca yola devam eder".
    birak_derece: float = 10.0
    #: Yön hatası, referansın bu kadar İLERİSİNDEKİ noktaya bakılarak ölçülür.
    #: Hemen yanı başındaki nokta gürültülüdür; çok uzağı ise virajı görmez.
    #: Varsayılan `terminal_lookahead_m` ile aynı (3,0 m) — bilinçli.
    ufuk_m: float = 3.0


class PivotKapisi:
    """Yön hatası büyükken "yalnız dön" kararı üreten histerezisli kapı."""

    def __init__(self, config: Optional[PivotKapisiConfig] = None) -> None:
        self._cfg = config or PivotKapisiConfig()
        self._aktif = False

    @property
    def config(self) -> PivotKapisiConfig:
        return self._cfg

    @property
    def aktif(self) -> bool:
        return self._aktif

    def sifirla(self) -> None:
        self._aktif = False

    def guncelle(
        self,
        x: float,
        y: float,
        psi: float,
        referans: Optional[Sequence[Tuple[float, float]]],
    ) -> Tuple[bool, Optional[float]]:
        """(pivot_aktif, yön_hatası_rad) döndürür.

        `referans` yoksa/boşsa kapı **kapanır** ve `None` hata döner: neye
        döneceğini bilmeden dönmek, kör sürmenin dönen hâli olurdu.
        """
        if self._cfg.tetik_derece <= 0.0:
            self._aktif = False
            return False, None

        hedef = self._ufuk_noktasi(x, y, referans)
        if hedef is None:
            self._aktif = False
            return False, None

        hata = _sar(math.atan2(hedef[1] - y, hedef[0] - x) - psi)
        mutlak = abs(math.degrees(hata))
        if self._aktif:
            if mutlak <= self._cfg.birak_derece:
                self._aktif = False
        elif mutlak > self._cfg.tetik_derece:
            self._aktif = True
        return self._aktif, hata

    def _ufuk_noktasi(
        self, x: float, y: float,
        referans: Optional[Sequence[Tuple[float, float]]],
    ) -> Optional[Tuple[float, float]]:
        """Referans üzerinde araçtan >= `ufuk_m` uzaktaki İLK nokta.

        Hiçbiri o kadar uzak değilse (hedefe varılmak üzere) **son nokta**
        kullanılır; o da araca çok yakınsa yön açısı anlamsızdır → None.
        """
        if not referans:
            return None
        for px, py in referans:
            if math.hypot(px - x, py - y) >= self._cfg.ufuk_m:
                return (px, py)
        sx, sy = referans[-1]
        # Çok yakın nokta → atan2 gürültüye döner; kapıyı açma.
        return (sx, sy) if math.hypot(sx - x, sy - y) >= 0.5 else None


def pivot_itkisi(hata_rad: float, itki: float) -> List[float]:
    """Saf dönüş itkisi: ileri bileşen SIFIR, yalnız diferansiyel.

    `_publish_cmd_vel` çevirisi: `linear.x ∝ (u0+u1)` → toplam sıfır olmalı;
    `angular.z ∝ (u1-u0)` → pozitif hata (hedef saat yönünün TERSİNDE) için
    `u1 > u0`.
    """
    a = abs(itki)
    return [-a, a] if hata_rad >= 0.0 else [a, -a]


def _sar(a: float) -> float:
    """Açıyı (-π, π] aralığına sar."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

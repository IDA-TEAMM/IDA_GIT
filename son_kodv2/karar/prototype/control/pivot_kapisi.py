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
    #: 🔴 F-F.24 — YAKIN ALAN KÖRLÜĞÜ: bu yarıçapın içindeki referans noktasına
    #: kerteriz ölçülmez, kapı AÇILMAZ. `atan2` bu mesafede gürültüye döner.
    #:
    #: ÖLÇÜLEN ARIZA (17.08, `session_20260817_193312`, 1. GUIDED penceresi):
    #: tekne hedefe **0,71 m** yaklaştı ve saniye saniye şu oldu —
    #:     t=73,3  mesafe 1,04 m  |yön| 35°   komut ux=0,000 wz=+0,289
    #:     t=75,1  mesafe 0,71 m  |yön| 98°   (aynı saf dönüş komutu)
    #:     t=77,5  mesafe 1,25 m  |yön| 178°  ← kerteriz SÜPÜRDÜ, tekne yetişemedi
    #:     t=80,8  mesafe 2,64 m  |yön| 165°  komut ux=+0,905 ← ARKASINA tam gaz
    #: Tekne 0,17 m/s'lik artık süratiyle kayarken hedefin kerterizi teknenin
    #: dönebileceğinden hızlı döndü; kapı bir daha asla bırakma eşiğine (10°)
    #: inemedi. Bu bir ayar sorunu değil, **geometrik singülarite**.
    #:
    #: 🌐 ARAŞTIRMA: denizcilik güdüm literatüründe (LOS — line-of-sight)
    #: standart karşılığı "circle of acceptance"; pratik değer **iki gemi
    #: boyu**. Gövdemiz 0,785 m ⇒ 1,57 m. Eski sabit 0,50 m bunun üçte biri.
    #: Kaynak: Lekkas & Fossen, "Line-of-Sight Guidance for Path Following of
    #: Marine Vehicles"; Esso Osaka model çalışması (kabul yarıçapı = 2 L).
    #:
    #: ⚠ VARSAYILAN 0,50 = ESKİ DAVRANIŞ BİREBİR. Ölçülen değer 1,57'dir ama
    #: bunu varsayılan yapmak, kapının yakın alanda hiç açılmaması demek —
    #: sahada A/B ile ölçülmeden açılmaz (§0.8a).
    yakin_esik_m: float = 0.50
    #: Yön hatası, referansın bu kadar İLERİSİNDEKİ noktaya bakılarak ölçülür.
    #: Hemen yanı başındaki nokta gürültülüdür; çok uzağı ise virajı görmez.
    #: Varsayılan `terminal_lookahead_m` ile aynı (3,0 m) — bilinçli.
    ufuk_m: float = 3.0


class PivotKapisi:
    """Yön hatası büyükken "yalnız dön" kararı üreten histerezisli kapı."""

    #: 🔴 F-F.25 — KAPININ SESSİZLİĞİ. 17.08 göl bandında yön hatası ortanca
    #: 130° olan geri komutların **%91'inde bu kapı KAPALIYDI** ve neden
    #: kapalı olduğu hiçbir yerde yazmıyordu: `guncelle` üç ayrı sebepten
    #: `(False, None)` dönüyor ve üçü de aynı görünüyor. Aynı bantta nöbetçi
    #: 43 kez `RRT-RED global plan uretilemedi` bastı — yani "referans yoktu"
    #: kuvvetli bir aday, ama `global_path` banda kaydedilmediği için
    #: **ne doğrulanabildi ne çürütülebildi.**
    #:
    #: Bu deponun en sık tekrarlayan deseni: *arıza vardı, kod onu biliyordu,
    #: kimseye söylemiyordu.* Sebep artık dışarı açılıyor; mekanizma
    #: DEĞİŞMİYOR, yalnız sessizlik kalkıyor.
    SEBEP_KAPALI = "KAPI-KAPALI"        # tetik_derece <= 0 (kapı devre dışı)
    SEBEP_REFERANS_YOK = "REFERANS-YOK"  # plan boş/None → RRT-RED adayı
    SEBEP_COK_YAKIN = "COK-YAKIN"        # yakın alan singülaritesi (F-F.24)
    SEBEP_HATA_KUCUK = "HATA-KUCUK"      # ölçüldü, tetik eşiğinin altında
    SEBEP_AKTIF = "AKTIF"                # kapı açık, pivot uygulanıyor

    def __init__(self, config: Optional[PivotKapisiConfig] = None) -> None:
        self._cfg = config or PivotKapisiConfig()
        self._aktif = False
        self._son_sebep: str = self.SEBEP_REFERANS_YOK
        self._son_hata_derece: Optional[float] = None

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
        self._son_hata_derece = None
        if self._cfg.tetik_derece <= 0.0:
            self._aktif = False
            self._son_sebep = self.SEBEP_KAPALI
            return False, None

        # F-F.25: iki "hedef yok" hâli AYRI raporlanır — biri plan eksikliği
        # (RRT-RED), diğeri yakın alan singülaritesi (F-F.24). Aynı görünen
        # iki arızanın çaresi farklı; ayırmayan log teşhis ettirmez.
        if not referans:
            self._aktif = False
            self._son_sebep = self.SEBEP_REFERANS_YOK
            return False, None

        hedef = self._ufuk_noktasi(x, y, referans)
        if hedef is None:
            self._aktif = False
            self._son_sebep = self.SEBEP_COK_YAKIN
            return False, None

        hata = _sar(math.atan2(hedef[1] - y, hedef[0] - x) - psi)
        mutlak = abs(math.degrees(hata))
        self._son_hata_derece = mutlak
        if self._aktif:
            if mutlak <= self._cfg.birak_derece:
                self._aktif = False
        elif mutlak > self._cfg.tetik_derece:
            self._aktif = True
        self._son_sebep = self.SEBEP_AKTIF if self._aktif else self.SEBEP_HATA_KUCUK
        return self._aktif, hata

    @property
    def son_sebep(self) -> str:
        """F-F.25: son `guncelle` çağrısında kapının NEDEN o durumda olduğu."""
        return self._son_sebep

    @property
    def son_hata_derece(self) -> Optional[float]:
        """Ölçülebildiyse son yön hatası (derece); ölçülemediyse None."""
        return self._son_hata_derece

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
        # Çok yakın nokta → atan2 gürültüye döner; kapıyı açma (F-F.24).
        return (sx, sy) if math.hypot(sx - x, sy - y) >= self._cfg.yakin_esik_m else None


def pivot_itkisi(
    hata_rad: float,
    itki: float,
    *,
    orantili: bool = False,
    taban: float = 0.30,
    tetik_derece: float = 60.0,
    birak_derece: float = 10.0,
) -> List[float]:
    """Saf dönüş itkisi: ileri bileşen SIFIR, yalnız diferansiyel.

    `_publish_cmd_vel` çevirisi: `linear.x ∝ (u0+u1)` → toplam sıfır olmalı;
    `angular.z ∝ (u1-u0)` → pozitif hata (hedef saat yönünün TERSİNDE) için
    `u1 > u0`.

    ─────────────────────────────────────────────────────────────────────
    `orantili=False` (VARSAYILAN) → eski BANG-BANG davranışı, BİT BİREBİR.
    `orantili=True`  → itki yön hatasıyla ölçeklenir.
    ─────────────────────────────────────────────────────────────────────

    🔴 NEDEN ORANTILI SEÇENEĞİ VAR (17.08 bant ölçümü + literatür):

    Bang-bang kontrolün bilinen kusuru: *"ara kontrol seviyeleri olmadan
    sistem HEDEFE YAKLAŞIRKEN BİLE tam aktüasyon uygular, bu da hedefi
    aşmasına ve DÖNGÜYE girmesine neden olur."* ArduPilot'un kendi pivot
    çözümü de orantılıdır (`target_turn_rate = (steering/4500)·ACRO_TURN_RATE`).

    Bandımızda o imza ÖLÇÜLDÜ (16.08 183648, 92 pivot atağı, GUIDED):
        atak süresi     : medyan 9,5 s · %90 73,3 s · maks 265 s
        geometrik beklenti (50° / 21,8 °/s) : 2,3 s      ⇒ 4-32 KAT uzun
        atak içinde dönüş YÖNÜ değişimi (aşımın doğrudan izi):
            ≥1 kez : 24/87 = %28
            ≥2 kez : 12/87 = %14   ← salınım
    Orantılı kontrolde bu ~0 olmalıydı.

    ⚠️ AMA ÖLÇÜM İKİNCİ BİR ŞEY DE GÖSTERDİ — dönüş hızı profili:
        ilk çeyrek 7,7 °/s → son çeyrek 16,6 °/s   (2,16 KAT ARTIYOR)
    Bang-bang'de SABİT, orantılıda AZALAN olmalıydı. Artması, teknenin
    komutu ancak yavaş yavaş yakaladığını gösteriyor ⇒ **birincil sebep
    `ATC_STR_RAT_FF`'in üçte bir olması** (0,20 ↔ ölçülen 0,55).
    ⇒ Orantılı pivot İKİNCİL bir düzeltmedir; FF düzeltilmeden tek başına
      beklenen kazancı vermeyebilir.

    📐 `taban` NEREDEN GELİYOR (ölçüldü, tahmin DEĞİL):
    Teknenin gerçekten dönmeye başladığı en küçük direksiyon çıkışı, GUIDED
    kesitlerinde 9.890 örnekle kova kova ölçüldü:
        |direksiyon| 0,00-0,02 → medyan  2,4 °/s · >3°/s oranı %44
        |direksiyon| 0,02-0,05 → medyan  4,2 °/s · >3°/s oranı %61  ← eşik
        |direksiyon| 0,05-0,08 → medyan  7,6 °/s · >3°/s oranı %75
    Ham eşik **0,035** çıktı ama o REDDEDİLDİ: `birak_derece`de (10°) yalnız
    0,8 °/s bırakır ⇒ tekne fiilen durur, **pivot HİÇ BİTMEZ** — yeni bir
    arıza sınıfı açardı (14.08'in *"dönmek yerine ileri-geri saldırıyordu"*
    vakasının kardeşi).
    **`taban = 0,30` seçildi:**
        10° (bırakma) → kazanç 0,30 ⇒  6,5 °/s   — 0,05-0,08 bandı 7,6 °/s
                                                    ürettiği için ULAŞILABİLİR
        35°           → kazanç 0,52 ⇒ 14,2 °/s
        60° (tetik)   → kazanç 1,00 ⇒ 21,8 °/s   — bang-bang ile AYNI
    Aşım payı: 10°'de 6,5 °/s ⇒ 0,1 s'de 0,65° ⇒ bırakma eşiği aşılmaz.

    ⛔ VARSAYILAN KAPALI: bu YENİ bir davranış ve suda sınanmadı. Yarışmaya
    3 gün kala varsayılanı değiştirmek, ölçülmemiş bir riski görev gününe
    taşımak olurdu. Sahada `--orantili` ile A/B edilebilir; ölçüt hazır:
    `inhibit_reason`daki PIVOT oranı (%71) ve atak süresi (medyan 9,5 s).
    """
    a = abs(itki)
    if orantili:
        # hata bırakma eşiğindeyken `taban`, tetik eşiğinde tam itki
        pay = max(1e-6, tetik_derece - birak_derece)
        oran = (abs(math.degrees(hata_rad)) - birak_derece) / pay
        kazanc = taban + (1.0 - taban) * min(1.0, max(0.0, oran))
        a *= kazanc
    return [-a, a] if hata_rad >= 0.0 else [a, -a]


def _sar(a: float) -> float:
    """Açıyı (-π, π] aralığına sar."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

"""
Girdap İDA — Kuşbakışı (BEV) çizici: teslim edilecek görsellerin ORTAK çekirdeği.

ROS-bağımsız (prototype/ kuralı): girdi saf Python/numpy, çıktı RGB ndarray.
Layer 2 node'ları bunu sarar; mp4 yazımı `Mp4Yazici`'da (cv2 TEMBEL import).

════════════════════════════════════════════════════════════════════════════
NEDEN TEK MODÜL, İKİ TESLİM — şartname md 4.2 (s.14) birebir
════════════════════════════════════════════════════════════════════════════
İki ayrı zorunlu teslim aynı geometrik işi istiyor; ikisini iki kez yazmak
ikisinin ayrışması demekti (bkz. algı katmanındaki `saat.py` gerekçesi):

  ▪ "Diğer Otonomi Sensörleri Veri Seti" — LiDAR (md 487-493)
        · "her bir sensör tipi için ayrı ayrı"
        · "En az 1 Hz"
        · "Her bir veri seti ZAMAN ETİKETİNE sahip olacak şekilde mp4"
        · 🔑 "Tespit ve takip işlemleri sonucunda KÜMELEME, AYIRMA vs. gibi
             bir işlem yapıldıysa GÖRÜNECEK şekilde"
     → `render_lidar()`

  ▪ "Dosya 3: Lokal harita/cost map/engel haritası" (md 505-506)
        · "En Az 1 Hz"   ← ŞARTNAMEDE FORMAT YOK (tek format belirtilmeyen kalem)
     → `render_costmap()`

🔑 **"KÜMELEME, AYIRMA GÖRÜNECEK ŞEKİLDE" NE DEMEK — tasarım kararı.**
Madde bir *sonucun* değil bir *işlemin* görünmesini istiyor. Bu yüzden ham
noktalar da çizilir: izleyici "şu nokta yığını → şu küme oldu → şu sınıfı
aldı" zincirini karede görür. Yalnız daireler çizilseydi kümelemenin
YAPILDIĞI görünmezdi. Üç katman üst üste biner:
    1) ham nokta bulutu        → koyu gri (sensörün gördüğü)
    2) küme üyeliği            → küme başına AYRI renk (= "ayırma")
    3) sınıf + küme kimliği    → daire + "K3 TURUNCU" etiketi

════════════════════════════════════════════════════════════════════════════
KUZEY YUKARI — ve bu bilerek yazılıyor
════════════════════════════════════════════════════════════════════════════
Raster DÜNYA eksenlidir (kuzey yukarı), araca göre DÖNMEZ; yalnız araç
merkezlidir. Aracın yönü ortadaki üçgenden okunur.

⚠️ **Bu, projenin daha önce iki kez yediği hatanın panzehiri.** `frame_id`
etiketiyle verinin gerçek çerçevesi bir kez `perception_lidar_node`'da
(GIRDAP_DURUM §0.0b), bir kez de `_publish_local_map`'te ayrıştı. Burada
çerçeve KAREYE YAZILIYOR ("KUZEY YUKARI" + kuzey oku): teslim edilen görsel
kendi çerçevesini kendi taşır, etikete güvenmek zorunda kalınmaz.

════════════════════════════════════════════════════════════════════════════
ZAMAN VE ÖLÇEK KAREYE YAKILIR
════════════════════════════════════════════════════════════════════════════
Kare kendi zamanını ve ölçeğini taşır. Sebep üç katlı:
  · md 479-480/491-492 zaman etiketini AÇIKÇA istiyor (Dosya-1 ve LiDAR);
  · dosya adı/mtime'a güvenilemez — Jetson saati 07.08'de ~3 saat geri
    ölçüldü (bkz. GIRDAP_DURUM §0.14 / algı `saat.py`);
  · "En Az 1 Hz" ancak kareden okunabiliyorsa DOĞRULANABİLİR; mp4'ün fps
    metadata'sı oynatma hızıdır, örnekleme hızı değildir.
`saat_guvenilir=False` verilirse damganın yanına "SAAT?" basılır — yalan
söylemek yerine bilmediğimizi yazarız.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

Point = Tuple[float, float]
RGB = Tuple[int, int, int]

# ---------------------------------------------------------------------------
# Sınıf renkleri — camera_buoys sabitleriyle AYNI anlam (0/1/2/99).
# Renkler şartnamenin duba renklerine sadık: hakem kareye bakınca sahadaki
# nesneyle eşleştirebilsin.
# ---------------------------------------------------------------------------
SINIF_RENK: dict = {
    0: (255, 140, 0),     # turuncu — KENAR dubası (kapı), RAL2003
    1: (255, 215, 0),     # sarı    — ENGEL dubası, RAL1026
    2: (220, 20, 60),     # kırmızı — Parkur-3 HEDEF
    99: (150, 150, 150),  # gri     — UNKNOWN (füzyon eşleşmedi)
}
SINIF_AD: dict = {0: "KENAR", 1: "ENGEL", 2: "HEDEF", 99: "?"}

# Küme paleti — "ayırma"nın görünmesi için komşu kümeler ayrı renk almalı.
# Sabit ve döngüsel: aynı girdi aynı kareyi verir (belirlenimci, test edilebilir).
KUME_PALETI: Tuple[RGB, ...] = (
    (0, 200, 255), (255, 0, 255), (0, 255, 128), (255, 128, 0),
    (128, 128, 255), (255, 255, 0), (0, 255, 255), (255, 100, 100),
)

_ARKA = (18, 22, 28)          # koyu arka plan — beyaz nokta/renk okunsun
_IZGARA = (45, 52, 62)
_HAM_NOKTA = (110, 118, 128)
_METIN = (235, 240, 245)
_ARAC = (60, 255, 120)

# PNG yedek klasöründe fps'i taşıyan makine-okur işaret dosyası.
# Teslim toplayıcısı bunu okuyup ffmpeg'i doğru hızda koşturur.
FPS_ISARET_ADI = "_fps.txt"
# ffmpeg kare adı deseni — `kare_00000.png` ile birebir uyumlu olmalı.
PNG_KARE_DESENI = "kare_%05d.png"


@dataclass(frozen=True)
class BevConfig:
    """Çıktı geometrisi. `menzil_m` = karenin YARI genişliği (araçtan kenara)."""

    genislik_px: int = 720
    yukseklik_px: int = 720
    menzil_m: float = 25.0        # 50 m × 50 m pencere (local_map ile aynı)
    izgara_m: float = 5.0         # referans ızgara adımı
    nokta_yaricap_px: int = 1
    kume_nokta_yaricap_px: int = 2

    def __post_init__(self) -> None:
        if self.genislik_px <= 0 or self.yukseklik_px <= 0:
            raise ValueError("BevConfig: piksel boyutları pozitif olmalı")
        if self.menzil_m <= 0.0:
            raise ValueError("BevConfig: menzil_m pozitif olmalı")

    @property
    def m_per_px(self) -> float:
        """Metre/piksel — kısa kenardan türer (kare pencere garantisi)."""
        return (2.0 * self.menzil_m) / min(self.genislik_px, self.yukseklik_px)


@dataclass
class Kume:
    """Bir LiDAR kümesi + (varsa) füzyondan gelen sınıfı.

    `noktalar` boş olabilir: füzyon yalnız merkez/yarıçap veriyorsa daire
    yine çizilir. `kume_id` teslimde "ayırma"nın kanıtıdır — kareye basılır.
    """

    merkez: Point
    yaricap: float = 0.15
    sinif: Optional[int] = None
    kume_id: Optional[int] = None
    noktalar: Sequence[Point] = field(default_factory=tuple)


def _yazi_tipi() -> ImageFont.ImageFont:
    """Varsayılan bitmap font — sistemde TTF aramaz (Jetson'da olmayabilir)."""
    return ImageFont.load_default()


class BevRenderer:
    """Dünya ENU nesnelerini araç merkezli, KUZEY YUKARI rastere çizer."""

    def __init__(self, cfg: Optional[BevConfig] = None) -> None:
        self.cfg = cfg or BevConfig()
        self._font = _yazi_tipi()

    # ----- koordinat -----

    def dunya_to_px(self, p: Point, arac: Point) -> Tuple[float, float]:
        """Dünya ENU (x=doğu, y=kuzey) → piksel (sağ=doğu, aşağı=güney).

        Araç merkezdedir; DÖNME UYGULANMAZ (kuzey yukarı — modül docstring'i).
        """
        cfg = self.cfg
        mpp = cfg.m_per_px
        return (
            cfg.genislik_px / 2.0 + (p[0] - arac[0]) / mpp,
            cfg.yukseklik_px / 2.0 - (p[1] - arac[1]) / mpp,
        )

    # ----- ortak süsler -----

    def _zemin(self, d: ImageDraw.ImageDraw, arac: Point) -> None:
        """Metrik ızgara — ölçek hissi kareden okunsun (yalnız bar yetmez)."""
        cfg = self.cfg
        adim = cfg.izgara_m
        if adim <= 0:
            return
        # Izgarayı DÜNYA koordinatlarına kilitle: araç ilerledikçe çizgiler
        # akar → izleyici hareketi görür (sabit ızgarada tekne duruyor sanılır).
        k0 = math.floor((arac[0] - cfg.menzil_m) / adim)
        k1 = math.ceil((arac[0] + cfg.menzil_m) / adim)
        for k in range(k0, k1 + 1):
            x, _ = self.dunya_to_px((k * adim, arac[1]), arac)
            d.line([(x, 0), (x, cfg.yukseklik_px)], fill=_IZGARA, width=1)
        k0 = math.floor((arac[1] - cfg.menzil_m) / adim)
        k1 = math.ceil((arac[1] + cfg.menzil_m) / adim)
        for k in range(k0, k1 + 1):
            _, y = self.dunya_to_px((arac[0], k * adim), arac)
            d.line([(0, y), (cfg.genislik_px, y)], fill=_IZGARA, width=1)

    def _arac_isareti(
        self, d: ImageDraw.ImageDraw, yaw: Optional[float]
    ) -> None:
        """Merkeze araç üçgeni. yaw=None → yalnız artı (yön bilinmiyor)."""
        cfg = self.cfg
        cx, cy = cfg.genislik_px / 2.0, cfg.yukseklik_px / 2.0
        if yaw is None:
            d.line([(cx - 8, cy), (cx + 8, cy)], fill=_ARAC, width=2)
            d.line([(cx, cy - 8), (cx, cy + 8)], fill=_ARAC, width=2)
            return
        # ENU yaw: 0 = doğu, CCW pozitif. Ekranda y aşağı → sin işareti ters.
        boy = 14.0
        en = 7.0
        ux, uy = math.cos(yaw), -math.sin(yaw)          # ileri birim (piksel)
        sx, sy = -uy, ux                                # sancak birim
        d.polygon(
            [
                (cx + ux * boy, cy + uy * boy),
                (cx - ux * boy * 0.6 + sx * en, cy - uy * boy * 0.6 + sy * en),
                (cx - ux * boy * 0.6 - sx * en, cy - uy * boy * 0.6 - sy * en),
            ],
            fill=_ARAC,
        )

    def _kuzey_ve_olcek(self, d: ImageDraw.ImageDraw) -> None:
        """Kuzey oku + ölçek çubuğu — çerçeve ve ölçek KAREDE taşınsın."""
        cfg = self.cfg
        # kuzey oku (sağ üst)
        ox, oy = cfg.genislik_px - 30, 34
        d.line([(ox, oy), (ox, oy - 20)], fill=_METIN, width=2)
        d.polygon(
            [(ox, oy - 26), (ox - 5, oy - 16), (ox + 5, oy - 16)], fill=_METIN
        )
        d.text((ox - 4, oy + 2), "K", fill=_METIN, font=self._font)
        # ölçek çubuğu (sol alt) — tam `izgara_m` uzunluğunda.
        # ⚠ Alt satır istatistik metnine ayrıldı; çubuk onun ÜSTÜNDE durur
        # (ilk denemede ikisi üst üste binip ikisi de okunamaz olmuştu).
        bar_px = cfg.izgara_m / cfg.m_per_px
        bx, by = 14, cfg.yukseklik_px - 40
        d.line([(bx, by), (bx + bar_px, by)], fill=_METIN, width=2)
        d.line([(bx, by - 4), (bx, by + 4)], fill=_METIN, width=2)
        d.line([(bx + bar_px, by - 4), (bx + bar_px, by + 4)],
               fill=_METIN, width=2)
        d.text((bx, by - 16), f"{cfg.izgara_m:g} m", fill=_METIN,
               font=self._font)

    def _basliklar(
        self,
        d: ImageDraw.ImageDraw,
        baslik: str,
        zaman_metni: str,
        kare_no: Optional[int],
        saat_guvenilir: bool,
    ) -> None:
        """Zaman damgası + başlık — md 479-480/491-492 'zaman etiketi'."""
        d.text((10, 8), baslik, fill=_METIN, font=self._font)
        damga = zaman_metni if saat_guvenilir else f"{zaman_metni}  [SAAT?]"
        d.text((10, 22), damga, fill=_METIN, font=self._font)
        d.text((10, 36), "KUZEY YUKARI (dunya ENU) - arac merkezli",
               fill=_METIN, font=self._font)
        if kare_no is not None:
            d.text((10, 50), f"kare {kare_no}", fill=_METIN, font=self._font)

    # ----- LiDAR / füzyon karesi (md 487-493) -----

    def render_lidar(
        self,
        arac: Point,
        yaw: Optional[float] = None,
        ham_noktalar: Sequence[Point] = (),
        kumeler: Sequence[Kume] = (),
        zaman_metni: str = "",
        kare_no: Optional[int] = None,
        saat_guvenilir: bool = True,
    ) -> np.ndarray:
        """LiDAR karesi: ham nokta + küme ayrımı + sınıf. → (H, W, 3) uint8.

        Üç katman şartnamenin *"kümeleme, ayırma ... görünecek şekilde"*
        maddesini karşılar (modül docstring'i). Ham noktalar boş verilirse
        yalnız kümeler çizilir — madde yine sağlanır ama işlemin girdisi
        görünmez; mümkünse ham bulutu da geç.
        """
        cfg = self.cfg
        img = Image.new("RGB", (cfg.genislik_px, cfg.yukseklik_px), _ARKA)
        d = ImageDraw.Draw(img)
        self._zemin(d, arac)

        # 1) ham nokta bulutu
        r = cfg.nokta_yaricap_px
        for p in ham_noktalar:
            x, y = self.dunya_to_px(p, arac)
            if -r <= x <= cfg.genislik_px + r and -r <= y <= cfg.yukseklik_px + r:
                d.ellipse([x - r, y - r, x + r, y + r], fill=_HAM_NOKTA)

        # 2) küme üyeliği — küme başına AYRI renk ("ayırma" görünür)
        rk = cfg.kume_nokta_yaricap_px
        for i, k in enumerate(kumeler):
            kume_renk = KUME_PALETI[
                (k.kume_id if k.kume_id is not None else i) % len(KUME_PALETI)
            ]
            for p in k.noktalar:
                x, y = self.dunya_to_px(p, arac)
                d.ellipse([x - rk, y - rk, x + rk, y + rk], fill=kume_renk)

            # 3) sınıf dairesi + kimlik etiketi
            # 🔑 Halka küme noktalarının DIŞINDA durmalı. Gerçek ölçekte
            # 30 cm'lik duba ~4 px'tir; halka tam üstüne çizilirse küme
            # rengini KAPATIR ve md 493'ün istediği "ayırma" görünmez olur
            # (ilk denemede tam bu oldu). Bu yüzden yarıçap, küme nokta
            # yarıçapı + boşluk kadar dışarı itilir.
            cxp, cyp = self.dunya_to_px(k.merkez, arac)
            rp = max(7.0, k.yaricap / cfg.m_per_px + rk + 3.0)
            sinif_renk = SINIF_RENK.get(
                k.sinif if k.sinif is not None else 99, SINIF_RENK[99]
            )
            d.ellipse([cxp - rp, cyp - rp, cxp + rp, cyp + rp],
                      outline=sinif_renk, width=2)
            etiket = SINIF_AD.get(
                k.sinif if k.sinif is not None else 99, "?"
            )
            if k.kume_id is not None:
                etiket = f"K{k.kume_id} {etiket}"
            d.text((cxp + rp + 3, cyp - 6), etiket, fill=sinif_renk,
                   font=self._font)

        self._arac_isareti(d, yaw)
        self._kuzey_ve_olcek(d)
        self._basliklar(
            d, "GIRDAP IDA - LiDAR kumeleme (md 4.2)",
            zaman_metni, kare_no, saat_guvenilir,
        )
        d.text((10, cfg.yukseklik_px - 18),
               f"ham nokta: {len(ham_noktalar)}   kume: {len(kumeler)}",
               fill=_METIN, font=self._font)
        return np.asarray(img, dtype=np.uint8)

    # ----- Dosya-3 maliyet haritası (md 505-506) -----

    def render_costmap(
        self,
        occupancy: np.ndarray,
        cozunurluk_m: float,
        arac: Point,
        yaw: Optional[float] = None,
        kenar_dubalari: Sequence[Point] = (),
        zaman_metni: str = "",
        kare_no: Optional[int] = None,
        saat_guvenilir: bool = True,
    ) -> np.ndarray:
        """Dosya-3: occupancy ızgarası → ölçekli, zaman damgalı BEV karesi.

        `occupancy`: (H, W) int, ROS OccupancyGrid anlamı — 0 serbest,
        1..100 maliyet, <0 bilinmiyor. **Satır 0 = GÜNEY** (ROS konvansiyonu);
        burada `flipud` ile kuzey yukarı çevrilir (local_map.py ile aynı kural).

        🔑 `kenar_dubalari`: turuncu KENAR dubaları MPPI'nin engel torbasından
        bilerek çıkarılır (`planning_node._on_classified`) — dolayısıyla
        occupancy'de HİÇ görünmezler. Teslim edilen "engel haritası"nın
        parkurun ana nesnesini göstermemesi kabul edilemez olduğu için ayrı
        katman olarak buradan basılır. Planlama etkilenmez: bu yalnız çizim.
        """
        cfg = self.cfg
        arr = np.asarray(occupancy)
        if arr.ndim != 2:
            raise ValueError(
                f"occupancy 2 boyutlu olmalı, geldi: {arr.shape}"
            )
        if cozunurluk_m <= 0.0:
            raise ValueError("cozunurluk_m pozitif olmalı")

        h, w = arr.shape
        # ROS satır-major (satır 0 = güney) → kuzey yukarı
        g = np.flipud(arr)
        bilinmiyor = g < 0
        maliyet = np.clip(g, 0, 100).astype(np.float64)
        # 0 = serbest su (koyu mavi-gri), 100 = engel (kırmızı): gri tonlamadan
        # RENKLİ'ye çıkarıldı — hakem "boş mu, dolu mu"yu bir bakışta görsün.
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        t = (maliyet / 100.0)[..., None]
        serbest = np.array([30, 60, 90], dtype=np.float64)
        dolu = np.array([230, 60, 50], dtype=np.float64)
        rgb[:] = (serbest * (1.0 - t) + dolu * t).astype(np.uint8)
        rgb[bilinmiyor] = (70, 70, 70)                    # bilinmiyor → nötr gri

        # Izgara hücresi → piksel: pencereyi kaplayacak şekilde ölçekle.
        img = Image.fromarray(rgb, mode="RGB").resize(
            (cfg.genislik_px, cfg.yukseklik_px), Image.NEAREST
        )
        d = ImageDraw.Draw(img)

        # Kenar dubaları ayrı katman (yukarıdaki gerekçe).
        for p in kenar_dubalari:
            x, y = self.dunya_to_px(p, arac)
            rp = max(3.0, 0.15 / cfg.m_per_px)
            d.ellipse([x - rp, y - rp, x + rp, y + rp],
                      outline=SINIF_RENK[0], width=2)

        self._arac_isareti(d, yaw)
        self._kuzey_ve_olcek(d)
        self._basliklar(
            d, "GIRDAP IDA - Dosya-3 lokal harita/cost map",
            zaman_metni, kare_no, saat_guvenilir,
        )
        d.text((10, cfg.yukseklik_px - 18),
               f"izgara {w}x{h} @ {cozunurluk_m:g} m/hucre   "
               f"kirmizi=engel  gri=bilinmiyor",
               fill=_METIN, font=self._font)
        return np.asarray(img, dtype=np.uint8)


class PngSerisiYazici:
    """mp4 açılamadığında ACİL YEDEK — kareleri PNG serisi olarak yazar.

    `Mp4Yazici` ile **aynı arayüz** (`yaz` / `kapat` / `kare_sayisi`): çağıran
    node ikisinden hangisini tuttuğunu bilmek zorunda kalmaz.

    🔑 **Neden yeterli — teslim KURTARILIR.** PNG serisi sonradan tek komutla
    mp4'e çevrilir:
        ffmpeg -framerate 2 -i kare_%05d.png -c:v libx264 -pix_fmt yuv420p out.mp4
    Zaman damgası zaten KAREYE yakılı olduğu için dönüşümde hiçbir bilgi
    kaybolmaz. Bu yüzden "mp4 açılamadı" durumu artık teslim kaybı değil,
    yalnız fazladan bir dönüştürme adımıdır.

    🔴 **Neden gerekli:** Jetson'ın OpenCV derlemesinde `mp4v` codec'i
    olmayabilir (`ida_topics/kamera_kayit_node` bu riske karşı zaten F-P.11
    koruması taşıyor → proje bunu daha önce yaşamış). Ekransız bir makinede
    kaydedici sessizce ölürse o koşumun teslim dosyası hiç üretilmez ve bu
    ancak hakem masasında anlaşılır (md 5.5.4.3.5: dosya başına 5 ceza).
    """

    def __init__(self, dizin, fps: float = 2.0) -> None:
        self.dizin = Path(dizin)
        self.dizin.mkdir(parents=True, exist_ok=True)
        self.fps = float(fps)
        self.kare_sayisi = 0
        # 🔑 MAKİNE-OKUR fps işareti — teslim toplayıcısı bu klasörü bulunca
        # ffmpeg'i KENDİ koşturup mp4'ü üretiyor (`prototype/teslim/
        # toplayici.py::png_yedegini_mp4_yap`). fps'i metinden ayrıştırmak
        # kırılgan olurdu; tek satır sayı bırakıyoruz.
        (self.dizin / FPS_ISARET_ADI).write_text(f"{self.fps:g}\n",
                                                 encoding="utf-8")
        # İnsan için son çare talimatı. Normalde OKUNMASI GEREKMEZ — dönüşüm
        # otomatik. ffmpeg hiç yoksa geriye kalan tek yol bu.
        (self.dizin / "NASIL_MP4_YAPILIR.txt").write_text(
            "Bu klasor mp4 ACILAMADIGI icin yedege dusuldugunde olustu.\n"
            "Kareler zaman damgali; tek komutla mp4'e cevrilir:\n\n"
            f"  ffmpeg -framerate {self.fps:g} -i kare_%05d.png \\\n"
            "         -c:v libx264 -pix_fmt yuv420p teslim.mp4\n\n"
            "Sartname md 4.2 mp4 istiyor; donusturup oyle teslim edin.\n",
            encoding="utf-8",
        )

    def yaz(self, kare_rgb: np.ndarray) -> bool:
        """PNG karesi yaz. Disk hatasında False, istisna YOK (F-S.5 deseni)."""
        yol = self.dizin / f"kare_{self.kare_sayisi:05d}.png"
        try:
            Image.fromarray(kare_rgb, mode="RGB").save(yol)
        except OSError:
            return False
        self.kare_sayisi += 1
        return True

    def kapat(self) -> None:
        """PNG serisinde kapatılacak tampon yok — arayüz simetrisi için."""
        return None


class Mp4Yazici:
    """BEV karelerini mp4'e yazar. cv2 TEMBEL import (çekirdek testleri cv2'siz).

    🔑 **fps = ÖRNEKLEME hızı olmalı, oynatma tercihi değil.** Şartname
    "En Az 1 Hz" derken üretim hızını kastediyor; mp4'ün fps'i ise oynatma
    metadata'sıdır. İkisi eşitse video GERÇEK ZAMANLI akar ve hakem ileri
    sararak hızı doğrulayabilir. 1 Hz örnekleyip fps=30 yazmak 20 dakikalık
    görevi 40 saniyeye indirir ve "veri eksik" izlenimi verir.

    ⚠ Varsayılan 2.0 Hz: tam 1.0'da PAY YOK — tek bir atlanan kare (bozuk
    ızgara / disk hatası) teslimi "En Az 1 Hz"in ALTINA düşürür.
    """

    def __init__(
        self,
        yol,
        fps: float = 2.0,
        boyut: Tuple[int, int] = (720, 720),
    ) -> None:
        import cv2                                    # tembel: çekirdek cv2'siz

        self._cv2 = cv2
        self.yol = str(yol)
        self.fps = float(fps)
        if self.fps < 1.0:
            raise ValueError(
                f"fps < 1 Hz şartname ihlali (md 4.2 'En Az 1 Hz'): {fps}"
            )
        self._w = cv2.VideoWriter(
            self.yol, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, boyut
        )
        if not self._w.isOpened():                    # F-P.11 deseni
            raise RuntimeError(
                f"VideoWriter açılamadı: {self.yol} (codec=mp4v). "
                "Teslim dosyası ÜRETİLMEZ — md 5.5.4.3.5: 5 ceza puanı."
            )
        self.kare_sayisi = 0

    def yaz(self, kare_rgb: np.ndarray) -> bool:
        """RGB kareyi yaz (cv2 BGR bekler). Disk hatasında False, istisna YOK."""
        try:
            self._w.write(self._cv2.cvtColor(kare_rgb, self._cv2.COLOR_RGB2BGR))
        except Exception:                             # F-S.5: teslim akışı sürsün
            return False
        self.kare_sayisi += 1
        return True

    def kapat(self) -> None:
        if self._w is not None:
            self._w.release()
            self._w = None

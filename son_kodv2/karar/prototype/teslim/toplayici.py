"""
Girdap İDA — md 4.2 teslim dosyalarını toplayıp USB'ye kopyalayan çekirdek.

ROS-bağımsız, saf dosya sistemi işi (prototype/ kuralı) → pytest ile test edilir.
Mount/unmount ve udev tarafı `scripts/teslim_topla.py`'de.

════════════════════════════════════════════════════════════════════════════
NEDEN OTOMATİK — md 5.5.4.3.5
════════════════════════════════════════════════════════════════════════════
    "İDA'nın karaya alım anından itibaren **20 dakika** içerisinde, her bir
     takımın **kendi USB flash belleği** ile birlikte teslim edilmeyen ...
     **her bir dosya için 5'er ceza puanı**"

20 dakika, tekneyi sudan çıkarma + söküm + Jetson'a erişme ile paylaşılıyor.
Dosyalar **dört ayrı dizine** yazılıyor ve her biri kendi oturum klasöründe:
elle toplamak, koşu sonrası telaşta yapılacak en kırılgan iş. Üstelik md 4.1
gereği yarışma günü **Wi-Fi/BT kapalı ve SSH yok** → Jetson'dan dosya çekmenin
kalan tek pratik yolu **USB takmak**.

🔑 **EN DEĞERLİ ÖZELLİK KOPYALAMA DEĞİL, EKSİK RAPORU.** Kopyalama zaten
saniyeler sürüyor (~50 MB). Asıl kazanç: USB'ye yazılan `RAPOR.txt` hangi
zorunlu kalemin ÜRETİLMEDİĞİNİ söylüyor. O satırı sahada görmek, ceza puanını
hakem masasında öğrenmekten iyidir — ve hâlâ müdahale edilebilecek tek an odur.

════════════════════════════════════════════════════════════════════════════
EMNİYET KURALLARI (kod bunları GARANTİ EDER)
════════════════════════════════════════════════════════════════════════════
· USB'de **hiçbir şey SİLİNMEZ/ÜZERİNE YAZILMAZ** — her koşum kendi
  `GIRDAP_TESLIM_<zaman>/` klasörüne gider. Yanlış USB takılsa bile veri kaybı
  olmaz (birinin kişisel belleği olabilir).
· Kopya öncesi **boş alan kontrolü**; yetmiyorsa hiç başlamaz (yarım teslim,
  hiç teslimden daha kötü: dosya var sanılır).
· Her dosya **sha256 + boyut** ile doğrulanır; uyuşmazsa rapora BOZUK yazılır.
· Kaynak dizinler yalnız OKUNUR.
"""

from __future__ import annotations

import hashlib
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# md 4.2'de TANIMLANAN kalemler. Ceza "tanımlanan her bir dosya için" (md
# 5.5.4.3.5) → bu liste doğrudan ceza yüzeyidir. `zorunlu=False` olan tek
# kalem PNG yedeği: teslim sözleşmesinde yok, kayıpsız yedek olarak taşınıyor.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class KalemTanimi:
    anahtar: str
    ad: str
    madde: str
    alt_dizin: str            # ~/girdap_logs altında
    desen: str                # glob
    zorunlu: bool = True


KALEMLER: Tuple[KalemTanimi, ...] = (
    KalemTanimi(
        "kamera", "Dosya-1 işlenmiş kamera verisi",
        "md 4.2 Dosya 1 (mp4, ≥1 Hz, zaman etiketli, bbox+sınıf)",
        "kamera", "**/*.mp4",
    ),
    KalemTanimi(
        "lidar", "Dosya-1b LiDAR veri seti",
        "md 4.2 Diğer Otonomi Sensörleri (her sensör tipi AYRI dosya)",
        "lidar", "**/*.mp4",
    ),
    KalemTanimi(
        "telemetri", "Dosya-2 araç telemetri verisi",
        "md 4.2 Dosya 2 (csv, ≥1 Hz, header satırlı)",
        "telemetry", "**/telemetri_*.csv",
    ),
    KalemTanimi(
        "harita", "Dosya-3 lokal harita / cost map",
        "md 4.2 Dosya 3 (≥1 Hz)",
        "local_map", "**/*.mp4",
    ),
    KalemTanimi(
        "harita_png", "Dosya-3 PNG yedeği (opsiyonel)",
        "teslim sözleşmesinde YOK — kayıpsız yedek",
        "local_map", "**/*.png", zorunlu=False,
    ),
)


@dataclass
class Bulgu:
    """Bir kalem için bulunanlar."""

    tanim: KalemTanimi
    dosyalar: List[Path] = field(default_factory=list)

    @property
    def bulundu(self) -> bool:
        return len(self.dosyalar) > 0

    @property
    def toplam_bayt(self) -> int:
        return sum(p.stat().st_size for p in self.dosyalar if p.exists())


@dataclass
class Rapor:
    hedef: Optional[Path] = None
    kopyalanan: int = 0
    bayt: int = 0
    eksik_zorunlu: List[str] = field(default_factory=list)
    bozuk: List[str] = field(default_factory=list)
    uyarilar: List[str] = field(default_factory=list)

    @property
    def basarili(self) -> bool:
        """Zorunlu kalemlerin hepsi tam kopyalandıysa True."""
        return not self.eksik_zorunlu and not self.bozuk

    @property
    def tahmini_ceza(self) -> int:
        """md 5.5.4.3.5 — eksik/geçersiz TANIMLI dosya başına 5 puan."""
        return 5 * (len(self.eksik_zorunlu) + len(self.bozuk))


def _en_yeni_oturum(kok: Path) -> Optional[Path]:
    """`kok` altındaki en yeni oturum dizini; alt dizin yoksa `kok`.

    Oturum dizinleri `session_*` / `oturum_*` deseninde. mtime'a göre değil
    **ADA göre** sıralanır: ad zaman damgası taşır ve kopyalama/dokunma
    mtime'ı bozabilir (dosya adı daha güvenilir bir sıralama anahtarıdır).
    """
    if not kok.is_dir():
        return None
    adaylar = sorted(
        (d for d in kok.iterdir()
         if d.is_dir() and (d.name.startswith("oturum_")
                            or d.name.startswith("session_"))),
        key=lambda d: d.name,
    )
    return adaylar[-1] if adaylar else kok


def kalemleri_bul(
    log_koku: Path, hepsi: bool = False
) -> List[Bulgu]:
    """Her tanımlı kalem için dosyaları bul.

    `hepsi=False` (varsayılan): yalnız **en yeni oturum** — teslim tek koşuya
    aittir; eski oturumları da koymak hakemi yanıltır ve süreyi yer.
    `hepsi=True`: tüm oturumlar (koşu sonrası arşivleme için).
    """
    log_koku = Path(log_koku).expanduser()
    out: List[Bulgu] = []
    for t in KALEMLER:
        kok = log_koku / t.alt_dizin
        b = Bulgu(tanim=t)
        if kok.is_dir():
            arama_koku = kok if hepsi else (_en_yeni_oturum(kok) or kok)
            b.dosyalar = sorted(
                p for p in arama_koku.glob(t.desen) if p.is_file()
            )
        out.append(b)
    return out


def _sha256(p: Path, blok: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for parca in iter(lambda: f.read(blok), b""):
            h.update(parca)
    return h.hexdigest()


def kopyala(
    bulgular: List[Bulgu],
    usb_koku: Path,
    zaman_damgasi: Optional[str] = None,
    dogrula: bool = True,
) -> Rapor:
    """Bulunanları USB'ye kopyala. USB'de HİÇBİR ŞEY SİLİNMEZ.

    Hedef: `<usb>/GIRDAP_TESLIM_<zaman>/<kalem>/...`
    """
    usb_koku = Path(usb_koku)
    damga = zaman_damgasi or datetime.now().strftime("%Y%m%d_%H%M%S")
    hedef = usb_koku / f"GIRDAP_TESLIM_{damga}"
    rapor = Rapor(hedef=hedef)

    gereken = sum(b.toplam_bayt for b in bulgular if b.bulundu)
    try:
        bos = shutil.disk_usage(usb_koku).free
    except OSError as e:
        rapor.uyarilar.append(f"USB boş alan okunamadı: {e}")
        bos = None
    # %10 pay: FAT32 küme kaybı + dizin girdileri.
    if bos is not None and gereken * 1.1 > bos:
        rapor.uyarilar.append(
            f"YETERSİZ ALAN: {gereken/1e6:.1f} MB gerekiyor, {bos/1e6:.1f} MB "
            "boş → HİÇ kopyalanmadı (yarım teslim, hiç teslimden kötüdür: "
            "dosya var sanılır)"
        )
        rapor.eksik_zorunlu = [b.tanim.ad for b in bulgular if b.tanim.zorunlu]
        return rapor

    hedef.mkdir(parents=True, exist_ok=True)
    for b in bulgular:
        if not b.bulundu:
            if b.tanim.zorunlu:
                rapor.eksik_zorunlu.append(b.tanim.ad)
            continue
        klasor = hedef / b.tanim.anahtar
        klasor.mkdir(parents=True, exist_ok=True)
        for src in b.dosyalar:
            dst = klasor / src.name
            # Aynı adlı dosya varsa ÜZERİNE YAZMA — sırala.
            n = 1
            while dst.exists():
                dst = klasor / f"{src.stem}_{n}{src.suffix}"
                n += 1
            try:
                shutil.copy2(src, dst)
            except OSError as e:
                rapor.bozuk.append(f"{b.tanim.ad}/{src.name}: kopyalanamadı ({e})")
                continue
            if dogrula:
                if dst.stat().st_size != src.stat().st_size or (
                    _sha256(dst) != _sha256(src)
                ):
                    rapor.bozuk.append(f"{b.tanim.ad}/{src.name}: DOĞRULAMA HATASI")
                    continue
            rapor.kopyalanan += 1
            rapor.bayt += dst.stat().st_size
    return rapor


def rapor_metni(bulgular: List[Bulgu], rapor: Rapor) -> str:
    """USB'ye yazılacak insan-okur rapor. Sahada bakılacak TEK dosya budur."""
    L: List[str] = []
    L.append("GİRDAP İDA — Takım 989124 — md 4.2 VERİ TESLİMİ")
    L.append(f"Toplama zamanı : {datetime.now():%Y-%m-%d %H:%M:%S}")
    L.append(f"Hedef klasör   : {rapor.hedef.name if rapor.hedef else '-'}")
    L.append(f"Kopyalanan     : {rapor.kopyalanan} dosya, "
             f"{rapor.bayt/1e6:.1f} MB")
    L.append("")
    L.append("KALEMLER (md 4.2):")
    for b in bulgular:
        isaret = "OK " if b.bulundu else ("EKSİK" if b.tanim.zorunlu else "yok")
        L.append(f"  [{isaret:5s}] {b.tanim.ad}")
        L.append(f"           {b.tanim.madde}")
        if b.bulundu:
            L.append(f"           {len(b.dosyalar)} dosya, "
                     f"{b.toplam_bayt/1e6:.1f} MB")
    L.append("")
    if rapor.eksik_zorunlu:
        L.append("🔴 EKSİK ZORUNLU DOSYALAR:")
        for ad in rapor.eksik_zorunlu:
            L.append(f"   - {ad}")
    if rapor.bozuk:
        L.append("🔴 BOZUK/DOĞRULANAMAYAN:")
        for s in rapor.bozuk:
            L.append(f"   - {s}")
    for u in rapor.uyarilar:
        L.append(f"⚠ {u}")
    L.append("")
    if rapor.basarili:
        L.append("✅ TÜM ZORUNLU KALEMLER TESLİME HAZIR.")
    else:
        L.append(f"🔴 md 5.5.4.3.5'e göre TAHMİNİ CEZA: "
                 f"{rapor.tahmini_ceza} puan "
                 f"(eksik/geçersiz dosya başına 5)")
    L.append("")
    L.append("Not: bu klasör kopyalandıktan sonra USB güvenle çıkarılabilir;")
    L.append("betik kopyayı bitirince diski senkronize edip ayırır.")
    return "\n".join(L)


def manifest_metni(rapor: Rapor, hedef: Path) -> str:
    """sha256 manifest — teslim sonrası bütünlük iddiası için."""
    L = ["# sha256  boyut  dosya"]
    for p in sorted(hedef.rglob("*")):
        if p.is_file() and p.name not in ("MANIFEST.txt", "RAPOR.txt"):
            L.append(f"{_sha256(p)}  {p.stat().st_size}  "
                     f"{p.relative_to(hedef)}")
    return "\n".join(L)


def topla_ve_yaz(
    log_koku: Path,
    usb_koku: Path,
    hepsi: bool = False,
    zaman_damgasi: Optional[str] = None,
) -> Tuple[Rapor, List[Bulgu]]:
    """Uçtan uca: bul → kopyala → RAPOR.txt + MANIFEST.txt yaz."""
    bulgular = kalemleri_bul(log_koku, hepsi=hepsi)
    rapor = kopyala(bulgular, usb_koku, zaman_damgasi=zaman_damgasi)
    if rapor.hedef is not None and rapor.hedef.is_dir():
        (rapor.hedef / "RAPOR.txt").write_text(
            rapor_metni(bulgular, rapor), encoding="utf-8"
        )
        (rapor.hedef / "MANIFEST.txt").write_text(
            manifest_metni(rapor, rapor.hedef), encoding="utf-8"
        )
    return rapor, bulgular

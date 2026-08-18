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
· USB'de **hiçbir şey SİLİNMEZ**. Yanlış USB takılsa bile (birinin kişisel
  belleği olabilir) veri kaybı olmaz. Kökte önceki koşumun kalemleri varsa
  `onceki_kosum_<zaman>/` altına **taşınır** — silinmez; hakemin kökte gördüğü
  daima SON koşumdur (md 5.5.3.1: 1 yeniden başlama hakkı → aynı USB'ye iki
  koşum düşebilir).
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

from prototype.mapping.bev_renderer import (
    FPS_ISARET_ADI, PNG_KARE_DESENI,
)

# ═══════════════════════════════════════════════════════════════════════════
# USB YERLEŞİMİ — ADLAR ŞARTNAMEDEN BİREBİR, UYDURMA YOK
# ═══════════════════════════════════════════════════════════════════════════
# Şartname md 4.2 klasör yapısı TARİF ETMİYOR; yalnız kalemleri ve formatları
# adlandırıyor. Bu yüzden buradaki tek kural: **belgenin kendi başlıkları**
# kullanılır, fazladan hiyerarşi ya da numara ICAT EDİLMEZ.
#
# Belgedeki dört madde (girinti sütunları ölçüldü — dördü de AYNI seviyede;
# "Dosya 1"in kaymış görünmesi sayfa kırılmasından):
#     ▪ Dosya 1: Otonomi Sensörleri Veri seti   → • İşlenmiş kamera verisi
#     ▪ Diğer Otonomi Sensörleri Veri Seti      → lidar vs., HER SENSÖR TİPİ AYRI
#     ▪ Dosya 2: Araç telemetri verisi
#     ▪ Dosya 3: Lokal harita/cost map/engel haritası
# ⚠ Giriş cümlesi "Veriler 3 dosya olacak şekilde" diyor ama madde DÖRT.
#   Büyük olasılıkla "Diğer Otonomi Sensörleri" sonradan eklenip numaralandırma
#   güncellenmemiş. Yerleşim her iki okumada da çalışacak şekilde kuruldu:
#   kök dizinde tam olarak bu dört başlık görünür, başka hiçbir şey.
#
# 🔑 İlk iki kalem KLASÖR, son ikisi TEK DOSYA — bu bir tercih değil verinin
#   dayattığı şey: kamera kaydedicisi segmentler yazıyor (seg_0000.mp4 …) ve
#   "her bir sensör tipi için ayrı ayrı" birden çok sensör dosyası demek.
#   Dosya-2 ve Dosya-3 tek dosya olduğu için düz duruyor.
#
# Türkçe karakter KULLANILMADI: USB'ler FAT32/exFAT ve dosya adı kodlaması
# sistemler arasında bozulabiliyor; hakemin makinesinde okunamayan ad riski
# alınmaz.
# ═══════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class KalemTanimi:
    anahtar: str
    ad: str                   # şartnamedeki başlık (rapor metni için)
    madde: str
    alt_dizin: str            # ~/girdap_logs altında (KAYNAK)
    desen: str                # glob
    hedef: str                # USB kökündeki ad — ŞARTNAMEDEN
    klasor: bool              # True → klasör (çok dosya), False → tek dosya
    zorunlu: bool = True


KALEMLER: Tuple[KalemTanimi, ...] = (
    KalemTanimi(
        "kamera",
        "Dosya 1: Otonomi Sensörleri Veri seti — İşlenmiş kamera verisi",
        "md 4.2 (mp4, ≥1 Hz, her kare zaman etiketli, bbox + sınıf)",
        "kamera", "**/*.mp4",
        hedef="Dosya1_Otonomi_Sensorleri_Veri_Seti", klasor=True,
    ),
    KalemTanimi(
        "lidar",
        "Diğer Otonomi Sensörleri Veri Seti (lidar)",
        "md 4.2 (her sensör tipi AYRI, mp4, ≥1 Hz, zaman etiketli, "
        "kümeleme/ayırma görünür)",
        "lidar", "**/*.mp4",
        hedef="Diger_Otonomi_Sensorleri_Veri_Seti", klasor=True,
    ),
    # 🔴 mp4 açılamayıp PNG'ye düşüldüyse (codec yok) kareler BURADA olur.
    # Toplanmazsa LiDAR teslimi tamamen kaybolur — mp4 zaten yok, PNG de
    # USB'ye gitmezse geriye hiçbir şey kalmaz. `zorunlu=False` çünkü normal
    # koşumda bu klasör HİÇ oluşmaz; oluştuysa rapor ayrıca bağırır.
    KalemTanimi(
        "lidar_png_yedek",
        "⚠ Diğer Otonomi Sensörleri — mp4 AÇILAMADI, PNG yedeği",
        "TESLİMDEN ÖNCE mp4'E ÇEVİR (klasördeki NASIL_MP4_YAPILIR.txt)",
        "lidar", "**/*.png",
        hedef="Diger_Otonomi_Sensorleri_PNG_YEDEK_mp4e_cevrilecek",
        klasor=True, zorunlu=False,
    ),
    KalemTanimi(
        "telemetri",
        "Dosya 2: Araç telemetri verisi",
        "md 4.2 (csv, ≥1 Hz, ilk satır header)",
        "telemetry", "**/telemetri_*.csv",
        hedef="Dosya2_Arac_Telemetri_Verisi.csv", klasor=False,
    ),
    KalemTanimi(
        "harita",
        "Dosya 3: Lokal harita/cost map/engel haritası",
        "md 4.2 (≥1 Hz; FORMAT ŞARTNAMEDE BELİRTİLMEMİŞ — diğer görsel "
        "kalemlerle tutarlı olsun diye mp4)",
        "local_map", "**/*.mp4",
        hedef="Dosya3_Lokal_Harita_Cost_Map_Engel_Haritasi.mp4", klasor=False,
    ),
    KalemTanimi(
        "harita_png",
        "Dosya 3 — kayıpsız PNG yedeği",
        "şartnamede YOK; mp4 kabul edilmezse elde kalsın diye taşınır",
        "local_map", "**/*.png",
        hedef="Dosya3_png_yedek", klasor=True, zorunlu=False,
    ),
)

# Bizim kendi kontrol dosyalarımız — teslim kalemi DEĞİL. Alt çizgiyle
# başlıyorlar ki listede başta dursunlar ve hakem bunları teslim sanmasın.
RAPOR_ADI = "_GIRDAP_kontrol_raporu.txt"
MANIFEST_ADI = "_GIRDAP_manifest_sha256.txt"


@dataclass
class Bulgu:
    """Bir kalem için bulunanlar."""

    tanim: KalemTanimi
    dosyalar: List[Path] = field(default_factory=list)
    #: Tek-dosya kaleminde EN YENİ seçilince dışarıda kalanlar (yalnız rapor
    #: için; USB'ye kopyalanmazlar). Boş liste = eleme yapılmadı.
    elenen: List[Path] = field(default_factory=list)

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


def png_yedegini_mp4_yap(log_koku: Path) -> List[str]:
    """PNG'ye düşmüş kayıtları **ffmpeg ile otomatik mp4'e çevir**.

    🔑 **NEDEN OTOMATİK — insan adımı teslim zincirinde en zayıf halka.**
    Kaydedici mp4 codec'ini açamazsa PNG'ye düşüyor (kayıt kurtuluyor), ama
    şartname md 4.2 **`mp4 formatında`** istiyor. Dönüşüm elle bırakılsaydı
    sıra şöyle olurdu: koşum biter → 20 dakikalık teslim saati başlar → tekne
    sudan çıkarılır → USB takılır → **birinin `RAPOR.txt`'yi açıp okuması,
    klasörü bulması, ffmpeg komutunu koşması** → teslim. O akışta durup dosya
    okuyan bir adım YOK; uyarı üç yere de yazılsa görülmez ve PNG teslim edilir
    = **5 ceza** (md 5.5.4.3.5). Bu yüzden dönüşüm buraya, USB takma anına
    bağlandı: insan yalnız USB takıyor.

    Dönüşüm **kaynakta** (`~/girdap_logs`) yapılır, USB'de değil:
      · `kalemleri_bul` sonra normal mp4'ü bulur → kopya/rapor/manifest yolu
        hiç özel duruma girmez (tek kod yolu = az hata);
      · mp4 Jetson'da da kalır, USB çıkarılsa bile kaybolmaz;
      · USB genelde FAT32 ve yavaş — yazma değil okuma yapıyoruz.

    ⚠ PNG'ler **SİLİNMEZ** (kayıpsız kaynak). Dönüşüm başarılıysa `kalemleri_bul`
    onları listeye almaz — USB kökü şartnamenin dört kalemiyle temiz kalır.

    Döner: kullanıcıya gösterilecek bilgi/uyarı satırları.
    """
    import shutil as _sh
    import subprocess

    mesajlar: List[str] = []
    kok = Path(log_koku).expanduser() / "lidar"
    if not kok.is_dir():
        return mesajlar
    for png_dizin in sorted(kok.glob("**/lidar_kumeleme_png")):
        if not png_dizin.is_dir():
            continue
        hedef = png_dizin.parent / "lidar_kumeleme.mp4"
        if hedef.exists():
            continue                                  # zaten mp4 var
        kareler = sorted(png_dizin.glob("kare_*.png"))
        if not kareler:
            continue
        if _sh.which("ffmpeg") is None:
            mesajlar.append(
                f"🔴 {png_dizin.name}: {len(kareler)} PNG karesi var ama "
                "ffmpeg KURULU DEĞİL → mp4'e çevrilemedi. PNG'ler USB'ye "
                "alındı; şartname mp4 istiyor (md 4.2), başka makinede "
                "çevirin. Jetson'a `sudo apt install ffmpeg` kalıcı çözüm."
            )
            continue
        # fps'i kaydedicinin bıraktığı makine-okur işaretten al.
        fps = 2.0
        isaret = png_dizin / FPS_ISARET_ADI
        if isaret.is_file():
            try:
                fps = float(isaret.read_text(encoding="utf-8").strip())
            except ValueError:
                pass                                   # varsayılanla devam
        # libx264 her ffmpeg derlemesinde YOK; yoksa mpeg4'e düş. İkisi de
        # olmazsa PNG yolu zaten yedek olarak duruyor.
        for kodek in ("libx264", "mpeg4"):
            komut = [
                "ffmpeg", "-y", "-loglevel", "error",
                "-framerate", f"{fps:g}",
                "-i", str(png_dizin / PNG_KARE_DESENI),
                "-c:v", kodek, "-pix_fmt", "yuv420p", str(hedef),
            ]
            try:
                r = subprocess.run(komut, capture_output=True, timeout=300)
            except (OSError, subprocess.TimeoutExpired) as e:
                mesajlar.append(f"🔴 ffmpeg çalıştırılamadı ({kodek}): {e}")
                break
            if r.returncode == 0 and hedef.exists() and hedef.stat().st_size > 0:
                mesajlar.append(
                    f"✅ mp4 codec'i yoktu → {len(kareler)} PNG karesi ffmpeg "
                    f"({kodek}, {fps:g} fps) ile OTOMATİK mp4'e çevrildi. "
                    "Elle yapılacak bir şey YOK."
                )
                break
            hedef.unlink(missing_ok=True)              # yarım dosya bırakma
        else:
            mesajlar.append(
                f"🔴 {png_dizin.name}: ffmpeg mp4 üretemedi (libx264 ve mpeg4 "
                "denendi). PNG'ler USB'ye alındı, elle çevrilmeli."
            )
    return mesajlar


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


def mp4_oynatilabilir_mi(yol: Path) -> bool:
    """mp4 dosyasında `moov` kutusu var mı? (bağımlılıksız üst düzey tarama)

    🔴 NEDEN — 17.08.2026'da ÖLÇÜLDÜ, sistematik: kamera kaydedicisi
    segmentler yazıyor ve süreç temiz kapanmadan ölürse **son segmentin `moov`
    atomu yazılmaz** ⇒ o dosya HİÇBİR oynatıcıda açılmaz. Bu makinedeki
    kayıtlarda **37 oturumun 28'inde (%76) son segment bozuk** çıktı.
    Yarışmada son segment koşunun SONUNU taşır (son kapılar, P3 teması) ve
    md 4.2'ye göre teslim edilmeyen her dosya **5 ceza puanı**.

    `dosya1_birlestir.py` bozuk segmenti zaten doğrulayıp atlıyor — **ama o
    betik teslim yolunda ÇAĞRILMIYOR** (17.08'de doğrulandı): `teslim_topla.py`
    ham segmentleri kopyalıyor, bozuk olan da USB'ye gidiyor. Bu yüzden
    doğrulama teslim tarafına da kondu.

    ⚠️ Bu fonksiyon **kopyalamayı ENGELLEMEZ** — bozuk dosya yine USB'ye gider
    (silmek/atlamak bilgi kaybıdır, üstelik hakem isterse ham veriyi görmeli).
    Yaptığı tek şey **rapora yazmak**: modülün kendi felsefesi (*"en değerli
    özellik kopyalama değil, EKSİK RAPORU"*) burada da geçerli — operatör
    USB'yi teslim etmeden ÖNCE görsün.

    Yöntem: cv2/ffprobe yok (bağımlılık + hız). mp4 üst düzey kutu zinciri
    yürünür (`[4 bayt boyut][4 bayt tip]`), `moov` tipi aranır. Bozuk/kesik
    dosyada zincir erken biter ve `moov` hiç görünmez.
    """
    try:
        boyut = yol.stat().st_size
        if boyut < 16:
            return False
        with open(yol, "rb") as f:
            konum = 0
            for _ in range(64):              # üst düzeyde 64 kutu fazlasıyla yeter
                f.seek(konum)
                bas = f.read(8)
                if len(bas) < 8:
                    return False
                kutu = int.from_bytes(bas[:4], "big")
                tip = bas[4:8]
                if tip == b"moov":
                    return True
                if kutu == 1:                # 64-bit genişletilmiş boyut
                    genis = f.read(8)
                    if len(genis) < 8:
                        return False
                    kutu = int.from_bytes(genis, "big")
                elif kutu == 0:              # "dosya sonuna kadar" — moov gelmedi
                    return False
                if kutu < 8:
                    return False
                konum += kutu
                if konum >= boyut:
                    return False
        return False
    except OSError:
        return False


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
            # 🔴 16.08.2026 — DÜZ YERLEŞİMLİ TEK-DOSYA KALEMİ (Dosya-2) SIZIYORDU.
            # `_en_yeni_oturum` yalnız `oturum_*`/`session_*` ALT DİZİNİ arar.
            # `local_map` öyle yazıyor (çalışıyor), ama `telemetry` dosyaları
            # dizine DÜZ yazılıyor (`telemetri_<UTC>.csv`) ⇒ alt dizin yok ⇒
            # `_en_yeni_oturum` kök'ü döndürüyor ⇒ "yalnız en yeni oturum"
            # kuralı Dosya-2'ye HİÇ uygulanmıyordu. Bu cihazda ölçüldü:
            # **127 CSV** toplandı ve `dosyalar[0]` (ada göre EN ESKİ) şartname
            # adını (`Dosya2_Arac_Telemetri_Verisi.csv`) aldı ⇒ hakem resmî ad
            # altında AYLAR ÖNCEKİ geliştirme koşusunu görürdü. Rapor "elle
            # seç" diyordu ama 20 dakikalık teslim penceresinde (md 4.2, geç
            # dosya 5 ceza) o el hareketi tam da atlanacak adımdır.
            # Aynı sınıf: `dosya1_birlestir` isim sırası → PROVA oturumu teslimi.
            if not hepsi and not t.klasor and len(b.dosyalar) > 1:
                b.elenen = b.dosyalar[:-1]
                b.dosyalar = b.dosyalar[-1:]     # ada göre EN YENİ
        out.append(b)
    # 🔑 mp4 varsa PNG yedeğini teslime KOYMA: dönüşüm başarılıysa (ya da
    # codec zaten çalışıyorsa) o klasör yalnız kayıpsız kaynaktır ve USB
    # kökünü kirletir. PNG'ler Jetson'da duruyor, silinmiyor.
    d = {b.tanim.anahtar: b for b in out}
    if d["lidar"].bulundu and "lidar_png_yedek" in d:
        d["lidar_png_yedek"].dosyalar = []
    return out


def _sha256(p: Path, blok: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for parca in iter(lambda: f.read(blok), b""):
            h.update(parca)
    return h.hexdigest()


def _onceki_kosumu_arsivle(usb_koku: Path, damga: str) -> Optional[str]:
    """Kökte önceki koşumun kalemleri varsa onları bir yana AL (SİLME).

    Şartname 1 yeniden başlama hakkı veriyor (md 5.5.3.1) → aynı USB'ye iki
    koşum düşebilir. Hakemin kökte gördüğü şey DAİMA son koşum olmalı; eski
    dosyalar karışırsa hangi koşuma ait olduğu belirsizleşir.
    ⚠ Taşınır, ASLA silinmez.
    """
    mevcut = [
        usb_koku / t.hedef for t in KALEMLER if (usb_koku / t.hedef).exists()
    ]
    if not mevcut:
        return None
    arsiv = usb_koku / f"onceki_kosum_{damga}"
    arsiv.mkdir(parents=True, exist_ok=True)
    for p in mevcut:
        shutil.move(str(p), str(arsiv / p.name))
    return arsiv.name


def kopyala(
    bulgular: List[Bulgu],
    usb_koku: Path,
    zaman_damgasi: Optional[str] = None,
    dogrula: bool = True,
) -> Rapor:
    """Bulunanları USB KÖKÜNE, ŞARTNAMEDEKİ adlarla kopyala.

    Kökte tam olarak şunlar görünür (başka hiçbir şey):
        Dosya1_Otonomi_Sensorleri_Veri_Seti/
        Diger_Otonomi_Sensorleri_Veri_Seti/
        Dosya2_Arac_Telemetri_Verisi.csv
        Dosya3_Lokal_Harita_Cost_Map_Engel_Haritasi.mp4
        _GIRDAP_kontrol_raporu.txt / _GIRDAP_manifest_sha256.txt  (bizim)
    Sarmalayıcı klasör ICAT EDİLMEZ — şartname öyle bir şey istemiyor.
    """
    usb_koku = Path(usb_koku)
    damga = zaman_damgasi or datetime.now().strftime("%Y%m%d_%H%M%S")
    rapor = Rapor(hedef=usb_koku)

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

    arsiv = _onceki_kosumu_arsivle(usb_koku, damga)
    if arsiv:
        rapor.uyarilar.append(
            f"Önceki koşumun dosyaları '{arsiv}/' altına ALINDI (silinmedi) — "
            "kökte daima SON koşum görünür"
        )

    for b in bulgular:
        if not b.bulundu:
            if b.tanim.zorunlu:
                rapor.eksik_zorunlu.append(b.tanim.ad)
            continue
        t = b.tanim
        if t.klasor:
            hedef_dizin = usb_koku / t.hedef
            hedef_dizin.mkdir(parents=True, exist_ok=True)
            ciftler = [(s, hedef_dizin / s.name) for s in b.dosyalar]
        else:
            # Tek dosya kalemi: şartnamedeki adı taşır. Birden fazla kaynak
            # varsa (beklenmez) ilki ada, kalanlar sıralı eke gider — hiçbiri
            # kaybolmasın.
            hedef_dizin = usb_koku
            ana = usb_koku / t.hedef
            ciftler = [(b.dosyalar[0], ana)]
            for i, s in enumerate(b.dosyalar[1:], start=1):
                ciftler.append((s, usb_koku / f"{ana.stem}_{i}{ana.suffix}"))
            if len(b.dosyalar) > 1:
                rapor.uyarilar.append(
                    f"{t.ad}: {len(b.dosyalar)} kaynak dosya bulundu, "
                    "beklenen 1 — hepsi kopyalandı, hakeme HANGİSİ verilecek "
                    "elle seçilmeli"
                )
            if b.elenen:
                # Sessiz seçim YAPMIYORUZ: hangi dosyanın şartname adını aldığı
                # ve kaçının elendiği rapora YAZILIR. Operatör yanlış koşunun
                # teslim edildiğini ancak burada fark edebilir.
                rapor.uyarilar.append(
                    f"{t.ad}: {len(b.elenen) + 1} aday vardı, EN YENİSİ seçildi "
                    f"→ {b.dosyalar[0].name} (elenen {len(b.elenen)} dosya USB'ye "
                    "KOPYALANMADI, Jetson'da duruyor). Yanlışsa: --hepsi ile "
                    "tekrar topla ve doğru dosyayı elle adlandır."
                )
                if "SAAT-GUVENILMEZ" in b.dosyalar[0].name:
                    # Seçim ADA göre; ad zaman damgası taşıyor. Saat güvenilmezse
                    # "en yeni ad" ≠ "en son koşu" olabilir (§ saat kaynağı).
                    rapor.uyarilar.append(
                        f"🔴 {t.ad}: seçilen dosya SAAT-GUVENILMEZ damgalı — "
                        "sıralama ada (zaman damgasına) dayandığı için seçim de "
                        "şüpheli. Teslimden önce dosyanın içeriğini GÖZLE doğrula."
                    )
        for src, dst in ciftler:
            n = 1
            while dst.exists():                # ÜZERİNE YAZMA
                dst = dst.with_name(f"{dst.stem}_{n}{dst.suffix}")
                n += 1
            try:
                shutil.copy2(src, dst)
            except OSError as e:
                rapor.bozuk.append(f"{t.ad}/{src.name}: kopyalanamadı ({e})")
                continue
            if dogrula:
                if dst.stat().st_size != src.stat().st_size or (
                    _sha256(dst) != _sha256(src)
                ):
                    rapor.bozuk.append(f"{t.ad}/{src.name}: DOĞRULAMA HATASI")
                    continue
            rapor.kopyalanan += 1
            rapor.bayt += dst.stat().st_size
    return rapor


def rapor_metni(bulgular: List[Bulgu], rapor: Rapor) -> str:
    """USB'ye yazılacak insan-okur rapor. Sahada bakılacak TEK dosya budur."""
    L: List[str] = []
    L.append("GİRDAP İDA — Takım 989124 — md 4.2 VERİ TESLİMİ")
    L.append(f"Toplama zamanı : {datetime.now():%Y-%m-%d %H:%M:%S}")
    L.append(f"USB kökü       : {rapor.hedef if rapor.hedef else '-'}")
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
    # 🔴 En kritik saha uyarısı: mp4 yok ama PNG yedeği var → teslim
    # KURTARILABİLİR ama ÇEVİRMEK gerekiyor. Bu satır görülmezse 5 ceza.
    d = {b.tanim.anahtar: b for b in bulgular}
    if not d["lidar"].bulundu and d["lidar_png_yedek"].bulundu:
        L.append("╔" + "═" * 68 + "╗")
        L.append("║ 🔴 LiDAR mp4 YOK ama PNG YEDEĞİ VAR — ÇEVİRMEDEN TESLİM ETME!     ║")
        L.append("║ Kayıt sırasında mp4 codec'i açılamamış, PNG'ye düşülmüş.          ║")
        L.append("║ USB'deki 'Diger_Otonomi_Sensorleri_PNG_YEDEK_mp4e_cevrilecek/'    ║")
        L.append("║ klasöründe NASIL_MP4_YAPILIR.txt tek satırlık komutu veriyor.     ║")
        L.append("║ Çevirmezsen md 4.2 'mp4 formatında' şartı sağlanmaz = 5 ceza.     ║")
        L.append("╚" + "═" * 68 + "╝")
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
    L.append("USB KÖKÜNDE HAKEME GÖSTERİLECEKLER (adlar şartname md 4.2'den):")
    for t in KALEMLER:
        if t.zorunlu:
            L.append(f"  {t.hedef}")
    L.append("")
    L.append("Bu dosya ve _GIRDAP_manifest_sha256.txt TESLİM KALEMİ DEĞİLDİR;")
    L.append("kendi kontrolümüz için yazıldı (adları alt çizgiyle başlıyor).")
    L.append("Betik kopyayı bitirince diski senkronize edip ayırır —")
    L.append("USB'nin LED'i sönünce güvenle çıkarılabilir.")
    return "\n".join(L)


def manifest_metni(rapor: Rapor, hedef: Path) -> str:
    """sha256 manifest — teslim sonrası bütünlük iddiası için."""
    L = ["# sha256  boyut  dosya"]
    for p in sorted(hedef.rglob("*")):
        if p.is_file() and p.name not in (MANIFEST_ADI, RAPOR_ADI):
            L.append(f"{_sha256(p)}  {p.stat().st_size}  "
                     f"{p.relative_to(hedef)}")
    return "\n".join(L)


def topla_ve_yaz(
    log_koku: Path,
    usb_koku: Path,
    hepsi: bool = False,
    zaman_damgasi: Optional[str] = None,
) -> Tuple[Rapor, List[Bulgu]]:
    """Uçtan uca: **PNG→mp4 otomatik çevir** → bul → kopyala → rapor+manifest.

    Dönüşüm ÖNCE yapılır ki `kalemleri_bul` normal bir mp4 bulsun; böylece
    kopyalama/rapor/manifest yolu özel duruma hiç girmez.
    """
    cevrim_mesajlari = png_yedegini_mp4_yap(log_koku)
    bulgular = kalemleri_bul(log_koku, hepsi=hepsi)
    rapor = kopyala(bulgular, usb_koku, zaman_damgasi=zaman_damgasi)
    rapor.uyarilar[:0] = cevrim_mesajlari      # dönüşüm notları en üstte

    # 🔴 17.08 — TESLİM EDİLEN mp4'ler GERÇEKTEN AÇILIYOR MU (bkz.
    # `mp4_oynatilabilir_mi`). Bu makinedeki kayıtlarda 37 oturumun 28'inde
    # (%76) SON segmentin `moov` atomu yoktu ⇒ o dosya hiçbir oynatıcıda
    # açılmıyor. Kopyalamayı ENGELLEMİYORUZ (ham veri hakemde kalsın), yalnız
    # rapora yazıyoruz — operatör USB'yi teslim etmeden önce görsün.
    for b in bulgular:
        bozuk = [x for x in b.dosyalar
                 if x.suffix.lower() == ".mp4" and not mp4_oynatilabilir_mi(x)]
        if not bozuk:
            continue
        adlar = ", ".join(x.name for x in bozuk[:4])
        if len(bozuk) > 4:
            adlar += f" … (+{len(bozuk) - 4})"
        mp4ler = [x for x in b.dosyalar if x.suffix.lower() == ".mp4"]
        hepsi_bozuk = len(bozuk) == len(mp4ler)
        rapor.uyarilar.append(
            f"🔴 {b.tanim.ad}: {len(bozuk)}/{len(mp4ler)} mp4 AÇILMIYOR "
            f"(moov atomu yok — sürec temiz kapanmadan olmus): {adlar}. "
            "USB'ye yine de kopyalandi. Genelde SON segment bozulur; kaydin "
            "geri kalani saglamdir. Birlestirilmis tek dosya isteniyorsa "
            "`algi/scripts/dosya1_birlestir.py` bozuk segmenti ATLAYIP birlestirir."
        )
        # 🔴 AYRIM: bir kalemin mp4'lerinin HEPSİ açılmıyorsa o kalem fiilen
        # TESLİM EDİLMEMİŞTİR (md 4.2: teslim edilmeyen her dosya 5 ceza).
        # 36 segmentin 1'i bozuksa kayıt duruyor — uyarı yeter. Ama tek dosyalık
        # bir kalemin o tek dosyası açılmıyorsa rapor "HAZIR" DEMEMELİ.
        # 17.08'de bu ayrım olmadan rapor ✅ diyordu, oysa Dosya-3'ün ve
        # Dosya-1b'nin (lidar) TEK mp4'ü de açılmıyordu — yanlış yeşil.
        if hepsi_bozuk and b.tanim.zorunlu:
            rapor.bozuk.append(
                f"{b.tanim.ad}: mp4'lerin HEPSİ açılmıyor ({len(bozuk)}/"
                f"{len(mp4ler)}) — bu kalem fiilen TESLİM EDİLMEMİŞ sayılır"
            )
    kok = Path(usb_koku)
    if kok.is_dir():
        (kok / RAPOR_ADI).write_text(
            rapor_metni(bulgular, rapor), encoding="utf-8"
        )
        (kok / MANIFEST_ADI).write_text(
            manifest_metni(rapor, kok), encoding="utf-8"
        )
    return rapor, bulgular

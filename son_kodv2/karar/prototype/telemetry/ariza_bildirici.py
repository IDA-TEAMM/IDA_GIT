"""
Girdap İDA — ARIZA KODLARININ YER KONTROL İSTASYONUNA BİLDİRİLMESİ (ROS'SUZ).

Neden bu dosya var (2026-08-13, kaptan isteği):
    Kaptan: *"pixhawk'a çıkan şeylerin hata kodunu at — 'lidar bağlanmadı'
    ya da 'RRT reddetti' gibi; ben mesajlarda onu göreyim."*

    Ölçülen durum (13.08.2026, 02:09, canlı yığın): LiDAR ağı düşmüştü,
    `planning_node` saniyede bir *"engel haritası HİÇ gelmedi → MPPI
    DURDURULDU"* basıyordu — ama **yalnız ROS günlüğüne**. Yer kontrol
    istasyonuna giden `/mavros/statustext/send` hattında **12 saniye
    boyunca tek mesaj yoktu**. Kıyıdaki operatörün elinde yalnız Mission
    Planner var; teknenin niye durduğunu görmesinin hiçbir yolu yoktu.

    `fsm_node` zaten görev durumunu bu hattan bildiriyor (ve `TAKILDI
    BASLAT-YOK` gibi kısa kodlar üretiyor) ama **alt sistem arızaları** o
    hatta hiç girmiyordu. Bu modül o boşluğu kapatır.

Tasarım kısıtları (üçü de ölçülmüş gerçeklerden geliyor):

1. **50 karakter.** MAVLink STATUSTEXT metni 50 karakterle sınırlı; taşan
   kısım sessizce kesilir. Bütün kodlar+metinler bu sınıra göre seçildi ve
   `test_ariza_bildirici.py` sınırı donduruyor.

2. **Telsiz bütçesi dar.** §10.1: 868 MHz telsizin hava hızı ~2,1 KB/s ve
   16.07'de uçuş kontrolcüsü hattı doldurunca *"komut kabul etmiyor"*
   arızası yaşandı. Bu yüzden bildirici **her tick'te göndermez**: yalnız
   (a) en öncelikli arıza DEĞİŞTİĞİNDE ve (b) değişmese de `tazeleme_s`'de
   bir tekrar eder. (b) gerekli çünkü operatör Mission Planner'ı arıza
   başladıktan sonra açarsa ekranı boş kalır — `fsm_node`'un aynı gerekçesi.

3. **ASCII.** Mission Planner'ın mesaj akışında Türkçe harfler bozuk
   görünebiliyor; buradaki metinler bilerek aksansız yazıldı. Bu, §0.31b'nin
   (kısaltma/çevrilmemiş terim yasağı) istisnası değil — kural kullanıcıya
   yazılan metin için; burası 50 baytlık bir telsiz çerçevesi.

Kullanım (düğüm tarafı):
    bildirici = ArizaBildirici(tazeleme_s=20.0)
    ...
    bildirici.bildir(ENGEL_YOK)         # arıza sürüyor (her tick çağrılabilir)
    bildirici.temizle(ENGEL_YOK)        # arıza düştü
    gonderim = bildirici.gonderilecek(simdi)
    if gonderim is not None:
        metin, seviye = gonderim
        ...  # StatusText olarak yayınla

ROS/rclpy/mavros GEREKTİRMEZ — seviye sayıları MAVLink'in kendi sabitleri
(mavros_msgs/StatusText ile birebir aynı: CRITICAL=2, ERROR=3, WARNING=4).
"""

from __future__ import annotations

from dataclasses import dataclass

# MAVLink SEVERITY sabitleri — mavros_msgs/StatusText ile birebir doğrulandı
# (13.08.2026: EMERGENCY=0 ALERT=1 CRITICAL=2 ERROR=3 WARNING=4 NOTICE=5).
# Burada elle yazılıyor ki çekirdek mavros'suz da koşsun; `test_ariza_
# bildirici.py` mavros varsa iki tarafı karşılaştırıp ayrışmayı yakalar.
SEVIYE_CRITICAL = 2
SEVIYE_ERROR = 3
SEVIYE_WARNING = 4
SEVIYE_NOTICE = 5

# Her satırın başına konur ki Mission Planner akışında teknenin kendi
# mesajları uçuş kontrolcüsünün mesajlarından ayırt edilebilsin.
ONEK = "GIRDAP"

# MAVLink STATUSTEXT metin sınırı.
AZAMI_UZUNLUK = 50

# Bütün arıza aktifken temizlenince bir kez basılan "her şey yolunda" satırı.
TEMIZ_METNI = f"{ONEK} ariza yok"


@dataclass(frozen=True)
class ArizaTanimi:
    """Tek bir arıza kodunun sabit tanımı."""

    kod: str
    metin: str
    seviye: int

    def statustext(self) -> str:
        """Telsize çıkacak tam satır (`GIRDAP <KOD> <metin>`)."""
        return f"{ONEK} {self.kod} {self.metin}"


# --------------------------------------------------------------- kod defteri
#
# ⚠ SIRA ÖNEMLİ: aynı seviyedeki iki arıza aynı anda aktifse buradaki sıra
# kazanır (üstteki daha kritiktir). Sıra "arızayı gören operatör önce neye
# bakmalı" mantığına göre dizildi.

KONTROL_HATA = ArizaTanimi(
    "KONTROL", "kontrol coktu, motor durdu", SEVIYE_CRITICAL
)
KONTROLCU_YOK = ArizaTanimi(
    "KONTROLCU", "kontrolcu hazir degil", SEVIYE_ERROR
)
# 🔴 F-F.1 (14.08.2026, §0.98) — POZ-YOK/POZ-BAYAT'IN ÜSTÜNDE, ÇÜNKÜ DAHA SİNSİ.
# Ölçülen olay: `/girdap/fusion/pose` iSAM2 diverjansıyla **10¹⁴⁹** mertebesine
# çıktı; poz TAZE ve DÜZENLİ (10 Hz) akıyordu, yalnızca ANLAMSIZDI. Üç mevcut
# kapı da (KAR-05 "hiç girdi yok" · F8.2 "bayat" · F-P.7 "hız bayat") tazelik
# ölçtüğü için hiçbiri görmedi: `inhibit_reason` bütün koşum boyunca **`YOK`**
# dedi, yani sistem "sürmemem için sebep yok" derken hiç sürmüyordu.
# Operatörün elinde arızayı gösteren TEK BİR işaret yoktu.
# Sıra POZ-YOK'un ÜSTÜNDE: makullük kapısı yayını kestiğinde POZ-YOK da
# tetiklenir; ikisi aynı anda aktifken operatöre gösterilmesi gereken,
# "poz gelmiyor" değil "poz patladı" — sebebi söyleyen kod odur.
POZ_SACMA = ArizaTanimi(
    "POZ-SACMA", "poz sayisal patladi, MPPI durdu", SEVIYE_ERROR
)
POZ_YOK = ArizaTanimi("POZ-YOK", "poz yok, MPPI durdu", SEVIYE_ERROR)
POZ_BAYAT = ArizaTanimi("POZ-BAYAT", "poz bayat, MPPI durdu", SEVIYE_ERROR)
ENGEL_YOK = ArizaTanimi(
    "ENGEL-YOK", "engel haritasi yok, MPPI durdu", SEVIYE_ERROR
)
ENGEL_BAYAT = ArizaTanimi(
    "ENGEL-BAYAT", "engel bayat, MPPI durdu", SEVIYE_ERROR
)
RRT_RED = ArizaTanimi("RRT-RED", "global plan uretilemedi", SEVIYE_ERROR)
SETPOINT_BOSLUK = ArizaTanimi(
    "SETPOINT", "setpoint kesildi, failsafe riski", SEVIYE_ERROR
)
CMDVEL_KESIK = ArizaTanimi(
    "CMDVEL", "cmd_vel kesildi, tekne durabilir", SEVIYE_ERROR
)
ENGEL_BOS = ArizaTanimi(
    "ENGEL-BOS", "algi akiyor ama hep bos", SEVIYE_WARNING
)
SINIF_YOK = ArizaTanimi(
    "SINIF-YOK", "kapi takibi kapali, sinif yok", SEVIYE_WARNING
)
KAPI_YOK = ArizaTanimi("KAPI-YOK", "kapi secilemedi", SEVIYE_WARNING)
GPU_YOK = ArizaTanimi("GPU-YOK", "MPPI CPU'da, adim suresi riskli", SEVIYE_WARNING)
# §0.61h — kaptanın isteği: *"bu veriyi fix oldu ya da olmadı diye pixhawkta
# göreyim."* Sistem saati GPS'ten kurulamadıysa bu kod ekranda DURUR; saat
# kurulunca (açılışta seri yoldan ya da sonradan `girdap-saat-gec` ile)
# KENDİLİĞİNDEN düşer — yani kodun YOKLUĞU "fix alındı, saat kuruldu"
# demektir. Ölçüt çekirdeğin `adjtimex` STA_UNSYNC bayrağı (teslim
# damgalarının dayandığı ölçütün AYNISI, `saat_guveni.py`) — "tarih makul mü"
# değil "bir referansa göre düzeltildi mi".
# ⚠ Seviye WARNING ve liste SONUNDA: saat yokluğu görevi durdurmaz, veri
# geçerlidir; yalnız mutlak saat iddia edilmez. Gerçek arızaları GÖLGELEMEZ.
SAAT_YOK = ArizaTanimi(
    "SAAT-YOK", "saat GPS'ten kurulmadi (fix?)", SEVIYE_WARNING
)

#: Öncelik sırasıyla bütün tanımlar (üstteki en kritik).
ARIZALAR: tuple[ArizaTanimi, ...] = (
    KONTROL_HATA,
    KONTROLCU_YOK,
    POZ_SACMA,
    POZ_YOK,
    POZ_BAYAT,
    ENGEL_YOK,
    ENGEL_BAYAT,
    RRT_RED,
    SETPOINT_BOSLUK,
    CMDVEL_KESIK,
    ENGEL_BOS,
    SINIF_YOK,
    KAPI_YOK,
    GPU_YOK,
    SAAT_YOK,
)


# `planning_node`'un `/girdap/control/inhibit_reason` sebep etiketleri →
# arıza kodu. 🔑 TESPİT MANTIĞI İKİ KEZ YAZILMAZ: düğüm zaten "thrust neden
# sıfır" listesini üretiyor (KAR-04); telsize çıkan kod da aynı listeden
# türetilir, böylece ikisi hiçbir zaman ayrışamaz.
#
# ⚠ Listedeki BAZI sebepler bilerek dışarıda: `FSM-DISI(...)` ve
# `DISARM-VEYA-KILL` arıza değil, NORMAL durumlardır (görev başlamadı /
# operatör durdurdu) — üstelik görev durumunu `fsm_node` zaten aynı hattan
# bildiriyor. Onları da yollamak hem telsizi doldurur hem gerçek arızayı
# gürültüye gömer.
_SEBEP_KODLARI: dict[str, ArizaTanimi] = {
    "KONTROLCU-HAZIR-DEGIL": KONTROLCU_YOK,
    "POZ-SACMA": POZ_SACMA,
    "POZ-YOK": POZ_YOK,
    "POZ-BAYAT": POZ_BAYAT,
    "ENGEL-YOK": ENGEL_YOK,
    "ENGEL-BAYAT": ENGEL_BAYAT,
}


def sebepten_kodla(sebepler: list[str] | tuple[str, ...]) -> list[ArizaTanimi]:
    """Kilit sebebi etiketlerini arıza tanımlarına çevir (bilinmeyeni atla)."""
    return [
        _SEBEP_KODLARI[s] for s in sebepler if s in _SEBEP_KODLARI
    ]


#: Kilit sebebinden türetilen kodların tamamı — düğüm bunları her turda
#: yeniden hesaplar, dolayısıyla temizlemesi de bu küme üzerinden yapılır.
SEBEP_TUREVLI_ARIZALAR: tuple[ArizaTanimi, ...] = tuple(_SEBEP_KODLARI.values())


class ArizaBildirici:
    """Aktif arızaları tutar, telsize NE ZAMAN ne gideceğine karar verir.

    Zamanı kendi okumaz — `gonderilecek(simdi)` çağıranın saatini kullanır
    (test edilebilirlik + düğümün ROS saatiyle tutarlılık).
    """

    def __init__(
        self,
        tazeleme_s: float = 20.0,
        azami_uzunluk: int = AZAMI_UZUNLUK,
    ) -> None:
        if tazeleme_s < 0.0:
            raise ValueError("tazeleme_s negatif olamaz")
        self._tazeleme_s = float(tazeleme_s)
        self._azami_uzunluk = int(azami_uzunluk)
        self._aktif: set[str] = set()
        self._son_metin: str | None = None
        self._son_gonderim: float | None = None

    # ------------------------------------------------------------ durum girişi

    def bildir(self, ariza: ArizaTanimi) -> None:
        """Arızayı aktif işaretle. Her tick çağrılabilir (tekrar bedava)."""
        self._aktif.add(ariza.kod)

    def temizle(self, ariza: ArizaTanimi) -> None:
        """Arıza düştü."""
        self._aktif.discard(ariza.kod)

    def ayarla(self, ariza: ArizaTanimi, *, aktif: bool) -> None:
        """`bildir`/`temizle`'nin tek çağrılık hâli — koşulu doğrudan geçir."""
        if aktif:
            self.bildir(ariza)
        else:
            self.temizle(ariza)

    @property
    def aktif_kodlar(self) -> frozenset[str]:
        return frozenset(self._aktif)

    def oncelikli(self) -> ArizaTanimi | None:
        """Aktif arızalar içinde en öncelikli olanı döndür (yoksa None)."""
        for tanim in ARIZALAR:
            if tanim.kod in self._aktif:
                return tanim
        return None

    # ------------------------------------------------------------ gönderim kararı

    def gonderilecek(self, simdi: float) -> tuple[str, int] | None:
        """Şu an gönderilmesi gereken (metin, seviye) — yoksa None.

        Üç durum:
          * en öncelikli arıza değişti  → hemen gönder
          * aynı arıza sürüyor          → `tazeleme_s` dolduysa tekrarla
          * bütün arızalar düştü        → bir kez "ariza yok" bas, sonra sus
        """
        tanim = self.oncelikli()
        metin = tanim.statustext() if tanim is not None else TEMIZ_METNI
        seviye = tanim.seviye if tanim is not None else SEVIYE_NOTICE

        if tanim is None and self._son_metin is None:
            # Hiç arıza olmadı ve hâlâ yok → sessiz kal (açılışta gürültü yok).
            return None

        degisti = metin != self._son_metin
        if not degisti:
            if tanim is None:
                # "ariza yok" TEKRARLANMAZ — telsizi boşuna doldurmasın.
                return None
            if self._tazeleme_s <= 0.0:
                return None
            if (
                self._son_gonderim is not None
                and simdi - self._son_gonderim < self._tazeleme_s
            ):
                return None

        self._son_metin = metin
        self._son_gonderim = simdi
        return metin[: self._azami_uzunluk], seviye

"""
Girdap İDA — Duba kapısı takibi (gate-following) çekirdeği, ROS-bağımsız.

PROBLEM (Şartname md 5.5.2.2 / Şekil 2): Parkur-1 ve Parkur-2'de puan,
GPS görev noktasına (GN) tam basmaktan DEĞİL, karşılıklı KENAR dubası ikilisinin
ARASINDAN geçmekten gelir (G1/G2 = geçiş sayısı). Üstelik hakemin verdiği görev
noktası "doğrudan iki kenar dubasının arasında bir nokta olmayabilir" (md
5.5.2.2) ve dubaların konumu paylaşılmaz / önceden haritalanamaz. Yani ham GPS
noktasına yönelmek YETMEZ: araç, kamera/füzyonla algıladığı kenar dubalarından
kapı orta noktasını KENDİSİ hesaplayıp oradan geçmelidir.

BU MODÜL: algılanan turuncu KENAR dubalarından (class "0") öndeki kapıyı seçer
ve geçilecek noktayı "rafine hedef" olarak döndürür. Kapı görünmüyorsa ham GPS
görev noktasına düşer (fallback) — böylece kapı görüş menziline girene kadar
araç yine de doğru genel yöne ilerler.

🔑 HEDEF KÖR ORTA NOKTA DEĞİL: geçilecek nokta kapı kirişi üzerinde
ENGELLERE GÖRE kaydırılır (`aim_point`). Sebebi mimari: kenar dubaları MPPI'nin
engel torbasından çıkarılmak ZORUNDA (ceza halkası geçidin içini kaplar ve araç
kapıdan geçmeyi dolanmaktan pahalı bulur) — ama çıkarılınca geçitte dubadan
İTEN hiçbir kuvvet kalmaz, yalnız ortanın çekimi vardır. İtme bu yüzden MPPI
maliyetine değil HEDEFİN KENDİSİNE gömülür. Engel yokken nişan tam ortadadır,
yani kapısız/engelsiz senaryoda davranış birebir eskisi gibidir.

MİMARİ YERİ: mission_manager'ın `current_target`'ını (ham GN) RAFİNE eder;
RRT*/MPPI zinciri değişmez, sadece daha doğru bir hedef alır. SARI engel
dubaları (class "1") bu modüle GİRMEZ — onlar planning'de CircleObstacle olarak
kaçınma nesnesi olarak kalır. Turuncu kenar dubaları ise "kapı" olur (arasından
geçilir), engel torbasından çıkarılır (bkz. Layer 2 node entegrasyon notu).

FRAME: tek bir 2D dünya ENU çerçevesi (x=doğu, y=kuzey). Tüm girdiler AYNI
çerçevede olmalı; base_link→ENU dönüşümü Layer 2 node'un işidir (çekirdek
frame-bağımsız kalır, pytest ile rclpy'siz doğrulanır — parkur_fsm.py deseni).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
# Dairesel engel: (merkez_x, merkez_y, yarıçap) — dünya ENU, metre.
Circle = Tuple[float, float, float]

# Şartname md 5.5.2.1: kenar/engel dubası çapı 30 cm. Şekil 3'ün aksine bu
# ÖLÇÜLMÜŞ değil ŞARTNAME sabiti — metinde kesin verilen iki sayıdan biri
# (diğeri 50 cm yükseklik), dolayısıyla "tahmine dayalı sayı yok" kuralını
# bozmaz. Dubalar merkezlerinden algılandığı için serbest açıklık hesabında
# yarıçapın iki kez düşülmesi gerekir.
BUOY_RADIUS_M = 0.15

# Nişan noktası aramasında kiriş başına en fazla kaç adım denenir (hesap
# kapağı, davranış ayarı DEĞİL): çok geniş kapıda örnek sayısı patlamasın.
# Adım gövde genişliğinden türer, bu yalnız üst sınır.
_AIM_MAX_ADIM = 64

# B5 — kilitlenmeden ÖNCE aynı kapının kaç tick üst üste görülmesi gerektiği.
#
# 🔑 **Neden 2, ve neden bu bir "ayar" değil:** 2, "anlık" ile "kalıcı" arasındaki
# EN KÜÇÜK ayrımdır — bir kez görülen şey tek karelik olabilir, iki kez görülen
# olamaz. 3/5/10 seçmek ise tahmin olurdu (modülün tasarım kuralı: tahmine dayalı
# sayı yok). Bu yüzden sayı, gecikmeyi de en aza indiren kategorik alt sınırda
# bırakıldı: 20 Hz'lik bir kontrol çevriminde bedeli **50 ms**.
#
# 🔴 **Neden gerekli:** kilitlenme tek karelik bir yanlış tespiti KALICI hâle
# getiriyordu. `update()` kilitli kapıyı oklüzyona karşı korur (taze algı
# görmüyorsa eskisini saklar) — bu doğru bir davranıştır, ama onaysız kilitle
# birleşince bir karelik hayalet kapı, aracın onun düzlemini fiilen geçmesine
# kadar hedef olarak KALIYORDU. Onay kapısı bu iki doğru davranışın kesişimindeki
# arızayı kapatır. (Kilitlendikten sonraki oklüzyon koruması aynen sürer.)
ONAY_TICK = 2

#: F-K.5 bayat kilit bırakma katsayısı (float('inf') → kapalı; A/B için).
_BAYAT_KILIT_KATSAYI = 2.0


@dataclass(frozen=True)
class GateFollowerConfig:
    r"""Kapı seçimi — **TAHMİNE DAYALI TEK BİR SAYI YOK.**

    🔴 **Neden:** kapı geometrisi önceden bilinemez. Şartname üç yerde söylüyor:
      · *"Parkurların uzunlukları, parkurlarda kullanılan duba sayıları ve
        KENAR DUBALARI ARASINDAKİ MESAFELER yarışma alanına göre değişkenlik
        gösterecektir."*
      · *"Dubalar arasındaki mesafeler, duba sayıları, parkur uzunluğu yarışma
        alanına göre BELİRLENECEKTİR. Duba sayılarına göre bir akış
        tasarlanmaması tavsiye edilmektedir."*
      · *"kenar dubaları ve engeller de DENİZ ŞARTLARINDAN DOLAYI YER
        DEĞİŞTİREBİLİR."* → koşu SIRASINDA bile sabit değil.
    Parkur önceden görülemez, önceden haritalama zaten yasak. Dolayısıyla
    "sahada ölçüp gir" diye bir eşik OLAMAZ — ölçülecek bir şey yok.

    **Tasarım kuralı (2026-08-03): her sayı ya ÖLÇÜLMÜŞ bir tekne boyutudur ya
    da saf geometridir. Serbest/ayarlanabilir eşik bırakılmadı.**

    | eski (tahmin) | yerine geçen |
    |---|---|
    | `gate_width_max` = 20 m | **YOK** — genişlik hiç kısıt değil |
    | `gate_width_min` = 1.0 m | **gövde genişliği + 2×duba yarıçapı** (fizik: dar kapıdan geçilemez; dubalar merkezden algılanır) |
    | `max_lookahead` = 25 m | **YOK** — menzili algı katmanı belirler |
    | `pair_depth_tol` = 3.0 m | **\|Δileri\| < \|Δyanal\|** (45° geometrik ayrım) |
    | `min_forward` = 0.5 m | **gövde yarı boyu** (burnun önü) |
    | `release_distance` = 0.8 m | **ileri mesafe ≤ 0** (kapı düzlemi geçildi) |
    | `match_radius` = 2.5 m | **kapının kendi genişliğinin yarısı** (öz-ölçekli) |

    🔑 **"Yan yana mı" testi neden `|Δileri| < |Δyanal|`:** bir kapı, kursa
    DİK duran bir duba çiftidir; ardışık iki kapının dubaları ise kurs boyunca
    dizilir. Bu eşitsizlik "aradaki doğru paralel olmaktan çok dike yakın mı"
    demektir — kesişim noktası tam 45°, yani **ayarlanan bir eşik değil, iki
    hâl arasındaki geometrik ayrım noktası**. Ölçek-bağımsız: kapı 2 m de olsa
    40 m de olsa aynı çalışır. Aynı zamanda tek dubası görünmeyen bir kapının
    yanlışlıkla komşu kapıyla eşleşmesini de eler (onlar ileri yönde ayrıktır).

    Tekne boyutları `~/Desktop/GIRDAP_DURUM.md` §1'den (ölçülmüş):
    0,78 × 1,04 × 0,52 m.
    """

    # ÖLÇÜLMÜŞ tekne boyutları — tek girdi bunlar, tahmin değil.
    # Genişlik: bundan dar bir açıklıktan tekne fiziksel olarak GEÇEMEZ, o
    # yüzden öyle bir çift kapı değildir (büyük olasılıkla tek dubanın iki
    # tespite bölünmesidir). Bu bir eşik ayarı değil, geçilebilirlik testi.
    hull_width_m: float = 0.785
    # Boy: dubanın "önümüzde" sayılması için burnun ötesinde olması gerekir.
    hull_length_m: float = 1.04

    @property
    def min_forward(self) -> float:
        """Duba en az bu kadar önde olmalı = gövde yarı boyu (burun hattı)."""
        return self.hull_length_m / 2.0

    @property
    def half_beam(self) -> float:
        """Gövde yarı genişliği — kiriş üzerindeki ölü bandın bir bileşeni."""
        return self.hull_width_m / 2.0

    @property
    def min_passable_width(self) -> float:
        """Geçilebilir en dar kapı: MERKEZDEN merkeze mesafe.

        🔴 Dubalar MERKEZLERİNDEN algılanır ama gövde duba YÜZEYLERİ arasından
        geçer: serbest açıklık = `merkez_mesafe − 2r`. Yalnız `hull_width_m`
        ile karşılaştırmak duba çapı kadar (30 cm) GEÇ kapanan bir süzgeçtir →
        `[0.785 ; 1.085)` aralığındaki çiftler geçilemez oldukları hâlde kapı
        sayılır, araç sığmayacağı bir orta noktaya nişan alır ve **iki dubaya
        birden çarpar**. Üstelik tam bu aralık sahte çiftlerin (tek dubanın iki
        kümeye bölünmesi, su yansıması) düştüğü yerdir.
        """
        return self.hull_width_m + 2.0 * BUOY_RADIUS_M


@dataclass(frozen=True)
class Gate:
    """Seçilmiş bir kapı: iki kenar dubası + orta nokta (hepsi dünya ENU).

    `midpoint` KİMLİKTİR, `aim` HEDEFTİR — ikisi bilerek ayrı:
      · `midpoint` = saf geometrik orta. Kapının değişmez kimliği; "aynı kapı
        mı" eşleşmesi ve "geçildi mi" testi buna bakar. Engel gelip gidince
        kaymaz, yoksa kilit her tick'te kendi kendine kırılırdı.
      · `aim` = fiilen sürülecek nokta; kiriş üzerinde engellerden en açık yer
        (bkz. `aim_point`). Engel yokken `midpoint` ile BİREBİR aynıdır.
    """

    left: Point       # kursa göre SOL kenar dubası (+lateral)
    right: Point      # kursa göre SAĞ kenar dubası (-lateral)
    midpoint: Point   # geometrik orta — kapının kimliği
    # Engele göre ayarlanmış nişan noktası; None → engel bilgisi yok, orta kullan.
    aim: Optional[Point] = None
    # Kapının KENDİ ileri normali (birim, kirişe dik). Kilitlenme anındaki
    # gidiş yönüne göre işaretlenir. None → eski kurs-ekseni testine düşülür.
    normal: Optional[Point] = None

    @property
    def width(self) -> float:
        """Kenar dubaları arası mesafe (m) — MERKEZDEN merkeze."""
        return math.hypot(self.left[0] - self.right[0], self.left[1] - self.right[1])

    @property
    def drive_target(self) -> Point:
        """Sürülecek nokta: nişan varsa o, yoksa geometrik orta."""
        return self.aim if self.aim is not None else self.midpoint

    def yaklasma_hedefi(self, geri: float) -> Point:
        """🔴 F-K.2 (13.08.2026) — KAPININ ÖNÜNDEKİ HİZALANMA NOKTASI.

        Kapıya **eğik** yaklaşan araç, kapıya varmadan yanına düşer; o noktadan
        çift artık "kapı" gibi görünmez (iki duba görüş hattı BOYUNCA dizilir,
        `select_gate`'in dikeylik testi haklı olarak reddeder) ⇒ **kapı bir
        daha seçilemez ve kalıcı olarak kaçırılır.**

        📏 Sanal gölde ölçüldü (gerçek geometri: 8 kapı, 12 m açıklık, 4 m
        aralık, zigzag ±5 m): tekne **5/8** kapı geçti, sonra parkurun sağına
        kaçtı — iz `(1.5, 12.1) → (8.5, 28.5) → (9.7, 23.7) → (8.1, 12.4)`,
        ψ 252-282°, yani **geri geri** salınıma girdi. Logda 5 kez *"çift
        kursa DİK DEĞİL"*. Şartname P2 puanı **oransal** `(G2/KD2)×40` →
        8 kapıda 3 kaçırmak **15 puan**.

        🔑 Çözüm literatürün kalıbı: **mesafelen → DİK yaklaş → geç.** Nişan
        önce kapının **önündeki** hizalanma noktasıdır; araç oraya varınca
        kapı normaliyle hizalanmış olur, sonra `gecis_hedefi` onu düzlemin
        ötesine taşır (F-K.1). RoboBoat kuralı da geçişin kapıdan **önce**
        başlamasını şart koşar.

        `geri` uydurulmaz: **kapının kendi yarı genişliği** (öz-ölçekli, dış
        eşik yok — modülün donmuş kuralı §0.0d). 12 m kapıda 6 m ≈ ölçülmüş
        seyir hızıyla (1,05 m/s) ~6 s hizalanma payı.

        `normal` yoksa uzatma gibi bu da YAPILMAZ — yön bilinmeden geri
        gitmek aracı parkurdan uzaklaştırırdı.
        """
        if self.normal is None:
            return self.drive_target
        mx, my = self.midpoint
        nx, ny = self.normal
        return (mx - nx * geri, my - ny * geri)

    def surus_noktasi(self, arac: Point, geri: float, uzatma: float) -> Point:
        """İKİ FAZLI sürüş hedefi: hizalan → geç. **ASLA GERİYE nişan almaz.**

        🔴 Kaptanın uyarısı (13.08): *"korkum şu, Parkur-3'te modelin geriye
        gitmesi."* Haklı ve tasarımın gerçek açığına işaret ediyor — araç
        yaklaşma noktasını ÇOKTAN GEÇMİŞSE oraya nişan almak **geri gitmek**
        demektir. Kapı takibi `gate_following_enabled` tek parametresine bağlı
        olduğu için PARKUR3'te de etkindir, yani risk teoriktir değil.

        🔑 GARANTİ: faz seçimi aracın kapı normali üzerindeki İŞARETLİ
        MESAFESİNE bakar (`signed_distance`):
          · `d < −geri` → araç hizalanma noktasının **gerisinde**; hedef
            YAKLAŞMA noktası — tanım gereği **ileride** (d = −geri > d).
          · aksi hâlde → hedef ÇIKIŞ noktası (kapı düzleminin ötesi, F-K.1).
        Her iki dalda da hedefin normal üzerindeki izdüşümü aracınkinden
        BÜYÜKTÜR ⇒ normal yönünde geri hareket komutu üretilemez.
        (Literatürdeki LOS/waypoint-switching kuralının ta kendisi: geçilen
        noktaya bir daha nişan alınmaz.)

        ⚠ Yanal hareket geri hareket DEĞİLDİR: araç kapının yanına düşmüşse
        hizalanmak için yana gider, ileri bileşeni yine pozitiftir.
        """
        d = self.signed_distance(arac)
        if d is None:
            return self.drive_target
        # 🔴 HİZALANMA FAZI ÖLÇÜLDÜ VE GERİ ALINDI (13.08). Fikir doğruydu
        # (literatür: mesafelen → dik yaklaş → geç) ama bu parkurda ÇALIŞMIYOR:
        # hizalanma payı kapının yarı genişliği (12 m kapıda **6 m**) iken
        # kapılar **4 m aralıklı** → bir sonraki kapının yaklaşma noktası bir
        # öncekinin GERİSİNE düşüyor, çekişler çatışıyor. Sanal gölde ölçüldü:
        #     yalnız çıkış (F-K.1)      → 5 kapı, takıldı
        #     hizalanma + çıkış (F-K.2) → **1 kapı**, tekne dondu
        #     yalnız havuç              → parkur TAMAMLANDI
        # `yaklasma_hedefi` yöntemi ve testleri KALIYOR (seyrek kapılı bir
        # parkurda doğru araç olabilir), ama sürüş yolundan ÇIKARILDI.
        # ⚠ Yeniden açılacaksa önce hizalanma payı ile KAPI ARALIĞI
        # kıyaslanmalı: pay < aralık olmadan bu faz kullanılamaz.
        del geri
        # 🔴 ÇIKIŞ HAVUCU HER ZAMAN ARACIN ÖNÜNDE. Sabit `uzatma` yetmez:
        # araç düzlemi `uzatma`dan FAZLA geçmişse (kilit henüz bırakılmamış
        # olabilir — bırakma bir sonraki `update`'te olur, araç bu arada yol
        # alır) sabit nokta ARKADA kalır ve GERİ komut üretir. Özellik testi
        # bunu yakaladı: araç d=3,97 iken hedef d=1,04 (kaptanın korktuğu hâl).
        # Çözüm pure-pursuit havucunun kendisi: hedef, düzlem ile aracın
        # DAHA İLERİDEKİNDEN `uzatma` kadar öteye konur.
        ileri = max(uzatma, d + uzatma)
        mx, my = self.midpoint
        nx, ny = self.normal                       # d not None ⇒ normal var
        ax, ay = self.drive_target                 # kiriş üzerindeki nişan
        # Nişanın yanal bileşeni korunur (engelden açık yer), ileri bileşen
        # havuçtan gelir.
        yanal = (ax - mx) * (-ny) + (ay - my) * nx
        return (mx + nx * ileri - ny * yanal, my + ny * ileri + nx * yanal)

    def gecis_hedefi(self, uzatma: float) -> Point:
        """🔴 F-K.1 (13.08.2026) — SÜRÜLECEK NOKTA KAPININ ÖTESİNDEDİR.

        Kapı bir VARIŞ NOKTASI DEĞİL, GEÇİLECEK BİR EŞİKTİR. Nişan kapı
        düzleminin üstünde bırakılırsa MPPI referansı orada BİTER; araç
        oraya varır, terminal maliyet sıfırlanır ve **tam kapı ortasında
        durur**. Düzlem geçilmediği için kilit de çözülmez → görev bir daha
        ilerlemez.

        📏 Sanal gölde ölçüldü (13.08, kapalı döngü, gerçek düğümler):
        tekne 25 m sürüp 1. kapıya vardı ve **(0.02, 24.95)'te kilitlendi** —
        `current_target` (−0.02, 25.04)'te takılı kaldı, MPPI thrust
        (−0.13, +0.05) N, yani fiilen sıfır. Görev yöneticisi bir sonraki
        noktaya geçmiş olmasına rağmen araç durdu.

        🔑 UZATMA MİKTARI UYDURULMADI: **gövde boyu** (`hull_length_m`,
        ölçülmüş 1,04 m). Gerekçe yarışma tanımının kendisi: geçiş süresi
        *pruva* dubaları geçince başlar, ***kıç* geçince biter** — yani
        geçmiş sayılmak için TÜM tekne düzlemin ötesine çıkmalı. Nişanı
        gövde boyu kadar öteye koymak bunu garanti eder ve yeni bir
        ayarlanabilir eşik getirmez.

        `normal` yoksa (eski kurs-ekseni kolu) uzatma yapılmaz — davranış
        birebir eskisi gibi kalır.
        """
        if self.normal is None:
            return self.drive_target
        ax, ay = self.drive_target
        nx, ny = self.normal
        return (ax + nx * uzatma, ay + ny * uzatma)

    @property
    def aim_shift(self) -> float:
        """Nişanın geometrik ortadan kayma miktarı (m) — saha teşhisi."""
        ax, ay = self.drive_target
        return math.hypot(ax - self.midpoint[0], ay - self.midpoint[1])

    def signed_distance(self, vehicle: Point) -> Optional[float]:
        r"""Aracın kapı DÜZLEMİNE işaretli mesafesi (m). Normal yoksa None.

        `> 0` → kapı düzlemi geçildi (araç kapının ötesinde),
        `< 0` → kapıya hâlâ gidiliyor.

        🔑 **Neden kurs ekseni DEĞİL, kapının kendi normali** (B4/B6):
        eski test `(orta − araç)·f` idi; `f` araçtan ham görev noktasına (GN)
        bakan birim vektör. İki ayrı arızası vardı:
          1. Şartname *"görev noktası doğrudan iki kenar dubasının arasında bir
             nokta OLMAYABİLİR"* diyor (md 5.5.2.2). **GN yana kaçıksa** araç
             kapıdan fiilen geçtiği hâlde izdüşüm pozitif kalabilir → kilit
             çözülmez, araç geçtiği kapıya GERİ DÖNER.
          2. GN'ye yaklaşırken `f` hızla döner (1 m kala 0,3 m yanal hata ≈ 17°)
             → eşik kayar, seçim tick'ler arasında zıplar.
        Kapının kendi normali kurstan da ölçekten de bağımsızdır; yalnız
        İŞARETİ kilitlenme anındaki gidiş yönünden alınır ve araç 90°'den
        fazla dönmedikçe sabit kalır.
        """
        if self.normal is None:
            return None
        nx, ny = self.normal
        return (vehicle[0] - self.midpoint[0]) * nx + (
            vehicle[1] - self.midpoint[1]
        ) * ny

    def lateral_offset(self, vehicle: Point) -> Optional[float]:
        """Aracın kapı ORTASINDAN kiriş boyunca yanal uzaklığı (m, işaretsiz).

        Normal yoksa None. `passed_between` (puan) ve K1 "bu kapının yanından
        mı geçtim" (geometri) testlerinin ORTAK ölçüsü.
        """
        if self.normal is None:
            return None
        nx, ny = self.normal
        tx, ty = -ny, nx                         # kiriş boyunca birim
        return abs(
            (vehicle[0] - self.midpoint[0]) * tx
            + (vehicle[1] - self.midpoint[1]) * ty
        )

    def passed_between(self, vehicle: Point) -> bool:
        """Araç kapının ARASINDAN mı geçti? (düzlemi aşmak YETMEZ)

        İki şart: (1) düzlem geçildi, (2) yanal sapma kapının yarı
        genişliğini aşmıyor — yani direklerin ARASINDAN geçildi. (2) olmazsa
        kapıyı yandan DOLAŞAN araç da "geçti" sayılırdı; oysa şartnameye göre
        bu geçiş değildir (G1/G2 sayılmaz) ve üstüne parkur-dışı cezası gelir.

        ⚠ Bu, kilidi bırakma testinden AYRIDIR: dolaşarak geçilen kapı da
        bırakılır (arkada kaldı), ama SAYILMAZ.
        """
        d = self.signed_distance(vehicle)
        if d is None or d <= 0.0:
            return False
        yanal = self.lateral_offset(vehicle)
        return yanal is not None and yanal <= self.width / 2.0


@dataclass
class GateDiagnostics:
    """Neden kapı bulunamadı? — sahada bandı düzeltebilmek için.

    Kapı genişliği önceden bilinemediği için (bkz. GateFollowerConfig) en
    tehlikeli arıza SESSİZ RET'tir: turuncu dubalar pekâlâ görülüyordur ama
    genişlikleri banda girmediği için hiç kapı oluşmaz ve araç ham görev
    noktasına gider — hiçbir hata basılmadan, puan kaybederek.
    Bu sayaçlar node tarafından loglanır; operatör ÖLÇÜLEN genişlikleri görüp
    `planning.gate_width_max:=14` gibi bir launch-arg ile anında düzeltebilir.
    """

    n_edge_buoys: int = 0            # gelen turuncu duba sayısı
    n_in_range: int = 0              # burun hattının önünde kalanlar
    n_pairs_checked: int = 0         # değerlendirilen çift sayısı
    # Gövdenin sığmadığı (geçilemez) çiftlerin ÖLÇÜLEN açıklığı.
    reddedilen_genislik: List[float] = None
    # "Yan yana değil" (kursa dik değil) diye elenen çift sayısı — bunlar
    # neredeyse hep ardışık kapıların dubalarıdır, normal ve beklenen.
    reddedilen_derinlik: int = 0
    # K1: "bu kapının düzlemini zaten geçtim" diye elenen çift sayısı. Sahada
    # 0'dan büyük olması NORMALDİR (arkada bıraktığımız kapıları görmeye devam
    # ederiz); asıl anlamı, K1 salınımının kapalı olduğunun kanıtı olmasıdır.
    reddedilen_gecilmis: int = 0
    secilen_genislik: Optional[float] = None
    # Nişan noktasının geometrik ortadan kayması (m). 0.0 → engel kirişe
    # dokunmuyor, tam ortadan geçiliyor. Büyük değer sahada "kapının içinde/
    # ağzında bir şey var" demektir; operatör buna bakarak duba mı hayalet mi
    # ayırt eder.
    aim_kaymasi_m: float = 0.0
    # B5 onay sayacı: aday kapı üst üste kaç tick görüldü (0 → aday yok).
    # `ONAY_TICK`'e ulaşmadan kilitlenilmez; bu arada hedef ham GN'dir.
    # Sahada "kapıyı görüyor ama kilitlenmiyor" hâlini ayırt etmeye yarar.
    aday_onay_sayaci: int = 0

    def __post_init__(self) -> None:
        if self.reddedilen_genislik is None:
            self.reddedilen_genislik = []


@dataclass(frozen=True)
class GateResult:
    """update() çıktısı: kullanılacak hedef + bağlam."""

    target: Point            # rafine hedef (kapı ortası) ya da fallback (ham GN)
    gate: Optional[Gate]     # seçilen kapı; fallback'te None
    used_fallback: bool      # True → kapı yok, ham GN'ye düşüldü
    # 🔴 F-K.1: KONTROLE giden nokta. `target` kapının NİŞANIDIR (kimlik,
    # teşhis, RViz); `surus_hedefi` onun gövde boyu kadar ÖTESİDİR — MPPI
    # referansı buraya kurulur ki araç kapıda durmayıp GEÇSİN. Kapı yokken
    # ikisi aynıdır (ham GN), yani kapısız davranış birebir korunur.
    surus_hedefi: Point = (0.0, 0.0)

    def __post_init__(self) -> None:
        if self.surus_hedefi == (0.0, 0.0) and self.gate is None:
            object.__setattr__(self, "surus_hedefi", self.target)


def _forward_left_axes(
    vehicle: Point, hedef: Point
) -> Optional[Tuple[float, float, float, float]]:
    """Araçtan `hedef`e "ileri" birim vektörü + "sol" dik birim vektörü.

    İki ayrı yerde kullanılır ve **hedefi bilerek farklıdır**:
      · `hedef = ham GN` → **kurs ekseni**. Yalnız (1) dubanın önümüzde olup
        olmadığı kaba süzgeci ve (2) eşitlikte sıralama için. Aracın anlık
        heading'i değil GN kullanılır: kursun gittiği yönü verir ve momentary
        heading sapmalarına dayanıklıdır.
      · `hedef = çiftin orta noktası` → **radyal çerçeve**. Bir çiftin "kapı mı
        yoksa ardışık kapıların dubaları mı" testi bunda yapılır; böylece test
        GN'nin nerede olduğundan TAMAMEN bağımsızdır (bkz. `select_gate` (b)).
    Araç hedefin üstündeyse (yön tanımsız) None döner.
    """
    dx = hedef[0] - vehicle[0]
    dy = hedef[1] - vehicle[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    fx, fy = dx / norm, dy / norm      # ileri birim
    lx, ly = -fy, fx                   # sol dik birim (90° CCW)
    return fx, fy, lx, ly


def _gate_normal(
    left: Point, right: Point, forward: Point
) -> Optional[Point]:
    """Kirişe dik birim vektör, `forward` ile aynı yarı düzleme işaretlenmiş.

    `forward` yalnız İŞARET içindir (hangi taraf "ileri"); vektörün yönü
    tamamen kapı geometrisinden gelir. Kiriş dejenere ise None.
    """
    dx, dy = right[0] - left[0], right[1] - left[1]
    w = math.hypot(dx, dy)
    if w <= 1e-9:
        return None
    nx, ny = -dy / w, dx / w                  # kirişe dik (90° CCW)
    if nx * forward[0] + ny * forward[1] < 0.0:
        nx, ny = -nx, -ny                     # ters yarı düzlem → çevir
    return (nx, ny)


def _clearance(px: float, py: float, circles: Sequence[Circle]) -> float:
    """Noktanın en yakın engel YÜZEYİNE mesafesi (m). Engel yoksa +sonsuz.

    Merkeze değil yüzeye bakılır: farklı yarıçaplı nesneler (30 cm duba ile
    bölünmüş büyük bir küme) aynı ölçüte girsin.
    """
    best = math.inf
    for cx, cy, r in circles:
        d = math.hypot(px - cx, py - cy) - r
        if d < best:
            best = d
    return best


def aim_point(
    left: Point,
    right: Point,
    circles: Sequence[Circle],
    cfg: GateFollowerConfig,
) -> Optional[Point]:
    r"""Kapı kirişi üzerinde engellerden EN AÇIK nişan noktası.

    🔑 **NEDEN GEREKLİ.** Kenar dubaları MPPI'nin engel torbasından bilerek
    çıkarılır (`obstacle_margin`=1.0 m'lik ceza halkası geçidin içini kaplar,
    ölçüldü: 1.5 m'de araç geçitten HİÇ geçmiyor). Bedeli şu: geçitte dubadan
    **iten hiçbir kuvvet kalmaz**, yalnız orta noktanın çekimi vardır. Dalga
    sürüklenmesi ya da tek taraflı bir tespit hatası doğrudan TEMAS demektir
    (`Ç1`/`Ç2`; md 815-818: aynı dubaya 30 sn temas = 2 çarpma).

    Çözüm dubayı engele geri çevirmek DEĞİL (geçit kapanır), **nişanı kirişin
    üzerinde kaydırmak**: itme MPPI'ye değil hedefe gömülür. Engel yokken
    nişan zaten tam ortadır → geriye birebir uyumlu.

    **Ölü bant:** kirişin iki ucunda `r_duba + gövde/2` kadar yer gövdeye
    yasaktır (duba yüzeyi + yarım gövde). Bu bant kirişten uzunsa kapı
    geçilemez → `None`. Yani `min_passable_width` testi burada da kendini
    doğrular.

    **Neden tarama, neden kapalı form yok:** açıklık = tüm dairelerin üzerinde
    `min(|p−c| − r)`; bunun maksimumu parçalı hiperbolik, analitik çözümü yok.
    Izgara **merkezden dışa** kurulur → engel bağlamadığında argmax tam merkez
    çıkar (kayan ızgarada merkez örneklenmeyip sonuç 1-2 cm kayabilirdi).
    Adım gövde genişliğinin dörtte biri: ayarlanan eşik değil, "tekne bu
    çözünürlüğün altındaki farkı zaten süremez" ölçüsü.

    Girdi/çıktı dünya ENU. `circles` kirişi ilgilendirmeyen uzak engelleri de
    içerebilir — uzak engel `min`'i bağlamaz, sonucu değiştirmez.
    """
    lx, ly = left
    rx, ry = right
    w = math.hypot(rx - lx, ry - ly)
    if w <= 1e-9:
        return None
    tx, ty = (rx - lx) / w, (ry - ly) / w          # kiriş boyunca birim vektör
    olu_bant = BUOY_RADIUS_M + cfg.half_beam
    s_lo, s_hi = olu_bant, w - olu_bant
    if s_lo > s_hi:
        return None                                # gövde sığmıyor → kapı değil
    merkez = 0.5 * w                               # s_lo ≤ merkez ≤ s_hi (garanti)

    # Merkezden dışa doğru simetrik ızgara + bandın iki ucu.
    adaylar = [merkez]
    adim = max(cfg.hull_width_m / 4.0, 1e-3)
    for k in range(1, _AIM_MAX_ADIM + 1):
        o = k * adim
        geri, ileri = merkez - o, merkez + o
        if geri < s_lo and ileri > s_hi:
            break
        if geri >= s_lo:
            adaylar.append(geri)
        if ileri <= s_hi:
            adaylar.append(ileri)
    adaylar.append(s_lo)
    adaylar.append(s_hi)

    en_iyi_s = merkez
    en_iyi_a = -math.inf
    for s in adaylar:
        px, py = lx + tx * s, ly + ty * s
        a = _clearance(px, py, circles)
        # Eşitlikte MERKEZE en yakın aday kazanır: engel simetrikse ya da hiç
        # yoksa sonuç geometrik ortadır (belirlenimci, tarama sırasından bağımsız).
        if a > en_iyi_a + 1e-9 or (
            abs(a - en_iyi_a) <= 1e-9 and abs(s - merkez) < abs(en_iyi_s - merkez)
        ):
            en_iyi_s, en_iyi_a = s, a
    return (lx + tx * en_iyi_s, ly + ty * en_iyi_s)


def select_gate(  # noqa: PLR0913
    vehicle: Point,
    coarse_target: Point,
    edge_buoys: Sequence[Point],
    cfg: GateFollowerConfig,
    diag: Optional[GateDiagnostics] = None,
    obstacles: Sequence[Circle] = (),
    gecilmis: Sequence[Circle] = (),
    kurs: Optional[Point] = None,
) -> Optional[Gate]:
    """Öndeki en yakın geçerli kapıyı seç (durumsuz, saf fonksiyon).

    Adımlar:
      1. Kurs eksenini (ileri/sol) ham GN yönünden kur.
      2. Her kenar dubasını (ileri, lateral) uzayına projelendir; yalnız burun
         hattının ÖNÜNDE olanları tut (üst menzil kapağı YOK — menzili algı
         katmanı belirler).
      3. Çiftleri değerlendir: (a) merkez mesafesi ≥ `min_passable_width`
         (= gövde + 2r, gövde fiilen sığıyor mu) ve (b) `|Δileri| < |Δyanal|`
         — **çiftin KENDİ radyal çerçevesinde** (araç → o çiftin orta noktası).
      4. Geçerli çiftlerden orta noktası EN YAKIN olanı seç; eşitlikte orta
         noktası kurs çizgisine en yakın olan (küçük |lateral|).
      5. Kazanan kapı için `aim_point` ile engellerden en açık nişanı hesapla.

    🔑 **(b) neden ÇİFTİN KENDİ çerçevesinde ölçülür (seçim ekseni kalıntısı,
    2026-08-06).** Test eskiden kurs ekseninde (araç→GN) yapılıyordu; oysa
    şartname *"görev noktası doğrudan iki kenar dubasının arasında bir nokta
    OLMAYABİLİR"* diyor. GN yana kaçıkken kurs ekseni dönüyor, kapı kirişi o
    eksene göre "dik" olmaktan çıkıyor ve **gerçek bir kapı sessizce
    reddediliyordu** (ölçülen kırılma noktası: GN yönü kapı yönünden **45°**
    saptığında). Araç→orta-nokta ekseni ise kapının kendi geometrisinden gelir:
    "bu çift bakış hattıma DİK mi (kapı) yoksa BOYUNCA mı dizili (ardışık
    kapılar)" — GN'nin nerede olduğu sonucu değiştirmez. Ayrım noktası yine tam
    45°, yine ölçek-bağımsız; yalnız ölçüldüğü çerçeve doğrusuna taşındı.

    ⚠ **Kalan GN bağımlılığı bilinçli:** (2)'deki "önümüzde mi" süzgeci ve
    (4)'teki eşitlik bozucu hâlâ kurs eksenini kullanır. Bunlar kaba: bir duba
    ancak GN yönüne göre **neredeyse tam kenarda/arkada** kalırsa elenir
    (10 m'deki bir duba için ~87°). Yani sessiz-ret penceresi 45°'den ~87°'ye
    daraldı. Tamamen kaldırmak "önde" kavramını gerektirir; onsuz araç geçtiği
    kapıya geri dönebilir. Aracın heading'i de çözmez: kapı seçilmediği için
    araç zaten GN'ye doğru yönelmiş olur (aynı yön, aynı sonuç).

    🔴 **K1 — `gecilmis` parametresi tam O "önde" kavramıdır (2026-08-06).**
    Yukarıdaki *"araç geçtiği kapıya geri dönebilir"* riski kapalı döngüde
    ÖLÇÜLDÜ ve sonsuz salınım çıktı (GIRDAP_DURUM §0.9b): P1'de engel bile
    yokken araç 199 s boyunca kapı-1 ↔ kapı-2 arasında gidip geldi, P2'de
    300 s'de 4 waypoint'in 1'ini bitirdi (88 m GERİ gidiş). Zincir:
      ① araç kapıya **eğik** yaklaşınca (dönüş savrulması / engelden kaçış)
         öndeki gerçek kapı (b) testine takılır — o açıdan kiriş bakış hattı
         BOYUNCA durur, "kapı değil" sayılır;
      ② geriye tek geçerli aday olarak ARKADAKİ kapı kalır ve kurs ekseni GN'ye
         doğru ~90° dönmüş olduğu için "önde" görünür;
      ③ geçilmiş kapılar hiçbir yerde elenmediği için yeniden seçilir → araç
         geri döner → yaklaşma düzelir → ileri kapı yine geçerli olur → ...
    Düzeltme ③'ü kapatır: **yanından geçtiğim bir kapı bir daha aday olamaz.**
    Ayırt etme ölçüsü GEÇİLEN kapının kendi yarı genişliğidir (öz-ölçekli, dış
    eşik yok). ①-② dokunulmadan bırakıldı: onlar sessiz ret üretir, ③ ise aracı
    geriye sürüyordu; ③ kapalıyken ①'in sonucu "bir tick ham GN'ye düş, açı
    düzelince kilitlen" gibi kendi kendini düzelten bir davranışa iniyor.
    ⚠ "Geçtim" kaydı YANAL KOŞULLUdur (bkz. `GateFollower._kaydet_gecilmis`):
    kapının bir genişliği kadar yanından geçmek gerekir. Kapı düzlemini çok
    uzaktan kesen araç için kapı HÂLÂ ADAYDIR — yoksa henüz yaklaşmakta olduğu
    kapıyı, düzlemine teğet geçer geçmez kalıcı olarak kaybederdi (ölçüldü).

    Dönüş: Gate ya da geçerli kapı yoksa None. Kenar dubaları frame'i = girdi
    frame'i (dünya ENU); left/right yaklaşma yönüne göre etiketlenir.
    `gecilmis`: arkada bırakılmış kapılar — `(orta_x, orta_y, yarı_genişlik)`
    üçlüleri (dünya ENU); yarıçap eşleştirme penceresidir.
    """
    if diag is not None:
        diag.n_edge_buoys = len(edge_buoys)
    if len(edge_buoys) < 2:
        return None
    axes = _forward_left_axes(vehicle, coarse_target)
    if axes is None:
        return None
    fx, fy, lx, ly = axes
    vx, vy = vehicle

    # (buoy, ileri, lateral) — sadece öndeki menzil içi dubalar.
    projected: List[Tuple[Point, float, float]] = []
    for bx, by in edge_buoys:
        rx, ry = bx - vx, by - vy
        fwd = rx * fx + ry * fy
        # Yalnız burun hattının ÖNÜNDEKİLER. Üst menzil kapağı YOK: menzili
        # algı katmanı belirler (LiDAR/kamera zaten ne görüyorsa onu verir);
        # buraya ikinci bir sayı koymak tahmin olurdu.
        if fwd < cfg.min_forward:
            continue
        lat = rx * lx + ry * ly
        projected.append(((bx, by), fwd, lat))

    if diag is not None:
        diag.n_in_range = len(projected)
    if len(projected) < 2:
        return None

    # 🔴 F-K.3 (13.08.2026) — SIRALAMA ANAHTARI: KURS BOYUNCA AYRIM.
    # Kaptan: *"TEKNOFEST'te bir gate'de 2 gate'in dubalarını sağda
    # görebiliyorsun."* Ölçüldü: gerçek geometride (12 m açıklık, 4 m aralık)
    # **50 aday çiftin 43'ü SAHTE** — iki AYRI kapının dubalarından kurulmuş.
    # Sanal gölde tekne 8,06 m genişlikte sahte bir kapıya kilitlenip dondu.
    #
    # 📏 ÖLÇÜM (35 araç konumu, en iyi sıradaki aday gerçek mi):
    #     mevcut (menzile göre)      → **%26**
    #     dikeylik oranına göre      → %20
    #     **kurs BOYUNCA ayrıma göre → %100**
    # Gerçek kapının iki direği kurs ekseninde AYNI istasyondadır (ayrım ≈ 0);
    # çapraz çiftler ise kapı aralığının katları kadar ayrıktır (4 m, 8 m…).
    # Eşik DEĞİL, SIRALAMA — §0.0d korunur.
    #
    # ⚠ KURS EKSENİ ARAÇ→GN'DEN ALINAMAZ: GN kaçık olduğu için (şartname md
    # 5.5.2.2; §0.17c'de 2,0-6,4 m ölçüldü) tekne yana kaçınca eksen 37°'ye
    # kadar döner ve ölçüt çöker (aynı taramada %20'ye düşüyor). Eksen
    # PARKURUN kendi ekseninden gelir: `GateFollower` onu GEÇİLEN SON KAPININ
    # NORMALİNDEN alır — kapılar tanımı gereği kursa diktir, yani eksen kendini
    # besler ve dışarıdan sayı gerektirmez. İlk kapıda eksen yoktur → eski
    # (menzil) sıralaması kullanılır, davranış birebir korunur.
    best: Optional[Tuple[Tuple[float, float], Gate]] = None
    n = len(projected)
    for i in range(n):
        pi, _fi, li = projected[i]
        for j in range(i + 1, n):
            pj, _fj, lj = projected[j]
            if diag is not None:
                diag.n_pairs_checked += 1
            sep = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
            # (a) GEÇİLEBİLİRLİK — gövde sığmıyorsa bu bir kapı değildir.
            # `sep` MERKEZDEN merkeze; serbest açıklık `sep − 2r` olduğu için
            # eşik gövde genişliği DEĞİL `hull + 2r`'dir (bkz.
            # `min_passable_width`). Üst sınır YOK: kapı ne kadar geniş olursa
            # olsun geçilebilir.
            if sep < cfg.min_passable_width:
                if diag is not None:
                    diag.reddedilen_genislik.append(sep)
                continue
            midpoint = (0.5 * (pi[0] + pj[0]), 0.5 * (pi[1] + pj[1]))
            # (a2) K1 — DÜZLEMİ ZATEN GEÇİLMİŞ Mİ? Geçtiğimiz bir kapı bir daha
            # hedef olamaz; bu, "önde" kavramının kurs ekseninden bağımsız tek
            # güvenilir hâlidir (docstring K1).
            # 🔑 Ölçü GEÇİLEN kapının kendi yarı genişliğidir, adayınki DEĞİL:
            # aday, iki AYRI kapının dubasından kurulmuş sahte bir çift olabilir
            # (20 m açıklık → 10 m yarıçap) ve o kadar geniş bir pencere, 10 m
            # ötedeki geçilmiş bir kapıyla yanlış eşleşir. Ölçümde tam bu oldu:
            # tek gerçek eşleşme yerine 3 çift eleniyordu.
            if gecilmis and any(
                math.hypot(midpoint[0] - gx, midpoint[1] - gy) <= g_yari
                for gx, gy, g_yari in gecilmis
            ):
                if diag is not None:
                    diag.reddedilen_gecilmis += 1
                continue
            # (b) BAKIŞ HATTINA DİK Mİ — bir kapı, ona bakan hatta dik duran
            # çifttir; ardışık kapıların dubaları ise o hat BOYUNCA dizilir.
            # |Δileri| < |Δyanal| tam olarak "paralel olmaktan çok dike yakın"
            # demek (ayrım noktası 45°: ayarlanan eşik değil, iki hâl
            # arasındaki geometrik sınır). Ölçek-bağımsız → kapı genişliğini
            # bilmeyi GEREKTİRMEZ. Çerçeve ÇİFTİN KENDİSİNDEN gelir (araç →
            # o çiftin orta noktası), kurs ekseninden DEĞİL: GN yana kaçıkken
            # gerçek kapının sessizce reddedilmesi böyle kapanır (docstring).
            pair_axes = _forward_left_axes(vehicle, midpoint)
            if pair_axes is None:
                continue                       # kapı tam aracın üstünde: dejenere
            pfx, pfy, plx, ply = pair_axes
            ddx, ddy = pi[0] - pj[0], pi[1] - pj[1]
            d_fwd = ddx * pfx + ddy * pfy
            d_lat = ddx * plx + ddy * ply
            if abs(d_fwd) >= abs(d_lat):
                if diag is not None:
                    diag.reddedilen_derinlik += 1
                continue
            # +lateral = sol; Δyanal'ın işareti hangi dubanın solda olduğunu
            # doğrudan verir (yaklaşma yönüne göre, kurs eksenine göre değil).
            left, right = (pi, pj) if d_lat >= 0.0 else (pj, pi)
            gate = Gate(left=left, right=right, midpoint=midpoint)
            # F-K.3 sıralaması (yukarıdaki blokta gerekçesi):
            #   kurs EKSENİ VARSA → (kurs boyunca ayrım, menzil)
            #   yoksa (ilk kapı)  → (menzil, |kurs çizgisine uzaklık|) — eski.
            menzil = math.hypot(midpoint[0] - vx, midpoint[1] - vy)
            mid_lat = 0.5 * (li + lj)
            if kurs is not None:
                kx, ky = kurs
                boyuna = abs(ddx * kx + ddy * ky)   # kurs BOYUNCA ayrım
                # ⚠ HAM `boyuna` ile sıralama YANLIŞ: sürekli bir sayı olduğu
                # için UZAKTAKİ bir kapı 0,01 m farkla yakındakini geçer ve
                # nişan zinciri kopar (P1 saha senaryosunda ölçüldü: geçiş
                # ortalaması 2,02 m → 2,60 m KÖTÜLEŞTİ). Bu yüzden "aynı
                # istasyon" DUBA ÇAPINA (şartname: 30 cm) göre kovalanır:
                # gerçek kapılar aynı kovaya düşüp eşitlenir, aralarında
                # MENZİL karar verir (eski davranış). Ölçülen fiziksel boyut,
                # ayarlanabilir eşik değil (§0.0d).
                kova = int(boyuna / (2.0 * BUOY_RADIUS_M))
                key = (float(kova), menzil)
            else:
                key = (menzil, abs(mid_lat))
            if best is None or key < best[0]:
                best = (key, gate)

    if best is None:
        return None
    gate = best[1]

    # Nişan noktası YALNIZ kazanan çift için hesaplanır (her aday için değil —
    # tarama gereksiz maliyet olurdu). Açıklık dairelerine kapının kendi
    # direkleri de girer: MPPI'de engel olmadıkları için iten tek kuvvet budur.
    # Diğer kenar dubaları da katılır — koridora sarkan üçüncü bir duba
    # nişanı ondan uzağa iter (yanlış eşleşmenin sessiz zararını azaltır).
    circles: List[Circle] = [(bx, by, BUOY_RADIUS_M) for bx, by in edge_buoys]
    circles.extend(obstacles)
    aim = aim_point(gate.left, gate.right, circles, cfg)
    # Kapının KENDİ ileri normali: kirişe dik iki adaydan YAKLAŞMA yönüyle aynı
    # tarafta olanı. İşaret BİR KEZ burada belirlenir; sonrasında "geçildi"
    # testi kurstan da GN'nin yerinden de bağımsızdır (B4/B6).
    # İşaret kaynağı araç→orta nokta (GN değil): kapıya fiilen hangi taraftan
    # yaklaştığımız budur, ve (b) testiyle aynı çerçeve olduğu için tutarlı.
    # Adayın işareti kesin: çift (b) testini geçtiği için kiriş bakış hattına
    # PARALEL olmaktan çok DİK — yani n·f sıfıra yakın olamaz.
    yaklasma = _forward_left_axes(vehicle, gate.midpoint)
    normal = (
        None if yaklasma is None
        else _gate_normal(gate.left, gate.right, (yaklasma[0], yaklasma[1]))
    )
    gate = Gate(
        left=gate.left, right=gate.right, midpoint=gate.midpoint,
        aim=aim, normal=normal,
    )
    if diag is not None:
        diag.secilen_genislik = gate.width
        diag.aim_kaymasi_m = gate.aim_shift
    return gate


class GateFollower:
    """Durumlu kapı takipçisi — seçime histerezis (kapıya kilitlen, geçince bırak).

    Saf `select_gate` her tick'te öndeki en yakın kapıyı verir; bu yeterince
    kararlıdır ama (a) kapı anlık görüşten çıkınca (oklüzyon/dalga) hedef zıplar,
    (b) geçiş anında bir sonraki kapıya erken atlayabilir. Bu sınıf seçilen
    kapıya KİLİTLENİR, taze algıyla günceller, ancak orta noktasını GEÇİNCE
    serbest bırakıp sonraki kapıyı seçer. Kapı hiç yoksa ham GN'ye düşer.

    **B5 — kilitlenmeden önce onay (2026-08-06):** kilitlenme, aynı kapı
    `ONAY_TICK` kez üst üste görülene kadar ERTELENİR; o ana kadar hedef ham
    GN'dir (kapısız davranışın birebir aynısı). Sebep: oklüzyon koruması ile
    onaysız kilit birleşince **tek karelik bir yanlış tespit kalıcı hedef**
    oluyordu (bkz. `ONAY_TICK`). Kilitlendikten sonraki koruma değişmedi.

    **K1 — geçilen kapı bir daha aday olamaz (2026-08-06).** Kilit bırakılırken
    kapının orta noktası `_gecilen_kapilar`'a yazılır ve `select_gate` onu eler.
    Olmadığında kapalı döngüde ÖLÇÜLEN davranış: araç kapıya eğik yaklaşınca
    öndeki kapı geometrik testte elenir, arkadaki kapı "önde" görünüp yeniden
    seçilir ve tekne iki kapı arasında SONSUZ salınır (P1'de 199 s, P2'de 88 m
    geri gidiş — GIRDAP_DURUM §0.9b). Ayrıntılı gerekçe `select_gate` docstring'i.

    Durum minimaldir (bir kilitli kapı + bir aday + arkadakilerin listesi);
    MPPI'nin warm-start'ı + FSM güvenlik çatısı zaten üstte — aşırı mühendislik yok.
    """

    def __init__(self, cfg: Optional[GateFollowerConfig] = None) -> None:
        self._cfg = cfg or GateFollowerConfig()
        self._committed: Optional[Gate] = None
        # B5 — kilitlenme ÖNCESİ onay penceresi: aday kapı + üst üste görülme
        # sayısı. `ONAY_TICK`'e ulaşmadan kilitlenilmez (bkz. sabit).
        # F-K.5: bayat kilit bırakma durumu — kilitlendiği andaki mesafe (d0),
        # o gün bu yana katedilen yol ve son araç konumu.
        self._kilit_d0: Optional[float] = None
        self._kilit_yolu: float = 0.0
        self._son_arac: Optional[Point] = None
        self._aday: Optional[Gate] = None
        self._aday_sayaci: int = 0
        # Onayı en son ilerleten algı karesinin kimliği (bkz. update/gozlem_no).
        self._son_onay_gozlemi: Optional[int] = None
        # Son update()'in teşhis sayaçları — node bunu loglar (sessiz ret kapanı).
        self.last_diagnostics = GateDiagnostics()
        # Şartname G1/G2 = "FARKLI karşılıklı kenar dubaları arasından geçiş
        # sayısı". Düzlem-kesişimi testi (B4/B6) bu sayacı bedavaya veriyor:
        # aynı hesap hem kilidi bırakıyor hem geçişi sayıyor.
        self._passed_midpoints: List[Point] = []
        # K1 — ARKADA BIRAKILAN kapılar: YANINDAN geçilmiş (bir kapı genişliği
        # içinde) her kapı — aradan geçilmiş (sayıldı) ya da geçilmemiş olsun.
        # `_passed_midpoints` PUAN kanıtıdır (yalnız aradan geçilenler);
        # bu liste GEOMETRİdir (arkada ne kaldı) — ikisi bilerek ayrı.
        # Eleman: (orta_x, orta_y, YARI GENİŞLİK) — eşleştirme yarıçapı kapının
        # KENDİ ölçüsüdür (bkz. select_gate K1 notu).
        self._gecilen_kapilar: List[Circle] = []
        # F-K.3: PARKURUN kurs ekseni — geçilen son kapının normali. Kapılar
        # tanımı gereği kursa diktir, yani eksen KENDİNİ BESLER; dışarıdan
        # sayı ya da yeni bir topic gerekmez. İlk kapıya kadar None → eski
        # (menzil) sıralaması, davranış birebir korunur.
        self._kurs_ekseni: Optional[Point] = None

    def reset(self) -> None:
        """Kilitli kapıyı temizle (parkur geçişi / yeniden başlama).

        ⚠ Geçiş SAYACINA dokunmaz: sayaç puan kanıtıdır, parkur geçişinde
        sıfırlanmaz. Yeniden başlama için `reset_passed_gates()` kullan.
        ⚠ K1 listesine (`_gecilen_kapilar`) de dokunmaz: parkur değişse de o
        kapılar FİZİKSEL olarak hâlâ arkadadır; temizlemek salınımı geri
        getirirdi. Yalnız yeniden başlama (araç fiilen başa döner) temizler.
        """
        self._committed = None
        # Onay penceresi de sıfırlanır: parkur değişince yarım kalmış bir aday
        # yeni parkurun ilk kapısıymış gibi sayılmamalı.
        self._aday = None
        self._aday_sayaci = 0
        self._son_onay_gozlemi = None
        self._kilit_d0 = None
        self._kilit_yolu = 0.0
        self._son_arac = None

    def reset_passed_gates(self) -> None:
        """Geçiş sayacını sıfırla — YALNIZ yeniden başlamada.

        Şartname md 5.5.3.1: *"Yeniden başlama hakkını kullanan takımın
        topladığı puanlar SIFIRLANACAKTIR."* İkinci turda aynı geçitlerden
        yeniden geçilir; hafıza temizlenmezse hepsi "zaten geçildi" sayılır
        ve HİÇBİRİ sayılmaz.

        K1 listesi de burada temizlenir — aynı gerekçe: araç fiilen başa
        döndüğü için o kapılar artık ARKADA değil, ÖNDEDİR. (Temizlenmezse
        ikinci turda hiçbir kapı aday olamaz ve araç ham GN'lere sürer.)
        """
        self._passed_midpoints.clear()
        self._gecilen_kapilar.clear()

    @property
    def committed_gate(self) -> Optional[Gate]:
        return self._committed

    @property
    def passed_gate_count(self) -> int:
        """Geçilen FARKLI kapı sayısı (şartname G1/G2 tanımı)."""
        return len(self._passed_midpoints)

    @property
    def gecilen_kapilar(self) -> List[Circle]:
        """K1 — arkada bırakılanlar: (orta_x, orta_y, yarı genişlik) kopyası.

        Puan kanıtı DEĞİLDİR (`passed_gate_count` odur): aradan geçilmeden
        yanından geçilen kapılar da buradadır. Teşhis/test için açıldı.
        """
        return list(self._gecilen_kapilar)

    def _say_gecis(self, gate: Gate) -> None:
        """Geçilen kapıyı say — daha önce sayılanlardan FARKLIYSA.

        Ayırt etme ölçüsü kapının KENDİ yarı genişliği: manevra/geri dönüş
        yüzünden aynı kapıdan tekrar geçmek sayılmaz, ama komşu bir kapı
        (en az bir kapı genişliği ötede) ayrı sayılır. Öz-ölçekli — dışarıdan
        bir mesafe eşiği GEREKTİRMEZ (modülün tasarım kuralı).
        """
        yari = gate.width / 2.0
        for px, py in self._passed_midpoints:
            if math.hypot(gate.midpoint[0] - px, gate.midpoint[1] - py) <= yari:
                return                      # aynı kapı — tekrar sayma
        self._passed_midpoints.append(gate.midpoint)

    def _kaydet_gecilmis(self, gate: Gate, vehicle: Point) -> None:
        """K1 — kapıyı "arkada" olarak işaretle (yeniden aday olmasın).

        `_say_gecis`'ten AYRI: orada yalnız ARADAN geçilenler (puan kanıtı)
        birikir; burada kapının YANINDAN geçilenler de birikir, çünkü soru
        "sayıldı mı" değil **"arkamda mı"**dır.

        🔑 **Yanal koşul ŞART — kapı düzlemi SONSUZDUR.** Bu koşul olmadan,
        kapıya hâlâ 3 m yandan yaklaşmakta olan araç düzlemi teğet geçer
        geçmez kapı "arkada" yazılıyor ve **kalıcı olarak kaybediliyordu**
        (ölçüm: kapı-2 bu yüzden hiç geçilemedi). Ölçü kapının KENDİ
        genişliği — yarı genişlik "aradan geçtim" (puan) ölçüsüdür; bir tam
        genişlik "o kapının yanındaydım" demektir. Öz-ölçekli, dış eşik yok.
        Daha uzaktan düzlemi kesen araç için kapı hâlâ ADAYDIR: açı düzelince
        yeniden kilitlenip usulünce geçilir.
        """
        yanal = gate.lateral_offset(vehicle)
        if yanal is not None and yanal > gate.width:
            return                       # kapının yanından değil, uzağından geçtik
        if gate.normal is not None:
            self._kurs_ekseni = gate.normal        # F-K.3: kurs ekseni öğrenildi
        yari = gate.width / 2.0
        for px, py, p_yari in self._gecilen_kapilar:
            if math.hypot(gate.midpoint[0] - px, gate.midpoint[1] - py) <= p_yari:
                return
        self._gecilen_kapilar.append((gate.midpoint[0], gate.midpoint[1], yari))

    def update(
        self,
        vehicle: Point,
        coarse_target: Point,
        edge_buoys: Sequence[Point],
        obstacles: Sequence[Circle] = (),
        gozlem_no: Optional[int] = None,
    ) -> GateResult:
        """Bir kontrol tick'i: rafine hedefi (kapı nişanı ya da ham GN) üret.

        `obstacles`: dünya ENU dairesel engeller (sarı duba, UNKNOWN küme…).
        Verilmezse nişan geometrik ortaya düşer — eski davranış birebir.

        `gozlem_no`: **algı karesinin kimliği** — B5 onay sayacı yalnız bu
        değer DEĞİŞTİĞİNDE ilerler.
        🔑 **Neden gerekli:** kontrol tick'i ile algı karesi aynı hızda DEĞİL.
        `planning_node` hedefi 5 Hz tazeler; algı ise 10 Hz de olabilir, kapalı
        alanda ölçüldüğü gibi ~1 Hz de (§11.3, kümeleme 1-3,3 s/kare). Sayaç
        çağrı başına ilerleseydi, algı yavaşladığında AYNI karenin iki kez
        sayılmasıyla onay boşa çıkardı — yani B5 tam da en çok gerektiği
        (algının zorlandığı) durumda susardı. `None` verilirse her çağrı ayrı
        gözlem sayılır (çekirdeği doğrudan çağıran testler/simülasyon için).
        """
        self.last_diagnostics = GateDiagnostics()
        # F-K.5: kilitten bu yana KATEDİLEN YOL — bayat kilit bırakma ölçütü.
        if self._son_arac is not None and self._committed is not None:
            self._kilit_yolu += _dist(self._son_arac, vehicle)
        self._son_arac = vehicle
        fresh = select_gate(
            vehicle, coarse_target, edge_buoys, self._cfg,
            self.last_diagnostics, obstacles,
            gecilmis=self._gecilen_kapilar,          # K1: arkadakiler aday değil
            # 🔴 F-K.3 BAĞLI DEĞİL — KÖK NEDEN BULUNDU (13.08, ölçümle).
            # Varsayım "çapraz çift = zararlı" idi; BU PARKURDA YANLIŞ:
            # kapılar 4 m aralıklı ama ±5 m ZİGZAG, yani ardışık gerçek kapı
            # merkezleri 5 m YANAL atlıyor. Çapraz çiftlerin ortası ise tam
            # iki kapının ARASINDA — koridor ORTA ÇİZGİSİ, her kapıya yalnız
            # 2,5 m. Gerçek merkeze nişan almak tekneyi bir sonraki kapıya
            # ERKEN çekip mevcut kapıyı daha kaçık geçirtiyor.
            # Ölçüldü (P1 kapalı döngü, geçiş sapması ortalaması):
            #     mevcut (çapraz çiftler serbest) → 1,91 m · 7 kapı
            #     F-K.3 sıralama                  → 2,60 m · 5 kapı
            #     F-K.3 red                       → 2,73 m · 5 kapı
            # ⇒ Çapraz çift burada YUMUŞATICI görev görüyor. Ayrıca F-K.3'ü
            # tetikleyen sanal göl kilitlenmesi zaten F-K.1b (havuç) ile
            # kapandı ("yalnız havuç" koşumu PARKUR TAMAMLANDI).
            # Mekanizma + ölçüm testleri duruyor; SEYREK kapılı bir parkurda
            # (aralık ≫ zigzag genliği) yeniden değerlendirilmeli.
            kurs=None,
        )

        if self._committed is not None:
            axes = _forward_left_axes(vehicle, coarse_target)
            # Geçildi mi? — ÖNCE kapının KENDİ normali (B4/B6), yalnız o yoksa
            # eski kurs ekseni. "Geçildi" = kapı düzlemi ARKADA kaldı; tam
            # geometrik tanım (eski `release_distance`=0.8 m bir tahmindi ve
            # kapıyı erken bırakıyordu).
            d = self._committed.signed_distance(vehicle)
            if d is not None:
                passed = d > 0.0
                # Direklerin ARASINDAN mı geçtik yoksa yandan mı DOLAŞTIK?
                # Kilit her iki hâlde de bırakılır (kapı arkada kaldı), ama
                # yalnız aradan geçiş şartnamenin saydığı geçiştir (G1/G2).
                if passed and self._committed.passed_between(vehicle):
                    self._say_gecis(self._committed)
            else:
                # Normal yok (elle kurulmuş Gate) → geriye uyumlu eski test.
                passed = True
                if axes is not None:
                    fx, fy, _, _ = axes
                    mx, my = self._committed.midpoint
                    fwd = (mx - vehicle[0]) * fx + (my - vehicle[1]) * fy
                    passed = fwd <= 0.0
            if not passed:
                # Kapıya hâlâ gidiyoruz. Taze algı aynı kapıyı görüyorsa
                # drift'i güncelle; görmüyorsa (oklüzyon) kilitli kapıyı KORU.
                # "Aynı kapı mı" ölçütü kapının KENDİ genişliğinin yarısı:
                # öz-ölçekli, dolayısıyla kapı 2 m de olsa 40 m de olsa
                # çalışır ve dışarıdan bir sayı gerektirmez (eski
                # `match_radius`=2.5 m bir tahmindi ve geniş kapılarda drift
                # güncellemesini kaçırırdı).
                if fresh is not None and _dist(
                    fresh.midpoint, self._committed.midpoint
                ) <= self._committed.width / 2.0:
                    self._committed = fresh
                    self._kilit_yolu = 0.0     # taze görüldü → yol sayacı sıfır
                # 🔴 F-K.5 (16.08, §1.22) — BAYAT KİLİT BIRAKILIR.
                # Oklüzyon koruması kilidi taze algı olmadan da tutuyordu ve
                # bırakma YALNIZ düzlem geçilince oluyordu; sahada bunun sonucu
                # 12:02'de kurulan 10,8 m'lik hayalet kapının **30 dakika**
                # kilitli kalması ve o süre boyunca hiçbir gerçek kapının
                # seçilememesi oldu (geçilen kapı: 0).
                # Ölçüt AYARLANABİLİR BİR SÜRE DEĞİL, geometrik: kilitlendiği
                # anda kapı `d0` uzaktaydı; manevrayla birlikte en fazla ~2·d0
                # yol yeter. Bu yolu katettiğimiz hâlde kapı ne geçildi ne de
                # tazelendiyse, o kapı orada değildir.
                if (
                    self._kilit_d0 is not None
                    and self._kilit_yolu > _BAYAT_KILIT_KATSAYI * self._kilit_d0 + self._committed.width
                ):
                    self._committed = None     # geçilmiş SAYILMAZ, yalnız bırakılır
                    self._kilit_d0 = None
                    self._kilit_yolu = 0.0
                    return GateResult(
                        target=coarse_target, gate=None, used_fallback=True,
                        surus_hedefi=coarse_target,
                    )
                return GateResult(
                    target=self._committed.drive_target,
                    gate=self._committed,
                    used_fallback=False,
                    surus_hedefi=self._committed.surus_noktasi(
                        vehicle,
                        self._committed.width / 2.0,      # öz-ölçekli hizalanma payı
                        self._cfg.hull_length_m,
                    ),
                )
            # Geçildi → serbest bırak, aşağıda yeniden seç.
            # K1: bırakmadan ÖNCE "arkada" olarak işaretle — yoksa bir sonraki
            # tick'te aynı kapı yeniden aday olur (ölçülen sonsuz salınım).
            self._kaydet_gecilmis(self._committed, vehicle)
            self._committed = None

        # Kilitli kapı yok. B5 ONAY KAPISI: aynı kapı `ONAY_TICK` kez üst üste
        # görülmeden kilitlenilmez; o ana kadar hedef ham GN'dir (kapısız
        # davranışın birebir aynısı, yani onay beklemek hiçbir şey bozmaz).
        if fresh is None:
            self._aday = None
            self._aday_sayaci = 0
            return GateResult(
                target=coarse_target, gate=None, used_fallback=True,
                surus_hedefi=coarse_target,
            )

        # Sayaç yalnız YENİ bir algı karesinde ilerler (bkz. `gozlem_no`);
        # aynı kareden gelen ikinci bir kontrol tick'i onayı ilerletemez.
        if gozlem_no is None or gozlem_no != self._son_onay_gozlemi:
            # "Aynı aday mı" ölçütü kilitli kapıdaki drift testiyle AYNI:
            # kapının kendi yarı genişliği (öz-ölçekli, dış eşik gerekmez).
            if self._aday is not None and _dist(
                fresh.midpoint, self._aday.midpoint
            ) <= self._aday.width / 2.0:
                self._aday_sayaci += 1
            else:
                self._aday_sayaci = 1
            self._aday = fresh          # aday hep taze geometriyle taşınır
            self._son_onay_gozlemi = gozlem_no
        self.last_diagnostics.aday_onay_sayaci = self._aday_sayaci

        if self._aday_sayaci < ONAY_TICK:
            return GateResult(
                target=coarse_target, gate=None, used_fallback=True,
                surus_hedefi=coarse_target,
            )

        self._committed = fresh
        self._kilit_d0 = _dist(vehicle, fresh.midpoint)   # F-K.5 referans mesafe
        self._kilit_yolu = 0.0
        self._aday = None
        self._aday_sayaci = 0
        return GateResult(
            target=fresh.drive_target, gate=fresh, used_fallback=False,
            surus_hedefi=fresh.surus_noktasi(
                vehicle, fresh.width / 2.0, self._cfg.hull_length_m
            ),
        )


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

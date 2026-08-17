#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GİRDAP İDA — geçit mantığı (SAF katman, ROS/depthai bağımsız).

Neden ayrı: puanın tamamı geçitten geliyor — Parkur-1 `(G1/KD1)×10`,
Parkur-2 `(G2/KD2)×40` (şartname md 5.5.4.2) — ama `duba_gecis_navigator.py`
depthai+rclpy import ettiği için kamerasız test EDİLEMİYORDU. Puanı üreten
mantık buraya alındı: masada, donanımsız, testli.

İçerik ve gerekçeleri:
  1) Pinhole ile menzil (D = f·W/w_px). OAK-D Lite baseline 7,5 cm → 480P
     stereoda ~8 m ötesinde Z hatası metrelerce (Luxonis depth-accuracy).
     Duba ÇAPI (30 cm) bilinen ve SABİT bir ölçü: şartname "dubaların su
     üstünde kalan yüksekliği o anki şartlara bağlıdır" diyor → yükseklik
     güvenilmez, ÇAP güvenilir. Bu yüzden genişlikten menzil.
  2) Stereo ↔ mono çapraz kontrolü: iki bağımsız ölçüm çelişirse çift atılır.
  3) Yanlış çift elemesi: iki kenar dubası "karşılıklı" sayılacaksa
     ARALARINDA başka kenar dubası olmamalı (varsa koridorun iki yanı
     seçilmiştir → yanlış geçitten sürüş = parkur dışı cezası).
  4) FARKLI geçit sayımı: şartname G'yi "FARKLI karşılıklı kenar dubaları
     arasından geçiş sayısı" diye tanımlar; aynı geçitten tekrar geçmek
     sayılmaz → geçilen geçit orta noktaları dünya çerçevesinde tutulur.
  5) Parkur eşikleri: P1 ORANSAL (G1/KD1 ≥ 0,5), P2 MUTLAK (≥2 ikili).
"""
import math

# Şartname md 5.5.2.1: kenar/engel dubası armut tip, çap 30 cm.
DUBA_CAP_M = 0.30
# IMX214 (OAK-D Lite CAM_A) yatay görüş açısı — Luxonis donanım dokümanı.
HFOV_RAD = math.radians(69.0)


def odak_px(genislik_px: float, hfov_rad: float = HFOV_RAD) -> float:
    """Pinhole odak uzaklığı (piksel): f = (W/2) / tan(HFOV/2).

    YATAY eksen her iki ön işlemede de tam FOV'u korur (deploy'daki SIKIŞTIRMA
    yatayda sadece ölçekler; letterbox'a dönülse şerit yalnız dikeye eklenir)
    → yatay piksel ölçeği doğrudan HFOV'a bağlanır. Menzil bu yüzden dikeyden
    DEĞİL yataydan hesaplanır: dikey, ön işleme değişirse kayan eksendir.
    """
    return (genislik_px / 2.0) / math.tan(hfov_rad / 2.0)


#: Deploy ön işlemesi 4:3 kareyi 1:1'e **SIKIŞTIRIYOR** (`setPreviewKeepAspectRatio(False)`),
#: yani dikey piksel ölçeği yatayınkinden 4/3 kat büyük. `odak_px` YATAY odağı
#: verir; su hattı hesabı DİKEY eksende olduğu için bu çarpan şart.
#: 🔴 Ön işleme letterbox'a dönerse bu sayı da değişir — ikisi ASLA ayrı
#: değişmez (aynı sözleşme: `duba_gecis_navigator.py:272-280`).
DIKEY_ODAK_KATI = 4.0 / 3.0


def su_hatti_menzili(cy_alt_norm: float, ufuk_cy: float, egim: float,
                     pay_cy: float = 0.0):
    """Su değme noktasından menzil (m) — **en CÖMERT** (en uzak) tahmin.

    Yüzen cisim su üzerindedir; bbox'ın ALT kenarı su değme noktasıdır:
        cy_alt = ufuk + (h·f_y)·(1/d)      ⇒      d = egim / (cy_alt − ufuk)
    `egim` = h·f_y ve `ufuk_cy` **veriden çıkarıldı** (aşağıya bak), pitch
    işaret konvansiyonuna hiç bağlanmadan.

    `pay_cy` (dalga/trim belirsizliği) ufku **aşağı** alarak paydayı küçültür
    ⇒ dönen değer *"olabileceğin en uzağı"*. Bu yüzden bununla kurulan çelişki
    kapısı tek yönlü ve güvenlidir: ufka yakın (uzak) cisimlerde payda sıfırın
    altına düşer ve **None** döner = *"çelişki İDDİA EDEMEM"*.

    🔴 09.08'de reddedilen "ufuk süzgeci"nin aynısı DEĞİL: o, ufkun üstünü
    kesip atıyordu ve pitch ±5° belirsizliği onu güvensiz yapıyordu. Burada
    aynı belirsizlik **kapıyı gevşetmek** için kullanılıyor.
    """
    if egim is None or egim <= 0.0:
        return None
    dy = (float(cy_alt_norm) - float(ufuk_cy)) - float(pay_cy)
    if dy <= 1e-3:
        return None                     # ufka çok yakın ⇒ çelişki iddia edilemez
    return float(egim) / dy


def su_hatti_celiskili(cy_alt_norm: float, d_genislik_m: float,
                       ufuk_cy: float, egim: float, pay_cy: float = 0.0,
                       kat: float = 3.0) -> bool:
    """Su hattı "çok yakın" derken genişlik "uzak" mı diyor? (TEK YÖNLÜ)

    ⚖️ Tek yönlü, bilerek: yalnız *"su hattına göre olduğundan UZAK görünen"*
    adayı eler. Tersi elenmez — dalga, kırpık bbox ve gövdenin kendi
    yansımasıyla birleşmesi o yöne sapma üretebilir.
    """
    if not d_genislik_m or d_genislik_m <= 0.0:
        return False
    d_su = su_hatti_menzili(cy_alt_norm, ufuk_cy, egim, pay_cy)
    if d_su is None:
        return False
    return d_su * float(kat) < float(d_genislik_m)


def mesafe_genislikten(w_px: float, cap_m: float, f_px: float):
    """Bilinen çaptan menzil: D = f·W/w_px. Geçersiz pikselde None.

    Doğruluk: yöntem literatürde 1,1-7,5 m bandında MAE ~0,085 m (%1,94) —
    bizim geçit bandımız (~2-8 m) tam bu aralık. Uzakta w_px küçüldükçe
    (15 m ≈ 6 px) hata büyür; o yüzden menzil tavanı ayrıca sınırlanır.
    """
    if w_px is None or w_px <= 0.0:
        return None
    return f_px * cap_m / w_px


# Mono (pinhole) menzilin KABUL EDİLDİĞİ bant.
# Alt: bu kadar yakında duba kadrajı doldurur; daha küçük genişlik ölçüm değil
# gürültüdür. Üst: 15 m'de 30 cm duba ~6 px kalır (kalibrasyondan: w_px=90,1/Z),
# altında bağıl hata hızla büyür → hayalet duba üretmemek için tavan.
MONO_MENZIL_ALT_M = 0.5
MONO_MENZIL_UST_M = 15.0
# Stereo Z'nin "geçerli" sayıldığı alt sınır (m). depthai geçersiz derinlikte 0 verir.
STEREO_GECERLI_ALT_M = 0.05


def menzil_coz(z_stereo, w_px, f_px, cap_m: float = DUBA_CAP_M,
               alt: float = MONO_MENZIL_ALT_M, ust: float = MONO_MENZIL_UST_M):
    """Tespitin menzili: stereo varsa O, yoksa bbox genişliğinden pinhole YEDEĞİ.

    Döner: ``(menzil_m, kaynak)`` — kaynak ``"stereo"`` | ``"mono"`` | ``None``.

    🔴 NEDEN VAR (2026-08-08): eski kod ``z <= 0.05`` olan tespiti KOMPLE
    atıyordu. Su, derinlik sensörleri için en zor yüzeylerden biri — mesafede
    dokusuz, ışıkta aynasal (NODAR su yüzeyi testi, 11.06.2026: durgun su ayna
    gibi davranıp yüzeyin ALTINDA sahte derinlik üretiyor; dalgalıyken çalışıyor).
    Ayrıca deploy'da ``setDepthUpperThreshold(10000)`` var ⇒ **10 m ötesi zaten
    geçersiz** dönüyor. Sonuç: duba görülüyor ama yayınlanmıyor →
    ``/perception/buoys`` boş → füzyon CLASS_UNKNOWN → ``gate_follower`` ham
    GPS'e düşer → P1 (G1/KD1≥0,5) ve P2 (≥2 ikili) gider, **hata basılmadan**.

    Yedek yol stereo'dan BAĞIMSIZ: duba çapı şartnamede SABİT (md 5.5.2.1,
    Ø30 cm) — yükseklik değil, çünkü su üstünde kalan yükseklik "o anki
    şartlara bağlı". Bu yüzden GENİŞLİKTEN.

    ⚖️ Kapsam bilinçli olarak dar: yalnız **bugün atılan** tespitleri kurtarır,
    geçerli stereo'ya DOKUNMAZ. Absürt sonuç (bant dışı) yine elenir.
    """
    if z_stereo is not None and z_stereo > STEREO_GECERLI_ALT_M:
        return float(z_stereo), "stereo"
    d = mesafe_genislikten(w_px, cap_m, f_px)
    if d is None or not (alt <= d <= ust):
        return None, None
    return float(d), "mono"


def yanal_konum(z_m: float, cx_norm: float, f_px_norm: float) -> float:
    """Kamera çerçevesinde yanal konum (m, sağ +): x = z · (cx − 0,5) / f.

    Mono yedeğe düşüldüğünde stereo'nun X'i de geçersizdir (0 gelir); yanal
    konum bbox merkezinden geometriyle üretilmelidir, yoksa duba kaza eseri
    tam karşımızda sanılır. Normalize bbox (0..1) ve normalize odak ile çalışır.
    """
    if f_px_norm is None or f_px_norm <= 0.0:
        return 0.0
    return float(z_m) * (float(cx_norm) - 0.5) / float(f_px_norm)


def menzil_tutarli(z_stereo, d_mono, bagil_tol: float = 0.35) -> bool:
    """Stereo Z ile mono menzil çelişmiyor mu?

    Ölçümlerden biri yoksa ÇELİŞKİ İDDİA EDİLEMEZ → True (kör reddetme yok).
    Bağıl fark = |z-d| / max(z,d).
    """
    if z_stereo is None or d_mono is None:
        return True
    if z_stereo <= 0.0 or d_mono <= 0.0:
        return True
    return abs(z_stereo - d_mono) / max(z_stereo, d_mono) <= bagil_tol


def arada_duba_var(ax, az, bx, bz, digerleri, z_payi: float = 2.0) -> bool:
    """(a,b) çiftinin ARASINDA başka bir kenar dubası duruyor mu?

    Kamera çerçevesi: x sağ+, z ileri+. Bir aday çiftin bearing aralığında,
    benzer menzilde üçüncü bir kenar dubası varsa o çift "karşılıklı ikili"
    değildir (büyük olasılıkla koridorun iki ayrı tarafından birer duba
    seçilmiştir). Farklı menzildeki dubalar bir SONRAKİ geçide aittir —
    onlar bu kararı bozmamalı.
    """
    ba = math.atan2(ax, az)
    bb = math.atan2(bx, bz)
    lo, hi = (ba, bb) if ba <= bb else (bb, ba)
    z_alt = min(az, bz) - z_payi
    z_ust = max(az, bz) + z_payi
    for (cx, cz) in digerleri:
        if cz <= 0.0:
            continue
        if not (z_alt <= cz <= z_ust):
            continue                      # başka geçide ait
        bc = math.atan2(cx, cz)
        if lo < bc < hi:
            return True
    return False


def letterbox_payi(nn_w, nn_h, kaynak_w, kaynak_h):
    """LETTERBOX üst/alt şerit payı (normalize, 0..0.5). Geçersizse None.

    NEDEN HESAP, NEDEN SABİT DEĞİL: eski kod `_LB_PAY = 0.125` yazıyordu —
    "kaynak 4:3, NN kare" varsayımı. Varsayım bozulursa (kamera çıkışı 16:9
    istenirse, model girişi kare olmazsa) bbox'lar dikeyde KAYAR ve kimse fark
    etmez. DepthAI v3'te `ImgDetections.getTransformation()` gerçek boyutları
    verdiği için (`getSize` / `getSourceSize`, 3.7.1'de doğrulandı) pay
    ÖLÇÜLEBİLİR: varsayıma gerek yok.

    LETTERBOX geometrisi: kaynak, NN karesine oranı bozulmadan sığdırılır →
    ölçek = min(nn_w/kaynak_w, nn_h/kaynak_h); artan yer üst/altta şerit olur.
    """
    if not nn_w or not nn_h or not kaynak_w or not kaynak_h:
        return None
    if nn_w <= 0 or nn_h <= 0 or kaynak_w <= 0 or kaynak_h <= 0:
        return None
    olcek = min(nn_w / kaynak_w, nn_h / kaynak_h)
    icerik_h = kaynak_h * olcek
    return max(0.0, (nn_h - icerik_h) / 2.0 / nn_h)


def bbox_piksel(cx, cy, w, h, lb_pay, hedef_w, hedef_h):
    """NN-normalize bbox → YAYINLANAN piksel uzayında (x, y, w, h).

    NEDEN AYRI PARAMETRE (hedef_w/h ≠ gerçek kare boyutu):
    `/perception/buoys` sözleşmesi bbox'ı PİKSEL uzayında taşır ama mesajda
    görüntü boyutu YOK. Tüketici (perception_fusion_node) merkezi kendi
    `camera_image_width_px` parametresine BÖLEREK normalize eder — yani
    yayınlayanın piksel uzayı ile tüketicinin parametresi AYNI olmak
    zorundadır. Uyuşmazsa hiçbir hata basılmaz, yalnız bearing kayar:
    640 uzayı yayınlayıp 1280'e bölmek her dubayı gerçek açısının yarısında
    gösterir (kare ortasındaki duba +17°'de görünür, tolerans 8,6°) → kamera
    tespiti hiçbir LiDAR kümesine eşleşmez → sınıf bilgisi düşer → geçit
    bulunamaz (P1 şartı G1/KD1≥0,5 · P2 şartı ≥2 ikili).
    Bu yüzden yayın uzayı GERÇEK kare boyutundan bağımsız verilir.

    Yatay: letterbox tam FOV'u korur → doğrudan ölçeklenir.
    Dikey: üst/alt şerit payı çıkarılıp içeriğe geri açılır, sonra kırpılır.
    """
    if not hedef_w or not hedef_h or hedef_w <= 0 or hedef_h <= 0:
        raise ValueError(
            f"gecersiz bbox hedef uzayi: {hedef_w}x{hedef_h} "
            "(tuketicinin camera_image_width_px/height_px degeriyle ayni olmali)"
        )
    icerik = max(1e-6, 1.0 - 2.0 * lb_pay)
    cy_ic = min(1.0, max(0.0, (cy - lb_pay) / icerik))
    h_ic = min(1.0, h / icerik)
    return (cx * hedef_w, cy_ic * hedef_h, w * hedef_w, h_ic * hedef_h)


def yan_yana_mi(a_ileri, a_yanal, b_ileri, b_yanal) -> bool:
    """Bu iki duba bir KAPI mı (kursa dik), yoksa ardışık kapılara mı ait?

    Ölçüt: **|Δileri| < |Δyanal|**. Kapı, kursa DİK duran bir çifttir; ardışık
    kapıların dubaları ise kurs boyunca dizilir. Ayrım noktası tam 45° — bu
    ayarlanan bir eşik değil, iki hâl arasındaki **geometrik sınır**; kapı 2 m
    de olsa 40 m de olsa aynı çalışır.

    Kaynak: ekibin `son_kodv2/karar/prototype/mission/gate_follower.py`
    tasarımı (2026-08-04'te incelendi, bizim sabit `GECIT_MAX_DZ=4 m`
    eşiğimizden daha iyi olduğu için ALINDI).

    Şartname gerekçesi — metre cinsinden sabit eşik TAHMİNDİR:
      · Şekil 3 notu (s.20): "kenar dubaları arasındaki mesafeler yarışma
        alanına göre değişkenlik gösterecektir"
      · s.23: "Duba sayılarına göre bir akış tasarlanmaması tavsiye edilmektedir"
    """
    return abs(a_ileri - b_ileri) < abs(a_yanal - b_yanal)


def gecilebilir_mi(merkez_mesafe, hull_en, duba_cap: float = DUBA_CAP_M) -> bool:
    """Bu açıklıktan tekne fiziksel olarak geçebilir mi?

    Dubalar MERKEZDEN ölçülür → serbest açıklık = merkez_mesafe − duba_cap.
    Gövde sığmıyorsa bu bir kapı DEĞİLDİR (büyük olasılıkla tek duba iki
    tespite bölünmüştür). **Üst sınır YOK** — kapı ne kadar geniş olursa olsun
    geçilebilir; üst sınır koymak yukarıdaki şartname maddeleri karşısında
    tahmin olurdu (eski `GECIT_MAX_GEN=10 m` bu yüzden kaldırıldı).

    NOT: bu bir GEÇİLEBİLİRLİK testidir, emniyet payı DEĞİL. Emniyet payı
    sürüş katmanının işi; algı katmanında pay eklemek gerçek ama dar bir kapıyı
    görünmez kılar ve doğrudan puan kaybettirir ((G/KD)×10 ve ×40).
    """
    return (merkez_mesafe - duba_cap) >= hull_en


def gecitten_gecti(px, py, mx, my, nx, ny, tx, ty, yari_genislik, ek_yol) -> bool:
    """Araç geçidin ARASINDAN mı geçti? (yalnız düzlemi aşmak YETMEZ)

    Dünya çerçevesinde: `(px,py)` araç, `(mx,my)` geçit orta noktası,
    `(nx,ny)` geçide dik İLERİ birim vektör, `(tx,ty)` geçit çizgisi boyunca
    birim vektör, `yari_genislik` = iki duba merkezi arası mesafenin yarısı.

    İKİ şart birden:
      1) ileri projeksiyon > ek_yol  → geçidi (ve kıçı) gerçekten aştı,
      2) |yanal projeksiyon| ≤ yari_genislik → dubaların ARASINDAN geçti.

    (2) olmazsa tekne geçidi es geçip YANDAN dolaşsa bile "geçti" sayılırdı;
    oysa şartnameye göre bu geçiş değildir ve üstüne parkur dışına çıkma
    cezasıdır (md 5.5.4.2). Geriye doğru geçiş de (1) sayesinde sayılmaz —
    normal, geçit görüldüğü andaki burun yönüne kilitlidir.

    `yari_genislik is None` → geçit genişliği bilinmiyor (geçit FOV'dan
    kaybolup tek bearing'den kurulduğu durum): bilgi yokken kör reddetme
    yapılmaz, yalnız (1) uygulanır.
    """
    ileri = (px - mx) * nx + (py - my) * ny
    if ileri <= ek_yol:
        return False
    if yari_genislik is None:
        return True
    yanal = abs((px - mx) * tx + (py - my) * ty)
    return yanal <= yari_genislik


def yeni_gecit_mi(mx: float, my: float, gecilenler, ayirt_m: float = 3.0) -> bool:
    """Bu geçit daha önce geçilenlerden FARKLI mı? (şartname G tanımı)

    `gecilenler`: daha önce geçilmiş geçit orta noktaları [(x, y), ...],
    DÜNYA/odom çerçevesinde. Yeni orta nokta hepsinden `ayirt_m`'den uzaksa
    farklı geçittir. Aksi hâlde aynı geçit tekrar sayılırdı (manevra, geri
    dönüş, tekrar yaklaşma).
    """
    for (px, py) in gecilenler:
        if math.hypot(mx - px, my - py) <= ayirt_m:
            return False
    return True


# FSM durum adları (girdap-decision `prototype/fsm/mission_fsm.py` MissionState).
AKTIF_PARKUR_DURUMLARI = ("PARKUR1", "PARKUR2", "PARKUR3")
# Yeniden başlamada FSM bu durumlara geri döner.
YENIDEN_BASLAMA_DURUMLARI = ("BEKLEMEDE", "ARM", "BOOT")


def sifirlama_gerekir(eski, yeni) -> bool:
    """Yeniden başlama oldu mu? (geçit hafızası sıfırlanmalı mı?)

    Şartname md 5.5.3.1: *"Takımların 1 kereye mahsus yeniden başlama hakkı
    vardır... Yeniden başlama hakkını kullanan takımın topladığı puanlar
    SIFIRLANACAKTIR."* Yani ikinci turda parkur baştan koşulur ve aynı
    geçitlerden yeniden geçilir. Geçilen-geçit hafızası temizlenmezse
    `yeni_gecit_mi` onları "zaten geçildi" sayar ve **hiçbiri sayılmaz**.

    Tetik: aktif bir parkur durumundan başlangıç durumlarına GERİ dönüş.
    TAMAMLANDI/KILL yeniden başlama DEĞİLDİR (sayaç rapor için korunur).
    """
    if not eski or not yeni:
        return False
    return eski in AKTIF_PARKUR_DURUMLARI and yeni in YENIDEN_BASLAMA_DURUMLARI


def gecit_hedefi(ox, oy, px, py, oteleme):
    """Geçit ortasının ÖTESİNDEKİ hedef noktası (dünya/base_link, aynı çerçeve).

    `(ox,oy)` geçit ortası, `(px,py)` geçide DİK ileri birim vektör.
    Hedef ortada olursa araç geçidin içinde durur; geçit çizgisine dik yönde
    ötelenir ki hem komple geçsin hem de çapraz girip dubaya sürtmesin.
    """
    return ox + px * oteleme, oy + py * oteleme


def p1_tamam(g, kd) -> bool:
    """Parkur-1 tamamlama şartı (md 5.5.2.3): geçiş puanı ≥ 5.

    Geçiş puanı (G1/KD1)×10 olduğundan şart **G1/KD1 ≥ 0,5**: karşılıklı
    kenar duba ikililerinin en az yarısı. KD saha verisidir; bilinmiyorsa
    (None/0) "tamamlandı" İDDİA EDİLEMEZ → False.
    """
    if not kd or kd <= 0:
        return False
    return (g / kd) >= 0.5


def p2_gecit_sarti(g, min_gecit: int = 2) -> bool:
    """Parkur-2'nin geçit ayağı (md 5.5.2.4): en az 2 duba ikilisi.

    NOT: tam tamamlama şartı bunun ÜSTÜNE 'son görev noktasına ulaşmak'
    (= son karşılıklı ikiliden geçmek) ister; o kısım görev/FSM katmanının
    işidir, algı yalnız geçit ayağını raporlar.
    """
    return g >= min_gecit

# ── P3 hedef dubası: yayından SÜZÜLECEK büyük cisim ───────────────────────
#: Şartname s.18: hedef dubası Ø640 mm (kenar/engel Ø300 mm'nin 2,13 katı).
HEDEF_CAP_M = 0.64
#: Bir tespit "bizim duba değil, daha BÜYÜK bir cisim" sayılma eşiği.
#: Türetim — 0,30 m varsayarak hesaplanan pinhole menzilin stereo menzile oranı:
#:     gerçek çap 0,30 m → 1,00   ·   0,50 m → 0,60   ·   0,64 m → 0,47
#: Eşik 0,60 ⇒ **≥0,50 m** cisimler süzülür; gerçek kenar dubası (1,00) **2 kat
#: payla** geçer. Tek yönlü: yalnız "olduğundan BÜYÜK" olanı eler, küçüğe dokunmaz.
BUYUK_CISIM_ORANI = 0.60


def buyuk_cisim_mi(z_stereo, w_norm, f_px_norm, cap_m: float = DUBA_CAP_M,
                   esik: float = BUYUK_CISIM_ORANI) -> bool:
    """Bu tespit, kenar/engel dubasından BELİRGİN BÜYÜK bir cisim mi?

    🔴 NEDEN (2026-08-11): P3 hedef dubaları (Ø0,64 m) parkurun SONUNDA duruyor
    ama 64 cm'lik cisim 25 m'den 7,7 px eder — yani tekne daha **Parkur-2'nin
    içindeyken** onları görüyor. Modelimiz 2 sınıflı: kırmızı hedefi
    `kenar_dubasi` sanıyor (ölçüm 11.08: 793/909). O tespit `/perception/buoys`'a
    girerse füzyon → `EdgeBuoyMemory` **kalıcı kenar kaydı** açar (o hafızada
    unutma YOK) → iki hedef arasında **hayalet kapı** kurulur → tekne kapı sanıp
    aralarından geçer. P2'de rotayı bozar, P3'te angajmanı kaçırtır.

    Ölçüt ayarlanabilir bir tahmin değil, FİZİK: bilinen çapla hesaplanan menzil
    ile stereo menzil aynı cisimde örtüşmek zorundadır. Cisim varsayılandan
    büyükse pinhole menzil orantılı olarak KISA çıkar.

    Ölçemediğimiz durumda İDDİA ETMEYİZ → False (kör eleme yok):
    stereo yoksa, genişlik yoksa ya da menzil mono'dan geldiyse çağıran atlar.
    """
    if z_stereo is None or z_stereo <= 0.0:
        return False
    d_pin = mesafe_genislikten(w_norm, cap_m, f_px_norm)
    if d_pin is None or d_pin <= 0.0:
        return False
    return (d_pin / z_stereo) < esik



# ─────────────────────── PARKUR-3 HEDEF RENGİ (FAZ 2) ─────────────────────
#: Hedef dubası renkleri — şartname s.18: RAL 3026 (floresan kırmızı) ·
#: RAL 6037 (saf yeşil) · RAL 9005 (siyah). OpenCV HSV (h 0-179 = derece/2).
#:
#: 🔑 **Renk ADAY üretir, BOYUT karar verir.** Bu eşikler tek başına hedef
#: seçmez: buraya yalnız `buyuk_cisim_mi` süzgecinden geçmiş, yani boyutu
#: 0,30 m'lik dubadan belirgin büyük çıkmış tespitler gelir. Sıralama böyle
#: olduğu için eşikler **gevşek** tutulabiliyor — sıkı eşik gerçek hedefi
#: kaçırırdı (11.08 ölçümü: S>190'da gerçek hedef %41'e düşüyor).
#:
#: ⚠️ Aynı tablo `girdap-ida-p3/p3_hedef/hedef_mantik.py`'de de var (o repo
#: FAZ 2'den önce yazıldı). ÜRETİMDE KULLANILAN BURASIDIR — orası artık
#: referans. Birini değiştiren ötekini de değiştirmeli.
HEDEF_RENK_ESIK = {
    "kirmizi": dict(h=((0, 7), (174, 179)), s=130, v=90),   # RAL 3026 ≈ 3,2°
    "yesil":   dict(h=((62, 85),),          s=80,  v=50),   # RAL 6037 ≈ 143,9°
    "siyah":   dict(h=None,                 s=70,  v=-60),  # RAL 9005, akromatik
}
#: Kutu pikselinin bu oranı rengi tutmalı. %25: hedefin bir kısmı gölgede/
#: parlamada olsa da geçer, ama bbox'ın çoğu su olan bir tespit geçmez.
HEDEF_RENK_ORANI = 0.25

#: Renk → sayısal kod. **Karar tarafıyla AYNI sözleşme**
#: (`prototype/mission/renk_kodu.py KOD_RENK`): 0 = renk çözülemedi.
#: 🔴 Üç repoda yaşıyor (algı · karar · İHA) — biri değişirse hepsi değişmeli;
#: ayrışırsa yanlış hedefe angajman olur ve **hiç belirti vermez** (TS3).
HEDEF_RENK_KODU = {None: 0, "kirmizi": 1, "yesil": 2, "siyah": 3}


def _renk_kapsama(h, s, v, esik) -> float:
    """Piksellerin `esik`e uyan oranı. h/s/v numpy dizisi."""
    import numpy as np
    h = np.asarray(h); s = np.asarray(s); v = np.asarray(v)
    if h.size == 0:
        return 0.0
    if esik["h"] is None:
        # Siyah: ton anlamsız (akromatik) ⇒ parlaklık TAVANI + doygunluk
        # TAVANI. Doygunluk şart: gölgeli SU/zemin de karanlıktır ama rengini
        # KORUR (13.08 İHA ölçümü: gölgeli çimen S=121 ↔ RAL 9005 S=23-32).
        m = (v < -int(esik["v"])) & (s < int(esik["s"]))
        return float(m.mean())
    m = np.zeros(h.shape, bool)
    for lo, hi in esik["h"]:
        m |= (h >= lo) & (h <= hi)
    return float((m & (s > esik["s"]) & (v > esik["v"])).mean())


def hedef_rengi_bgr(roi_bgr, oran: float = HEDEF_RENK_ORANI):
    """Hedef bbox'ının BGR kırpımından renk çöz — **üretimde çağrılan budur**.

    🔴 **Gürültü bastırma ZORUNLU (13.08 ölçümü).** Doygunluk S=(max−min)/max
    olduğu için **düşük parlaklıkta kararsızdır**: RAL 9005 (gerçek S≈20) σ=8
    gürültüde **S≈89** okunuyor ⇒ doygunluk tavanı siyahı **eliyordu**
    (kapsama %23, eşik %25 — sınırın hemen altında, yani sessiz ve rastgele).
    3×3 Gauss sonrası: siyah kapsama **%71-82**, gölgeli su **%3-6**, gölgeli
    çim **%1-2** ⇒ temiz ayrım. (Aynı olgu İHA plaka tarafında da bulundu.)

    ⚠️ **Bilinen sınır:** siyah, herhangi bir KOYU AKROMATİK yüzeyden ayırt
    edilemez (ölçüldü: koyu asfalt %92 geçiyor) — fizik böyle. Bizi koruyan
    şey üstteki **boyut kapısı**: buraya yalnız suda yüzen, 0,50-0,91 m çapında
    bir cisim gelir. Renk tek başına asla hedef seçmez.
    """
    import cv2
    import numpy as np
    roi = np.asarray(roi_bgr)
    if roi.size == 0 or roi.ndim != 3:
        return None, 0.0
    if min(roi.shape[:2]) >= 3:
        roi = cv2.GaussianBlur(roi, (3, 3), 0)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    return hedef_rengi(hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2], oran)


def hedef_rengi(h, s, v, oran: float = HEDEF_RENK_ORANI):
    """Hedef dubasının rengi: ("kirmizi"|"yesil"|"siyah"|None, kapsama).

    `None` = **renk çözülemedi**; hata değildir (gölge, parlama, kırpık bbox).
    Çağıran yine de yayınlar — konum bilgisi tek başına değerlidir; rengi
    bilinmeyen hedef `HEDEF_RENK_KODU[None] = 0` ile gider.

    Birden fazla renk eşiği tutarsa **en yüksek kapsama** kazanır; siyah en
    son bakılır çünkü gölgeli kırmızı/yeşil de karanlıktır — renkli bir
    eşleşme varsa o önceliklidir (yanlış hedefe angajman TS3'ü artırır:
    şartname s.25, 1 yanlış temas 100→50, 2 yanlış 100→**5**).
    """
    skor = {ad: _renk_kapsama(h, s, v, e) for ad, e in HEDEF_RENK_ESIK.items()}
    renkli = {a: k for a, k in skor.items() if a != "siyah" and k > oran}
    if renkli:
        en = max(renkli, key=renkli.get)
        return en, renkli[en]
    if skor["siyah"] > oran:
        return "siyah", skor["siyah"]
    return None, 0.0

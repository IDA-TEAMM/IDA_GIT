# -*- coding: utf-8 -*-
"""GEÇİŞ SAYILMADI — SEBEBİ ÖLÇÜLÜYOR MU? (19.08.2026)

🔴 DOĞURAN SORU (Eyüp): *"gate'den geçmiyor — REALDE neden geçmediğini
öğrenmek istiyoruz."*

Eski kod geçişi reddettiğinde şunu basıyordu:
    "Geçiş zaman aşımı — odometri geçişi DOĞRULAMADI, sayılmadı.
     (MPPI takılmış olabilir: obstacle_margin / engel haritasına bak)"
Bu bir **tahmindi**. Oysa `gecitten_gecti` iki BAĞIMSIZ şart arar ve
hangisinin düştüğü ölçülebilir:

    ① ileri > ek_yol       → düzlemi aştı mı (araç kapıya VARDI mı)
    ② |yanal| ≤ yarı_gen   → dubaların ARASINDAN mı geçti

İkisi bambaşka arızadır:
    DUZLEMI_ASMADI → kontrol/planlama; araç kapıya hiç varamıyor
    YANDAN_DOLASTI → nişan noktası kayması; varıyor ama yandan

**Sanal gölde ölçüldü (şartname geometrisi, 9-11,4 m kapı):**
    SEBEP: DUZLEMI_ASMADI · en ileri −5,64 m (gerekli > 1,53)
                          · en yakın yanal 0,00 m (gerekli ≤ 3,60)
Yani nişan MÜKEMMEL hizalı, araç kapının 5,6 m GERİSİNDE kalıyor.
"Yandan dolaşıyor" hipotezi bu ölçümle çürüdü.

⚠ Bu dosya davranışı DÜZELTMEZ — sebebi ÖLÇÜLEBİLİR kılar. Aynı log
sahada da basılacak; "realde neden geçmedi" sorusu orada da veriyle
cevaplanacak.
"""
from __future__ import annotations

import math

import pytest

from girdap_ida_algi import gecit_mantik as gm

EK = 1.53          # PASS_EK_YOL ~ ARAC_BOY + 0.5
YARI = 4.0


def test_bilesenler_ISARETLI_ileri_MUTLAK_yanal():
    """İleri işaretli (geride = negatif), yanal mutlak."""
    # kapı orijinde, normal +x, teğet +y
    ileri, yanal = gm.gecis_bilesenleri(-5.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
    assert ileri == pytest.approx(-5.0)
    assert yanal == pytest.approx(2.0)
    ileri2, yanal2 = gm.gecis_bilesenleri(3.0, -2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
    assert ileri2 == pytest.approx(3.0)
    assert yanal2 == pytest.approx(2.0), "yanal mutlak değer olmalı"


@pytest.mark.parametrize("ileri,yanal,beklenen", [
    (5.0, 1.0, "GECTI"),
    (-5.64, 0.0, "DUZLEMI_ASMADI"),      # 🔑 gölde ÖLÇÜLEN hâl
    (0.5, 0.0, "DUZLEMI_ASMADI"),        # eşiğin altında
    (5.0, 9.0, "YANDAN_DOLASTI"),
    (-3.0, 9.0, "IKISI_DE"),
])
def test_red_sebebi_DOGRU_adlandiriliyor(ileri, yanal, beklenen):
    assert gm.gecis_red_sebebi(ileri, yanal, YARI, EK) == beklenen


def test_genislik_BILINMEYENDE_yanal_sart_ARANMAZ():
    """Kapı FOV'dan kaybolup tek bearing'den kurulduysa genişlik yok —
    bilgi yokken kör reddetme yapılmaz (`gecitten_gecti` ile aynı kural)."""
    assert gm.gecis_red_sebebi(5.0, 99.0, None, EK) == "GECTI"
    assert gm.gecis_red_sebebi(-1.0, 99.0, None, EK) == "DUZLEMI_ASMADI"


def test_teshis_KARAR_yolunu_degistirmiyor():
    """🔑 Teşhis, `gecitten_gecti`'nin verdiği kararla BİREBİR tutarlı olmalı.

    Ayrışırsa log bir şey derken sayaç başka şey yapar — teşhis aracının
    kendisi yanlış yönlendirir.
    """
    for px in (-6.0, -1.0, 0.0, 1.6, 5.0):
        for py in (0.0, 3.9, 4.1, 9.0):
            gecti = gm.gecitten_gecti(px, py, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                                      YARI, EK)
            ileri, yanal = gm.gecis_bilesenleri(px, py, 0.0, 0.0,
                                                1.0, 0.0, 0.0, 1.0)
            sebep = gm.gecis_red_sebebi(ileri, yanal, YARI, EK)
            assert gecti == (sebep == "GECTI"), (
                f"({px},{py}): gecitten_gecti={gecti} ama teşhis={sebep}")


def test_dugum_TAHMIN_degil_OLCUM_basiyor():
    """Eski 'MPPI takılmış olabilir' tahmini geri gelmemeli."""
    import ast
    import inspect
    import textwrap

    from girdap_ida_algi import duba_gecis_navigator as nav
    agac = ast.parse(textwrap.dedent(inspect.getsource(nav.DubaNavigator)))
    src = ast.unparse(agac)                      # yorumlar DÜŞER
    assert "gecis_red_sebebi" in src, "düğüm red sebebini ölçmüyor"
    assert "_gecis_en_ileri" in src and "_gecis_en_yanal" in src, (
        "geçiş bileşenleri izlenmiyor — sebep ölçülemez")
    # ⚠ Ham metin araması YETMEZ: eski mesajı ANLATAN yorum satırında da
    # geçiyor ve test boşuna kırmızı yanıyordu (bu oturumda 4. kez aynı
    # sınıf). Yalnız GERÇEK dize sabitleri taranır.
    dizeler = [n.value for n in ast.walk(agac)
               if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert not any("MPPI takılmış" in d for d in dizeler), (
        "ölçüm yerine tahmin yürüten eski mesaj geri gelmiş")


def test_pencere_CRUISE_HIZ_varsayimi_1_0():
    """🔴 SAHADA GEÇERLİ BULGU: pencere `CRUISE_HIZ` ile boyutlanıyor.

    `gecis_baslat`: pencere = max(3·(orta_z + PASS_EK_YOL)/CRUISE_HIZ, 8).
    `CRUISE_HIZ = 1,0 m/s`, ama saha bandında ölçülen seyir **0,62 m/s**
    ⇒ aynı mesafe gerçekte **1,61×** sürüyor, pencere o kadar iyimser.
    3× pay bunu şimdilik örtüyor (0,62'de yeterli çıkıyor), ama pay
    daraltılırsa ya da hız düşerse geçişler sessizce sayılmaz olur.

    Bu test sabiti DONDURMUYOR — değiştiğinde görünür olmasını sağlıyor.
    """
    from girdap_ida_algi import duba_gecis_navigator as nav
    assert nav.CRUISE_HIZ == 1.0, (
        f"CRUISE_HIZ {nav.CRUISE_HIZ} — pencere hesabı ve bu notun "
        "dayanağı birlikte gözden geçirilmeli")
    assert nav.GECIS_ZAMAN_KATSAYI >= 3.0, (
        "pay 3×'in altına indi — 0,62 m/s'de geçişler sayılmayabilir")
    # 0,62 m/s'de en kötü hâl (kapı FOV'dan 8,73 m'de kaybolur) yeterli mi?
    orta_z = 8.73
    pencere = max(nav.GECIS_ZAMAN_KATSAYI * (orta_z + 1.53) / nav.CRUISE_HIZ, 8.0)
    assert (orta_z + 1.53) / 0.62 <= pencere, (
        "saha hızında (0,62 m/s) geçiş penceresi YETMİYOR")

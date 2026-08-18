"""`planning_node` yürütücü grupları — AÇLIK nöbetçisi.

🔴 Bu dosyanın var olma sebebi 18.08.2026, Gazebo koşumu:

FAZ 5 (§1.17a) ağır ALGI geri çağrılarını varsayılan gruptan çıkarmıştı ama
POZ ALIMINI (`/girdap/fusion/odom`) kontrol zamanlayıcısının arkasında
bıraktı. Varsayılan grup `MutuallyExclusive`dir → MPPI koşarken `_on_odom`
**sevk edilmez** (kilidi beklemez, sıraya bile alınmaz). Ölçülen zincir:

    poz yaşı 1,1-1,2 s → `POZ-BAYAT` → MPPI durur → thrust sıfır →
    cmd_vel akışı kesilir → ArduPilot son DÖNÜŞ komutunu tutar →
    tekne yerinde döner ve çıkamaz

Kontrol kilidi dağılımı bunu birebir gösterdi: 101 × `YOK|PIVOT` ↔
101 × `POZ-BAYAT|PIVOT`. Yani düğüm kendi bekçisini AÇLIKTAN tetikliyordu —
§1.11'de canlı yığında ölçülen davranışın aynısı.

Buradaki testler **kaynak metni** okur (ROS başlatmadan): niyet, grup
atamalarının yerinde kalması. Davranış testi değil, REGRESYON kilidi.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

_KAYNAK_YOLU = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision" / "girdap_decision"
    / "planning_node.py"
)
_KAYNAK = _KAYNAK_YOLU.read_text(encoding="utf-8")
_AGAC = ast.parse(_KAYNAK)


def _abonelik_gruplari() -> dict[str, str | None]:
    """{konu: callback_group ismi} — `create_subscription` çağrılarından."""
    out: dict[str, str | None] = {}
    for dugum in ast.walk(_AGAC):
        if not isinstance(dugum, ast.Call):
            continue
        if getattr(dugum.func, "attr", None) != "create_subscription":
            continue
        konu = next(
            (a.value for a in dugum.args
             if isinstance(a, ast.Constant) and isinstance(a.value, str)),
            None,
        )
        if konu is None:
            continue
        grup = None
        for kw in dugum.keywords:
            if kw.arg == "callback_group":
                grup = getattr(kw.value, "attr", None) or getattr(
                    kw.value, "id", None)
        out[konu] = grup
    return out


# ───────────────────────────────── asıl kilit: poz açlıktan ölmesin
def test_poz_aboneligi_KONTROL_GRUBUNDA_DEGIL() -> None:
    """`/girdap/fusion/odom` varsayılan gruba düşerse açlık geri gelir.

    Varsayılan grupta kontrol zamanlayıcısı (MPPI) var; ikisi aynı
    MutuallyExclusive grupta olursa poz, adım süresi kadar DEĞİL, kuyruk
    dolana kadar yaşlanır.
    """
    gruplar = _abonelik_gruplari()
    assert "/girdap/fusion/odom" in gruplar, "poz aboneliği kaybolmuş"
    assert gruplar["/girdap/fusion/odom"] == "_grup_durum", (
        "poz aboneliği `_grup_durum` dışına çıkmış — kontrol adımının "
        "arkasında sevk sırası bekler ve POZ-BAYAT açlıktan tetiklenir "
        "(18.08 Gazebo ölçümü: yaş 1,1 s, tekne sonsuza kadar döndü)"
    )


@pytest.mark.parametrize("konu", [
    "/girdap/mission/state",
    "/mavros/state",
    "/girdap/mission/current_target",
])
def test_durum_girdileri_AYRI_grupta(konu: str) -> None:
    """Mod/görev durumu da kontrolün arkasında beklememeli.

    `mavros/state` geç gelirse köprü arm/mod geçişini geç görür; görev
    durumu geç gelirse auto-GUIDED geçidi yanlış zamanda açılır.

    🔴 18.08 — `current_target` EKLENDİ. Adı "video bypass hedefi" gibi
    görünüyor ama `_on_target` RRT* kolunda da koşup `_pivot_yedek_hedef`i
    (F-F.23) yazar; yani durum girdisidir. Varsayılan grupta bırakılmıştı ve
    orada kontrol adımı (MPPI, ~144 ms > 100 ms bütçe) var — abonelik
    kuyrukta yaşlanıyordu. Etkisi bugün gizli (`pivot_yedek_referans`
    varsayılanı false ⇒ yedek hedefi kimse okumuyor), tuzak o şalter
    açılınca kurulurdu. Bu test onu açmadan önce dondurur.
    """
    assert _abonelik_gruplari().get(konu) == "_grup_durum", (
        f"{konu} varsayılan gruba düşmüş — kontrol adımı boyunca sevk edilmez"
    )


def test_agir_algi_KENDI_grubunda_KALIYOR() -> None:
    """FAZ 5'in kazanımı geri alınmasın (kadans 10 → 1,9 Hz idi)."""
    gruplar = _abonelik_gruplari()
    for konu in ("/perception/obstacle_map", "/perception/classified_obstacles"):
        assert gruplar.get(konu) == "_grup_algi", (
            f"{konu} algı grubundan çıkmış — FAZ 5 regresyonu"
        )


def test_yurutucu_GRUP_SAYISI_KADAR_is_parcacigi() -> None:
    """Dört MutuallyExclusive grup varsa dört iş parçacığı gerekir.

    İki iş parçacığıyla üçüncü grup, diğer ikisinden biri bitene kadar sevk
    sırası bulamaz — düzeltmenin yarısı boşa gider.
    """
    for dugum in ast.walk(_AGAC):
        if isinstance(dugum, ast.Call) and getattr(
                dugum.func, "id", None) == "MultiThreadedExecutor":
            sayi = next((kw.value.value for kw in dugum.keywords
                         if kw.arg == "num_threads"), None)
            assert sayi is not None and sayi >= 4, (
                f"num_threads={sayi} — dört geri çağrı grubu için yetersiz"
            )
            return
    pytest.fail("MultiThreadedExecutor çağrısı bulunamadı")


def test_paylasilan_durum_KILITLI_kaliyor() -> None:
    """Grupları ayırmak ancak kilit dururken güvenli.

    `_on_odom` ile `_on_control_step` artık FARKLI iş parçacıklarında
    koşabilir; ikisi de `_pipe`'a dokunuyor. `_pipe_kilidiyle` biri
    üzerinden kalkarsa veri yarışı doğar — ve bu tür yarış sahada
    yakalanamaz.
    """
    for ad in ("_on_odom", "_on_control_step"):
        for dugum in ast.walk(_AGAC):
            if isinstance(dugum, ast.FunctionDef) and dugum.name == ad:
                sus = {getattr(d, "id", None) or getattr(d, "attr", None)
                       for d in dugum.decorator_list}
                assert "_pipe_kilidiyle" in sus, (
                    f"{ad} `_pipe_kilidiyle` sarmalayıcısını kaybetmiş — "
                    "gruplar ayrıldığı için artık gerçek veri yarışı olur"
                )
                break
        else:
            pytest.fail(f"{ad} bulunamadı")


# ───────────────────────── 🛟 kadans bekçisi (18.08)
def test_kadans_bekcisi_AYRI_grupta_ve_KILITSIZ() -> None:
    """Bekçi `_pipe` kilidini alırsa tam koruması gereken anda susar.

    Kontrol adımı (MPPI) kilidi 625 ms tutuyordu; bekçi de aynı kilidi
    isteseydi onun arkasında bekler ve cmd_vel boşluğunu HİÇ dolduramazdı.
    """
    assert "_grup_bekci" in _KAYNAK, "kadans bekçisi grubu kaldırılmış"
    for dugum in ast.walk(_AGAC):
        if isinstance(dugum, ast.FunctionDef) and dugum.name == "_setpoint_bekcisi":
            sus = {getattr(d, "id", None) or getattr(d, "attr", None)
                   for d in dugum.decorator_list}
            assert "_pipe_kilidiyle" not in sus, (
                "bekçiye `_pipe_kilidiyle` eklenmiş — MPPI kilidi tutarken "
                "bloke olur, yani kesintiyi tam da olduğu anda kaçırır"
            )
            return
    pytest.fail("_setpoint_bekcisi bulunamadı")


def test_bekci_ACIK_SIFIR_basiyor() -> None:
    """Sessiz kalmak ile sıfır basmak ArduPilot için AYNI ŞEY DEĞİL.

    18.08 ölçümü: setpoint akışı 178 sn kesildi, ArduPilot son DÖNÜŞ
    komutunu tuttu ve tekne durmadı — yani "yayınlamamak" bir duruş değil.
    """
    for dugum in ast.walk(_AGAC):
        if isinstance(dugum, ast.FunctionDef) and dugum.name == "_setpoint_bekcisi":
            metin = ast.unparse(dugum)
            assert "_pub_cmd_vel.publish" in metin, (
                "bekçi artık komut yayınlamıyor — yalnız uyarmak işe yaramaz"
            )
            return
    pytest.fail("_setpoint_bekcisi bulunamadı")


def test_kasitli_sessizlikte_bekci_SUSUYOR() -> None:
    """Geçit kapalıyken (disarm / GUIDED dışı) sessizlik normaldir.

    `_son_cmd_vel_t` None'a çekilmezse bekçi her turda sıfır basar ve
    gerçek kesintiyi gürültüye boğar.
    """
    assert "self._son_cmd_vel_t = None" in _KAYNAK, (
        "geçit kapanınca `_son_cmd_vel_t` sıfırlanmıyor — bekçi yanlış tetiklenir"
    )


def test_poz_kuyruk_derinligi_VARSAYILAN_ESKI_DAVRANIS() -> None:
    """Poz aboneliğinin kuyruk derinliği ölçülmeden değiştirilmesin.

    `_grup_durum` AÇLIĞI bitirdi (geri çağrı artık kontrol adımının arkasında
    sıraya bile alınmadan bekletilmiyor). BİRİKİM ayrı bir kusurdur: 10 Hz'de
    `depth=10` bir saniyelik kuyruk demek, yani bir gecikmeden sonra geri
    çağrı güncel pozu görmeden önce 10 eski pozu işler. ROS 2 QoS deseni
    "yalnız en sonu isteyen tüketici → Keep Last = 1".

    Bu test şalterin VARSAYILANINI dondurur: 1'e çekmek bant koşumunda
    ölçülmüş bir kazanç göstermeden yapılamaz.
    """
    m = re.search(r'declare_parameter\("odom_qos_depth",\s*([0-9]+)\)', _KAYNAK)
    assert m is not None, "odom_qos_depth şalteri kaybolmuş"
    assert m.group(1) == "10", (
        "odom_qos_depth varsayılanı değişmiş — eski davranış 10'du; "
        "değiştirmeden önce bant koşumuyla ölç ve gerekçesini yaz"
    )
    assert "self._odom_qos_depth," in _KAYNAK, (
        "abonelik şalteri KULLANMIYOR — sabit derinliğe geri dönmüş"
    )

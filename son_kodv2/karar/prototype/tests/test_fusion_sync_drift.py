"""
Girdap İDA — kamera↔LiDAR sync kuyruğunun config-drift kapıları (ROS'SUZ).

🔴 NEDEN BU DOSYA VAR (2026-07-09 tezgahı → 2026-08-09 algı ekibi raporu)

Gerçek Livox + OAK ile koşulan tezgah ölçümü: kapalı alanda ~20k yoğun nokta
→ clustering 1-3,3 s/kare → `/perception/obstacle_map` damgası DOĞRU ama GEÇ
varıyor. `ApproximateTimeSynchronizer` aynı damgalı kamera karesini o gecikme
boyunca kuyrukta tutamazsa kare düşer ve **eşleşme HİÇ oluşmaz**:
`/perception/classified_obstacles` üretilmez → `planning_node._edge_buoys`
boş kalır → kapı takibi ham GPS noktasına düşer → P1 (G1/KD1) ve P2 (≥2 duba
ikilisi) şartları sağlanamaz. **Belirtisi yok** (node sağ, log temiz).

Ölçülen: queue=10 hiç tutmadı · queue=50 KIL PAYI yetmedi · queue=100 tuttu.
Damgalar aynı tabandaydı (fark ~27 ms) → slop=0,1 s zaten yeterli. Yani bu bir
**gecikme** sorunu, saat/slop sorunu DEĞİL.

Düzeltme 09.07'de yapıldı ama 14.07'deki klasör taşınmasında (git'lenmemiş zip
çalışma kopyası silindi, yeni depo başka bir anlık görüntüden kuruldu) düştü;
`queue_size=10` literal'i geri geldi ve 09.08'de algı ekibi raporlayana kadar
bir ay boyunca fark edilmedi. Bu testler o düşüşün tekrarını imkânsız kılar.

`rclpy`/`launch_ros` GEREKTİRMEZ (kaynak `ast` ile okunur) → CI'ın ROS'suz
çekirdek job'ında da koşar. F16.2 dersi: gerçekten koşabilen test.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml

_PKG_DIR = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
)
_NODE_FILE = _PKG_DIR / "girdap_decision" / "perception_fusion_node.py"
_LAUNCH_FILE = _PKG_DIR / "launch" / "hardware.launch.py"
_PARAMS_FILE = _PKG_DIR / "config" / "params.yaml"
_HARDWARE_FILE = _PKG_DIR / "config" / "hardware.yaml"

# Tezgahta yetmediği ÖLÇÜLEN en büyük değer. Alt sınır bundan türer.
_OLCULEN_YETMEYEN = 50


def _node_agaci() -> ast.Module:
    return ast.parse(_NODE_FILE.read_text(encoding="utf-8"))


def _declare_varsayilanlari() -> dict:
    """Node'daki `self.declare_parameter("ad", <literal>)` çağrılarını topla."""
    bulunan: dict = {}
    for dugum in ast.walk(_node_agaci()):
        if not isinstance(dugum, ast.Call):
            continue
        fn = dugum.func
        if not (isinstance(fn, ast.Attribute) and fn.attr == "declare_parameter"):
            continue
        if len(dugum.args) != 2:
            continue
        try:
            ad = ast.literal_eval(dugum.args[0])
            bulunan[ad] = ast.literal_eval(dugum.args[1])
        except ValueError:                  # literal olmayan varsayılan — atla
            continue
    return bulunan


def _sync_cagrisi() -> ast.Call:
    """`ApproximateTimeSynchronizer(...)` çağrısını bul."""
    for dugum in ast.walk(_node_agaci()):
        if isinstance(dugum, ast.Call):
            fn = dugum.func
            ad = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", "")
            if ad == "ApproximateTimeSynchronizer":
                return dugum
    raise AssertionError("ApproximateTimeSynchronizer çağrısı bulunamadı")


def _launch_sabiti(ad: str) -> dict:
    """Launch dosyasındaki modül düzeyi `<ad> = {...}` sabitini ast ile oku."""
    tipler = {"float": float, "int": int, "str": str, "bool": bool}

    def coz(dugum: ast.AST):
        if isinstance(dugum, ast.Name) and dugum.id in tipler:
            return tipler[dugum.id]
        if isinstance(dugum, ast.Tuple):
            return tuple(coz(e) for e in dugum.elts)
        if isinstance(dugum, ast.Dict):
            return {coz(k): coz(v) for k, v in zip(dugum.keys, dugum.values)}
        return ast.literal_eval(dugum)

    agac = ast.parse(_LAUNCH_FILE.read_text(encoding="utf-8"))
    for dugum in agac.body:
        hedefler = (
            dugum.targets if isinstance(dugum, ast.Assign)
            else [dugum.target] if isinstance(dugum, ast.AnnAssign)
            else []
        )
        for hedef in hedefler:
            if isinstance(hedef, ast.Name) and hedef.id == ad:
                return coz(dugum.value)
    raise AssertionError(f"{ad} launch dosyasında bulunamadı")


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


# --------------------------------------------------------------- asıl nöbetçi

def test_sync_queue_size_LITERAL_OLARAK_GOMULU_DEGIL() -> None:
    """`queue_size=` parametreden gelmeli — literal yazılırsa saha ayarı ÖLÜR.

    Kaybedilen düzeltmenin tam belirtisi buydu: `queue_size=10` literal'i,
    `slop` config'den gelirken kuyruğun parametre bile OLMAMASI. O hâlde
    sahada launch-arg ile düzeltmek imkânsızdır; yeniden derleme gerekir.
    """
    cagri = _sync_cagrisi()
    kw = {k.arg: k.value for k in cagri.keywords}
    assert "queue_size" in kw, "ApproximateTimeSynchronizer'a queue_size verilmiyor"
    kaynak = ast.dump(kw["queue_size"])
    assert not isinstance(kw["queue_size"], ast.Constant), (
        "queue_size LİTERAL yazılmış — 2026-07-09'da ölçülüp kaybedilen "
        "düzeltmenin aynısı geri geldi. Değeri `sync_queue_size` "
        "parametresinden oku (dosya docstring'indeki ölçüme bak)."
    )
    assert "sync_queue_size" in kaynak, (
        "queue_size bir şeyden geliyor ama `sync_queue_size` parametresinden "
        f"değil: {kaynak}"
    )


def test_sync_queue_size_dort_yerde_de_AYNI() -> None:
    """Node varsayılanı ↔ launch ↔ params.yaml ↔ hardware.yaml birebir.

    Dördü ayrışırsa `ros2 run` ile `ros2 launch` FARKLI kuyrukla koşar ve fark
    sahada görünmez (arıza sessiz: yalnız classified_obstacles hiç gelmez).
    """
    node_varsayilan = _declare_varsayilanlari()["sync_queue_size"]
    launch_deger, launch_tip = _launch_sabiti("_FUSION_DEFAULTS")["sync_queue_size"]
    params = _yaml(_PARAMS_FILE)["perception_fusion_node"]["ros__parameters"]
    hardware = _yaml(_HARDWARE_FILE)["perception"]["fusion"]

    assert launch_tip is int
    assert launch_deger == node_varsayilan, (
        f"launch={launch_deger} ≠ node varsayılanı={node_varsayilan}"
    )
    assert params["sync_queue_size"] == node_varsayilan, (
        f"params.yaml={params['sync_queue_size']} ≠ node={node_varsayilan}"
    )
    assert hardware["sync_queue_size"] == node_varsayilan, (
        f"hardware.yaml={hardware['sync_queue_size']} ≠ node={node_varsayilan}"
    )


def test_sync_queue_size_olculen_yetmeyen_degerin_USTUNDE() -> None:
    """Kuyruk, tezgahta yetmediği ÖLÇÜLEN 50'nin üstünde olmalı.

    ⚠ Bu test kırmızıya dönerse çözüm sayıyı test dosyasında küçültmek DEĞİL:
    kuyruk gereksinimi `kamera Hz × clustering gecikmesi`nden türer. Açık suda
    (seyrek nokta → clustering ms'ler) gerçekten daha küçüğü yetebilir — ama o
    zaman ÖNCE ölçümü yap, ölçüm değerini buraya gerekçesiyle yaz.
    """
    node_varsayilan = _declare_varsayilanlari()["sync_queue_size"]
    assert node_varsayilan > _OLCULEN_YETMEYEN, (
        f"sync_queue_size={node_varsayilan}; tezgahta {_OLCULEN_YETMEYEN} "
        "KIL PAYI yetmemiş, 100 tutmuştu (docstring)"
    )


@pytest.mark.parametrize("dosya,blok_yolu", [
    ("params", ("perception_fusion_node", "ros__parameters")),
    ("hardware", ("perception", "fusion")),
])
def test_yaml_fusion_anahtarlari_launch_ile_ayni(dosya: str, blok_yolu: tuple) -> None:
    """yaml'daki fusion anahtar KÜMESİ `_FUSION_DEFAULTS` ile birebir olmalı.

    ROS bilinmeyen yaml anahtarını SESSİZCE atar → `sync_queue_sizee: 100`
    yazımı hiçbir uyarı üretmeden yok sayılır ("düzelttim ama değişmedi").
    """
    kok = _yaml(_PARAMS_FILE if dosya == "params" else _HARDWARE_FILE)
    for k in blok_yolu:
        kok = kok[k]
    assert set(kok) == set(_launch_sabiti("_FUSION_DEFAULTS"))

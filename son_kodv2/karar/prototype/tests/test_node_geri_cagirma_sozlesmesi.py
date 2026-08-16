"""Node kurucularında ADI GEÇEN her geri çağırma GERÇEKTEN var mı? (AST)

🔴 NEDEN VAR — 2026-08-16 akşamı yaşandı, tahmin değil:

`acc6247` birleştirmesi `planning_node.py`'ye `/perception/targets`
aboneliğini geri getirdi ama **`def _on_targets` gövdesini getirmedi**.
Sonuç zinciri:

    PlanningNode.__init__ → create_subscription(..., self._on_targets, ...)
      → AttributeError: 'PlanningNode' object has no attribute '_on_targets'
      → launch açılışta ölür → `girdap-karar` servisi 3 kez deneyip vazgeçer
      → MPPI yok, thrust yok ⇒ **P1 + P2 + P3 = 0**

Neden kimse görmedi:
  · `git log -S 'def _on_targets'` **birleştirme commit'lerini diff'lemez**
    (`-m`/`--first-parent` verilmedikçe) ⇒ arama kaybı sessiz.
  · Node testleri rclpy ister; rclpy'siz ortamda `importorskip` ile ATLANIR
    ⇒ CI yeşil kalır.
  · Hata yalnız node KURULURKEN çıkar; birim testler çekirdek modülleri
    (`prototype/`) çağırdığı için orada hiç görünmez.

Bu test o üç boşluğu da kapatır: **rclpy GEREKTİRMEZ**, kaynağı `ast` ile
okur, birleştirme sonrası dosyanın kendisine bakar.

⛔ GERİ ALINIRSA: aynı sınıf hata (abonelik var, gövde yok / adı değişmiş)
bir daha yalnız SAHADA, servis açılmayınca fark edilir.

Kapsam: `create_subscription`, `create_publisher` hariç tutulur (yayıncı
geri çağırma almaz); `create_timer`, `create_service`, `create_client` ve
`message_filters` `registerCallback` da denetlenir.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

_PAKET = (
    pathlib.Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision" / "girdap_decision"
)

#: Geri çağırma argümanı taşıyan rclpy kurucuları ve argümanın SIRASI.
#: (create_publisher yok — yayıncı geri çağırma almaz.)
_KURUCULAR = {
    "create_subscription": 2,      # (msg_type, topic, CALLBACK, qos)
    "create_timer": 1,             # (period, CALLBACK)
    "create_service": 2,           # (srv_type, name, CALLBACK)
    "registerCallback": 0,         # message_filters: (CALLBACK)
}


def _node_dosyalari() -> list[pathlib.Path]:
    return sorted(p for p in _PAKET.glob("*_node.py"))


def _sinif_metotlari(agac: ast.Module) -> set[str]:
    """Dosyadaki TÜM sınıfların metot adları (kalıtım yok — hepsi düz Node)."""
    adlar: set[str] = set()
    for dugum in ast.walk(agac):
        if isinstance(dugum, (ast.FunctionDef, ast.AsyncFunctionDef)):
            adlar.add(dugum.name)
    return adlar


def _atanan_alanlar(agac: ast.Module) -> set[str]:
    """`self._x = ...` ile atanan alanlar — geri çağırma bir alan da olabilir."""
    adlar: set[str] = set()
    for dugum in ast.walk(agac):
        hedefler = []
        if isinstance(dugum, ast.Assign):
            hedefler = dugum.targets
        elif isinstance(dugum, ast.AnnAssign):
            hedefler = [dugum.target]
        for h in hedefler:
            if (isinstance(h, ast.Attribute)
                    and isinstance(h.value, ast.Name) and h.value.id == "self"):
                adlar.add(h.attr)
    return adlar


def _beklenen_geri_cagirmalar(agac: ast.Module) -> list[tuple[str, str, int]]:
    """(kurucu, geri_çağırma_adı, satır) — yalnız `self.<ad>` biçiminde olanlar."""
    bulunan: list[tuple[str, str, int]] = []
    for dugum in ast.walk(agac):
        if not isinstance(dugum, ast.Call):
            continue
        fn = dugum.func
        ad = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", None)
        if ad not in _KURUCULAR:
            continue
        idx = _KURUCULAR[ad]
        if len(dugum.args) <= idx:
            continue                       # anahtar-kelimeli çağrı → atla
        arg = dugum.args[idx]
        # Yalnız `self.<ad>` denetlenir: lambda / kısmi uygulama / dışarıdan
        # gelen fonksiyon bu testin konusu değil (orada AttributeError olmaz).
        if (isinstance(arg, ast.Attribute)
                and isinstance(arg.value, ast.Name) and arg.value.id == "self"):
            bulunan.append((ad, arg.attr, arg.lineno))
    return bulunan


@pytest.mark.parametrize("yol", _node_dosyalari(), ids=lambda p: p.name)
def test_adi_gecen_geri_cagirma_TANIMLI(yol: pathlib.Path) -> None:
    """Kurucuya verilen her `self._x` ya metot ya atanan alan olmalı."""
    agac = ast.parse(yol.read_text(encoding="utf-8"), filename=str(yol))
    tanimli = _sinif_metotlari(agac) | _atanan_alanlar(agac)

    eksikler = [
        f"{yol.name}:{satir} → {kurucu}(..., self.{cb}, ...) ama "
        f"`def {cb}` YOK (ve `self.{cb} = ...` ataması da yok)"
        for kurucu, cb, satir in _beklenen_geri_cagirmalar(agac)
        if cb not in tanimli
    ]
    assert not eksikler, (
        "Node kurucusunda ADI GEÇEN ama TANIMSIZ geri çağırma — bu node "
        "AÇILIRKEN AttributeError ile ölür, servis boot'ta kalkmaz:\n  "
        + "\n  ".join(eksikler)
    )


def test_denetim_GERCEKTEN_dosya_buluyor() -> None:
    """Kontrol grubu: tarama boş küme üzerinde 'yeşil' vermesin.

    Bu dosyanın kendisi de bir tuzağa düşebilirdi — `_PAKET` yolu kayarsa
    `glob` boş döner, parametrize sıfır test üretir ve süit "geçti" der.
    (Memory: *"bir alarm her zaman yanıyorsa alarm değildir"*ın tersi —
    hiç ateşlenemeyen alarm.)
    """
    dosyalar = _node_dosyalari()
    assert len(dosyalar) >= 8, f"node dosyaları bulunamadı: {_PAKET}"
    assert any(p.name == "planning_node.py" for p in dosyalar)


def test_planning_node_ON_TARGETS_vakasi_yakalanirdi() -> None:
    """Mutasyon: gövdeyi silersek test KIRMIZIYA dönmeli (16.08 vakası birebir)."""
    yol = _PAKET / "planning_node.py"
    kaynak = yol.read_text(encoding="utf-8")
    assert "self._on_targets" in kaynak, "abonelik kaybolmuş — vaka geçersiz"

    agac = ast.parse(kaynak)
    tanimli = _sinif_metotlari(agac) | _atanan_alanlar(agac)
    assert "_on_targets" in tanimli, "gerçek dosyada gövde YOK (canlı hata!)"

    # Gövdeyi AST'ten düşür → denetim eksik bildirmeli.
    mutant = {ad for ad in tanimli if ad != "_on_targets"}
    eksikler = [
        cb for _, cb, _ in _beklenen_geri_cagirmalar(agac) if cb not in mutant
    ]
    assert "_on_targets" in eksikler, (
        "gövde silindiği hâlde denetim sessiz kaldı — test kendi kendini "
        "kanıtlamıyor"
    )

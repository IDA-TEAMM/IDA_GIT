# -*- coding: utf-8 -*-
"""MPPI İÇ API'LERİ — testler HESAP YOLUNDA mı koşuyor? (18.08.2026)

🔴 BUNU DOĞURAN KUSUR (Yahya, `IDA_GIT@06142a29`):
`test_ileri_tercihi.py`'deki `test_GERI_GIDEN_yorunge_elenir_FREN_ELENMEZ`
`_trajectory_cost`'u **ham numpy** dizisiyle çağırıyordu. Etki makineye göre
değişiyordu — sessiz olan da buydu:

    laptop : CuPy YOK  → `auto` → `xp` = numpy → test GEÇİYOR
    Jetson : CuPy VAR  → `auto` → `xp` = cupy  →
             `TypeError: Unsupported type <class 'numpy.ndarray'>`

Yani nöbetçi, teknenin **gerçekten kullandığı** hesap yolunda hiç sınanmamıştı.
Üretim kodunda kusur yoktu; kusur testin kendisindeydi.

## ⚠ AYIRT EDİCİ "ham çağrı" DEĞİL, **backend pinli mi**
İlk taramam `_trajectory_cost(traj, U)` yazan her yeri kusurlu saymıştı;
**yanlış alarmdı.** `test_mppi_koridor.py` ve `test_mppi.py` config'lerinin
tamamı `backend="numpy"` pinliyor (ikincisi `_cfg()` yardımcısındaki dict
literal üzerinden — `MPPIConfig(**base)` olduğu için AST'de keyword görünmez).
`backend="numpy"` ⇒ `_resolve_backend` `xp = np` döndürür ⇒ ham dizi zararsız.

**Ölçüldü** (`jetson_cupy_taklidi.py` eklentisiyle, `auto` → sahte CuPy):

| dosya | taklit altında |
|---|---|
| `test_ileri_tercihi.py` (düzeltilmiş) | 40 geçti |
| `test_ileri_tercihi.py` (düzeltme geri alınmış) | **TypeError** |
| `test_mppi_koridor.py` | 13 geçti |
| `test_mppi.py` | yalnız `_has_cupy()` korumalı 2 test — **tezgâh kusuru** |

Bu yüzden aşağıdaki kural **modül düzeyinde ve kaba**: bir test modülü
`backend`'i HİÇ anmıyorsa `auto`ya (yani Jetson'da GPU'ya) düşer; o modülde
her `_trajectory_cost` çağrısı çevrilmiş olmalıdır. `backend`'i anan modüller
kendi yolunu seçmiştir, karışılmaz — kabalık bilinçli: yanlış alarm üretmemek
gerçek kusuru kaçırmamaktan daha önemli değil, ama burada ikisi çatışmıyor.
"""
from __future__ import annotations

import ast
import io
import pathlib

import pytest

_TESTLER = pathlib.Path(__file__).resolve().parent

#: `xp` üzerinde çalışan, girdisi de aynı kütüphaneden OLMAK ZORUNDA olan
#: iç metotlar. GPU yolunda ham numpy verilirse TypeError.
_HESAP_YOLU_API = ("_trajectory_cost", "_batch_derivatives")

#: Girdiyi hesap yoluna çeviren kabul edilen kalıplar.
_CEVIRICI = ("xp.asarray", "_as_numpy", "asarray(")


def _auto_backende_dusuyor(src: str) -> bool:
    """Modül `backend`'i hiç anmıyorsa `MPPIConfig` varsayılanı = "auto"."""
    return "MPPIConfig" in src and "backend" not in src


def _cevrilmemis_cagrilar(src: str) -> list:
    try:
        agac = ast.parse(src)
    except SyntaxError:
        return []
    bulgular = []
    for n in ast.walk(agac):
        if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)):
            continue
        if n.func.attr not in _HESAP_YOLU_API:
            continue
        for arg in n.args:
            k = ast.unparse(arg)
            if not any(c in k for c in _CEVIRICI):
                bulgular.append((n.lineno, n.func.attr, k[:30]))
                break
    return bulgular


def _modul_kaynaklari():
    for p in sorted(_TESTLER.glob("test_*.py")):
        if p.name == pathlib.Path(__file__).name:
            continue
        yield p, io.open(p, encoding="utf-8").read()


def test_MPPIConfig_varsayilan_backendi_HALA_auto():
    """Kuralın dayanağı. Varsayılan "numpy" olursa bu dosya anlamsızlaşır."""
    from prototype.planning.mppi import MPPIConfig
    assert MPPIConfig().backend == "auto"


def test_backend_pinlenmemis_modullerde_HAM_dizi_YOK():
    """🔑 Asıl kural — Jetson'da (CuPy) kırmızı yanacak tek desen."""
    suclu = {}
    for p, src in _modul_kaynaklari():
        if not _auto_backende_dusuyor(src):
            continue
        ham = _cevrilmemis_cagrilar(src)
        if ham:
            suclu[p.name] = ham
    assert not suclu, (
        f"`backend` pinlenmemiş (=auto) modülde ham dizi: {suclu} — "
        "girdiyi `c.xp.asarray(...)` ile çevir; doğrulama: "
        "`-p jetson_cupy_taklidi` (bkz. modül docstring'i)")


def test_ileri_tercihi_auto_backendde_KALIYOR():
    """Bu dosya kuralın kapsamından sessizce çıkmasın.

    Birisi `backend="numpy"` pinlerse nöbetçi yeşile döner ama GPU yolunu
    bir daha HİÇ sınamaz — kusurun ilk hâlinden beter, çünkü artık gerekçeli
    görünür. Kapsamdan çıkarmak isteyen bu testi de bilerek silmek zorunda.
    """
    src = io.open(_TESTLER / "test_ileri_tercihi.py", encoding="utf-8").read()
    assert _auto_backende_dusuyor(src), (
        "test_ileri_tercihi.py artık `backend` pinliyor — `geri_hiz_yasak` "
        "nöbetçisi Jetson'ın GPU yolunda koşmaz hale gelir")
    assert not _cevrilmemis_cagrilar(src)


def test_jetson_taklidi_ARACI_yerinde():
    """Ölçüm tezgâhı repoda kalsın — iddianın kanıtı yeniden üretilebilir."""
    arac = _TESTLER / "jetson_cupy_taklidi.py"
    assert arac.exists(), "jetson_cupy_taklidi.py silinmiş"
    metin = io.open(arac, encoding="utf-8").read()
    assert "_resolve_backend" in metin and "TEZGÂH SINIRI" in metin


@pytest.mark.parametrize("dosya", ["test_mppi_koridor.py", "test_mppi.py"])
def test_pinli_moduller_pinli_KALIYOR(dosya):
    """Bu ikisi ham dizi kullanıyor; güvenliği TAMAMEN pine bağlı.

    Pin kalkarsa yukarıdaki asıl kural onları zaten yakalar — bu test o anı
    doğrudan ve okunur biçimde işaretler.
    """
    src = io.open(_TESTLER / dosya, encoding="utf-8").read()
    assert "backend" in src, f"{dosya} backend pinini kaybetti → auto → GPU"

"""`bant_kosum.sh` özet sayaçları DOĞRU mesaja çıpalı mı.

🔴 NEDEN VAR (18.08.2026 — dördüncü sessiz yanlış-sonuç kusuru):
Özet satırı `damga tampon dışı` değerini `grep -oE 'toplam [0-9]+' | tail -1`
ile alıyordu. Ama *"toplam N"* ifadesi **KADANS BEKÇİSİ** ve **SETPOINT
BOŞLUK** mesajlarında da geçiyor ve onlar logda DAHA SONRA basılıyor.
Gerçek göl bandında ölçüldü: gerçek değer **300** iken özet **6** yazıyordu —
**50× İYİMSER** yönde, yani araç "düzeldi" diyordu.

Bu, betiğin kendi başlığında belgelenen üç kusurla aynı sınıf: ölçüm aracının
sessizce yanlış sonuç vermesi. Nöbetçi, çıkarma komutunu **betiğin
kendisinden** okur; birisi deseni geri alırsa bu test kırmızı yanar.
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

BETIK = (Path(__file__).resolve().parents[2] / "scripts" / "bant_kosum.sh")

# Gerçek koşumdan alınmış iki satır. Sıra ÖNEMLİ: tuzak, kadans bekçisinin
# logda SONRA gelmesiydi (`tail -1` onu yakalıyordu).
_LOG = (
    "[WARN] [1787080138.703] [planning_node]: sınıflı algı: damgada poz "
    "bulunamadı (tampon dışı/boş, toplam 300) → EN SON poza düşüldü.\n"
    "[WARN] [1787080167.640] [planning_node]: 🛟 KADANS BEKÇİSİ: cmd_vel "
    "2.27 sn'dir yayınlanmadı (eşik 0.50) → AÇIK SIFIR basıldı (toplam 6).\n"
    "[ERROR] [1787080167.701] [planning_node]: 🔴 SETPOINT AKISINDA 2.27s "
    "BOSLUK (esik 0.50s, toplam 6) — ArduPilot GUIDED'da...\n"
)


def _cikarma_komutu() -> str:
    """`damga tampon dışı` özet satırındaki `$( … )` komutunu betikten al."""
    metin = BETIK.read_text(encoding="utf-8")
    for satir in metin.splitlines():
        if "damga tampon dışı" in satir and "$(" in satir:
            ic = satir[satir.index("$(") + 2:]
            return ic[: ic.rindex(")")]
    pytest.fail("bant_kosum.sh'te `damga tampon dışı` özet satırı bulunamadı")


def test_BETIK_VAR():
    assert BETIK.is_file(), f"ölçüm aracı kayıp: {BETIK}"


def test_damga_sayaci_KENDI_mesajina_capali(tmp_path):
    """Gerçek değer 300; kadans bekçisinin 6'sı yakalanmamalı."""
    (tmp_path / "planning.log").write_text(_LOG, encoding="utf-8")
    sonuc = subprocess.run(
        ["bash", "-c", _cikarma_komutu()],
        env={"L": str(tmp_path), "PATH": "/usr/bin:/bin"},
        capture_output=True, text=True, timeout=30,
    )
    cikti = sonuc.stdout.strip()
    assert "300" in cikti, (
        f"damga sayacı yanlış mesajdan okunuyor: {cikti!r}. "
        "Kadans bekçisinin 'toplam N'ini yakalıyor olabilir (18.08 kusuru)."
    )
    assert cikti.strip() != "6", "kadans bekçisinin sayacı okunmuş — kusur geri geldi"


def test_MUTASYON_eski_desen_bu_nobetciyi_KIRAR(tmp_path):
    """Kusurlu desen bilerek koşturulur: 6 dönmeli (yani nöbetçi iş görüyor)."""
    (tmp_path / "planning.log").write_text(_LOG, encoding="utf-8")
    eski = "grep -oE 'toplam [0-9]+' \"$L/planning.log\" | tail -1"
    sonuc = subprocess.run(
        ["bash", "-c", eski],
        env={"L": str(tmp_path), "PATH": "/usr/bin:/bin"},
        capture_output=True, text=True, timeout=30,
    )
    assert sonuc.stdout.strip() == "toplam 6", (
        "mutasyon beklendiği gibi davranmadı; nöbetçinin iş gördüğü "
        "gösterilemiyor"
    )


def test_UC_ESKI_KUSUR_hala_kapali():
    """Betiğin başlığında belgelenen üç kusurun düzeltmesi yerinde mi."""
    metin = BETIK.read_text(encoding="utf-8")
    assert "use_sim_time" in metin, "kusur 1 geri gelmiş: use_sim_time verilmiyor"
    assert "/girdap/mission/state" in metin, "kusur 2 geri gelmiş: FSM durumu oynatılmıyor"
    # Kusur 3'ün düzeltmesi launch'a dönmek DEĞİL: `ros2 run` kalıyor ama
    # launch varsayılanı olan şalterler ARTIK AÇIKÇA veriliyor. Nöbetçi o
    # düzeltmeye bakar — `rrt_hedef_kurtarma_m` verilmezse düğüm varsayılanı
    # (0.0) ile koşulur ve düzeltme KAPALI ölçülür.
    assert "rrt_hedef_kurtarma_m:=" in metin, (
        "kusur 3 geri gelmiş: launch şalterleri açıkça verilmiyor, "
        "düzeltme KAPALI ölçülür"
    )

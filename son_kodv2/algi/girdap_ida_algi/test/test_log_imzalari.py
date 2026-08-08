#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS logger çağrılarının imzası — kamerasız, ROS kurulumu GEREKTİRMEZ (AST).

NEDEN BU TEST VAR (2026-08-09 arızası):
`duba_gecis_navigator.py`nin "sessiz ret" uyarısına iki yeni tanı sayacı
eklenirken bunlar f-string'e katılmak yerine **ayrı konumsal argüman** olarak
verilmişti:

    self.get_logger().warn(f"...", f"mono_menzil=...", f"menzil_yok=...",
                           throttle_duration_sec=10.0)

rclpy'nin imzası `log(self, message, severity, **kwargs)` — printf tarzı
`*args` YOK (kurulu rclpy 3.3.21/humble'dan doğrulandı; rolling'de de aynı).
Sonuç ölçüldü: `TypeError: RcutilsLogger.warn() takes 2 positional arguments
but 4 were given` → istisna timer callback'inden `rclpy.spin()`e çıkıyor,
`main()` yalnız `KeyboardInterrupt` yakaladığı için **node ölüyor** — hem de
tam o uyarının gerekli olduğu anda (kenar dubası görünüyor, kapı kurulamıyor).
Sahada karşılığı: `/perception/buoys` durur → füzyon CLASS_UNKNOWN →
`gate_follower` ham GPS'e düşer → P1 (G1/KD1≥0,5) ve P2 (≥2 ikili) gider.
`girdap-algi.service`te StartLimit bilerek yok ⇒ sonsuz çakılma döngüsü.

Neden mevcut testler yakalamadı: 126 testin tamamı saf `gecit_mantik`
katmanında; bu satır ROS logger'ına dokunuyor ve o katman testlerde hiç
çalıştırılmıyor. Bu yüzden kontrol ÇALIŞTIRMA ile değil **AST** ile yapılıyor —
ROS kurulu olmayan makinede de koşar, tıpkı scripts/depthai_api_denetimi.py
mantığı gibi (kaynağı kurulu paketin gerçek imzasına karşı doğrula).

Koşum:  python3 -m pytest girdap_ida_algi/test/test_log_imzalari.py -q
"""
import ast
from pathlib import Path

KOK = Path(__file__).resolve().parents[2]

# rclpy'de severity metotlarının tamamı: log(message, **kwargs).
# `warn` humble'da var ama DEPRECATED (rolling'de tamamen kaldırılmış, yalnız
# `warning` kaldı) — biz humble'a pinliyiz, o yüzden ikisi de taranıyor.
SEVIYELER = {"debug", "info", "warn", "warning", "error", "fatal"}


def _python_dosyalari():
    """Repodaki tüm .py — testler ve scripts/ dâhil (hata her yerde olabilir)."""
    for p in sorted(KOK.rglob("*.py")):
        if ".git" in p.parts:
            continue
        yield p


def _logger_cagrilari(agac: ast.AST):
    """`<bir şey>.get_logger().<seviye>(...)` biçimindeki çağrılar."""
    for dugum in ast.walk(agac):
        if not isinstance(dugum, ast.Call):
            continue
        fn = dugum.func
        if not (isinstance(fn, ast.Attribute) and fn.attr in SEVIYELER):
            continue
        alici = fn.value
        if (isinstance(alici, ast.Call)
                and isinstance(alici.func, ast.Attribute)
                and alici.func.attr == "get_logger"):
            yield dugum, fn.attr


def test_logger_cagrilarinda_tek_konumsal_argüman():
    """Hiçbir logger çağrısı 1'den fazla konumsal argüman almamalı."""
    hatalar = []
    for yol in _python_dosyalari():
        try:
            agac = ast.parse(yol.read_text(encoding="utf-8"))
        except SyntaxError as e:                       # pragma: no cover
            hatalar.append(f"{yol.relative_to(KOK)}: ayrıştırılamadı ({e})")
            continue
        for cagri, seviye in _logger_cagrilari(agac):
            if len(cagri.args) > 1:
                hatalar.append(
                    f"{yol.relative_to(KOK)}:{cagri.lineno} "
                    f"get_logger().{seviye}() → {len(cagri.args)} konumsal argüman; "
                    f"rclpy yalnız 1 alır (mesajı f-string'de birleştir)")
    assert not hatalar, (
        "ROS logger çağrısı çalışma anında TypeError atar ve node'u öldürür:\n  "
        + "\n  ".join(hatalar))


def test_logger_cagrilarinda_yildizli_argüman_yok():
    """`*liste` ile açılan argüman da konumsal sayılır → aynı TypeError."""
    hatalar = []
    for yol in _python_dosyalari():
        try:
            agac = ast.parse(yol.read_text(encoding="utf-8"))
        except SyntaxError:                            # pragma: no cover
            continue
        for cagri, seviye in _logger_cagrilari(agac):
            if any(isinstance(a, ast.Starred) for a in cagri.args):
                hatalar.append(f"{yol.relative_to(KOK)}:{cagri.lineno} "
                               f"get_logger().{seviye}(*...) ")
    assert not hatalar, "logger'a yıldızlı argüman:\n  " + "\n  ".join(hatalar)


def test_tarayici_hatali_deseni_gercekten_yakaliyor():
    """Testin kendisi işe yarıyor mu — 2026-08-09'daki HATALI kod üzerinde."""
    hatali = (
        "self.get_logger().warn(\n"
        "    f'kapı kurulamıyor',\n"
        "    f'mono_menzil={x}',\n"
        "    throttle_duration_sec=10.0)\n"
    )
    bulunan = [len(c.args) for c, _ in _logger_cagrilari(ast.parse(hatali))]
    assert bulunan == [2], "tarayıcı hatalı deseni kaçırıyor"

    dogru = "self.get_logger().warn(f'a {x} b {y}', throttle_duration_sec=10.0)\n"
    assert [len(c.args) for c, _ in _logger_cagrilari(ast.parse(dogru))] == [1]

#!/usr/bin/env python3
"""USB takılınca md 4.2 teslim dosyalarını kopyalayan CLI (udev tetikler).

Kullanım:
    teslim_topla.py --aygit /dev/sda1          # udev bu şekilde çağırır
    teslim_topla.py --hedef /media/usb         # zaten mount'luysa
    teslim_topla.py --hedef /tmp/x --kuru      # deneme (kopyalar, ayırmaz)

Emniyet: USB'de hiçbir şey silinmez; her koşum kendi klasörüne gider.
Çekirdek mantık `prototype/teslim/toplayici.py`'de (pytest ile test edilir).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from prototype.teslim.toplayici import topla_ve_yaz          # noqa: E402


def _kos(*a: str) -> int:
    return subprocess.run(a, capture_output=True).returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--aygit", help="/dev/sdX1 — mount edilecek USB bölümü")
    ap.add_argument("--hedef", help="zaten mount'lu dizin")
    ap.add_argument("--log-koku", default=str(Path.home() / "girdap_logs"))
    ap.add_argument("--hepsi", action="store_true",
                    help="tüm oturumlar (varsayılan: yalnız en yeni)")
    ap.add_argument("--kuru", action="store_true", help="ayırma (unmount) yapma")
    args = ap.parse_args()

    ayrilacak = False
    if args.hedef:
        hedef = Path(args.hedef)
    elif args.aygit:
        hedef = Path(tempfile.mkdtemp(prefix="girdap_usb_"))
        if _kos("mount", args.aygit, str(hedef)) != 0:
            print(f"HATA: {args.aygit} mount edilemedi", file=sys.stderr)
            return 2
        ayrilacak = True
    else:
        ap.error("--aygit ya da --hedef gerekli")

    try:
        rapor, bulgular = topla_ve_yaz(
            Path(args.log_koku), hedef, hepsi=args.hepsi
        )
        print(f"{rapor.kopyalanan} dosya, {rapor.bayt/1e6:.1f} MB → {rapor.hedef}")
        if rapor.eksik_zorunlu:
            print("🔴 EKSİK:", ", ".join(rapor.eksik_zorunlu), file=sys.stderr)
        if rapor.bozuk:
            print("🔴 BOZUK:", "; ".join(rapor.bozuk), file=sys.stderr)
        os.sync()                       # veri diske insin (yarım dosya olmasın)
        return 0 if rapor.basarili else 1
    finally:
        if ayrilacak and not args.kuru:
            _kos("umount", str(hedef))  # ayrılması = "bitti" işareti


if __name__ == "__main__":
    raise SystemExit(main())

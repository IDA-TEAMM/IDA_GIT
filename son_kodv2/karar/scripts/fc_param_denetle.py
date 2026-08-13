#!/usr/bin/env python3
"""FC parametre denetimi — LAPTOPTA, Jetson'sız, ROS'suz.

Kullanım:
    python3 scripts/fc_param_denetle.py YENI_DOKUM.param
    python3 scripts/fc_param_denetle.py YENI.param --referans BASKA.param
    python3 scripts/fc_param_denetle.py YENI.param --hepsi   # tüm farklar

🔴 **Neden var.** Parametreleri belirlemek takımda BAŞKASININ görevi ve her
testten sonra güncelleniyor — bizim kodumuzdan bağımsız. 13.08'de Pixhawk'a
bağlanıldığında **39 parametre** değişmiş bulundu; ölçülmüş IMU konumlarımız
sıfırlanmış, batarya izleme kapatılmış, failsafe eylemi kaldırılmıştı. Farkı
elle ayıklamak yarım saat sürdü.

Bu betik o yarım saati birkaç saniyeye indirir ve **yalnız ölümcül olanı**
söyler. Ölümcül listesi `prototype/control/param_denetimi.py`'de; ölçüt tek:
*"yanlışken görev başarısız olur ya da tekne güvensizleşir VE sahada fark
edilmez."* `LOG_*`, `WP_SPEED`, `ATC_*` bilerek dışarıda — yanlışsa yalnız
teşhis/ayar kaybederiz, görev yürür.

Referans dosya (`docs/fc_REFERANS.param`) bizim ONAYLADIĞIMIZ dökümdür;
parametre sorumlusu FC'yi değiştirdiğinde referans DEĞİŞMEZ — fark orada
görünür. Referansı bilerek güncellemek istersen `--referansi-guncelle`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

KOK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KOK))

from prototype.control.param_denetimi import OLUMCUL, denetle  # noqa: E402

VARSAYILAN_REFERANS = KOK / "docs" / "fc_REFERANS.param"


def oku(yol: Path) -> dict:
    """`.param` dosyasını sözlüğe çevir (ad → float)."""
    d = {}
    for satir in yol.read_text(encoding="utf-8", errors="replace").splitlines():
        satir = satir.strip()
        if not satir or satir.startswith("#") or "," not in satir:
            continue
        ad, _, deger = satir.partition(",")
        try:
            d[ad.strip()] = float(deger.strip())
        except ValueError:
            continue
    return d


#: Her açılışta kendiliğinden değişenler — fark sayımında GÜRÜLTÜ sayılır.
#: Bunları elemezsek "39 fark" der ve kimse listeye bakmaz.
KENDILIGINDEN = (
    "STAT_", "INS_GYR", "INS_ACC", "BARO1_GND_PRESS", "COMPASS_DEC",
    "MIS_TOTAL", "ARSPD_OFFSET", "AHRS_TRIM",
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dokum", type=Path, help="FC'den yeni alınan .param dosyası")
    ap.add_argument("--referans", type=Path, default=VARSAYILAN_REFERANS)
    ap.add_argument("--hepsi", action="store_true",
                    help="ölümcül olmayan farkları da listele")
    ap.add_argument("--referansi-guncelle", action="store_true",
                    help="denetim temizse yeni dökümü referans yap")
    a = ap.parse_args()

    if not a.dokum.exists():
        print(f"🔴 dökum bulunamadi: {a.dokum}"); return 2
    if not a.referans.exists():
        print(f"🔴 referans bulunamadi: {a.referans}"); return 2

    yeni, ref = oku(a.dokum), oku(a.referans)

    # 1) ÖLÜMCÜL denetim — beklenen değerler koddan gelir, referans dosyadan DEĞİL
    bulgular = denetle({ad: yeni.get(ad) for ad in OLUMCUL})

    # 2) Bilgi amaçlı: referansa göre kaç şey oynamış
    tum = {k for k in set(ref) | set(yeni)
           if not any(k.startswith(p) for p in KENDILIGINDEN)}
    farklar = sorted(k for k in tum if ref.get(k) != yeni.get(k))

    print(f"dökum   : {a.dokum.name}")
    print(f"referans: {a.referans.name}")
    print(f"izlenen ölümcül parametre: {len(OLUMCUL)}")
    print(f"referansa göre değişen   : {len(farklar)} (kendiliğinden değişenler hariç)")
    print()

    if bulgular:
        print(f"🔴 {len(bulgular)} ÖLÜMCÜL SAPMA — bunlar BİLEREK mi değişti, sor:")
        for b in bulgular:
            print(f"   · {b}")
    else:
        print("✅ ÖLÜMCÜL SAPMA YOK — görevi etkileyecek bir değişiklik görünmüyor.")

    if a.hepsi and farklar:
        olumcul_adlar = {b.ad for b in bulgular}
        digerleri = [k for k in farklar if k not in olumcul_adlar]
        if digerleri:
            print(f"\nölümcül olmayan {len(digerleri)} fark (bilgi):")
            for k in digerleri:
                print(f"   {k:<22} {ref.get(k, 'YOK')} -> {yeni.get(k, 'YOK')}")

    if a.referansi_guncelle:
        if bulgular:
            print("\n🔴 ölümcül sapma varken referans GÜNCELLENMEDİ — önce düzelt.")
            return 1
        a.referans.write_text(a.dokum.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"\n✅ referans güncellendi: {a.referans}")

    return 1 if bulgular else 0


if __name__ == "__main__":
    raise SystemExit(main())

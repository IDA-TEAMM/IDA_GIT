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

## Referansın sözleşmesi — iki farklı kural

Referans (`docs/fc_REFERANS.param`) İKİ TÜR değer taşır ve ikisi farklı
davranır:

- **Ölümcül parametreler** → HER ZAMAN bizim değerimizde kalır. FC'de ne
  yazarsa yazsın referans değişmez; sapma her koşumda bildirilir. Bunlar
  bizim ölçümlerimiz ve güvenlik kararlarımız, FC'nin görüşüne tabi değil.
- **Ölümcül olmayanlar** → FC'de ne varsa referansa YAZILIR (kendiliğinden).

Neden kendiliğinden: parametre sorumlusu her testten sonra onlarca ölümcül
olmayan değeri değiştiriyor. Referans onları emmezse her koşumda aynı 18
farkı gösterir, liste anlamsızlaşır ve kimse bakmaz — bu bekçinin ölüm
sebebi tam olarak budur.

Emilen her değişiklik `docs/fc_param_gunlugu.md`'ye yazılır, yani sessizce
kaybolmaz: "ne zaman, hangi dökümde, ne değişti" sonradan okunabilir.

`--guncelleme-yok` ile emme kapatılır (yalnız rapor).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

KOK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KOK))

from prototype.control.param_denetimi import OLUMCUL, denetle  # noqa: E402

VARSAYILAN_REFERANS = KOK / "docs" / "fc_REFERANS.param"
DOKUM_ADI = "?"


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
    ap.add_argument("--guncelleme-yok", action="store_true",
                    help="referansı GÜNCELLEME, yalnız rapor ver")
    a = ap.parse_args()

    if not a.dokum.exists():
        print(f"🔴 dökum bulunamadi: {a.dokum}"); return 2
    if not a.referans.exists():
        print(f"🔴 referans bulunamadi: {a.referans}"); return 2

    global DOKUM_ADI
    DOKUM_ADI = a.dokum.name
    yeni, ref = oku(a.dokum), oku(a.referans)

    # 1) ÖLÜMCÜL denetim — beklenen değerler koddan gelir, referans dosyadan DEĞİL
    bulgular = denetle({ad: yeni.get(ad) for ad in OLUMCUL})

    # 2) Bilgi amaçlı: referansa göre kaç şey oynamış
    tum = {k for k in set(ref) | set(yeni)
           if not any(k.startswith(p) for p in KENDILIGINDEN)}
    # ⚠ "değişmiş" ile "dökümde hiç yok" AYRI şeyler. Bir parametre FC'nin
    # dökümünden tamamen kaybolabilir: `BATT_MONITOR=0` yapılınca ArduPilot
    # tüm `BATT_*` alt ağacını listelemez. Bunu "15 şey değişti" diye saymak
    # sayıyı kalıcı olarak şişirir ve operatör kısa sürede sayıya bakmayı
    # bırakır — bekçinin güvenilirliğini yiyen şey tam budur.
    farklar = sorted(k for k in tum if k in ref and k in yeni and ref[k] != yeni[k])
    eksikler = sorted(k for k in tum if k in ref and k not in yeni)

    print(f"dökum   : {a.dokum.name}")
    print(f"referans: {a.referans.name}")
    print(f"izlenen ölümcül parametre: {len(OLUMCUL)}")
    print(f"referansa göre değişen   : {len(farklar)} (kendiliğinden değişenler hariç)")
    if eksikler:
        print(f"dökümde HİÇ YOK          : {len(eksikler)} "
              f"(ör. {eksikler[0]} — üst ayar kapalıysa ArduPilot alt ağacı listelemez)")
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

    # --- Referansı tazele: ölümcül olmayanları EMER, ölümcülleri KORUR ------
    if not a.guncelleme_yok:
        emilen = _referansi_tazele(a.referans, ref, yeni, {b.ad for b in bulgular})
        if emilen:
            print(f"\n↻ referansa {len(emilen)} ölümcül-olmayan değişiklik işlendi "
                  f"(günlük: {GUNLUK.name})")
        if bulgular:
            print("   ⚠ ölümcül parametreler referansta BİZİM değerimizde bırakıldı — "
                  "FC'de düzeltilene kadar her koşumda bildirilecek.")

    return 1 if bulgular else 0


GUNLUK = KOK / "docs" / "fc_param_gunlugu.md"


def _referansi_tazele(yol: Path, ref: dict, yeni: dict, olumcul_sapan: set) -> list:
    """Ölümcül olmayanları FC'den al, ölümcülleri bizim değerimizde bırak.

    ⚠ Ölümcül parametrelerin referans değeri BİLEREK dokunulmaz: FC'de yanlış
    olan bir değeri referansa yazmak, o sapmayı bir daha asla göremememiz
    demektir — bekçiyi kendi eliyle kör etmek.
    """
    emilen = []
    yeni_ref = dict(ref)
    for ad, deger in yeni.items():
        if ad in OLUMCUL:
            continue                       # ölümcül → referans bizim değerimizde kalır
        if any(ad.startswith(k) for k in KENDILIGINDEN):
            yeni_ref[ad] = deger           # gürültü: sessizce güncelle, günlüğe yazma
            continue
        if ref.get(ad) != deger:
            emilen.append((ad, ref.get(ad), deger))
        yeni_ref[ad] = deger
    # referansta olup yeni dökümde olmayanlar (ör. BATT_MONITOR=0 iken kaybolan
    # BATT_* satırları) BIRAKILIR — silmek bilgi kaybı olurdu.

    # ⚠ `:g` KULLANMA — 6 anlamlı haneye kırpar (0.08542461 → 0.0854246) ve
    # bir sonraki okumada değer "değişmiş" görünür; referans her koşumda aynı
    # satırları sonsuza kadar raporlar. `repr` Python'un gidiş-dönüş güvenli
    # en kısa gösterimidir. (13.08'de bu hata canlı yakalandı: ikinci koşum
    # 18 yerine 30 fark gösterdi, günlükte eski==yeni satırları belirdi.)
    yol.write_text("\n".join(f"{k},{yeni_ref[k]!r}" for k in sorted(yeni_ref)) + "\n",
                   encoding="utf-8")
    if emilen:
        satirlar = [f"\n## {DOKUM_ADI} → referans ({len(emilen)} değişiklik)\n",
                    "| parametre | eski | yeni |", "|---|---|---|"]
        satirlar += [f"| `{ad}` | {e if e is not None else 'YOK'} | {y!r} |"
                     for ad, e, y in emilen]
        with GUNLUK.open("a", encoding="utf-8") as f:
            f.write("\n".join(satirlar) + "\n")
    return emilen


if __name__ == "__main__":
    raise SystemExit(main())

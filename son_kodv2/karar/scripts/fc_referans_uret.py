#!/usr/bin/env python3
"""FC'ye bağlanıp TAM referans dökümü üret — varsayılanlar FC'den çekilerek.

    python3 scripts/fc_referans_uret.py --baglanti /dev/ttyUSB0:57600
    python3 scripts/fc_referans_uret.py --pck kaydedilmis.pck      # çevrimdışı

🔴 **Neden bu araç var.** "Ölümcül olanlar bizim değerimizde, kalibrasyon
sonuçları olduğu gibi, amaçsızca değişmiş olanlar varsayılana" diye bir döküm
istendi. Bunun için ArduPilot'un **varsayılan tablosu** gerekiyor ve o tablo
hiçbir `.param` dökümünde YOK.

Mission Planner'ın "Default" sütunu FC'den geliyor: ArduPilot 4.3+ MAVFTP
üzerinden `@PARAM/param.pck?withdefaults=1` sunar — her parametrenin hem
mevcut hem varsayılan değeri. (MP'nin `ParameterMetaDataBackup.xml`'i
kontrol edildi: açıklama/aralık var, **varsayılan YOK** — o yüzden dosyadan
okunamıyor, FC'den çekmek şart.)

## Dört kural — her parametre bunlardan birine girer

1. **ÖLÜMCÜL (19)** → değer KODDAN gelir (`param_denetimi.OLUMCUL`).
   FC'de ne yazarsa yazsın bizim ölçtüğümüz/karar verdiğimiz değer yazılır.
2. **BİZİM DİĞER AYARLARIMIZ** → `GIRDAP_ZORUNLU_PARAMETRELER.param`'daki
   kalan satırlar (SERVO trim, GPS tipi, log ayarları…). Belgeli gerekçeleri
   var, varsayılana döndürülmez.
3. **KALİBRASYON TÜREVİ** → FC'deki değer AYNEN korunur. Bunlar ölçümle
   üretilir, elle yazılmaz: pusula, ivmeölçer, jiro, ufuk trim'i, RC.
   ⚠ Varsayılana döndürmek = kalibrasyonu silmek.
4. **GERİ KALAN** → FC'deki değer varsayılandan farklıysa **VARSAYILANA**
   döndürülür ve raporlanır. "Amaçsızca değişmiş" dediğimiz sınıf budur.

Çıktı iki dosyadır: yüklenecek `.param` ve yanında insan okuması için
`_RAPOR.md` — hangi parametrenin hangi kurala göre ne aldığı, ve 4. kuralla
geri alınanların tam listesi. **Rapor okunmadan yükleme yapılmamalı.**
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

KOK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KOK))
from prototype.control.param_denetimi import OLUMCUL          # noqa: E402

ZORUNLU_DOSYA = Path.home() / "Masaüstü" / "GIRDAP_ZORUNLU_PARAMETRELER.param"

#: Kalibrasyon/ölçüm türevi — FC'deki değer KORUNUR, asla varsayılana dönmez.
KALIBRASYON_ONEKLERI = (
    "COMPASS_",        # pusula kalibrasyonu (OFS/DIA/ODI) + cihaz kimlikleri
    "INS_ACC", "INS_GYR",   # ivmeölçer ve jiro kalibrasyonu
    "AHRS_TRIM_",      # ufuk (level) kalibrasyonu
    "RC1_", "RC2_", "RC3_", "RC4_", "RC5_", "RC6_", "RC7_", "RC8_",  # RC kal.
    "BARO", "STAT_", "SYSID_", "FORMAT_VERSION",
)


def _oku_param_dosyasi(yol: Path) -> dict:
    d = {}
    for l in yol.read_text(encoding="utf-8", errors="replace").splitlines():
        l = l.strip()
        if not l or l.startswith("#") or "," not in l:
            continue
        ad, deger = l.split(",", 1)
        try:
            d[ad.strip()] = float(deger.split("#")[0].strip())
        except ValueError:
            pass
    return d


def pck_coz(ham: bytes) -> dict:
    """`param.pck?withdefaults=1` çöz → {ad: (deger, varsayilan|None)}.

    Biçim ArduPilot'un `AP_Param::pack()`'inden gelir; MAVProxy'nin
    `mavproxy_param.py`'si aynı yapıyı okur. Sürüm farklarına karşı
    çözümleyici HATA VERİRSE sessizce yarım sonuç dönmez — patlar, çünkü
    yarım bir referans, olmayandan tehlikelidir.
    """
    from MAVProxy.modules.lib import mp_util          # noqa: F401  (varlık kontrolü)
    from MAVProxy.modules import mavproxy_param as mp
    return mp.ParamState.unpack_param_pck(ham) if hasattr(
        mp.ParamState, "unpack_param_pck") else _pck_coz_yerel(ham)


def _pck_coz_yerel(ham: bytes) -> dict:
    """MAVProxy sürümü yardımcı sunmuyorsa elle çöz."""
    magic, num, total = struct.unpack("<HHH", ham[:6])
    if magic not in (0x671B, 0x671C):
        raise ValueError(f"beklenmeyen param.pck sihirli sayisi: 0x{magic:04X}")
    varsayilanli = magic == 0x671C
    ofs, sonuc, ad_onceki = 6, {}, ""
    tipler = {1: ("b", 1), 2: ("h", 2), 3: ("i", 4), 4: ("f", 4)}
    while ofs < len(ham) and len(sonuc) < num:
        ptype = ham[ofs]; plen = ham[ofs + 1]; ofs += 2
        tip = ptype & 0x0F
        ortak = plen >> 4
        adlen = (plen & 0x0F) + 1
        ad = ad_onceki[:ortak] + ham[ofs:ofs + adlen].decode("ascii", "replace")
        ofs += adlen
        f, n = tipler[tip]
        deger = struct.unpack("<" + f, ham[ofs:ofs + n])[0]; ofs += n
        vars_ = None
        if varsayilanli and (ptype & 0x40):
            vars_ = struct.unpack("<" + f, ham[ofs:ofs + n])[0]; ofs += n
        sonuc[ad] = (float(deger), None if vars_ is None else float(vars_))
        ad_onceki = ad
    return sonuc


def indir(baglanti: str) -> bytes:
    from pymavlink import mavutil, mavftp
    print(f"bağlanılıyor: {baglanti} …")
    m = mavutil.mavlink_connection(baglanti)
    m.wait_heartbeat(timeout=20)
    print(f"✅ FC bağlı (sys {m.target_system}) · param.pck indiriliyor…")
    ftp = mavftp.MAVFTP(m, m.target_system, m.target_component)
    hedef = Path("/tmp/param_withdefaults.pck")
    ftp.cmd_get(["@PARAM/param.pck?withdefaults=1", str(hedef)])
    ftp.process_ftp_reply("OpenFileRO", timeout=30)
    return hedef.read_bytes()


def main() -> int:
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--baglanti", help="pymavlink bağlantı dizgesi")
    a.add_argument("--pck", type=Path, help="daha önce indirilmiş param.pck")
    a.add_argument("--cikti", type=Path,
                   default=Path.home() / "Masaüstü" / "fc_REFERANS_YENI.param")
    n = a.parse_args()

    if not n.baglanti and not n.pck:
        print("🔴 --baglanti ya da --pck ver"); return 2
    ham = n.pck.read_bytes() if n.pck else indir(n.baglanti)
    tablo = pck_coz(ham)
    varsayilani_olan = sum(1 for _, v in tablo.values() if v is not None)
    print(f"okunan: {len(tablo)} parametre · varsayılanı bilinen: {varsayilani_olan}")
    if varsayilani_olan == 0:
        print("🔴 FC varsayılan bilgisi vermedi (eski firmware?) — 4. kural "
              "uygulanamaz, iş yarım kalır. Durduruldu."); return 1

    zorunlu = _oku_param_dosyasi(ZORUNLU_DOSYA) if ZORUNLU_DOSYA.exists() else {}
    cikti, rapor = {}, {"olumcul": [], "bizim": [], "kalibrasyon": [],
                        "varsayilana_donen": [], "zaten_varsayilan": []}

    for ad, (deger, vars_) in sorted(tablo.items()):
        if ad in OLUMCUL:
            y = OLUMCUL[ad].deger
            if OLUMCUL[ad].asgari:
                y = max(y, deger)                    # asgari: FC iyiyse koru
            cikti[ad] = y
            rapor["olumcul"].append((ad, deger, y))
        elif ad in zorunlu:
            cikti[ad] = zorunlu[ad]
            rapor["bizim"].append((ad, deger, zorunlu[ad]))
        elif any(ad.startswith(p) for p in KALIBRASYON_ONEKLERI):
            cikti[ad] = deger
            rapor["kalibrasyon"].append((ad, deger))
        elif vars_ is not None and abs(deger - vars_) > 1e-9:
            cikti[ad] = vars_
            rapor["varsayilana_donen"].append((ad, deger, vars_))
        else:
            cikti[ad] = deger
            rapor["zaten_varsayilan"].append(ad)

    n.cikti.write_text(
        "\n".join(f"{k},{cikti[k]!r}" for k in sorted(cikti)) + "\n", encoding="utf-8")

    r = [f"# FC referans üretimi — {n.cikti.name}", "",
         f"- okunan parametre: **{len(tablo)}**",
         f"- ölümcül (kodumuzdan): **{len(rapor['olumcul'])}**",
         f"- bizim diğer ayarlarımız: **{len(rapor['bizim'])}**",
         f"- kalibrasyon türevi (KORUNDU): **{len(rapor['kalibrasyon'])}**",
         f"- 🔴 varsayılana DÖNDÜRÜLEN: **{len(rapor['varsayilana_donen'])}**",
         f"- zaten varsayılanda: {len(rapor['zaten_varsayilan'])}", "",
         "## 🔴 Varsayılana döndürülenler — YÜKLEMEDEN ÖNCE OKU", "",
         "Bunlar bir sebeple değiştirilmiş olabilir; listede tanıdığın bir",
         "parametre varsa yüklemeden önce sor.", "",
         "| parametre | FC'de | → varsayılan |", "|---|---|---|"]
    r += [f"| `{ad}` | {d:g} | {v:g} |" for ad, d, v in rapor["varsayilana_donen"]]
    r += ["", "## Ölümcül — değer kodumuzdan", "", "| parametre | FC'de | → bizim |",
          "|---|---|---|"]
    r += [f"| `{ad}` | {d:g} | {y:g} |" for ad, d, y in rapor["olumcul"] if abs(d - y) > 1e-9]
    Path(str(n.cikti).replace(".param", "_RAPOR.md")).write_text("\n".join(r) + "\n",
                                                                 encoding="utf-8")
    print(f"\n📄 {n.cikti}")
    print(f"📄 {str(n.cikti).replace('.param', '_RAPOR.md')}")
    print(f"   🔴 varsayılana döndürülen: {len(rapor['varsayilana_donen'])} "
          f"— raporu OKUMADAN yükleme")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

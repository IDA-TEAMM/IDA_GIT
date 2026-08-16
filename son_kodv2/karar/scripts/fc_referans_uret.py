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
#: Oturumlar boyunca birikerek doğrulanmış KANONİK referans — `fc_param_
#: denetle.py`'nin sürekli güncellediği dosya. 39 satırlık ZORUNLU_DOSYA'dan
#: çok daha geniş: WP_SPEED, CRUISE_*, SERVO1/3 aralığı, ATC_STR_ANG_P,
#: RC10_OPTION (kaptanın MotorEStop ataması), BRD_SAFETY_DEFLT, BATT_* tam
#: kalibrasyon bloğu gibi haftalarca sahada doğrulanmış ayarları taşıyor.
KANONIK_DOSYA = KOK / "docs" / "fc_REFERANS.param"

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

    🔴 **15.08 DÜZELTMESİ — burada elle yazılmış bir çözücü vardı ve
    ÜÇ yerden yanlıştı**, gerçek donanıma karşı ilk çalıştırıldığında
    (`ofs=15`'te "bilinmeyen tip nibble 13" ile) ortaya çıktı:
      1. Kayıtlar arasındaki **dolgu (pad) baytları hiç atlanmıyordu**.
      2. "Varsayılan değer de var mı" biti `ptype & 0x40` sanılmıştı,
         gerçeği `(ptype >> 4) & 0x1` (yani `ptype & 0x10`).
      3. `plen` baytının **ad-uzunluğu ve ortak-önek-uzunluğu nibble'ları
         TERS** okunuyordu (üst nibble ad uzunluğu, alt nibble ortak önek —
         ben ikisini birbirine karıştırmıştım).
    Üçü birlikte: yalnız İLK kayıtta bile yanlış bayt sayısı tüketilip
    akış hemen desenkron oluyordu. Bu üç hata bir REFERANS DOSYASINDA
    yanlış ada yanlış değer yazdırabilirdi — ikinci bir çözücüyle
    "çapraz kontrol" yapmak yeterli değil, TEK doğru kaynağa geçildi.

    Doğru biçim ArduPilot'un `AP_Param::pack()`'inden gelir; kendi elle
    yazdığım decoder yerine bunu üreten firmware'in TEK gerçek muadili olan
    MAVProxy'nin kendi `param_ftp.ftp_param_decode()`'u kullanılıyor —
    ArduPilot ekibiyle aynı takım bu kütüphaneyi bakımlı tutuyor.
    """
    from MAVProxy.modules.lib import param_ftp
    veri = param_ftp.ftp_param_decode(ham)
    if veri is None:
        raise ValueError(
            "param_ftp.ftp_param_decode None döndü — bozuk magic ya da "
            "kayıt sayısı uyuşmuyor (kütüphane kendi hatasını stderr'e "
            "bastı). Yarım sonuç ÜRETİLMEDİ."
        )
    def _str(ad):
        return ad.decode("ascii") if isinstance(ad, bytes) else ad

    varsayilanlar = {_str(ad): deger for ad, deger, _tip in (veri.defaults or [])}
    sonuc = {}
    for ad, deger, _tip in veri.params:
        ad = _str(ad)
        vars_ = varsayilanlar.get(ad)
        sonuc[ad] = (float(deger), None if vars_ is None else float(vars_))
    return sonuc


def indir(baglanti: str, deneme: int = 4) -> bytes:
    """`param.pck?withdefaults=1`'i MAVFTP ile indir.

    🔴 **15.08 — bayt-sayısı denetimi YANLIŞ ALARMDI, kaldırıldı.**
    İlk yazımda MAVFTP'nin `remote_file_size`'ına (OpenFileRO yanıtından
    gelen bir "beklenen boyut" alanı) güvenip onunla eşleşmeyen indirmeyi
    "eksik" sayıp yeniden deniyordum. 4 üst üste denemede DE aynı 10327
    bayt geldi (10908 "bekleniyordu") — bu rastgele bir kayıp olsaydı
    tekrar aynı sayıya düşmezdi, DETERMİNİSTİKTİ. Doğrudan test edince
    (`param_ftp.ftp_param_decode` o 10327 baytı **909 parametrenin 909'unu
    da temiz çözdü**) `remote_file_size`'ın kendisinin GÜVENİLMEZ olduğu
    ortaya çıktı — ArduPilot'un stat() tahmini ile gerçek üretilen akış
    uzunluğu bu FC/firmware kombinasyonunda örtüşmüyor.

    ⇒ Doğru tamlık ölçütü bayt sayısı DEĞİL, **gerçek başarılı çözümleme**:
    `pck_coz` içindeki `ftp_param_decode` zaten kendi `count != total_params`
    denetimini yapıyor ve eksikse `None` dönüyor — o denetim yeterli, onu
    tekrarlamak (yanlış bir alanla) sahte kırmızı üretiyordu.
    """
    from pymavlink import mavutil, mavftp
    print(f"bağlanılıyor: {baglanti} …")
    m = mavutil.mavlink_connection(baglanti)
    m.wait_heartbeat(timeout=20)
    print(f"✅ FC bağlı (sys {m.target_system}) · param.pck indiriliyor…")

    son_boyut = None
    for i in range(1, deneme + 1):
        ftp = mavftp.MAVFTP(m, m.target_system, m.target_component)
        hedef = Path("/tmp/param_withdefaults.pck")
        hedef.unlink(missing_ok=True)
        ftp.cmd_get(["@PARAM/param.pck?withdefaults=1", str(hedef)])
        ftp.process_ftp_reply("OpenFileRO", timeout=30)
        if not hedef.exists():
            print(f"  [{i}/{deneme}] indirme boş döndü, yeniden deneniyor…")
            continue
        ham = hedef.read_bytes()
        son_boyut = len(ham)
        try:
            if len(ham) < 6:
                raise ValueError("başlık bile gelmedi")
            pck_coz(ham)                     # yalnız tamlık testi; sonuç atılır
        except ValueError as exc:
            print(f"  [{i}/{deneme}] çözülemedi ({son_boyut} bayt: {exc}), "
                  f"yeniden deneniyor…")
            continue
        if i > 1:
            print(f"  ✅ {i}. denemede tamamlandı ({son_boyut} bayt)")
        return ham
    raise RuntimeError(
        f"param.pck {deneme} denemede de ÇÖZÜLEMEDİ (son: {son_boyut} bayt) "
        "— gerçek bir bağlantı sorunu olabilir. Yarım sonuç ÜRETİLMEDİ."
    )


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
    kanonik = _oku_param_dosyasi(KANONIK_DOSYA) if KANONIK_DOSYA.exists() else {}
    cikti, rapor = {}, {"olumcul": [], "bizim": [], "kalibrasyon": [],
                        "kanonik": [],
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
        elif ad in kanonik and vars_ is not None and abs(kanonik[ad] - vars_) > 1e-6:
            # 🔴 15.08 gecesi bulundu — bu dal EKSİKTİ. Kanonik dosyada
            # varsayılandan FARKLI bir değer kayıtlıysa bu SAHADA doğrulanmış
            # bilinçli bir ayardır (WP_SPEED, CRUISE_*, SERVO aralığı, RC10
            # MotorEStop, BRD_SAFETY_DEFLT…) — "amaçsız sapma" değil. Kural 4
            # bunu hiç görmeden varsayılana döndürüyordu; 8-9 gerçek ayarı
            # sessizce sildi ve saha testinde arm'ı bile engelledi
            # (BRD_SAFETY_DEFLT). Tolerans kanonik'in KENDİSİ de varsayılandan
            # anlamlı ölçüde ayrışmışsa devreye girer — kanonik'in varsayılanla
            # aynı olduğu durumlarda (asıl "hiç dokunulmamış") kural 4 zaten
            # doğru sonucu veriyordu, o yol bozulmadı.
            cikti[ad] = kanonik[ad]
            rapor["kanonik"].append((ad, deger, kanonik[ad]))
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
         f"- kanonik dosyadan (saha-doğrulanmış, varsayılan DEĞİL): **{len(rapor['kanonik'])}**",
         f"- kalibrasyon türevi (KORUNDU): **{len(rapor['kalibrasyon'])}**",
         f"- 🔴 varsayılana DÖNDÜRÜLEN: **{len(rapor['varsayilana_donen'])}**",
         f"- zaten varsayılanda: {len(rapor['zaten_varsayilan'])}", "",
         "## Kanonik dosyadan geri konanlar — saha-doğrulanmış, DOKUNULMADI", "",
         "| parametre | FC'de | → kanonik |", "|---|---|---|"]
    r += [f"| `{ad}` | {d:g} | {v:g} |" for ad, d, v in rapor["kanonik"]]
    r += ["", "## 🔴 Varsayılana döndürülenler — YÜKLEMEDEN ÖNCE OKU", "",
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

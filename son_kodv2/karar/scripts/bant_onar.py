#!/usr/bin/env python3
"""GİRDAP İDA — ani güç kesintisinde yarım kalmış MCAP bandını onarır.

12.08.2026 (§0.53). Kaptan: "bu rosbag'i aniden Jetson kapansa bile
çalışacak şekilde ayarla" → "direkt kaydetsin, Pixhawk'taki log gibi düşün."

NE İŞE YARAR
------------
Kayıt artık doğrudan yazım yapıyor (sıkıştırma yok, parçalama kapalı), bu
yüzden fiş çekilince dosyanın yalnız SONU kesiliyor — öncesi sağlam. Ölçüldü:
`ros2 bag info` böyle bir dosyayı sorunsuz okuyor (5.267 mesaj). Ancak
`ros2 bag play` en sondaki YARIM kayda gelince şu hatayla duruyor:

    record type 0x05 at offset 908834 has length 738 but only 469 bytes remaining

Bu araç o yarım kuyruğu keser ve dosyayı kurallara uygun biçimde kapatır
(DataEnd + Footer + bitiş imzası). Sonuç: dosya baştan sona oynatılabilir.

MCAP BİÇİMİ (kullanılan kadarı)
-------------------------------
    8 bayt  başlangıç imzası: \\x89 M C A P 0 \\r \\n
    kayıtlar: [1 bayt işlem kodu][8 bayt uzunluk, küçük sonlu][gövde]
    8 bayt  bitiş imzası (aynı dizi)

Kullanım
--------
    python3 bant_onar.py <dosya.mcap>            # yalnız DENETLE, dokunma
    python3 bant_onar.py <dosya.mcap> --yaz      # onar (yedeğini alarak)
    python3 bant_onar.py <dizin>/ --yaz          # dizindeki tüm .mcap'ler
"""
from __future__ import annotations

import shutil
import struct
import sys
from pathlib import Path

IMZA = b"\x89MCAP0\r\n"
OP_MESSAGE = 0x05
OP_DATA_END = 0x0F
OP_FOOTER = 0x02

ISLEM_ADLARI = {
    0x01: "Header", 0x02: "Footer", 0x03: "Schema", 0x04: "Channel",
    0x05: "Message", 0x06: "Chunk", 0x07: "MessageIndex", 0x08: "ChunkIndex",
    0x09: "Attachment", 0x0A: "AttachmentIndex", 0x0B: "Statistics",
    0x0C: "Metadata", 0x0D: "MetadataIndex", 0x0E: "SummaryOffset",
    0x0F: "DataEnd",
}


def tara(yol: Path):
    """Dosyayı tara. (son_saglam_ofset, mesaj_sayisi, durum) döner."""
    veri = yol.read_bytes()
    n = len(veri)
    if n < len(IMZA) or not veri.startswith(IMZA):
        return None, 0, "MCAP değil (başlangıç imzası yok)"

    i = len(IMZA)
    mesaj = 0
    son_saglam = i
    kapali = False

    while i < n:
        # Bitiş imzası? (dosya düzgün kapanmışsa sonda durur)
        if veri[i:i + len(IMZA)] == IMZA and i + len(IMZA) == n:
            kapali = True
            son_saglam = i
            break
        if i + 9 > n:
            break                                   # başlık bile tam değil
        islem = veri[i]
        (uzunluk,) = struct.unpack_from("<Q", veri, i + 1)
        son = i + 9 + uzunluk
        if son > n:
            break                                   # 🔑 yarım kayıt burada
        if islem == OP_MESSAGE:
            mesaj += 1
        if islem == OP_FOOTER:
            son_saglam = son
            kapali = True
            i = son
            continue
        i = son
        son_saglam = i

    durum = "kapalı (sağlam)" if kapali else "YARIM KALMIŞ"
    return son_saglam, mesaj, durum


def onar(yol: Path, yaz: bool) -> bool:
    boyut = yol.stat().st_size
    son_saglam, mesaj, durum = tara(yol)
    if son_saglam is None:
        print(f"  ❌ {yol.name}: {durum}")
        return False

    kesilen = boyut - son_saglam
    print(f"  {yol.name}")
    print(f"     boyut      : {boyut:,} bayt")
    print(f"     mesaj      : {mesaj:,}")
    print(f"     durum      : {durum}")

    if durum.startswith("kapalı"):
        print("     → onarım GEREKMİYOR")
        return True

    print(f"     kesilecek  : {kesilen:,} bayt (dosyanın %{kesilen/boyut*100:.3f}'i)")
    if not yaz:
        print("     → yalnız denetim; onarmak için --yaz ver")
        return True

    yedek = yol.with_suffix(yol.suffix + ".kesik")
    shutil.copy2(yol, yedek)

    with open(yol, "r+b") as f:
        f.truncate(son_saglam)
        f.seek(son_saglam)
        # DataEnd: data_section_crc (uint32) = 0 (hesaplanmadı)
        f.write(bytes([OP_DATA_END]) + struct.pack("<Q", 4) + struct.pack("<I", 0))
        # Footer: summary_start=0, summary_offset_start=0, summary_crc=0
        #         (0 = "özet yok" → okuyucu dosya sırasıyla okur)
        f.write(bytes([OP_FOOTER]) + struct.pack("<Q", 20)
                + struct.pack("<QQI", 0, 0, 0))
        f.write(IMZA)

    print(f"     ✅ onarıldı (kesik hâli: {yedek.name})")
    return True


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    hedef = Path(sys.argv[1])
    yaz = "--yaz" in sys.argv[2:]

    if hedef.is_dir():
        dosyalar = sorted(hedef.glob("**/*.mcap"))
    else:
        dosyalar = [hedef]
    if not dosyalar:
        print(f"MCAP dosyası bulunamadı: {hedef}")
        return 1

    print(f"== GİRDAP bant onarımı == ({'ONAR' if yaz else 'yalnız DENETLE'})\n")
    for d in dosyalar:
        onar(d, yaz)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

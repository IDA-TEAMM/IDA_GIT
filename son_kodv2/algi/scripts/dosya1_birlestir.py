#!/usr/bin/env python3
"""DOSYA-1 TESLİM ARACI — kayıt segmentlerini TEK mp4'e birleştirir.

🔴 NEDEN VAR (şartname taraması, 11.08.2026 — doğrulanmış PDF sha256 09116afe…):
    md 4.2: *"Otonomi amacıyla kullanılan aşağıdaki veriler kaydedilecek ve
    teslim edilecektir. Veriler **3 dosya** olacak şekilde teslim edilecektir."*
    Biz ise çökme dayanımı için 120 sn'lik `seg_0001.mp4, seg_0002.mp4, …`
    yazıyoruz (20 dk tur ⇒ **10 dosya**). Segmentler TEK dosyaya indirilmezse
    Dosya-1 şartnamedeki biçimde teslim edilmemiş olur.
    md 5.5.4.3.5: teslim edilmeyen **her bir dosya için 5 ceza puanı**, üstelik
    süre penceresi dar: *"İDA'nın karaya alım anından itibaren 20 dakika
    içerisinde, her bir takımın kendi USB flash belleği ile birlikte"*.
    ⇒ Bu betik o 20 dakikada koşacak. Elle ffmpeg komutu yazmak için sahada
      ne vakit var ne de sinir.

🔑 TASARIM: `-c copy` (akış kopyası) — YENİDEN KODLAMA YOK. Segmentlerin hepsi
   aynı VideoWriter ayarlarıyla yazıldığı için (aynı kodek/çözünürlük/FPS)
   kopyalama geçerli ve **saniyeler sürer**; yeniden kodlama dakikalar alır ve
   20 dk penceresinde risk olur. Overlay'ler (bbox, sınıf, zaman etiketi)
   karelere GÖMÜLÜ olduğu için kopyalamada aynen korunur.

🔴 SON SEGMENT BOZUK OLABİLİR: ani güç kesintisinde (fiş çekme / pil bitmesi)
   sürece hiçbir sinyal ulaşmaz, `finally` çalışmaz, o segmentin moov atomu
   yazılmaz. Betik bu yüzden her segmenti ÖNCE doğrular, bozuk olanı ATLAYIP
   devam eder ve ekrana açıkça yazar — tek bozuk segment yüzünden teslim
   edilecek dosyanın hiç üretilememesi kabul edilemez.

Kullanım:
    python3 scripts/dosya1_birlestir.py                     # en son oturum
    python3 scripts/dosya1_birlestir.py --oturum session_20260820_141500
    python3 scripts/dosya1_birlestir.py --usb /media/girdap/USB
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

KAYIT_DIZIN = os.path.expanduser("~/girdap_logs/kamera")


def kare_sayisi(yol):
    """Segmenti gerçekten AÇIP kare sayar. Bozuksa (moov yok) None döner."""
    try:
        import cv2
    except ImportError:
        return -1                      # cv2 yoksa doğrulama atlanır, -1 = bilinmiyor
    c = cv2.VideoCapture(yol)
    if not c.isOpened():
        return None
    n = int(c.get(cv2.CAP_PROP_FRAME_COUNT))
    ok = c.read()[0]
    c.release()
    return n if ok and n > 0 else None


def main():
    ap = argparse.ArgumentParser(description="Dosya-1 segmentlerini tek mp4'e birleştir")
    ap.add_argument("--kayit-dizin", default=KAYIT_DIZIN)
    ap.add_argument("--oturum", default=None,
                    help="session_YYYYmmdd_HHMMSS; verilmezse EN SON oturum")
    ap.add_argument("--cikti", default=None, help="hedef mp4 (varsayılan: oturumun içinde)")
    ap.add_argument("--usb", default=None, help="birleşmiş dosyayı buraya da kopyala")
    a = ap.parse_args()

    if not os.path.isdir(a.kayit_dizin):
        print(f"🔴 kayıt dizini yok: {a.kayit_dizin}")
        return 1

    if a.oturum:
        oturum = os.path.join(a.kayit_dizin, a.oturum)
    else:
        oturumlar = sorted(glob.glob(os.path.join(a.kayit_dizin, "session_*")))
        if not oturumlar:
            print(f"🔴 hiç oturum yok: {a.kayit_dizin}/session_*")
            return 1
        oturum = oturumlar[-1]
        print(f"[i] en son oturum seçildi: {os.path.basename(oturum)}")
        if len(oturumlar) > 1:
            print(f"    (toplam {len(oturumlar)} oturum var — yanlışsa --oturum ile seç)")

    segmentler = sorted(glob.glob(os.path.join(oturum, "seg_*.mp4")))
    if not segmentler:
        print(f"🔴 segment yok: {oturum}/seg_*.mp4")
        return 1

    print(f"\n[i] {len(segmentler)} segment doğrulanıyor…")
    saglam, bozuk, toplam_kare = [], [], 0
    for s in segmentler:
        n = kare_sayisi(s)
        if n is None:
            bozuk.append(s)
            print(f"   🔴 BOZUK, atlanıyor: {os.path.basename(s)} "
                  f"({os.path.getsize(s)/1e6:.1f} MB) — büyük ihtimalle ani güç kesintisi")
        else:
            saglam.append(s)
            if n > 0:
                toplam_kare += n
            print(f"   ✅ {os.path.basename(s)}: {n if n >= 0 else '?'} kare")

    if not saglam:
        print("\n🔴 SAĞLAM SEGMENT YOK — birleştirilecek bir şey kalmadı.")
        return 1

    cikti = a.cikti or os.path.join(oturum, "Dosya1_islenmis_kamera.mp4")
    liste = os.path.join(oturum, "_birlestir_listesi.txt")
    with open(liste, "w", encoding="utf-8") as fh:
        for s in saglam:
            fh.write(f"file '{os.path.abspath(s)}'\n")

    print(f"\n[i] birleştiriliyor → {cikti}")
    if shutil.which("ffmpeg") is None:
        print("🔴 ffmpeg YOK. Kur: sudo apt install -y ffmpeg")
        return 1
    r = subprocess.run(
        ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", liste,
         "-c", "copy", cikti],
        capture_output=True, text=True)
    os.remove(liste)
    if r.returncode != 0:
        print("🔴 ffmpeg başarısız:")
        print(r.stderr[-1500:])
        return 1

    son = kare_sayisi(cikti)
    print(f"\n✅ ÜRETİLDİ: {cikti}")
    print(f"   boyut: {os.path.getsize(cikti)/1e6:.1f} MB")
    print(f"   kare : {son}   (segmentlerin toplamı: {toplam_kare})")
    if son is None:
        print("   🔴 çıktı AÇILAMIYOR — teslim etme, elle kontrol et!")
        return 1
    if toplam_kare and son not in (toplam_kare, -1) and abs(son - toplam_kare) > 2:
        print(f"   ⚠️ kare sayısı tutmuyor ({son} ≠ {toplam_kare}) — gözle doğrula")
    if bozuk:
        print(f"   ⚠️ {len(bozuk)} bozuk segment ATLANDI — o aralık videoda YOK")

    if a.usb:
        if not os.path.isdir(a.usb):
            print(f"   🔴 USB yolu yok: {a.usb}")
            return 1
        hedef = os.path.join(a.usb, os.path.basename(cikti))
        shutil.copy2(cikti, hedef)
        os.sync()                       # 🔴 fişi çekmeden önce gerçekten yazılsın
        print(f"   ✅ USB'ye kopyalandı: {hedef}")

    print("\n🔴 TESLİM: Dosya-1 (bu mp4) + Dosya-2 (telemetri csv, KARAR tarafı)")
    print("   + Dosya-3 (maliyet haritası, KARAR tarafı) — üçü de USB'de olmalı.")
    print("   md 5.5.4.3.5: eksik her dosya için 5 ceza puanı, karaya alımdan 20 dk içinde.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

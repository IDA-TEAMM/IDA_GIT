#!/usr/bin/env python3
"""ARAÇ 5 — DERİNLİK + YOLO, 15 FPS SABİT DAYANIKLILIK TESTİ (depthai 2.x).

Neden bu araç var
-----------------
05.08 ölçümlerinde FPS kararlılığı doğrulandı (15,00 FPS, 5 dk sapma yok) ama
DERİNLİK KALİTESİ doğrulanmadı: test sahnesi düz beyaz duvardı ve stereo
dokusuz yüzeyde disparite üretemez. Yani "derinlik açık ve FPS düşmüyor"
biliniyordu, "derinlik GERÇEKTEN geçerli piksel üretiyor mu" bilinmiyordu.
Bu araç ikisini birden ölçer.

USB notu (05.08 teşhisi)
-----------------------
Bu Jetson'da SuperSpeed link, `tegra-xusb` sürücüsünün U1/U2 güç durumu
pazarlığında çöküyor (kernel: "Disable of device-initiated U1/U2 failed" +
error -71) → cihaz bootlanır, USB3'e geçer, düşer, ROM'a döner. Bu yüzden
varsayılan olarak USB2'ye ZORLANIR (`--usb high`): 5/5 açılış ölçüldü.
Bant genişliği sorun değil — bu yapılandırma ~8 MB/s, USB2 tavanı ~35-40 MB/s.

Kullanım
--------
    python3 scripts/oak_derinlik_termal_testi.py                 # 5 dk, varsayılan
    python3 scripts/oak_derinlik_termal_testi.py --sure 1800     # 30 dk (termal plato)
    python3 scripts/oak_derinlik_termal_testi.py --usb super     # USB3 denemek istersen

Çıktı `~/girdap_logs/derinlik_testi/<zaman>/` altına YAZILIR (JSONL satır satır,
her pencerede flush) — cihaz kapanışta çöktüğü için (3/3 gözlendi) sonuç
bellekte tutulmaz.

🔴 SAHNE UYARISI: kamerayı DOKULU bir sahneye doğrult (eşya, raf, insan,
desenli yüzey). Düz beyaz duvara bakarsa geçerli piksel oranı düşük çıkar ve
bu KAMERA ARIZASI DEĞİLDİR — stereonun fiziksel sınırıdır.
"""
import argparse
import fcntl
import glob
import json
import os
import struct
import sys
import time
import zlib

import numpy as np
import depthai as dai

USBDEVFS_RESET = ord('U') << 8 | 20          # _IO('U', 20)


# ─────────────────────────── USB kurtarma ────────────────────────────
def _usb_dugumu():
    for d in glob.glob("/sys/bus/usb/devices/*/"):
        try:
            if open(d + "idVendor").read().strip() == "03e7":
                b = int(open(d + "busnum").read())
                n = int(open(d + "devnum").read())
                return f"/dev/bus/usb/{b:03d}/{n:03d}"
        except Exception:
            pass
    return None


def usb_reset(bekle=2.5):
    """OAK'a USBDEVFS_RESET gönder. sudo GEREKMEZ — 80-movidius.rules MODE=0666.

    Sahada fişe erişim olmayacağı için kilitlenmenin tek yazılımsal çaresi bu.
    """
    y = _usb_dugumu()
    if not y:
        return False
    try:
        fd = os.open(y, os.O_WRONLY)
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        os.close(fd)
    except OSError:
        return False
    t = time.time()
    while time.time() - t < 10:
        if _usb_dugumu():
            time.sleep(bekle)
            return True
        time.sleep(0.2)
    return False


def cihaz_ac(pipeline, hiz, deneme=4):
    """Kilitlenmeye dayanıklı açılış: X_LINK hatasında USB reset atıp tekrar dener."""
    son = None
    for i in range(deneme):
        try:
            return dai.Device(pipeline, hiz)
        except RuntimeError as e:
            son = e
            print(f"  [açılış] deneme {i + 1}/{deneme}: {str(e)[:70]}")
            if i < deneme - 1:
                print(f"  [açılış] USB reset {'✓' if usb_reset() else '✗'}")
                time.sleep(1.0)
    raise son


# ─────────────────────────── PNG (bağımlılıksız) ──────────────────────
def png_yaz(yol, arr):
    """uint8 dizisini PNG olarak yaz (HxW gri veya HxWx3 RGB). cv2 gerekmez."""
    renk = 0 if arr.ndim == 2 else 2
    h, w = arr.shape[:2]
    ham = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))

    def parca(tip, veri):
        return (struct.pack(">I", len(veri)) + tip + veri
                + struct.pack(">I", zlib.crc32(tip + veri) & 0xFFFFFFFF))

    with open(yol, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + parca(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, renk, 0, 0, 0))
                + parca(b"IDAT", zlib.compress(ham, 6))
                + parca(b"IEND", b""))


def derinlik_renklendir(d_mm, tavan_mm):
    """Derinliği görsel denetim için renklendir: yakın=mavi, uzak=kırmızı, GEÇERSİZ=siyah."""
    gecerli = d_mm > 0
    n = np.clip(d_mm.astype(np.float32) / max(tavan_mm, 1), 0, 1)
    rgb = np.zeros(d_mm.shape + (3,), np.uint8)
    rgb[..., 0] = (n * 255)                      # uzak → kırmızı
    rgb[..., 1] = ((1 - np.abs(n - 0.5) * 2) * 255)
    rgb[..., 2] = ((1 - n) * 255)                # yakın → mavi
    rgb[~gecerli] = 0
    return rgb


# ─────────────────────────── pipeline ─────────────────────────────────
def pipeline_kur(a):
    p = dai.Pipeline()

    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(416, 416)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setPreviewKeepAspectRatio(False)         # letterbox davranışı (deploy ile aynı)
    cam.setFps(a.fps)

    sol = p.create(dai.node.MonoCamera)
    sag = p.create(dai.node.MonoCamera)
    sol.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    sag.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for m in (sol, sag):
        m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        m.setFps(a.fps)

    st = p.create(dai.node.StereoDepth)
    st.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, a.preset))
    st.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    # 🔴 ZORUNLU: setDepthAlign(CAM_A) tek başına derinliği RGB çözünürlüğüne
    # (1920×1080 = 4,1 MB/kare) ölçekler ve USB2'yi doldurup FPS'i ~8'e düşürür.
    # setOutputSize ile 640×400 = 512 KB/kare → ölçüldü: 8 → ~15 FPS.
    st.setOutputSize(*a.cikti)
    if a.lrc:
        st.setLeftRightCheck(True)               # yanlış eşleşmeleri ele (kalite ↑, hız ↓)
    sol.out.link(st.left)
    sag.out.link(st.right)

    xd = p.create(dai.node.XLinkOut)
    xd.setStreamName("derinlik")
    xd.input.setBlocking(False)
    xd.input.setQueueSize(2)

    xr = p.create(dai.node.XLinkOut)
    xr.setStreamName("rgb")
    xr.input.setBlocking(False)
    xr.input.setQueueSize(2)

    if a.model:
        nn = p.create(dai.node.YoloSpatialDetectionNetwork)
        nn.setBlobPath(a.model)
        nn.setConfidenceThreshold(0.5)
        nn.setNumClasses(a.sinif)
        nn.setCoordinateSize(4)
        nn.setIouThreshold(0.5)
        nn.setBoundingBoxScaleFactor(0.5)        # derinlik ROI'si bbox'ın %50'si
        nn.setDepthLowerThreshold(100)           # mm
        nn.setDepthUpperThreshold(10000)
        cam.preview.link(nn.input)
        st.depth.link(nn.inputDepth)
        nn.passthroughDepth.link(xd.input)
        nn.passthrough.link(xr.input)
        xn = p.create(dai.node.XLinkOut)
        xn.setStreamName("tespit")
        nn.out.link(xn.input)
    else:
        st.depth.link(xd.input)
        cam.preview.link(xr.input)

    return p


# ─────────────────────────── ana ölçüm ────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sure", type=float, default=300, help="test süresi (sn)")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--pencere", type=float, default=15.0, help="ölçüm penceresi (sn)")
    ap.add_argument("--preset", default="ROBOTICS",
                    choices=["ROBOTICS", "HIGH_DENSITY", "HIGH_ACCURACY", "DEFAULT", "FACE"])
    ap.add_argument("--usb", default="high", choices=["high", "super", "auto"])
    ap.add_argument("--model", default="", help="yolo .blob yolu (boşsa NN yok)")
    ap.add_argument("--sinif", type=int, default=80)
    ap.add_argument("--tavan", type=int, default=10000, help="derinlik tavanı mm (görsel)")
    ap.add_argument("--lrc", type=int, default=1, help="setLeftRightCheck (1/0)")
    ap.add_argument("--cikti", type=int, nargs=2, default=[640, 400],
                    metavar=("G", "Y"), help="stereo çıktı boyutu (USB yükünü belirler)")
    ap.add_argument("--kare-kaydet", type=float, default=60.0, help="kaç sn'de bir PNG")
    a = ap.parse_args()

    hiz = {"high": dai.UsbSpeed.HIGH, "super": dai.UsbSpeed.SUPER,
           "auto": dai.UsbSpeed.SUPER_PLUS}[a.usb]

    kok = os.path.expanduser("~/girdap_logs/derinlik_testi/"
                             + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(kok, exist_ok=True)
    jsonl = open(os.path.join(kok, "olcum.jsonl"), "w")

    print("=" * 70)
    print(f"  DERİNLİK + {'YOLO ' if a.model else ''}{a.fps:.0f} FPS SABİT TESTİ"
          f" — {a.sure:.0f} sn, preset {a.preset}")
    print(f"  depthai {dai.__version__} | USB isteği: {a.usb.upper()}")
    print(f"  çıktı: {kok}")
    print("=" * 70)
    print("  🔴 Kamerayı DOKULU bir sahneye doğrult — düz beyaz duvarda stereo")
    print("     disparite üretemez, düşük geçerli-piksel oranı arıza DEĞİLDİR.\n")

    dev = cihaz_ac(pipeline_kur(a), hiz)
    with dev:
        gercek_usb = str(dev.getUsbSpeed()).split(".")[-1]
        print(f"  ✅ AÇILDI — USB linki: {gercek_usb}"
              f" | MxId {dev.getMxId()}\n")

        q_d = dev.getOutputQueue("derinlik", 2, False)
        q_r = dev.getOutputQueue("rgb", 2, False)
        q_n = dev.getOutputQueue("tespit", 4, False) if a.model else None

        t0 = time.time()
        pen_bas = t0
        n_d = n_r = n_n = 0            # pencere sayaçları
        top_d = top_r = 0              # toplam
        son_kayit = 0.0
        son_rgb = None
        son_derinlik = None
        # Geçerli piksel oranı pencere içindeki TÜM karelerden ortalanır.
        # (Tek kareden okumak yanıltıyor: aynı sahnede kareler arası %5 ↔ %60 oynuyor.)
        pen_gecerli, pen_medyan = [], []
        fps_gecmisi, gecerli_gecmisi, sicaklik_gecmisi = [], [], []
        pencere_no = 0
        derinlik_boyut = None

        while time.time() - t0 < a.sure:
            calisti = False

            r = q_r.tryGet()
            if r is not None:
                n_r += 1
                top_r += 1
                # getCvFrame() cv2 ister; venv'de yok → ham diziyi kendimiz çeviriyoruz.
                # preview + setInterleaved(False) → planar CHW (3,H,W), sıra BGR.
                f = r.getFrame()
                son_rgb = np.transpose(f, (1, 2, 0)) if f.ndim == 3 and f.shape[0] == 3 else f
                calisti = True

            if q_n is not None:
                nn = q_n.tryGet()
                if nn is not None:
                    n_n += len(nn.detections)
                    calisti = True

            d = q_d.tryGet()
            if d is not None:
                n_d += 1
                top_d += 1
                calisti = True
                dm = son_derinlik = d.getFrame()      # uint16, mm
                gp = dm > 0
                pen_gecerli.append(float(gp.mean() * 100))
                if gp.any():
                    pen_medyan.append(float(np.median(dm[gp])) / 1000)
                if derinlik_boyut is None:
                    derinlik_boyut = dm.shape
                    print(f"  derinlik kare boyutu: {dm.shape[1]}x{dm.shape[0]}"
                          f" ({dm.dtype}) → {dm.nbytes / 1e3:.0f} KB/kare,"
                          f" {dm.nbytes * a.fps / 1e6:.1f} MB/s\n")

                # ── PNG anlık görüntü (görsel doğrulama için) ──
                if time.time() - t0 - son_kayit >= a.kare_kaydet or son_kayit == 0:
                    son_kayit = time.time() - t0
                    etiket = f"{int(son_kayit):05d}sn"
                    png_yaz(os.path.join(kok, f"derinlik_{etiket}.png"),
                            derinlik_renklendir(dm, a.tavan))
                    if son_rgb is not None and son_rgb.ndim == 3:
                        png_yaz(os.path.join(kok, f"rgb_{etiket}.png"),
                                son_rgb[..., ::-1].copy())   # BGR→RGB

            if not calisti:
                time.sleep(0.001)

            # ── pencere kapanışı ──
            gecen = time.time() - pen_bas
            if gecen >= a.pencere:
                gecerli = sum(pen_gecerli) / len(pen_gecerli) if pen_gecerli else 0.0
                gecerli_en_dusuk = min(pen_gecerli) if pen_gecerli else 0.0
                gec_px = (son_derinlik[son_derinlik > 0]
                          if son_derinlik is not None else np.array([], np.uint16))
                fps_d = n_d / gecen
                fps_r = n_r / gecen
                sic = dev.getChipTemperature().average
                pencere_no += 1

                kayit = {
                    "pencere": pencere_no,
                    "t_sn": round(time.time() - t0, 1),
                    "fps_derinlik": round(fps_d, 2),
                    "fps_rgb": round(fps_r, 2),
                    "gecerli_piksel_yuzde": round(gecerli, 1),
                    "gecerli_piksel_en_dusuk_kare": round(gecerli_en_dusuk, 1),
                    "kare_sayisi": len(pen_gecerli),
                    "derinlik_medyan_m": round(float(np.median(gec_px)) / 1000, 2) if gec_px.size else None,
                    "derinlik_p10_m": round(float(np.percentile(gec_px, 10)) / 1000, 2) if gec_px.size else None,
                    "derinlik_p90_m": round(float(np.percentile(gec_px, 90)) / 1000, 2) if gec_px.size else None,
                    "vpu_C": round(sic, 1),
                    "tespit": n_n,
                    "usb": gercek_usb,
                }
                jsonl.write(json.dumps(kayit) + "\n")
                jsonl.flush()
                os.fsync(jsonl.fileno())          # kapanış çökmesine karşı

                med = kayit["derinlik_medyan_m"]
                print(f"  [{int(time.time() - t0):4d} sn] derinlik {fps_d:5.2f} FPS | "
                      f"RGB {fps_r:5.2f} FPS | geçerli piksel ort %{gecerli:4.1f} "
                      f"(en kötü kare %{gecerli_en_dusuk:4.1f}) | "
                      f"medyan {med if med is not None else '—':>5} m | "
                      f"VPU {sic:4.1f}°C" + (f" | {n_n} tespit" if q_n else ""))

                fps_gecmisi.append(fps_d)
                gecerli_gecmisi.append(gecerli)
                sicaklik_gecmisi.append(sic)
                n_d = n_r = n_n = 0
                pen_gecerli, pen_medyan = [], []
                pen_bas = time.time()

        sure = time.time() - t0

    # ── özet ──
    ozet = {
        "sure_sn": round(sure, 1),
        "istenen_fps": a.fps,
        "derinlik_fps_ort": round(top_d / sure, 2),
        "rgb_fps_ort": round(top_r / sure, 2),
        "derinlik_fps_min": round(min(fps_gecmisi), 2) if fps_gecmisi else None,
        "derinlik_fps_maks": round(max(fps_gecmisi), 2) if fps_gecmisi else None,
        "gecerli_piksel_ort": round(sum(gecerli_gecmisi) / len(gecerli_gecmisi), 1) if gecerli_gecmisi else None,
        "gecerli_piksel_min": round(min(gecerli_gecmisi), 1) if gecerli_gecmisi else None,
        "vpu_bas_C": round(sicaklik_gecmisi[0], 1) if sicaklik_gecmisi else None,
        "vpu_son_C": round(sicaklik_gecmisi[-1], 1) if sicaklik_gecmisi else None,
        "vpu_tepe_C": round(max(sicaklik_gecmisi), 1) if sicaklik_gecmisi else None,
        "usb": gercek_usb,
        "preset": a.preset,
    }
    jsonl.write(json.dumps({"OZET": ozet}) + "\n")
    jsonl.close()

    print("\n" + "=" * 70)
    print("  ÖZET")
    print("=" * 70)
    print(f"  Süre                  : {ozet['sure_sn']:.0f} sn")
    print(f"  Derinlik FPS          : ort {ozet['derinlik_fps_ort']} "
          f"(min {ozet['derinlik_fps_min']} / maks {ozet['derinlik_fps_maks']}) "
          f"— istenen {a.fps:.0f}")
    print(f"  RGB FPS               : ort {ozet['rgb_fps_ort']}")
    print(f"  Geçerli derinlik piks.: ort %{ozet['gecerli_piksel_ort']} "
          f"(en düşük %{ozet['gecerli_piksel_min']})")
    print(f"  VPU sıcaklığı         : {ozet['vpu_bas_C']}°C → {ozet['vpu_son_C']}°C "
          f"(tepe {ozet['vpu_tepe_C']}°C)")
    print(f"  USB linki             : {ozet['usb']}")
    print(f"\n  Ham ölçüm + PNG'ler   : {kok}")
    print("=" * 70)


if __name__ == "__main__":
    sys.exit(main())

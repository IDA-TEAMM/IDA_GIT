#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Masa testi (ROS'suz): YOLO11n OAK-D Lite VPU'sunda çalışır,
tespitleri kutu + sınıf + X/Z (m) + bearing ile ekrana çizer.
Suya girmeden önce sınıf indekslerini ve mesafeleri burada doğrula.
Çıkış: q

depthai **v2 (2.30.0.0)** — 2026-08-05'te v3'ten taşındı. Pipeline, deploy
node'unun (duba_gecis_navigator.pipeline_kur) AYNASIDIR: 12MP+ispScale(1,3)
RGB (tam 4:3 FOV), 480P monolar, ROBOTICS, setOutputSize(640,400), LRC,
anchor'sız YOLO. Deploy değişirse BURASI DA değişmeli — bu scriptin amacı
deploy'da ne koşacaksa ONU masada göstermek.
"""
import json
import math
import os
import time

import cv2
import depthai as dai

MODEL_BLOB = "/home/girdap/models/yolo11n_duba_rvc2.blob"
FPS = 11          # asıl node ile aynı; ekrandaki ölçüm ~11 olmalı (tavan 12,2:
                  # YOLO 416x416 + stereo birlikte, 2026-08-05 ölçümü)
CONF_ESIK = 0.5

ETIKET = {0: "KENAR", 1: "ENGEL"}                      # yedek (config.json yoksa)
RENK = {0: (0, 140, 255), 1: (0, 230, 255)}            # BGR: turuncu / sarı


def siniflari_oku(blob_yolu):
    """Blob yanındaki config.json'dan (NNArchive formatı) sınıf isimleri."""
    for yol in (os.path.join(os.path.dirname(blob_yolu), "config.json"),
                os.path.splitext(blob_yolu)[0] + ".json"):
        try:
            with open(yol, encoding="utf-8") as f:
                cfg = json.load(f)
            return [str(s) for s in
                    cfg["model"]["heads"][0]["metadata"]["classes"]]
        except (OSError, ValueError, KeyError, IndexError):
            continue
    return []


siniflar = siniflari_oku(MODEL_BLOB)

pipeline = dai.Pipeline()

cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
cam_rgb.setIspScale(1, 3)                    # 1352×1014 — tam 4:3, kırpmasız
cam_rgb.setPreviewSize(416, 416)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam_rgb.setPreviewKeepAspectRatio(False)     # tam kare sıkıştırılır (deploy ile aynı)
cam_rgb.setFps(FPS)

mono_sol = pipeline.create(dai.node.MonoCamera)
mono_sag = pipeline.create(dai.node.MonoCamera)
mono_sol.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_sag.setBoardSocket(dai.CameraBoardSocket.CAM_C)
for m in (mono_sol, mono_sag):
    m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    m.setFps(FPS)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(640, 400)               # ZORUNLU: yoksa FPS 8'e düşer (ölçüldü)
stereo.setLeftRightCheck(True)
mono_sol.out.link(stereo.left)
mono_sag.out.link(stereo.right)

sdn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
sdn.setBlobPath(MODEL_BLOB)
sdn.setConfidenceThreshold(CONF_ESIK)
sdn.setNumClasses(len(siniflar) if siniflar else 2)
sdn.setCoordinateSize(4)
sdn.setIouThreshold(0.5)
# YOLOv11 anchor-free → setAnchors çağrılmaz
sdn.input.setBlocking(False)
sdn.setBoundingBoxScaleFactor(0.5)
sdn.setDepthLowerThreshold(300)
sdn.setDepthUpperThreshold(10000)            # deploy ile aynı (8 m bandı + pay)
cam_rgb.preview.link(sdn.input)
stereo.depth.link(sdn.inputDepth)

xo_det = pipeline.create(dai.node.XLinkOut); xo_det.setStreamName("tespit")
sdn.out.link(xo_det.input)
xo_rgb = pipeline.create(dai.node.XLinkOut); xo_rgb.setStreamName("rgb")
sdn.passthrough.link(xo_rgb.input)

# USB2'ye zorla (tegra-xusb SuperSpeed arızası — 5/5 açılış ölçümü)
with dai.Device(pipeline, dai.UsbSpeed.HIGH) as dev:
    det_q = dev.getOutputQueue("tespit", maxSize=4, blocking=False)
    rgb_q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)

    print("Pipeline başladı — YOLO VPU'da. q ile çık.")
    print("USB:", dev.getUsbSpeed(), "| Model sınıf sırası:", siniflar or "(config.json yok → yedek)")

    dets = []
    t0, sayac, fps_txt = time.time(), 0, "..."

    while not dev.isClosed():
        d_msg = det_q.tryGet()
        if d_msg is not None:
            dets = d_msg.detections
            sayac += 1
            if time.time() - t0 >= 1.0:
                fps_txt = f"{sayac / (time.time() - t0):.1f} FPS"
                t0, sayac = time.time(), 0

        f_msg = rgb_q.tryGet()
        if f_msg is None:
            continue
        frame = f_msg.getCvFrame()
        h, w = frame.shape[:2]

        for d in dets:
            if d.confidence < CONF_ESIK:
                continue
            x1, y1 = int(d.xmin * w), int(d.ymin * h)
            x2, y2 = int(d.xmax * w), int(d.ymax * h)
            renk = RENK.get(d.label, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), renk, 2)

            X = d.spatialCoordinates.x / 1000.0
            Z = d.spatialCoordinates.z / 1000.0
            bearing = math.degrees(math.atan2(X, Z)) if Z > 0 else 0.0
            ad = (siniflar[d.label] if d.label < len(siniflar)
                  else ETIKET.get(d.label, str(d.label)))
            cv2.putText(frame, f"{ad} {d.confidence:.2f}", (x1, y1 - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2)
            cv2.putText(frame, f"X:{X:+.2f}m Z:{Z:.2f}m {bearing:+.0f}deg",
                        (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, renk, 1)

        cv2.putText(frame, fps_txt, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("GIRDAP duba testi", frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()

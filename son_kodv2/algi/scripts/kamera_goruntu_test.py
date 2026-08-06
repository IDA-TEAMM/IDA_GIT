#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
⚠️⚠️ GEÇİCİ SCRIPT — MODEL GELİNCE SİLİNECEK ⚠️⚠️

NN Archive (yolo11n_duba_rvc2.tar.xz) henüz üretilmediği için OAK-D Lite'ı
MODELSİZ test eder: yalnız RGB görüntü akışı + canlı FPS. Kamera/USB/udev
zincirinin sağlığını bugün doğrulamak için var — tespit/YOLO YOK.

Model dosyası Jetson'a konduğu gün ASIL test scripts/duba_kamera_test.py'dir
ve bu dosya KARIŞIKLIK OLMASIN diye SİLİNİR (docs/bekleyen_girdiler.md §B'ye
işlendi). Bu scripti yarışma yazılımının parçası SANMA.

Kullanım:  python3 scripts/kamera_goruntu_test.py     (çıkış: q)
Beklenen:  canlı görüntü, ~11 FPS (sensorFps=11), 640×480 pencere.
"""
import time

import cv2
import depthai as dai

FPS = 11                     # asıl node ile aynı (VPU bandı karşılaştırılabilir)

# depthai v2 (2.30.0.0) — 05.08'de v3'ten taşındı. Deploy ile aynı RGB yolu:
# 12MP + ispScale(1,3) = 1352×1014 tam 4:3 (THE_1440X1080 bu cihazda sessizce
# 0 kare üretiyor — 05.08 mod taraması). Önizleme 640×480'e sıkıştırılır.
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
cam_rgb.setIspScale(1, 3)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setPreviewKeepAspectRatio(False)
cam_rgb.setFps(FPS)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam_rgb.preview.link(xout.input)

# USB2'ye zorla (tegra-xusb SuperSpeed arızası — 5/5 açılış ölçümü)
with dai.Device(pipeline, dai.UsbSpeed.HIGH) as dev:
    rgb_q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
    print("GEÇİCİ kamera testi (MODELSİZ) — görüntü + FPS. q ile çık.")

    t0, sayac, fps_txt = time.time(), 0, "..."
    while not dev.isClosed():
        msg = rgb_q.tryGet()
        if msg is None:
            continue
        frame = msg.getCvFrame()
        sayac += 1
        if time.time() - t0 >= 1.0:
            fps_txt = f"{sayac / (time.time() - t0):.1f} FPS"
            t0, sayac = time.time(), 0
        cv2.putText(frame, f"{fps_txt}  (GECICI TEST - YOLO YOK)", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("kamera_goruntu_test (GECICI)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()

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
Beklenen:  canlı görüntü, ~12 FPS (sensorFps=12), 640×480 pencere.
"""
import time

import cv2
import depthai as dai

FPS = 12                     # asıl node ile aynı (VPU bandı karşılaştırılabilir)

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.Camera).build(
    dai.CameraBoardSocket.CAM_A, sensorFps=FPS
)
rgb_q = cam_rgb.requestOutput((640, 480)).createOutputQueue(
    maxSize=4, blocking=False
)

pipeline.start()
print("GEÇİCİ kamera testi (MODELSİZ) — görüntü + FPS. q ile çık.")

t0, sayac, fps_txt = time.time(), 0, "..."
while pipeline.isRunning():
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

pipeline.stop()
cv2.destroyAllWindows()

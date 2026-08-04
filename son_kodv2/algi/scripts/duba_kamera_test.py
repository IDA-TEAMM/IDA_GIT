#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Masa testi (ROS'suz): YOLO11n OAK-D Lite VPU'sunda çalışır,
tespitleri kutu + sınıf + X/Z (m) + bearing ile ekrana çizer.
Suya girmeden önce sınıf indekslerini ve mesafeleri burada doğrula.
Çıkış: q
"""
import math
import time

import cv2
import depthai as dai

MODEL_NNARCHIVE = "/home/girdap/models/yolo11n_duba_rvc2.tar.xz"
FPS = 12          # ekrandaki ölçüm 10-14 bandında olmalı (YOLO 416x416 + stereo)
CONF_ESIK = 0.5

ETIKET = {0: "KENAR", 1: "ENGEL"}                      # data.yaml sırana göre!
RENK = {0: (0, 140, 255), 1: (0, 230, 255)}            # BGR: turuncu / sarı

nn_archive = dai.NNArchive(MODEL_NNARCHIVE)
pipeline = dai.Pipeline()

cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=FPS)
mono_sol = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
mono_sag = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
mono_sol.requestOutput((640, 400)).link(stereo.left)
mono_sag.requestOutput((640, 400)).link(stereo.right)

sdn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
    cam_rgb, stereo, nn_archive,
    resizeMode=dai.ImgResizeMode.LETTERBOX)  # tam yatay FOV
sdn.input.setBlocking(False)
sdn.setBoundingBoxScaleFactor(0.5)
sdn.setDepthLowerThreshold(300)
sdn.setDepthUpperThreshold(20000)
sdn.setConfidenceThreshold(CONF_ESIK)

rgb_q = sdn.passthrough.createOutputQueue(maxSize=4, blocking=False)
det_q = sdn.out.createOutputQueue(maxSize=4, blocking=False)

pipeline.start()
print("Pipeline başladı — YOLO VPU'da. q ile çık.")
print("Model sınıf sırası (data.yaml):", sdn.getClasses())

dets = []
t0, sayac, fps_txt = time.time(), 0, "..."

while pipeline.isRunning():
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
        ad = getattr(d, "labelName", "") or ETIKET.get(d.label, str(d.label))
        cv2.putText(frame, f"{ad} {d.confidence:.2f}", (x1, y1 - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2)
        cv2.putText(frame, f"X:{X:+.2f}m Z:{Z:.2f}m {bearing:+.0f}deg",
                    (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, renk, 1)

    cv2.putText(frame, fps_txt, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("GIRDAP duba testi", frame)
    if cv2.waitKey(1) == ord("q"):
        break

pipeline.stop()
cv2.destroyAllWindows()

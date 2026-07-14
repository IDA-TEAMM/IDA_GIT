---
name: ida-oak-d-lite
description: "OAK-D Lite'ı ROS 2'de topic yayınlatma — udev + USB2/RGB-only config şart"
metadata: 
  node_type: memory
  type: project
  originSessionId: a70a49cc-c538-46e5-8247-4e8e9aa9b1d1
---

İDA topic testinde OAK-D Lite'ı `depthai_ros_driver` (ros-humble-depthai-ros-driver 2.12.2, apt) ile yayınlattık. İki tuzak:

1. **udev kuralı şart:** `/etc/udev/rules.d/80-movidius.rules` → `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`; yoksa "Insufficient permissions ... X_LINK_UNBOOTED". Kural sonrası çıkar-tak. (sudo gerektiği için betik `~/oak_udev.sh` ile kullanıcı çalıştırdı.)

2. **Varsayılan config X_LINK_ERROR veriyor:** Stok `camera.yaml` OAK-D **Lite'ta olmayan IR**'ı (`i_enable_ir`) ve ağır **RGBD** pipeline'ını USB3-SUPER'de açınca boot sonrası `sys_logger_queue X_LINK_ERROR` ile kopuyor. Çözüm: hafif params ile başlat →
   `camera.i_pipeline_type: RGB`, `camera.i_usb_speed: HIGH` (USB2), `i_enable_imu: false`, `i_enable_ir: false`. Bu config'le `/oak/rgb/image_raw` ~21.6 Hz stabil aktı.
   Params dosyası: scratchpad `oak_lite.yaml`.

Uyarılar: Bu config'te depth (`/oak/stereo/depth`) YOK, sadece RGB (`camera_buoys` RGB kullanıyor, sorun değil). `usbfs_memory_mb=16` (varsayılan, düşük) — depth/yüksek çözünürlük açılırsa 256'ya çıkarmak gerekebilir. Kurulu python `depthai` 3.7.1 (v3 API, XLinkOut yok) ama ROS sürücüsü kendi depthai-core v2'sini kullanıyor, bağımsız.

Bkz [[ida-hardware]], [[ida-software-status]].

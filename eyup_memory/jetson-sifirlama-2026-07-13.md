---
name: jetson-sifirlama-2026-07-13
description: "Jetson görev PC'si 2026-07-13'te sıfırdan kuruldu — yeni temiz taban, ne silindi/ne korundu, doğrulama sonuçları"
metadata: 
  node_type: memory
  type: project
  originSessionId: 800efebd-d9e6-4774-9760-fef20493312e
---

**Jetson 2026-07-13 akşamı Eyüp'ün isteğiyle SIFIRLANDI ve baştan kuruldu. Yeni durum = TEK GERÇEK; eski kurulum notları (çift opencv, torch'lu ortam) BAYAT.**

## Silinen (geri gelmez)
- Eski `~/ros2_ws`, `~/girdap_ws.eski`, `~/ws_livox`, `~/Livox-SDK2` (kaynak), `~/.local/lib/python3.10` (5.9 GB pip: torch, ultralytics, onnxruntime-gpu, opencv-python NON-headless, **depthai-ros-driver** — F3.1 cihaz-çalma riskiydi, bilinçli geri KURULMADI), ev dizinindeki başıboş dosyalar.

## Korunan / yedeklenen
- `~/girdap_logs/` (masa+test günü ham logları) olduğu gibi duruyor.
- `~/Desktop/oakdlite/` — YOLO model dosyaları (stok COCO 416x416yolov11n.tar.xz, .pt/.onnx/.engine) hiçbir repoda yok, elle taşınmıştı — SİLİNMEDİ.
- `~/yedek_sifirlama_2026-07-13/`: eski `pip freeze` (380 paket), **Livox MID360_config.json (LiDAR IP=192.168.117.100, host=192.168.117.50 — lokal değişiklikti, repoda YOK)**, lidar_bench_gecici.py, check_girdap.sh.
- `~/.bashrc`, git kimlik/credentials, `~/.claude`, `/usr/local`'daki Livox SDK (sistem), apt katmanı (ROS Humble, mavros, vision_msgs), OAK udev kuralı, rfkill WiFi/BT kapalı.

## Yeni kurulum (doğrulanmış)
- Klonlar: `~/ros2_ws/src/{girdap-ida-algi, girdap-decision}` (ikisi de origin ile senkron, girdap-decision `15dc238` = girdap-video'daki dondurulmuş kod ile AYNI commit), `~/ws_livox/src/livox_ros_driver2` (IP config geri kondu, colcon ile derlendi), `~/girdap-video` (yeni video repo'su).
- pip (yalın, --user): numpy==1.26.4 + opencv-python-headless==4.11.0.86 (tek komut), depthai==3.7.1, gtsam==4.3a0, scipy==1.13.1, matplotlib==3.10.9, cupy-cuda12x==13.6.0, pytest==6.2.5. Torch/ultralytics YOK (YOLO OAK VPU'da; gerekirse yedekteki freeze'den).
- colcon build: girdap_ida_algi + girdap_decision + livox_ros_driver2 ✅.
- **DOĞRULAMA: girdap-decision suite Jetson YENİ TABAN 265 passed / 2 skipped** (PC 262/5'ten iyi, cupy'liler koşuyor); `jetson_kontrol.sh` tek HATA = duba NN Archive (`~/models/yolo11n_duba_rvc2.tar.xz`) — zaten hiç gelmemişti ([[yolo-model-durumu]]); OAK takılı değil (donanım sökük, normal); **girdap-video `testler/video_testleri.sh --tam` = 12/12 BİLEŞEN YEŞİL Jetson'da** (kontrol-listesi.md madde 0 ✅).
- girdap-ida-algi'de unit suite YOK (eski "246/2" notu girdap-decision'ındı); kökten `pytest` koşma — `scripts/duba_kamera_test.py` import'ta model arar, collection patlar.

## systemd servisi KURULDU (2026-07-13 akşam, Eyüp sudo'yla)
- **`girdap-karar.service` enabled+running:** boot'ta `hardware.launch.py mission_source:=fc` otomatik kalkar; operatör yalnız QGC kullanır (Upload→ARM→GUIDED). Dosya: `girdap-decision/scripts/girdap-karar.service` (repoya yazıldı, HENÜZ COMMIT'LENMEDİ). `Environment=ROS_DOMAIN_ID=42` ŞART — bashrc'deki 42 non-interactive shell'e geçmiyor (erken return), ilk kurulumda servis domain 0'da kaldı, `ros2 node list` boş göründü; düzeltme sonrası tüm node'lar Eyüp'ün terminalinden görünür TEYİTLİ. FCU'suz beklenen davranış: heartbeat-KILL basıp BEKLEMEDE'de oturur, zararsız.
- **Masa testi kuralı:** elle `ros2 launch` yapmadan önce `sudo systemctl stop girdap-karar` (yoksa çift node/çift yayın). Algı servisi (`girdap-algi.service`) AYRI ve kurulmadı (videoda algı gerekmiyor).

## Açık kalanlar
- ~~girdap-decision origin çift-push geri kurulacak~~ **İPTAL (2026-07-14):** Eyüp kararıyla fork+yedek repoları artık kullanılmayacak; video işi yalnız girdap-video — bkz. [[video-repo-tek-kaynak]].
- FC OLAY aksiyonları hâlâ bekliyor (mission clear + BRD_SAFETY_DEFLT=1 + RC mod kanalı — [[donanim-test-plani]]); ilgili yeni dokümanlar repoda: `docs/fc_parametre_onerileri.md`, `docs/video_denetimi.md`.
- F-M.1 ve F-M.2 upstream'de DÜZELTİLDİ (`dff52af`, `3931220`) — donanım-test-planı memory'sindeki "fix'siz görev başlatma reddi = sonraki oturum 1. işi" maddesi KAPANDI.

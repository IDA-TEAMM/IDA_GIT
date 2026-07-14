---
name: ida-sim-workspace
description: "GİRDAP İDA Gazebo simülasyon workspace'inin konumu, kurulu yığını ve bilinen kurulum/yol sorunları"
metadata: 
  node_type: memory
  type: project
  originSessionId: b947f7ad-b0f6-4a34-b949-cbbe03fd8065
---

**Workspace konumu:** `~/Desktop/girdap_ida_ws` (tek paket: `src/girdap_ida_sim`, ament_cmake). NOT: yazılım analiz raporu workspace'i `~/ida_ws` diye anar — yol tutarsızlığı var.

**Kurulu yığın (doğrulandı, 2026-06-22):** ROS 2 Humble desktop, Gazebo Harmonic (gz-sim8 8.14.0, `gz sim`), `ros-humble-ros-gzharmonic` + bridge + image + sim, libgtsam-dev. Hepsi Ubuntu 22.04 (jammy).

**Paket içeriği:** `launch/` (simulation.launch.py = ocean_world.sdf; parkur1.launch.py = parkur1_world.sdf), `worlds/` (ocean_world.sdf, parkur1_world.sdf), `urdf/` (girdap_ida.urdf.xacro, girdap_sonida.urdf.xacro), `config/ros_gz_bridge.yaml`, `scripts/usv_diff_drive.py` (/cmd_vel→sol/sağ thrust), `plugins/` (libWaves.so, libWaveVisual.so, libSurface.so, libSimpleHydrodynamics.so — asv_wave_sim/gz-waves), `meshes/` STL'ler, `models/` (ocean_waves, turuncu_buoy).

**Launch akışı:** gz sim (duraklı başlar, -r yok) → 8s spawn (ros_gz_sim create, -topic /robot_description) → 12s ros_gz_bridge → 13s usv_diff_drive. Ortam değişkenleri launch içinde set ediliyor: GZ_IP=127.0.0.1, GZ_SIM_RESOURCE_PATH (pkg parent + models + sistem), GZ_SIM_SYSTEM_PLUGIN_PATH + LD_LIBRARY_PATH (plugins/).

**SORUN DURUMU:**
1. ✅ DÜZELTİLDİ (2026-06-22): `run_sim.sh` artık WS_DIR'i script konumundan çözüyor + `/opt/ros/humble/setup.bash` source ediyor + chmod +x verildi.
2. ✅ DÜZELTİLDİ: launch dosyalarında GZ_SIM_RESOURCE_PATH varsayılanı `/opt/ros/jazzy/share` → `/opt/ros/humble/share` (her iki launch).
3. ⏳ AÇIK: `girdap_ida_sim` paketi install'a HİÇ build edilmemiş. `build/install/log` ev dizininde (stray, yanlış cwd'den kalma) — `~/build` & `~/log`'da COLCON_IGNORE var, `~/install`'da paket yok. Doğru kullanım: `cd ~/Desktop/girdap_ida_ws && colcon build`. (Sonraki adım: 2 = build.)
4. ⏳ AÇIK: Rapora göre simülasyon crash'leri: ocean/ida world'de WavesModel eksik (su yüzeyi fiziği yok) ve OGRE2 `Ogre2DisplacementMap.cc` assertion crash (render çöküyor). gz-waves sürüm uyumu şüpheli; B planı düz su yüzeyiyle başlamak. (Adım 3'te ele alınacak.)

**YENİ KURULUM (2026-06-28) — `~/girdap_ws`:** Eski `~/Desktop/girdap_ida_ws`'ten ayrı, temiz bir dalga ortamı kuruldu (kullanıcının indirdiği `GIRDAP_Gazebo_Kurulum.docx` kılavuzuna göre).
- `asv_wave_sim` (srmainwaring) `~/asv_wave_sim`'e klonlandı, `gz-waves` derlenip `sudo make install` ile `/usr/local/lib`'e kuruldu (7 lib: libgz-waves1*.so). Eksik bağımlılıklar elle eklendi: `libcgal-dev`, `libfftw3-dev` (kılavuzun listesinde fftw3 YOKTU).
- `.bashrc`'ye eklendi: `LD_LIBRARY_PATH=/usr/local/lib`, `GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib`, `GZ_SIM_RESOURCE_PATH=~/asv_wave_sim/gz-waves-models/{models,world_models}`.
- `~/girdap_ws/{worlds,models,launch}` oluşturuldu; `worlds/girdap_deniz.sdf` = sade deniz dünyası (physics+sensors(ogre2)+imu+buoyancy(1025)+`model://waves` include; GPS ref Mersin 36.8N/34.6E; debug eksenleri). İDA aracı modeli HENÜZ YOK.
- Açma: `gz sim ~/girdap_ws/worlds/girdap_deniz.sdf` (yeni terminalde env hazır). Headless test temiz (WavesModel+Wavefield yükleniyor, hata yok). GUI açıldı/çalıştı; sadece pencere KAPANIRKEN OGRE2 HlmsPbsDatablock teardown abort'u var (zararsız). → eski rapor issue #4 (gz-waves çökmesi) bu temiz kurulumla pratikte aşıldı.
- AÇIK: kullanıcı "2 SDF indirdim" dedi ama Downloads'ta tek SDF (girdap_deniz) var — ikinci SDF (muhtemelen İDA aracının kendi modeli?) bulunamadı.

**GEMİ MODELİ + ÇÖKME ÇÖZÜMÜ (2026-06-28):** İDA gemisi SolidWorks URDF export'u (`Girdap`, katamaran: base_link gövde+2 ponton + sabit eklemli Lidar/GPS/Kamera/L-R Thruster — hepsi base_link'e lump). `~/girdap_ws/models/Girdap`'a kuruldu, `worlds/girdap_deniz.sdf` `model://Girdap` include eder.
- ÇÖKME SEBEBİ ÇÖZÜLDÜ: İlk RAR'daki 5 STL (base_link hariç) BOZUKTU (binary STL üçgen-sayısı=0, transfer/RAR hasarı) → OGRE2 `StagingBuffer cannot map 0 bytes` → GUI çökmesi. Yeniden export + ZIP (RAR DEĞİL) ile 6 mesh de geçerli geldi. Ders: mesh'leri ZIP'le aktar, RAR bozuyor. Bütünlük testi: `(boyut-84)/50` tam sayı olmalı.
- FİZİK MOTORU: DART mesh-collision desteklemiyor (`not implemented for dartsim`). Çözüm: **Bullet** (`--physics-engine gz-physics-bullet-featherstone-plugin`) — mesh collision'ı convex hull olarak kabul ediyor, model değişmeden.
- YÜZME: dünya `uniform_fluid_density` buoyancy gemiyi gökyüzüne fırlatıyor. Doğrusu asv_wave_sim **Hydrodynamics plugin** (`gz-waves1-hydrodynamics-system`, wam-v örneği referans) + uniform buoyancy KALDIR. base_link watertight ama **2286 non-manifold iç yüzey** var → hidrodinamik hacmi şaşırıp tekneyi savuruyor; trimesh ile temizlenecek + katsayılar İDA'ya (8.8 kg, 0.78×1.04×0.40 m) ölçeklenecek. (DEVAM EDİYOR)
- World varyantları: girdap_deniz.sdf (gemili), _gemisiz, _baseonly, _hydro. Açma env'i launch/girdap.launch.py içinde + NVIDIA offload (ama dri2/EGL uyarısı zararsız çıktı).

İlgili: [[ida-software-status]] · [[ida-project]]

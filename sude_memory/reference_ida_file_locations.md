---
name: reference-ida-file-locations
description: "Where IDA USV project files, backups, and docs live outside this repo"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 0177bfd4-f55e-432e-8505-9f5599a8723d
---

- **GitHub (primary backup):** `https://github.com/IDA-TEAMM/IDA_GIT`, branch `main`. Node sources live at `ida_topics_yeni/ida_topics/` (11 current files) and `sistem_baslat.sh` at `ida_topics_yeni/sistem_baslat.sh`. Legacy 4 nodes at `ida_topics/ida_topics/` (do not touch). Push pattern used previously: `git push --force https://snnazz:TOKEN@github.com/IDA-TEAMM/IDA_GIT.git sude-feature-v2:main` (force-push to main from feature branch — confirm with user before ever repeating this).
- **Host Desktop (`~/Masaüstü/`):** `ros2_baslat_v5.sh` (container launcher — updated 2026-07-13: now uses image `ros2_harmonic_final4`, live `-v /dev:/dev` mount instead of static `--device` entries, corrected `sistem_baslat.sh` path in its help text), `sistem_baslat.sh` (node launcher copy), plain-named node `.py` copies (`gps_imu_driver_node.py`, `oakd_driver_node.py`, `livox_driver_node.py`, `kamera_kayit_node.py`, `local_map_node.py`, `telemetri_node.py`, `control_node_mavros.py` — see [[feedback-ida-three-location-sync]]), `girdap_GUNCEL_20260625.sdf` + `p_GUNCEL_20260625.world` (Gazebo model/world backups), `decision_node_GUNCEL_20260625.py`/`perception_node_GUNCEL_20260625.py`/`sensor_node_GUNCEL_20260625.py` (kept as backups), `QGroundControl.AppImage` (v5.0.8), `MicoAssistant_EN/` (telemetry radio config tool, runs via Wine).
- **`~/Masaüstü/IDA_YAZILIM/`** — curated backup tree: `nodes/` (11 node .py files), `scripts/` (both shell scripts), `yolo/best.pt` (train7, mAP50=0.984), `config/config_ekf.yaml`, `gazebo/` (SDF/world/pose_bridge backups), `docs/IDA_ROS2_Topic_Dokumantasyonu.docx` + `IDA_Yazilim_Hiyerarsi.docx`.
- **Container-only paths (lost if `docker rm`, not in git):** `/root/ardupilot/` (ArduPilot SITL), `/root/gz_ws/` (Gazebo plugin ws), `/root/.gazebo/models/girdap/` (sim model), `/root/p.world`, `/root/best.pt`.
- **`~/Masaüstü/ida_son_yazılım/`** — full clone of the captain's (Yahya Seha Danış) `girdap-kaptan-video` GitHub repo (`https://github.com/yahyaseha/girdap-kaptan-video`), copied here 2026-07-14 for review. Contains `.git`, so `git pull` works directly in place. This is the **parallel `girdap_decision` codebase**, not `ida_topics` — see [[project_ida_captain_decision_repo]] for what's in it.
- **`~/Masaüstü/eyup_memory/`** — full clone of teammate EyupEker1's Claude memory export repo (`https://github.com/EyupEker1/memory`), copied here 2026-07-14. 25 files including `girdap-decision-entegrasyon.md` (79KB, largest), `donanim-test-plani.md` (23KB), `girdap-ida-proje-durumu.md`, `sartname-ida-2026.md` — not yet read in depth, worth checking for project state not otherwise captured in this project's own memory. Contains `.git`, `git pull` works in place.
- **`eyup_memory/` at repo root** (both `main` and `sude-feature-v2`, commit `72a8546`) — same EyupEker1/memory content, minus `.git`, committed into `IDA_GIT` top-level alongside `son_kod/` (same pattern/precedent).
- **Source of the project-context memories** ([[project-ida-teknofest-overview]] etc.): `~/İndirilenler/IDA_Tam_Ozet.docx`, a session-summary doc, scanned/extracted 2026-07-12.

**How to apply:** If asked to find the YOLO model, Gazebo world, or old decision node, point to these locations rather than searching this repo — they're intentionally kept outside `ros2_ws`.

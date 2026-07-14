---
name: project-ida-teknofest-overview
description: "IDA/Girdap USV project context — TEKNOFEST 2026 competition, team, deadlines, current phase"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0177bfd4-f55e-432e-8505-9f5599a8723d
---

This repo (`~/ros2_ws`) is the ROS2 workspace for **IDA/Girdap USV**, an unmanned surface vehicle built for **TEKNOFEST 2026**, Takım 989124, Alt Alan B. The active package is `src/ida_topics_yeni` (11 nodes); `src/ida_topics/ida_topics/` holds 4 legacy nodes that should not be touched. `src/girdap_yenimodel` is the Gazebo simulation model package.

**Why:** As of 2026-07 the project moved from Gazebo simulation to real hardware bring-up. A prior chat session (documented in `IDA_Tam_Ozet.docx`, summarized 2026-07) wrote/updated the driver and node layer for real sensors (F9P GPS, OAK-D Lite, Livox Mid-360) and fixed several MAVROS/QoS integration bugs.

**Key dates:**
- Video teslimi (submission deadline): 21 Temmuz 2026, 17:00 KYS — YouTube, min 720p, 2–5 dk.

**Current status (as of 2026-07-12):** Software tests pass in the Docker container (all 11 nodes start, syntax-clean, telemetry CSV/MP4/PGM outputs verified, QoS warnings resolved). Hardware bring-up is in progress and NOT yet complete: Pixhawk/QGroundControl connection, telemetry radio QGC pairing, and real-hardware tests for GPS, OAK-D, Livox, and motors/ESC are all still pending (⏳ in the source doc).

**How to apply:** When working in this repo, assume the target is real hardware (Pixhawk/ArduPilot boat, F9P GPS, OAK-D Lite, Livox Mid-360) via a Jetson + laptop network, not the Gazebo sim (sim is now just a reference in `src/girdap_yenimodel`). See [[project-ida-docker-env]], [[project-ida-hardware-network]], [[project-ida-nodes-and-tests]], [[feedback-ida-critical-rules]], [[reference-ida-file-locations]].

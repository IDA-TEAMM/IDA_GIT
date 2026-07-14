---
name: project-ida-hardware-network
description: IDA USV hardware port assignments and laptop↔Jetson network configuration
metadata: 
  node_type: memory
  type: project
  originSessionId: 0177bfd4-f55e-432e-8505-9f5599a8723d
---

**Hardware ports:**
- Pixhawk (MAVROS): `/dev/ttyUSB0`, 57600 baud (serial). Falls back to `udp://127.0.0.1:14550@` test mode if absent.
- Telemetry radio: also `/dev/ttyUSB0` on the ground-station computer side, 57600 baud.
- F9P GPS: `/dev/ttyACM0` (primary) or `/dev/ttyUSB1` (alternate), 115200 baud, NMEA ($GNGGA/$GNRMC).
- Livox Mid-360 LiDAR: Ethernet, device IP `192.168.1.100`, UDP data port `56100`.
- OAK-D Lite: USB, detected via DepthAI SDK.

**Network (laptop ↔ Jetson):**
- OVS bridge `ida-bridge` on Fedora laptop, `192.168.10.1/24`, persistent via `/etc/systemd/network/ida-bridge.netdev`+`.network`, `systemd-networkd`.
- Laptop Ethernet iface `enp3s0` → `192.168.117.1/24` (persistent config).
- Jetson static IP `192.168.117.50/24` on `enP8p1s0`, user `girdap`. SSH: `ssh girdap@192.168.117.50`.
- `ROS_DOMAIN_ID=42` is set in laptop `~/.bashrc`, Jetson `~/.bashrc`, AND in `sistem_baslat.sh` (must be exported before `source /opt/ros/humble/setup.bash`). All three must stay in sync for Jetson↔laptop topic discovery to work.
- Docker container runs with `--network host`, sharing the host's network namespace (required for this cross-machine ROS2 discovery to reach the container).

**Why:** Confirmed working as of 2026-07 — `ros2 topic list` from the Jetson shows all laptop topics, ping works both directions. If this breaks, check ROS_DOMAIN_ID consistency and the `--network host` flag first before assuming a code bug.

**How to apply:** Any driver node parameter defaults (ports, IPs) should match this table unless the user says hardware changed. See [[feedback-ida-critical-rules]] for the Livox `host_ip` gotcha specifically.

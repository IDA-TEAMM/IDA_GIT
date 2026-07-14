---
name: feedback-ida-critical-rules
description: "Hard-won gotchas in the IDA USV codebase — do not \"fix\" these back"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0177bfd4-f55e-432e-8505-9f5599a8723d
---

These were each debugged in a prior session and are easy to accidentally "correct" back into a bug. Preserve them as-is:

- **MAVROS message class name is `VfrHud`, not `VFR_HUD`.** Using `VFR_HUD` causes a Python import error. Applies to `telemetri_node.py` and anywhere else `mavros_msgs.msg` is imported for `/mavros/vfr_hud`.
- **`cmd.angular.z = -angular` in `decision_node.py` is intentional sign inversion**, not a bug — matches ArduPilot's yaw convention. Do not remove the negation.
- **All MAVROS topic subscriptions need `QoSProfile(reliability=BEST_EFFORT)` explicitly.** MAVROS publishes BEST_EFFORT; the rclpy default subscriber QoS (RELIABLE) causes silent non-delivery / "incompatible QoS" warnings, not an error, so it's easy to miss. This bit `decision_node.py`, `telemetri_node.py`, and `local_map_node.py` (for `/lidar/scan`) already — check any new subscriber to `/mavros/*` or `/lidar/scan` for this.
- **Livox driver `host_ip` parameter must be `'0.0.0.0'`, not a specific interface IP.** A specific IP causes a UDP socket bind error with the Mid-360.
- **`sistem_baslat.sh` must `cd /root/ros2_ws/src/ida_topics_yeni` (container path)** before launching nodes with relative `ida_topics/*.py` paths — it runs inside the Docker container, not on the host, so host-side absolute paths are wrong here.
- **Guard optional serial ports before use**: F9P GPS port must be passed as `--ros-args -p port:=...` only `if [ -n "$GPS_PORT" ]`; passing an empty `-p port:=""` breaks node startup. Same pattern should be used for any other optional hardware param.
- **`livox_driver_node.py`'s `LIVOX_DEVICE_IP` and `LIVOX_DATA_PORT` constants are hardware-specific, verified 2026-07-13 via `tcpdump`**: real device IP is `192.168.117.100` (not the generic `192.168.1.100` Livox docs suggest), real point-cloud data port is `56301` (not `56100` — Livox SDK2 convention is host-side ports 563xx for points / 564xx for IMU, offset from a base, not literally 56100/56200 as originally guessed). If this ever moves to different hardware/network config, re-verify with `tcpdump -i <iface> -n udp` rather than trusting the docstring.
- **`oakd_driver_node.py` requires `depthai==2.32.0.0` (or any 2.x), not the pip default (3.x)**. The code uses v2-only API (`pipeline.createColorCamera()`, `pipeline.createXLinkOut()`); v3's pipeline API is unrelated and throws `AttributeError`. Don't `pip3 install depthai` without pinning to 2.x.
- **The `ros2_final` container needs `-v /dev:/dev` (live device mount)** for any USB device that re-enumerates during connection (OAK-D/DepthAI does this on every boot). Without it the container's `/dev` is a stale snapshot from container-start time and DepthAI fails with `X_LINK_DEVICE_NOT_FOUND` no matter how many times you retry or even `docker restart`. See [[project-ida-docker-env]] for the 2026-07-13 container recreation that fixed this.
- **`gps_imu_driver_node.py` no longer reads raw serial NMEA — it's a MAVROS bridge now (rewritten 2026-07-13).** The physical GPS (Holybro H-RTK F9P Rover) only has one combined connector to Pixhawk's GPS1, and whatever it sends over any tappable pin is an unidentified binary protocol, not NMEA/UBX/RTCM. Don't "fix" this back to serial parsing without first getting a genuine second data path from the module (e.g. official Holybro docs/config tool confirming a real secondary NMEA-capable UART) — see [[project-ida-nodes-and-tests]] for the full investigation trail.

**Why:** All were found and fixed through real hardware/integration testing in the container; they don't show up as syntax errors or in isolated unit testing, only at runtime with the actual MAVROS/Livox/GPS/OAK-D stack.

**How to apply:** Before editing `decision_node.py`, `telemetri_node.py`, `local_map_node.py`, `livox_driver_node.py`, `gps_imu_driver_node.py`, `oakd_driver_node.py`, or `sistem_baslat.sh`, check the change against this list.

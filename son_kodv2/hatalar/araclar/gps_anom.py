#!/usr/bin/env python3
"""GPS ve RC anomalilerini bag icinde tam olarak yerini bularak listele."""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
so = rosbag2_py.StorageOptions(uri=bag, storage_id="mcap")
co = rosbag2_py.ConverterOptions("", "")
try:
    rd = rosbag2_py.SequentialReader(); rd.open(so, co)
except RuntimeError:
    rd = rosbag2_py.SequentialCompressionReader(); rd.open(so, co)
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
want = {"/mavros/global_position/global", "/mavros/rc/in"}
want &= set(types)
try:
    rd.set_filter(rosbag2_py.StorageFilter(topics=list(want)))
except Exception:
    pass

cache = {}
prev = None
n = 0
anom = []
rc_seg = []      # (t, nch) degisim noktalari
rc_prev = None
lat_hist = {}
while rd.has_next():
    topic, data, t = rd.read_next()
    if topic not in want:
        continue
    tn = types[topic]
    cls = cache.get(tn) or cache.setdefault(tn, get_message(tn))
    m = deserialize_message(data, cls)
    ts = t / 1e9
    if topic == "/mavros/rc/in":
        nch = len(m.channels)
        if rc_prev != nch:
            rc_seg.append((round(ts, 3), nch, list(m.channels)[:10]))
            rc_prev = nch
        continue
    n += 1
    lat, lon = m.latitude, m.longitude
    key = (round(lat, 2), round(lon, 2))
    lat_hist[key] = lat_hist.get(key, 0) + 1
    if prev is not None:
        dm = math.hypot((lat - prev[0]) * 111320.0,
                        (lon - prev[1]) * 111320.0 * math.cos(math.radians(lat or 40.7)))
        if dm > 1000.0 and len(anom) < 30:
            anom.append((round(ts, 3), n, round(prev[0], 6), round(prev[1], 6),
                         round(lat, 6), round(lon, 6), round(dm / 1000, 1),
                         int(m.status.status), round(m.altitude, 1)))
    prev = (lat, lon)

print(f"### {bag}")
print(f"GPS mesaj={n}")
print("-- 1km+ sicramalar (t, idx, onceki lat/lon, sonraki lat/lon, km, status, alt):")
for a in anom:
    print("   ", a)
print("-- lat/lon kumeleri (0.01 derece yuvarlanmis, en sik 12):")
for k, v in sorted(lat_hist.items(), key=lambda x: -x[1])[:12]:
    print(f"    {k}: {v}")
print("-- RC kanal sayisi degisimleri (t, nch, ilk10):")
for s in rc_seg[:40]:
    print("   ", s)

#!/usr/bin/env python3
"""GPS kumelerini frame_id/cov/status/alt imzasi ile ayirt et -> kac farkli yayinci var?"""
import sys
from collections import Counter, defaultdict
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
topic_arg = sys.argv[2] if len(sys.argv) > 2 else "/mavros/global_position/global"
so = rosbag2_py.StorageOptions(uri=bag, storage_id="mcap")
co = rosbag2_py.ConverterOptions("", "")
try:
    rd = rosbag2_py.SequentialReader(); rd.open(so, co)
except RuntimeError:
    rd = rosbag2_py.SequentialCompressionReader(); rd.open(so, co)
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
if topic_arg not in types:
    print("topic yok"); sys.exit(0)
try:
    rd.set_filter(rosbag2_py.StorageFilter(topics=[topic_arg]))
except Exception:
    pass
cls = get_message(types[topic_arg])

sig = Counter()
ornek = {}
first_last = defaultdict(lambda: [None, None])
while rd.has_next():
    topic, data, t = rd.read_next()
    if topic != topic_arg:
        continue
    m = deserialize_message(data, cls)
    ts = t / 1e9
    k = (round(m.latitude, 2), round(m.longitude, 2), m.header.frame_id,
         int(m.status.status), int(m.status.service),
         round(m.altitude, 0), round(m.position_covariance[0], 2),
         int(m.position_covariance_type),
         "stamp0" if (m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0) else "stampOK")
    sig[k] += 1
    fl = first_last[k]
    if fl[0] is None:
        fl[0] = ts
        ornek[k] = f"lat={m.latitude:.7f} lon={m.longitude:.7f} alt={m.altitude:.2f} cov={[round(c,2) for c in m.position_covariance]}"
    fl[1] = ts

print(f"### {bag}\n### topic={topic_arg}  toplam={sum(sig.values())}")
print(f"{'lat':>7} {'lon':>7} {'frame':>12} {'st':>3} {'sv':>3} {'alt':>7} {'cov0':>7} {'ct':>3} {'stamp':>8} {'adet':>8}  ilk..son")
for k, v in sig.most_common(20):
    fl = first_last[k]
    print(f"{k[0]:7.2f} {k[1]:7.2f} {k[2]:>12} {k[3]:3d} {k[4]:3d} {k[5]:7.0f} {k[6]:7.2f} {k[7]:3d} {k[8]:>8} {v:8d}  {fl[0]:.1f}..{fl[1]:.1f}")
    print(f"        ornek: {ornek[k]}")

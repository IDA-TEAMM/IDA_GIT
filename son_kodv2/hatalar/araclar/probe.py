#!/usr/bin/env python3
"""Belirli topic'lerden ham ornek dizisi cikar (kanit toplama)."""
import sys, argparse, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--topic", action="append", required=True)
ap.add_argument("--n", type=int, default=40)
ap.add_argument("--every", type=int, default=1)
ap.add_argument("--skip", type=int, default=0)
a = ap.parse_args()

so = rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap")
co = rosbag2_py.ConverterOptions("", "")
try:
    rd = rosbag2_py.SequentialReader(); rd.open(so, co)
except RuntimeError:
    rd = rosbag2_py.SequentialCompressionReader(); rd.open(so, co)
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
want = set(a.topic)
f = rosbag2_py.StorageFilter(topics=list(want))
try:
    rd.set_filter(f)
except Exception:
    pass

cnt = {t: 0 for t in want}
shown = {t: 0 for t in want}
cache = {}
while rd.has_next():
    topic, data, t = rd.read_next()
    if topic not in want:
        continue
    cnt[topic] += 1
    if cnt[topic] <= a.skip:
        continue
    if (cnt[topic] - a.skip - 1) % a.every:
        continue
    if shown[topic] >= a.n:
        if all(shown[x] >= a.n for x in want):
            break
        continue
    shown[topic] += 1
    tn = types[topic]
    cls = cache.get(tn) or cache.setdefault(tn, get_message(tn))
    m = deserialize_message(data, cls)
    ts = t / 1e9
    if tn == "sensor_msgs/msg/NavSatFix":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
              f"frame={m.header.frame_id} status={m.status.status} service={m.status.service} "
              f"lat={m.latitude:.7f} lon={m.longitude:.7f} alt={m.altitude:.2f} "
              f"cov={[round(c,2) for c in m.position_covariance]} covtype={m.position_covariance_type}")
    elif tn == "mavros_msgs/msg/RCIn":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec} rssi={m.rssi} "
              f"nch={len(m.channels)} ch={list(m.channels)[:10]}")
    elif tn == "std_msgs/msg/String":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} '{m.data}'")
    elif tn == "std_msgs/msg/Float32MultiArray":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} data={[round(x,4) for x in m.data]}")
    elif tn == "nav_msgs/msg/Odometry":
        p = m.pose.pose.position; q = m.pose.pose.orientation; tw = m.twist.twist
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
              f"frame={m.header.frame_id}/{m.child_frame_id} p=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
              f"q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f}) v=({tw.linear.x:.3f},{tw.linear.y:.3f}) w={tw.angular.z:.3f} "
              f"covdiag={[round(m.pose.covariance[i*6+i],3) for i in range(6)]}")
    elif tn == "geometry_msgs/msg/PoseStamped":
        p = m.pose.position
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
              f"frame={m.header.frame_id} p=({p.x:.3f},{p.y:.3f},{p.z:.3f})")
    elif tn == "geometry_msgs/msg/Twist":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} lin=({m.linear.x:.3f},{m.linear.y:.3f},{m.linear.z:.3f}) "
              f"ang=({m.angular.x:.3f},{m.angular.y:.3f},{m.angular.z:.3f})")
    elif tn == "geometry_msgs/msg/PoseArray":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
              f"frame={m.header.frame_id} n={len(m.poses)} "
              f"ilk5={[(round(p.position.x,2), round(p.position.y,2), round(p.position.z,2)) for p in m.poses[:5]]}")
    elif tn == "vision_msgs/msg/Detection3DArray":
        it = []
        for det in m.detections[:5]:
            c = det.bbox.center.position
            r = det.results[0] if det.results else None
            it.append((round(c.x,2), round(c.y,2), round(c.z,2),
                       r.hypothesis.class_id if r else None,
                       round(r.hypothesis.score,3) if r else None))
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec} frame={m.header.frame_id} "
              f"n={len(m.detections)} ilk5={it}")
    elif tn == "vision_msgs/msg/Detection2DArray":
        it = []
        for det in m.detections[:6]:
            r = det.results[0] if det.results else None
            b = det.bbox
            it.append((round(b.center.position.x,1), round(b.center.position.y,1),
                       round(b.size_x,1), round(b.size_y,1),
                       r.hypothesis.class_id if r else None,
                       round(r.hypothesis.score,3) if r else None))
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec} frame={m.header.frame_id} "
              f"n={len(m.detections)} ilk6={it}")
    elif tn == "sensor_msgs/msg/PointCloud2":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
              f"frame={m.header.frame_id} w={m.width} h={m.height} pointstep={m.point_step} "
              f"fields={[fl.name for fl in m.fields]}")
    elif tn == "mavros_msgs/msg/State":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} conn={m.connected} armed={m.armed} guided={m.guided} "
              f"manual={m.manual_input} mode={m.mode} sys_status={m.system_status}")
    elif tn == "std_msgs/msg/Bool":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} {m.data}")
    elif tn == "std_msgs/msg/Int32":
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} {m.data}")
    else:
        print(f"[{topic}] #{cnt[topic]} t={ts:.3f} {tn}")

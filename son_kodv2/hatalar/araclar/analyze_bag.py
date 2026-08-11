#!/usr/bin/env python3
"""Rosbag (mcap) derin analiz aracı — IDA/Girdap USV.

Bag'i tek geçişte tarar, topic bazlı anomali istatistikleri çıkarır ve JSON yazar.
Bellek dostu: mesajları saklamaz, akış halinde özetler.
"""
import sys, json, math, argparse
from collections import defaultdict, Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

NS = 1e9


def q_to_yaw(x, y, z, w):
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)


class RateTracker:
    """Alım zamanı (bag t) üzerinden frekans ve boşluk (gap) takibi."""

    def __init__(self):
        self.n = 0
        self.first = None
        self.last = None
        self.gaps = []          # (t_before, dt) en büyük boşluklar
        self.dt_sum = 0.0
        self.dt_sq = 0.0

    def add(self, t):
        if self.first is None:
            self.first = t
        else:
            dt = (t - self.last) / NS
            self.dt_sum += dt
            self.dt_sq += dt * dt
            if dt > 0.5:
                self.gaps.append((self.last / NS, round(dt, 3)))
        self.last = t
        self.n += 1

    def summary(self, gap_limit=25):
        if self.n == 0:
            return {"count": 0}
        span = (self.last - self.first) / NS
        mean_dt = self.dt_sum / max(1, self.n - 1)
        var = self.dt_sq / max(1, self.n - 1) - mean_dt ** 2
        self.gaps.sort(key=lambda g: -g[1])
        return {
            "count": self.n,
            "span_s": round(span, 3),
            "hz_ort": round(self.n / span, 3) if span > 0 else None,
            "dt_ort_ms": round(mean_dt * 1000, 2),
            "dt_std_ms": round(math.sqrt(max(0.0, var)) * 1000, 2),
            "gap_sayisi_0.5s+": len(self.gaps),
            "en_buyuk_gaplar": self.gaps[:gap_limit],
            "toplam_gap_s": round(sum(g[1] for g in self.gaps), 2),
        }


class BagAnalyzer:
    def __init__(self, path):
        self.path = path
        self.rates = defaultdict(RateTracker)
        self.types = {}

        # --- topic'e özel toplayıcılar ---
        # diagnostics: (name, level, message) -> [count, first_t, last_t]
        self.diag = defaultdict(lambda: [0, None, None])
        self.diag_kv = defaultdict(Counter)     # name -> deger ornekleri
        # durum makineleri
        self.state_seq = defaultdict(list)      # topic -> [(t, state)] gecisler
        self.state_dwell = defaultdict(Counter)
        # odom
        self.odom = {
            "nan": 0, "jump": [], "xmin": 1e18, "xmax": -1e18,
            "ymin": 1e18, "ymax": -1e18, "zmin": 1e18, "zmax": -1e18,
            "vmax": 0.0, "v_over": 0, "yaw_jump": [], "cov_nan": 0,
            "yol_m": 0.0, "sifir_pose": 0,
        }
        self._odom_prev = None
        # thrust
        self.thrust = {"n": 0, "min": 1e18, "max": -1e18, "sat_hi": 0,
                       "sat_lo": 0, "nan": 0, "len_hist": Counter(),
                       "sifir": 0, "ornekler": []}
        # perception
        self.percep = defaultdict(lambda: {"n": 0, "bos": 0, "det_top": 0,
                                           "det_max": 0, "nan": 0,
                                           "mesafe_min": 1e18, "mesafe_max": 0.0,
                                           "arka": 0, "uzak": 0, "sinif": Counter(),
                                           "skor_min": 1e18, "skor_max": 0.0})
        # mavros/state
        self.mstate = {"gecis": [], "son": None}
        # rc/in
        self.rc = {"n": 0, "ch": defaultdict(Counter), "bos": 0}
        # gps
        self.gps = {"n": 0, "nan": 0, "status": Counter(), "cov0": 0,
                    "lat": [1e18, -1e18], "lon": [1e18, -1e18],
                    "atlama": []}
        self._gps_prev = None
        # imu
        self.imu = {"n": 0, "nan": 0, "gyro_max": 0.0, "acc_max": 0.0,
                    "cov_neg1": 0}
        # local_position/pose
        self.lpose = {"n": 0, "nan": 0, "xmax": -1e18, "xmin": 1e18,
                      "ymax": -1e18, "ymin": 1e18, "sifir": 0}
        # cmd_vel
        self.cmdvel = {"n": 0, "lin": [1e18, -1e18], "ang": [1e18, -1e18],
                       "nan": 0, "sifir": 0}
        # lidar
        self.lidar = {"n": 0, "nokta_min": 10**9, "nokta_max": 0,
                      "nokta_top": 0, "bos": 0, "frame": Counter()}
        # mission/complete
        self.mcomplete = Counter()
        # header stamp vs bag t (saat tutarliligi)
        self.stamp_delta = defaultdict(lambda: {"n": 0, "min": 1e18,
                                                "max": -1e18, "sum": 0.0,
                                                "sifir_stamp": 0,
                                                "gelecek": 0, "eski": 0})
        self.clock_jump = []
        self._prev_bag_t = None

    # ---------- yardimcilar ----------
    def _hdr(self, topic, msg, t):
        h = getattr(msg, "header", None)
        if h is None:
            return
        st = h.stamp.sec + h.stamp.nanosec / NS
        d = self.stamp_delta[topic]
        if h.stamp.sec == 0 and h.stamp.nanosec == 0:
            d["sifir_stamp"] += 1
            return
        delta = t / NS - st
        d["n"] += 1
        d["sum"] += delta
        d["min"] = min(d["min"], delta)
        d["max"] = max(d["max"], delta)
        if delta < -0.05:
            d["gelecek"] += 1
        if delta > 1.0:
            d["eski"] += 1

    # ---------- topic isleyicileri ----------
    def h_diagnostics(self, msg, t):
        for s in msg.status:
            lvl = s.level
            if isinstance(lvl, (bytes, bytearray)):
                lvl = int.from_bytes(lvl, "little")
            k = (s.name, int(lvl), s.message)
            e = self.diag[k]
            e[0] += 1
            if e[1] is None:
                e[1] = t / NS
            e[2] = t / NS
            if len(self.diag_kv[s.name]) < 400:
                for kv in s.values:
                    self.diag_kv[s.name][f"{kv.key}={kv.value}"] += 1

    def h_state_str(self, topic, msg, t):
        seq = self.state_seq[topic]
        v = msg.data
        if not seq or seq[-1][1] != v:
            seq.append((round(t / NS, 3), v))
        self.state_dwell[topic][v] += 1

    def h_odom(self, msg, t):
        o = self.odom
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                tw.linear.x, tw.linear.y, tw.angular.z]
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            o["nan"] += 1
            return
        if any(math.isnan(c) or math.isinf(c) for c in msg.pose.covariance):
            o["cov_nan"] += 1
        if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
            o["sifir_pose"] += 1
        o["xmin"] = min(o["xmin"], p.x); o["xmax"] = max(o["xmax"], p.x)
        o["ymin"] = min(o["ymin"], p.y); o["ymax"] = max(o["ymax"], p.y)
        o["zmin"] = min(o["zmin"], p.z); o["zmax"] = max(o["zmax"], p.z)
        v = math.hypot(tw.linear.x, tw.linear.y)
        o["vmax"] = max(o["vmax"], v)
        if v > 5.0:
            o["v_over"] += 1
        yaw = q_to_yaw(q.x, q.y, q.z, q.w)
        if self._odom_prev is not None:
            pt, px, py, pyaw = self._odom_prev
            dt = (t - pt) / NS
            d = math.hypot(p.x - px, p.y - py)
            o["yol_m"] += d
            if dt > 0 and d / max(dt, 1e-3) > 10.0 and d > 1.0:
                if len(o["jump"]) < 60:
                    o["jump"].append({"t": round(t / NS, 3),
                                      "mesafe_m": round(d, 2),
                                      "dt_s": round(dt, 3),
                                      "hiz_ms": round(d / max(dt, 1e-3), 1)})
            dyaw = abs((yaw - pyaw + math.pi) % (2 * math.pi) - math.pi)
            if dt > 0 and dyaw / max(dt, 1e-3) > 6.0 and dyaw > 0.5:
                if len(o["yaw_jump"]) < 60:
                    o["yaw_jump"].append({"t": round(t / NS, 3),
                                          "dyaw_deg": round(math.degrees(dyaw), 1),
                                          "dt_s": round(dt, 3)})
        self._odom_prev = (t, p.x, p.y, yaw)

    def h_thrust(self, msg, t):
        d = list(msg.data)
        th = self.thrust
        th["n"] += 1
        th["len_hist"][len(d)] += 1
        if not d:
            return
        if any(math.isnan(v) or math.isinf(v) for v in d):
            th["nan"] += 1
            return
        th["min"] = min(th["min"], min(d))
        th["max"] = max(th["max"], max(d))
        if all(abs(v) < 1e-6 for v in d):
            th["sifir"] += 1
        for v in d:
            if v >= 0.999:
                th["sat_hi"] += 1
            if v <= -0.999:
                th["sat_lo"] += 1
        if len(th["ornekler"]) < 12:
            th["ornekler"].append({"t": round(t / NS, 3),
                                   "d": [round(x, 4) for x in d]})

    def h_percep_posearray(self, topic, msg, t):
        p = self.percep[topic]
        p["n"] += 1
        n = len(msg.poses)
        p["det_top"] += n
        p["det_max"] = max(p["det_max"], n)
        if n == 0:
            p["bos"] += 1
        for ps in msg.poses:
            x, y, z = ps.position.x, ps.position.y, ps.position.z
            if math.isnan(x) or math.isnan(y) or math.isnan(z) or \
               math.isinf(x) or math.isinf(y) or math.isinf(z):
                p["nan"] += 1
                continue
            r = math.hypot(x, y)
            p["mesafe_min"] = min(p["mesafe_min"], r)
            p["mesafe_max"] = max(p["mesafe_max"], r)
            if x < 0:
                p["arka"] += 1
            if r > 100.0:
                p["uzak"] += 1

    def h_percep_det2d(self, topic, msg, t):
        p = self.percep[topic]
        p["n"] += 1
        n = len(msg.detections)
        p["det_top"] += n
        p["det_max"] = max(p["det_max"], n)
        if n == 0:
            p["bos"] += 1
        for det in msg.detections:
            for r in det.results:
                cid = getattr(r.hypothesis, "class_id", None) if hasattr(r, "hypothesis") else getattr(r, "id", None)
                sc = getattr(r.hypothesis, "score", None) if hasattr(r, "hypothesis") else getattr(r, "score", None)
                if cid is not None:
                    p["sinif"][str(cid)] += 1
                if sc is not None:
                    p["skor_min"] = min(p["skor_min"], sc)
                    p["skor_max"] = max(p["skor_max"], sc)

    def h_percep_det3d(self, topic, msg, t):
        p = self.percep[topic]
        p["n"] += 1
        n = len(msg.detections)
        p["det_top"] += n
        p["det_max"] = max(p["det_max"], n)
        if n == 0:
            p["bos"] += 1
        for det in msg.detections:
            c = det.bbox.center.position
            if math.isnan(c.x) or math.isnan(c.y) or math.isinf(c.x) or math.isinf(c.y):
                p["nan"] += 1
            else:
                r = math.hypot(c.x, c.y)
                p["mesafe_min"] = min(p["mesafe_min"], r)
                p["mesafe_max"] = max(p["mesafe_max"], r)
                if c.x < 0:
                    p["arka"] += 1
                if r > 100.0:
                    p["uzak"] += 1
            for res in det.results:
                cid = getattr(res.hypothesis, "class_id", None) if hasattr(res, "hypothesis") else None
                sc = getattr(res.hypothesis, "score", None) if hasattr(res, "hypothesis") else None
                if cid is not None:
                    p["sinif"][str(cid)] += 1
                if sc is not None:
                    p["skor_min"] = min(p["skor_min"], sc)
                    p["skor_max"] = max(p["skor_max"], sc)

    def h_mavros_state(self, msg, t):
        cur = (bool(msg.connected), bool(msg.armed), bool(msg.guided), str(msg.mode))
        if self.mstate["son"] != cur:
            if len(self.mstate["gecis"]) < 300:
                self.mstate["gecis"].append(
                    {"t": round(t / NS, 3), "connected": cur[0], "armed": cur[1],
                     "guided": cur[2], "mode": cur[3]})
            self.mstate["son"] = cur

    def h_rc(self, msg, t):
        self.rc["n"] += 1
        ch = list(msg.channels)
        if not ch:
            self.rc["bos"] += 1
            return
        for i, v in enumerate(ch[:10]):
            self.rc["ch"][i][int(v)] += 1

    def h_gps(self, msg, t):
        g = self.gps
        g["n"] += 1
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            g["nan"] += 1
            return
        g["status"][int(msg.status.status)] += 1
        if all(c == 0.0 for c in msg.position_covariance):
            g["cov0"] += 1
        g["lat"][0] = min(g["lat"][0], msg.latitude)
        g["lat"][1] = max(g["lat"][1], msg.latitude)
        g["lon"][0] = min(g["lon"][0], msg.longitude)
        g["lon"][1] = max(g["lon"][1], msg.longitude)
        if self._gps_prev is not None:
            plat, plon = self._gps_prev
            dm = math.hypot((msg.latitude - plat) * 111320.0,
                            (msg.longitude - plon) * 111320.0 * math.cos(math.radians(msg.latitude)))
            if dm > 20.0 and len(g["atlama"]) < 40:
                g["atlama"].append({"t": round(t / NS, 3), "metre": round(dm, 1)})
        self._gps_prev = (msg.latitude, msg.longitude)

    def h_imu(self, msg, t):
        i = self.imu
        i["n"] += 1
        a, w = msg.linear_acceleration, msg.angular_velocity
        if any(math.isnan(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)):
            i["nan"] += 1
            return
        i["gyro_max"] = max(i["gyro_max"], math.sqrt(w.x**2 + w.y**2 + w.z**2))
        i["acc_max"] = max(i["acc_max"], math.sqrt(a.x**2 + a.y**2 + a.z**2))
        if msg.orientation_covariance[0] < 0:
            i["cov_neg1"] += 1

    def h_lpose(self, msg, t):
        l = self.lpose
        l["n"] += 1
        p = msg.pose.position
        if math.isnan(p.x) or math.isnan(p.y):
            l["nan"] += 1
            return
        if p.x == 0.0 and p.y == 0.0:
            l["sifir"] += 1
        l["xmin"] = min(l["xmin"], p.x); l["xmax"] = max(l["xmax"], p.x)
        l["ymin"] = min(l["ymin"], p.y); l["ymax"] = max(l["ymax"], p.y)

    def h_cmdvel(self, msg, t):
        c = self.cmdvel
        c["n"] += 1
        lx, az = msg.linear.x, msg.angular.z
        if math.isnan(lx) or math.isnan(az):
            c["nan"] += 1
            return
        if lx == 0.0 and az == 0.0:
            c["sifir"] += 1
        c["lin"][0] = min(c["lin"][0], lx); c["lin"][1] = max(c["lin"][1], lx)
        c["ang"][0] = min(c["ang"][0], az); c["ang"][1] = max(c["ang"][1], az)

    def h_lidar(self, msg, t):
        l = self.lidar
        l["n"] += 1
        n = msg.width * msg.height
        l["nokta_top"] += n
        l["nokta_min"] = min(l["nokta_min"], n)
        l["nokta_max"] = max(l["nokta_max"], n)
        if n == 0:
            l["bos"] += 1
        if len(l["frame"]) < 20:
            l["frame"][msg.header.frame_id] += 1

    # ---------- ana dongu ----------
    def run(self, limit=None, progress=0):
        so = rosbag2_py.StorageOptions(uri=self.path, storage_id="mcap")
        co = rosbag2_py.ConverterOptions("", "")
        try:
            rd = rosbag2_py.SequentialReader()
            rd.open(so, co)
        except RuntimeError:
            # zstd ile sikistirilmis bag: metadata.yaml'li dizin uzerinden oku
            rd = rosbag2_py.SequentialCompressionReader()
            rd.open(so, co)
        for tm in rd.get_all_topics_and_types():
            self.types[tm.name] = tm.type

        cache = {}
        i = 0
        while rd.has_next():
            topic, data, t = rd.read_next()
            i += 1
            if progress and i % progress == 0:
                print(f"  ... {i} mesaj", file=sys.stderr, flush=True)
            if limit and i > limit:
                break

            # saat sicramasi tespiti (bag alim zamani monoton olmali)
            if self._prev_bag_t is not None:
                d = (t - self._prev_bag_t) / NS
                if abs(d) > 60.0 and len(self.clock_jump) < 20:
                    self.clock_jump.append(
                        {"onceki": round(self._prev_bag_t / NS, 3),
                         "sonraki": round(t / NS, 3),
                         "atlama_s": round(d, 3), "topic": topic})
            self._prev_bag_t = t
            self.rates[topic].add(t)

            tn = self.types.get(topic)
            if tn is None:
                continue
            cls = cache.get(tn)
            if cls is None:
                try:
                    cls = get_message(tn)
                except Exception:
                    cache[tn] = False
                    continue
                cache[tn] = cls
            if cls is False:
                continue
            try:
                msg = deserialize_message(data, cls)
            except Exception:
                continue

            self._hdr(topic, msg, t)

            if topic == "/diagnostics":
                self.h_diagnostics(msg, t)
            elif tn == "std_msgs/msg/String":
                self.h_state_str(topic, msg, t)
            elif tn == "nav_msgs/msg/Odometry":
                self.h_odom(msg, t)
            elif tn == "std_msgs/msg/Float32MultiArray":
                self.h_thrust(msg, t)
            elif tn == "geometry_msgs/msg/PoseArray":
                self.h_percep_posearray(topic, msg, t)
            elif tn == "vision_msgs/msg/Detection2DArray":
                self.h_percep_det2d(topic, msg, t)
            elif tn == "vision_msgs/msg/Detection3DArray":
                self.h_percep_det3d(topic, msg, t)
            elif tn == "mavros_msgs/msg/State":
                self.h_mavros_state(msg, t)
            elif tn == "mavros_msgs/msg/RCIn":
                self.h_rc(msg, t)
            elif tn == "sensor_msgs/msg/NavSatFix":
                self.h_gps(msg, t)
            elif tn == "sensor_msgs/msg/Imu":
                self.h_imu(msg, t)
            elif tn == "sensor_msgs/msg/PointCloud2":
                self.h_lidar(msg, t)
            elif tn == "geometry_msgs/msg/Twist":
                self.h_cmdvel(msg, t)
            elif topic == "/mavros/local_position/pose":
                self.h_lpose(msg, t)
            elif tn == "std_msgs/msg/Bool":
                self.mcomplete[bool(msg.data)] += 1
        return i

    def report(self):
        out = {"bag": self.path, "tipler": self.types}
        out["oranlar"] = {k: v.summary() for k, v in sorted(self.rates.items())}
        out["saat_sicramalari"] = self.clock_jump
        out["diagnostics"] = sorted(
            [{"ad": k[0], "level": k[1], "mesaj": k[2], "adet": v[0],
              "ilk_t": v[1], "son_t": v[2]} for k, v in self.diag.items()],
            key=lambda d: (-d["level"], -d["adet"]))
        out["diag_degerler"] = {k: v.most_common(25) for k, v in self.diag_kv.items()}
        out["durum_gecisleri"] = {k: v[:400] for k, v in self.state_seq.items()}
        out["durum_gecis_sayisi"] = {k: len(v) for k, v in self.state_seq.items()}
        out["durum_dagilimi"] = {k: v.most_common() for k, v in self.state_dwell.items()}
        if self.odom["xmin"] < 1e17:
            out["odom"] = self.odom
        if self.thrust["n"]:
            th = dict(self.thrust)
            th["len_hist"] = dict(th["len_hist"])
            out["thrust"] = th
        pp = {}
        for k, v in self.percep.items():
            d = dict(v)
            d["sinif"] = dict(d["sinif"])
            if d["mesafe_min"] > 1e17:
                d["mesafe_min"] = None
            if d["skor_min"] > 1e17:
                d["skor_min"] = None
            d["det_ort"] = round(d["det_top"] / d["n"], 3) if d["n"] else 0
            d["bos_yuzde"] = round(100.0 * d["bos"] / d["n"], 2) if d["n"] else 0
            pp[k] = d
        if pp:
            out["algi"] = pp
        if self.mstate["gecis"]:
            out["mavros_state"] = self.mstate["gecis"]
        if self.rc["n"]:
            out["rc"] = {"n": self.rc["n"], "bos": self.rc["bos"],
                         "kanallar": {f"ch{i+1}": c.most_common(6)
                                      for i, c in sorted(self.rc["ch"].items())}}
        if self.gps["n"]:
            out["gps"] = self.gps
        if self.imu["n"]:
            out["imu"] = self.imu
        if self.lpose["n"]:
            out["local_pose"] = self.lpose
        if self.cmdvel["n"]:
            out["cmd_vel"] = self.cmdvel
        if self.lidar["n"]:
            l = dict(self.lidar)
            l["frame"] = dict(l["frame"])
            l["nokta_ort"] = round(l["nokta_top"] / l["n"], 1)
            out["lidar"] = l
        if self.mcomplete:
            out["mission_complete"] = {str(k): v for k, v in self.mcomplete.items()}
        sd = {}
        for k, v in self.stamp_delta.items():
            if v["n"] or v["sifir_stamp"]:
                d = dict(v)
                d["ort_s"] = round(v["sum"] / v["n"], 4) if v["n"] else None
                d["min"] = round(v["min"], 4) if v["n"] else None
                d["max"] = round(v["max"], 4) if v["n"] else None
                del d["sum"]
                sd[k] = d
        out["stamp_gecikme"] = sd
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--progress", type=int, default=0)
    a = ap.parse_args()

    an = BagAnalyzer(a.bag)
    n = an.run(limit=a.limit, progress=a.progress)
    rep = an.report()
    rep["okunan_mesaj"] = n
    with open(a.out, "w") as f:
        json.dump(rep, f, indent=1, ensure_ascii=False, default=str)
    print(f"OK {a.bag} -> {a.out} ({n} mesaj)")


if __name__ == "__main__":
    main()

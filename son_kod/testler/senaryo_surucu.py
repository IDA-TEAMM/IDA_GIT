#!/usr/bin/env python3
"""
GIRDAP — Video günü KOŞUL MATRİSİ sahte sürücüsü (SADECE TEST).

fake_mavros_publisher'ın senaryolu hali: /mavros/* topic'lerinin sahtesini
üretir ama ARM/AUTO zamanlaması, veri kesintileri ve mod oyunları senaryoya
göre kurgulanır. Yığının (hardware.launch with_mavros:=false) her koşulda
doğru davrandığını CANLI doğrulamak için.

Kullanım (izole domain ŞART — ROS_DOMAIN_ID=77):
    python3 testler/senaryo_surucu.py --senaryo auto-once-arm

Senaryolar (video günü riskleri):
    nominal          ARM(3s) → AUTO(6s) → 4 nokta → TAMAMLANDI
    auto-once-arm    AUTO(3s) → ARM(13s)  [F-V.6: QGC Start Mission akışı]
    disarm-ortada    görev ortasında (50s) beklenmedik DISARM → KILL beklenir
    fc-kopma         50s'de TÜM akış durur (mavros öldü) → heartbeat-KILL
    gps-kayip        45s'de yalnız GPS kesilir → yığın YAŞAMALI, hedef donar
    tamamlandi-mod   tur biter → MANUAL(140s) → tekrar AUTO(148s):
                     görev YENİDEN BAŞLAMAMALI (TAMAMLANDI terminal)
    boot-gec         30s HİÇBİR mesaj yok (FC busy) → sahte KILL OLMAMALI
                     (F-M.7), sonra normal akış → görev başlar

Sürücü /girdap/mission/state + /girdap/mission/complete'i dinler ve
geçişleri "[GOZLEM] t=..s state=..." satırlarıyla basar — koşu sonrası rapor
bu satırlardan derlenir.
"""

import argparse
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import RCOut, State

BASE_LAT = 40.0
BASE_LON = 29.0
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = M_PER_DEG_LAT * math.cos(math.radians(BASE_LAT))

SENARYOLAR = {
    "nominal": dict(sure=150.0, arm_t=3.0, auto_t=6.0),
    "auto-once-arm": dict(sure=150.0, arm_t=13.0, auto_t=3.0),
    "disarm-ortada": dict(sure=80.0, arm_t=3.0, auto_t=6.0, disarm_t=50.0),
    "fc-kopma": dict(sure=80.0, arm_t=3.0, auto_t=6.0, akis_bitis_t=50.0),
    "gps-kayip": dict(sure=100.0, arm_t=3.0, auto_t=6.0, gps_bitis_t=45.0),
    "tamamlandi-mod": dict(
        sure=165.0, arm_t=3.0, auto_t=6.0,
        mod_olaylari=[(140.0, "MANUAL"), (148.0, "AUTO")],
    ),
    "boot-gec": dict(sure=120.0, arm_t=33.0, auto_t=36.0, akis_baslangic_t=30.0),
}


class SenaryoSurucu(Node):
    def __init__(self, ad: str, cfg: dict):
        super().__init__("senaryo_surucu")
        self.ad = ad
        self.cfg = cfg
        self.t = 0.0
        self.x_e = 0.0
        self.y_n = 0.0
        self.gorev_t = None          # hareket başlangıcı (arm+auto anı)
        self._son_state = None

        self.pub_vel = self.create_publisher(
            TwistStamped, "/mavros/local_position/velocity_local", 10)
        self.pub_vel_body = self.create_publisher(
            TwistStamped, "/mavros/local_position/velocity_body", 10)
        self.pub_imu = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self.pub_state = self.create_publisher(State, "/mavros/state", 10)
        self.pub_hdg = self.create_publisher(
            Float64, "/mavros/global_position/compass_hdg", 10)
        self.pub_rc = self.create_publisher(RCOut, "/mavros/rc/out", 10)
        self.pub_gps = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", 10)

        self.create_subscription(
            String, "/girdap/mission/state", self._gozle_state, 10)
        self.create_subscription(
            Bool, "/girdap/mission/complete", self._gozle_complete, 10)

        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.tick_state)
        self.get_logger().info(f"[SENARYO] {ad} basladi ({cfg['sure']}s)")
        self._complete_gorudu = False

    # ---- gözlem ----
    def _gozle_state(self, msg: String) -> None:
        if msg.data != self._son_state:
            print(f"[GOZLEM] t={self.t:5.1f}s state={msg.data}", flush=True)
            self._son_state = msg.data

    def _gozle_complete(self, msg: Bool) -> None:
        if msg.data and not self._complete_gorudu:
            print(f"[GOZLEM] t={self.t:5.1f}s COMPLETE=true", flush=True)
            self._complete_gorudu = True

    # ---- senaryo durumu ----
    def _akista(self) -> bool:
        if self.t < self.cfg.get("akis_baslangic_t", 0.0):
            return False
        if self.t >= self.cfg.get("akis_bitis_t", 1e9):
            return False
        return True

    def _armed(self) -> bool:
        if self.t >= self.cfg.get("disarm_t", 1e9):
            return False
        return self.t >= self.cfg["arm_t"]

    def _mode(self) -> str:
        mod = "AUTO" if self.t >= self.cfg["auto_t"] else "MANUAL"
        for olay_t, olay_mod in self.cfg.get("mod_olaylari", []):
            if self.t >= olay_t:
                mod = olay_mod
        return mod

    # ---- veri üretimi ----
    def tick(self) -> None:
        self.t += 0.1
        if self.t >= self.cfg["sure"]:
            raise SystemExit(0)
        if not self._akista():
            return
        now = self.get_clock().now().to_msg()

        surur = self._armed() and self._mode() == "AUTO"
        if surur and self.gorev_t is None:
            self.gorev_t = self.t
            print(f"[SENARYO] t={self.t:5.1f}s FC gorevi surmeye basladi",
                  flush=True)
        gt = (self.t - self.gorev_t) if (surur and self.gorev_t is not None) else None

        if gt is not None:
            kenar = int(gt // 30) % 4
            hedef_yon = kenar * 90.0
            pusula = hedef_yon + 8.0 * math.exp(-(gt % 30) / 5.0) \
                + 2.0 * math.sin(gt * 1.3)
            hiz = 1.0 + 0.15 * math.sin(gt * 0.7) \
                - 0.4 * math.exp(-(gt % 30) / 3.0)
        else:
            pusula = 0.0
            hiz = 0.0

        yon_rad = math.radians(90.0 - pusula)
        vx = hiz * math.cos(yon_rad)
        vy = hiz * math.sin(yon_rad)
        self.x_e += vx * 0.1
        self.y_n += vy * 0.1

        v = TwistStamped()
        v.header.stamp = now
        v.twist.linear.x = vx
        v.twist.linear.y = vy
        self.pub_vel.publish(v)

        vb = TwistStamped()
        vb.header.stamp = now
        vb.twist.linear.x = hiz
        self.pub_vel_body.publish(vb)

        imu = Imu()
        imu.header.stamp = now
        imu.orientation.z = math.sin(yon_rad / 2.0)
        imu.orientation.w = math.cos(yon_rad / 2.0)
        self.pub_imu.publish(imu)

        h = Float64()
        h.data = pusula % 360.0
        self.pub_hdg.publish(h)

        rc = RCOut()
        rc.header.stamp = now
        rc.channels = [0] * 8
        if gt is not None:
            fark = 80.0 * math.exp(-(gt % 30) / 3.0)
            rc.channels[0] = int(1650 + fark)
            rc.channels[2] = int(1650 - fark)
        else:
            rc.channels[0] = 1500      # nötr (armed değil / AUTO değil)
            rc.channels[2] = 1500
        self.pub_rc.publish(rc)

        if self.t < self.cfg.get("gps_bitis_t", 1e9):
            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = "base_link"
            gps.status.status = 0
            gps.latitude = BASE_LAT + self.y_n / M_PER_DEG_LAT
            gps.longitude = BASE_LON + self.x_e / M_PER_DEG_LON
            self.pub_gps.publish(gps)

    def tick_state(self) -> None:
        if not self._akista():
            return
        s = State()
        s.connected = True
        s.armed = self._armed()
        s.mode = self._mode()
        self.pub_state.publish(s)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--senaryo", required=True, choices=sorted(SENARYOLAR))
    args = p.parse_args()

    rclpy.init()
    node = SenaryoSurucu(args.senaryo, SENARYOLAR[args.senaryo])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print(f"[SENARYO] {args.senaryo} bitti (t={node.t:.1f}s)", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

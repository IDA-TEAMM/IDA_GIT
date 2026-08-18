#!/usr/bin/env python3
"""Sanal göl izini CSV'ye yaz — /mavros/local_position/pose (domain 77)."""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Kayit(Node):
    def __init__(self, yol: str) -> None:
        super().__init__("iz_kaydet")
        self.f = open(yol, "w")
        self.f.write("t,x,y,psi\n")
        self.t0 = None
        self.n = 0
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on, 10)

    def _on(self, m: PoseStamped) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        z, w = m.pose.orientation.z, m.pose.orientation.w
        psi = 2.0 * math.atan2(z, w)
        self.f.write(f"{t - self.t0:.3f},{m.pose.position.x:.4f},"
                     f"{m.pose.position.y:.4f},{psi:.4f}\n")
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()


def main() -> None:
    rclpy.init()
    d = Kayit(sys.argv[1])
    try:
        rclpy.spin(d)
    except KeyboardInterrupt:
        pass
    finally:
        d.f.flush()
        d.f.close()


if __name__ == "__main__":
    main()

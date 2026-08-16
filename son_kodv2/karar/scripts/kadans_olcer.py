#!/usr/bin/env python3
"""İtki kadansı ölçer — /girdap/control/thrust varış zamanlarını dosyaya yazar.

Yalıtılmış bant deneyi için (ROS_DOMAIN_ID=77): planning_node'un kontrol
döngüsü her adımda thrust yayınlar; varış aralıkları = gerçek döngü kadansı.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

CIKTI = sys.argv[1]
SURE_S = float(sys.argv[2]) if len(sys.argv) > 2 else 400.0


def main():
    rclpy.init()
    n = Node("kadans_olcer")
    zamanlar: list[float] = []
    n.create_subscription(
        Float32MultiArray, "/girdap/control/thrust",
        lambda m: zamanlar.append(time.monotonic()), 50,
    )
    bitis = time.monotonic() + SURE_S
    while rclpy.ok() and time.monotonic() < bitis:
        rclpy.spin_once(n, timeout_sec=0.5)
    with open(CIKTI, "w") as f:
        f.writelines(f"{t:.6f}\n" for t in zamanlar)
    print(f"{len(zamanlar)} thrust mesajı → {CIKTI}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

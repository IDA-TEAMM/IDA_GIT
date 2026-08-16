#!/usr/bin/env python3
"""Parkur-3 hedef rengi parametresini BARINDIRAN node.

🔴 NEDEN AYRI NODE (16.08.2026): `KamikazeHedefKapisi` daha önce
`perception_camera_node` içinde yaşıyordu. O node (HSV yedek kamera hattı)
kaldırılınca `kamikaze_target_color` parametresi **sahipsiz** kaldı ⇒
operatörün rengi yükleyeceği yer yok ⇒ `p3_bekleniyor` hiç true olmaz ⇒
FSM **PARKUR3'e geçmez** ⇒ Parkur-3 = 0 puan (145 puan, toplamın %48'i).
Ve bu sessiz olurdu: hiçbir hata basılmaz, tekne son waypoint'te temiz durur.

Neden `fsm_node`/`planning_node` değil: kapı `/girdap/mission/state`'e ABONE,
`fsm_node` ise onu YAYINLIYOR — aynı node'da kendi kendine abonelik gereksiz
bir dolaşıklık olurdu. Kapı `/girdap/mission/hedef_rengi` yayınlıyor,
`planning_node` onu tüketiyor; ikisinin arasında bağımsız durmak en temizi.

Operatör kullanımı (KALKIŞTAN ÖNCE — şartname s.22):
    ros2 param set /kamikaze_param_node kamikaze_target_color kirmizi
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from girdap_decision.kamikaze_param import KamikazeHedefKapisi


class KamikazeParamNode(Node):
    def __init__(self) -> None:
        super().__init__("kamikaze_param_node")
        self._kapi = KamikazeHedefKapisi(self)
        self.get_logger().info(
            "Parkur-3 hedef rengi kapisi hazir. Yukleme (KALKISTAN ONCE): "
            "ros2 param set /kamikaze_param_node kamikaze_target_color <renk>"
        )

    @property
    def kapi(self) -> KamikazeHedefKapisi:
        return self._kapi


def main(args=None) -> None:                              # noqa: ANN001
    rclpy.init(args=args)
    n = KamikazeParamNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Keep a /map subscription alive so slam_toolbox actually publishes.

Humble slam_toolbox skips OccupancyGrid publishing when
get_subscription_count() == 0. RViz in this project displays
/global_costmap/costmap, not /map, so mapping would otherwise stay silent.
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapKeepalive(Node):
    def __init__(self):
        super().__init__('map_keepalive')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)

    def _on_map(self, _msg: OccupancyGrid) -> None:
        return


def main():
    rclpy.init()
    node = MapKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

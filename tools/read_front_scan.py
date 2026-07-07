#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FrontScanReader(Node):
    def __init__(self):
        super().__init__("front_scan_reader")
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if not valid_ranges:
            self.get_logger().info("No obstacle detected")
            return

        min_distance = min(valid_ranges)

        self.get_logger().info(
            f"Closest detected obstacle: {min_distance:.3f} m"
        )


def main():
    rclpy.init()
    node = FrontScanReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

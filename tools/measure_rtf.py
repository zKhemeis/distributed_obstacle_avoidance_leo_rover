import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class RTFMonitor(Node):
    def __init__(self):
        super().__init__("rtf_monitor")

        self.sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            10
        )

        self.first_sim_time = None
        self.first_wall_time = None
        self.last_print_time = time.monotonic()

    def clock_callback(self, msg):
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        wall_time = time.monotonic()

        if self.first_sim_time is None:
            self.first_sim_time = sim_time
            self.first_wall_time = wall_time
            return

        sim_elapsed = sim_time - self.first_sim_time
        wall_elapsed = wall_time - self.first_wall_time

        if wall_elapsed <= 0:
            return

        rtf = sim_elapsed / wall_elapsed

        if wall_time - self.last_print_time >= 2.0:
            print(f"RTF = {rtf:.3f} | sim_elapsed = {sim_elapsed:.1f}s | wall_elapsed = {wall_elapsed:.1f}s")
            self.last_print_time = wall_time


def main():
    rclpy.init()
    node = RTFMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

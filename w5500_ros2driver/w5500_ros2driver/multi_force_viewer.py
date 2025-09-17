#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force

class MultiForceViewer(Node):
    def __init__(self):
        super().__init__("multi_force_viewer")

        # Suscribirse a los 4 publicadores con namespaces
        self.sub1 = self.create_subscription(Force, "/sensor_1/force", self.cb1, 10)
        self.sub2 = self.create_subscription(Force, "/sensor_2/force", self.cb2, 10)
        self.sub3 = self.create_subscription(Force, "/sensor_3/force", self.cb3, 10)
        self.sub4 = self.create_subscription(Force, "/sensor_4/force", self.cb4, 10)

        self.data = {}

    def cb1(self, msg): self.data["sensor_1"] = msg; self.print_all()
    def cb2(self, msg): self.data["sensor_2"] = msg; self.print_all()
    def cb3(self, msg): self.data["sensor_3"] = msg; self.print_all()
    def cb4(self, msg): self.data["sensor_4"] = msg; self.print_all()

    def format_msg(self, sensor, msg):
        return (f"{sensor}: "
                f"FxMy1={msg.fx_my_1:.3f}, "
                f"FyMx1={msg.fy_mx_1:.3f}, "
                f"FyMx2={msg.fy_mx_2:.3f}, "
                f"FxMy2={msg.fx_my_2:.3f}, "
                f"Mz={msg.mz:.3f}, "
                f"Fz1={msg.fz_1:.3f}, "
                f"Fz2={msg.fz_2:.3f}")

    def print_all(self):
        # Solo imprime cuando tenga datos de todos los sticks
        if len(self.data) == 4:
            out = [self.format_msg(sensor, msg) for sensor, msg in self.data.items()]
            self.get_logger().info(" | ".join(out))

def main():
    rclpy.init()
    node = MultiForceViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

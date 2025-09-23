#!/usr/bin/env python3

import csv
import time
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force


class ForceCSVLogger(Node):
    def __init__(self):
        super().__init__("force_csv_logger")

        # Parameter for output file name
        self.declare_parameter("csv_file", "forces.csv")
        self.csv_file = self.get_parameter("csv_file").get_parameter_value().string_value

        # Open file
        self.file = open(self.csv_file, "w", newline="", buffering=1)  # line-buffered
        self.writer = csv.writer(self.file)

        # Write header
        self.writer.writerow([
            "timestamp",
            "fx_my_1", "fy_mx_1", "fy_mx_2",
            "fx_my_2", "mz", "fz_1", "fz_2"
        ])

        # Subscribe to a force topic (edit to match your namespace!)
        self.sub = self.create_subscription(Force, "/sensor_2/force", self.callback, 10)

    def callback(self, msg: Force):
        now = time.time()  # seconds since epoch
        row = [
            now,
            msg.fx_my_1,
            msg.fy_mx_1,
            msg.fy_mx_2,
            msg.fx_my_2,
            msg.mz,
            msg.fz_1,
            msg.fz_2,
        ]
        self.writer.writerow(row)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ForceCSVLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

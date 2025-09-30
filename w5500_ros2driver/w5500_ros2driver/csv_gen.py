#!/usr/bin/env python3

import csv
import time
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force

'''Data is processed sequentially for each sensor, this could be a possible cause for bottleneck.
Paralellism might be implemented later if needed.
'''

# mapping from sensor name to port
PORT_MAP = {
    "sensor_1": 5000,
    "sensor_2": 5001,
    "sensor_3": 5002,
    "sensor_4": 5003,
}

SENSORS = [f"sensor_{i}" for i in range(1, 5)]


class MultiForceCSVLogger(Node):
    def __init__(self):
        super().__init__("multi_force_csv_logger")

        # Dictionary to store file handles and writers for each sensor
        self.files = {}
        self.writers = {}
        
        # Create CSV file and writer for each sensor
        for sensor in SENSORS:
            port = PORT_MAP.get(sensor, "unknown")
            
            if port != "unknown":
                filename = f"{sensor}_{port}.csv"
            else:
                filename = f"{sensor}.csv"
            
            # Open file with line buffering
            file_handle = open(filename, "w", newline="", buffering=1)
            writer = csv.writer(file_handle)
            
            # Write header
            writer.writerow([
                "timestamp",
                "fx_my_1", "fy_mx_1", "fy_mx_2",
                "fx_my_2", "mz", "fz_1", "fz_2"
            ])
            
            self.files[sensor] = file_handle
            self.writers[sensor] = writer
            
            self.get_logger().info(f"Created CSV file: {filename}")

        # Subscribe to all sensor topics
        self.subs = []
        for sensor in SENSORS:
            topic = f"/{sensor}/force"
            callback = self.make_callback(sensor)
            sub = self.create_subscription(Force, topic, callback, 10)
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to {topic}")

    # Create a callback function for each sensor
    def make_callback(self, sensor_name):
        def callback(msg: Force):
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
            # Write to the appropriate CSV file
            self.writers[sensor_name].writerow(row)
        return callback

    def destroy_node(self):
        # Close all files
        for sensor, file_handle in self.files.items():
            file_handle.close()
            self.get_logger().info(f"Closed CSV file for {sensor}")
        super().destroy_node()


def main():
    rclpy.init()
    node = MultiForceCSVLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
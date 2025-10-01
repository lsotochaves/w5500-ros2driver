#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force


class ForceCSVLogger(Node):
    def __init__(self):
        super().__init__("force_csv_logger")

        # Parameters: sensor name and port (namespace will also help identify)
        self.declare_parameter("sensor_name", "sensor_1")
        self.declare_parameter("port", 5000)

        sensor_name = self.get_parameter("sensor_name").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        filename = f"{sensor_name}_{port}.csv"

        # Open CSV file with larger buffer for 1kHz performance
        self.file = open(filename, "w", newline="", buffering=65536)
        self.writer = csv.writer(self.file)

        # Header
        self.writer.writerow([
            "info", "timestamp",
            "fx_my_1", "fy_mx_1", "fy_mx_2",
            "fx_my_2", "mz", "fz_1", "fz_2"
        ])

        # CRITICAL FIX: Use relative topic name
        # Since node is in namespace "sensor_1", "force" resolves to "/sensor_1/force"
        topic = "force"
        
        # Increased queue size for 1kHz
        self.create_subscription(Force, topic, self.callback, 100)

        self.get_logger().info(f"Logging /{sensor_name}/force to {filename}")
        
        # Batch writing for better performance
        self.buffer = []
        self.batch_size = 50
        
        # Periodic flush every 50ms
        self.flush_timer = self.create_timer(0.05, self.flush_buffer)
        
        # Statistics tracking
        self.msg_count = 0
        self.last_report_time = self.get_clock().now()
        self.stats_timer = self.create_timer(5.0, self.report_stats)

    def callback(self, msg: Force):
        # Extract timestamp from message
        t_ros = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        
        # Prepare row
        row = [
            msg.info, t_ros,
            msg.fx_my_1, msg.fy_mx_1, msg.fy_mx_2,
            msg.fx_my_2, msg.mz, msg.fz_1, msg.fz_2
        ]
        
        self.buffer.append(row)
        self.msg_count += 1
        
        # Flush when batch is full
        if len(self.buffer) >= self.batch_size:
            self.flush_buffer()

    def flush_buffer(self):
        """Write buffered rows to disk"""
        if self.buffer:
            self.writer.writerows(self.buffer)
            self.file.flush()
            self.buffer.clear()

    def report_stats(self):
        """Report message reception rate"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_report_time).nanoseconds / 1e9
        
        if elapsed > 0:
            hz = self.msg_count / elapsed
            self.get_logger().info(f"Receiving at {hz:.1f} Hz, buffer: {len(self.buffer)} msgs")
        
        self.msg_count = 0
        self.last_report_time = current_time

    def destroy_node(self):
        """Ensure all data is written before shutdown"""
        self.flush_buffer()
        self.file.close()
        self.get_logger().info("CSV file closed")
        super().destroy_node()


def main():
    rclpy.init()
    node = ForceCSVLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
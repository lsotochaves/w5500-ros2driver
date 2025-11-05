#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from w5500_msg.msg import ForceF4
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import defaultdict
import threading

SENSORS = [f"sensor_{i}" for i in range(1, 5)]
CHANNELS = ["fx_my_1", "fy_mx_1", "fy_mx_2", "fx_my_2", "mz", "fz_1", "fz_2"]
PORT_MAP = {"sensor_1": 5000, "sensor_2": 5001, "sensor_3": 5002, "sensor_4": 5003}


class MultiForceF4Plotter(Node):
    def __init__(self):
        super().__init__("multi_ForceF4_plotter")

        # Each sensor → dictionary of lists (no window limit)
        self.buffers = defaultdict(lambda: {ch: [] for ch in CHANNELS})

        self.subs = []
        for s in SENSORS:
            topic = f"/{s}/Force"
            self.subs.append(
                self.create_subscription(ForceF4, topic, self.make_callback(s), 10)
            )
            self.get_logger().info(f"Subscribed to {topic}")

    # Append incoming data to correct sensor buffer
    def make_callback(self, sensor_name):
        def callback(msg: ForceF4):
            b = self.buffers[sensor_name]
            b["fx_my_1"].append(msg.fx_my_1)
            b["fy_mx_1"].append(msg.fy_mx_1)
            b["fy_mx_2"].append(msg.fy_mx_2)
            b["fx_my_2"].append(msg.fx_my_2)
            b["mz"].append(msg.mz)
            b["fz_1"].append(msg.fz_1)
            b["fz_2"].append(msg.fz_2)
        return callback


def main():
    rclpy.init()
    node = MultiForceF4Plotter()

    # ---- Matplotlib setup ----
    fig, axes = plt.subplots(len(SENSORS), 1, figsize=(8, 10), sharex=True)
    if len(SENSORS) == 1:
        axes = [axes]

    lines = {}
    for ax, sensor in zip(axes, SENSORS):
        port = PORT_MAP.get(sensor, "?")
        ax.set_title(f"{sensor} (port {port})")
        ax.set_ylabel("ForceF4 Value")

        for ch in CHANNELS:
            (line,) = ax.plot([], [], label=ch, alpha=0.8)
            lines[(sensor, ch)] = line

        ax.legend(loc="upper right", fontsize="small")

    axes[-1].set_xlabel("Samples")

    # ---- Animation update ----
    def update(_):
        for sensor in SENSORS:
            buf = node.buffers[sensor]
            for ch in CHANNELS:
                data = buf[ch]
                line = lines[(sensor, ch)]
                line.set_data(range(len(data)), data)

        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        return lines.values()

    # Real-time 20 ms update interval (≈50 fps)
    ani = FuncAnimation(fig, update, interval=20, blit=False)

    # ---- Spin ROS in background ----
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

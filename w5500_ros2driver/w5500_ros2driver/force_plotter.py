#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque, defaultdict
import threading

WINDOW = 500  # Amount of data seen in plot window. Last 500 samples.
SENSORS = [f"sensor_{i}" for i in range(1, 5)]
CHANNELS = ["fx_my_1", "fy_mx_1", "fy_mx_2", "fx_my_2", "mz", "fz_1", "fz_2"]

# Sensor names mapped to port numbers
PORT_MAP = {"sensor_1": 5000, "sensor_2": 5001, "sensor_3": 5002, "sensor_4": 5003}


class MultiForcePlotter(Node):
    def __init__(self):
        super().__init__("multi_force_plotter")

        # Generates a dequeue buffer for each channel of each sensor
        self.buffers = defaultdict(lambda: {ch: deque() for ch in CHANNELS})

        # subscribe to all 4 sensor topics
        self.subs = []
        for s in SENSORS:
            topic = f"/{s}/force"
            self.subs.append(
                self.create_subscription(Force, topic, self.make_callback(s), 10)
            )

    # Function in charge of adding data to the correct buffer position for each sensor.
    def make_callback(self, sensor_name):
        def callback(msg: Force):
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
    node = MultiForcePlotter()

    # Create one subplot per sensor (stacked vertically)
    fig, axes = plt.subplots(len(SENSORS), 1, figsize=(8, 10), sharex=True)

    if len(SENSORS) == 1:
        axes = [axes]  # ensure iterable

    lines = {}
    #Initialize plots and lines for each sensor/channel
    for ax, sensor in zip(axes, SENSORS):
        # title includes sensor name and port
        port = PORT_MAP.get(sensor, "?")
        ax.set_title(f"{sensor} (port {port})")
        ax.set_ylabel("Force Value")

        for ch in CHANNELS:
            (line,) = ax.plot([], [], label=ch, alpha=0.8)
            lines[(sensor, ch)] = line

        ax.legend(loc="upper right", fontsize="small")

    axes[-1].set_xlabel("Samples")

    # Takes the latest values from each sensor and updates matplotlib Line2D object with new X and Y values.
    def update(frame):
        for sensor in SENSORS:
            buf = node.buffers[sensor]
            for ch in CHANNELS:
                data = list(buf[ch])
                line = lines[(sensor, ch)]
                line.set_data(range(len(data)), data)

        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        return lines.values()

    # calls update repeatedly to change plot over time.
    ani = FuncAnimation(fig, update, interval=20, blit=False) # redraw every 20ms; 50fps approx. Change if PC can't keep up.

    # spin ROS in background thread for concurrency in between ROS and matplotlib
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

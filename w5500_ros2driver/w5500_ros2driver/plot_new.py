#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# how many samples to keep visible
WINDOW = 500  


class ForcePlotter(Node):
    def __init__(self):
        super().__init__("force_plotter")

        # Subscribe to the "force" topic
        self.sub = self.create_subscription(Force, "/sensor_2/force", self.callback, 10)

        # rolling buffers for each channel
        self.fx_my_1 = deque(maxlen=WINDOW)
        self.fy_mx_1 = deque(maxlen=WINDOW)
        self.fy_mx_2 = deque(maxlen=WINDOW)
        self.fx_my_2 = deque(maxlen=WINDOW)
        self.mz = deque(maxlen=WINDOW)
        self.fz_1 = deque(maxlen=WINDOW)
        self.fz_2 = deque(maxlen=WINDOW)

    def callback(self, msg: Force):
        """Append new data sample to the buffers"""
        self.fx_my_1.append(msg.fx_my_1)
        self.fy_mx_1.append(msg.fy_mx_1)
        self.fy_mx_2.append(msg.fy_mx_2)
        self.fx_my_2.append(msg.fx_my_2)
        self.mz.append(msg.mz)
        self.fz_1.append(msg.fz_1)
        self.fz_2.append(msg.fz_2)


def main():
    rclpy.init()
    node = ForcePlotter()

    fig, ax = plt.subplots()
    ax.set_title("Forces (real-time)")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Value")

    # create line objects for each channel
    (line1,) = ax.plot([], [], label="Fx My 1")
    (line2,) = ax.plot([], [], label="Fy Mx 1")
    (line3,) = ax.plot([], [], label="Fy Mx 2")
    (line4,) = ax.plot([], [], label="Fx My 2")
    (line5,) = ax.plot([], [], label="Mz")
    (line6,) = ax.plot([], [], label="Fz 1")
    (line7,) = ax.plot([], [], label="Fz 2")
    ax.legend(loc="upper right")

    def update(frame):
        # update data from node buffers
        line1.set_data(range(len(node.fx_my_1)), list(node.fx_my_1))
        line2.set_data(range(len(node.fy_mx_1)), list(node.fy_mx_1))
        line3.set_data(range(len(node.fy_mx_2)), list(node.fy_mx_2))
        line4.set_data(range(len(node.fx_my_2)), list(node.fx_my_2))
        line5.set_data(range(len(node.mz)), list(node.mz))
        line6.set_data(range(len(node.fz_1)), list(node.fz_1))
        line7.set_data(range(len(node.fz_2)), list(node.fz_2))

        # rescale axes
        ax.relim()
        ax.autoscale_view()
        return (line1, line2, line3, line4, line5, line6, line7)

    # refresh plot every 50 ms (~20 FPS)
    ani = FuncAnimation(fig, update, interval=50, blit=False)

    # spin ROS in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import serial
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from forcestick_msg.msg import Force

record = []
# openCoRoCo interface, which connects the microcontroler to
# a ROS2 node
class OpenCoRoCo(object):
    # Constructor, initialize the arguments
    def __init__(self):
        # Params of serial port connection
        self.port_name = None
        self.baudrate = None
        self.bytesize = None
        self.parity = None
        self.stopbits = None
        self.timeout = None
        self.connected = False

        # Variables for force data processing
        self.values = [0, 0, 0, 0, 0, 0, 0]
        self.fx_my_1 = self.values[0]
        self.fy_mx_1 = self.values[1]
        self.fy_mx_2 = self.values[2]
        self.fx_my_2 = self.values[3]
        self.mz = self.values[4]
        self.fz_1 = self.values[5]
        self.fz_2 = self.values[6]
        self.raw_values = ""
        # The fit curve is a linear equation y = ax
        # Each element corresponds to an axis
        # The first element corresponds to the X-axis
        # The second element corresponds to the Y-axis
        # The third element corresponds to the Z-axis
        # Voltage as a function of applied force
        self.calibration_curve = [
            0.012482267917127,
            0.027715361973854,
            0.009115448082126
        ]
        self.offset = [0, 0, 0, 0, 0, 0, 0]
        self.initialize = False
        self.offset_init = False
        self.init_counter = 0
        self.offset_init_counter = 0

    # Function to connect to the serial port
    def connect(self, port_name="/dev/ttyACM1", baudrate=115200, bytesize=8,
                parity="N", stopbits=1, timeout=None):
        self.port_name = port_name
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        print(f"Connecting to {self.port_name} ...")
        self.serial_device = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout
        )
        print(f"Connected to {self.serial_device.portstr}")
        self.connected = True

    # Function to get force data from serial port and
    # processes it
    def get_forces(self):
        updated = False
        self.raw_values = self.serial_device.read_until().decode()
        self.raw_values = self.raw_values.replace("\r", "").replace("\n", "")
        self.raw_values = self.raw_values.split(",")
        if len(self.raw_values) == 7:
            if not self.initialize:
                self.initialize = True
                updated = False
            else:
                for i in range(0, len(self.raw_values)):
                    self.values[i] = int(self.raw_values[i])
                    self.values[i] *= 3/4095
                self.fx_my_1 = self.values[0]
                self.fy_mx_1 = self.values[1]
                self.fy_mx_2 = self.values[2]
                self.fx_my_2 = self.values[3]
                self.mz = self.values[4]
                self.fz_1 = self.values[5]
                self.fz_2 = self.values[6]
                updated = True
            # if not self.initialize and not self.offset_init:
            #     self.initialize = True
            #     updated = False
            #     self.init_counter += 1
            #     print(f"The force driver is initialized")
            # elif self.initialize and not self.offset_init:
            #     if self.init_counter < 10:
            #         self.init_counter += 1
            #     else:
            #         if self.offset_init_counter < 100:
            #             for i in range(0, len(self.raw_values)):
            #                 self.values[i] = int(self.raw_values[i])
            #                 self.values[i] *= 3/4095
            #                 self.offset[i] += self.values[i] ** 2
            #             self.offset_init_counter += 1
            #         else:
            #             for i in range(0, len(self.raw_values)):
            #                 self.offset[i] = (self.offset[i]/100) ** (1/2)
            #             self.offset_init = True
            #             print(f"The force driver is set up")
            #             print(f"The force driver will start reading force data")
            #         updated = False
            # elif self.initialize and self.offset_init:
            #     for i in range(0, len(self.raw_values)):
            #         self.values[i] = int(self.raw_values[i])
            #         self.values[i] *= 3/4095
            #         self.values[i] -= self.offset[i]
            #         self.values[i] /= self.calibration_curve[i]
            #     self.force_frontal_1 = self.values[0]
            #     self.force_frontal_2 = self.values[1]
            #     self.force_upper = self.values[2]
            #     self.force_lateral = self.values[3]
            #     self.torque_frontal_1 = self.values[4]
            #     self.torque_frontal_2 = self.values[5]
            #     updated = True
        return updated

class ForcestickPublisher(Node):
    # Constructor, create the publisher and timer
    def __init__(self, opencoroco):
        super().__init__("Forcestick_joint")
        self.opencoroco = opencoroco
        self.force_publisher = self.create_publisher(Force, "force", 10)
        timer_periodo = 1/1000
        self.timer = self.create_timer(timer_periodo, self.timer_callback)
        self.init_time = time.time()

    # Function to publish the force data processed through
    # the topic using the custom force_msg format
    # Also save the force data to a buffer (list)
    def timer_callback(self):
        if self.opencoroco.get_forces():
            record_data = str(time.time() - self.init_time) + ","
            record_data += ",".join(
                str(value) for value in self.opencoroco.values
            )
            force_msg = Force()
            force_msg.fx_my_1 = self.opencoroco.fx_my_1
            force_msg.fy_mx_1 = self.opencoroco.fy_mx_1
            force_msg.fy_mx_2 = self.opencoroco.fy_mx_2
            force_msg.fx_my_2 = self.opencoroco.fx_my_2
            force_msg.mz = self.opencoroco.mz
            force_msg.fz_1 = self.opencoroco.fz_1
            force_msg.fz_2 = self.opencoroco.fz_2

            self.force_publisher.publish(force_msg)
            self.get_logger().info(
                f"Fx My 1: {str(force_msg.fx_my_1)}"
            )
            self.get_logger().info(
                f"Fy Mx 1: {str(force_msg.fy_mx_1)}"
            )
            self.get_logger().info(
                f"Fy Mx 2: {str(force_msg.fy_mx_2)}"
            )
            self.get_logger().info(
                f"Fx My 2: {str(force_msg.fx_my_2)}"
            )
            self.get_logger().info(
                f"Mz: {str(force_msg.mz)}"
            )
            self.get_logger().info(
                f"Fz 1: {str(force_msg.fz_1)}"
            )
            self.get_logger().info(
                f"Fz 2: {str(force_msg.fz_2)}"
            )
            record.append(record_data)

# Function to store the recorded force data in a csv file
def output_record(record):
    record = record[:1000]
    with open("output.csv", "w") as f:
        f.write("Timestamp,fxmy1,fxmy2,fymx1,fymx2,mz,fz1,fz2\n")
        for item in record:
            f.write(f"{item}\n")

# Function to handle the CTRL-C signal (SIGINT) and SIGTERM
def terminate_handler(signum, stack_frame):
    print("\nCatched signal: ", signum)
    print("Record size: ", len(record))
    output_record(record)
    sys.exit()

signal.signal(signal.SIGINT, terminate_handler)
signal.signal(signal.SIGTERM, terminate_handler)

def main():
    opencoroco = OpenCoRoCo()
    opencoroco.connect()
    rclpy.init()
    force_joint = ForcestickPublisher(opencoroco)
    rclpy.spin(force_joint)
    force_joint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

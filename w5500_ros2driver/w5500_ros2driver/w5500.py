#!/usr/bin/env python3

import socket
import threading
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force

# --- Config ---
''' These parameters are declared as global for simplicity. 
    They could be moved to class attributes later. '''
MODE_UDP = True          # False = TCP, True = UDP
HOST = "0.0.0.0"
SIZE_SINGLE_BATCH = 15
AMOUNT_FORCE_READINGS = 7
INFO_BYTE_AMOUNT = 1


class OpenCoRoCo:
    def __init__(self):
        self.host = HOST
        self.port = None
        self.mode = MODE_UDP
        self.server_socket = None
        self.conn = None
        self.latest_frame = None   # holds the last decoded frame
        self.running = False
        self.info = 0

    def start_server(self):
        if self.port is None:
            raise ValueError("Port must be set before starting server")

        # Creates a server for listening incoming UDP packets
        if self.mode:  # UDP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((self.host, self.port))
            print(f"UDP server ready on {self.host}:{self.port}")
        
        #Creates a server for listening incoming TCP packets
        else:  # TCP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"TCP server listening on {self.host}:{self.port}")
            self.conn, self.addr = self.server_socket.accept()
            print("Connected by", self.addr)

        # Launch background receiver thread so network I/O does not block ROS2 timer
        self.running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        # Continuously receive packets and update self.latest_frame
        while self.running:
            try:
                if self.mode:  # UDP
                    data, addr = self.server_socket.recvfrom(1024)
                    if len(data) >= SIZE_SINGLE_BATCH:
                        frame = data[-SIZE_SINGLE_BATCH:]
                        self.latest_frame = self._parse_frame(frame)
                else:  # TCP
                    data = self.conn.recv(4096)
                    if not data:
                        continue
                    frame = data[-SIZE_SINGLE_BATCH:]
                    if len(frame) == SIZE_SINGLE_BATCH:
                        self.latest_frame = self._parse_frame(frame)
            except Exception as e:
                print(f"Receiver error: {e}")
                continue

    def _parse_frame(self, frame):
        # Convert raw frame bytes into scaled values
        self.info = frame[0]
        frame = bytearray(frame)
        del frame[:INFO_BYTE_AMOUNT]  # remove info byte
        readings = []
        for i in range(AMOUNT_FORCE_READINGS):
            msb = frame[2 * i]
            lsb = frame[2 * i + 1]
            val = ((msb << 8) | lsb) & 0x0FFF
            readings.append(val)
        return [r * (3.0 / 4095.0) for r in readings]

    def get_latest_frame(self):
        ''' Return last decoded frame (may be None if nothing received yet) '''
        return self.latest_frame


class ForcestickPublisher(Node):
    def __init__(self, opencoroco):
        super().__init__("forcestick_pub")

        # Parameters
        self.declare_parameter("server_port", 5000)
        server_port = self.get_parameter("server_port").get_parameter_value().integer_value

        self.opencoroco = opencoroco
        self.opencoroco.port = server_port
        self.opencoroco.start_server()

        # ROS publisher
        self.force_publisher = self.create_publisher(Force, "force", 10)

        ''' Timer executes every 1 ms (1 kHz).
            It reads the latest frame (non-blocking) and publishes it. '''
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        values = self.opencoroco.get_latest_frame()
        if values is None:
            return

        msg = Force()
        (
            msg.fx_my_1,
            msg.fy_mx_1,
            msg.fy_mx_2,
            msg.fx_my_2,
            msg.mz,
            msg.fz_1,
            msg.fz_2,
        ) = map(float, values)

        self.force_publisher.publish(msg) 
        # Optional debug output
        self.get_logger().info(f"Published: {values} + {self.opencoroco.info}")


def main():
    rclpy.init()
    opencoroco = OpenCoRoCo()
    node = ForcestickPublisher(opencoroco)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

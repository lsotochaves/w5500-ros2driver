
'''
#!/usr/bin/env python3

import signal
import sys
import time
import socket
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force

MODE_UDP = False
SERVER_COMMUNICATION_PORT = 5000
INFO_BYTE_AMOUNT = 1
HOST = "0.0.0.0"
SIZE_SINGLE_BATCH = 15
AMOUNT_FORCE_READINGS = 7
AMOUNT_READINGS_PER_BATCH = 10
SIZE_TOTAL_PAYLOAD = SIZE_SINGLE_BATCH * AMOUNT_READINGS_PER_BATCH

record = []


class OpenCoRoCo(object):
    def __init__(self):
        self.host = HOST
        self.port = SERVER_COMMUNICATION_PORT
        self.mode = MODE_UDP
        self.server_socket = None
        self.conn = None
        self.buf = bytearray()
        self.batch_member = 0
        self.decoded_frames = []  # queue of (information, values)

    def start_server(self):
        if self.mode:  # UDP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((self.host, self.port))
            print(f"UDP server ready on {self.host}:{self.port}")
        else:  # TCP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"TCP server listening on {self.host}:{self.port}")
            self.conn, self.addr = self.server_socket.accept()
            print("Connected by", self.addr)

    def get_forces(self):
        updated = False
        if self.server_socket is None:
            return False

        # === UDP mode ===
        if self.mode:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                if not data:
                    return None
                self.addr = addr
            except (BlockingIOError, TimeoutError):
                return False

            if len(data) < SIZE_SINGLE_BATCH:
                return False

            frame = bytearray(data[:SIZE_SINGLE_BATCH])
            information = frame[:INFO_BYTE_AMOUNT]
            del frame[:INFO_BYTE_AMOUNT]

            readings = []
            for i in range(AMOUNT_FORCE_READINGS):
                msb = frame[2 * i]
                lsb = frame[2 * i + 1]
                val = ((msb << 8) | lsb) & 0x0FFF
                readings.append(val)

            values = [r * (3.0 / 4095.0) for r in readings]
            self.decoded_frames.append((information, values))
            updated = True
            return updated

        # === TCP mode ===
        else:
            if self.conn is None:
                return None

            data = self.conn.recv(1024)
            if not data:
                return False

            self.buf.extend(data)

            while len(self.buf) >= SIZE_SINGLE_BATCH:
                frame = self.buf[:SIZE_SINGLE_BATCH]
                del self.buf[:SIZE_SINGLE_BATCH]

                information = frame[:INFO_BYTE_AMOUNT]
                del frame[:INFO_BYTE_AMOUNT]

                readings = []
                for i in range(AMOUNT_FORCE_READINGS):
                    msb = frame[2 * i]
                    lsb = frame[2 * i + 1]
                    val = ((msb << 8) | lsb) & 0x0FFF
                    readings.append(val)

                values = [r * (3.0 / 4095.0) for r in readings]
                self.decoded_frames.append((information, values))
                updated = True

            return updated


class ForcestickPublisher(Node):
    def __init__(self, opencoroco):
        super().__init__("Forcestick_joint")
        self.opencoroco = opencoroco
        self.force_publisher = self.create_publisher(Force, "force", 10)
        timer_periodo = 1 / 1000
        self.timer = self.create_timer(timer_periodo, self.timer_callback)
        self.init_time = time.time()
        self.first_line = True

        self.csv_path = "output.csv"
        self.csv_file = open(self.csv_path, "w", buffering=1)
        self.csv_file.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")

    def timer_callback(self):
        if self.opencoroco.get_forces():
            while self.opencoroco.decoded_frames:
                information, values = self.opencoroco.decoded_frames.pop(0)
                self.opencoroco.batch_member += 1

                # First frame of the batch → timestamp
                if self.opencoroco.batch_member == 1:
                    record_data = f"{time.time() - self.init_time},"
                else:
                    record_data = f"{information[0]},"

                record_data += ",".join(str(v) for v in values)

                # Build ROS message
                force_msg = Force()
                (
                    force_msg.fx_my_1,
                    force_msg.fy_mx_1,
                    force_msg.fy_mx_2,
                    force_msg.fx_my_2,
                    force_msg.mz,
                    force_msg.fz_1,
                    force_msg.fz_2,
                ) = map(float, values)

                # Publish and save
                self.force_publisher.publish(force_msg)
                self.csv_file.write(record_data + "\n")
                record.append(record_data)

                # Pretty terminal output
                labels = ["Fx My 1", "Fy Mx 1", "Fy Mx 2", "Fx My 2", "Mz", "Fz 1", "Fz 2"]
                for label, val in zip(labels, values):
                    self.get_logger().info(f"{label}: {val}")

                # Reset batch counter if needed
                if self.opencoroco.batch_member == AMOUNT_READINGS_PER_BATCH:
                    self.opencoroco.batch_member = 0


def output_record(record):
    with open("output.csv", "w") as f:
        f.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")
        for item in record:
            f.write(f"{item}\n")


def terminate_handler(signum, stack_frame):
    print("\nCatched signal: ", signum)
    print("Record size: ", len(record))
    output_record(record)
    sys.exit()


signal.signal(signal.SIGINT, terminate_handler)
signal.signal(signal.SIGTERM, terminate_handler)


def main():
    opencoroco = OpenCoRoCo()
    opencoroco.start_server()
    rclpy.init()
    force_joint = ForcestickPublisher(opencoroco)
    try:
        rclpy.spin(force_joint)
    finally:
        force_joint.csv_file.close()
        force_joint.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
#!/usr/bin/env python3

import signal
import sys
import time
import socket
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force

MODE_UDP = False
SERVER_COMMUNICATION_PORT = 5000
INFO_BYTE_AMOUNT = 1
HOST = "0.0.0.0"
SIZE_SINGLE_BATCH = 15
AMOUNT_FORCE_READINGS = 7
AMOUNT_READINGS_PER_BATCH = 10
SIZE_TOTAL_PAYLOAD = SIZE_SINGLE_BATCH * AMOUNT_READINGS_PER_BATCH

record = []


class OpenCoRoCo(object):
    def __init__(self):
        self.host = HOST
        self.port = SERVER_COMMUNICATION_PORT
        self.mode = MODE_UDP
        self.server_socket = None
        self.conn = None
        self.buf = bytearray()
        self.batch_member = 0
        self.decoded_frames = []  # queue of (information, values)

    def start_server(self):
        if self.mode:  # UDP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((self.host, self.port))
            print(f"UDP server ready on {self.host}:{self.port}")
        else:  # TCP
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"TCP server listening on {self.host}:{self.port}")
            self.conn, self.addr = self.server_socket.accept()
            print("Connected by", self.addr)

    def get_forces(self):
        updated = False
        if self.server_socket is None:
            return False

        # === UDP mode ===
        if self.mode:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                if not data:
                    return None
                self.addr = addr
            except (BlockingIOError, TimeoutError):
                return False

            if len(data) < SIZE_SINGLE_BATCH:
                return False

            frame = bytearray(data[:SIZE_SINGLE_BATCH])
            information = frame[:INFO_BYTE_AMOUNT]
            del frame[:INFO_BYTE_AMOUNT]

            readings = []
            for i in range(AMOUNT_FORCE_READINGS):
                msb = frame[2 * i]
                lsb = frame[2 * i + 1]
                val = ((msb << 8) | lsb) & 0x0FFF
                readings.append(val)

            values = [r * (3.0 / 4095.0) for r in readings]
            self.decoded_frames.append((information, values))
            updated = True
            return updated

        # === TCP mode ===
        else:
            if self.conn is None:
                return None

            data = self.conn.recv(1024)
            if not data:
                return False

            self.buf.extend(data)

            while len(self.buf) >= SIZE_SINGLE_BATCH:
                frame = self.buf[:SIZE_SINGLE_BATCH]
                del self.buf[:SIZE_SINGLE_BATCH]

                information = frame[:INFO_BYTE_AMOUNT]
                del frame[:INFO_BYTE_AMOUNT]

                readings = []
                for i in range(AMOUNT_FORCE_READINGS):
                    msb = frame[2 * i]
                    lsb = frame[2 * i + 1]
                    val = ((msb << 8) | lsb) & 0x0FFF
                    readings.append(val)

                values = [r * (3.0 / 4095.0) for r in readings]
                self.decoded_frames.append((information, values))
                updated = True

            return updated


class ForcestickPublisher(Node):
    def __init__(self, opencoroco):
        super().__init__("Forcestick_joint")
        self.opencoroco = opencoroco
        self.force_publisher = self.create_publisher(Force, "force", 10)
        timer_periodo = 1 / 1000
        self.timer = self.create_timer(timer_periodo, self.timer_callback)
        self.init_time = time.time()
        self.first_line = True

        self.csv_path = "output.csv"
        self.csv_file = open(self.csv_path, "w", buffering=1)
        self.csv_file.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")

    def timer_callback(self):
        if self.opencoroco.get_forces():
            # Take only the last 2 frames in the buffer
            if len(self.opencoroco.decoded_frames) > 2:
                self.opencoroco.decoded_frames = self.opencoroco.decoded_frames[-2:]

            while self.opencoroco.decoded_frames:
                information, values = self.opencoroco.decoded_frames.pop(0)
                self.opencoroco.batch_member += 1

                # First frame of the batch → timestamp
                if self.opencoroco.batch_member == 1:
                    record_data = f"{time.time() - self.init_time},"
                else:
                    record_data = f"{information[0]},"

                record_data += ",".join(str(v) for v in values)

                # Build ROS message
                force_msg = Force()
                (
                    force_msg.fx_my_1,
                    force_msg.fy_mx_1,
                    force_msg.fy_mx_2,
                    force_msg.fx_my_2,
                    force_msg.mz,
                    force_msg.fz_1,
                    force_msg.fz_2,
                ) = map(float, values)

                # Publish and save
                self.force_publisher.publish(force_msg)
                self.csv_file.write(record_data + "\n")
                record.append(record_data)

                # Pretty terminal output
                labels = ["Fx My 1", "Fy Mx 1", "Fy Mx 2", "Fx My 2", "Mz", "Fz 1", "Fz 2"]
                for label, val in zip(labels, values):
                    self.get_logger().info(f"{label}: {val}")

                # Reset batch counter if needed
                if self.opencoroco.batch_member == AMOUNT_READINGS_PER_BATCH:
                    self.opencoroco.batch_member = 0


def output_record(record):
    with open("output.csv", "w") as f:
        f.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")
        for item in record:
            f.write(f"{item}\n")


def terminate_handler(signum, stack_frame):
    print("\nCatched signal: ", signum)
    print("Record size: ", len(record))
    output_record(record)
    sys.exit()


signal.signal(signal.SIGINT, terminate_handler)
signal.signal(signal.SIGTERM, terminate_handler)


def main():
    opencoroco = OpenCoRoCo()
    opencoroco.start_server()
    rclpy.init()
    force_joint = ForcestickPublisher(opencoroco)
    try:
        rclpy.spin(force_joint)
    finally:
        force_joint.csv_file.close()
        force_joint.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

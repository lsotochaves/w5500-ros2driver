#!/usr/bin/env python3
import numpy as np
import time
import socket
import struct
import threading
from abc import ABC, abstractmethod
from ahrs.filters import Mahony
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from w5500_msg.msg import ForceF4
from w5500_msg.msg import ForceF3


# ---------- Abstract Socket Handler ----------
class SocketHandler(ABC):
    @abstractmethod
    def setup_server(self, host, port):
        raise NotImplementedError

    @abstractmethod
    def receive(self):
        raise NotImplementedError

    @abstractmethod
    def close(self):
        raise NotImplementedError


# ---------- UDP Handler ----------
class UDPHandler(SocketHandler):
    def __init__(self):
        self.socket = None

    def setup_server(self, host, port):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((host, port))
            print(f"[UDP] Server ready on {host}:{port}")
        except OSError as e:
            print(f"[UDP] Error binding to port {port}: {e}")
            self.socket = None

    def receive(self):
        if not self.socket:
            return b""
        try:
            data, _ = self.socket.recvfrom(1024)
            return data
        except Exception:
            return b""

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None


# ---------- TCP Handler ----------
class TCPHandler(SocketHandler):
    def __init__(self):
        self.socket = None
        self.conn = None
        self.accepting = False
        self._lock = threading.Lock()  # protects access to self.conn

    def setup_server(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.listen(1)
        self.socket.settimeout(1.0)
        print(f"[TCP] Server listening on {host}:{port}")
        self.accepting = True
        threading.Thread(target=self._accept_connection, daemon=True).start()

    def _accept_connection(self):
        while self.accepting:
            try:
                conn, addr = self.socket.accept()
                with self._lock:
                    self.conn = conn
                print(f"[TCP] Client connected: {addr}")
                break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP] Accept error: {e}")
                break

    def receive(self):
        with self._lock:
            if self.conn is None:
                return b""
            try:
                return self.conn.recv(4096)
            except Exception:
                return b""

    def close(self):
        self.accepting = False
        with self._lock:
            if self.conn:
                self.conn.close()
                self.conn = None
        if self.socket:
            self.socket.close()
            self.socket = None


# ---------- Base Processor ----------
class BaseProcessor(ABC):
    INFO_BYTES = 1
    BYTES_PER_READING = 2
    AMOUNT_FINAL_DATA = None
    MSG_TYPE = None

    @abstractmethod
    def parse_frame(self, frame: bytes):
        raise NotImplementedError

    @abstractmethod
    def build_msg(self, node: Node, info: int, values: list[float]):
        raise NotImplementedError


# ---------- STM32F4 Processor ----------
class STM32F4Processor(BaseProcessor):
    AMOUNT_FINAL_DATA = 7  # fx_my_1, fy_mx_1, fy_mx_2, fx_my_2, mz, fz_1, fz_2
    start = 1
    end = start + AMOUNT_FINAL_DATA * BaseProcessor.BYTES_PER_READING
    MSG_TYPE = ForceF4

    def parse_frame(self, frame: bytes):
        info = frame[0]
        body = memoryview(frame)[self.start:self.end]
        readings = struct.unpack(f">{self.AMOUNT_FINAL_DATA}H", body)
        scale = 3.0 / 4095.0
        scaled = [(r & 0x0FFF) * scale for r in readings]
        return info, scaled

    def build_msg(self, node: Node, info: int, values: list[float]):
        msg = self.MSG_TYPE()
        msg.info = info
        (
            msg.fx_my_1,
            msg.fy_mx_1,
            msg.fy_mx_2,
            msg.fx_my_2,
            msg.mz,
            msg.fz_1,
            msg.fz_2,
        ) = map(float, values)
        return msg


# ---------- STM32F3 Processor ----------
class STM32F3Processor(BaseProcessor):
    AMOUNT_FINAL_DATA = 10  # gyr(3), acc(3), mag(3), volts(1)
    start = 1
    end = start + AMOUNT_FINAL_DATA * BaseProcessor.BYTES_PER_READING
    MSG_TYPE = ForceF3

    def __init__(self, opmode=1, sistema=0, plane=0):
        self.opmode = opmode
        self.sistema = sistema
        self.plane = plane

        # Scaling constants
        self.scale_adc = 3.3 / 4095.0
        self.gyr_const = (8.75 / 1000) * (np.pi / 180)  # rad/s
        self.acc_const = (1 / 1000) * 9.80665           # m/s²
        self.mag_constxy = 100 / 1100
        self.mag_constz = 100 / 980

        # Calibration constants
        self.m = 35.4955 if sistema == 0 else 106.5217
        self.b = 0
        self.force_offset = 0
        self.force_offset_samples = []
        self.offset_ready = False

        # Mahony filter setup
        self.mahony = Mahony(frequency=100)
        self.q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.stabilization_start = None
        self.stabilization_time = 10
        self.angle_offset_init = False
        self.roll_buff, self.pitch_buff, self.yaw_buff = [], [], []
        self.roll_offset = self.pitch_offset = self.yaw_offset = 0.0
        self.angle_offset_samples = 100

        # Initial orientation per plane
        self.initial_quaternions = {
            0: np.array([0.0, 0.0, 0.0, 1.0]),
            1: np.array([-0.916, 0.0, 0.0, -0.4]),
            2: np.array([-1.0, 0.0, 0.0, 0.275]),
            3: np.array([-0.607, 0.0, 0.0, 0.795]),
        }
        if plane in self.initial_quaternions:
            self.q = self.initial_quaternions[plane]

        self.force_components = np.zeros(3)
        self.expected_rotated_force = np.zeros(3)

    def parse_frame(self, frame: bytes):
        info = frame[0]
        body = memoryview(frame)[self.start:self.end]
        readings = struct.unpack(f">{self.AMOUNT_FINAL_DATA}H", body)
        return info, readings

    def build_msg(self, node: Node, info: int, readings: list[int]):
        if len(readings) != 10:
            return None

        # Decode sensor data
        gyr = np.array(readings[0:3]) * self.gyr_const
        acc = np.array(readings[3:6]) * self.acc_const
        mag = np.array([
            readings[6] * self.mag_constxy,
            readings[7] * self.mag_constxy,
            readings[8] * self.mag_constz
        ])
        volts = (readings[9] & 0x0FFF) * self.scale_adc

        # Mode 0: Voltage only
        if self.opmode == 0:
            msg = self.MSG_TYPE()
            msg.info = info
            msg.force_z = float(volts)
            msg.fx = msg.fy = msg.fz = 0.0
            msg.rx = msg.ry = msg.rz = 0.0
            return msg

        # Offset initialization (first 100 samples)
        if not self.offset_ready:
            self.force_offset_samples.append(volts)
            if len(self.force_offset_samples) >= 100:
                self.force_offset = np.mean(self.force_offset_samples)
                self.offset_ready = True
                print("[STM32F3] Force offset initialized.")
            return None

        # Calibrated force
        force = ((volts - self.force_offset) * self.m) + self.b

        # Orientation update
        self.q = self.mahony.updateMARG(self.q, gyr=gyr, acc=acc, mag=mag)
        rot = R.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        # IMU stabilization
        if self.stabilization_start is None:
            self.stabilization_start = time.time()
            print("[STM32F3] Stabilizing IMU...")

        if not self.angle_offset_init and time.time() - self.stabilization_start > self.stabilization_time:
            self.roll_buff.append(roll)
            self.pitch_buff.append(pitch)
            self.yaw_buff.append(yaw)
            if len(self.roll_buff) >= self.angle_offset_samples:
                self.roll_offset = np.mean(self.roll_buff)
                self.pitch_offset = np.mean(self.pitch_buff)
                self.yaw_offset = np.mean(self.yaw_buff)
                self.angle_offset_init = True
                print("[STM32F3] Orientation offsets set.")

        # Compute forces after stabilization
        if self.angle_offset_init:
            self.force_components[0] = force * np.cos(pitch - self.pitch_offset) * np.cos(yaw - self.yaw_offset)
            self.force_components[1] = force * np.cos(pitch - self.pitch_offset) * np.sin(yaw - self.yaw_offset)
            self.force_components[2] = force * np.sin(pitch - self.pitch_offset)
            self.apply_plane_transform()

            msg = self.MSG_TYPE()
            msg.info = info
            msg.force_z = float(force)
            msg.fx = float(self.force_components[0])
            msg.fy = float(self.force_components[1])
            msg.fz = float(self.force_components[2])
            msg.rx = float(self.expected_rotated_force[0])
            msg.ry = float(self.expected_rotated_force[1])
            msg.rz = float(self.expected_rotated_force[2])
            return msg
        return None

    def apply_plane_transform(self):
        f = self.force_components
        p = self.plane
        if p == 0:
            self.expected_rotated_force[:] = [-f[0], -f[2], -f[1]]
        elif p == 1:
            self.expected_rotated_force[:] = [-f[1], -f[2], f[0]]
        elif p == 2:
            self.expected_rotated_force[:] = [f[0], -f[2], f[1]]
        elif p == 3:
            self.expected_rotated_force[:] = [f[1], -f[2], -f[0]]


# ---------- OpenCoRoCo ----------
class OpenCoRoCo:
    def __init__(self, handler: SocketHandler, processor: BaseProcessor):
        self.host = "0.0.0.0"
        self.port = None
        self.handler = handler
        self.processor = processor
        self.latest_frame = None
        self.info = 0
        self.running = False

    def start_server(self):
        if self.port is None:
            raise ValueError("Port must be set before starting server")
        self.handler.setup_server(self.host, self.port)
        self.running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        frame_size = self.processor.end
        while self.running:
            data = self.handler.receive()
            if not data:
                continue
            if len(data) >= frame_size:
                frame = memoryview(data)[-frame_size:]
                try:
                    self.info, self.latest_frame = self.processor.parse_frame(frame)
                except Exception as e:
                    print(f"Frame parsing error: {e}")

    def get_latest_frame(self):
        return self.latest_frame

    def stop(self):
        self.running = False
        self.handler.close()


# ---------- ROS2 Publisher ----------
class ForcestickPublisher(Node):
    def __init__(self, OpenCoRoCoClass):
        super().__init__("forcestick_pub")

        self.declare_parameter("server_port", 5000)
        self.declare_parameter("use_udp", True)
        self.declare_parameter("opmode", 1)
        self.declare_parameter("sistema", 0)
        self.declare_parameter("plane", 0)

        server_port = self.get_parameter("server_port").value
        use_udp = self.get_parameter("use_udp").value
        opmode = self.get_parameter("opmode").value
        sistema = self.get_parameter("sistema").value
        plane = self.get_parameter("plane").value

        handler = UDPHandler() if use_udp else TCPHandler()
        processor = STM32F3Processor(opmode, sistema, plane) if server_port == 5004 else STM32F4Processor()

        self.opencoroco = OpenCoRoCoClass(handler, processor)
        self.opencoroco.port = server_port
        self.opencoroco.start_server()

        self.force_publisher = self.create_publisher(processor.MSG_TYPE, "force", 10)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        frame = self.opencoroco.get_latest_frame()
        if frame is None:
            return
        info = self.opencoroco.info
        msg = self.opencoroco.processor.build_msg(self, info, frame)
        if msg is None:
            return
        msg.stamp = self.get_clock().now().to_msg()
        self.force_publisher.publish(msg)


# ---------- Main ----------
def main():
    rclpy.init()
    node = ForcestickPublisher(OpenCoRoCo)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

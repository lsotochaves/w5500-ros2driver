#!/usr/bin/env python3

import socket
import struct
import threading
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from w5500_msg.msg import Force


# ---------- Abstract base to enforce structure ----------
class SocketHandler(ABC):
    """Abstract interface for UDP/TCP socket handling."""

    @abstractmethod
    def setup_server(self, host, port):
        raise NotImplementedError

    @abstractmethod
    def receive(self):
        raise NotImplementedError

    @abstractmethod
    def close(self):
        raise NotImplementedError


# ---------- UDP implementation ----------
class UDPHandler(SocketHandler):
    def __init__(self):
        self.socket = None

    def setup_server(self, host, port):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((host, port))
            print(f"UDP server ready on {host}:{port}")
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


# ---------- TCP implementation ----------
class TCPHandler(SocketHandler):
    def __init__(self):
        self.socket = None
        self.conn = None
        self.accepting = False
        self._lock = threading.Lock()

    def setup_server(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.listen(1)
        self.socket.settimeout(1.0)
        print(f"TCP server listening on {host}:{port}")

        self.accepting = True
        threading.Thread(target=self._accept_connection, daemon=True).start()

    def _accept_connection(self):
        """Non-blocking accept loop."""
        while self.accepting:
            try:
                conn, addr = self.socket.accept()
                with self._lock:
                    self.conn = conn
                print(f"TCP client connected: {addr}")
                break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"TCP accept error: {e}")
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
    AMOUNT_FINAL_DATA = None  # to be defined in subclasses
    MSG_TYPE = None  # to be defined in subclasses

    @abstractmethod
    def parse_frame(self, frame: bytes):
        raise NotImplementedError

    @abstractmethod
    def build_msg(self, node: Node, info: int, values: list[float]):
        raise NotImplementedError


# ---------- STM32F4 Processor ----------
class STM32F4Processor(BaseProcessor):
    AMOUNT_FINAL_DATA = 7  # STM32F4 sends 7 force readings
    start = 1
    end = start + AMOUNT_FINAL_DATA * BaseProcessor.BYTES_PER_READING
    MSG_TYPE = Force

    def parse_frame(self, frame: bytes) -> tuple[int, list[float]]:
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


# ---------- STM32F3 Processor (placeholder for now) ----------
class STM32F3Processor(BaseProcessor):
    AMOUNT_FINAL_DATA = 10
    start = 1
    end = start + AMOUNT_FINAL_DATA * BaseProcessor.BYTES_PER_READING
    MSG_TYPE = Force  # placeholder until custom message is defined

    def parse_frame(self, frame: bytes) -> tuple[int, list[float]]:
        info = frame[0]
        body = memoryview(frame)[self.start:self.end]
        readings = struct.unpack(f">{self.AMOUNT_FINAL_DATA}H", body)
        scale = 3.3 / 4095.0
        scaled = [(r & 0x0FFF) * scale for r in readings]
        return info, scaled

    def build_msg(self, node: Node, info: int, values: list[float]):
        msg = self.MSG_TYPE()
        msg.info = info
        # Placeholder: adapt once message definition exists
        msg.data = values  # assuming a float[] field called "data"
        return msg


# ---------- OpenCoRoCo ----------
class OpenCoRoCo:
    def __init__(self, handler: SocketHandler, processor: BaseProcessor):
        self.host = "0.0.0.0"
        self.port = None  # Defined in launch file
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

        # Get parameters from launch file
        self.declare_parameter("server_port", 5000)
        self.declare_parameter("use_udp", True)

        # Assign parameters from launch file
        server_port = self.get_parameter("server_port").get_parameter_value().integer_value
        use_udp = self.get_parameter("use_udp").get_parameter_value().bool_value

        # Choose handler and processor based on parameters (port dictates type of microcontroller)
        handler = UDPHandler() if use_udp else TCPHandler()
        processor = STM32F3Processor() if server_port == 5004 else STM32F4Processor()

        # Instantiate OpenCoRoCo, using the hanlding, processor and server_port
        self.opencoroco = OpenCoRoCoClass(handler, processor)
        self.opencoroco.port = server_port
        self.opencoroco.start_server()

        # ---ROS2 PUBLISHER, with corresponding message type depending on microcontroller---
        self.force_publisher = self.create_publisher(processor.MSG_TYPE, "force", 10)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        frame = self.opencoroco.get_latest_frame()
        if frame is None:
            return

        info = self.opencoroco.info
        # Builds the message based on the frame and the info byte
        msg = self.opencoroco.processor.build_msg(self, info, frame)
        msg.stamp = self.get_clock().now().to_msg()  # kept outside for timing precision
        self.force_publisher.publish(msg)


# ---------- MAIN ----------
def main():
    rclpy.init()
    node = ForcestickPublisher(OpenCoRoCo)  # pass the class, not an instance
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

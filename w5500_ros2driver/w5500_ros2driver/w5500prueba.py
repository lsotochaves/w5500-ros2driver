#!/usr/bin/env python3

#https://stackoverflow.com/questions/68425239/how-to-handle-multithreading-with-sockets-in-python

import serial #Librería para manejo de puertos seriales
import signal
import sys
import time
import socket
import threading
import rclpy #Librer´ia que permite trabajar con Ros2
from rclpy.node import Node
from w5500_msg.msg import Force
number_clients = 5
  
"""
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))
s.listen(number_clients) 
"""

record = []
# openCoRoCo interface, which connects the microcontroler to
# a ROS2 node
class OpenCoRoCo(object):
    # Constructor, initialize the arguments
    def __init__(self):
        # Params of serial port connection
        self.baudrate = None
        self.bytesize = None
        self.parity = None
        self.stopbits = None
        self.timeout = None
        self.connected = False
        self.values = [0, 0, 0, 0, 0, 0, 0]
        self.all_threads = []

        # --- atributos para servidor TCP ---
        self.host = "0.0.0.0"  # escuchar en todas las interfaces
        self.port = 5000       # puerto TCP
        self.server_socket = None
        self.server_thread = None
        self.conn = None
        self.addr = None
        self.buf = bytearray[]

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
        # parece que almacena los valores de fuerza crudos leídos del puerto serial
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
        self.initialize = False #indica si ya se hizo la inicialización general del sensor
        self.offset_init = False #indica si ya se calculó y aplicó el offset
        self.init_counter = 0
        self.offset_init_counter = 0

    """
    def start_server(self):
        # Crear socket TCP
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(number_clients)
        print(f"Server listening on {self.host}:{self.port}")

        # Thread principal para aceptar clientes
        server_thread = threading.Thread(target=self.accept_clients, daemon=True)
        server_thread.start()
        self.connected = True
    
    def accept_clients(self):
        while True:
            conn, addr = self.server_socket.accept()
            print(f"Client connected: {addr}")
            t = threading.Thread(target=self.get_forces, args=(conn, addr), daemon=True)
            t.start()
            self.all_threads.append(t)
    """

    def connect_tcp(self, host="0.0.0.0", port=5000, backlog=1):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Reusar puerto si reinicias rápido
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(backlog)
        print(f"TCP server escuchando en {host}:{port} ...")
        self.conn, self.addr = srv.accept()
        print(f"Conectado con {self.addr}")
        # No bloqueante para no trabar el timer de ROS2
        self.conn.setblocking(False)
        self.buf.clear()

    # Function to get force data from serial port and
    # processes it
    """
    def get_forces(self):

        #Genera un buffer en el que se pueden manipular bytes
        buf = bytearray()
        updated = False  # estado inicial

        try:
            while True:
                received = conn.recv(1024)
                if not received:
                    break
                
                buf.extend(received)

                while len(buf) >= 14:
                    frame = buf[:14]
                    buf = buf[14:]

                    readings = []
                    for i in range(7):
                        msb = frame[2*i]       # first byte of the pair
                        lsb = frame[2*i + 1]   # second byte of the pair
                        value = (msb << 8) | lsb   # combine into uint16
                        value &= 0x0FFF            # keep only lower 12 bits
                        readings.append(value)

                    for i in range(7):
                        self.values[i] = readings[i] * (3.0 / 4095.0)

                    self.fx_my_1 = self.values[0]
                    self.fy_mx_1 = self.values[1]
                    self.fy_mx_2 = self.values[2]
                    self.fx_my_2 = self.values[3]
                    self.mz = self.values[4]
                    self.fz_1 = self.values[5]
                    self.fz_2 = self.values[6]
        
                    updated = True

        except Exception as e:
            print(f"Error en get_forces con {addr}: {e}")
        finally:
            return updated
        """
    def get_forces(self):
        updated = False

        if self.conn is None:
            return False  # aún no hay conexión

        try:
            try:
                chunk = self.conn.recv(1024)
                if chunk == b'':                 # peer cerró
                    self.conn.close()
                    self.conn = None
                    self.buf.clear()
                    return False
            except (BlockingIOError, TimeoutError):
                chunk = b''
            except ConnectionResetError:
                self.conn = None
                self.buf.clear()
                return False

            if chunk:
                self.buf.extend(chunk)

            # Procesa todos los frames completos disponibles
            while len(self.buf) >= 14:
                frame = self.buf[:14]
                del self.buf[:14]

                readings = []
                for i in range(7):
                    msb = frame[2*i]
                    lsb = frame[2*i + 1]
                    val = ((msb << 8) | lsb) & 0x0FFF
                    readings.append(val)

                for i in range(7):
                    self.values[i] = readings[i] * (3.0 / 4095.0)

                self.fx_my_1 = self.values[0]
                self.fy_mx_1 = self.values[1]
                self.fy_mx_2 = self.values[2]
                self.fx_my_2 = self.values[3]
                self.mz      = self.values[4]
                self.fz_1    = self.values[5]
                self.fz_2    = self.values[6]

                updated = True

        except Exception as e:
            print(f"Error en get_forces con {self.addr}: {e}")

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
            # Calcula el tiempo relativo desde que se inició el nodo
            # Lo convierte en string y le agrega una coma
            # Este será el comienzo de una línea de datos
            record_data += ",".join(
                str(value) for value in self.opencoroco.values
            )
            # Recorre la lista self.opencoroco.values 
            # que contiene los 7 valores crudos de fuerza
            # record_data queda tipo "0.123456,10,20,30,40,50,60,70"
            force_msg = Force()
            force_msg.fx_my_1 = self.opencoroco.fx_my_1
            force_msg.fy_mx_1 = self.opencoroco.fy_mx_1
            force_msg.fy_mx_2 = self.opencoroco.fy_mx_2
            force_msg.fx_my_2 = self.opencoroco.fx_my_2
            force_msg.mz = self.opencoroco.mz
            force_msg.fz_1 = self.opencoroco.fz_1
            force_msg.fz_2 = self.opencoroco.fz_2

            self.force_publisher.publish(force_msg)
            # Publica el mensaje force_msg en el tópico "force".
            # Cualquier nodo suscrito a "force" recibirá estos datos en tiempo real
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
            # Manda un mensaje al log de ROS2 mostrando los valores publicados
            # self.get_logger().info(...) imprime en consola con nivel INFO
            
# Function to store the recorded force data in a csv file
def output_record(record):
    #record = record[:1000]
    with open("output.csv", "w") as f:
        f.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")
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

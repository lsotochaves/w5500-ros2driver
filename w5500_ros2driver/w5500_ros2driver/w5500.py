#!/usr/bin/env python3

import serial #Librería para manejo de puertos seriales
import signal
import sys
import time
import socket
import rclpy #Librer´ia que permite trabajar con Ros2
from rclpy.node import Node
from w5500_msg.msg import Force

MODE_UDP = False
SERVER_COMMUNICATION_PORT = 5000
INFO_BYTE_AMOUNT = 1
HOST = "0.0.0.0"  #escuchar en todas las interfaces
SIZE_SINGLE_BATCH = 15 #Por el momento, debe de ser un calculo.
AMOUNT_FORCE_READINGS = 7
AMOUNT_READINGS_PER_BATCH = 10
SIZE_TOTAL_PAYLOAD = SIZE_SINGLE_BATCH * AMOUNT_READINGS_PER_BATCH

record = []
# openCoRoCo interface, which connects the microcontroler to
# a ROS2 node
class OpenCoRoCo(object):
    # Constructor, initialize the arguments
    def __init__(self):

        # --- Communication params ---
        self.host = HOST
        self.port = SERVER_COMMUNICATION_PORT #puerto 
        self.mode = MODE_UDP
        self.server_socket = None
        self.conn = None
        self.buf = bytearray()
        self.batch_member = 0

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


    '''Va a contener la funcion de procesar los datos de informcion
    def process_info(self):

    def calculate_udp_forces(self...):


    def calculate_tcp_forces(self...):

    '''


    def get_forces(self):

        updated = False
        if self.server_socket is None:
            return False

        elif self.mode: #Procesamiento datos UDP
            try:
                data, addr = self.server_socket.recvfrom(1024)
                if not data:
                    return None
                self.addr = addr  # save last sender
            except (BlockingIOError, TimeoutError):
                return False

            # At this point, 'data' is already one full datagram
            if len(data) < SIZE_SINGLE_BATCH:
                return False  # ignore incomplete packet

            frame = bytearray(data[:SIZE_SINGLE_BATCH])

            information = frame[:INFO_BYTE_AMOUNT]
            del frame[:INFO_BYTE_AMOUNT]
            self.batch_member += 1


            readings = []
            for i in range(AMOUNT_FORCE_READINGS):
                msb = frame[2*i]
                lsb = frame[2*i + 1]
                val = ((msb << 8) | lsb) & 0x0FFF
                readings.append(val)

            for i in range(AMOUNT_FORCE_READINGS):
                self.values[i] = readings[i] * (3.0 / 4095.0)

            updated = True
            return updated

        #Procesamiento datos TCP/IP
        else: 

            if self.conn is None:
                return None

            data = self.conn.recv(1024)
            self.buf.extend(data)

            while len(self.buf) >= SIZE_SINGLE_BATCH:
                frame = self.buf[:SIZE_SINGLE_BATCH] #Obtiene el tamaño de un paquete de datos
                del self.buf[:SIZE_SINGLE_BATCH]

                information = frame[:INFO_BYTE_AMOUNT]
                del frame[:INFO_BYTE_AMOUNT]
                self.batch_member += 1 #En este punto, frame contiene el dato dentro de si.

                readings = []
                for i in range(AMOUNT_FORCE_READINGS):
                    msb = frame[2*i]
                    lsb = frame[2*i + 1]
                    val = ((msb << 8) | lsb) & 0x0FFF
                    readings.append(val)

                for i in range(AMOUNT_FORCE_READINGS):
                    self.values[i] = readings[i] * (3.0 / 4095.0)

                updated = True
                
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

        import os
        self.csv_path = "output.csv"  # usa misma ruta que el script de matplotlib
        self.csv_file = open(self.csv_path, "w", buffering=1)  # line buffering
        
        self.csv_file.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")

    # Function to publish the force data processed through
    # the topic using the custom force_msg format
    # Also save the force data to a buffer (list)
    def timer_callback(self):
        if self.opencoroco.get_forces():

            if (self.opencoroco.batch_member == 1):
                record_data = str(time.time() - self.init_time) + ","

            else:
                record_data = "--------,"
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
            force_msg.fx_my_1 = float(self.opencoroco.values[0])
            force_msg.fy_mx_1 = float(self.opencoroco.values[1])
            force_msg.fy_mx_2 = float(self.opencoroco.values[2])
            force_msg.fx_my_2 = float(self.opencoroco.values[3])
            force_msg.mz      = float(self.opencoroco.values[4])
            force_msg.fz_1    = float(self.opencoroco.values[5])
            force_msg.fz_2    = float(self.opencoroco.values[6])


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

            self.csv_file.write(record_data + "\n")
            if self.opencoroco.batch_member == AMOUNT_READINGS_PER_BATCH:
                self.opencoroco.batch_member = 0

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
    opencoroco.start_server()
    rclpy.init()
    force_joint = ForcestickPublisher(opencoroco)
    try:
        rclpy.spin(force_joint)
    finally:
        # << NUEVO: cerrar CSV al terminar
        force_joint.csv_file.close()
        force_joint.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import socket
import time
import signal
import sys

# ---- Config ----
MODE_UDP = False              # True = UDP, False = TCP
SERVER_COMMUNICATION_PORT = 5000
INFO_BYTE_AMOUNT = 1
HOST = "0.0.0.0"
SIZE_SINGLE_BATCH = 15        # 1 info + 14 data (7 readings * 2 bytes)
AMOUNT_FORCE_READINGS = 7
AMOUNT_READINGS_PER_BATCH = 10

record = []   # for CSV storage

class ForceServer:
    def __init__(self, mode=False):
        self.host = HOST
        self.port = SERVER_COMMUNICATION_PORT
        self.mode = mode
        self.server_socket = None
        self.conn = None
        self.addr = None
        self.buf = bytearray()
        self.batch_member = 0
        self.values = [0.0] * AMOUNT_FORCE_READINGS

    def start(self):
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

        # UDP
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

        # TCP
        else:
            if self.conn is None:
                return None
            data = self.conn.recv(1024)
            if not data:   # closed
                return None
            self.buf.extend(data)

            while len(self.buf) >= SIZE_SINGLE_BATCH:
                frame = self.buf[:SIZE_SINGLE_BATCH]
                del self.buf[:SIZE_SINGLE_BATCH]

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


# --- CSV save ---
def output_record(record):
    with open("output.csv", "w") as f:
        f.write("Timestamp,fxmy1,fymx1,fymx2,fxmy2,mz,fz1,fz2\n")
        for item in record:
            f.write(f"{item}\n")

# --- signal handler ---
def terminate_handler(signum, frame):
    print("\nCaught signal:", signum)
    print("Record size:", len(record))
    output_record(record)
    sys.exit()

signal.signal(signal.SIGINT, terminate_handler)
signal.signal(signal.SIGTERM, terminate_handler)


def main():
    server = ForceServer(mode=MODE_UDP)
    server.start()
    start_time = time.time()

    while True:
        if server.get_forces():
            if server.batch_member == 1:
                ts = time.time() - start_time
                record_data = str(ts) + ","
            else:
                record_data = "--------,"

            record_data += ",".join(str(v) for v in server.values)
            print(record_data)  # print to console
            record.append(record_data)

            if server.batch_member == AMOUNT_READINGS_PER_BATCH:
                server.batch_member = 0


if __name__ == "__main__":
    main()

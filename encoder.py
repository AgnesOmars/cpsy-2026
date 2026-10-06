import socket
import subprocess
import time

class Encoders:
    def __init__(self, left_pins, right_pins, decoder="./decoder", port=5005):
        args = [decoder, *map(str, left_pins), *map(str, right_pins)]
        self.proc = subprocess.Popen(args)
        time.sleep(0.5)  # give the decoder time to start

        self.addr = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def read(self):
        self.sock.sendto(b"get", self.addr)
        data, _ = self.sock.recvfrom(64)
        left, right = map(int, data.split())
        return left, right

    def close(self):
        self.proc.terminate()  # sends SIGTERM, which the C++ handler catches
        self.proc.wait()
        self.sock.close()

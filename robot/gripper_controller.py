import serial
import time

class GripperController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        print("[GripperController] Connected to ESP32")

    def clamp(self):
        self.send_command("clamp")

    def unclamp(self):
        self.send_command("unclamp")

    def send_command(self, cmd):
        self.ser.write((cmd + "\n").encode('utf-8'))
        print(f"[GripperController] Sent: {cmd}")
        time.sleep(2.5)

if __name__ == "__main__":
    controller = GripperController()
    controller.clamp()
    controller.unclamp()
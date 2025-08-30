import serial
import time

class GripperController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        print("[GripperController] Connected to ESP32")

    def clamp(self):
        self.send_command("clamp")

    def unclamp(self):
        self.send_command("unclamp")

    def scan(self):
        self.send_command("scan")

    def send_command(self, cmd):
        self.ser.write((cmd + "\n").encode('utf-8'))
        print(f"[GripperController] Sent: {cmd}")

    def is_command_complete(self):
        """Non-blocking check whether scan has completed."""
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                print(f"[GripperController] Received: {line}")
                if "DONE" in line:
                    return True
        return False
    
    def await_command_complete(self):
        while not self.is_command_complete():
            continue

if __name__ == "__main__":
    controller = GripperController()
    # controller.clamp()
    # controller.unclamp()
    # controller.clamp()
    controller.scan()
import serial
import time

class XArm:
    def __init__(self):
        print("[LOG] Initializing serial connection...")
        PORT = 'COM8'
        BAUDRATE = 115200
        print(f"[LOG] Opening port {PORT} at {BAUDRATE} baud...")
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("[LOG] Serial port opened successfully.")
        print("[LOG] Waiting 2 seconds for ESP32 to initialize...")
        time.sleep(2)
        print("[LOG] ESP32 is ready.")

    def send_command(self, command):
        print(f"[LOG] Sending command: {command.strip()}")
        self.ser.write(command.encode('utf-8'))

    def send_multiple_commands(self, commands):
        full_command = "".join(cmd + "\r\n" for cmd in commands)
        self.send_command(full_command)

    def close_xarm_esp32(self):
        print("[LOG] Moving servos to neutral position before closing...")
        self.send_multiple_commands([
            "bus_servo.run(1,500,10)",
            "bus_servo.run(2,500,10)",
            "bus_servo.run(3,500,10)",
            "bus_servo.run(4,500,10)",
            "bus_servo.run(5,500,10)",
            "bus_servo.run(6,500,10)"
        ])
        print("[LOG] Waiting 2 seconds for movement...")
        time.sleep(2)
        print("[LOG] Closing serial connection...")
        self.ser.close()
        print("[LOG] Serial connection closed.")

    def camera_position_ready(self):
        print("[LOG] Moving arm to camera-ready position...")
        self.send_multiple_commands([
            "bus_servo.run(1,500,10)",
            "bus_servo.run(2,500,10)",
            "bus_servo.run(3,200,10)",
            "bus_servo.run(4,750,10)",
            "bus_servo.run(5,500,10)",
            "bus_servo.run(6,500,10)"
        ])
        print("[LOG] Waiting 2 seconds for movement...")
        time.sleep(2)
        print("[LOG] Arm is now in camera-ready position.")

    def get_servo_positions(self):
        print("[LOG] Getting servos positions...")
        cmds = [
            "bus_servo.get_position(1)",
            "bus_servo.get_position(2)",
            "bus_servo.get_position(3)",
            "bus_servo.get_position(4)",
            "bus_servo.get_position(5)",
            "bus_servo.get_position(6)",
        ]
        responses = []
        result = []

        for cmd in cmds:
            self.ser.write((cmd + "\r\n").encode('utf-8'))
            time.sleep(0.05)

            echo = self.ser.readline()
            response = self.ser.readline()

            if response:
                decoded = response.decode('utf-8', errors='ignore').strip()
                responses.append(decoded)
            else:
                responses.append("No response")

        print("Response :")
        for i, r in enumerate(responses, start=1):
            print(f"  Servo {i} : {r}")
            result.append(int(r))
        
        return result
import serial
import time

serialInst = serial.Serial('COM6', 9600, timeout=1)
last_command = None

def arduino_connect(port='COM6', baudrate=9600, timeout=1):
    global serialInst
    try:
        serialInst = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        return True
    except serial.SerialException as e:
        return False

def arduino_send_command(angle_correction):
    global last_command
    
    if serialInst is None or not serialInst.is_open:
        return

    try:
        x = int(angle_correction[0])
    except (ValueError, TypeError):
        print("Valeur invalide.")
        return

    command = f"{x}\n"

    if command == last_command:
        return

    last_command = command
    print(f"Send to Arduino : {command.strip()}")
    serialInst.write(command.encode('utf-8'))

def arduino_disconnect():
    global serialInst
    if serialInst is not None and serialInst.is_open:
        serialInst.close()
    serialInst = None
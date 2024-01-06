import serial
import time

ser = serial.Serial('COM5', 9600)  # Replace 'COMX' with your Arduino port

try:
    while True:
        servo_number = input("Enter servo number (9, 10, 11), or 'stop' to end: ")

        if servo_number.lower() == 'stop':
            break

        angle = input("Enter servo angle (0-180): ")

        command = f"{servo_number} {angle}"
        ser.write(command.encode())
        time.sleep(1)  # Give Arduino time to process

finally:
    ser.close()

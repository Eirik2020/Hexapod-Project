import serial
import time

# Define serial port and baudrate
ser = serial.Serial('COM6', 9600)  # Replace 'COMX' with the actual COM port of your Maestro

# Define function to send command to set servo position
def set_servo_position(servo_number, position):
    command = bytearray([0x84, servo_number, position & 0x7F, (position >> 7) & 0x7F])
    ser.write(command)

# Set servo 0 to 90 degrees
set_servo_position(0, 90)

# Close serial port
ser.close()
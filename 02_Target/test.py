import serial
import numpy as np
import com as com
import time
import kin as ks

# Set-up
# Open the serial port (replace 'COM3' with your port, e.g., '/dev/ttyUSB0' on Linux)
ser = serial.Serial('COM3', 9600, timeout=1)

# Initialize legs
legs = com.initialize_legs(6)


### MAIN PROGRAM ###
# Configuration
P_def = np.array([ 125, 0, -73.5 ])
V_D = np.array([ 0, 40, 0 ])
h = 30
n = 100
delay = 0.000000005
steps = 4


# Execute command
ks.generate_walk(P_def, V_D, h, n, legs)
ks.walk(ser, legs, steps, delay)


# Close the serial port when done
ser.close()
# Packages
import serial
import numpy as np
from PyMaestro import set_target_mini_ssc
from kinematics import leg_IK, subdivide_vector
import time


# Serial Settings
COM_port = "COM6" # USB COM port connected to servo controller.
baud_rate = 9600  # Baud rate.
ser = serial.Serial(COM_port, 9600, timeout=1) # Open Serial port


# Input Parameters
origin = [  0,   0,   42 ]         # ( x, y, z )
links =  [  0,  54.0, 71.0, 73.5 ] # Link lengths [mm]
offset = [ 45,  45,  135 ]         # Servo position offsets [deg]


# Data conversion
origin = np.array(origin)
print(origin)
links = np.array(links)
offset = np.deg2rad(np.array(offset))



velocity = 100 #[mm/s]
start_point =  np.array([  0, 125,  -31.5 ])       # [ x, y, z ] 
vector = np.array([0, 0, -30])
n = 254


# Find wait time between messages.
magnitude = np.linalg.norm(vector)
total_time = magnitude / velocity
n_time = total_time / n
n_time = n_time.item()

# Discrecize vectors
points = subdivide_vector(start_point, vector, n)

for point in points:
   # Find leg angles
   leg_angles = leg_IK(origin, point, links, offset) 
   # Convert to target values
   target_value = ( (leg_angles / (np.pi/2) ) * 254 ).astype(int)
   set_target_mini_ssc(ser, 9, target_value[0]) 
   set_target_mini_ssc(ser, 10, target_value[1]) 
   set_target_mini_ssc(ser, 11, target_value[2]) 
   time.sleep(n_time)






ser.close() # Close the serial connection
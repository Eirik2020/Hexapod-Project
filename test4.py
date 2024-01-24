# Packages
import serial
import numpy as np
from PyMaestro import set_target_mini_ssc, send_path
from kinematics import leg_IK, subdivide_vector, rotate_about_z, leg_path
import time
np.set_printoptions(precision=4, suppress=True)


# Functions



# Classes
class Leg:
  def __init__(self, number, origin_angle ):
    self.number = number
    self.origin_angle = origin_angle
    self.servos = None
    self.swing_path = None
    self.stance_path = None
    self.swing_angles = None
    self.stance_angles = None
    self.current_pos = None
    self.current_angles = None



###   MAIN PROGRAM   ###
##  SET-UP  ##
# Serial Settings
COM_port = "COM6" # USB COM port connected to servo controller.
baud_rate = 9600  # Baud rate.
ser = serial.Serial('COM6', 9600, timeout=1)
#ser.close() # Close the serial connection

# Robot Parameters
leg_origin_radius = 72.75 #[mm] # REMOVE AND ADD TO LINKS
number_of_legs = 6
origin = np.array([  0,   0,   0 ])
links = np.array([  72.75,  54.0, 71.0, 73.5 ]) # Link lengths [mm])
offsets = np.array([ 45,  45,  135 ])
offsets = np.deg2rad(offsets)
P_start = np.array([ 0, 125, -73.5 ]) # CHANGE TO DEFAULT LEG POSITION

# Move vector:
V_D = np.array([ 30, 0, 0 ]) #[mm] Direction vector
h = 30 #[mm] Left lifting height.
n = 100 # Subdivisions


##  Generate leg list  ##
# Determine leg spacing
leg_spacing = (2*np.pi) / number_of_legs

# Generate list of legs
legs = []
for i in range(number_of_legs):
 leg = Leg(i, i*leg_spacing)
 print(i*leg_spacing)
 legs.append(leg)

# Assign servos to legs
legs[0].servos = np.array([  0,  1,  2 ]) # Assign servo 0-2 to leg 0.
legs[1].servos = np.array([  3,  4,  5 ]) # Assign servo 3-5 to leg 1.
legs[2].servos = np.array([  6,  7,  8 ]) # Assign servo 6-8 to leg 2.
legs[3].servos = np.array([  9, 10, 11 ]) # Assign servo 9-11 to leg 3.
legs[4].servos = np.array([ 12, 13, 14 ]) # Assign servo 12-14 to leg 4.
legs[5].servos = np.array([ 15, 16, 17 ]) # Assign servo 15-17 to leg 5.

# Assign default position and angles
for i in range(number_of_legs):
   legs[i].current_pos = P_start
   legs[i].current_angles = leg_IK(origin, P_start, links, offsets)



##  MOVEMENT  ##
# Generate path and angles.
for i in range(number_of_legs):
   # Generate path coordinates
   path = leg_path(P_start, V_D, legs[i].origin_angle, h, n)
   legs[i].swing_path = path[0]
   legs[i].stance_path = path[1]

   # Convert to angles
   legs[i].swing_angles = []
   for j in range(len(legs[i].swing_path)):
      angles_IK = leg_IK(origin, legs[i].swing_path[j], links, offsets)
      legs[i].swing_angles.append(angles_IK)
   
   legs[i].stance_angles = []
   for j in range(len(legs[i].stance_path)):
      angles_IK = leg_IK(origin, legs[i].stance_path[j], links, offsets)
      legs[i].stance_angles.append(angles_IK)


# Target Legs
legs_T1 = [0, 2, 4]
legs_T2 = [1, 3, 5]

delay = 0

for i in range(20):
   #send_path(ser, legs, True, delay)
   #send_path(ser, legs, False, delay)


ser.close() # Close the serial connection
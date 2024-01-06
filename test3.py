# Packages
import serial
import numpy as np
from PyMaestro import set_target_mini_ssc
from kinematics import leg_IK, subdivide_vector, rotate_about_z
import time
np.set_printoptions(precision=4, suppress=True)


# Functions
def leg_path(P_start, V_D, leg_angle, lift, n):
      # Generate coordinates
      V_D = rotate_about_z(V_D, leg_angle)
      P_end = P_start + V_D
      V_lift = np.array([ 0, 0, lift ])
      P_start_lift = P_start + V_lift
      P_end_lift = P_end + V_lift

      # Generate path
      #points = []
      # Lift leg
      points = subdivide_vector(P_start, V_lift, n)
      #points[0] = np.concatenate((points, points_buffer)) # Add move points to list.

      # Move leg
      points_buffer = subdivide_vector(P_start_lift, V_D, n)
      points = np.concatenate((points, points_buffer)) # Add move points to list.

      # Lower leg.
      points_buffer = subdivide_vector(P_end_lift, -V_lift, n)
      points = np.concatenate((points, points_buffer)) # Add move points to list.

      # Return leg to default position
      points_buffer = subdivide_vector(P_end, -V_D, n)
      points = np.concatenate((points, points_buffer)) # Add move points to list.

      return points


def send_path(ser, legs, delay):
   leg = legs[0]
   print(leg.servos)

   # Extract leg information
   for i in range(len(leg.angles)):
      # Send target information
      for j in range(len(legs)):
         angles = legs[j].angles
         target = legs[j].servos
         # Convert to target values
         target_value = ( (angles[i] / (np.pi/2) ) * 254 ).astype(int)
         set_target_mini_ssc(ser, target[0], target_value[0]) 
         set_target_mini_ssc(ser, target[1], target_value[1]) 
         set_target_mini_ssc(ser, target[2], target_value[2]) 
      
      # Delay
      time.sleep(delay)


# Classes
class Leg:
  def __init__(self, number, origin_angle ):
    self.number = number
    self.origin_angle = origin_angle
    self.servos = None
    self.path = None
    self.angles = None
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
V_D = np.array([ 40, 0, 0 ]) #[mm] Direction vector
h = 30 #[mm] Left lifting height.
n = 100 # Subdivisions


##  Generate leg list  ##
# Determine leg spacing
leg_spacing = (2*np.pi) / number_of_legs

# Generate list of legs
legs = []
for i in range(number_of_legs):
 leg = Leg(i, i*leg_spacing)
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
   midpoint = number_of_legs / 2
   
   legs[i].path = leg_path(P_start, V_D, legs[i].origin_angle, h, n)

   # Convert to angles
   legs[i].angles = []
   for j in range(len(legs[i].path)):
      angles_IK = leg_IK(origin, legs[i].path[j], links, offsets)
      legs[i].angles.append(angles_IK)



send_path(ser, legs, 0.01)

ser.close() # Close the serial connection
# Packages
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time


def leg_IK(origin_bot, point, links, leg_angle):
    """
    # Hexapod Leg Inverse-Kinematics function
    # Author: Eirik Strandman
    # Version: V1.0

    # Description:
    A function that takes in the robot origin, leg configuration and desired 
    end-effector location and estimates joint angles for a 2 member hexapod length.


    # Inputs
    - origin_bot: (x,y,z)[mm] - The origin of the robot coordinate system.
    - point: (x,y,z)[mm] - Desired End-Effector (EF) location.
    - links: [L0, L1, L2, L3][mm] - The link lengths of the robot leg.
    - leg_angle: [rad] - The static default angle of the leg relative to the robot body.    

    # Output
    - [J0_ang, J1_ang, J2_ang, J3_ang][rad] - Numpy array containing estimated joint angles.
    """


    # Calculate leg origin relative to robot origin leg angle.
    origin_leg = np.array([
         (links[0]*np.cos(leg_angle) + origin_bot[0]), # x
         (links[0]*np.sin(leg_angle) + origin_bot[1]), # y
         (origin[2]) ])                                # z

    # Translate to local leg coordinate system
    point_leg = point - origin_leg 
 
    # Find joint angle for J1
    J1_ang = np.arctan2(point_leg[0], point_leg[1]) # atan(x,y), note: x and y are reversed so x-axis points forward.

    # Find 2D resultant vector H for P_x and P_y with Link 1 subtracted.
    H = np.sqrt( (point_leg[0]**2) + (point_leg[1]**2) ) - links[1]

    # Find resultant vector L from H and Z. 
    L = np.sqrt( (H**2) + (point_leg[2]**2) )

    # Find joint angle J3
    J3_ang =  np.arccos( ( (links[2]**2) + (links[3]**2) - (L**2) ) / (2 * links[2] * links[3]))
    
    # Find joint angle J2
    B = np.arccos( ( L**2 + (links[2]**2) - (links[3]**2) ) / ( 2 * L * links[2]))
    A = np.arctan2( point_leg[2], H )
    J2_ang = B + (-A)

    # Apply offsets
    J0_ang = leg_angle
    J1_ang = J1_ang
    J2_ang = np.pi/2 - J2_ang 
    J3_ang = J3_ang - np.pi

    # Store as numpy array
    angles = np.array([J0_ang, J1_ang, J2_ang, J3_ang])
    return angles

def leg_path_arch(start_point, direction_vector, height, num_points):
    t = np.linspace(0, 1, num_points)
    x = start_point[0] + direction_vector[0] * t
    y = start_point[1] + direction_vector[1] * t
    z = start_point[2] + direction_vector[2] * t - 0.5 * height * t ** 2
    return np.column_stack((x, y, z))

def leg_path_return(start, end, n):
    t = np.linspace(0, 1, n)
    x = start[0] + (end[0] - start[0]) * t
    y = start[1] + (end[1] - start[1]) * t
    z = start[2] + (end[2] - start[2]) * t
    return np.column_stack((x, y, z))

def set_target_mini_ssc(ser, channel, target):
  # Mini-SSC protocol: 0xFF, channel address, 8-bit target
  command = bytearray([0xFF, channel, target])

  # Send command to Maestro
  ser.write(command)

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

""" MAIN CODE """
# Parameters
origin = np.array([ 0, 0, 0 ])
angles = np.array([ 0, 0, np.pi/3, -np.pi/2 ])
links  = np.array([ 0, 50, 50, 50 ])
offset = np.array([ 0, 0, 0 ])
EF_pos  = np.array([ 0, 100, -50])
n = 30
V_D = np.array([-30, -30, 0])
height = 25
leg_angle = 0
num_leg = 6


# Generate Path
step_path = leg_path_arch(EF_pos, V_D, height, n)                           # Step path
print(step_path)
lean_path = leg_path_return(step_path[-1], step_path[0], n) # Lean path
foot_path = np.concatenate((step_path, lean_path), axis=0)   # Combined foot path
#foot_path = step_path

# Generate leg angles
leg_angles = []
for i in range(len(foot_path)):
    leg_angles.append(leg_IK(origin, foot_path[i], links, leg_angle))

# Convert the list of angles to a NumPy array
leg_angles = np.array(leg_angles)


for leg_ID in range(num_leg):
   if (leg_ID % 2 == 0):
      lean_path

# path --> leg_angles for leg --> leg_bits --> send message.
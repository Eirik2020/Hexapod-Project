import numpy as np
#from PyMaestro import set_target_mini_ssc, send_path
#from kinematics import leg_IK, subdivide_vector, rotate_about_z, leg_path
#import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R

def leg_path_arch(movement_vector, n):
    t = np.linspace(0, 1, n)
    x = movement_vector[0] * t
    y = movement_vector[1] * t
    z = movement_vector[2] * t - 0.5 * t ** 2  
    return np.column_stack((x, y, z))


def leg_path_return(start, end, n):
    t = np.linspace(0, 1, n)
    x = start[0] + (end[0] - start[0]) * t
    y = start[1] + (end[1] - start[1]) * t
    z = start[2] + (end[2] - start[2]) * t
    return np.column_stack((x, y, z))


def leg_FK(origin, angles, links, offset):
    # Remove offsets
    ang0 = angles[0]
    ang1 =  (angles[1] - offset[0])
    ang2 = (offset[1] - angles[2])
    ang3 = -(offset[2] - angles[3])

    # x-values
    x2 = links[0] + links[1]
    x3 = x2 + links[2]*np.cos(ang2)
    x4 = x3 + links[3]*np.cos(ang2 + ang3)

    # z-values
    z2 = origin[2] + links[2]*np.sin(ang2)
    z3 = z2 + links[3]*np.sin(ang2 + ang3)

    # Calculate position
    J1 = np.array([ links[0]*np.cos(ang0), links[0]*np.sin(ang0), origin[2] ])
    J2 = np.array([ x2*np.cos(ang0 + ang1), x2*np.sin(ang0 + ang1), origin[2] ])
    J3 = np.array([ x3*np.cos(ang0 + ang1), x3*np.sin(ang0 + ang1), z2 ])
    EF = np.array([ x4*np.cos(ang0 + ang1), x4*np.sin(ang0 + ang1), z3 ])


    # Return leg position as 2D array.
    leg_pos = np.array([J1, J2, J3, EF])
    return leg_pos


def leg_IK(robot_origin, point, links, offset, leg_angle, heading):
    # Calculate leg origin
    leg_origin = np.array([
         (links[0]*np.cos(leg_angle + heading) + robot_origin[0]), # x
         (links[0]*np.sin(leg_angle + heading) + robot_origin[1]), # y
         (origin[2]) ])                                            # z

    # Translate to local leg coordinate system
    point_leg = point - leg_origin

    # Find joint angle J1
    J1 = np.arctan2(point_leg[0], point_leg[1])

    # Calculate H position vector, subtract the length of L1
    H = np.sqrt((point_leg[0]**2) + (point_leg[1]**2)) - links[1]

    # Find resultant vector L from H and Z. 
    L = np.sqrt((H**2) + (point_leg[2]**2))

    # Find joint angle J3
    J3 =  math.acos( ( links[2]**2 + links[3]**2 - (L**2)) / (2 * links[2] * links[3]))
    
    # Find joint angle J2
    beta = np.arccos( ( L**2 + (links[2]**2) - (links[3]**2) ) / ( 2 * links[2] * L ))
    alpha = np.arctan2( point_leg[2], H )
    J2 = (beta  + (alpha) )

    # Apply offsets
    J0 = leg_angle
    J1 = offset[0] + J1
    J2 = offset[1] - J2
    J3 = offset[2] - J3

    return np.array([J0, J1, J2, J3])

def initialize_legs(num_legs, origin, links, start_angles, offset ):
    # Calculate leg spacing
    leg_spacing = 2*np.pi / num_legs

    # Initialize an empty list to hold the instances
    instances = []

    # Add legs to list
    for i in range(num_legs): # Adjust the range as needed
        start_angles[0] = leg_spacing * i
        pos = leg_FK(origin, links, start_angles, offset)
        instance = leg_data(start_angles, pos)
        instances.append(instance)

    # Convert the list of instances to a NumPy array
    leg_array = np.array(instances, dtype=object)
    return leg_array


class leg_data:
    def __init__(self, angles, pos):
        # Leg configuration
        self.angles = angles #[rad] - [ (Leg origin angle), (Coxa), (Femur), (Tibia) ] - Joint angles.

        # Joint Positions
        self.J1_pos = pos[0] #[rad] - [x, y, z] - Leg origin position.
        self.J2_pos = pos[1] #[rad] - [x, y, z] - Femur joint position.
        self.J3_pos = pos[2] #[rad] - [x, y, z] - Tibia joint position.
        self.EF_pos = pos[3] #[rad] - [x, y, z] - End-Effector/Foot position.



# Leg configuration
links = np.array([  72.75,  54.0, 71.0, 73.5 ]) #[mm]  - [ (Leg origin radius), (Coxa), (Femur), (Tibia) ] - Length of links between joints.
num_leg = 6
origin = np.array([  0,   0,   0 ])
offsets = np.array([ 45,  45,  135 ])
offsets = np.deg2rad(offsets)
P_start = np.array([ 0, 125, -73.5 ]) # CHANGE TO DEFAULT LEG POSITION
ang_start = np.array([ 0,  np.pi/4, np.pi/4, np.pi/4])

# Generate 3-point arch points
n = 100
V_D = np.array([30, 0, 10]) #[x, y, z] - [Forward movement, Sideways movement, Height of step]

# Convert the list of instances to a NumPy array
leg_array = initialize_legs(num_leg, origin, links, ang_start, offsets )



# Generate Path
step_path = leg_path_arch(V_D, n)                           # Step path
lean_path = leg_path_return(step_path[-1], step_path[0], n) # Lean path
foot_path = np.concatenate((step_path, lean_path), axis=0)   # Combined foot path

"""
# Generate leg angles
leg_angles = []
for i in range(len(foot_path)):
    leg_angles.append(leg_IK(origin, foot_path[i], links, offset, 0, 0))

# Convert the list of angles to a NumPy array
leg_angles = np.array(leg_angles)


# Generate joint positions
joint_pos = np.array([])
for i in range(len(leg_angles)):
    joint_pos = np.append(joint_pos, leg_FK(origin, links, leg_angles[i], offset))
"""

"""
print(P_start)
angus = leg_IK(origin, P_start, links, offset, 0, 0)
print(np.rad2deg(angus))
"""
angles = leg_IK(origin, P_start, links, offsets, 0, 0)
pointus = leg_FK(origin, angles, links, offsets)
#print("-------------")
print(pointus)
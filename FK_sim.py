# Packages
import numpy as np
from PyMaestro import set_target_mini_ssc, send_path
from kinematics import leg_IK, subdivide_vector, rotate_about_z, leg_path
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R




class leg_data:
    def __init__(self, angles, pos):
        # Leg configuration
        self.angles = angles #[rad] - [ (Leg origin angle), (Coxa), (Femur), (Tibia) ] - Joint angles.

        # Joint Positions
        self.J1_pos = pos[0] #[rad] - [x, y, z] - Leg origin position.
        self.J2_pos = pos[1] #[rad] - [x, y, z] - Femur joint position.
        self.J3_pos = pos[2] #[rad] - [x, y, z] - Tibia joint position.
        self.EF_pos = pos[3] #[rad] - [x, y, z] - End-Effector/Foot position.


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


def leg_FK(origin, links, angles, offset):
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



def leg_path(foot_pos, V_dir, points):
    # Add rotation to compensate for coordinate system of leg.
    one_third_points = points // 500
    points = one_third_points * 3

    V_leg = V_dir - foot_pos

    ver_path = np.array([ np.full(one_third_points, foot_pos[0]), 
                        np.full(one_third_points, foot_pos[1]), 
                        np.linspace(foot_pos[2], V_leg[2], one_third_points) ]) 

    hor_path = np.array([ np.linspace(foot_pos[0], V_leg[0], one_third_points),
                        np.linspace(foot_pos[1], V_leg[1], one_third_points),
                        np.full(one_third_points, V_leg[2]) ])

    # Combine the arrays into one long array
    raise_leg_path = np.concatenate((ver_path, hor_path, -ver_path))
    leg_lean_path = np.array([ np.linspace(V_leg[0], foot_pos[0], points),
                            np.linspace(V_leg[1], foot_pos[1], points),
                            np.full(points, foot_pos[2]) ])
    
    leg_path = [raise_leg_path, leg_lean_path]
    return leg_path

    
 


# Leg configuration
links = np.array([  72.75,  54.0, 71.0, 73.5 ]) #[mm]  - [ (Leg origin radius), (Coxa), (Femur), (Tibia) ] - Length of links between joints.
num_leg = 6
origin = np.array([  0,   0,   0 ])
offsets = np.array([ 45,  45,  135 ])
offsets = np.deg2rad(offsets)
P_start = np.array([ 0, 125, -73.5 ]) # CHANGE TO DEFAULT LEG POSITION
ang_start = np.array([ 0,  np.pi/4, np.pi/4, np.pi/4])




# Convert the list of instances to a NumPy array
leg_array = initialize_legs(num_leg, origin, links, ang_start, offsets )





points = 500
V_dir = np.array([30, 0, 50])
foot_pos = np.array([0, 0, 0])

[raise_leg_path, lean_leg_path] = leg_path(foot_pos, V_dir, points)




# Create a new figure
fig = plt.figure()

# Add a 3D subplot to the figure
ax = fig.add_subplot(111, projection='3d')

# Loop through each leg in leg_array
for leg_num in range(len(leg_array)):
    # Extract points for the current leg
    points = [leg_array[leg_num].J1_pos, leg_array[leg_num].J2_pos, leg_array[leg_num].J3_pos, leg_array[leg_num].EF_pos]

    # Separate the points into x, y, z coordinates
    x, y, z = zip(*points)

    # Plot lines between points for the current leg
    for i in range(len(points) - 1):
        ax.plot([x[i], x[i+1]], [y[i], y[i+1]], [z[i], z[i+1]], 'b')

    # Plot the points for the current leg
    ax.scatter(x, y, z, c='r') # 'r' for red color

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Line Plot with Points for All 6 Legs')

limit = 200

# Set the limits of the x, y, and z axes
ax.set_xlim3d(left=-limit, right=limit)
ax.set_ylim3d(bottom=-limit, top=limit)
ax.set_zlim3d(bottom=-limit, top=limit)

# Show the plot
plt.show()

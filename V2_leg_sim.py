"""
To-Do:
- Add documentation on code.
 - Document Inverse Kinematics
 - Document Forward Kinematics
"""


# Packages
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def leg_plot(ax, points):
    # Plot the points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')

    # Plot lines between the points
    for i in range(len(points)-1):
        ax.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], [points[i][2], points[i+1][2]], c='b')

    return ax


def leg_FK(origin_bot, angles, links):
    """
    # Hexapod Leg Forward-Kinematics function
    # Author: Eirik Strandman
    # Version: V1.0

    # Description:
    A function that takes in robot origin, joint angles and leg configuration and 
    calculates the position of the joints for a two-member hexapod leg.


    # Inputs
    - origin_bot: (x,y,z)[mm] - The origin of the robot coordinate system.
    - angles: [J0, J1, J2, J3][mm] - The joint angles.
    - links: [L0, L1, L2, L3][mm] - The link lengths of the robot leg.  

    # Output
    - [[J1_pos, J2_pos, J3_pos, EF_pos](x,y,z)[mm] - Numpy array containing estimated joint positions.
    """


    # Find x-values
    x2 = links[0] + links[1]
    x3 = x2 + links[2]*np.cos(angles[2])
    x4 = x3 + links[3]*np.cos(angles[2] + angles[3])

    # Find z-values
    z2 = origin_bot[2] + links[2]*np.sin(angles[2])
    z3 = z2 + links[3]*np.sin(angles[2] + angles[3])

    # Find joint positions
    J1_pos = np.array([ links[0]*np.sin(angles[0]),  links[0]*np.cos(angles[0]),  origin_bot[2] ])
    J2_pos = np.array([ x2*np.sin(angles[0] + angles[1]), x2*np.cos(angles[0] + angles[1]), origin_bot[2] ])
    J3_pos = np.array([ x3*np.sin(angles[0] + angles[1]), x3*np.cos(angles[0] + angles[1]), z2 ])
    EF_pos = np.array([ x4*np.sin(angles[0] + angles[1]), x4*np.cos(angles[0] + angles[1]), z3 ])

    # Return leg position as 2D array.
    leg_pos = np.array([J1_pos, J2_pos, J3_pos, EF_pos])
    return leg_pos


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


# Generate joint positions
joint_pos = np.array([])
for i in range(len(leg_angles)):
    joint_pos = np.append(joint_pos, leg_FK( origin, leg_angles[i], links ))
print(joint_pos[3])





















"""

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot points using the function
leg_plot(ax, joint_pos)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

limit = 150

# Set the limits of the x, y, and z axes
ax.set_xlim3d(left=-limit, right=limit)
ax.set_ylim3d(bottom=-limit, top=limit)
ax.set_zlim3d(bottom=-limit, top=limit)

# Show the plot
plt.show()
"""
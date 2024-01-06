# Packages
import serial
import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotate_vector_about_point(vector, angle, axis, point):
 # Translate the vector so that the point of rotation is at the origin
 vector = vector - point

 # Create a rotation object
 rotation = R.from_rotvec(angle * axis)

 # Rotate the vector
 rotated_vector = rotation.apply(vector)

 # Translate the vector back to its original position
 rotated_vector = rotated_vector + point

 return rotated_vector

def plot_3d_points(points):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # Plot each point with a label
  for i, point in enumerate(points):
      ax.scatter(*point, label=f'Point {i+1}')

  # Plot lines between points
  for i in range(len(points) - 1):
      ax.plot(*zip(points[i], points[i + 1]), label=f'Line {i+1}')

  # Add labels and legend
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.legend()

  plt.show()


# Inverse Kinematics
def leg_IK(origin, point, links, offset):
    # Translate to local leg coordinate system
    point_leg = point - origin

    # Find joint angle J1
    J1 = offset[0] + np.arctan2(point_leg[0], point_leg[1])

    # Calculate H position vector, subtract the length of L1
    H = np.sqrt((point_leg[0]**2) + (point_leg[1]**2)) - links[1]

    # Find resultant vector L from H and Z. 
    L = np.sqrt((H**2) + (point_leg[2]**2))

    # Find joint angle J3
    J3 = - (offset[2] - math.acos( ( links[2]**2 + links[3]**2 - (L**2)) / (2 * links[2] * links[3])))
    
    # Find joint angle J2
    beta = np.arccos( ( L**2 + (links[2]**2) - (links[3]**2) ) / ( 2 * links[2] * L ))
    alpha = np.arctan2( point[2], H )
    J2 = offset[1] + (beta  + (alpha) )

    return np.array([J1, J2, J3])

def leg_FK(origin, num, links, angles, offset, plot=False):
    # Extract Data
    J1_ang = angles[0] - offset[0]
    J2_ang = angles[1] - offset[0]
    J3_ang = angles[2] - offset[0]

    L0 = links[0]
    L1 = links[1]
    L2 = links[2]
    L3 = links[3]

    axis = np.array([0, 0, 1])

    # Find J1 position vector angle.
    leg_angle = (np.pi/3)*(num - 1)

    # Find J1 position
    J1_pos = np.array([ L0 * np.cos(leg_angle), L0 * np.sin(leg_angle), 0 ]) + origin
    #J1_pos = rotate_vector(J1_pos, J1_ang, np.array([0, 0, 1]))
    

    # Find J2 position
    J2_pos = np.array([L1, 0, 0]) + J1_pos
    J2_pos = rotate_vector_about_point(J2_pos, J1_ang, axis, J1_pos)

    # Find J3 position
    J3_pos = np.array([ L2 * np.cos(J2_ang), 0, L2 * np.sin(J2_ang) ]) + J2_pos
    J3_pos = rotate_vector_about_point(J3_pos, J1_ang, axis, J2_pos)

    # Find EF_pos
    EF_pos = np.array([ L3 * np.cos(J2_ang + J3_ang), 0, L3 * np.sin(J2_ang + J3_ang) ]) + J3_pos
    EF_pos = rotate_vector_about_point(EF_pos, J1_ang, axis, J3_pos)

    # Store points in array.
    points = np.array([J1_pos, J2_pos, J3_pos, EF_pos])
    #print("EF is")
    #print(points[3])

    if plot == True:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot each point with a label
        for i, point in enumerate(points):
            ax.scatter(*point, label=f'Point {i+1}')

        # Plot lines between points
        for i in range(len(points) - 1):
            ax.plot(*zip(points[i], points[i + 1]), label=f'Line {i+1}')

        # Add labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.legend()

        plt.show()
       

    return points


leg_par_num = 3 # Number of pairs of legs.
leg_spacing = 2*np.pi / (leg_par_num*2)
origin = np.array([0, 0, 0])
num = 2
links = np.array([75, 54.0, 71.0, 73.5])
angles = np.array([0, 0, -(np.pi/2)])
offset = np.array([(np.pi/2), (np.pi/2), (np.pi/2)])


angles = leg_IK(origin, np.array([0, 150, -40]), links, offset)
print("Angles")
print(np.rad2deg(angles))
print("-----------------------")
points = leg_FK(origin, num, links, angles, offset, True)


# Plot the points
print(points)















"""
# Prompt user for movement
origin = np.array([0,0,0])
point = np.array([0, 115.78, -38.5])
links = np.array([54, 71, 73.5])

joint_angles = leg_IK(origin, point, links)
print(joint_angles)


# Convert radians to degrees using list comprehension
degree_list = [math.degrees(radian) for radian in joint_angles]

print(degree_list)
"""

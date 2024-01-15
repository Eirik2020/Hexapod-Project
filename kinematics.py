"""
Python Program for serial communication with the Mini-Maestro 18 RC Servo controller.
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def rotate_about_z(vector, theta):
   """
   Rotate a 3D vector about the z-axis by a certain angle.

   Parameters:
   vector (numpy array): The 3D vector to rotate.
   theta (float): The angle to rotate the vector by, in radians.

   Returns:
   numpy array: The rotated vector.
   """
   rotation_matrix = np.array([
       [np.cos(theta), -np.sin(theta), 0],
       [np.sin(theta), np.cos(theta), 0],
       [0, 0, 1]
   ])
   return np.dot(rotation_matrix, vector)

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

def subdivide_vector(start_point, vector, divisions):
    # Create an array to hold the result
    result = []

    # Calculate the end point
    end_point = np.array(start_point) + np.array(vector)

    # Generate the points
    for i in range(len(start_point)):
        points = np.linspace(start_point[i], end_point[i], divisions)
        result.append(points)

    return np.transpose(result)

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
    J3 =  (offset[2] - math.acos( ( links[2]**2 + links[3]**2 - (L**2)) / (2 * links[2] * links[3])))
    
    # Find joint angle J2
    beta = np.arccos( ( L**2 + (links[2]**2) - (links[3]**2) ) / ( 2 * links[2] * L ))
    #print(np.rad2deg(beta))
    #print(point[2])
    #print(H)
    alpha = np.arctan2( point_leg[2], H )
    #print(np.rad2deg(alpha))
    #print(beta+alpha)
    #J2 = offset[1] - (beta  + (-alpha) )
    J2 = offset[1] - (beta  + (alpha) )

    return np.array([J1, J2, J3])

def leg_FK(origin, angles, links, offset):
   # Extract angles and adjust coordinate system.
   J1_ang = offset[0] - angles[0] 
   J2_ang = offset[1] - angles[1]  
   J3_ang = offset[2] + J2_ang - angles[2] - np.pi 

   # Extract link lengths
   L1 = links[1]
   L2 = links[2]
   L3 = links[3]

   # Find joint positions
   # J1
   J1 = origin

   # J2
   J2 = np.array([ 0, L1, 0 ]) + J1
   J2 = rotate_vector_about_point(J2, J1_ang, np.array([0, 0, 1]), J1)
  
   # J3
   J3 = np.array([ 0, L2*np.cos(J2_ang), L2*np.sin(J2_ang) ]) + J2
   J3 = rotate_vector_about_point(J3, J1_ang, np.array([0, 0, 1]), J2)

   # End-effector J3
   EF = np.array([ 0, L3*np.cos(J3_ang), L3*np.sin(J3_ang) ]) + J3
   EF = rotate_vector_about_point(EF, J1_ang, np.array([0, 0, 1]), J3)
   
   return np.array([J1, J2, J3, EF])


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
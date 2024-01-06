# Packages
import numpy as np


# Functions
def rotator(xyz, angle, axis):
    """
    Description:
    Takes in an array (xyz)[x,y,z] containing cartesian coordinates.
    Rotates it by the input angle (angle)[rad] about the selected axis (axis).
    Returns the rotated array.
    """

    # Select axis to rotate about
    match axis:
        case "x": #Rotate about x-axis
            R = np.array([
                [ 1,             0,              0 ],
                [ 0, np.cos(angle), -np.sin(angle) ],
                [ 0, np.sin(angle),  np.cos(angle) ]
                ])
        case "y": #Rotate about y-axis
            R = np.array([
                [  np.cos(angle), 0, np.sin(angle) ],
                [              0, 1,             0 ],
                [ -np.sin(angle), 0, np.cos(angle) ]
                ])
        case "z": #Rotate about z-axis
            R = np.array([
                [ np.cos(angle), -np.sin(angle), 0 ],
                [ np.sin(angle),  np.cos(angle), 0 ],
                [ 0,          0,                 1 ]
                ])
        case _: # For invalid input.
            raise ValueError("Error, invalid input. Axis input must be x, y or z")
    return  np.dot(xyz, R)


def translate(xyz, tx, ty, tz):
    """
    Description:
    Takes in an array (xyz)[x,y,z] containing cartesian coordinates.
    Translates it by the input (tx, ty, tz)[mm].
    Returns the translated array.
    """
    T = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

    # Ensure that xyz is a 1x4 array (homogeneous coordinates)
    xyz = np.append(xyz, 1)

    # Apply the translation matrix to the original point
    translated_xyz = np.dot(T, xyz)

    # Extract and return the translated 3D point (removing the homogeneous coordinate)
    return translated_xyz[:3]





# Constants
pi = np.pi
joint_origin = np.array([ 0, 0, 0]) #(x,y,z)[mm] Joint origin.

# DH Parameters
joint_angles = np.array([ 0, 0, -pi/2 ]) #[rad] Joint angles.
joint_offset = np.array([ 0, 0, 0  ]) #[mm] Joint offset.
link_lengths = np.array([ 100, 100, 100  ]) #[mm] Link lengths.
joint_twist  = np.array([ pi/2, 0, 0  ]) #[rad] Joint twist.



pos = joint_origin  # Start at the joint origin

# Apply transformations in a loop
for i in range(len(joint_angles)):
    # Rotate about z
    pos = rotator(pos, joint_angles[i], "z")
    # Add joint offset
    pos = translate(pos, 0, 0, joint_offset[i])
    # Add link length
    pos = translate(pos, link_lengths[i], 0, 0)
    # Rotate about x
    pos = rotator(pos, joint_twist[i], "x")

print("Final Position:")
print(pos)
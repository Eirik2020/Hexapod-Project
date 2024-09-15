import serial
import numpy as np
import com as com
import time

# Issue
# Paths are combined into a 6 item long array, instead into two 3 item arrays. 




### MAIN PROGRAM ###
# Open the serial port (replace 'COM3' with your port, e.g., '/dev/ttyUSB0' on Linux)
ser = serial.Serial('COM3', 9600, timeout=1)

# Initialize legs
legs = com.initialize_legs(6)



def rotate_vector(vector, angle, axis='z'):
    angle_rad = angle
    if axis == 'x':
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_rad), -np.sin(angle_rad)],
            [0, np.sin(angle_rad), np.cos(angle_rad)]
        ])
    elif axis == 'y':
        rotation_matrix = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad)],
            [0, 1, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad)]
        ])
    elif axis == 'z':
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0],
            [np.sin(angle_rad), np.cos(angle_rad), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")

    rotated_vector = np.dot(rotation_matrix, vector)
    return rotated_vector

def inter_3_point_arch(P_start, height, P_end, n):
    # Extract start and end coordinates
    x_start, y_start, z_start = P_start
    x_end, y_end, z_end = P_end
    
    # Calculate midpoint
    x_mid = (x_start + x_end) / 2
    y_mid = (y_start + y_end) / 2
    z_mid = (z_start + z_end) / 2 + height
    
    # Create arrays for x, y, z points
    t = np.linspace(0, 1, n)
    x_points = (1-t)**2 * x_start + 2*(1-t)*t * x_mid + t**2 * x_end
    y_points = (1-t)**2 * y_start + 2*(1-t)*t * y_mid + t**2 * y_end
    z_points = (1-t)**2 * z_start + 2*(1-t)*t * z_mid + t**2 * z_end
    
    # Return the arch points
    return np.vstack((x_points, y_points, z_points)).T

def inter_line(p1, p2, num_points):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    points = []

    for i in range(num_points):
        t = i / (num_points + 1)
        x = (1 - t) * x1 + t * x2
        y = (1 - t) * y1 + t * y2
        z = (1 - t) * z1 + t * z2
        points.append((x, y, z))
    
    return points

def path_leg(P_start, V_D, height, kick, n):
    # Kick
    if kick == True: # Kick is true, perform kick
        P_end = P_start - V_D
        path_line = inter_line(P_start, P_end, n)
        path_arch = inter_3_point_arch(P_end, height, P_start, n)
        path_combined = np.vstack((path_line, path_arch))
        
    # Step
    else: # Kick is false, perform step
        P_end = P_start + V_D
        path_arch = inter_3_point_arch(P_start, height, P_end, n)
        path_line = inter_line(P_end, P_start, n)
        path_combined = np.vstack((path_arch, path_line))
    
    # Return path
    return path_combined

def inv_kin(link, P_EF):
        # Translate from J_P1 to J_P2
        P_J2 = np.array([link[1], 0, 0]) #(x,y,z)[mm] - Eq:2.2.1

        # Calculate the vector V_L:
        V_L = P_EF - P_J2 #(x,y,z)[mm] - Eq:2.2.2

        # Calculate the squared magnitude of V_L:
        L_sq = V_L[0]**2 + V_L[2]**2 #(x,y,z)[mm] - Eq:2.2.3

        # Solve for angle J3:
        ang_J3 = - ( np.arccos( (link[2]**2 + link[3]**2  -  L_sq) / (2 * link[2] * link[3])) ) #[rad] - Eq:2.2.4

        # Calculate angle B
        ang_B = np.arccos( (L_sq + link[2]**2 - link[3]**2) / (2 * np.sqrt(L_sq) * link[2])) #[rad] - Eq:2.3.1

        # Calculate angle A
        ang_A = np.arctan2(V_L[2], V_L[0]) #[rad] - Eq:2.3.2

        # Calculate angle of joint J2
        ang_J2 = ang_B + ang_A

        # Calculate movement vector for P_EF in relation to P_J1
        V_EF_J1 = P_EF #(x,y,z)[mm] - Eq: 2.4.1

        # Calculate angle of joint J1
        ang_J1 = np.arctan2(V_EF_J1[1], V_EF_J1[0]) #[rad] - Eq: 2.4.2

        # Return angles
        return np.array([ang_J1, ang_J2, ang_J3])
    
def generate_walk(P_def, V_D, height, n, legs):
    for i in range(len(legs)):
        # Rotate vector V_D to the current leg's direction
        V_D_local = rotate_vector(V_D, ( ( (2 * np.pi) / len(legs) ) * i ), axis='z')

        # Determine if the leg is even or odd
        if i % 2 == 0:
            legs[i].path_step = path_leg(P_def, V_D_local, height, False, n)
            legs[i].path_kick = path_leg(P_def, V_D_local, height, True, n)
        else:
            legs[i].path_kick = path_leg(P_def, V_D_local, height, True, n)
            legs[i].path_step = path_leg(P_def, V_D_local, height, False, n)

""" 
def step(legs, fw, delay, ser):
    # Iterate through all points in leg path
    for i in range(len(legs[0].path_step)):
        # Iterate through each leg
        for leg in legs:
            if fw == True:
                path = leg.path_kick
            else:
                path = leg.path_step
            # Calculate inverse kinematics
            leg.angle = inv_kin(leg.link, path[i])  
            
            # Send Command
            com.send_command(ser, leg.ID, legs)
            time.sleep(delay)
"""
def step(legs, kick, delay, ser, leg_ID):
    # Iterate through all points in leg path
    for i in range(len(legs[leg_ID].path_step)):
        if kick == True:  # Kick
            path = legs[leg_ID].path_kick
        else:             # Step
            path = legs[leg_ID].path_step
        # Calculate inverse kinematics
        legs[leg_ID].angle = inv_kin(legs[leg_ID].link, path[i])  
        
        # Send Command
        com.send_command(ser, leg_ID, legs)
        time.sleep(delay)


# Configuration
P_def = np.array([ 125, 0, -73.5 ])
V_D = np.array([ 0, 40, 0 ])
h = 30
n = 100
delay = 0.005

target = 0
leg_angle = -((2 * np.pi) / 6)*target

V_D_R = rotate_vector(V_D, leg_angle, axis='z')
legs[target].path_step = path_leg(P_def, V_D_R, h, False, n)
legs[target].path_kick = path_leg(P_def, V_D_R, h, True, n)

for i in range(4):
        if i % 2 == 0:
            step(legs, False, delay, ser, target)
        else:
            step(legs, True, delay, ser, target)



# Close the serial port when done
ser.close()
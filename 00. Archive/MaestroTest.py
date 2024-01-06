import serial
import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




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
    J3 =  -(offset[2] - math.acos( ( links[2]**2 + links[3]**2 - (L**2)) / (2 * links[2] * links[3])))
    
    # Find joint angle J2
    beta = np.arccos( ( L**2 + (links[2]**2) - (links[3]**2) ) / ( 2 * links[2] * L ))
    alpha = np.arctan2( point[2], H )
    J2 = offset[1] + (beta  + (alpha) )

    return np.array([J1, J2, J3])


def send_set_multiple_targets_command(ser, device_number, targets):
   # Pololu protocol: 0xAA, device number, 0x1F, number of targets, channel number, target low bits, target high bits, ...
   command = bytearray([0xAA, device_number, 0x1F, len(targets)])
   print(len(targets))
   
   for channel, target in targets:
       # Append channel and target bytes to the command
       command.extend([channel, target & 0x7F, (target >> 7) & 0x7F])
   
   # Add the CRC checksum to the command
   crc = 0
   for byte in command[1:-1]: # Skip the first byte (0xAA) and the last byte (checksum)
       crc ^= byte
   command.append(crc)
   
   # Send command to Maestro
   ser.write(command)

# Function to send "Get Position" command
def send_get_position_command(ser, device_number, channel):
    # Pololu protocol: 0xAA, device number, 0x10, channel number
    command = bytes([0xAA, device_number, 0x10, channel])
    
    # Send command to Maestro
    ser.write(command)
    
    # Wait for a short time to ensure the response is available
    time.sleep(0.1)
    
    # Read the response
    response = ser.read(2)
    
    # Convert response bytes to an integer
    position = response[0] + (response[1] << 8)
    
    return position

# Replace 'COM3' with the actual serial port connected to the Maestro
ser = serial.Serial('COM6', 9600, timeout=1)


# Parameters
origin = np.array([0, 0, 0])
num = 1
links = np.array([0, 54.0, 71.0, 73.5])
offset = np.array([(np.pi/2), (np.pi/2), (np.pi/2)])


try:
    # Replace '0' with the desired device number and channel number
    device_number = 12
    channel_number = 0
    
    while True:
        # Prompt user for input coordinates
        while True:
            try:
                x, y, z = [float(s) for s in input('Enter x, y, z coordinates separated by spaces: ').split()]
            except ValueError:
                print('Please enter THREE values!')
            else:
                break
            
        # Calculate angles
        radians = leg_IK(origin, np.array([x, y, z]), links, offset)

        
        # Convert the input angle to the target value (servo position)
        target_value = ((radians / np.pi) * (10000-2000) + 2000).astype(int)  # convert degrees to servo value
        print(np.rad2deg(radians))
        #target_value[0] = 6000
        print(target_value)
        
        # Define the target positions for the servos
        targets = [(0, 6000), (1, 6000), (2, 6000)] # Replace with your desired positions

        # Send the command
        send_set_multiple_targets_command(ser, device_number, targets)
        
        
        # Verify the new position
        new_position1 = send_get_position_command(ser, device_number, 0)
        print(f"New Position on Channel {0}: {new_position1}")
        new_position2 = send_get_position_command(ser, device_number, 1)
        print(f"New Position on Channel {1}: {new_position2}")
        new_position3 = send_get_position_command(ser, device_number, 2)
        print(f"New Position on Channel {2}: {new_position3}")
        

except KeyboardInterrupt:
    print("\nExiting the program.")

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close the serial connection
    ser.close()

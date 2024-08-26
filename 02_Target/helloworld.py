import serial
import numpy as np


class leg_data:
    def __init__(self, leg_ID, joint_chan, joint_angle, invert_pos):
        self.leg_ID = leg_ID
        self.channel = joint_chan
        self.angle = joint_angle
        self.invert_pos = invert_pos


#### Communication #############################################################

def rad2bit8(angle):
    # Ensure the angle is within the range -π/4 to π/4
    if np.any(angle < -np.pi/4) or np.any(angle > np.pi/4):
        raise ValueError("All angles must be between -π/4 and π/4")
    
    # Map the angle from -π/4 to π/4 to the range 0 to 254
    scale_factor = 254 / (np.pi / 2)
    bit_value = (angle + np.pi / 4) * scale_factor
    
    # Clip values to the range 0 to 254 and convert to integers
    bit_value = np.clip(bit_value, 0, 254).astype(int)
    
    return bit_value

def invert_bits(bit_array, control_array):
    # Invert the bit values where the control array has a 0
    inverted_array = np.where(control_array == 1, 255 - bit_array, bit_array)
    
    return inverted_array

def cal_joint_chan(leg_ID):
    joint_channel = np.array([ (leg_ID*3), (leg_ID*3 + 1), (leg_ID*3 + 2)  ])

    return joint_channel

def set_target_mini_ssc(ser, channel, target):
  # Mini-SSC protocol: 0xFF, channel address, 8-bit target
  command = bytearray([0xFF, channel, target])

  # Send command to the serial device
  ser.write(command)

def initialize_legs(num_legs):
    legs = []
    for i in range(num_legs):
        leg = leg_data( 
            leg_ID = i,                          # Set leg_ID to the index in the list
            joint_chan = cal_joint_chan(i), # Set joint channels
            joint_angle = np.array([0, 0, 0]),     # Default leg_angle
            invert_pos = np.array([1, 1, 1])     # Default invert_pos
                       )
        legs.append(leg)

    return legs

def send_command(ser, leg_ID, leg):
    # Convert to bit
    bit = rad2bit8(leg[leg_ID].angle)
    
    # Invert select bits
    bit = invert_bits(bit, leg[leg_ID].invert_pos)
    
    # Send to target
    set_target_mini_ssc(ser, leg[leg_ID].channel[0], bit[0])
    set_target_mini_ssc(ser, leg[leg_ID].channel[1], bit[1])
    set_target_mini_ssc(ser, leg[leg_ID].channel[2], bit[2])

def send_multi_command(ser, leg, targets):
    for target in targets:
        send_command(ser, target, leg)
        
################################################################################

#### Kinematics ################################################################
def inv_kin(self, P_EF, update_pos=False):
        """ Description:
        """
        # Load robot configuration
        P_J1 = self.origin_leg
        link = self.link


        # Translate from J_P1 to J_P2
        P_J2 = P_J1 + np.array([link[1], 0, 0]) #(x,y,z)[mm] - Eq:2.2.1

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
        V_EF_J1 = P_EF - P_J1 #(x,y,z)[mm] - Eq: 2.4.1

        # Calculate angle of joint J1
        ang_J1 = np.arctan2(V_EF_J1[1], V_EF_J1[0]) #[rad] - Eq: 2.4.2


        # Update joint angles and pos
        self.angle = np.array([ang_J1, ang_J2, ang_J3])

        if update_pos == True:
            self.pos = self.fw_kin(self.angle)











### MAIN PROGRAM ###
# Open the serial port (replace 'COM3' with your port, e.g., '/dev/ttyUSB0' on Linux)
ser = serial.Serial('COM3', 9600, timeout=1)

# Initialize legs
leg = initialize_legs(6)


# Input Values
#leg_ID = 1
targets = [0]
# Leg 1
angle_deg1 = np.array([0, 25, 45])
leg[targets[0]].angle = np.radians(angle_deg1)


# Send Command
send_multi_command(ser, leg, targets)


# Close the serial port when done
ser.close()

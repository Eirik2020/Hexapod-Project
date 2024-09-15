import serial
import numpy as np


class leg_data:
    def __init__(self, leg_ID, joint_chan, joint_angle, invert_pos, offset, link, path=None):
        self.ID = leg_ID
        self.channel = joint_chan
        self.angle = joint_angle
        self.invert_pos = invert_pos
        self.offset = offset
        self.link = link
        self.path_step = path
        self.path_kick = path


#### Communication #############################################################

def rad2bit8(angle, offset):
    # Apply offset
    angle = angle + offset

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
            leg_ID = i,                                    # Set leg_ID to the index in the list
            joint_chan = cal_joint_chan(i),                # Set joint channels
            joint_angle = np.array([0, 0, 0]),             # Default leg_angle
            invert_pos = np.array([1, 1, 0]),              # Default invert_pos
            offset = np.array([0, 0, np.pi/2]),            # Joint Angle offsets
            link = np.array([  72.75,  54.0, 71.0, 73.5 ]) # Link lengths [mm])
                       )
        legs.append(leg)

    return legs

def send_command(ser, leg_ID, leg):
    # Convert to bit
    bit = rad2bit8(leg[leg_ID].angle, leg[leg_ID].offset )
    
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

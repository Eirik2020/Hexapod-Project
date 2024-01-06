"""
Python Program for serial communication with the Mini-Maestro 18 RC Servo controller.
"""
import time
import numpy as np


def set_multiple_targets(ser, device_number, targets):
  # Pololu protocol: 0xAA, device number, 0x1F, number of targets, first channel number, first target low bits, first target high bits, second target low bits, second target high bits, …
  command = bytearray([0xAA, device_number, 0x1F, len(targets)])
  
  for channel, target in targets:
      # Append channel and target bytes to the command
      command.extend([channel, target & 0x7F, (target >> 7) & 0x7F])
  
  # Send command to Maestro
  ser.write(command)

def set_target_mini_ssc(ser, channel, target):
  # Mini-SSC protocol: 0xFF, channel address, 8-bit target
  command = bytearray([0xFF, channel, target])

  # Send command to Maestro
  ser.write(command)

def send_path(ser, legs, delay):
   leg = legs[0]
   print(leg.servos)

   # Extract leg information
   for i in range(len(leg.angles)):
      # Send target information
      for j in range(len(legs)):
         angles = legs[j].angles
         target = legs[j].servos
         # Convert to target values
         target_value = ( (angles[i] / (np.pi/2) ) * 254 ).astype(int)
         set_target_mini_ssc(ser, target[0], target_value[0]) 
         set_target_mini_ssc(ser, target[1], target_value[1]) 
         set_target_mini_ssc(ser, target[2], target_value[2]) 
      
      # Delay
      time.sleep(delay)
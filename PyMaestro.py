"""
Python Program for serial communication with the Mini-Maestro 18 RC Servo controller.
"""



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
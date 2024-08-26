# Packages
import numpy as np
import os
import json



leg_ID = [0, 1, 2]
joint_ID = [0, 1, 2]
origin_leg = [0, 0, 0]
link = [ 100, 100, 100, 100 ]
ang_def = [ 0, 0, 0, -np.pi/2]


class leg:
    def __init__(self, leg_ID, joint_ID, origin_leg, link, angle, ser=None):
        """ Description:
        """
        self.leg_ID = leg_ID
        self.joint_ID = joint_ID
        self.link = link
        self.origin_leg = origin_leg
        self.angle = angle
        self.pos = self.fw_kin(self.angle)
        self.EF_path = self.pos
        self.ser = ser


    def fw_kin(self, angle):
        """ Function Description:
        """
        origin_leg = self.origin_leg
        link = self.link
        # Calculate joint origin P_J1
        P_J1 = origin_leg

        # Calculate resultant Rx vectors
        R1 = link[1] #[mm] Eq:8
        R2 = link[2] * np.cos(angle[2]) #[mm] Eq:9
        R3 = link[3] * np.cos(angle[3]) #[mm] Eq:10

        # Calculate joint positions
        P_J2 = P_J1 + np.array([ R1, P_J1[1], P_J1[2] ]) #(x,y,z)[mm] Eq:5 and Eq:12
        P_J3 = P_J2 + np.array([ R2*np.cos(angle[1]), R2*np.sin(angle[1]), P_J2[1] + link[2]*np.sin(angle[2]) ]) #(x,y,z)[mm] Eq:6 and Eq:13
        P_EF = P_J3 + np.array([ R3*np.cos(angle[1]), R3*np.sin(angle[1]), P_J3[1] + link[3]*np.sin(angle[3]) ]) #(x,y,z)[mm] Eq:6 and Eq:13

        # Update joint positions
        self.pos = np.array([ P_J1, P_J2, P_J3, P_EF ])
    
    def inv_kin(self, P_EF, update_pos):
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

    def command_servo(self):
        """ Description:
        """
        for joint_ID, angle in zip(self.joint_ID, self.angle):
            # Convert from radiance to bit
            angle_bit = ( ((angle + (np.pi / 2)) / np.pi) * 255 ).astype(int) #[bit]

            # Translate information into byte
            command = bytearray([0xFF, joint_ID, angle_bit]) #[byte]

            # Send command
            self.ser.write(command)

    #asd


meg = leg(leg_ID, joint_ID, origin_leg, link, ang_def)
print(meg.pos)

print("dfghj")











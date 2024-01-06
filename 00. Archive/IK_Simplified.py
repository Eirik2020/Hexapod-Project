# Packages
import numpy as np


def leg_IK(pos, links):
    R = np.sqrt(pos[0]**2 + pos[1]**2)
    angle1 = np.arccos( (R-links[0]-L2*np.cos()))

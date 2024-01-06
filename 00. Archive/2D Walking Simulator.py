import matplotlib.pyplot as plt
import numpy as np

# Shortcuts
pi = np.pi
cos = np.cos
sin = np.sin


# Functions
def polar_pos(radius, angle):
    pos = np.array([
        radius * cos(angle),
        radius * sin(angle),
    ])
    return pos


def triangle_center(P1, P2, P3):
    x = [P1[0], P2[0], P3[0]]
    y = [P1[1], P2[1], P3[1]]
    x_avg = sum(x) / len(x)
    y_avg = sum(y) / len(y)
    P_H = np.array([x_avg, y_avg])
    return P_H


def triangle_plot(P1, P2, P3):
    # Close shape
    triangle = [P1, P2, P3, P1]

    # Extract x and y coordinates from the points
    coords = [[], []]  # Initialize list
    coords[0] = [point[0] for point in triangle]  # Extract x-coordinates
    coords[1] = [point[1] for point in triangle]  # Extract y-coordinates
    return coords


def leg_phase_select(leg_cnt, phase):
    """
    Takes in two arguments, number of legs (leg_cnt) and phase (phase), if phase = 0, returns even legs, if phase = 1, returns odd numbers.
    """
    leg_list = np.linspace(1, leg_cnt, leg_cnt)
    if len(leg_list) % 2 != 0:
        # Check if the robot has a symmetric number of legs.
        print("ERROR! The number is legs is not symmetrical.")
    elif phase == False:
        # Return even legs.
        selected_legs = np.arange(1, leg_cnt, 2)  # 1 3 5 7
    elif phase == True:
        # Return odd legs.
        selected_legs = np.arange(2, leg_cnt + 1, 2)  # 2 4 6 8
    return selected_legs


# Classes
class leg:
    def __init__(self, number, robot_origin, leg_origin_radius, leg_spacing):
        # Parameters
        self.number = number  # Leg ID
        self._robot_origin = robot_origin  # Store robot_origin as a private attribute
        self.leg_origin_radius = leg_origin_radius
        self.leg_spacing = leg_spacing

    @property
    def robot_origin(self):
        return self._robot_origin

    @robot_origin.setter
    def robot_origin(self, new_origin):
        self._robot_origin = new_origin

    @property
    def origin(self):
        return polar_pos(self.leg_origin_radius, (self.leg_spacing * (self.number - 1))) + self.robot_origin

    @property
    def foot_pos_def(self):
        return self.origin + polar_pos(self.leg_origin_radius, (self.leg_spacing * (self.number - 1)))


###    Main Code    ####
# Hexapod Parameters
robot_origin = np.array([15,15]) #[mm](x,y) Robot's coordinate system origin.
leg_cnt = 6 # Number of legs.
leg_spacing = 2*pi / leg_cnt #[rad] Spacing between legs.
leg_origin_radius = 50 # Distance from robot origin to leg origin.

# Workspace Parameters
WK_R_min = 10 #[mm] Workspace inner radius.
WK_R_max = 30 #[mm] Workspace outer radius.
WK_R_avg = (WK_R_max + WK_R_min) / 2 #[mm] Workspace middle radius.
WK_margin = 10 #[%] Safety margin to workspace edge.
WK_border_right = -(leg_spacing / 2) * ((100-WK_margin)/100) #[rad] Right border angle.
WK_border_left = (leg_spacing / 2) * ((100-WK_margin)/100) #[rad] Right border angle.

# Leg phase select
phase = True


# Generate dictionary containing the legs.
leg_list = {i + 1: leg(i + 1, robot_origin, leg_origin_radius, leg_spacing) for i in range(leg_cnt)}
print(robot_origin)

# Lean legs
lean_legs = leg_phase_select(leg_cnt, phase)

# Step legs
step_legs = leg_phase_select(leg_cnt, not phase)

# Find robot's origin
robot_origin = triangle_center(leg_list[lean_legs[0]].origin, leg_list[lean_legs[1]].origin, leg_list[lean_legs[2]].origin)

# Update leg origin
for leg_obj in leg_list.values():
    leg_obj.robot_origin = robot_origin



##  Plotting  ##
# Extract x and y coordinates from the points
stance_joints = triangle_plot(leg_list[lean_legs[0]].origin, leg_list[lean_legs[1]].origin, leg_list[lean_legs[2]].origin)
walk_joints = triangle_plot(leg_list[step_legs[0]].origin, leg_list[step_legs[1]].origin, leg_list[step_legs[2]].origin)




# Extract the leg origins from the dictionary
foot_pos = [leg_obj.foot_pos_def for leg_obj in leg_list.values()]

# Extract x and y coordinates from the leg origins
x_coords = [point[0] for point in foot_pos]
y_coords = [point[1] for point in foot_pos]
# Plot the points
plt.scatter(x_coords, y_coords, marker='o')

# Plot the points
plt.plot(stance_joints[0], stance_joints[1], marker='o', linestyle='dashed', markersize=8, label='Stance Joints')
plt.plot(walk_joints[0], walk_joints[1], marker='o', linestyle='dashed', markersize=8, label='Walk Joints')
plt.plot(robot_origin[0], robot_origin[1], "o", markersize=8, label='Hexapod Center')

lim = 150
# Set labels and legend
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.xlim(-lim, lim)  # Set your desired x-axis limits (e.g., x_min and x_max)
plt.ylim(-lim, lim)  

# Show the plot
plt.grid()
plt.show()

 
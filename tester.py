import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

# Define force vectors
forces = [(1, 1), (-2, 2)]  # Example forces (x, y)

# Plot arrows
plt.figure()
ax = plt.gca()
for force in forces:
    ax.arrow(0, 0, force[0], force[1], head_width=0.1, head_length=0.1, fc='blue', ec='blue')

# Calculate and display angle between vectors
angle = np.arccos(np.dot(forces[0], forces[1]) / (np.linalg.norm(forces[0]) * np.linalg.norm(forces[1])))
angle_degrees = np.degrees(angle)
plt.text(0.5, 1, f'Angle: {angle_degrees:.2f} degrees', fontsize=12)

# Draw curved arrow indicating angle span
arc_radius = 0.5
angle_span = 90  # Angle span for the arc
arc = Arc([0, 0], arc_radius*2, arc_radius*2, angle=0, theta1=0, theta2=angle_degrees, color='red')
ax.add_patch(arc)

# Set plot limits
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])

# Show plot
plt.grid()
plt.show()

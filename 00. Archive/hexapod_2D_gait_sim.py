import numpy as np
import matplotlib.pyplot as plt

coxa_pos_angle = np.pi / 3
coxa_pos_radius = 145
leg_count = 6

coxa_pos = np.empty((leg_count, 2))

for i in range(leg_count):
    coxa_pos[i] = [coxa_pos_radius * np.cos(coxa_pos_angle * i), coxa_pos_radius * np.sin(coxa_pos_angle * i)]

# Complete the hexagon by connecting the coxa positions with lines
hexagon_points = np.concatenate((coxa_pos, np.roll(coxa_pos, 1, axis=0)), axis=0)

# Plot the hexagon
plt.plot(hexagon_points[:, 0], hexagon_points[:, 1], marker='o')
plt.title('Hexagon with Coxa Positions')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation







# Create a figure and a 3D axes
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Function to generate data points for the spiral path
def gen(n):
    phi = 0
    while phi < 2*np.pi:
        yield np.array([np.cos(phi), np.sin(phi), phi])
        phi += 2*np.pi/n

# Function to update the plot with new data points
def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])

# Number of points in the spiral path
N = 100
# Generate the spiral path data
data = np.array(list(gen(N))).T
# Plot the initial point
line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

# Set the axes properties
ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')
ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')
ax.set_zlim3d([0.0, 10.0])
ax.set_zlabel('Z')

# Create the animation
ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)

# Display the plot
plt.show()

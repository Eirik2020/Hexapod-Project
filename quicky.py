import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the limits for the axes
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([0, 10])

# Define the static curve along the z-axis
z_line = np.linspace(0, 10, 100)
x_line = np.zeros_like(z_line)
y_line = np.zeros_like(z_line)
ax.plot(x_line, y_line, z_line, 'b-') # Plot the line in blue

# Function to update the moving point
def update(frame):
    ax.clear() # Clear the axes
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([0, 10])
    ax.plot(x_line, y_line, z_line, 'b-') # Redraw the static curve
    
    # Define the moving point's position based on the frame
    x = np.cos(frame / 100)
    y = np.sin(frame / 100)
    z = frame / 100
    
    # Plot the moving point
    ax.scatter(x, y, z, color='r')

# Create the animation with a faster interval
ani = animation.FuncAnimation(fig, update, frames=100, interval=10, blit=False)

# Display the plot
plt.show()

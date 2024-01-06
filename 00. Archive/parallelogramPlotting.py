import numpy as np
import matplotlib.pyplot as plt

# Define the integrand function
def integrand(x, y):
    return x - 2

# Define the limits of integration
x_lower = 2
x_upper = 4

# Function representing the lower and upper bounds for y
def y_lower(x):
    return x - 2

def y_upper(x):
    return x - 1

# Create a meshgrid for x and y values
x_values = np.linspace(x_lower, x_upper, 1000)
y_values_lower = y_lower(x_values)
y_values_upper = y_upper(x_values)

# Plot the region between the curves
plt.plot(x_values, y_values_lower, label='$y = x - 2$', color='blue')
plt.plot(x_values, y_values_upper, label='$y = x - 1$', color='orange')

# Draw lines from (2,0) to (2,1) and from (4,2) to (4,3)
plt.plot([2, 2], [0, 1], color='green')
plt.plot([4, 4], [2, 3], color='red')

# Draw the box with dotted lines in green
plt.plot([2, 4, 4, 2, 2], [0, 0, 1, 1, 0], linestyle='--', color='green', label='Double Integral Deformed in uv-coordinate system')

# Add labels and legend
plt.xlabel('$x$')
plt.ylabel('$y$')
plt.legend()

# Set plot limits
plt.xlim(0, 4.5)
plt.ylim(-0.5, 4.5)

# Add grid
plt.grid(True, linestyle='--', alpha=0.7)

# Display the plot
plt.show()

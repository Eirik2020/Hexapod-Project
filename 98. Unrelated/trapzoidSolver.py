import numpy as np

# Define the integrand function
def integrand(x):
    return x - 2

# Define the limits of integration
x_values = np.linspace(2, 4, 1000)  # Adjust the number of points as needed

# Calculate the y values for the lower and upper bounds
y_lower_values = x_values - 2
y_upper_values = x_values - 1

# Calculate the area for different step sizes
step_sizes = [1.5, 0.75, 0.25]

for step_size in step_sizes:
    x_points = np.arange(2, 4 + step_size, step_size)
    y_lower_points = x_points - 2
    y_upper_points = x_points - 1

    area_lower = np.trapz(y_lower_points, x_points)
    area_upper = np.trapz(y_upper_points, x_points)

    total_area = area_upper - area_lower
    print(f"Step Size: {step_size}, Approximated Area: {total_area:.6f}")

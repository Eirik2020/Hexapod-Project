import numpy as np
import matplotlib.pyplot as plt

# Motor parameters
J = 0.01  # moment of inertia (kg*m^2)
B = 0.1   # damping coefficient (N*m*s)

# Applied torque function (you can define your own function)
def torque(t):
    return 5.0  # constant torque for simplicity

# Provided analytical solution for the DC motor
def analytical_solution(t):
    return (50 * np.exp(10 * t) - 40) / np.exp(10 * t)

# Differential equation for the DC motor
def motor_equation(t, omega):
    return (1 / J) * (torque(t) - B * omega)

# Euler method for numerical integration
def euler(h, t_span, initial_condition):
    num_steps = int((t_span[1] - t_span[0]) / h)
    t_values = np.linspace(t_span[0], t_span[1], num_steps + 1)
    omega_values = np.zeros(num_steps + 1)
    omega_values[0] = initial_condition

    for i in range(num_steps):
        omega_values[i + 1] = omega_values[i] + h * motor_equation(t_values[i], omega_values[i])

    return t_values, omega_values

# Initial condition with starting speed set to 10 rad/s
initial_angular_velocity = 10.0

# Time span
time_span = (0, 2)

# Three different step sizes for numerical integration
step_sizes = [0.1, 0.05, 0.01]

# Plot the results for each step size
plt.figure(figsize=(10, 6))

for step_size in step_sizes:
    time_values_euler, angular_velocity_values_euler = euler(step_size, time_span, initial_angular_velocity)
    plt.plot(time_values_euler, angular_velocity_values_euler, label=f'Euler Method (h={step_size})')

# Plot the provided analytical solution
time_values_analytical = np.linspace(time_span[0], time_span[1], 100)
plt.plot(time_values_analytical, analytical_solution(time_values_analytical),
         label='Analytical Solution', linestyle='--')

# Plot settings
plt.title('DC Motor Response to Constant Torque (Euler Method)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.grid(True)
plt.xlim(0, 1)
plt.ylim(0, 75)
plt.show()

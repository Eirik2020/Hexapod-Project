import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Motor parameters
J = 0.01  # kg*m^2
B = 0.1   # N*m*s

# Applied torque function
def torque(t):
    return 5  # constant torque of 5 N*m

# System of differential equations
def motor_equation(t, y):
    omega = y[0]
    domega_dt = (torque(t) - B * omega) / J
    return [domega_dt]

# Initial conditions
initial_conditions = [10]  # Initial angular velocity

# Time span
t_span = (0, 10)  # Simulation time from 0 to 10 seconds

# Solve the differential equation
solution = solve_ivp(motor_equation, t_span, initial_conditions, method='RK45')

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(solution.t, solution.y[0], label='Angular Velocity')
plt.title('DC Motor Response to Constant Torque')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.grid(True)
plt.show()

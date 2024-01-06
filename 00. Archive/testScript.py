import numpy as np
import matplotlib.pyplot as plt

# Motor parameters to be estimated
K_V_guess = 0.1
K_T_guess = 0.01
R_a = 1.0  # Armature resistance

# Simulated data
time = np.linspace(0, 10, 1000)
omega_true = 10 * np.sin(2 * np.pi * 0.5 * time)  # Simulated true velocity
current = 2.0 + 0.5 * np.sin(2 * np.pi * 0.5 * time)  # Simulated current (with noise)
voltage = K_V_guess * omega_true + R_a * current  # Simulated voltage

# Iterative parameter estimation using Newton's method
for _ in range(10):
    # Calculate motor acceleration from velocity data
    alpha = np.gradient(omega_true, time)

    # Calculate torque from motor acceleration
    J = 1.0  # Simulated motor inertia (constant for simplicity)
    torque = J * alpha

    # Calculate back EMF (E) from velocity
    back_emf = K_V_guess * omega_true

    # Calculate residuals (differences between calculated and measured voltage)
    residuals = voltage - R_a * current - back_emf

    # Jacobian matrix for parameter update
    J_matrix = np.array([
        -current,
        alpha
    ]).T

    # Parameter update using Newton's method
    delta_params = np.linalg.solve(J_matrix.T @ J_matrix, J_matrix.T @ residuals)
    K_V_guess += delta_params[0]
    K_T_guess += delta_params[1]

    print(f"Iteration: {_+1}, K_V: {K_V_guess}, K_T: {K_T_guess}")

# Plot results
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(time, voltage, label='Measured Voltage')
plt.plot(time, R_a * current + back_emf, label='Calculated Voltage')
plt.xlabel('Time')
plt.ylabel('Voltage')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, omega_true, label='True Velocity')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.tight_layout()
plt.show()

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

def forward_kinematics_dh(theta, d_params):
    a1, a2, a3 = d_params[:, 0]
    alpha1, alpha2, alpha3 = np.radians(d_params[:, 1])
    d1, d2, d3 = d_params[:, 3]

    T01 = np.array([
        [np.cos(theta[0]), -np.sin(theta[0]) * np.cos(alpha1), np.sin(theta[0]) * np.sin(alpha1), a1 * np.cos(theta[0])],
        [np.sin(theta[0]), np.cos(theta[0]) * np.cos(alpha1), -np.cos(theta[0]) * np.sin(alpha1), a1 * np.sin(theta[0])],
        [0, np.sin(alpha1), np.cos(alpha1), d1],
        [0, 0, 0, 1]
    ])

    T12 = np.array([
        [np.cos(theta[1]), -np.sin(theta[1]) * np.cos(alpha2), np.sin(theta[1]) * np.sin(alpha2), a2 * np.cos(theta[1])],
        [np.sin(theta[1]), np.cos(theta[1]) * np.cos(alpha2), -np.cos(theta[1]) * np.sin(alpha2), a2 * np.sin(theta[1])],
        [0, np.sin(alpha2), np.cos(alpha2), d2],
        [0, 0, 0, 1]
    ])

    T23 = np.array([
        [np.cos(theta[2]), -np.sin(theta[2]) * np.cos(alpha3), np.sin(theta[2]) * np.sin(alpha3), a3 * np.cos(theta[2])],
        [np.sin(theta[2]), np.cos(theta[2]) * np.cos(alpha3), -np.cos(theta[2]) * np.sin(alpha3), a3 * np.sin(theta[2])],
        [0, np.sin(alpha3), np.cos(alpha3), d3],
        [0, 0, 0, 1]
    ])

    T03 = np.dot(T01, np.dot(T12, T23))
    
    return T03[:3, 3]

def inverse_kinematics(target_position, d_params):
    def objective_function(theta):
        end_effector_position = forward_kinematics_dh(theta, d_params)
        return np.linalg.norm(end_effector_position - target_position)

    # Initial guess for joint angles
    initial_guess = np.zeros(3)

    # Bounds for joint angles (adjust as needed)
    bounds = [(0, 2*np.pi)] * 3

    result = minimize(objective_function, initial_guess, bounds=bounds)
    if result.success:
        return result.x
    else:
        raise ValueError("Inverse kinematics optimization failed.")
    
def plot_robot_arm(joint_positions, dh_params):
    # Create a list to store points along the robot arm
    points = []

    # Initial transformation matrix
    T = np.eye(4)

    # Plot each link
    for i, theta in enumerate(joint_positions):
        # Update the transformation matrix
        T_i = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(dh_params[i, 1]), np.sin(theta) * np.sin(dh_params[i, 1]), dh_params[i, 0] * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(dh_params[i, 1]), -np.cos(theta) * np.sin(dh_params[i, 1]), dh_params[i, 0] * np.sin(theta)],
            [0, np.sin(dh_params[i, 1]), np.cos(dh_params[i, 1]), dh_params[i, 3]],
            [0, 0, 0, 1]
        ])

        T = np.dot(T, T_i)

        # Extract the end effector position
        end_effector_position = T[:3, 3]

        # Append the end effector position to the list
        points.append(end_effector_position)

    # Convert the list to a numpy array for easier indexing
    points = np.array(points)

    # Plot the robot arm
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Example usage:
links = np.array([ 100, 80, 60 ])     # List of link lengths.
twists = np.array([ 0, np.pi/2, 0 ])  # List of joint twists.
joint_angles = np.array([ 0, 0, 0 ])  # List of joint angles.
joint_offset = np.array([ 0, 7, 7 ]) # List of joint offsets.

# Generate the DH parameter matrix.
dh_parameters = np.array([
    [ links[0], twists[0], joint_angles[0], joint_offset[0] ],
    [ links[1], twists[1], joint_angles[1], joint_offset[1] ],
    [ links[2], twists[2], joint_angles[2], joint_offset[2] ]
])


# Target end effector position (replace with your desired coordinates)
target_position = np.array([150, 100, 50])

# Calculate inverse kinematics
joint_angles = inverse_kinematics(target_position, dh_parameters)
print("Inverse Kinematics - Joint Angles:", np.degrees(joint_angles))

# Verify with forward kinematics
calculated_position = forward_kinematics_dh(joint_angles, dh_parameters)
print("Forward Kinematics - Calculated Position:", calculated_position)
print("Target Position:", target_position)

# Plot the robot arm
plot_robot_arm(joint_angles, dh_parameters)
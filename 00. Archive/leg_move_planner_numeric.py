import numpy as np
import matplotlib.pyplot as plt


def vector_parallel_check(ref_vector, vector):
    """
    Checks if reference vector (ref_vector) and vector (vector) is parallel using dot product.
    """
    # Normalize the reference vector
    normalized_reference_vector = ref_vector / np.linalg.norm(ref_vector)
    # Calculate dot product
    dot_product = np.dot(normalized_reference_vector, vector)
    # If dot product is bigger than 0, vector is parallel
    if dot_product > 0:
       return True
    else:
       return False



# Constants
pi = np.pi # Make writing pi faster.
phi_R = 0 #[rad] Right border angle.
phi_L = pi/3 #[rad] Left border angle.
R_max = 10 #[mm] Outer work surface radius.
R_min = 3 #[mm] Inner work surface radius.

# Intervals
phi_circ = np.linspace(phi_R, phi_L, 100)
R_border = np.linspace(R_min, R_max, 100)

# Calculate work surface border points.
work_surface_border_x = np.array([]) # Initialize empty array for x-coordinates.
work_surface_border_y = np.array([]) # Initialize empty array for y-coordinates.

# Outer circle coordinates
x1 = R_max * np.cos(phi_circ)
y1 = R_max * np.sin(phi_circ)
work_surface_border_x = np.append(work_surface_border_x, x1)
work_surface_border_y = np.append(work_surface_border_y, y1)

# Inner circle coordinates
x1 = R_min * np.cos(phi_circ)
y1 = R_min * np.sin(phi_circ)
work_surface_border_x = np.append(work_surface_border_x, x1)
work_surface_border_y = np.append(work_surface_border_y, y1)

# Right border coordinates
x1 = R_border * np.cos(phi_R)
y1 = R_border * np.sin(phi_R)
work_surface_border_x = np.append(work_surface_border_x, x1)
work_surface_border_y = np.append(work_surface_border_y, y1)

# Left border coordinates
x1 = R_border * np.cos(phi_L)
y1 = R_border * np.sin(phi_L)
work_surface_border_x = np.append(work_surface_border_x, x1)
work_surface_border_y = np.append(work_surface_border_y, y1)


# Vector line
S_vel = 1
V = [(S_vel*np.cos(phi_L/2)), (S_vel * (np.sin(phi_L/2)))]
t = np.linspace(0, R_max*2, 100)
P_EF = [(R_max-R_min)* np.cos(phi_L/2), (R_max-R_min)* np.sin(phi_L/2)]

# Generate Movement path.
move_vector_x = P_EF[0]+V[0]*t
move_vector_y = P_EF[1]+V[1]*t


# Find intersection points.
intersection_x = np.array([]) # Initialize empty array for x-coordinates.
intersection_y = np.array([]) # Initialize empty array for y-coordinates.

# Find intersection points
acc = 0.25
for i in range(len(work_surface_border_x)):
  for j in range(len(move_vector_x)):
    diff_x = abs(work_surface_border_x[i] - move_vector_x[j])
    diff_y = abs(work_surface_border_y[i] - move_vector_y[j])
    if diff_x < acc and diff_y < acc:
      intersection_x = np.append(intersection_x, work_surface_border_x[i])
      intersection_y = np.append(intersection_y, work_surface_border_y[i])






# Initialize the list of vectors
intersection_vectors = []

# Calculate the vectors
for i in range(len(intersection_x)):
    vector = np.array([intersection_x[i] - P_EF[0], intersection_y[i] - P_EF[1]])
    intersection_vectors.append(vector)


# Check if vectors are parallel and in the same direction, if true, adds to list.
parallel_vectors = []
for vector in intersection_vectors:
    if vector_parallel_check(V, vector) == True:
        parallel_vectors.append(vector)



# Now, parallel_vectors contains the vectors that are parallel and in the same direction as the reference vector
print("Parallel Vectors:", parallel_vectors)



mag1 = np.linalg.norm(V)
print("Mag1:", mag1)
mag2 = np.linalg.norm(parallel_vectors[0])

scale_factor = mag1 * mag2
print("Scaling Factor: ", scale_factor)



P_EF_2 = P_EF + parallel_vectors[0]
leg_vector = parallel_vectors[0] * mag1

print(P_EF_2)

# Create the plot
plt.figure(figsize=(10, 10))
plt.plot(work_surface_border_x, work_surface_border_y, linestyle='-', color='black', label='Work surface border')
plt.scatter(P_EF[0], P_EF[1], marker='o', color='black', label='End Effector start position' )
plt.scatter(P_EF_2[0], P_EF_2[1], marker='o', color='red', label='End Effector stop position')
plt.quiver(P_EF[0], P_EF[1], leg_vector[0], leg_vector[1], angles='xy', scale_units='xy', scale=1, color='red', label='End Effector Path')

# Set axis labels
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# Add a grid
plt.grid()

# Plot title
plt.title('Leg movement path')

# Show legend
plt.legend()

# Display the plot
plt.show()

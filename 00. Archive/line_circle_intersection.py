import numpy as np

# Function to check if a vector is parallel to V
def is_parallel(vector, V):
    # Check if the cross product of the two vectors is zero
    return np.all(np.cross(vector, V) == 0)

# Function to filter and return the parallel vectors
def find_parallel_vectors(vectors, V):
    parallel_vectors = []
    for vector in vectors:
        if is_parallel(vector, V):
            parallel_vectors.append(vector)
    return parallel_vectors




# Define the unit vector V
V = np.array([1, 1])  # Example values, you can change these

# Define the point P
P = np.array([2, 2])  # Example values, you can change these

# Define the radii of the circles
R_max = 4  # Radius of C_max
R_min = 1  # Radius of C_min

# Calculate the slope 'a' of the line
a = V[1] / V[0]

# Calculate the y-intercept 'b' of the line
b = P[1] - a * P[0]

# Define the function of the line (y = ax + b)
def line_function(x):
    return a * x + b

# Calculate the discriminant
discriminant = (2 * a * b) ** 2 - 4 * (a ** 2 + 1) * (b ** 2 - R_max ** 2)

# Calculate the intersection points with C_max
if discriminant >= 0:
    x1 = (-2 * a * b + np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
    y1 = line_function(x1)
    x2 = (-2 * a * b - np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
    y2 = line_function(x2)
    intersection_points_C_max = [(x1, y1), (x2, y2)]
else:
    intersection_points_C_max = []

# Calculate the intersection points with C_min
discriminant = (2 * a * b) ** 2 - 4 * (a ** 2 + 1) * (b ** 2 - R_min ** 2)

if discriminant >= 0:
    x1 = (-2 * a * b + np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
    y1 = line_function(x1)
    x2 = (-2 * a * b - np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
    y2 = line_function(x2)
    intersection_points_C_min = [(x1, y1), (x2, y2)]
else:
    intersection_points_C_min = []

# Combine intersection lists to one list.
intersection_points = intersection_points_C_min + intersection_points_C_max
#print(intersection_points)


# Initialize an empty list to store the vectors
intersection_vectors = []

# Calculate the vectors
for point in intersection_points:
    vector = point - P
    intersection_vectors.append(vector)
print("List of Vectors:", intersection_vectors)


# Normalize the reference vector
normalized_reference_vector = V / np.linalg.norm(V)

# Check if vectors are parallel and in the same direction
parallel_vectors = []

for vector in intersection_vectors:
    dot_product = np.dot(normalized_reference_vector, vector)
    if dot_product > 0:
        parallel_vectors.append(vector)

# Now, parallel_vectors contains the vectors that are parallel and in the same direction as the reference vector
print("Parallel Vectors:", parallel_vectors)

mag1 = np.linalg.norm(V)
mag2 = np.linalg.norm(parallel_vectors)

scale_factor = mag1 / mag2
print("Scaling Factor: ", scale_factor)

new_V = V*scale_factor
print("Foot Vector: ", new_V)



# Print the intersection points
#print("Intersection points with C_max:", intersection_points_C_max)
#print("Intersection points with C_min:", intersection_points_C_min)

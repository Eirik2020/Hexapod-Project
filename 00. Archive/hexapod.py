# Import necessary libraries
import numpy as np

# Shortcuts
pi = np.pi



# Functions
def vector_parallel_check(ref_vector, vector):
    # Check if two vectors are parallel using dot product
    normalized_reference_vector = ref_vector / np.linalg.norm(ref_vector)
    dot_product = np.dot(normalized_reference_vector, vector)
    return dot_product > 0


def line_function(a, x, b):
    return a * x + b


def circle_line_discriminant(a, b, r):
    discriminant = (2 * a * b) ** 2 - 4 * (a ** 2 + 1) * (b ** 2 - r ** 2)
    return discriminant


def circle_line_intersection(discriminant, a, b):
    if discriminant >= 0:
        x1 = (-2 * a * b + np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
        y1 = line_function(a, x1, b)
        x2 = (-2 * a * b - np.sqrt(discriminant)) / (2 * (a ** 2 + 1))
        y2 = line_function(a, x2, b)
        intersection_points = [(x1, y1), (x2, y2)]
    else:
        intersection_points = []
    return intersection_points


def line_line_intersection(a1, b1, a2, b2):
    if a1 != a2:
        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        intersection_points = [x, y]
    else:
        intersection_points = []
    return intersection_points


def step_size(V_direction, foot_pos, joint_origin, leg_angle, workspace_angle, R_min, R_max):
    ## Find direction vector and workspace border line function coefficients  ##
    # Direction Vector
    a1 = V_direction[1] / V_direction[0]
    b1 = foot_pos[1] - a1 * foot_pos[0]

    # Right workspace border
    phi_R = leg_angle - workspace_angle/2
    a2 = np.tan(phi_R)
    b2 = joint_origin[1] - a2 * joint_origin[0]

    # Left workspace border
    phi_L = leg_angle + workspace_angle/2
    a3 = np.tan(phi_L)
    b3 = joint_origin[1] - a3 * joint_origin[0]
    

    ##  Find intersection points between direction line and workspace border  ##
    # Outer border radius
    discriminant_max_border = circle_line_discriminant(a1, b1, R_max)
    intersection_points_C_max = circle_line_intersection(discriminant_max_border, a1, b1)

    # Inner border radius
    discriminant_min_border = circle_line_discriminant(a1, b1, R_min)
    intersection_points_C_min = circle_line_intersection(discriminant_min_border, a1, b1)

    # Right border line
    intersection_points_R_border = line_line_intersection(a1, b1, a2, b2)

    # Left line
    intersection_points_L_border = line_line_intersection(a1, b1, a3, b3)

    # Combine intersection point lists.
    intersection_points = intersection_points_R_border + intersection_points_C_max + intersection_points_C_min + intersection_points_L_border
    

    ##  Determine from the intersection points, which is the correct vector  ##
    # Calculate the vectors
    intersection_vectors = []
    for point in intersection_points:
        vector = np.array(point) - foot_pos
        intersection_vectors.append(vector)

    # Normalize the reference vector
    normalized_reference_vector = V / np.linalg.norm(V)

    # Check if vectors are parallel and in the same direction
    parallel_vectors = []
    for vector in intersection_vectors:
        dot_product = np.dot(normalized_reference_vector, vector)
        if dot_product > 0:
            parallel_vectors.append(vector)
    
    # If there are several parallel vectors, find the shortest.
    if len(parallel_vectors) > 1:
        # Calculate the magnitudes of each vector
        magnitudes = [np.linalg.norm(vector) for vector in parallel_vectors]

        # Find the index of the shortest vector
        shortest_vector_index = np.argmin(magnitudes)

        # Get the shortest vector
        result_vector = parallel_vectors[shortest_vector_index]
        mag2 = np.linalg.norm(result_vector)

    return mag2
    
    

# Define the unit vector V
V = np.array([1, 1])

# Joint origin
joint_origin = [0, 0]

# Leg angle
leg_angle = 0

# Workspace angle
workspace_angle = pi/3

# Define the radii of the circles
R_max = 10
R_min = 3

# Define the point P
P = [(R_max - R_min) * np.cos(pi/6), (R_max - R_min) * np.sin(pi/6)]


mag = step_size(V, P, joint_origin, leg_angle, workspace_angle, R_min, R_max)
print(mag)




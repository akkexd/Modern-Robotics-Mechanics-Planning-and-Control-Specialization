import numpy as np
from modern_robotics import FKinBody

# Given parameters
L = 1

# End-effector zero configuration matrix M
M = np.array([[1, 0, 0, (2 + np.sqrt(3)) * L],
              [0, 1, 0, 0],
              [0, 0, 1, (1 + np.sqrt(3)) * L],
              [0, 0, 0, 1]])

# Screw axes Si in the space frame when the robot is in its zero position
B = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 1.0, 1.0, 0.0, 0.0],
              [1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
              [0.0, 2.732, 3.732, 2.0, 0.0, 0.0],
              [2.732, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, -2.732, -1.0, 0.0, 1.0, 0.0]])

# Joint variables
theta = np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6])

# Calculate the forward kinematics using FKinBody
T = FKinBody(M, B, theta)

# Print the result with sufficient decimal places
print(np.array2string(T, precision=3, suppress_small=True))

# Save the result to a log file
np.set_printoptions(precision=8, suppress=True)
with open('FKinBody_log.txt', 'w') as log_file:
    log_file.write("End-Effector Configuration Matrix T:\n")
    log_file.write(np.array2string(T, precision=8, suppress_small=True))

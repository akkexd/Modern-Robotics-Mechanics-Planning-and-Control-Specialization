import numpy as np
import math

L = 1

# Define w and q for each frame
w1 = np.array([0, 0, 1])
q1 = np.array([1, 0, 0])
v1 = np.cross(q1, w1)

w2 = np.array([0, 1, 0])
q2 = np.array([1, 0, 0])
v2 = np.cross(q2, w2)

w3 = np.array([0, 1, 0])
q3 = np.array([1 + math.sqrt(3), 0, -1])
v3 = np.cross(q3, w3)

w4 = np.array([0, 1, 0])
q4 = np.array([2 + math.sqrt(3), 0, math.sqrt(3) - 1])
v4 = np.cross(q4, w4)

w5 = np.array([0, 0, 0])
v5 = np.array([0, 0, 1])

w6 = np.array([0, 0, 1])
q6 = np.array([2 + math.sqrt(3), 0, 0])
v6 = np.cross(q6, w6)

# Create the screw axis matrix
S = np.array([
    np.concatenate((w1, v1)),
    np.concatenate((w2, v2)),
    np.concatenate((w3, v3)),
    np.concatenate((w4, v4)),
    np.concatenate((w5, v5)),
    np.concatenate((w6, v6))
]).T

# Print the matrix with sufficient decimal places
print(np.array2string(S, precision=4, suppress_small=True))

# Save the result to a log file
np.set_printoptions(precision=8, suppress=True)
with open('ScrewAxes_log.txt', 'w') as log_file:
    log_file.write("Screw Axes Matrix S:\n")
    log_file.write(np.array2string(S, precision=8, suppress_small=True))

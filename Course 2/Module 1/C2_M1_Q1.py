import math
import numpy as np

# Define L
L = 1

# Calculate the values
a = 2 + math.sqrt(3)
b = 1 + math.sqrt(3)

# Create the matrix M
M = [
    [1, 0, 0, a * L],
    [0, 1, 0, 0],
    [0, 0, 1, b * L],
    [0, 0, 0, 1]
]

# Convert to numpy array for easier handling
M_np = np.array(M)

# Print the matrix with sufficient decimal places
print(np.array2string(M_np, precision=4, suppress_small=True))

# Save the result to a log file
np.set_printoptions(precision=8, suppress=True)
with open('M_np_log.txt', 'w') as log_file:
    log_file.write("M_np Matrix:\n")
    log_file.write(np.array2string(M_np, precision=8, suppress_small=True))

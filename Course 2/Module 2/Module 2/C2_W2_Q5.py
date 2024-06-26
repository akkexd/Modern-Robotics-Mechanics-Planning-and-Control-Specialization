import numpy as np

J_v = np.array([
    [0, -1, 0, 0, -1, 0, 0],
    [0, 0, 1, 0, 0, 1, 0],
    [1, 0, 0, 1, 0, 0, 1]
])

# Perform Singular Value Decomposition
U, S, Vt = np.linalg.svd(J_v)

# U contains the directions of the principal axes
# S contains the lengths of the principal semi-axes
print("U (directions of principal axes):")
print(U)

print("S (lengths of principal semi-axes):")
print(S)

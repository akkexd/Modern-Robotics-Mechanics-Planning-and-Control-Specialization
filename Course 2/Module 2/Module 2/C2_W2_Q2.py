import numpy as np

# Define the lengths of the links
L1 = L2 = L3 = L4 = 1

# Define the joint angles in radians
theta1 = 0
theta2 = 0
theta3 = np.pi / 2
theta4 = -np.pi / 2

# Define the applied wrench in the body frame
Fb = np.array([0, 0, 10, 10, 10, 0])

# Compute the sines and cosines of the sums of joint angles
s4 = np.sin(theta4)
c4 = np.cos(theta4)
s34 = np.sin(theta3 + theta4)
c34 = np.cos(theta3 + theta4)
s234 = np.sin(theta2 + theta3 + theta4)
c234 = np.cos(theta2 + theta3 + theta4)

# Compute the body Jacobian J_b(theta)
J_b = np.array([
    [1, 1, 1, 1],
    [-1, -1, -1, 0],
    [3, 2, 1, 1]
])

# Transpose the body Jacobian
J_b_T = J_b.T

# Compute the torques at each joint
tau = np.dot(J_b_T, Fb[2:5])  # Use only the relevant part of the wrench (forces and moments in the plane)

# Print the torques
print(f"Torques at each joint: {tau}")

# Expected output: [30.00, 20.00, 10.00, 20.00]

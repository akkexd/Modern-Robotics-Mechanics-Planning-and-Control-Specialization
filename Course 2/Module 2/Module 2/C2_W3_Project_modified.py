import numpy as np
from scipy.linalg import expm, logm
import logging
import csv
import os

# Set up logging
logging.basicConfig(filename='robot_kinematics.log', level=logging.INFO,
                    format='%(asctime)s %(levelname)s:%(message)s')

# Function Definitions
def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])

def vec_to_se3(V):
    return np.vstack((np.hstack((skew(V[:3]), V[3:].reshape((3, 1)))),
                      np.array([[0, 0, 0, 0]])))

def se3_to_vec(se3):
    return np.hstack((se3[:3, 3], se3[2, 1], se3[0, 2], se3[1, 0]))

def TransInv(T):
    R = T[:3, :3]
    p = T[:3, 3]
    Rt = R.T
    return np.vstack((np.hstack((Rt, -Rt @ p.reshape((3, 1)))), np.array([0, 0, 0, 1])))

def MatrixLog6(T):
    R, p = T[:3, :3], T[:3, 3]
    if np.allclose(R, np.eye(3)):
        return np.vstack((np.hstack((np.zeros((3, 3)), p.reshape((3, 1)))),
                          np.array([[0, 0, 0, 0]])))
    acosinput = (np.trace(R) - 1) / 2.0
    acosinput = min(max(acosinput, -1), 1)
    theta = np.arccos(acosinput)
    logR = theta / (2 * np.sin(theta)) * (R - R.T)
    return np.vstack((np.hstack((logR, (np.eye(3) - logR / 2) @ p.reshape((3, 1)))),
                      np.array([[0, 0, 0, 0]])))

def FKinBody(M, Blist, thetalist):
    T = np.eye(4)
    for i in range(len(thetalist) - 1, -1, -1):
        T = expm(vec_to_se3(-Blist[:, i] * thetalist[i])) @ T
    return T @ M

# Define UR5 robot parameters
W1 = 0.109
W2 = 0.082
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095

Blist = np.array([
    [0, 1, 0, W1 + W2, 0, L1 + L2],
    [0, 0, 1, H2, L1 - L2, 0],
    [0, 0, 1, H2, L2, 0],
    [0, 0, 1, H2, 0, 0],
    [0, -1, 0, -W2, 0, 0],
    [0, 0, 1, 0, 0, 0]
], dtype=np.float64).T

M = np.array([
    [-1, 0, 0, L1 + L2],
    [0, 0, 1, W1 + W2],
    [0, 1, 0, H1 - H2],
    [0, 0, 0, 1]
], dtype=np.float64)

Tsd = np.array([
    [0, 1, 0, -0.5],
    [0, 0, 1, 0.1],
    [1, 0, 0, 0.1],
    [0, 0, 0, 1]
], dtype=np.float64)

# Final joint angles from the last iteration
thetalist_final = np.array([-1.442044924, -1.511513979, 5.37441655, 5.179392523, -3.12338099, 0.307338989], dtype=np.float64)

# Compute the final end-effector configuration
T_final = FKinBody(M, Blist, thetalist_final)

# Compute the error twist
Vb = se3_to_vec(MatrixLog6(TransInv(T_final) @ Tsd))

# Check if the error is within the tolerance
eomg = 1e-3
ev = 1e-3
angular_error = np.linalg.norm(Vb[:3])
linear_error = np.linalg.norm(Vb[3:])

# Log the final results
logging.info(f"Final joint angles: {thetalist_final}")
logging.info(f"Final end-effector configuration:\n{T_final}")
logging.info(f"Desired end-effector configuration:\n{Tsd}")
logging.info(f"Error twist V_b: {Vb}")
logging.info(f"Angular error magnitude ||ω_b||: {angular_error:.4f}")
logging.info(f"Linear error magnitude ||v_b||: {linear_error:.4f}")
logging.info(f"Success: {angular_error <= eomg and linear_error <= ev}")

# Save results to CSV
output_file = 'robot_kinematics_results.csv'
fieldnames = ['Iteration', 'Theta1', 'Theta2', 'Theta3', 'Theta4', 'Theta5', 'Theta6',
              'T11', 'T12', 'T13', 'T14', 'T21', 'T22', 'T23', 'T24', 'T31', 'T32', 'T33', 'T34',
              'Angular_Error', 'Linear_Error', 'Success']

# Check if the file exists
file_exists = os.path.isfile(output_file)

with open(output_file, mode='a', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    
    # Write header if file does not exist
    if not file_exists:
        writer.writeheader()

    # Write data row
    writer.writerow({
        'Iteration': 50,
        'Theta1': thetalist_final[0],
        'Theta2': thetalist_final[1],
        'Theta3': thetalist_final[2],
        'Theta4': thetalist_final[3],
        'Theta5': thetalist_final[4],
        'Theta6': thetalist_final[5],
        'T11': T_final[0, 0],
        'T12': T_final[0, 1],
        'T13': T_final[0, 2],
        'T14': T_final[0, 3],
        'T21': T_final[1, 0],
        'T22': T_final[1, 1],
        'T23': T_final[1, 2],
        'T24': T_final[1, 3],
        'T31': T_final[2, 0],
        'T32': T_final[2, 1],
        'T33': T_final[2, 2],
        'T34': T_final[2, 3],
        'Angular_Error': angular_error,
        'Linear_Error': linear_error,
        'Success': angular_error <= eomg and linear_error <= ev
    })

print(f"Final end-effector configuration:\n{T_final}")
print(f"Desired end-effector configuration:\n{Tsd}")
print(f"Error twist V_b: {Vb}")
print(f"Angular error magnitude ||ω_b||: {angular_error:.4f}")
print(f"Linear error magnitude ||v_b||: {linear_error:.4f}")
print(f"Success: {angular_error <= eomg and linear_error <= ev}")

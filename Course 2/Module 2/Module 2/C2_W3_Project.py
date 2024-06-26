import numpy as np
from scipy.linalg import expm, logm

# Function to convert a screw axis to a matrix representation
def VecTose3(V):
    return np.array([
        [0, -V[2], V[1], V[3]],
        [V[2], 0, -V[0], V[4]],
        [-V[1], V[0], 0, V[5]],
        [0, 0, 0, 0]
    ])

# Function to convert a transformation matrix to a screw axis
def se3ToVec(se3mat):
    return np.array([se3mat[2, 1], se3mat[0, 2], se3mat[1, 0], se3mat[0, 3], se3mat[1, 3], se3mat[2, 3]])

# Function to compute the matrix exponential of a screw axis
def MatrixExp6(se3mat):
    return expm(se3mat)

# Function to compute the matrix logarithm of a transformation matrix
def MatrixLog6(T):
    return logm(T)

# Function to compute forward kinematics using the body frame
def FKinBody(M, Blist, thetalist):
    T = np.eye(4)
    for i in range(len(thetalist)-1, -1, -1):
        T = np.dot(MatrixExp6(VecTose3(Blist[:, i] * thetalist[i])), T)
    return np.dot(T, M)

# Function to compute the body Jacobian
def JacobianBody(Blist, thetalist):
    Jb = Blist.copy()
    T = np.eye(4)
    for i in range(len(thetalist)-1, 0, -1):
        T = np.dot(T, MatrixExp6(-VecTose3(Blist[:, i] * thetalist[i])))
        Jb[:, i-1] = np.dot(Adjoint(T), Blist[:, i-1])
    return Jb

# Function to compute the adjoint representation of a transformation matrix
def Adjoint(T):
    R, p = T[0:3, 0:3], T[0:3, 3]
    return np.vstack((np.hstack((R, np.zeros((3, 3)))), np.hstack((VecToso3(p) @ R, R))))

# Function to convert a vector to skew-symmetric matrix
def VecToso3(omg):
    return np.array([
        [0, -omg[2], omg[1]],
        [omg[2], 0, -omg[0]],
        [-omg[1], omg[0], 0]
    ])

# The main function for inverse kinematics iterations
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 5  # Limiting to 5 iterations
    Vb = se3ToVec(MatrixLog6(np.dot(np.linalg.inv(FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm(Vb[0:3]) > eomg or np.linalg.norm(Vb[3:6]) > ev
    iterates = [thetalist.copy()]
    log_data = []

    while err and i < maxiterations:
        log_entry = {
            "Iteration": i,
            "Joint Vector": thetalist.copy(),
            "SE(3) End-Effector Config": FKinBody(M, Blist, thetalist),
            "Error Twist V_b": Vb,
            "Angular Error Magnitude": np.linalg.norm(Vb[0:3]),
            "Linear Error Magnitude": np.linalg.norm(Vb[3:6])
        }
        log_data.append(log_entry)
        
        # Damped least squares method with increased damping factor
        Jb = JacobianBody(Blist, thetalist)
        damping_factor = 1.6  # Further increase damping factor for stability
        thetalist += np.dot(np.dot(Jb.T, np.linalg.inv(np.dot(Jb, Jb.T) + damping_factor**2 * np.eye(6))), Vb)
        
        iterates.append(thetalist.copy())
        i += 1
        Vb = se3ToVec(MatrixLog6(np.dot(np.linalg.inv(FKinBody(M, Blist, thetalist)), T)))
        err = np.linalg.norm(Vb[0:3]) > eomg or np.linalg.norm(Vb[3:6]) > ev

    log_entry = {
        "Iteration": i,
        "Joint Vector": thetalist.copy(),
        "SE(3) End-Effector Config": FKinBody(M, Blist, thetalist),
        "Error Twist V_b": Vb,
        "Angular Error Magnitude": np.linalg.norm(Vb[0:3]),
        "Linear Error Magnitude": np.linalg.norm(Vb[3:6])
    }
    log_data.append(log_entry)

    success = not err
    iterates = np.array(iterates)

    # Save iterates to a CSV file
    np.savetxt('iterates.csv', iterates, delimiter=',')

    # Save logs to a text file
    with open('iteration_log.txt', 'w') as log_file:
        for entry in log_data:
            log_file.write(f"Iteration {entry['Iteration']}:\n\n")
            log_file.write(f"Joint Vector :\n")
            log_file.write(" ".join([f"{x: .4f}" for x in entry['Joint Vector']]) + "\n\n")
            log_file.write(f"SE(3) end-effector config:\n")
            for row in entry["SE(3) End-Effector Config"]:
                log_file.write(" ".join([f"{x: .4f}" for x in row]) + "\n")
            log_file.write("\nerror twist V_b:\n")
            log_file.write(" ".join([f"{x: .4f}" for x in entry['Error Twist V_b']]) + "\n\n")
            log_file.write(f"angular error magnitude ||omega_b||: {entry['Angular Error Magnitude']: .4f}\n")
            log_file.write(f"linear error magnitude ||v_b||: {entry['Linear Error Magnitude']: .4f}\n\n")

    return thetalist, success


# UR5 robot parameters
W1 = 0.109
W2 = 0.082
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095

M = np.array([
    [-1, 0, 0, L1 + L2],
    [0, 0, 1, W1 + W2],
    [0, 1, 0, H1 - H2],
    [0, 0, 0, 1]
])

Blist = np.array([
    [0, 1, 0, W1 + W2, 0, L1 + L2],
    [0, 0, 1, H2, L1 - L2, 0],
    [0, 0, 1, H2, L2, 0],
    [0, 0, 1, H2, 0, 0],
    [0, -1, 0, -W2, 0, 0],
    [0, 0, 1, 0, 0, 0]
]).T

T_sd = np.array([
    [0, 1, 0, -0.5],
    [0, 0, -1, 0.1],
    [-1, 0, 0, 0.1],
    [0, 0, 0, 1]
])

# Initial guess for the joint angles
thetalist0 = [6.0, -2.5, 4.5, -5.0, 3.5, 1.5]

# Error tolerances
eomg = 0.001
ev = 0.0001

# Run the inverse kinematics iterations
final_thetalist, success = IKinBodyIterates(Blist, M, T_sd, thetalist0, eomg, ev)

print(f"Final joint angles: {final_thetalist}")
print(f"Success: {success}")
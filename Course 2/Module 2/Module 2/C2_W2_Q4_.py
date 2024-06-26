import numpy as np
from modern_robotics import MatrixExp6, VecTose3, Adjoint

def JacobianBody(Blist, thetalist):
    Jb = np.array(Blist).copy().astype(float)
    T = np.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = np.dot(T, MatrixExp6(VecTose3(np.array(Blist)[:, i + 1] * -thetalist[i + 1])))
        Jb[:, i] = np.dot(Adjoint(T), np.array(Blist)[:, i])
    return Jb

# Define Blist
Blist = np.array([[0, 1, 0, 3, 0, 0], [-1, 0, 0, 0, 3, 0], [0, 0, 0, 0, 0, 1]]).T

# Define thetalist
thetalist = np.array([np.pi / 2, np.pi / 2, 1])

# Compute the body Jacobian
Jb = JacobianBody(Blist, thetalist)
print(np.round(Jb,4))

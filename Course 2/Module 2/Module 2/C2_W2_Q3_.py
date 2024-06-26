import numpy as np
from scipy.linalg import expm

def VecTose3(V):
    """
    Convert a spatial velocity vector into a 4x4 matrix in se(3).
    """
    return np.array([[0, -V[2], V[1], V[3]],
                     [V[2], 0, -V[0], V[4]],
                     [-V[1], V[0], 0, V[5]],
                     [0, 0, 0, 0]])

def MatrixExp6(se3mat):
    """
    Calculate the matrix exponential of a matrix in se(3).
    """
    return expm(se3mat)

def Adjoint(T):
    """
    Calculate the adjoint representation of a transformation matrix T.
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_hat = np.array([[0, -p[2], p[1]],
                      [p[2], 0, -p[0]],
                      [-p[1], p[0], 0]])
    return np.vstack((
        np.hstack((R, np.zeros((3, 3)))),
        np.hstack((np.dot(p_hat, R), R))
    ))

def JacobianSpace(Slist, thetalist):
    Js = Slist.copy()
    T = np.eye(4)
    for i in range(1, len(thetalist)):
        T = np.dot(T, MatrixExp6(VecTose3(Slist[:, i - 1] * thetalist[i - 1])))
        Js[:, i] = np.dot(Adjoint(T), Slist[:, i])
    return Js

# Given screw axes
Slist = np.array([
    [0, 0, 1, 0, 0, 0],
    [1, 0, 0, 0, 2, 0],
    [0, 0, 0, 0, 1, 0]
]).T

# Joint angles in radians
thetalist = [np.pi / 2, np.pi / 2, 1]

# Calculate the space Jacobian
Js = JacobianSpace(Slist, thetalist)
print("Space Jacobian Js at the given configuration:")
print(Js)

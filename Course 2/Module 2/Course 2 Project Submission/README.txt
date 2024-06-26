Inverse Kinematics (IK) using Python
This repository contains Python code for solving inverse kinematics (IK) of a robotic arm using iterative methods with damped least squares. The implementation utilizes numerical methods from numpy and scipy libraries.

Files:
ik_functions.py: Contains functions for computing forward kinematics, Jacobian matrices, matrix exponentials, and logarithms required for IK calculations.

ik_main.py: Main script that initializes the robot parameters, performs IK iterations, and prints the final joint angles and success status.

iterates.csv: CSV file containing the joint angles at each iteration during IK calculations.

iteration_log.txt: Text file logging details of each iteration including joint vectors, end-effector configurations, error twists, and error magnitudes.

Code Structure:

damping_factor = 1.6  # Further increase damping factor for stability

VecTose3(V): Converts a screw axis vector V to its corresponding 4x4 matrix representation.

se3ToVec(se3mat): Converts a transformation matrix se3mat to a screw axis vector.

MatrixExp6(se3mat): Computes the matrix exponential of a 4x4 se(3) matrix se3mat.

MatrixLog6(T): Computes the matrix logarithm of a 4x4 transformation matrix T.

FKinBody(M, Blist, thetalist): Computes the forward kinematics of the robot given the base transformation matrix M, joint screw axes Blist, and joint angles thetalist.

JacobianBody(Blist, thetalist): Computes the body Jacobian matrix of the robot.

Adjoint(T): Computes the adjoint representation of a 4x4 transformation matrix T.

VecToso3(omg): Converts a 3-vector omg to its corresponding 3x3 skew-symmetric matrix.

IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev): Main function that performs iterative IK calculations using damped least squares method with specified error tolerances eomg (angular error) and ev (linear error).

Robot Parameters (UR5):
Base frame parameters:

W1 = 0.109
W2 = 0.082
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
End-effector target transformation matrix T_sd.

Output:
The final joint angles after convergence and success status are printed to the console. The iterates.csv file contains a record of joint angles at each iteration. The iteration_log.txt file provides detailed logs of each iteration including joint vectors, end-effector configurations, error twists, and error magnitudes.

Conclusion:
This IK implementation aims to solve for the joint angles of a UR5 robotic arm using an iterative approach with enhanced stability through damped least squares. Adjustments to the damping factor and error tolerances can be made based on specific robot dynamics to further optimize performance.
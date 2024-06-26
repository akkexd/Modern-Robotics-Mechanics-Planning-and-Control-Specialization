import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import MatrixExp6, VecTose3, Adjoint

from ModernRobotics.packages.Python.modern_robotics.core import FKinBody
from ModernRobotics.packages.Python.modern_robotics.core import MatrixLog6
from ModernRobotics.packages.Python.modern_robotics.core import TransInv
from ModernRobotics.packages.Python.modern_robotics.core import se3_to_vec

# Load iteration data from CSV
data = pd.read_csv("iterates.csv")

# Function to compute errors at each iteration
def compute_errors(data, Blist, M, Tsd):
    errors = []
    for index, row in data.iterrows():
        thetalist = row[1:].values
        T = FKinBody(M, Blist, thetalist)
        Vb = se3_to_vec(MatrixLog6(TransInv(T) @ Tsd))
        errors.append({
            'Iteration': row[0],
            'AngularError': np.linalg.norm(Vb[:3]),
            'LinearError': np.linalg.norm(Vb[3:])
        })
    return pd.DataFrame(errors)

# Compute errors
errors = compute_errors(data, Blist, M, Tsd)

# Plot errors
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.plot(errors['Iteration'], errors['AngularError'], label='Angular Error')
plt.xlabel('Iteration')
plt.ylabel('Error Magnitude')
plt.title('Angular Error over Iterations')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(errors['Iteration'], errors['LinearError'], label='Linear Error')
plt.xlabel('Iteration')
plt.ylabel('Error Magnitude')
plt.title('Linear Error over Iterations')
plt.legend()

plt.tight_layout()
plt.show()

# Analyze joint vector changes
joint_changes = np.diff(data.iloc[:, 1:], axis=0)
joint_changes_magnitude = np.linalg.norm(joint_changes, axis=1)

plt.figure()
plt.plot(range(1, len(joint_changes_magnitude)+1), joint_changes_magnitude)
plt.xlabel('Iteration')
plt.ylabel('Joint Vector Change Magnitude')
plt.title('Change in Joint Vector over Iterations')
plt.show()

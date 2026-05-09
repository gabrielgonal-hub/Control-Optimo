import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

R1 = 1000
R2 = 1000
R3 = 1000
C1 = 1e-6
C2 = 1e-6

# matrices del sistema
A = np.array([[-1/(R1*C1)-1/(R2*C1), 1/(R2*C1)],
            [1/(R2*C2), -1/(R2*C2)-1/(R3*C2)]])

B = np.array([[1/(R1*C1)],
            [0]])

Q = np.array([[1, 0],
            [0, 5]])

R = np.array([[1]])

# resolver Riccati
P = solve_continuous_are(A,B,Q,R)

print("P =", P)

# ganancia LQR
K = np.linalg.inv(R) @ B.T @ P
print("K =", K)
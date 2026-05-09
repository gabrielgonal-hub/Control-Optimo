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
A = np.array([[-1/(R1*C1)-1/(R2*C2), 1/(R2*C1)],
            [1/(R2*C2), -1/(R2*C2)-1/(R3*C2)]])

B = np.array([[1/(R1*C1)],
            [0]])

C = np.array([[0, 1]])

# sistema aumentado (LQI)

Aa = np.block([
    [np.zeros((1,1)), -C],
    [np.zeros((2,1)),  A]
])

Ba = np.vstack([
    np.zeros((1,1)),
    B
])

print("Aa =\n", Aa)
print("Ba =\n", Ba)

Qa = np.diag([10, 1, 5])

print("Qa =\n", Qa)

R = np.array([[1]])

# resolver Riccati
P = solve_continuous_are(Aa,Ba,Qa,R)

print("P =", P)

# ganancia LQI
K = np.linalg.inv(R) @ Ba.T @ P
print("K =", K)
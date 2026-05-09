import numpy as np
from scipy.linalg import solve_continuous_are
from sympy import symbols, Matrix, solve, pprint

R1, C1, R2, C2, R3 = symbols('R1 C1 R2 C2 R3')

A = Matrix([[-1/(R1*C1)-1/(R2*C1), 1/(R2*C1)],
            [1/(R2*C2), -1/(R2*C2)-1/(R3*C2)]])

B = Matrix([[1/(R1*C1)],
            [0]])
            
Q = Matrix([[1, 0],
              [0, 5]])
              
R = Matrix([1])

p11, p12, p22 = symbols('p11 p12 p22')

P = Matrix([[p11, p12],
            [p12, p22]])

Rinv = R.inv()

riccati = A.T*P + P*A - P*B*Rinv*B.T*P + Q

pprint(riccati)
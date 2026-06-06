import numpy as np
from scipy.linalg import solve_continuous_are

# ── ① Physical parameters (SrOmBURo — values confirmed in control.py) ──
m_b      = 8.0           # body mass [kg]
l        = 0.65          # wheel axis to body CoM height [m]
I_b      = m_b*l**2/3    # body inertia (rod approximation — measured/CAD preferred)
m_w      = 1.0           # wheel mass [kg]
R_w      = 0.101         # wheel radius [m]              ← control.py
r        = 0.0142        # roller radius [m]             ← control.py
N_ROLLER = 4.0           # roller gear ratio             ← control.py
I_w      = 0.5*m_w*R_w**2  # wheel inertia
g        = 9.81

# ── ② Linearisation: M q̈ + Kg q = Bt·u  (q = [φ, θ]) ──
M  = np.array([[(m_b+m_w)*R_w**2 + I_w,  m_b*R_w*l       ],
               [ m_b*R_w*l            ,  m_b*l**2 + I_b  ]])
Kg = np.array([[0, 0],
               [0, -m_b*g*l]])      # gravity = unstable term (negative stiffness for inverted pendulum)
Bt = np.array([[ 1.0],
               [-1.0]])             # torque: wheel +, body −
Minv = np.linalg.inv(M)

# state x = [φ, θ, φ̇, θ̇]
A = np.zeros((4,4)); A[0,2]=1; A[1,3]=1
A[2:4,0:2] = -Minv @ Kg
B = np.zeros((4,1)); B[2:4,0:1] = Minv @ Bt

# ── ③ Weights (tune these) ──
Q     = np.diag([1.0, 50.0, 0.1, 1.0])  # penalise θ heavily, φ lightly
R_lqr = np.array([[1.0]])               # larger = softer / less aggressive input

# ── ④⑤ Solve Riccati and extract K ──
P = solve_continuous_are(A, B, Q, R_lqr)
K = np.linalg.inv(R_lqr) @ B.T @ P
print("K =", K)   # u = -K·x  →  [K_φ, K_θ, K_φ̇, K_θ̇]

# ── ⑥ Empirical gain comparison from control.py (pitch axis) ──
# control.py: v_wheel = -(K11*pitch + K12*pitchdot + KI_PITCH*pitch_int)
K11_emp    = 85.0   # pitch angle gain  (control.py K11)
K12_emp    = 11.0   # pitchdot gain     (control.py K12)
KI_PITCH   =  2.0   # pitch integral    (control.py KI_PITCH)

# control.py: v_roller = (K21*roll + K22*rolldot + KI_ROLL*roll_int)*N_ROLLER
K21_emp    = 70.0   # roll angle gain   (control.py K21)
K22_emp    =  2.0   # rolldot gain      (control.py K22)
KI_ROLL    =  1.5   # roll integral     (control.py KI_ROLL)

print(f"\nEmpirical gains (control.py — pitch):  K_θ={K11_emp}, K_θ̇={K12_emp}, Ki={KI_PITCH}")
print(f"LQR gains                               K_θ={K[0,1]:.3f}, K_θ̇={K[0,3]:.3f}")
print(f"\nEmpirical gains (control.py — roll):   K_roll={K21_emp}, K_rolldot={K22_emp}, Ki={KI_ROLL}")

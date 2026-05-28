import numpy as np
from scipy.linalg import solve_continuous_are

# ── Physical parameters — UPDATED from SolidWorks mass properties ──────────────
BODY_MASS           = 0.253195   # kg  (253.195 g)
BODY_LENGTH         = 1.1938     # m   (47 inches)

# CoM at geometric centre of tube (SolidWorks: X=Y=Z=0 relative to part origin)
# Mount the tube so its midpoint is the pivot — CoM height from wheel contact:
BODY_HEIGHT         = BODY_LENGTH / 2.0   # 0.5969 m  ← adjust if mount differs

# Moments of inertia — converted from g·mm² → kg·m²
# Lxx = Lzz = 30153526.715 g·mm² = 0.030154 kg·m²  (bending axes — dominant)
# Lyy = 166613.541 g·mm²          = 1.6661e-4 kg·m² (spin axis — small)
I_BEND  = 30153526.715e-9   # kg·m²  — Ixx = Izz, used for tilt dynamics
I_SPIN  = 166613.541e-9     # kg·m²  — Iyy, spin about tube axis (small)

# Wheel / roller geometry
WHEEL_RADIUS        = 0.075   # m
ROLLER_EFF_RADIUS   = 0.030   # m
GRAVITY             = 9.81    # m/s²
DT                  = 0.005   # s  (200 Hz)

KT                  = 0.35    # Nm/A  (BEAR motor torque constant)

# ── Linearised plant A, B matrices ─────────────────────────────────────────────
# State: x = [θ_l, θ̇_l, θ_lat, θ̇_lat, ψ̇_l, ψ̇_lat]
# Using updated mass and I_BEND:
#   ω_n² = m·g·h / (I + m·h²)   (compound pendulum natural frequency)
g, m, h = GRAVITY, BODY_MASS, BODY_HEIGHT
I        = I_BEND
r        = WHEEL_RADIUS

_omega2 = (m * g * h) / (I + m * h**2)   # compound pendulum: ~15.5 rad²/s²
_b      = h / (I + m * h**2)              # input coupling
_c      = 1.0 / (m * r)

A_NOMINAL = np.array([
    [0,       1,       0,       0,       0,    0   ],
    [_omega2, 0,       0,       0,       0,    0   ],
    [0,       0,       0,       1,       0,    0   ],
    [0,       0,       _omega2, 0,       0,    0   ],
    [-_c,     0,       0,       0,       0,    0   ],
    [0,       0,       -_c,     0,       0,    0   ],
])

B_NOMINAL = np.array([
    [0,     0   ],
    [-_b,   0   ],
    [0,     0   ],
    [0,     -_b ],
    [_c,    0   ],
    [0,     _c  ],
])

# ── PID outer loop gains ────────────────────────────────────────────────────────
PID_KP = 1.2
PID_KI = 0.20
PID_KD = 0.05
INTEGRATOR_CLAMP = 5.0   # rad/s·s — anti-windup

# ── Robustness gain (ρ·vers term from the arm paper) ───────────────────────────
# ρ scales the discontinuous robustness injection.
# Start small (0.1–0.5) and increase until disturbance rejection improves
# without causing chattering.
RHO = 0.3

# ── Motor limits ────────────────────────────────────────────────────────────────
MAX_TORQUE      = 3.0    # Nm
MAX_TORQUE_RATE = 30.0   # Nm/s
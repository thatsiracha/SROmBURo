#!/usr/bin/env python3
"""
compute_lqr.py — Standalone LQR gain computation for SrOmBURo.

Usage:
    python3 compute_lqr.py
    → prints K_PITCH, K_ROLL, eigenvalue sanity, gain ratios vs hand-tuned.
    → copy the printed K arrays into control3.py constants.

State per axis:   x = [φ, θ, φ̇, θ̇]
  φ   = wheel CONTACT angle [rad]
        PITCH: φ = wheel-motor encoder (1:1)
        ROLL:  φ = N_ROLLER × roller-motor encoder  ← scale in controller
  θ   = body lean angle [rad]  (sign convention: positive = lean backward)
  u   = MOTOR torque [Nm]
        PITCH: u_motor = u_lqr  (direct drive)
        ROLL:  u_motor = u_lqr  (gear already in B; K gives motor torque directly)
"""

import numpy as np
from scipy.linalg import solve_continuous_are

# ── Physical parameters ────────────────────────────────────────────────────────
# [MEASURE] = replace with real value before trusting the gains.
# [OK]      = already measured / confirmed.

m_b      = 8.0           # body mass [kg]                  [MEASURE] weigh on scale
l        = 0.65          # wheel-axle to body CoM [m]      [MEASURE] tape/CAD — most sensitive param
I_b      = m_b*l**2/3   # body inertia about axle [kg·m²] [MEASURE] rod approx; use bifilar pendulum or CAD
m_w      = 1.0           # effective wheel + rotor mass [kg][MEASURE] estimate
R        = 0.101         # wheel radius [m]                 [OK]
I_w      = 0.5*m_w*R**2 # wheel inertia [kg·m²]           [MEASURE] solid-disk approx
g        = 9.81

N_ROLLER = 4.0           # roller gear coupling: φ̇_contact = N · ω_motor  [OK]

kt       = 0.35          # motor torque constant [Nm/A]   from Omburo.py  [OK]
I_MAX    = 6.0           # current limit [A]               from Omburo.py  [OK]
TAU_MAX  = kt * I_MAX    # max motor torque = 2.1 Nm

# ── LQR tuning weights ─────────────────────────────────────────────────────────
# Q = state cost matrix, R = control effort cost.
#
# Reading the gain ratios (printed below):
#   K_θ̇/K_θ  = optimal damping ratio; compare to K12/K11 in hand-tuned gains
#   K_φ/K_θ   = how hard the controller fights drift vs tilt
#   K_φ̇/K_θ  = wheel-velocity damping relative to angle gain
#
# Tuning recipe:
#   1. Increase Q[1,1] (θ weight) if robot falls too easily.
#   2. Increase Q[0,0] (φ weight) if robot drifts without correcting.
#   3. Increase R if output is jerky / motors saturate frequently.
#   4. Never tune Q and R simultaneously — change one at a time.

Q_PITCH = np.diag([1.0,  200.0,  0.5,  4.0])   # [φ, θ, φ̇, θ̇]
Q_ROLL  = np.diag([1.0,  200.0,  0.5,  4.0])
R_PITCH = np.array([[100.0]])  # higher = softer current command for first hardware tests
R_ROLL  = np.array([[100.0]])


# ── Model ─────────────────────────────────────────────────────────────────────
def build_model(gear_ratio: float = 1.0, motor_sign: float = 1.0):
    """
    Linearised inverted-pendulum-on-wheel state-space model.

    gear_ratio=1.0       → PITCH axis (direct drive; u = wheel motor torque)
    gear_ratio=N_ROLLER  → ROLL axis  (u = roller motor torque; B scaled so
                           the LQR K already accounts for the gear advantage)

    motor_sign: sign of BEAR positive-iq relative to model's positive-φ direction.
      +1  (ROLL)  : BEAR positive iq → roller spins in positive-φ direction
      -1  (PITCH) : BEAR positive iq → wheel spins in NEGATIVE-φ direction
                    (wheel motor is mounted inverted; readback negates vel_w to compensate)

    With motor_sign=-1, the solved K gains are all POSITIVE, so:
        u_pitch = -(K_PITCH @ x_pitch)  is NEGATIVE when states are positive,
    matching control2.py's  v_wheel = -(K11·pitch + …)  sign structure.

    Contact angle state: x = [φ_contact, θ, φ̇_contact, θ̇]
    For ROLL, feed x_roll = [N·φ_motor, roll, N·φ̇_motor, rolldot] to the K.
    """
    M = np.array([
        [(m_b + m_w)*R**2 + I_w,  m_b*R*l       ],
        [ m_b*R*l,                  m_b*l**2 + I_b]
    ])
    # Linearised gravity: negative stiffness → unstable at θ=0
    Kg = np.array([[0.0,          0.0       ],
                   [0.0,  -m_b * g * l      ]])
    # Generalised force: positive u accelerates wheel in positive-φ direction
    # and creates equal-and-opposite reaction on body.
    # motor_sign flips both components for an inverted-mounted motor.
    Bt = np.array([[motor_sign], [-motor_sign]])

    Minv = np.linalg.inv(M)

    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2:4, 0:2] = -Minv @ Kg           # gravity + coupling terms

    B = np.zeros((4, 1))
    B[2:4, 0:1] = Minv @ Bt * gear_ratio   # scaled for motor torque input

    return A, B


def compute_lqr(A, B, Q, R):
    """Solve continuous-time ARE; return K s.t. u = -K @ x is LQ-optimal."""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.inv(R) @ B.T @ P


def print_eigenvalues(A, label):
    evals = np.linalg.eigvals(A)
    max_real = max(ev.real for ev in evals)
    expected = np.sqrt(g / l)
    print(f"\n  {label} open-loop eigenvalues:")
    for ev in sorted(evals, key=lambda e: e.real, reverse=True):
        tag = "  ← unstable" if abs(ev.real - max_real) < 1e-6 else ""
        print(f"      {ev.real:+8.4f}  {'+' if ev.imag >= 0 else ''}{ev.imag:.4f}j{tag}")
    assert max_real > 0, "No unstable pole — check model signs!"
    print(f"  √(g/l) reference = {expected:.4f} rad/s")
    return evals


if __name__ == "__main__":
    sep = "=" * 58

    print(sep)
    print("  SrOmBURo LQR Gain Computation")
    print(f"  Hardware limits: TAU_MAX = {TAU_MAX:.2f} Nm "
          f"({I_MAX:.0f} A × {kt:.2f} Nm/A)")
    print(sep)

    # ── PITCH axis ────────────────────────────────────────────────────────────
    # motor_sign=-1: wheel motor BEAR encoder is mounted inverted.
    # This makes all K_PITCH gains POSITIVE, so u = -(K@x) is NEGATIVE when
    # states are positive — same sign structure as control2.py's v_wheel = -(K11·pitch+…).
    A_p, B_p = build_model(gear_ratio=1.0, motor_sign=-1.0)
    print_eigenvalues(A_p, "PITCH")
    K_p = compute_lqr(A_p, B_p, Q_PITCH, R_PITCH)[0]   # shape (4,)
    cl_eigs_p = np.linalg.eigvals(A_p - B_p @ K_p.reshape(1, -1))
    assert all(ev.real < 0 for ev in cl_eigs_p), \
        f"PITCH closed-loop NOT stable: {cl_eigs_p}"
    assert all(k > 0 for k in K_p), \
        f"PITCH K not all positive (sign issue?): {K_p}"
    Kp_phi, Kp_th, Kp_phidot, Kp_thdot = K_p

    print(f"\n  PITCH: u_pitch = -K_PITCH @ [phi_w, pitch, phidot_w, pitchdot]")
    print(f"    K_φ    = {Kp_phi:9.4f}  Nm/rad    (all gains POSITIVE)")
    print(f"    K_θ    = {Kp_th:9.4f}  Nm/rad")
    print(f"    K_φ̇   = {Kp_phidot:9.4f}  Nm·s/rad")
    print(f"    K_θ̇   = {Kp_thdot:9.4f}  Nm·s/rad")
    print(f"  Gain ratios (dimensionless — compare to velocity hand-tuning):")
    print(f"    K_φ/K_θ   = {Kp_phi/Kp_th:.5f}   (drift correction vs tilt; hand: {0.03/85.0:.5f})")
    print(f"    K_θ̇/K_θ  = {Kp_thdot/Kp_th:.5f}   (damping;      hand: {11.0/85.0:.5f})")
    print(f"    K_φ̇/K_θ  = {Kp_phidot/Kp_th:.5f}   (wheel vel;    hand: {0.3/85.0:.5f})")

    # ── ROLL axis ─────────────────────────────────────────────────────────────
    # motor_sign=+1: roller motor is not inverted.
    # K_ROLL gains are NEGATIVE, so u = -(K@x) is POSITIVE when states are positive —
    # matching control2.py's v_roller = +(K21·roll+…)·N_ROLLER sign structure.
    # In controller, feed x_roll = [N*phi_motor, roll, N*phidot_motor, rolldot].
    A_r, B_r = build_model(gear_ratio=N_ROLLER, motor_sign=1.0)
    print_eigenvalues(A_r, "ROLL  (B scaled by N_ROLLER; K → motor torque)")
    K_r = compute_lqr(A_r, B_r, Q_ROLL, R_ROLL)[0]
    cl_eigs_r = np.linalg.eigvals(A_r - B_r @ K_r.reshape(1, -1))
    assert all(ev.real < 0 for ev in cl_eigs_r), \
        f"ROLL closed-loop NOT stable: {cl_eigs_r}"
    assert all(k < 0 for k in K_r), \
        f"ROLL K not all negative (sign issue?): {K_r}"
    Kr_phi, Kr_th, Kr_phidot, Kr_thdot = K_r

    print(f"\n  ROLL:  u_roll = -K_ROLL @ [N*phi_motor, roll, N*phidot_motor, rolldot]")
    print(f"    K_φ    = {Kr_phi:9.4f}  Nm/rad    (all gains NEGATIVE)")
    print(f"    K_θ    = {Kr_th:9.4f}  Nm/rad")
    print(f"    K_φ̇   = {Kr_phidot:9.4f}  Nm·s/rad")
    print(f"    K_θ̇   = {Kr_thdot:9.4f}  Nm·s/rad")
    print(f"  Gain ratios (use absolute values for comparison):")
    K21_eff = 70.0 * N_ROLLER
    print(f"    |K_φ/K_θ|   = {abs(Kr_phi/Kr_th):.5f}   (hand: {0.03/K21_eff:.5f})")
    print(f"    |K_θ̇/K_θ|  = {abs(Kr_thdot/Kr_th):.5f}   (hand: {2.0/K21_eff:.5f})")
    print(f"    |K_φ̇/K_θ|  = {abs(Kr_phidot/Kr_th):.5f}   (hand: {0.3/K21_eff:.5f})")

    print(f"\n{sep}")
    print("  Paste into control3.py  (K_PITCH / K_ROLL constants):")
    print(f"  K_PITCH = np.array([{Kp_phi:.6f}, {Kp_th:.6f},"
          f" {Kp_phidot:.6f}, {Kp_thdot:.6f}])")
    print(f"  K_ROLL  = np.array([{Kr_phi:.6f}, {Kr_th:.6f},"
          f" {Kr_phidot:.6f}, {Kr_thdot:.6f}])")
    print(sep)

import numpy as np
from SROmBURo.Nathan_Robust_Claude.config import (
    BODY_MASS, BODY_HEIGHT, BODY_LENGTH, I_BEND, I_SPIN,
    GRAVITY, WHEEL_RADIUS, ROLLER_EFF_RADIUS
)

class InverseDynamics:
    """
    Computed-torque (inverse dynamics) compensation for OmBURo.

    The control law from your sketch is:
        u = B̂(q) · τ_pid  +  n̂(q, q̇)  +  robustness_term

    B̂(q)  — effective inertia matrix (maps acceleration cmd → torque)
    n̂(q,q̇) — nonlinear terms: Coriolis, centripetal, gravity projection

    State convention: x = [θ_l, θ̇_l, θ_lat, θ̇_lat, ψ̇_l, ψ̇_lat]
      θ_l    = longitudinal tilt   (rad)
      θ_lat  = lateral tilt        (rad)
      ψ̇_l   = wheel angular vel   (rad/s)
      ψ̇_lat = roller angular vel  (rad/s)

    Jacobian note: OmBURo is a nonholonomic system — the Jacobian
    coupling wheel velocity to body velocity is geometry-dependent.
    The simplified 2×2 decoupled form below is valid near upright
    equilibrium (small angle). For large tilts you need the full
    coupled Jacobian from the RoMeLa paper §III.
    """

    def __init__(self):
        self.m  = BODY_MASS
        self.h  = BODY_HEIGHT
        self.I  = I_BEND       # dominant bending moment of inertia
        self.I_s = I_SPIN      # spin axis (small)
        self.g  = GRAVITY
        self.r  = WHEEL_RADIUS
        self.r_r = ROLLER_EFF_RADIUS

    def B_hat(self, theta_l: float, theta_lat: float) -> np.ndarray:
        """
        Effective 2×2 inertia matrix B̂(q) mapping [α_long, α_lat] → [τ_long, τ_lat].

        Derived from the Lagrangian of a point-mass inverted pendulum on a wheel:
            I_eff_long = I + m·h²·cos²(θ_lat)   (lateral tilt reduces longitudinal inertia)
            I_eff_lat  = I + m·h²·cos²(θ_l)

        Off-diagonal coupling terms are small near equilibrium and set to zero here.
        Include them when operating at tilts > ~15°.
        """
        cos_lat = np.cos(theta_lat)
        cos_l   = np.cos(theta_l)

        I_eff_long = self.I + self.m * self.h**2 * cos_lat**2
        I_eff_lat  = self.I + self.m * self.h**2 * cos_l**2

        # Scale to wheel torque: τ = I_eff · α / r
        b11 = I_eff_long / self.r
        b22 = I_eff_lat  / self.r_r

        return np.array([[b11, 0.0],
                         [0.0, b22]])

    def n_hat(self, theta_l: float, theta_lat: float,
              dtheta_l: float, dtheta_lat: float,
              dpsi_l: float, dpsi_lat: float) -> np.ndarray:
        """
        Nonlinear compensation vector n̂(q, q̇).

        Terms included:
          1. Gravity:    m·g·h·sin(θ) — the restoring/destabilising gravity torque
          2. Centripetal: m·h²·θ̇²·sin·cos — cross-axis coupling from tilt rate
          3. Gyroscopic: I_spin · ψ̇ · θ̇ — small for this tube but included

        Returns [n_long, n_lat] in Nm at the wheel output.
        """
        s_l   = np.sin(theta_l)
        s_lat = np.sin(theta_lat)
        c_lat = np.cos(theta_lat)
        c_l   = np.cos(theta_l)

        # Gravity term (acts to topple the pendulum — must be cancelled)
        n_grav_l   = self.m * self.g * self.h * s_l   / self.r
        n_grav_lat = self.m * self.g * self.h * s_lat / self.r_r

        # Centripetal coupling (θ̇_lat affects longitudinal channel and vice versa)
        n_cent_l   = -self.m * self.h**2 * dtheta_lat**2 * s_l   * c_lat / self.r
        n_cent_lat = -self.m * self.h**2 * dtheta_l**2  * s_lat  * c_l   / self.r_r

        # Gyroscopic coupling from wheel spin through I_spin
        # (small for a slender tube but non-zero at speed)
        n_gyro_l   = self.I_s * dpsi_lat * dtheta_lat / self.r
        n_gyro_lat = self.I_s * dpsi_l   * dtheta_l   / self.r_r

        n_long = n_grav_l   + n_cent_l   + n_gyro_l
        n_lat  = n_grav_lat + n_cent_lat + n_gyro_lat

        return np.array([n_long, n_lat])

    def compute(self, tau_pid: np.ndarray, state: np.ndarray) -> np.ndarray:
        """
        Full inverse dynamics torque (before robustness term):
            u_id = B̂(q) · τ_pid + n̂(q, q̇)

        tau_pid : 2-vector from PID [long, lat]
        state   : 6-vector [θ_l, θ̇_l, θ_lat, θ̇_lat, ψ̇_l, ψ̇_lat]
        Returns 2-vector torque [τ_long, τ_lat]
        """
        theta_l, dtheta_l, theta_lat, dtheta_lat, dpsi_l, dpsi_lat = state

        B = self.B_hat(theta_l, theta_lat)
        n = self.n_hat(theta_l, theta_lat, dtheta_l, dtheta_lat, dpsi_l, dpsi_lat)

        return B @ tau_pid + n
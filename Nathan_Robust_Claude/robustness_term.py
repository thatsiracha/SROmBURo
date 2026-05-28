import numpy as np
from SROmBURo.Nathan_Robust_Claude.config import RHO

class RobustnessTerm:
    """
    Discontinuous robustness injection from the arm paper:
        u_rob = −ρ · vers(Dᵀ · s)

    Where:
        s   = sliding surface = state error (or prediction error in our case)
        D   = distribution matrix (maps robustness onto actuator space)
        ρ   = robustness gain (scalar, tuned experimentally)
        vers(v) = v / ‖v‖  (unit vector operator — as labelled in your sketch)

    The vers() operator makes this a direction-only injection of fixed
    magnitude ρ — unlike a proportional term it doesn't grow with error,
    which keeps actuator commands bounded.  The sign discontinuity is
    smoothed with a boundary layer (δ) to reduce chattering on hardware.

    For OmBURo, D = B (input distribution = inertia-scaled identity),
    so Dᵀ·s simplifies to a projection of the tilt error onto torque space.
    """

    BOUNDARY_LAYER = 0.05   # rad — smooth discontinuity to reduce chattering

    def __init__(self):
        self.rho = RHO

    def vers(self, v: np.ndarray) -> np.ndarray:
        """Unit vector of v, with boundary-layer smoothing near zero."""
        norm = np.linalg.norm(v)
        if norm < self.BOUNDARY_LAYER:
            # Smooth sat() approximation inside the boundary layer
            return v / self.BOUNDARY_LAYER
        return v / norm

    def compute(self, state: np.ndarray, ref: np.ndarray) -> np.ndarray:
        """
        state : 6-vector current state
        ref   : 6-vector reference state
        Returns 2-vector robustness torque injection [long, lat]
        """
        # Sliding surface: velocity + tilt error (tunable weighting)
        err_tilt = np.array([state[0] - ref[0], state[2] - ref[2]])  # θ errors
        err_vel  = np.array([state[4] - ref[4], state[5] - ref[5]])  # ψ̇ errors
        s = err_vel + 2.0 * err_tilt   # combined sliding surface

        return -self.rho * self.vers(s)

    def set_rho(self, rho: float):
        """Allow live gain adjustment during tuning."""
        self.rho = max(0.0, rho)
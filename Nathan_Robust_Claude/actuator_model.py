import numpy as np
from SROmBURo.Nathan_Robust_Claude.config import MAX_TORQUE, MAX_TORQUE_RATE, DT

class ActuatorModel:
    """
    Two-stage actuator model applied per motor:
      1. Rate limiter  — prevents mechanical shock to the roller gears
      2. Saturation    — enforces servo torque limits

    The rate limiter is important for OmBURo because the flexible-shaft
    roller drive is sensitive to impulsive torque commands.
    """

    def __init__(self):
        self._prev = np.zeros(2)

    def apply(self, u_cmd: np.ndarray) -> np.ndarray:
        """Input: desired [τ_1, τ_2]. Output: physically realisable torques."""
        # Rate limiting
        delta = u_cmd - self._prev
        max_delta = MAX_TORQUE_RATE * DT
        delta_clipped = np.clip(delta, -max_delta, max_delta)
        u_rate_limited = self._prev + delta_clipped

        # Saturation
        u_sat = np.clip(u_rate_limited, -MAX_TORQUE, MAX_TORQUE)

        self._prev = u_sat.copy()
        return u_sat
import numpy as np
from config import PID_KP, PID_KI, PID_KD, INTEGRATOR_CLAMP, DT

class PIDController:
    """
    Outer-loop PID on velocity error.
    Output is a generalised acceleration command τ_pid (before
    inertia shaping) — not a torque yet.  B̂(q) scales it downstream.

    State tracked: [ψ̇_long_err, ψ̇_lat_err]
    """

    def __init__(self):
        self._integral = np.zeros(2)
        self._prev_err = np.zeros(2)

    def compute(self, ref: np.ndarray, state: np.ndarray) -> np.ndarray:
        """
        ref   : 6-vector reference state (we use indices [4,5] for wheel vels)
        state : 6-vector measured state
        Returns 2-vector PID output [long, lat]
        """
        err = ref[[4, 5]] - state[[4, 5]]

        # Anti-windup integrator
        self._integral = np.clip(
            self._integral + err * DT,
            -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP
        )

        derivative = (err - self._prev_err) / DT
        self._prev_err = err.copy()

        return PID_KP * err + PID_KI * self._integral + PID_KD * derivative

    def reset(self):
        self._integral  = np.zeros(2)
        self._prev_err  = np.zeros(2)
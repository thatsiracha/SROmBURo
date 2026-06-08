# SrOmBURo LQR Notes

## Parameters still to measure

| Symbol | Where used | Current value | How to measure |
|--------|-----------|---------------|----------------|
| `l` | CoM height above wheel axle | 0.65 m | Suspend robot by a string from a known point; measure CoM height from axle with CAD or plumb-line. **Most sensitive parameter.** |
| `I_b` | Body rotational inertia about axle | `m_b·l²/3` (rod approx) | **Bifilar pendulum** (suspend body from two parallel strings, time oscillation) or export from CAD. Rod approx can be 2–3× off. |
| `m_b` | Body mass | 8.0 kg | Weigh on scale. |
| `m_w` | Effective wheel + rotor mass | 1.0 kg | Weigh wheel assembly. |
| `I_w` | Wheel rotational inertia | `0.5·m_w·R²` (disk approx) | Disk approx is reasonable; replace with CAD value if available. |

**Priority order:** `l` → `I_b` → `m_b` → `m_w`/`I_w`.  
A 10 % error in `l` shifts all gain magnitudes; a factor-of-2 error in `I_b` shifts
the damping terms significantly.

---

## How to pick Q and R

Edit `Q_PITCH`, `Q_ROLL`, `R_PITCH`, `R_ROLL` in `compute_lqr.py`, then re-run:

```
python3 compute_lqr.py
```

The printed K arrays update automatically; `control3.py` imports them at startup.

### Meaning of Q weights `[φ, θ, φ̇, θ̇]`

| Weight | Effect of increasing |
|--------|----------------------|
| `Q[0,0]` φ | Stiffer position-hold; robot returns to balance point more aggressively |
| `Q[1,1]` θ | Stronger angle correction; harder to knock over; may oscillate if too large |
| `Q[2,2]` φ̇ | Penalises wheel speed; robot moves more slowly |
| `Q[3,3]` θ̇ | More angular damping; suppresses oscillations |

**Typical starting procedure:**
1. Keep `R=1`. Increase `Q[1,1]` (θ) until robot balances without wobble.
2. Increase `Q[0,0]` (φ) if it drifts laterally.
3. If motors saturate, increase R (or tighten `TAU_MAX` in `control3.py`).
4. If oscillatory, increase `Q[3,3]` or decrease `Q[1,1]`.

---

## Reading the gain-ratio sanity check (velocity mode)

When `CONTROL_MODE = "velocity"`, `control3.py` prints a block like:

```
[VEL-MODE gain ratios vs LQR optimal]
PITCH  K_φ/K_θ  hand=0.00035  LQR=0.00312
       K_θ̇/K_θ  hand=0.12941  LQR=0.09876
       K_φ̇/K_θ  hand=0.00353  LQR=0.00241
ROLL   K_φ/K_θ  hand=0.00011  LQR=0.00312
       K_θ̇/K_θ  hand=0.00714  LQR=0.09876
```

**Interpretation:**  
The ratios are dimensionless (they factor out the unit difference between velocity
and torque modes). LQR gives the *optimal* ratio for the given Q/R weights; the
hand-tuned ratio shows how your empirical gains compare.

- `K_θ̇/K_θ` (damping/angle): if hand value << LQR, you are under-damped → increase K12/K22.
- `K_φ/K_θ` (drift/angle): if hand value << LQR, wheel-position correction is too weak → increase K14/K24.
- `K_φ̇/K_θ` (wheel-vel/angle): usually small; large LQR value suggests encoder noise is masking a coupling.

The ratios will not match exactly — the velocity and torque dynamics are different — but
large discrepancies (>5×) indicate a gain is significantly out of the optimal region.

---

## Encoder position sign convention

`readback()` in `Omburo.py` negates `velocity_wheel` but not `position_wheel`.
To keep φ consistent with the velocity sign convention, `control3.py` applies:

```
phi_w = PHI_POS_SIGN_W × (pos_w − zero)    # PHI_POS_SIGN_W = −1.0
phi_r = N_ROLLER × PHI_POS_SIGN_R × (pos_r − zero)  # PHI_POS_SIGN_R = +1.0
```

**If the robot jolts or falls immediately on enable** in current mode, the position
sign is likely wrong. Flip the relevant `PHI_POS_SIGN_*` constant in `control3.py`
(top of file) and re-test.

---

## Motor direction sign convention (current mode)

The wheel motor (ID 2) BEAR encoder is mounted inverted: `readback()` negates
`velocity_wheel` to compensate. `compute_lqr.py` accounts for this via
`motor_sign=-1` in `build_model()` for the pitch axis.

Result — gain signs match control2.py's structure:

| Axis | K gains | `u = −K·x` when states > 0 | control2.py equivalent |
|------|---------|--------------------------|------------------------|
| PITCH | all **positive** | **negative** → wheel brakes/reverses | `v_wheel = −(K11·pitch + …)` |
| ROLL  | all **negative** | **positive** → roller drives | `v_roller = +(K21·roll + …)·N` |

No extra sign constants are needed in `control3.py`. If the robot jolts on enable,
check `PHI_POS_SIGN_W` first (position sign), then `motor_sign` in `compute_lqr.py`.

---

## Current mode vs velocity mode — what changes

| | Velocity mode | Current mode |
|-|--------------|-------------|
| Output | `setVelocity(v_wheel, v_roller − v_wheel)` | `setTorque(u_pitch, u_roll)` |
| Differential | Yes (subtracts wheel contribution to lateral) | No (each motor independent) |
| φ source | Integrated φ̇ (`_phi1_pos`) | Encoder position directly |
| Station-keeping | Angle integral | LQR K_φ position feedback |
| Saturation | `VEL_MAX = 15 rad/s` | `TAU_MAX = 2.1 Nm` |
| Soft-start | No | Yes (75-step ramp) |

To revert to velocity mode: set `CONTROL_MODE = "velocity"` in `control3.py`.

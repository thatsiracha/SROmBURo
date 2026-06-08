# Tuning guide: `mpc_control_s.py` + `OmburoForce.py`

## Why direct force mode (mode 3)

The balance controller uses **`OmburoForce.py`**, not `OmburoVel.py`. BEAR mode 3 closes a **force/torque loop** on the actuator’s onboard estimate instead of treating `goal_iq` as a raw current setpoint (mode 0).

| Layer | What it does |
|-------|----------------|
| **Outer loop** (`mpc_control_s.py`) | SMC or MPC computes desired axis torque [Nm] at 500 Hz |
| **Force loop** (BEAR mode 3, `p/i/d_gain_force`) | Tracks commanded torque using sensed output force |
| **Current loop** (`p/i/d_gain_iq`) | Limits phase current; same underlying layer as other modes |

Force mode is a better fit here because the outer controller outputs **torques**, and the actuator can correct for friction, temperature, and model error without you retuning `kt` on every run.

## File roles

| File | Role |
|------|------|
| `OmburoForce.py` | BEAR driver: mode 3, force PID, `setForce()` [Nm] |
| `OmburoVel.py` | Unchanged — velocity mode for `control2_Suraj.py` |
| `mpc_control_s.py` | IMU + SMC/MPC outer loop |

## Motor wiring (same as OmburoVel)

```python
setForce(force_wheel, force_roller)
# force_wheel  → id2 (big wheel / pitch)
# force_roller → id1 (roller / roll)
```

Values are **Newton-metres**, sent directly to `goal_iq` — **do not divide by `kt`** in force mode.

## Inner-loop tuning (`OmburoForce.py`)

```python
P_GAIN_FORCE = 1.0
I_GAIN_FORCE = 0.15
D_GAIN_FORCE = 0.0

P_GAIN_IQ = 0.277   # current limit layer (matches OmburoVel)
I_GAIN_IQ = 0.061
```

| Symptom | Adjust |
|---------|--------|
| Sluggish torque response | Raise `P_GAIN_FORCE` or `I_GAIN_FORCE` |
| Buzzing / vibration on motor | Lower `P_GAIN_FORCE`, add small `D_GAIN_FORCE` |
| Current spikes / saturation | Lower `I_TEST_A` in `mpc_control_s.py` or hardware `iq` limit |

Westwood’s `spring2damp.py` example uses force mode for spring/damper behaviour (`p_gain_force` as stiffness). Here we use force mode for **setpoint tracking**: outer loop sets the goal, inner force PID follows it.

## Outer-loop tuning (`mpc_control_s.py`)

### Physical model

```python
M_BODY = 5.0    # kg
L_COM  = 1.6    # m, axle → centre of mass
```

If the body is a uniform 1.6 m rod, use `L_COM = 0.8` and `I_BODY = M_BODY * L_COM**2 / 3`.

### SMC (default)

```python
SMC_LAMBDA   = 12.0   # surface pole [1/s]
SMC_K_SWITCH = 8.0    # switching gain [Nm]
SMC_PHI      = 0.08   # boundary layer
```

CLI: `python3 mpc_control_s.py --lambda 15 --k-switch 6`

### MPC

```python
MPC_HORIZON = 12
MPC_Q = np.diag([0.5, 200.0, 0.2, 4.0])   # [φ, θ, φ̇, θ̇]
MPC_R = np.array([[80.0]])
```

Run: `python3 mpc_control_s.py --mode mpc`

## Safety bring-up

1. Start with `I_TEST_A = 1.0` (software current cap ≈ 0.35 Nm).
2. Hold robot near vertical; run `python3 mpc_control_s.py`.
3. Wrong-sign jolt → flip `PITCH_EU_SIGN` or `ROLL_EU_SIGN`.
4. Raise `I_TEST_A` gradually after balance is stable.
5. Leave `SMC_USE_WHEEL_STATE = False` until IMU-only balance works.

## Verify force mode on hardware

After enabling, gently resist the wheel by hand:

- Motor should push back proportionally to your push (not run away).
- `readPresentTorque()` returns an estimate from measured current (`iq * kt`).

If the motor does nothing or behaves like a position spring, check that mode reads back as **3** and that torque enable is on.

## Debugging

| Symptom | Likely cause |
|---------|--------------|
| Jolt wrong way on enable | IMU sign |
| Weak balance | `I_TEST_A` too low or `L_COM` / `M_BODY` wrong |
| Chatter | `SMC_K_SWITCH` too high |
| Runaway wheel spin | Enable encoder feedback only after sign check |

# Debug log — control2_Suraj.py PD + roller sign fix

Date: 2026-06-07

## Summary

Converted `control2_Suraj.py` from PI to PD control, fixed roller motor mixing so roll correction drives the correct direction, and introduced `OmburoVel.py` as the velocity-mode motor driver (replacing direct `Omburo.py` import).

---

## 1. PI → PD (`control2_Suraj.py`)

### Removed
- `KI_PITCH`, `KI_ROLL` integral gains
- `INT_VEL_LIM` anti-windup limit
- `_pitch_int`, `_roll_int` state variables
- Angle integral accumulation in `_step()`
- Integral terms from `roll_cmd` and `pitch_cmd`
- Integral line from debug printout

### Retained (PD terms)
- **P**: `K11`, `K21` (angle)
- **D**: `K12`, `K22` (gyro rate)
- **φ̇ damping**: `K13`, `K23` (encoder velocity)
- **φ position**: `K14`, `K24` (encoder position integral — drift correction, not angle integral)

### Control law (after change)

```
roll_cmd  = K11·roll  + K12·rolldot  + K13·φ̇_roll  + K14·φ_roll_pos
pitch_cmd = −(K21·pitch + K22·pitchdot + K23·φ̇_pitch + K24·φ_pitch_pos) · N_ROLLER
```

---

## 2. Motor mixing fix (`control2_Suraj.py`)

### Hardware mapping (confirmed by user)
- **id2 big wheel** → pitch (longitudinal balance)
- **id1 roller motor** → roll via differential coupling with id2

### Problem (original)
- `motor1_cmd = pitch_cmd − roll_cmd` had roll sign inverted → rollers fought balance
- `roll_cmd_op = −roll_cmd` was computed but never used

### Problem (intermediate fix — reverted)
Assigning `motor2_cmd = roll_cmd_op` broke pitch: when pitch tilted, rollers spun
but the big wheel stayed still because pitch_cmd no longer reached id2.

### Final motor mapping (differential)

| Motor | Command | Pure pitch | Pure roll |
|-------|---------|------------|-----------|
| id2 (big wheel) | `pitch_cmd − roll_cmd` | `+pitch` (spins) | `−roll` (cancels) |
| id1 (roller motor) | `pitch_cmd + roll_cmd` | `+pitch` (spins) | `+roll` (opposite id2) |

```python
motor2_cmd = clip(pitch_cmd - roll_cmd)  # roll: spins opposite to id1 → big wheel cancels
motor1_cmd = clip(pitch_cmd + roll_cmd)  # roll: both motors spin, rollers move differentially
```

Roll requires **both** motors to spin in **opposite** directions so rollers spin but the big wheel net rotation is zero. Sending roll only on id1 (`motor2 = pitch_cmd`) left the big wheel dragging.

---

## 3. New file: `OmburoVel.py`

Created from `Omburo.py` as a velocity-dedicated driver.

### Changes from `Omburo.py`
| Item | `Omburo.py` | `OmburoVel.py` |
|------|-------------|----------------|
| Class name | `Omburo` | `OmburoVel` |
| Default mode | velocity (mode 1) | velocity (mode 1) — same |
| Duplicate PID setup lines | duplicated `set_p/i/d_gain_iq` twice | single setup block |
| Torque methods | `setTorque`, `setCurrentLimit` | removed (not needed for velocity loop) |
| Instance attrs | none | `self.id_wheel`, `self.id_roller` |
| Docstring | minimal | documents velocity API and motor id mapping |

### Unchanged from `Omburo.py`
- Port `/dev/ttyUSB0`, baud `8000000`
- Motor ids: wheel=2, roller=1
- `setVelocity(vel_wheel, vel_roller)` argument order
- `readback()` wheel velocity negation convention
- `toggleTorque`, `setVelocityMode`, `setTorqueMode`, `close`

### Import change in `control2_Suraj.py`
```python
# before
from Omburo import Omburo
self.robot = Omburo()

# after
from OmburoVel import OmburoVel
self.robot = OmburoVel()
```

---

## Files touched

| File | Action |
|------|--------|
| `control2_Suraj.py` | Modified — PD control, motor mixing fix, OmburoVel import |
| `OmburoVel.py` | Created — velocity-mode motor driver |
| `debug.md` | Created — this log |
| `Omburo.py` | Unchanged |

---

## Tuning notes

If the robot still falls on one axis after this fix:
1. **Roll axis**: flip `ROLL_EU_SIGN` or `ROLLDOT_SIGN` (±1.0)
2. **Pitch axis**: flip `PITCH_EU_SIGN` or `PITCHDOT_SIGN` (already −1.0 on pitch)
3. **Roller still wrong**: try `pitch_cmd − roll_cmd` on motor1 only (single sign flip)
4. Re-enable station-keeping later by adding a small `KI` term if drift is observed

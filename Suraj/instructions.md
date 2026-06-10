You are an expert robotics control engineer. We need to upgrade the inverted pendulum balancing controller in `control_T1New.py`. 

We are implementing three structural upgrades:
1. Fix the friction feedforward drive source.
2. Enable low-pass filtering on the noisy wheel encoders.
3. Implement a Cascaded Position Loop (Option A) to eliminate positional drift and automate angle calibration.

Please update the script carefully while preserving the 500 Hz deterministic loop timing, the terminal safety fall-stop, and the startup command ramp logic.

---

### STEP 1: Fix Friction Feedforward Source
Right now, `tau_ff_roll_f` and `tau_ff_pitch_f` use body angular rates (`rolldot_f`). Drivetrain friction opposes wheel movement, not body tilt.
* Modify the `tanh` calculations in `_step` to use the filtered wheel velocities (`vel_roll_f` and `vel_pitch_f`) instead of the gyro rates.

### STEP 2: Update Configuration Constants
In the `Config` class, update or add the following fields:
* Change `EMA_VEL` from `0.0` to `0.15` (to smooth out high-frequency encoder velocity noise).
* Add `OUTER_LOOP_DIVISOR = 10` (runs the outer loop at 50 Hz within the 500 Hz inner loop).
* Add Outer Loop Gains:
  * `KP_POSITION = 0.1`
  * `KD_POSITION = 0.05`
* Add an outer loop safety limit: `MAX_TARGET_ANGLE = 0.10` (clamps the commanded lean to ~5.7 degrees).

### STEP 3: Modify `PIDTorque` to Accept a Dynamic Target
* Update `PIDTorque.compute()` to accept an optional parameter: `target_angle: float = 0.0`.
* Change the error calculation from `error = -angle` to:
  ```python
  error = target_angle - angle


In OmBUROPIDController.__init__:

Initialize tracked encoder positions:

Python
self.pos_roll = 0.0
self.pos_pitch = 0.0
Initialize target positions (the position the robot will try to hold):

Python
self.target_pos_roll = 0.0
self.target_pos_pitch = 0.0
Initialize variables to store the commanded outer-loop angles passed to the inner loop:

Python
self.cmd_angle_roll = 0.0
self.cmd_angle_pitch = 0.0

STEP 5: Implement Outer Loop Control Logic in _step
Inside the _step(self, tick) function:

Accumulate Positions: Immediately after reading vel_roll_raw and vel_pitch_raw, update your cumulative position trackers using the loop time-step:
self.pos_roll += vel_roll_raw * cfg.CTRL_DT
self.pos_pitch += vel_pitch_raw * cfg.CTRL_DT

Execute 50 Hz Outer Loop: Wrap the outer loop computation in a decimation check:
if tick % cfg.OUTER_LOOP_DIVISOR == 0:
    # Calculate position errors
    err_pos_roll  = self.target_pos_roll - self.pos_roll
    err_pos_pitch = self.target_pos_pitch - self.pos_pitch

    # PD loop to generate target pitch/roll angles
    # Note: Using filtered velocity as the derivative term
    self.cmd_angle_roll  = (cfg.KP_POSITION * err_pos_roll)  - (cfg.KD_POSITION * vel_roll_f)
    self.cmd_angle_pitch = (cfg.KP_POSITION * err_pos_pitch) - (cfg.KD_POSITION * vel_pitch_f)

# Clamp outputs for safety
self.cmd_angle_roll  = max(-cfg.MAX_TARGET_ANGLE, min(self.cmd_angle_roll,  cfg.MAX_TARGET_ANGLE))
self.cmd_angle_pitch = max(-cfg.MAX_TARGET_ANGLE, min(self.cmd_angle_pitch, cfg.MAX_TARGET_ANGLE))

Pass Targets to Inner Loop: Update the inner loop calls to feed these targets into the controllers:
tau_fb_roll  = self.pid_roll.compute(roll_f,  rolldot_f,  vel_roll_f,  target_angle=self.cmd_angle_roll)
tau_fb_pitch = self.pid_pitch.compute(pitch_f, pitchdot_f, vel_pitch_f, target_angle=self.cmd_angle_pitch)


STEP 6: Update Debug Telemetry
Modify the 1 Hz debug print block at the bottom of _step so it prints the current cmd_angle values alongside the actual angles. This allows me to see the outer loop dynamically adjusting the balance target.

Please apply these changes directly to the script, making sure to keep the code neat and idiomatic.
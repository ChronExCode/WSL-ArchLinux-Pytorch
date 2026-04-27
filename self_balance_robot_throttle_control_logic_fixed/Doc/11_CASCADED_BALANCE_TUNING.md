# Cascaded balance tuning notes

This build changes the stabilizer to a standard two-loop structure:

```text
RC throttle -> target_speed_tps_
             -> target lean angle / constrained_pitch_deg
             -> inner pitch stabilizer
             -> ODrive wheel velocity command
```

## Sign convention

The inner stabilizer uses this convention:

```text
positive theta_error = measured_pitch - target_pitch
positive theta_error -> positive wheel velocity command
```

Therefore a positive forward speed error must create a **negative** target pitch:

```cpp
theta_from_speed = -speed_to_pitch_ff * (target_speed_tps - measured_velocity_tps);
```

If the robot is ahead of the hold position:

```cpp
position_error = measured_position - target_position; // positive when robot is too far forward
theta_from_position = +reference_position_gain * position_error;
```

A positive target pitch makes theta_error smaller/negative and commands reverse wheel motion. This is the corrected position-hold sign.

## Conservative bring-up parameters

These defaults are intentionally gentle:

```text
auto_lqr_enable = 0
nmpc_enabled = false
max_velocity = 2.0
target_speed_rate_limit = 3.0
target_pitch_rate_limit_dps = 80.0
reference_position_gain = 0.10
reference_velocity_damping_gain = 0.06
speed_to_pitch_ff = 1.0
accel_to_pitch_ff = 0.05
lqr_k_theta = 1.2
lqr_k_theta_d = 0.06
lqr_k_x = 0.0
lqr_k_v = 0.0
lqr_integral_k = 0.0
```

## Why the old behavior could drift or start strangely

1. `target_speed_tps_` was added directly to the wheel command. That hard-pushes the motors instead of asking the robot to lean first.
2. Position and velocity feedback were applied twice: once through target pitch and again directly in `compute_stabilizing_control()`.
3. The old `theta_from_speed = +speed_to_pitch_ff * target_speed_tps_` was opposite for the confirmed sign convention. Forward command could initially produce reverse wheel velocity.
4. `fb_x` as direct velocity feedback can behave like a slow positive/negative integrator depending on sign and direction calibration, causing creeping or low-frequency hunting.

## First tests

1. Keep wheels off the ground or use a stand. Confirm positive pitch error produces positive wheel command.
2. With throttle zero, push the robot forward. Target pitch should become positive and wheel command should tend reverse.
3. Apply small forward throttle. Target pitch should become negative first; wheel command should then go positive.
4. Only after direction is correct, increase `speed_to_pitch_ff` in small steps, e.g. `+0.1`.
5. If it oscillates around the hold point, reduce `reference_position_gain` or `reference_velocity_damping_gain` first.

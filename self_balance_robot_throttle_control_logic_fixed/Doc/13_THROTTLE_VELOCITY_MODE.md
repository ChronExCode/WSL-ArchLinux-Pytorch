# Throttle velocity mode fix

Control chain:

```text
throttle -> target_speed_tps_ -> target pitch -> balance inner loop -> wheel velocity
```

The controller no longer integrates `target_speed_tps_` into `target_displacement_turns_` during joystick driving. The old logic made throttle behave like position tracking, which caused creeping, low-frequency hunting, and unnatural start/stop pre-lean.

When throttle is inside deadband, `raw_speed_ref` is zero. `target_speed_tps_` ramps toward zero with `target_speed_rate_limit`. If the robot is still moving, speed error generates opposite target pitch and brakes while staying balanced.

Conservative bring-up parameters:

```text
max_velocity = 1.0 to 1.5
target_speed_rate_limit = 2.0 to 3.0
target_pitch_rate_limit_dps = 60 to 100
speed_to_pitch_ff = 1.0
accel_to_pitch_ff = 0.03 to 0.05
reference_velocity_damping_gain = 0.02 to 0.06
lqr_k_theta = 1.0 to 1.4
lqr_k_theta_d = 0.05 to 0.08
lqr_k_x = 0
lqr_k_v = 0
lqr_integral_k = 0 initially
```

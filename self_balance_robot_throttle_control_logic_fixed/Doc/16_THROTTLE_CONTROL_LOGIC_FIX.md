# 16 Throttle Control Logic Fix

This update fixes the regression where `ff_term`, `fb_x`, and `fb_v` were always zero.

## Problem

The previous cascaded rewrite moved speed and position feedback into `generate_reference()` only. That left `compute_stabilizing_control()` with pitch feedback only:

```text
base_cmd = fb_theta + fb_theta_d
ff_term = 0
fb_x = 0
fb_v = 0
```

The robot could balance, but throttle response could be weak or absent because the speed command only changed target pitch and had no direct velocity-tracking path.

## Fixed control chain

```text
throttle
  -> target_speed_tps_ with rate limit
  -> target_pitch pre-lean
  -> inner stabilizer

inner stabilizer:
  base_cmd = velocity_ff + pitch_feedback + speed_feedback + gated_position_feedback + optional_integral + friction_comp
```

## Sign convention

```cpp
x_error = actual_position - hold_position;
v_error = actual_speed - target_speed;
fb_x = -k_x * x_error;       // only when position hold is active
fb_v = -k_v * v_error;       // always active for speed tracking
ff_term = velocity_feedforward_gain * target_speed;
```

If the robot is ahead of its hold point, `x_error > 0`, so `fb_x < 0` pulls it back.

If throttle requests forward speed and the robot is still slow, `v_error < 0`, so `fb_v > 0` helps it accelerate.

## Conservative starting parameters

```text
velocity_feedforward_gain = 0.25
lqr_k_v = 0.28
lqr_k_x = 0.12
speed_to_pitch_ff = 1.0
accel_to_pitch_ff = 0.05
friction_comp_tps = 0.0 initially
```

Increase `lqr_k_v` if throttle response is too soft. Increase `velocity_feedforward_gain` slowly if the robot follows speed too slowly. Keep `lqr_k_x` small; it is only for zero-throttle hold.

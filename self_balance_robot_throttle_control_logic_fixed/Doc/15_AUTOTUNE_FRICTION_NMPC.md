# 15 — Auto-tune, Anti-windup, Friction Compensation, and NMPC

This revision adds three safety-oriented control improvements around the existing cascaded balance controller.

## 1. Conservative auto-tune

`auto_tune_enable` enables a slow online gain assistant. It is not an aggressive black-box test routine. It watches pitch tracking RMS and speed tracking RMS while the robot is upright, then gently scales the inner pitch stabilizer gains:

- `lqr_k_theta`
- `lqr_k_theta_d`

The scale is bounded by:

```text
auto_tune_min_scale <= auto_tune_scale <= auto_tune_max_scale
```

Default is disabled:

```text
auto_tune_enable = 0
auto_tune_rate = 0.015
auto_tune_min_scale = 0.65
auto_tune_max_scale = 1.60
auto_tune_pitch_error_target_deg = 2.0
auto_tune_speed_error_target_tps = 0.15
```

Bring-up recommendation:

1. First tune manually until the robot can stand without violent oscillation.
2. Enable `auto_tune_enable = 1` only while the wheels are off the ground or with a safety stand.
3. If it becomes nervous, reduce `auto_tune_max_scale` to `1.2`.
4. If it is sluggish but stable, increase `auto_tune_max_scale` gradually.

## 2. Anti-windup

The existing PID class already had integral anti-windup. This revision adds anti-windup to the active balance integrator:

- The pitch integral freezes when the previous motor command is saturated and the current pitch error would push farther into saturation.
- The integral decays when outside the safe pitch/rate/speed window.

This prevents the controller from accumulating a large hidden correction while the motors are already at their velocity limit.

Default:

```text
lqr_integral_k = 0.0
lqr_integral_limit = 3.0
```

Keep `lqr_integral_k = 0` at first. Only add a very small value if the robot has a repeatable steady lean bias that is not fixed by `pitch_offset` or adaptive trim.

## 3. Friction compensation

New parameters:

```text
friction_comp_tps = 0.0
friction_comp_deadband_tps = 0.03
friction_comp_fade_tps = 0.35
```

`friction_comp_tps` adds a small signed velocity feed-forward to overcome motor/wheel stiction. It fades out as measured wheel speed rises, so it mainly helps near zero speed.

Tuning:

1. Leave `friction_comp_tps = 0.0` until the robot can balance.
2. Increase by `0.01` at a time.
3. Stop increasing as soon as the robot starts responding cleanly near zero.
4. If the robot twitches or hunts at zero throttle, reduce it.

Typical safe range:

```text
friction_comp_tps = 0.02 .. 0.12
```

## 4. NMPC status

NMPC already existed in the codebase. It is still optional and disabled by default in `Config`:

```text
nmpc_enabled = false
```

When enabled, NMPC currently refines the target pitch/velocity reference before the inner stabilizer. It is not a full torque-level replacement for the pitch stabilizer. This is intentional for safety: the LQR-like inner loop remains the fallback if NMPC is stale, mismatched, or unavailable.

Use NMPC only after the non-NMPC controller is stable.

## Recommended safe starting values

```text
# Main bring-up
max_velocity = 2.0
target_speed_rate_limit = 3.0
target_pitch_rate_limit_dps = 80.0
speed_to_pitch_ff = 1.0
accel_to_pitch_ff = 0.05
reference_position_gain = 0.10
reference_velocity_damping_gain = 0.06

# Inner balance
lqr_k_theta = 1.2
lqr_k_theta_d = 0.06
lqr_integral_k = 0.0

# New features off at first
auto_tune_enable = 0
friction_comp_tps = 0.0
nmpc_enabled = 0
```

## Activation order

1. Balance with conservative LQR/manual gains.
2. Verify throttle velocity mode and zero-throttle position hold.
3. Add friction compensation only if there is near-zero deadband.
4. Enable auto-tune only after the robot is already stable.
5. Enable NMPC last, and keep stale-result fallback enabled.

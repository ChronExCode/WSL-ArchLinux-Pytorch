# Zero-throttle position hold

Joystick driving uses velocity following while throttle is active, then gated position hold after throttle is zero and the robot has slowed down.

```text
throttle -> target_speed_tps_ -> speed error -> target pitch
zero throttle + nearly stopped -> capture hold position
hold position error -> small target pitch correction
inner balance loop -> wheel velocity command
```

Sign convention:
- Positive `theta_error` commands positive wheel velocity.
- Forward speed error requests a negative target pitch.
- If the robot rolls forward past the hold point, `position_error_turns > 0`, so `theta_from_position > 0`; this makes `theta_error` negative and commands backward wheel velocity.

Safe starting values:
- `reference_position_gain = 0.05 .. 0.15`
- `reference_velocity_damping_gain = 0.04 .. 0.10`
- `stop_speed_threshold_tps = 0.10 .. 0.20`
- `stop_hold_capture_window_tps = 0.15 .. 0.30`
- `speed_to_pitch_ff = 0.5 .. 1.2`
- `target_pitch_rate_limit_dps = 60 .. 120`

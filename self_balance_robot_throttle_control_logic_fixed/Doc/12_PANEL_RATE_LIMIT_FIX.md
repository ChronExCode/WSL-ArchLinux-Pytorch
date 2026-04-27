# Panel crash and rate-limit parameter fix

## Fixed

- `speed_rate_limit` and `pitch_rate_limit` used `step = 0.0` in the controller panel, so D-pad left/right could not tune them.
- `speed_rate_limit` default is now `3.0` with step `0.1`.
- `pitch_rate_limit` default is now `80.0 deg/s` with step `5.0 deg/s`.
- Parameter group splitting now uses an explicit `(base group, chunk)` map. This avoids invalid group indices and makes group switching safe.
- Group switching now guards against empty groups and clamps the selected row after changing group.
- The renderer now draws from the actual current parameter vector size instead of recomputing the count separately.

## Conservative bring-up values

- `target_speed_rate_limit = 3.0`
- `target_pitch_rate_limit_dps = 80.0`

If the robot feels too sluggish after it can reliably stand, increase gradually:

- `target_speed_rate_limit`: `3.0 -> 4.0 -> 6.0`
- `target_pitch_rate_limit_dps`: `80 -> 100 -> 120`

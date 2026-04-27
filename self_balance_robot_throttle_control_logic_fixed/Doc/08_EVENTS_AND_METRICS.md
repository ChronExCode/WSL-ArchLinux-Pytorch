# Events and Derived Metrics

## Why event logging matters

For a control system, raw timeseries alone are not enough. You also need context:

- when the system armed
- when a fault occurred
- whether a USB disconnect happened
- whether a parameter update was applied before or after a bad run

Without this context, replay and tuning become guesswork.

## Logged event timeline

The robot now records lifecycle events directly into the log file. This makes each log self-describing.

Examples:

- `#EVENT t_s=... name=ARM message="Armed and balancing"`
- `#EVENT t_s=... name=FAULT message="Tilt exceeded safety limit"`

## Derived metrics from replay

`tools/replay_csv.py` now computes the following metrics per log:

- pitch range
- peak absolute pitch
- peak absolute pitch tracking error
- pitch RMS tracking error
- mean absolute encoder velocity
- static drift (difference between early and late average displacement)
- recovery time (first interval where tracking error remains below 2 degrees for 0.5 s)

These metrics are intended for:

- manual tuning comparison
- autotune cost-function design
- regression tracking after controller changes

## Recommended tuning process with metrics

1. Keep one baseline log.
2. Change exactly one parameter family.
3. Repeat the same experiment.
4. Compare logs with replay metrics.
5. Only keep the change if both subjective feel and objective metrics improve.

## Recommended next metrics

Future additions that would be valuable:

- integrated control effort
- command saturation time
- time spent outside safe tilt band
- event-aware segmentation (for example, measure only from ARM to DISARM)

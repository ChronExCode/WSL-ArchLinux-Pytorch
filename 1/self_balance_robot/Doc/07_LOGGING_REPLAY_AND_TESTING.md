# Logging, Replay, and Testing

## What is logged

Each run creates a CSV file under `logs/` with:

- one CSV header row for time-series data
- metadata lines beginning with `#`

Metadata currently includes:

- `#CONFIG_SNAPSHOT {json}` at startup
- `#CONFIG_UPDATE ... payload={json}` whenever parameters are updated remotely
- `#EVENT ...` for important lifecycle events

## Event classes

The robot logs the following event types:

- `STATE` — state-machine transitions
- `USB_ATTACH` / `USB_DETACH`
- `ARM`
- `DISARM`
- `ESTOP`
- `FAULT`

These event lines are intentionally easy to grep and also safe for replay tooling to skip.

## Replay workflow

Typical workflow:

1. run experiment A and B
2. collect `logs/run_a.csv` and `logs/run_b.csv`
3. compare:

```bash
python tools/replay_csv.py logs/run_a.csv logs/run_b.csv --plot
```

The replay tool prints summary metrics and can plot pitch, target pitch, velocity, and cross-log error comparison.

## Minimal testing

Current tests validate:

- mini JSON parsing basics
- config sanitization / validation basics
- logger writes snapshot/event/data lines

This is intentionally small, but it prevents regressions in the software plumbing that directly affects tuning and field debugging.

#!/usr/bin/env python3
"""Replay/inspection tool for robot CSV logs.
Supports one or more CSV files. Metadata lines beginning with '#' are parsed as
parameter snapshots / events and ignored by the CSV reader.
Also extracts key metrics useful for tuning and regression checking.
Usage:
  python tools/replay_csv.py logs/run_*.csv --plot
"""
import argparse, csv, math, statistics
from pathlib import Path


def split_metadata_and_rows(path):
    metadata = []
    rows = []
    with open(path, newline='') as f:
        filtered = []
        for line in f:
            if line.startswith('#'):
                metadata.append(line.strip())
            elif line.strip():
                filtered.append(line)
    if filtered:
        rows = list(csv.DictReader(filtered))
    return metadata, rows


def f(row, key):
    try:
        return float(row.get(key, 0.0) or 0.0)
    except ValueError:
        return 0.0


def compute_metrics(rows):
    if not rows:
        return {}
    pitch = [f(r, 'pitch') for r in rows]
    tgt = [f(r, 'tgt_pitch') for r in rows]
    vel = [f(r, 'enc_vel') for r in rows]
    t = [f(r, 't_s') for r in rows]
    abs_err = [abs(a-b) for a, b in zip(pitch, tgt)]
    sq_err = [(a-b)**2 for a, b in zip(pitch, tgt)]
    peak_abs_pitch = max(abs(p) for p in pitch) if pitch else 0.0
    peak_abs_err = max(abs_err) if abs_err else 0.0
    rms = math.sqrt(statistics.mean(sq_err)) if sq_err else 0.0
    mean_abs_vel = statistics.mean(abs(v) for v in vel) if vel else 0.0
    # static drift: difference between mean of first/last 10% position samples.
    disp = [f(r, 'enc_disp') for r in rows]
    n = max(1, len(disp)//10)
    static_drift = statistics.mean(disp[-n:]) - statistics.mean(disp[:n]) if disp else 0.0
    # recovery time: first time after start when error stays below 2 deg for 0.5 s.
    recovery_time = None
    if len(rows) > 5:
        window_s = 0.5
        for i in range(len(rows)):
            t0 = t[i]
            j = i
            ok = True
            while j < len(rows) and t[j] - t0 <= window_s:
                if abs_err[j] > 2.0:
                    ok = False
                    break
                j += 1
            if ok and j < len(rows):
                recovery_time = t0 - t[0]
                break
    return {
        'pitch_min': min(pitch),
        'pitch_max': max(pitch),
        'peak_abs_pitch': peak_abs_pitch,
        'peak_abs_error': peak_abs_err,
        'pitch_rms': rms,
        'vel_abs_mean': mean_abs_vel,
        'static_drift': static_drift,
        'recovery_time': recovery_time,
    }


def summarize(path):
    metadata, rows = split_metadata_and_rows(path)
    events = [m for m in metadata if m.startswith('#EVENT')]
    out = {'name': Path(path).name, 'rows': len(rows), 'metadata': metadata, 'events': events, 'rows_data': rows}
    out.update(compute_metrics(rows))
    return out


def print_summary(summary):
    print(f"== {summary['name']} ==")
    print(f"Rows: {summary['rows']}")
    print(f"Metadata lines: {len(summary['metadata'])}")
    print(f"Events: {len(summary['events'])}")
    for line in summary['metadata'][:3]:
        print(f"  {line[:140]}")
    if summary['rows']:
        print(f"Pitch range: {summary['pitch_min']:.3f} .. {summary['pitch_max']:.3f}")
        print(f"Peak |pitch|: {summary['peak_abs_pitch']:.3f}")
        print(f"Peak |pitch-target|: {summary['peak_abs_error']:.3f}")
        print(f"Pitch RMS error: {summary['pitch_rms']:.3f}")
        print(f"Velocity mean abs: {summary['vel_abs_mean']:.3f}")
        print(f"Static drift: {summary['static_drift']:.4f}")
        rt = summary['recovery_time']
        print(f"Recovery time: {rt:.3f}s" if rt is not None else 'Recovery time: n/a')


def plot_logs(summaries):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print('matplotlib unavailable:', exc)
        return
    for s in summaries:
        rows = s.get('rows_data', [])
        if not rows:
            continue
        t0 = f(rows[0], 't_s')
        t = [f(r, 't_s') - t0 for r in rows]
        plt.figure(figsize=(12, 6))
        plt.plot(t, [f(r, 'pitch') for r in rows], label='pitch')
        plt.plot(t, [f(r, 'tgt_pitch') for r in rows], label='target_pitch')
        plt.plot(t, [f(r, 'enc_vel') for r in rows], label='enc_vel')
        plt.legend(); plt.grid(True); plt.xlabel('s'); plt.title(s['name'])
    if len(summaries) > 1:
        plt.figure(figsize=(12, 6))
        for s in summaries:
            rows = s.get('rows_data', [])
            if not rows:
                continue
            t0 = f(rows[0], 't_s')
            t = [f(r, 't_s') - t0 for r in rows]
            err_series = [abs(f(r, 'pitch') - f(r, 'tgt_pitch')) for r in rows]
            plt.plot(t, err_series, label=f"|pitch-target| {s['name']}")
        plt.legend(); plt.grid(True); plt.xlabel('s'); plt.title('Log comparison')
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_paths', nargs='+')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()
    summaries = [summarize(p) for p in args.csv_paths]
    for s in summaries:
        print_summary(s)
    if args.plot:
        plot_logs(summaries)

if __name__ == '__main__':
    main()

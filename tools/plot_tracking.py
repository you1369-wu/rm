"""Plot recorded page-0 telemetry and report errors for each uninterrupted RUNNING segment."""
import argparse
import csv
import json
import math
from pathlib import Path

def metrics(rows):
    if not rows:
        return {'samples_100Hz': 0}
    result = {'samples_100Hz': len(rows)}
    for axis in ('yaw', 'pitch'):
        errors = [r[f'{axis}_ref']-r[f'{axis}_actual'] for r in rows]
        result[axis+'_rms_rad'] = math.sqrt(sum(e*e for e in errors)/len(errors))
        result[axis+'_peak_rad'] = max(map(abs, errors))
        result[axis+'_mean_rad'] = sum(errors)/len(errors)
    result['saturation_fraction_100Hz'] = sum(r['saturation'] != 0 for r in rows)/len(rows)
    result['b_peak_hold_error_rad'] = max(abs(r['b_error']) for r in rows)
    result['firmware_rms_yaw_rad_at_last_frame'] = rows[-1]['rms_yaw']
    result['firmware_rms_pitch_rad_at_last_frame'] = rows[-1]['rms_pitch']
    return result

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', type=Path)
    ap.add_argument('--output', type=Path, default=Path('tracking_report'))
    args = ap.parse_args()
    with args.csv.open(encoding='utf-8-sig', newline='') as f:
        rows = [{k: float(v) if v else math.nan for k, v in r.items()} for r in csv.DictReader(f)]
    required = ['yaw_ref','yaw_actual','pitch_ref','pitch_actual','state','fault','saturation','b_error']
    if not rows or any(k not in rows[0] for k in required):
        ap.error('Input must be a non-empty PAGE 0 capture from this firmware.')
    if not all(math.isfinite(r[k]) for r in rows for k in required):
        ap.error('Non-finite telemetry found; inspect the capture before evaluation.')
    runs, current = [], []
    for r in rows:
        if int(r['state']) == 5:
            current.append(r)
        elif current:
            runs.append(current); current = []
    if current:
        runs.append(current)
    report = {'note': 'Measured capture only. Host metrics use decimated samples; MCU RMS uses control samples. No stability guarantee.',
              'running_segments': [metrics(run) for run in runs],
              'fault_codes_seen': sorted({int(r['fault']) for r in rows if r['fault']})}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'metrics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        ap.exit(1, 'Metrics saved. Plotting requires: python -m pip install matplotlib\n')
    timestamps = [r.get('host_s', math.nan) for r in rows]
    if not all(math.isfinite(t) for t in timestamps):
        x = [i*.01 for i in range(len(rows))]
        xlabel = 'Nominal time (s), assumes 100 Hz without frame loss'
    else:
        x = [t-timestamps[0] for t in timestamps]
        xlabel = 'Host receive time (s), includes buffering jitter'
    fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    for i, axis in enumerate(('yaw','pitch')):
        ax[i].plot(x,[r[axis+'_ref'] for r in rows],label='target')
        ax[i].plot(x,[r[axis+'_actual'] for r in rows],label='actual',alpha=.8)
        ax[i].set_ylabel(axis+' (rad)'); ax[i].legend()
    for axis in ('yaw','pitch'):
        ax[2].plot(x,[r[axis+'_ref']-r[axis+'_actual'] for r in rows],label=axis)
    ax[2].plot(x,[r['b_error'] for r in rows],label='B hold')
    ax[2].set_ylabel('Error (rad)'); ax[2].legend()
    for axis in ('a','b','c'):
        ax[3].plot(x,[r['tau_'+axis] for r in rows],label=axis.upper())
    ax[3].set_ylabel('Command torque (N m)'); ax[3].legend()
    ax[4].step(x,[r['state'] for r in rows],where='post',label='state')
    ax[4].step(x,[r['fault'] for r in rows],where='post',label='fault')
    ax[4].set_ylabel('State / fault'); ax[4].set_xlabel(xlabel); ax[4].legend()
    for a in ax:
        a.grid(True,alpha=.3)
    fig.suptitle('Gimbal tracking: recorded hardware telemetry')
    fig.tight_layout(); fig.savefig(args.output/'tracking.png',dpi=160)
    plt.close(fig)

if __name__ == '__main__':
    main()

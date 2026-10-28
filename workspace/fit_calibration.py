#!/usr/bin/env python3
"""Fit 6D sim<->policy affine calibrations from *measured equivalent poses*.

Input pairs must be aligned to the same physical configurations, not merely similar
numbers or independent min/max observations from different mechanisms.

This tool NEVER authorizes executing a policy; calibration_confirmed remains false.
"""
import argparse
import json
import math
from pathlib import Path

NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
]


def vector(value, label):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{label}: expected list[6]")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"{label}: every entry must be finite number")
    return [float(x) for x in value]


def pairs(records, key):
    if len(records) < (3 if key == 'fit' else 2):
        raise ValueError(f"{key}: need >= {3 if key == 'fit' else 2} matched poses")
    result = []
    for i, record in enumerate(records):
        result.append((vector(record['sim'], f'{key}[{i}].sim'),
                       vector(record['policy'], f'{key}[{i}].policy')))
    return result


def fit_one(data, j):
    xs = [x[j] for x, _ in data]
    ys = [y[j] for _, y in data]
    xm = sum(xs) / len(xs)
    ym = sum(ys) / len(ys)
    den = sum((x - xm) ** 2 for x in xs)
    if den < 1e-12:
        raise ValueError(f'{NAMES[j]}: the simulated joint values do not vary enough')
    slope = sum((x - xm) * (y - ym) for x, y in zip(xs, ys)) / den
    if abs(slope) < 1e-9:
        raise ValueError(f'{NAMES[j]}: fitted slope is zero; check measurements')
    return slope, ym - slope * xm


def residuals(data, j, s, b):
    err = [abs(y[j] - (s * x[j] + b)) for x, y in data]
    return max(err), (sum(e**2 for e in err) / len(err))**0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements', type=Path)
    parser.add_argument('--output', type=Path, default=Path('/outputs/calibration.json'))
    args = parser.parse_args()

    source = json.loads(args.measurements.read_text(encoding='utf-8'))
    if source.get('joint_names') != NAMES:
        raise ValueError(f'joint_names must be {NAMES} (in this order)')
    training = pairs(source['fit'], 'fit')
    checking = pairs(source['validate'], 'validate')
    scales, biases = [], []
    report = []

    for i, name in enumerate(NAMES):
        s, b = fit_one(training, i)
        max_train, _ = residuals(training, i, s, b)
        max_val, rmse_val = residuals(checking, i, s, b)
        policy_vals = [p[i] for _, p in training + checking]
        span = max(policy_vals) - min(policy_vals)
        if span < 1e-8:
            raise ValueError(f'{name}: insufficient policy-side range')
        frac = max_val / span
        scales.append(s)
        biases.append(b)
        report.append(dict(joint=name, max_fit_error=max_train,
                           max_validation_error=max_val,
                           validation_rmse=rmse_val,
                           max_validation_fraction_of_span=frac,
                           heuristic_warning=frac > 0.05))

    result = {
        'calibration_confirmed': False,
        '_warning': 'An affine fit does not prove kinematic, camera or grasp alignment. Never set true until manually verified.',
        'joint_names': NAMES,
        'state_to_policy': {'scale': scales, 'bias': biases},
        'action_to_sim': {
            'scale': [1.0 / s for s in scales],
            'bias': [-b / s for s, b in zip(scales, biases)]
        },
        '_validation': report,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Wrote:', args.output)
    for item in report:
        print(f"{item['joint']:15s} validation max error={item['max_validation_error']:.5f}"
              f" ({item['max_validation_fraction_of_span']*100:.2f}% of measured span)"
              + ('  CHECK' if item['heuristic_warning'] else ''))
    print('calibration_confirmed is intentionally FALSE. Also verify camera/geometry/gripper.')


if __name__ == '__main__':
    main()

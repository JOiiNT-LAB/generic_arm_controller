#!/usr/bin/env python3
"""
Score hand-eye calibrations on a capture and say whether they are good enough.

The marker is fixed, so a good calibration keeps its position in the base frame constant
while the arm moves. Record a capture with capture_hand_eye, then compare calibrations on it
without installing them.

Usage:
    # the calibration in use for a robot
    ros2 run generic_arm_controller evaluate_hand_eye --capture CAP.json --robot fr3_real
    # in use vs a new result, plus the best one the data can support (ready for install_hand_eye)
    ros2 run generic_arm_controller evaluate_hand_eye --capture CAP.json --robot fr3_real \
        new_result.yaml --fit --write-fit best.yaml

Exit code is 0 if the first calibration listed (the one in use when --robot is given) keeps
the marker within --max-spread-mm on every axis, 1 otherwise: usable as an acceptance test.
"""
import argparse
import os
import sys

import yaml

from generic_arm_controller.hand_eye import calibration_io as cio
from generic_arm_controller.hand_eye import evaluate as ev

DEFAULT_MAX_SPREAD_MM = 5.0   # marker excursion allowed over a ~250 mm arm motion


def _candidates(args):
    found = []
    if args.robot:
        found.append(('in use', cio.load_calibration(
            cio.calibration_file(args.robot, args.calibration_dir))))
    for path in args.candidates:
        found.append((os.path.basename(path), cio.load_calibration(path)))
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--capture', required=True, help='JSON written by capture_hand_eye')
    parser.add_argument('candidates', nargs='*', help='calibration YAML files to score')
    parser.add_argument('--robot', help='also score the calibration in use for this profile')
    parser.add_argument('--calibration-dir', default=None,
                        help='where hand_eye_transform_<robot>.yaml lives (default: installed)')
    parser.add_argument('--fit', action='store_true',
                        help='also estimate the best calibration from the data')
    parser.add_argument('--write-fit', metavar='FILE', help='save the --fit result as a YAML')
    parser.add_argument('--max-spread-mm', type=float, default=DEFAULT_MAX_SPREAD_MM)
    args = parser.parse_args(argv)

    try:
        candidates = _candidates(args)
        if not candidates:
            print('ERROR: give at least one calibration file or --robot')
            return 1
        g, m, offset = ev.load_capture(args.capture)
    except (cio.CalibrationError, OSError, KeyError, ValueError) as ex:
        print(f'ERROR: {ex}')
        return 1

    flange = g[:, :3, 3]
    motion = (flange.max(axis=0) - flange.min(axis=0)) * 1000.0
    print(f'samples: {len(g)}')
    print('flange motion: X {:.0f}  Y {:.0f}  Z {:.0f} mm\n'.format(*motion))

    print('{:<26} {:>8} {:>8} {:>8} {:>9}'.format('calibration', 'X mm', 'Y mm', 'Z mm', 'RMS mm'))
    print('-' * 62)
    results = []
    for name, cal in candidates:
        spread, rms = ev.evaluate(g, m, offset, cal)
        results.append((name, cal, spread, rms))
        print('{:<26} {:8.1f} {:8.1f} {:8.1f} {:9.1f}'.format(name[:26], *spread, rms))
    best = min(results, key=lambda r: r[3])
    print(f'\nbest of the list: {best[0]} (RMS {best[3]:.1f} mm)')

    if args.fit:
        estimate = ev.fit(g, m, offset, best[1])
        spread = estimate['spread_mm']
        print('\nbest estimate from the data: X {:.1f}  Y {:.1f}  Z {:.1f} mm   RMS {:.1f} mm'
              .format(*spread, estimate['rms_mm']))
        for v in estimate['validation']:
            print('   validation {:<12} train RMS {:.1f} mm   test RMS {:.1f} mm'.format(
                v['split'], v['train_rms_mm'], v['test_rms_mm']))
        print('\n   as a calibration file:')
        text = yaml.safe_dump(estimate['calibration'], default_flow_style=False)
        print('\n'.join('   ' + line for line in text.splitlines()))
        if args.write_fit:
            cio.save_calibration(args.write_fit, cio.normalized(estimate['calibration']))
            print(f'\n   written to {args.write_fit} (install it with install_hand_eye --from)')

    judged_name, _, judged_spread, _ = results[0]
    worst = float(max(judged_spread))
    ok = worst <= args.max_spread_mm
    verdict = 'PASS' if ok else 'FAIL'
    print(f"\n{verdict}: '{judged_name}' moves the marker by up to {worst:.1f} mm "
          f"(limit {args.max_spread_mm:.1f} mm)")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

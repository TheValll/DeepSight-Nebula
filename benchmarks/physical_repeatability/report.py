#!/usr/bin/env python3
"""Offline repeatability report from a session's committed journal (no ROS, no cameras).

Only visits with measurement_valid (motion succeeded, >= required valid distinct
frames, joint guards satisfied) contribute. Partial medians are never used.
Deviation statistics need >= 2 valid repetitions of a target; a one-cycle run is
a functional test, not a repeatability result. Raw data is never modified; the
report goes to a new directory <session>/reports/<utc>/.
"""
import argparse
import collections
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np

import geometry
import storage


def load(session):
    path = Path(session) / storage.DB_NAME
    if not path.is_file():
        raise FileNotFoundError(path)
    connection = sqlite3.connect(path, timeout=30)
    try:
        visits = [json.loads(r[0]) for r in connection.execute('SELECT data FROM visits ORDER BY cycle, target_id')]
        cycles = [dict(cycle=r[0], seconds=r[1], completed_visits=r[2])
                  for r in connection.execute('SELECT cycle, seconds, completed_visits FROM cycles ORDER BY cycle')]
        frames = collections.Counter()
        for camera, category, valid in connection.execute(
                "SELECT json_extract(data,'$.camera'), json_extract(data,'$.failure_category'), "
                "json_extract(data,'$.valid') FROM frames"):
            frames[(camera, category or ('valid' if valid else 'unknown'))] += 1
        meta = {r[0]: json.loads(r[1]) for r in connection.execute('SELECT key, data FROM meta')}
        meta['report_ball_frames'] = [json.loads(r[0]) for r in connection.execute(
            "SELECT data FROM frames WHERE json_extract(data,'$.ball_status') IS NOT NULL")]
    finally:
        connection.close()
    return visits, cycles, frames, meta


def _timing(values):
    values = [v for v in values if isinstance(v, (int, float))]
    if not values:
        return None
    return {'n': len(values), 'mean_s': float(np.mean(values)), 'median_s': float(np.median(values)),
            'min_s': float(np.min(values)), 'max_s': float(np.max(values))}


def analyse(visits, cycles, frames, config=None, *, ball_frames=None, tcp_calibration=None):
    targets = sorted({v['target_id'] for v in visits} | set((config or {}).get('target_ids', [])))
    per_target = {}
    for target in targets:
        rows = [v for v in visits if v['target_id'] == target]
        valid = [v for v in rows if v.get('measurement_valid') and v.get('motion_success')
                 and v.get('marker_xyz_mm') is not None]
        entry = {'visits': len(rows), 'valid_repetitions': len(valid),
                 'status': dict(collections.Counter(v.get('status') for v in rows)),
                 'failures': dict(collections.Counter(v.get('failure_category') for v in rows
                                                      if v.get('failure_category'))),
                 'marker8': geometry.repeatability_stats([v['marker_xyz_mm'] for v in valid])}
        tcp = [v['tcp_xyz_mm'] for v in valid if v.get('tcp_xyz_mm') is not None]
        entry['tcp'] = geometry.repeatability_stats(tcp) if tcp else {'n': 0, 'note': 'no calibrated TCP values'}
        if len(valid) < 2:
            entry['note'] = 'fewer than 2 valid repetitions: no deviation statistics'
        per_target[str(target)] = entry
    seconds = [c['seconds'] for c in cycles]
    timings = {'cycle': _timing(seconds),
               **{name: _timing([v.get(name) for v in visits]) for name in ('visit_s', 'motion_s', 'capture_s', 'home_s')}}
    configured = (config or {}).get('cycles')
    if seconds and configured:
        timings['estimated_total_for_configured_cycles_min'] = float(np.mean(seconds)) * configured / 60
    crosscheck = [v['ball_crosscheck_median_mm'] for v in visits if v.get('ball_crosscheck_median_mm') is not None]
    min_valid = min((e['valid_repetitions'] for e in per_target.values()), default=0)
    warnings = []
    if len(cycles) < 2 or min_valid < 2:
        warnings.append('FUNCTIONAL TEST ONLY: fewer than 2 valid repetitions for at least one target; '
                        'this is not a repeatability conclusion.')
    if any(v.get('status') in ('incomplete',) or v.get('status') not in storage.TERMINAL for v in visits):
        warnings.append('Session contains incomplete visits (interrupted run); see status counts.')
    if not any(v.get('tcp_xyz_mm') for v in visits):
        warnings.append('No calibrated TCP values: marker-8 values only.')
    elif not (tcp_calibration or {}).get('multi_pose_verified', False):
        warnings.append('PROVISIONAL TCP: multi-pose calibration is unverified or unavailable; '
                        'derived TCP values are not independently validated across the workspace.')
    warnings.append('Marker 8 is upstream of wrist roll: downstream roll backlash or gripper motion '
                    'is not independently measured by marker-derived TCP positions.')
    checks = ball_frames or []
    errors = [float(f['ball_error_mm']) for f in checks if f.get('ball_error_mm') is not None]
    ball_details = {'attempted_frames': len(checks), 'frames_with_error': len(errors),
                    'frames_without_error': len(checks) - len(errors),
                    'checked_target_ids': sorted({f['target_id'] for f in checks
                                                 if f.get('ball_error_mm') is not None}),
                    'median_mm': float(np.median(errors)) if errors else None,
                    'p95_mm': float(np.percentile(errors, 95)) if errors else None,
                    'max_mm': max(errors) if errors else None,
                    'diagnostic_threshold_mm': 8.0,
                    'frames_over_8mm': sum(e > 8.0 for e in errors)}
    if errors and ball_details['frames_over_8mm']:
        warnings.append(f"Ball cross-check: {ball_details['frames_over_8mm']}/{len(errors)} frames exceed "
                        f"8 mm (maximum {max(errors):.1f} mm); cause not established.")
    warnings.append('TCP is a translation-only operational point (held-ball centre) valid only at the '
                    'calibrated wrist roll and gripper opening; TCP orientation is not reported. '
                    'ISO 9283-inspired method; no compliance claim.')
    return {'generated_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'cycles_recorded': len(cycles), 'visits_recorded': len(visits),
            'visit_status': dict(collections.Counter(v.get('status') for v in visits)),
            'per_target': per_target, 'timings': timings,
            'frame_outcomes': {f'{camera}:{category}': n for (camera, category), n in sorted(frames.items(), key=str)},
            'ball_crosscheck': {'visits_with_value': len(crosscheck),
                                'median_of_visit_medians_mm': float(np.median(crosscheck)) if crosscheck else None,
                                **ball_details},
            'warnings': warnings}


def markdown(result):
    lines = ['# Physical repeatability report', '', f'Generated {result["generated_utc"]}.', '']
    lines += [f'> {w}' for w in result['warnings']] + ['']
    lines += [f'Cycles recorded: {result["cycles_recorded"]}; visits: {result["visits_recorded"]}; '
              f'status: {result["visit_status"]}', '']
    lines += ['| Target | valid n | TCP mean xyz (mm) | TCP std xyz (mm) | TCP mean/max/p95 dev (mm) | '
              'marker-8 mean/max/p95 dev (mm) | failures |', '|---|---|---|---|---|---|---|']
    def fmt(values):
        return '—' if values is None else ', '.join(f'{v:.3f}' for v in values)
    for target, entry in result['per_target'].items():
        tcp, marker = entry['tcp'], entry['marker8']
        dev = lambda s: '—' if 'mean_deviation_mm' not in s else (
            f'{s["mean_deviation_mm"]:.3f} / {s["max_deviation_mm"]:.3f} / {s["p95_deviation_mm"]:.3f}')
        lines.append(f'| {target} | {entry["valid_repetitions"]} | {fmt(tcp.get("mean_xyz_mm"))} | '
                     f'{fmt(tcp.get("std_xyz_mm"))} | {dev(tcp)} | {dev(marker)} | {entry["failures"] or "—"} |')
    lines += ['', '## Timings', '', '```json', json.dumps(result['timings'], indent=2), '```',
              '', '## Frame outcomes', '', '```json', json.dumps(result['frame_outcomes'], indent=2), '```',
              '', '## Ball cross-check', '', '```json', json.dumps(result['ball_crosscheck'], indent=2), '```', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(prog='run.sh report', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session', type=Path)
    args = parser.parse_args(argv)
    try:
        visits, cycles, frames, meta = load(args.session)
    except (OSError, sqlite3.Error) as error:
        print(f'cannot read session (run recover first?): {error}', file=sys.stderr)
        return 1
    config_path = args.session / 'snapshot' / 'effective_config.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else None
    calibration = None
    calibration_copy = meta.get('manifest', {}).get('files', {}).get('tcp_calibration', {}).get('copy')
    if calibration_copy:
        calibration = json.loads((args.session / calibration_copy).read_text())
    result = analyse(visits, cycles, frames, config,
                     ball_frames=meta.get('report_ball_frames'), tcp_calibration=calibration)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output = args.session / 'reports' / stamp
    output.mkdir(parents=True, exist_ok=False)
    storage.atomic_json(output / 'report.json', result)
    storage.atomic_bytes(output / 'report.md', markdown(result).encode())
    print(markdown(result))
    print(f'Report written to {output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Offline recovery/export of a benchmark session. Never imports ROS, never opens a
camera, never moves or resumes the robot.

Verifies the SQLite journal, imports complete frame sidecars missing from the DB,
reports partial/orphan files, marks unfinished visits 'incomplete' and rebuilds
measurements.csv, cycles.csv, frames.jsonl, frames.csv and events.jsonl.
After recovery start a NEW session if more data is needed; sessions are never resumed.
"""
import argparse
import json
from pathlib import Path
import sys

import storage


def main(argv=None):
    parser = argparse.ArgumentParser(prog='run.sh recover', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session', type=Path, help='session directory containing journal.sqlite3')
    parser.add_argument('--force-unlock', action='store_true',
                        help='recover even if the lock PID is alive (only if that PID is unrelated)')
    args = parser.parse_args(argv)
    try:
        report = storage.recover(args.session, force_unlock=args.force_unlock)
    except (OSError, RuntimeError) as error:
        print(f'recovery failed: {error}', file=sys.stderr)
        return 1
    summary = {k: (len(v) if isinstance(v, list) else v) for k, v in report.items()}
    print(json.dumps(summary, indent=2, default=str))
    if report['partial_files'] or report['rejected_sidecars']:
        print('Partial or rejected files were left untouched and are listed in the recovery JSON.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

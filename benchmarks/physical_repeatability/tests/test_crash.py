"""SIGKILL a real recorder process and recover offline."""
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import textwrap
import unittest

from tests import helpers
import storage

HERE = Path(__file__).resolve().parents[1]

CHILD = textwrap.dedent('''
    import sys, time
    sys.path.insert(0, sys.argv[2])
    for name in ('rclpy', 'robot'):
        sys.modules[name] = None
    import numpy as np
    import storage
    session = sys.argv[1]
    r = storage.Recorder(session, min_free_bytes=0, queue_size=4)
    def say(text):
        print(text, flush=True)
    r.event('session_start', {}, durable=True); say('ACK event')
    r.start_visit({'visit_id': 'c001_t0', 'cycle': 1, 'target_id': 0, 'status': 'started'}); say('ACK start c001_t0')
    for i in range(3):
        r.frame({'frame_id': f'c001_t0_external_{i:03d}', 'visit_id': 'c001_t0', 'camera': 'external', 'valid': True},
                {'external': (np.full((40, 60, 3), i, np.uint8), 'png')})
    r.barrier(); say('ACK frames 3')
    r.update_visit({'visit_id': 'c001_t0', 'cycle': 1, 'target_id': 0, 'status': 'measured',
                    'marker_xyz_mm': [1, 2, 3], 'measurement_valid': True}); say('ACK measured c001_t0')
    r.start_visit({'visit_id': 'c001_t1', 'cycle': 1, 'target_id': 1, 'status': 'started'}); say('ACK start c001_t1')
    say('READY')
    i = 0
    while True:   # unacknowledged in-flight frames when the kill arrives
        r.frame({'frame_id': f'c001_t1_external_{i:05d}', 'visit_id': 'c001_t1', 'camera': 'external'},
                {'external': (np.full((240, 320, 3), i % 255, np.uint8), 'png')})
        i += 1
''')


class SigkillRecoveryTests(unittest.TestCase):
    def test_sigkill_preserves_acknowledged_records(self):
        session = helpers.tempdir(self) / 'session'
        session.mkdir()
        process = subprocess.Popen([sys.executable, '-c', CHILD, str(session), str(HERE)],
                                   stdout=subprocess.PIPE, text=True)
        acks = []
        try:
            for line in process.stdout:
                if line.startswith('READY'):
                    break
                acks.append(line.strip())
            import time
            time.sleep(0.5)
        finally:
            os.kill(process.pid, signal.SIGKILL)
            process.wait(10)
            process.stdout.close()
        self.assertEqual(process.returncode, -signal.SIGKILL)
        self.assertEqual(len(acks), 5, acks)
        self.assertTrue((session / storage.LOCK_NAME).exists())
        report = storage.recover(session)
        self.assertEqual(sorted(v['visit_id'] for v in report['incomplete_visits']), ['c001_t0', 'c001_t1'])
        phases = {v['visit_id']: v['phase'] for v in report['incomplete_visits']}
        self.assertEqual(phases, {'c001_t0': 'measured', 'c001_t1': 'started'})
        rows = {r['visit_id']: r for r in helpers.read_csv((session / 'measurements.csv'))}
        self.assertEqual(rows['c001_t0']['marker_x_mm'], '1')
        self.assertEqual(rows['c001_t0']['status'], 'incomplete')
        frames = [json.loads(line) for line in (session / 'frames.jsonl').read_text().splitlines()]
        ids = {f['frame_id'] for f in frames}
        self.assertTrue({f'c001_t0_external_{i:03d}' for i in range(3)} <= ids)
        for item in frames:  # every recovered frame has its verified image
            for ref in item['images'].values():
                self.assertTrue((session / ref['path']).is_file())
        events = [json.loads(line)['kind'] for line in (session / 'events.jsonl').read_text().splitlines()]
        self.assertIn('session_start', events)
        self.assertIn('offline_recovery', events)
        self.assertTrue((session / 'session.lock.stale').exists())


if __name__ == '__main__':
    unittest.main()

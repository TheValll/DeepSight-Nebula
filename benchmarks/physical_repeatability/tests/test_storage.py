import csv
import json
import sqlite3
import threading
import time
import unittest

import numpy as np

from tests import helpers
from errors import PersistenceFailure
import storage


def image(seed=0):
    return np.random.default_rng(seed).integers(0, 255, (24, 32, 3), dtype=np.uint8)


def frame(visit, index):
    return {'frame_id': f'{visit}_external_{index:03d}', 'visit_id': visit, 'cycle': 1, 'target_id': 0,
            'camera': 'external', 'valid': True, 'T_target_marker': np.eye(4).tolist(),
            'T_target_tcp': np.eye(4).tolist()}


def visit(cycle=1, target=0, status='started', **extra):
    return dict({'visit_id': f'c{cycle:03d}_t{target}', 'cycle': cycle, 'target_id': target, 'status': status},
                **extra)


class AtomicTests(unittest.TestCase):
    def test_atomic_write(self):
        directory = helpers.tempdir(self)
        storage.atomic_json(directory / 'a' / 'x.json', {'v': np.arange(3)})
        self.assertEqual(json.loads((directory / 'a/x.json').read_text()), {'v': [0, 1, 2]})
        self.assertEqual(list(directory.rglob('*.partial')), [])
        with self.assertRaises(ValueError):
            storage.atomic_json(directory / 'nan.json', {'v': float('nan')})
        self.assertFalse((directory / 'nan.json').exists())


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.dir = helpers.tempdir(self)

    def recorder(self, cls=storage.Recorder, **kwargs):
        kwargs.setdefault('min_free_bytes', 0)
        recorder = cls(self.dir, **kwargs)
        self.addCleanup(lambda: recorder.closed or self._close(recorder))
        return recorder

    @staticmethod
    def _close(recorder):
        try:
            recorder.close()
        except PersistenceFailure:
            pass

    def test_full_record_and_exports(self):
        recorder = self.recorder()
        recorder.start_visit(visit())
        for index in range(3):
            recorder.frame(frame('c001_t0', index), {'external': (image(index), 'png'), 'left': (image(9), 'jpg')})
        recorder.barrier()
        recorder.update_visit(visit(status='complete', tcp_xyz_mm=[1, 2, 3], marker_xyz_mm=[4, 5, 6],
                                    measurement_valid=True))
        recorder.cycle(1, 12.5, 1)
        recorder.event('note', {'x': 1})
        recorder.close()
        rows = list(helpers.read_csv((self.dir / 'measurements.csv')))
        self.assertEqual(rows[0]['status'], 'complete')
        self.assertEqual((rows[0]['tcp_x_mm'], rows[0]['marker_z_mm']), ('1', '6'))
        frames = [json.loads(line) for line in (self.dir / 'frames.jsonl').read_text().splitlines()]
        self.assertEqual(len(frames), 3)
        self.assertEqual(len(list(helpers.read_csv((self.dir / 'frames.csv')))), 3)
        for item in frames:
            for ref in item['images'].values():
                path = self.dir / ref['path']
                self.assertEqual(storage.hashlib.sha256(path.read_bytes()).hexdigest(), ref['sha256'])
            self.assertTrue((self.dir / 'frames/c001_t0' / f'{item["frame_id"]}.json').is_file())
        self.assertTrue((self.dir / 'visits/c001_t0.json').is_file())
        self.assertFalse((self.dir / storage.LOCK_NAME).exists())
        connection = sqlite3.connect(self.dir / storage.DB_NAME)
        self.assertEqual(connection.execute('PRAGMA journal_mode').fetchone()[0], 'wal')
        connection.close()

    def test_no_retry_and_no_update_without_start(self):
        recorder = self.recorder()
        recorder.start_visit(visit())
        with self.assertRaisesRegex(PersistenceFailure, 'retries'):
            recorder.start_visit(visit())
        recorder2 = None
        with self.assertRaises(PersistenceFailure):
            recorder.update_visit(visit(target=5))
        self.assertIsNone(recorder2)

    def test_refuses_existing_session_and_lock(self):
        self.recorder()
        with self.assertRaises(PersistenceFailure):
            storage.Recorder(self.dir, min_free_bytes=0)

    def test_backpressure_is_bounded_and_lossless(self):
        class Slow(storage.Recorder):
            def _handle(self, connection, kind, data, images):
                if kind == 'frame':
                    time.sleep(0.1)
                super()._handle(connection, kind, data, images)
        recorder = self.recorder(Slow, queue_size=2)
        recorder.start_visit(visit())
        started = time.monotonic()
        for index in range(8):
            recorder.frame(frame('c001_t0', index), {'external': (image(index), 'png')})
        submit_time = time.monotonic() - started
        recorder.barrier()
        self.assertLessEqual(recorder.stats['queue_high_water'], 2)
        self.assertGreater(submit_time, 0.4)  # producer was throttled, not buffered without bound
        connection = sqlite3.connect(self.dir / storage.DB_NAME)
        self.assertEqual(connection.execute('SELECT COUNT(*) FROM frames').fetchone()[0], 8)
        connection.close()

    def test_backpressure_timeout_raises(self):
        release = threading.Event()
        self.addCleanup(release.set)

        class Stuck(storage.Recorder):
            def _handle(self, connection, kind, data, images):
                if kind == 'frame':
                    release.wait(10)
                super()._handle(connection, kind, data, images)
        recorder = self.recorder(Stuck, queue_size=1, max_submit_block_s=0.3)
        with self.assertRaisesRegex(PersistenceFailure, 'backpressure'):
            for index in range(5):
                recorder.frame(frame('c001_t0', index), {'external': (image(index), 'png')})
        release.set()

    def test_writer_error_is_sticky(self):
        class Broken(storage.Recorder):
            def _handle(self, connection, kind, data, images):
                if kind == 'frame':
                    raise OSError('simulated disk failure')
                super()._handle(connection, kind, data, images)
        recorder = self.recorder(Broken)
        recorder.start_visit(visit())
        recorder.frame(frame('c001_t0', 0), {'external': (image(), 'png')})
        with self.assertRaisesRegex(PersistenceFailure, 'simulated'):
            recorder.barrier()
        for call in (lambda: recorder.update_visit(visit(status='measured')),
                     lambda: recorder.event('x', {}), lambda: recorder.close()):
            with self.assertRaises(PersistenceFailure):
                call()
        self.assertTrue((self.dir / storage.LOCK_NAME).exists())  # failed session stays marked

    def test_low_disk_space_stops(self):
        recorder = self.recorder()
        recorder.min_free_bytes = 10**18
        recorder.frame(frame('c001_t0', 0), {'external': (image(), 'png')})
        with self.assertRaisesRegex(PersistenceFailure, 'low disk'):
            recorder.barrier()


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.dir = helpers.tempdir(self)

    def test_recover_imports_sidecars_and_reports_partials(self):
        recorder = storage.Recorder(self.dir, min_free_bytes=0)
        recorder.start_visit(visit())
        recorder.frame(frame('c001_t0', 0), {'external': (image(), 'png')})
        recorder.close()
        (self.dir / storage.LOCK_NAME).write_text(json.dumps({'pid': 999999999}))  # stale crash lock
        # A sidecar + image committed to disk before its DB transaction (crash window).
        payload = b'not-really-a-png'
        storage.atomic_bytes(self.dir / 'images/c001_t0/extra.png', payload)
        data = frame('c001_t0', 1)
        data['images'] = {'external': {'path': 'images/c001_t0/extra.png',
                                       'sha256': storage.hashlib.sha256(payload).hexdigest()}}
        storage.atomic_json(self.dir / 'frames/c001_t0' / f'{data["frame_id"]}.json', data)
        bad = dict(frame('c001_t0', 2), images={'external': {'path': 'images/c001_t0/missing.png', 'sha256': 'x'}})
        storage.atomic_json(self.dir / 'frames/c001_t0' / f'{bad["frame_id"]}.json', bad)
        (self.dir / 'images/c001_t0/half.png.partial').write_bytes(b'xx')
        (self.dir / 'measurements.csv').unlink()
        report = storage.recover(self.dir)
        self.assertEqual(report['imported_sidecars'], ['c001_t0_external_001'])
        self.assertEqual(len(report['rejected_sidecars']), 1)
        self.assertEqual(report['partial_files'], ['images/c001_t0/half.png.partial'])
        self.assertEqual(report['incomplete_visits'], [{'visit_id': 'c001_t0', 'phase': 'started'}])
        self.assertEqual(report['frames'], 2)
        rows = list(helpers.read_csv((self.dir / 'measurements.csv')))
        self.assertEqual((rows[0]['status'], rows[0]['phase_at_interruption']), ('incomplete', 'started'))
        self.assertTrue((self.dir / 'session.lock.stale').exists())
        again = storage.recover(self.dir)  # idempotent
        self.assertEqual(again['imported_sidecars'], [])
        self.assertEqual(again['frames'], 2)

    def test_recover_refuses_live_lock(self):
        recorder = storage.Recorder(self.dir, min_free_bytes=0)
        try:
            with self.assertRaisesRegex(RuntimeError, 'locked'):
                storage.recover(self.dir)
        finally:
            recorder.close()


if __name__ == '__main__':
    unittest.main()

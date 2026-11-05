"""Crash-resistant recording: bounded writer thread, SQLite WAL journal, atomic sidecars.

Durability model (honest limits):
* SQLite runs in WAL mode with synchronous=FULL; every job is its own committed
  transaction. A call that returned from a *durable* method (start_visit,
  update_visit, cycle, barrier, durable event) survives SIGKILL and, as far as
  the OS/disk honour fsync, power loss.
* Frames are queued asynchronously (bounded queue, blocking backpressure, never
  dropped). Frames still in the queue or being encoded when the process dies are
  lost; a barrier() waits until every earlier job is committed. The benchmark
  issues a barrier before persisting a visit measurement, so every frame that
  contributed to a persisted measurement is durable.
* Image files and per-frame JSON sidecars are written as <name>.partial, fsynced,
  renamed and the directory fsynced. A crash can leave .partial files (reported
  by recovery, never trusted) or a complete sidecar not yet in the DB (imported
  by recovery after its image hashes are verified).
* The append-only frames.jsonl/frames.csv/events.jsonl mirrors are rewritten
  from the DB by recovery; measurements.csv/cycles.csv are atomically rewritten
  after every visit/cycle update.
* Any writer error (I/O, encoding, low disk space, SQLite) is sticky and is
  raised by every later call, so the benchmark stops issuing robot motion.
This module never imports ROS or robot code. OpenCV is imported lazily for
image encoding only.
"""
import csv
from concurrent.futures import Future, TimeoutError as FutureTimeout
import hashlib
import io
import json
import os
from pathlib import Path
import queue
import shutil
import sqlite3
import threading
import time

from errors import PersistenceFailure

DB_NAME = 'journal.sqlite3'
LOCK_NAME = 'session.lock'
TERMINAL = {'complete', 'failed', 'incomplete'}

VISIT_FIELDS = [
    'visit_id', 'cycle', 'target_id', 'repetition', 'status', 'started_utc', 'finished_utc',
    'measurement_valid', 'measurement_frame', 'timestamp',
    'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm', 'marker_x_mm', 'marker_y_mm', 'marker_z_mm',
    'valid_frames', 'analyzed_frames', 'required_frames', 'requested_frames', 'frame_target_reached', 'detection_success',
    'motion_success', 'home_success', 'robot_target_visible', 'onboard_frames',
    'onboard_max_abs_skew_s', 'onboard_issues', 'failure_category', 'frame_failures',
    'ball_crosscheck_attempts', 'ball_crosscheck_ok', 'ball_crosscheck_median_mm',
    'wrist_roll_rad', 'gripper_m', 'max_joint_error_rad', 'transit_roll_excursion_rad',
    'motion_s', 'settling_s', 'capture_s', 'home_s', 'visit_s',
    'partial_tcp_xyz_mm', 'partial_marker_xyz_mm', 'phase_at_interruption',
    'config_sha256', 'fingerprint_sha256', 'tcp_calibration_sha256', 'notes']

FRAME_FIELDS = [
    'frame_id', 'visit_id', 'cycle', 'target_id', 'camera', 'sequence', 'acquired_utc',
    'acquired_monotonic', 'analysis_index', 'valid', 'failure_category', 'notes',
    'target_reprojection_px', 'marker_reprojection_px',
    'marker_x_mm', 'marker_y_mm', 'marker_z_mm', 'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
    'wrist_roll_rad', 'gripper_m', 'joint_age_s', 'paired_external_frame', 'skew_s',
    'robot_target_visible', 'ball_status', 'ball_error_mm', 'image_sha256',
    'T_camera_target', 'T_camera_marker', 'T_target_marker', 'T_target_tcp', 'corners', 'images']

CYCLE_FIELDS = ['cycle', 'seconds', 'completed_visits', 'timestamp']


def dumps(value):
    def encode(item):
        if hasattr(item, 'tolist'):
            return item.tolist()
        if isinstance(item, Path):
            return str(item)
        raise TypeError(type(item).__name__)
    return json.dumps(value, ensure_ascii=False, allow_nan=False, default=encode)


def fsync_directory(directory):
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_bytes(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.partial')
    with temporary.open('wb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    fsync_directory(path.parent)


def atomic_json(path, value):
    atomic_bytes(path, (json.dumps(json.loads(dumps(value)), indent=2) + '\n').encode())


def _cell(value):
    if isinstance(value, (dict, list, tuple)):
        return dumps(value)
    return '' if value is None else value


def csv_bytes(fields, rows):
    buffer = io.StringIO(newline='')
    writer = csv.DictWriter(buffer, fieldnames=fields, extrasaction='ignore', lineterminator='\n')
    writer.writeheader()
    for row in rows:
        writer.writerow({k: _cell(row.get(k)) for k in fields})
    return buffer.getvalue().encode()


def visit_csv_row(data):
    row = dict(data)
    for prefix in ('tcp', 'marker'):
        xyz = data.get(f'{prefix}_xyz_mm')
        if xyz is not None:
            row.update({f'{prefix}_{axis}_mm': value for axis, value in zip('xyz', xyz)})
    return row


def frame_csv_row(data):
    row = dict(data)
    for prefix in ('tcp', 'marker'):
        transform = data.get('T_target_tcp' if prefix == 'tcp' else 'T_target_marker')
        if transform is not None:
            row.update({f'{prefix}_{axis}_mm': transform[i][3] * 1000 for i, axis in enumerate('xyz')})
    row['image_sha256'] = {k: v.get('sha256') for k, v in (data.get('images') or {}).items()}
    return row


SCHEMA = '''
CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY, data TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY, kind TEXT NOT NULL, data TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS visits(cycle INTEGER NOT NULL, target_id INTEGER NOT NULL,
    status TEXT NOT NULL, data TEXT NOT NULL, PRIMARY KEY(cycle, target_id));
CREATE TABLE IF NOT EXISTS frames(id TEXT PRIMARY KEY, visit_id TEXT NOT NULL, data TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS cycles(cycle INTEGER PRIMARY KEY, seconds REAL, completed_visits INTEGER, data TEXT);
'''


def connect(directory):
    connection = sqlite3.connect(Path(directory) / DB_NAME, timeout=30)
    mode = connection.execute('PRAGMA journal_mode=WAL').fetchone()[0]
    if mode.lower() != 'wal':
        raise PersistenceFailure(f'SQLite WAL mode unavailable ({mode})')
    connection.execute('PRAGMA synchronous=FULL')
    connection.executescript(SCHEMA)
    connection.commit()
    return connection


def export_tables(connection, directory, *, frames=False):
    """Atomically rebuild CSV/JSONL mirrors from committed DB rows."""
    directory = Path(directory)
    visits = [json.loads(r[0]) for r in connection.execute('SELECT data FROM visits ORDER BY cycle, target_id')]
    atomic_bytes(directory / 'measurements.csv', csv_bytes(VISIT_FIELDS, [visit_csv_row(v) for v in visits]))
    cycles = [dict(json.loads(r[0] or '{}'), cycle=r[1], seconds=r[2], completed_visits=r[3])
              for r in connection.execute('SELECT data, cycle, seconds, completed_visits FROM cycles ORDER BY cycle')]
    atomic_bytes(directory / 'cycles.csv', csv_bytes(CYCLE_FIELDS, cycles))
    if frames:
        rows = [json.loads(r[0]) for r in connection.execute('SELECT data FROM frames ORDER BY rowid')]
        atomic_bytes(directory / 'frames.jsonl', ''.join(dumps(r) + '\n' for r in rows).encode())
        atomic_bytes(directory / 'frames.csv', csv_bytes(FRAME_FIELDS, [frame_csv_row(r) for r in rows]))
        events = [dict(json.loads(r[1]), kind=r[0]) for r in connection.execute('SELECT kind, data FROM events ORDER BY id')]
        atomic_bytes(directory / 'events.jsonl', ''.join(dumps(e) + '\n' for e in events).encode())


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class Recorder:
    """Single writer thread owning the DB and all files in a session directory."""

    def __init__(self, directory, *, queue_size=16, image_format='png', png_compression=1,
                 jpeg_quality=95, min_free_bytes=5 * 1024**3, max_submit_block_s=30.0,
                 durable_timeout_s=120.0):
        self.directory = Path(directory)
        if not self.directory.is_dir():
            raise PersistenceFailure(f'session directory {self.directory} does not exist')
        if (self.directory / DB_NAME).exists():
            raise PersistenceFailure('refusing to reuse an existing session journal')
        self.lock = self.directory / LOCK_NAME
        try:
            descriptor = os.open(self.lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError as error:
            raise PersistenceFailure('session is locked by another recorder') from error
        os.write(descriptor, dumps({'pid': os.getpid(), 'started': time.time()}).encode())
        os.fsync(descriptor)
        os.close(descriptor)
        self.queue = queue.Queue(maxsize=queue_size)
        self.image_format = image_format
        self.png_compression = png_compression
        self.jpeg_quality = jpeg_quality
        self.min_free_bytes = min_free_bytes
        self.max_submit_block_s = max_submit_block_s
        self.durable_timeout_s = durable_timeout_s
        self.error = None
        self.closed = False
        self.stats = {'queue_high_water': 0, 'max_submit_block_s': 0.0, 'frames': 0}
        self._streams = {}
        self.thread = threading.Thread(target=self._worker, name='benchmark-recorder', daemon=True)
        self.thread.start()
        self.barrier()

    # ---------------------------------------------------------------- producer side
    def check(self):
        if self.error is not None:
            raise PersistenceFailure(f'recording failed: {self.error!r}') from self.error

    def _submit(self, kind, data, images=None):
        if self.closed:
            raise PersistenceFailure('recorder is closed')
        future = Future()
        job = (kind, data, images, future)
        started = time.monotonic()
        while True:
            self.check()
            try:
                self.queue.put(job, timeout=0.1)
                break
            except queue.Full:
                if time.monotonic() - started > self.max_submit_block_s:
                    raise PersistenceFailure(
                        f'writer backpressure exceeded {self.max_submit_block_s:.0f} s; disk too slow') from None
        blocked = time.monotonic() - started
        self.stats['max_submit_block_s'] = max(self.stats['max_submit_block_s'], blocked)
        self.stats['queue_high_water'] = max(self.stats['queue_high_water'], self.queue.qsize())
        return future

    def _wait(self, future):
        try:
            future.result(timeout=self.durable_timeout_s)
        except FutureTimeout:
            raise PersistenceFailure('durable write timed out') from None
        except PersistenceFailure:
            raise
        except BaseException as error:
            raise PersistenceFailure(f'recording failed: {error!r}') from error
        self.check()

    def barrier(self):
        """Return only after every earlier job is committed and mirrors fsynced."""
        self._wait(self._submit('barrier', None))

    def meta(self, key, value):
        self._wait(self._submit('meta', (key, value)))

    def event(self, kind, data=None, durable=False):
        future = self._submit('event', (kind, dict(data or {}, utc=_utc())))
        if durable:
            self._wait(future)
        return future

    def start_visit(self, row):
        """Insert a new visit (no retries: an existing cycle/target is an error)."""
        self._wait(self._submit('visit_start', dict(row)))

    def update_visit(self, row):
        self._wait(self._submit('visit_update', dict(row)))

    def frame(self, data, images):
        """Queue an analyzed frame. images: {name: (ndarray, 'png'|'jpg')}.

        Blocks under backpressure (never drops); arrays are copied so callers may reuse buffers.
        """
        detached = {name: (array.copy(), fmt) for name, (array, fmt) in images.items()}
        return self._submit('frame', dict(data), detached)

    def cycle(self, cycle, seconds, completed, extra=None):
        self._wait(self._submit('cycle', (cycle, seconds, completed, dict(extra or {}, timestamp=_utc()))))

    def close(self):
        if self.closed:
            return
        try:
            if self.error is None:
                self._wait(self._submit('close', None))
            elif self.thread.is_alive():
                try:  # error raised outside the worker: release it without further writes
                    self.queue.put_nowait(('stop', None, None, Future()))
                except queue.Full:
                    pass
        finally:
            self.closed = True
            self.thread.join(timeout=self.durable_timeout_s)
            if self.error is None and not self.thread.is_alive():
                try:
                    self.lock.unlink()
                except FileNotFoundError:
                    pass
        self.check()

    # ---------------------------------------------------------------- writer side
    def _stream(self, name):
        if name not in self._streams:
            self._streams[name] = (self.directory / name).open('a', newline='')
        return self._streams[name]

    def _sync_streams(self):
        for stream in self._streams.values():
            stream.flush()
            os.fsync(stream.fileno())

    def _check_disk(self):
        free = shutil.disk_usage(self.directory).free
        if free < self.min_free_bytes:
            raise PersistenceFailure(f'low disk space: {free/1e9:.2f} GB free < {self.min_free_bytes/1e9:.2f} GB')

    def _encode(self, array, fmt):
        import cv2
        if fmt == 'png':
            ok, encoded = cv2.imencode('.png', array, [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression])
        elif fmt == 'jpg':
            ok, encoded = cv2.imencode('.jpg', array, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        else:
            raise ValueError(f'unsupported image format {fmt}')
        if not ok:
            raise OSError('image encoding failed')
        return encoded.tobytes()

    def _write_frame(self, connection, data, images):
        self._check_disk()
        identifier = data['frame_id']
        visit = data['visit_id']
        references = {}
        for name, (array, fmt) in images.items():
            payload = self._encode(array, fmt)
            relative = Path('images') / visit / f'{identifier}_{name}.{fmt}'
            atomic_bytes(self.directory / relative, payload)
            references[name] = {'path': str(relative), 'sha256': hashlib.sha256(payload).hexdigest(),
                                'bytes': len(payload), 'format': fmt, 'shape': list(array.shape)}
        data['images'] = references
        atomic_json(self.directory / 'frames' / visit / f'{identifier}.json', data)
        connection.execute('INSERT INTO frames(id, visit_id, data) VALUES(?,?,?)', (identifier, visit, dumps(data)))
        connection.commit()
        self._stream('frames.jsonl').write(dumps(data) + '\n')
        stream = self._stream('frames.csv')
        if stream.tell() == 0:
            stream.write(csv_bytes(FRAME_FIELDS, []).decode())
        stream.write(csv_bytes(FRAME_FIELDS, [frame_csv_row(data)]).decode().split('\n', 1)[1])
        self.stats['frames'] += 1

    def _write_visit(self, connection, data, insert):
        if insert:
            try:
                connection.execute('INSERT INTO visits(cycle, target_id, status, data) VALUES(?,?,?,?)',
                                   (data['cycle'], data['target_id'], data['status'], dumps(data)))
            except sqlite3.IntegrityError as error:
                raise PersistenceFailure(f'visit {data["visit_id"]} already exists; retries are not allowed') from error
        else:
            cursor = connection.execute('UPDATE visits SET status=?, data=? WHERE cycle=? AND target_id=?',
                                        (data['status'], dumps(data), data['cycle'], data['target_id']))
            if cursor.rowcount != 1:
                raise PersistenceFailure(f'visit {data["visit_id"]} was never started')
        connection.commit()
        atomic_json(self.directory / 'visits' / f'{data["visit_id"]}.json', data)
        self._sync_streams()
        export_tables(connection, self.directory)

    def _handle(self, connection, kind, data, images):
        if kind == 'frame':
            self._write_frame(connection, data, images)
        elif kind in ('visit_start', 'visit_update'):
            self._write_visit(connection, data, kind == 'visit_start')
        elif kind == 'event':
            connection.execute('INSERT INTO events(kind, data) VALUES(?,?)', (data[0], dumps(data[1])))
            connection.commit()
            self._stream('events.jsonl').write(dumps(dict(data[1], kind=data[0])) + '\n')
        elif kind == 'cycle':
            cycle, seconds, completed, extra = data
            connection.execute('INSERT INTO cycles VALUES(?,?,?,?)', (cycle, seconds, completed, dumps(extra)))
            connection.commit()
            self._sync_streams()
            export_tables(connection, self.directory)
        elif kind == 'meta':
            connection.execute('INSERT OR REPLACE INTO meta VALUES(?,?)', (data[0], dumps(data[1])))
            connection.commit()
        elif kind in ('barrier', 'close'):
            self._check_disk()
            self._sync_streams()
            if kind == 'close':
                export_tables(connection, self.directory, frames=True)
                connection.execute('PRAGMA wal_checkpoint(TRUNCATE)')
        else:
            raise ValueError(f'unknown recording operation {kind}')

    def _worker(self):
        connection = None
        current = None
        try:
            connection = connect(self.directory)
            while True:
                current = self.queue.get()
                kind, data, images, future = current
                if kind == 'stop':
                    future.set_result(None)
                    break
                self._handle(connection, kind, data, images)
                future.set_result(None)
                current = None
                if kind == 'close':
                    break
        except BaseException as error:  # sticky: every later call raises
            self.error = error
            if current is not None and not current[3].done():
                current[3].set_exception(error)
            while True:
                try:
                    job = self.queue.get_nowait()
                except queue.Empty:
                    break
                job[3].set_exception(error)
        finally:
            for stream in self._streams.values():
                try:
                    stream.close()
                except OSError:
                    pass
            if connection is not None:
                connection.close()


def _utc():
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def recover(directory, *, force_unlock=False):
    """Offline recovery/export. No ROS, motion or camera imports; never resumes a robot.

    * verifies SQLite integrity (WAL is replayed by SQLite on open),
    * imports complete per-frame sidecars missing from the DB after verifying image hashes,
    * reports .partial files, sidecars with missing/corrupt images and orphan images,
    * marks visits whose last durable status is not terminal as 'incomplete',
    * rebuilds measurements.csv, cycles.csv, frames.jsonl, frames.csv and events.jsonl.
    """
    directory = Path(directory)
    if not (directory / DB_NAME).is_file():
        raise FileNotFoundError(f'{directory / DB_NAME} not found')
    lock = directory / LOCK_NAME
    lock_info = None
    if lock.exists():
        try:
            lock_info = json.loads(lock.read_text() or '{}')
        except ValueError:
            lock_info = {'unreadable': True}
        pid = lock_info.get('pid')
        if pid and _pid_alive(pid) and not force_unlock:
            raise RuntimeError(f'session still locked by running process {pid}; refusing to recover')
    connection = sqlite3.connect(directory / DB_NAME, timeout=30)
    try:
        integrity = connection.execute('PRAGMA integrity_check').fetchone()[0]
        if integrity != 'ok':
            raise RuntimeError(f'database integrity check failed: {integrity}')
        connection.execute('PRAGMA synchronous=FULL')
        known = {r[0] for r in connection.execute('SELECT id FROM frames')}
        report = {'timestamp': _utc(), 'integrity': integrity, 'imported_sidecars': [],
                  'rejected_sidecars': [], 'partial_files': [], 'orphan_images': [],
                  'incomplete_visits': [], 'stale_lock': lock_info}
        referenced = set()
        for path in sorted((directory / 'frames').rglob('*.json')):
            try:
                data = json.loads(path.read_text())
                images = data.get('images', {})
                ok = all((directory / v['path']).is_file() and
                         hashlib.sha256((directory / v['path']).read_bytes()).hexdigest() == v['sha256']
                         for v in images.values())
            except (ValueError, KeyError, OSError) as error:
                report['rejected_sidecars'].append({'path': str(path.relative_to(directory)), 'error': repr(error)})
                continue
            if not ok:
                report['rejected_sidecars'].append({'path': str(path.relative_to(directory)),
                                                    'error': 'missing or corrupt image'})
                continue
            referenced.update(v['path'] for v in images.values())
            if data['frame_id'] not in known:
                data['recovered_from_sidecar'] = True
                connection.execute('INSERT INTO frames(id, visit_id, data) VALUES(?,?,?)',
                                   (data['frame_id'], data['visit_id'], dumps(data)))
                report['imported_sidecars'].append(data['frame_id'])
        for row in connection.execute('SELECT data FROM frames'):
            referenced.update(v['path'] for v in json.loads(row[0]).get('images', {}).values())
        report['partial_files'] = sorted(str(p.relative_to(directory)) for p in directory.rglob('*.partial'))
        if (directory / 'images').is_dir():
            report['orphan_images'] = sorted(str(p.relative_to(directory)) for p in (directory / 'images').rglob('*')
                                             if p.is_file() and not p.name.endswith('.partial')
                                             and str(p.relative_to(directory)) not in referenced)
        for cycle, target, status, raw in connection.execute(
                'SELECT cycle, target_id, status, data FROM visits').fetchall():
            if status in TERMINAL:
                continue
            data = json.loads(raw)
            data.update(status='incomplete', phase_at_interruption=status,
                        notes=(data.get('notes') or '') + ' | incomplete: process ended before this visit finished')
            connection.execute('UPDATE visits SET status=?, data=? WHERE cycle=? AND target_id=?',
                               ('incomplete', dumps(data), cycle, target))
            report['incomplete_visits'].append({'visit_id': data.get('visit_id'), 'phase': status})
        connection.execute('INSERT INTO events(kind, data) VALUES(?,?)',
                           ('offline_recovery', dumps({k: (len(v) if isinstance(v, list) else v)
                                                       for k, v in report.items()})))
        connection.commit()
        export_tables(connection, directory, frames=True)
        connection.execute('PRAGMA wal_checkpoint(TRUNCATE)')
        report['frames'] = connection.execute('SELECT COUNT(*) FROM frames').fetchone()[0]
        report['visits'] = connection.execute('SELECT COUNT(*) FROM visits').fetchone()[0]
        report['visit_status'] = dict(connection.execute('SELECT status, COUNT(*) FROM visits GROUP BY status'))
    finally:
        connection.close()
    stamp = report['timestamp'].replace(':', '').replace('-', '').replace('.', '_')
    atomic_json(directory / f'recovery_{stamp}.json', report)
    session = directory / 'session.json'
    if session.exists():
        state = json.loads(session.read_text())
        if state.get('status') == 'running':
            state.update(status='interrupted_recovered', recovered_at=report['timestamp'])
            atomic_json(session, state)
    if lock.exists() and not (lock_info and lock_info.get('pid') and _pid_alive(lock_info['pid'])):
        lock.rename(directory / 'session.lock.stale')
    return report

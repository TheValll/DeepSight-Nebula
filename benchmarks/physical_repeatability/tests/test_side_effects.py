"""help/preflight/recover/report/test imports must not touch ROS, cameras or launch processes."""
import json
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest

from tests import helpers

HERE = Path(__file__).resolve().parents[1]

PROBE = textwrap.dedent('''
    import json, subprocess, sys
    sys.path.insert(0, sys.argv[1])
    FORBIDDEN = ('rclpy', 'robot', 'moveit_msgs', 'action_msgs', 'sensor_msgs', 'launch', 'launch_ros', 'metrology')
    class Block:
        def find_spec(self, name, path=None, target=None):
            if name.split('.')[0] in FORBIDDEN:
                raise ImportError('forbidden import ' + name)
    sys.meta_path.insert(0, Block())
    import cv2
    def no_camera(*a, **k):
        raise AssertionError('camera opened')
    cv2.VideoCapture = no_camera
    launched = []
    real_popen = subprocess.Popen
    class Popen(real_popen):
        def __init__(self, args, *a, **k):
            launched.append(args)
            if args and args[0] != 'git':
                raise AssertionError('process launched: %r' % (args,))
            super().__init__(args, *a, **k)
    subprocess.Popen = Popen
    import benchmark, recover, report, storage
    codes = {}
    for name, call in (('help', lambda: benchmark.main(['--help'])),
                       ('recover_help', lambda: recover.main(['--help'])),
                       ('report_help', lambda: report.main(['--help']))):
        try:
            call()
        except SystemExit as stop:
            codes[name] = stop.code
    codes['preflight'] = benchmark.main(['preflight', '--calibration-validation', '--cycles', '1',
                                         '--tcp-calibration', sys.argv[2] + '/none.json',
                                         '--output', sys.argv[2] + '/never_created'])
    session = sys.argv[2] + '/session'
    import os
    os.mkdir(session)
    r = storage.Recorder(session, min_free_bytes=0)
    r.start_visit({'visit_id': 'c001_t0', 'cycle': 1, 'target_id': 0, 'status': 'started'})
    r.close()
    codes['recover'] = recover.main([session])
    codes['report'] = report.main([session])
    loaded = sorted(m for m in FORBIDDEN if m in sys.modules)
    print('RESULT ' + json.dumps({'codes': codes, 'loaded': loaded, 'launched': launched}))
''')


class SideEffectTests(unittest.TestCase):
    def test_offline_commands_have_no_hardware_side_effects(self):
        directory = helpers.tempdir(self)
        result = subprocess.run([sys.executable, '-c', PROBE, str(HERE), str(directory)],
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        line = next(l for l in result.stdout.splitlines() if l.startswith('RESULT '))
        data = json.loads(line[7:])
        self.assertEqual(data['codes'], {'help': 0, 'recover_help': 0, 'report_help': 0, 'preflight': 0,
                                         'recover': 0, 'report': 0})
        self.assertEqual(data['loaded'], [])
        self.assertEqual(data['launched'], [])
        self.assertFalse((directory / 'never_created').exists())
        self.assertIn('Offline preflight', result.stdout)
        self.assertIn('FUNCTIONAL TEST ONLY', result.stdout)

    def test_run_sh_help(self):
        result = subprocess.run([str(HERE / 'run.sh'), '--help'], capture_output=True, text=True, timeout=60,
                                env={'PATH': '/usr/bin:/bin', 'PYTHON': sys.executable})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('50 cycles, velocity 1.0, acceleration 0.10', result.stdout)
        self.assertIn('default: 50', result.stdout)
        self.assertIn('default: 1.0', result.stdout)


if __name__ == '__main__':
    unittest.main()

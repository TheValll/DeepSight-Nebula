import unittest

import numpy as np

import report


def visit(target, cycle, xyz, valid=True, status='complete', **extra):
    row = {'visit_id': f'c{cycle:03d}_t{target}', 'cycle': cycle, 'target_id': target, 'status': status,
           'measurement_valid': valid, 'motion_success': True, 'visit_s': 7.0}
    if valid:
        row.update(marker_xyz_mm=xyz, tcp_xyz_mm=[v + 10 for v in xyz])
    else:
        row.update(partial_marker_xyz_mm=xyz, partial_tcp_xyz_mm=xyz, failure_category='ARUCO_TARGET_NOT_DETECTED')
    row.update(extra)
    return row


class ReportTests(unittest.TestCase):
    def test_only_complete_measurements_count(self):
        visits = [visit(0, 1, [0, 0, 0]), visit(0, 2, [2, 0, 0]), visit(0, 3, [1000, 0, 0], valid=False, status='failed'),
                  visit(1, 1, [5, 5, 5]), visit(1, 2, [6, 6, 6], valid=False, status='incomplete')]
        result = report.analyse(visits, [{'cycle': 1, 'seconds': 60.0}, {'cycle': 2, 'seconds': 62.0}], {},
                                {'cycles': 50, 'target_ids': [0, 1]})
        target0 = result['per_target']['0']
        self.assertEqual(target0['valid_repetitions'], 2)
        np.testing.assert_allclose(target0['marker8']['mean_xyz_mm'], [1, 0, 0])
        np.testing.assert_allclose(target0['tcp']['mean_xyz_mm'], [11, 10, 10])
        np.testing.assert_allclose(target0['tcp']['std_xyz_mm'], [np.sqrt(2), 0, 0])
        self.assertAlmostEqual(target0['tcp']['max_deviation_mm'], 1.0)
        self.assertEqual(target0['failures'], {'ARUCO_TARGET_NOT_DETECTED': 1})
        target1 = result['per_target']['1']
        self.assertEqual(target1['valid_repetitions'], 1)
        self.assertNotIn('mean_deviation_mm', target1['tcp'])
        self.assertTrue(any('FUNCTIONAL TEST ONLY' in w for w in result['warnings']))
        self.assertTrue(any('incomplete' in w for w in result['warnings']))
        self.assertAlmostEqual(result['timings']['estimated_total_for_configured_cycles_min'], 61 * 50 / 60)
        self.assertIn('| 0 | 2 |', report.markdown(result))

    def test_marker_only_session(self):
        rows = [dict(visit(0, c, [c, 0, 0]), tcp_xyz_mm=None) for c in (1, 2, 3)]
        result = report.analyse(rows, [], {}, None)
        self.assertEqual(result['per_target']['0']['tcp']['n'], 0)
        self.assertEqual(result['per_target']['0']['marker8']['n'], 3)
        self.assertTrue(any('marker-8 values only' in w for w in result['warnings']))

    def test_provisional_tcp_and_crosscheck_outliers_are_visible(self):
        rows = [visit(2, c, [c, 0, 0]) for c in (1, 2)]
        result = report.analyse(rows, [], {}, ball_frames=[
            {'target_id': 2, 'ball_error_mm': 3.5},
            {'target_id': 2, 'ball_error_mm': 32.0},
            {'target_id': 7, 'ball_status': 'failed'}],
            tcp_calibration={'multi_pose_verified': False})
        check = result['ball_crosscheck']
        self.assertEqual(check['frames_over_8mm'], 1)
        self.assertEqual(check['max_mm'], 32.0)
        self.assertEqual(check['frames_without_error'], 1)
        self.assertEqual(check['checked_target_ids'], [2])
        self.assertTrue(any('PROVISIONAL TCP' in w for w in result['warnings']))
        self.assertTrue(any('cause not established' in w for w in result['warnings']))
        self.assertTrue(any('upstream' in w for w in result['warnings']))


if __name__ == '__main__':
    unittest.main()

import unittest

from compute_pei import WEIGHTS, meets_requirements


class PeiRequirementsTests(unittest.TestCase):
    def row(self, **overrides):
        row = dict(detection=.87, precision_percent=96.0, latency_ms_p95=40.0, confidence=.82)
        row.update(overrides)
        return row

    def test_weights_sum_to_one(self):
        self.assertAlmostEqual(sum(WEIGHTS.values()), 1)
        self.assertNotIn("detection", WEIGHTS)

    def test_exact_limits_pass_and_low_confidence_is_not_eliminatory(self):
        self.assertTrue(meets_requirements(self.row()))

    def test_limits_use_unrounded_values(self):
        for change in (dict(detection=.86999), dict(precision_percent=95.999), dict(latency_ms_p95=40.001)):
            with self.subTest(change=change):
                self.assertFalse(meets_requirements(self.row(**change)))

    def test_undefined_precision_fails(self):
        self.assertFalse(meets_requirements(self.row(precision_percent=None)))


if __name__ == "__main__":
    unittest.main()

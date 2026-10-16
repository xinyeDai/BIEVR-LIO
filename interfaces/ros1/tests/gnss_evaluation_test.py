#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

spec = importlib.util.spec_from_file_location(
    'gnss_report', Path(__file__).resolve().parents[3] / 'scripts/plot_gnss_optimization.py')
reporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reporter)


class EvaluationTest(unittest.TestCase):
    def test_known_errors_lever_arm_and_duplicate_fixes(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            stamps = np.arange(100., 105.)
            ref = np.column_stack((stamps, stamps - 100, np.ones(5), np.zeros(5)))
            # Duplicate receiver epochs must not overweight that fix.
            np.savetxt(directory / 'gnss.txt', np.vstack((ref, ref[2])), fmt='%.9f')
            errors = np.arange(-2., 3.)
            body = np.column_stack((stamps - 100 + errors, np.full(5, -3.), np.full(5, 4.)))
            optimized = np.column_stack((stamps - 100 + .25, np.full(5, -.5), np.full(5, .75)))
            # A 90 degree yaw turns body lever arm [1, 0, 0] into ENU [0, 1, 0].
            quat = np.tile([0, 0, np.sqrt(.5), np.sqrt(.5)], (5, 1))
            for name, positions in [('lio_aligned_enu_tum.txt', body),
                                    ('gnss_optimized_tum.txt', optimized)]:
                np.savetxt(directory / name, np.column_stack((stamps, positions, quat)), fmt='%.12f')
            summary = {'gnss_enu': str(directory / 'gnss.txt'),
                       'gnss_lever_arm_body_m': [1, 0, 0], 'evaluation_max_time_gap_s': .2}
            (directory / 'summary.yaml').write_text(yaml.safe_dump(summary))
            result = reporter.report(directory)
            self.assertEqual(result['matched_samples'], 5)
            self.assertAlmostEqual(result['lio']['x']['rmse_m'], np.sqrt(2))
            self.assertAlmostEqual(result['lio']['x']['max_m'], 2)
            self.assertAlmostEqual(result['lio']['y']['max_m'], 3)
            self.assertAlmostEqual(result['lio']['three_dimensional']['rmse_m'], np.sqrt(27))
            self.assertAlmostEqual(result['lio']['three_dimensional']['max_m'], np.sqrt(29))
            self.assertAlmostEqual(result['gnss_optimized_lio']['three_dimensional']['rmse_m'],
                                   np.sqrt(.25**2 + .5**2 + .75**2))
            saved = json.loads((directory / 'evaluation_metrics.json').read_text())
            self.assertEqual(saved, result)
            for filename in ['trajectory_xyz.png', 'position_error_xyz.png']:
                self.assertTrue((directory / filename).read_bytes().startswith(b'\x89PNG'))
            self.assertEqual(len((directory / 'evaluation_metrics.csv').read_text().splitlines()), 11)
            self.assertEqual(len((directory / 'evaluation_samples.csv').read_text().splitlines()), 6)

    def test_no_interpolation_across_large_gap_or_extrapolation(self):
        stamps = np.array([10., 10.1, 11., 11.1])
        positions = np.column_stack((stamps, stamps, stamps))
        query = np.array([9.9, 10., 10.05, 10.5, 11., 11.1, 11.2])
        _, valid = reporter.interpolate(stamps, positions, query, .2)
        self.assertEqual(valid.tolist(), [False, True, True, False, True, True, False])


if __name__ == '__main__':
    unittest.main()

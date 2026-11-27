import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from run_indoor_dynamic_baseline import commands, verify_point_parameters


class IndoorBaselineTest(unittest.TestCase):
    def test_stages_use_raw_keyframes_and_recorded_parameters(self):
        runs = commands(Path('/input'), Path('/output'), Path('/visibility'), Path('/scripts'))
        self.assertEqual(runs[0][-2:], ['--output-voxel', '0.05'])
        self.assertEqual(runs[1][runs[1].index('--map') + 1], '/output/01_point/clean_map.pcd')
        self.assertEqual(runs[1][runs[1].index('--sigma-factor') + 1], '2.5')
        self.assertEqual(runs[2][runs[2].index('--map') + 1], '/output/02_outlier/kept.pcd')
        self.assertEqual(runs[2][runs[2].index('--keyframes') + 1], '/input/keyframes')
        self.assertEqual(runs[2][runs[2].index('--temporal-mode') + 1], 'bidirectional')

    def test_rejects_changed_visibility_defaults(self):
        with TemporaryDirectory() as tmp:
            report = Path(tmp) / 'report.yaml'
            report.write_text('angular_resolution_rad: 0.02\n')
            with self.assertRaises(ValueError):
                verify_point_parameters(report)

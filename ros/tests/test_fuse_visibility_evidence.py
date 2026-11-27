import sys
import json
import subprocess
from pathlib import Path
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from fuse_visibility_evidence import decide, voxel_keys
from multiframe_outlier_review import load_pcd, write_pcd


class FusionTests(unittest.TestCase):
    def test_unknown_or_conflicting_evidence_is_retained(self):
        point = np.array([1, 1, 1, 1, 0], dtype=bool)
        cluster = np.zeros(5, dtype=bool)
        free = np.array([2, 0, 2, 0, 2])
        occupied = np.array([0, 3, 3, 0, 0])
        fine = np.array([2, 0, 2, 0, 2])
        coarse = np.array([2, 0, 2, 0, 2])
        span = np.array([0, 25, 30, 0, 0])
        dynamic, restored, unknown = decide(point, cluster, free, occupied, fine, coarse, span)
        np.testing.assert_array_equal(dynamic, [1, 0, 0, 0, 0])
        np.testing.assert_array_equal(restored, [0, 1, 0, 0, 0])
        np.testing.assert_array_equal(unknown, [0, 0, 1, 1, 0])

    def test_voxel_keys_are_translation_consistent(self):
        points = np.array([[.011, -.01, 1.02, 1], [.012, -.02, 1.03, 2], [.06, -.02, 1.03, 3]])
        keys = voxel_keys(points, .05)
        self.assertEqual(keys[0], keys[1])
        self.assertNotEqual(keys[0], keys[2])

    def test_end_to_end_keeps_unknown_and_rejects_mismatched_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = root / 'session'; frames = session / 'keyframes'; frames.mkdir(parents=True)
            cloud = np.array([[1., 0, 0, 3.], [2., 0, 0, 4.]], dtype=np.float32)
            map_path = session / 'map.pcd'; write_pcd(map_path, cloud)
            for i in range(2): write_pcd(frames / f'{i:06d}.pcd', cloud)
            trajectory = session / 'optimized_trajectory_tum.txt'
            trajectory.write_text('0 0 0 0 0 0 0 1\n3 0 0 0 0 0 0 1\n')
            stamps = session / 'timestamps.txt'; stamps.write_text('0 0\n1 3\n')
            point_dir = root / 'point'; point_dir.mkdir()
            point = point_dir / 'removed_points.pcd'; write_pcd(point, cloud[:1])
            (point_dir / 'report.yaml').write_text(f'input_session: {session}\noutput_voxel_resolution_m: 0.05\n')
            group_dir = root / 'group'; group_dir.mkdir()
            group = group_dir / 'dynamic_candidates.pcd'; write_pcd(group, cloud[:1])
            report = {'parameters': {'map': str(map_path), 'keyframes': str(frames), 'trajectory': str(trajectory), 'timestamps': str(stamps)}}
            (group_dir / 'report.json').write_text(json.dumps(report))
            output = root / 'out'
            command = [sys.executable, str(Path(__file__).parents[1] / 'scripts/fuse_visibility_evidence.py'), '--map', str(map_path), '--point-candidates', str(point), '--cluster-candidates', str(group), '--keyframes', str(frames), '--trajectory', str(trajectory), '--timestamps', str(stamps), '--output', str(output)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            np.testing.assert_array_equal(load_pcd(output / 'kept.pcd'), cloud)
            self.assertEqual(json.loads((output / 'report.json').read_text())['uncertain_retained'], 1)
            report['parameters']['map'] = str(root / 'different.pcd')
            (group_dir / 'report.json').write_text(json.dumps(report))
            command[-1] = str(root / 'bad')
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)


if __name__ == '__main__':
    unittest.main()

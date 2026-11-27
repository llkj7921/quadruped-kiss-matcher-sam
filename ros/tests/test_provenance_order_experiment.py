import sys
import json
import subprocess
from pathlib import Path
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from provenance_order_experiment import decide_orders, voxel_keys, voxelized
from capture_visibility_evidence import sha256_files, source_hashes
from multiframe_outlier_review import load_pcd, write_pcd
from compare_label_aware_orders import labels


class ProvenanceOrderTest(unittest.TestCase):
    def test_label_aware_orders_use_same_observations_but_distinct_groups(self):
        point = np.array([True, False, False, False])
        raw_group = np.array([False, True, False, True])
        after_group = np.array([False, False, True, True])
        free = np.array([3, 2, 2, 0]); occupied = np.zeros(4, dtype=int)
        point_first, cluster_first = labels(point, raw_group, after_group, free, occupied)
        np.testing.assert_array_equal(point_first, [True, False, True, False])
        np.testing.assert_array_equal(cluster_first, [True, True, False, False])
        np.testing.assert_array_equal(raw_group, [False, True, False, True])

    def test_fixed_candidate_labels_commute(self):
        point = np.array([True, False, True, False])
        cluster = np.array([False, True, True, False])
        forward, reverse = decide_orders(point, cluster)
        np.testing.assert_array_equal(forward, [True, True, True, False])
        np.testing.assert_array_equal(forward, reverse)
        np.testing.assert_array_equal(point, [True, False, True, False])

    def test_voxel_rebuild_preserves_centroids_and_shared_sources(self):
        cloud = np.array([[.011, 0, 0, 1], [.021, 0, 0, 3], [.09, 0, 0, 5]], dtype=np.float32)
        keys = voxel_keys(cloud, .05)
        self.assertEqual(keys[0], keys[1])
        self.assertNotEqual(keys[1], keys[2])
        rebuilt = voxelized(cloud, .05)
        self.assertEqual(len(rebuilt), 2)
        centroid = rebuilt[np.argmin(rebuilt[:, 0])]
        np.testing.assert_allclose(centroid, [.016, 0, 0, 2], atol=1e-6)
        filtered = voxelized(cloud[[False, True, True]], .05)
        self.assertEqual(len(filtered), 2)
        self.assertAlmostEqual(float(filtered[np.argmin(filtered[:, 0]), 0]), .021, places=6)

    def test_full_provenance_and_source_specific_labels(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); session = root / 'map'; frames = session / 'keyframes'; frames.mkdir(parents=True)
            points = np.array([[1., 0, 0, 1], [2., 0, 0, 2]], dtype=np.float32)
            for i in range(2): write_pcd(frames / f'{i:06d}.pcd', points)
            raw = session / 'lab_map.pcd'; write_pcd(raw, points)
            trajectory = session / 'optimized_trajectory_tum.txt'; trajectory.write_text('0 0 0 0 0 0 0 1\n3 0 0 0 0 0 0 1\n')
            stamps = session / 'timestamps.txt'; stamps.write_text('0 0\n1 3\n')
            evidence_dir = root / 'evidence'; evidence_dir.mkdir()
            evidence = evidence_dir / 'point_evidence.tsv'; evidence.write_text('# frame point free occupied occluded\n0 0 3 0 0\n0 1 0 0 0\n1 0 0 0 0\n1 1 0 0 0\n')
            manifest = {'session': str(session), 'source_hashes': source_hashes(session), 'evidence_sha256_name_and_bytes': sha256_files([evidence])}
            (evidence_dir / 'manifest.json').write_text(json.dumps(manifest))
            group = root / 'cluster'; group.mkdir(); write_pcd(group / 'dynamic_candidates.pcd', points[:1])
            (group / 'report.json').write_text(json.dumps({'parameters': {'map': str(raw), 'keyframes': str(frames), 'trajectory': str(trajectory), 'timestamps': str(stamps)}}))
            output = root / 'output'
            cmd = [sys.executable, str(Path(__file__).parents[1] / 'scripts/provenance_order_experiment.py'), '--map', str(raw), '--keyframes', str(frames), '--trajectory', str(trajectory), '--timestamps', str(stamps), '--point-evidence', str(evidence), '--cluster-review', str(group), '--output', str(output)]
            subprocess.run(cmd, check=True, capture_output=True)
            meta = json.loads((output / 'report.json').read_text())
            self.assertEqual(meta['mapped_returns'], 4)
            self.assertEqual(meta['removed_returns'], 1)
            self.assertEqual(meta['ordering_differences'], 0)
            sidecar = np.load(output / 'source_provenance.npz')
            self.assertEqual(len(sidecar['voxel_source_indices']), 4)
            np.testing.assert_array_equal(load_pcd(output / 'point_then_cluster.pcd'), load_pcd(output / 'cluster_then_point.pcd'))
            self.assertEqual(len(load_pcd(output / 'uncertain_group_protected.pcd')), 1)
            # Modified source after capture must fail closed.
            write_pcd(frames / '000001.pcd', points * 2)
            cmd[-1] = str(root / 'bad')
            self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)


if __name__ == '__main__':
    unittest.main()

import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('outlier_review', Path(__file__).parents[1] / 'scripts' / 'multiframe_outlier_review.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


class OutlierReviewTest(unittest.TestCase):
    def test_cloud_round_trip(self):
        cloud = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.pcd'
            review.write_pcd(path, cloud)
            np.testing.assert_array_equal(review.load_pcd(path), cloud)

    def test_multiframe_support_protects_sparse_point(self):
        rng = np.random.default_rng(4)
        xyz = np.vstack([rng.normal(0, .02, (50, 3)), [2, 0, 0], [2, .02, 0]])
        support = np.ones(len(xyz), dtype=int)
        support[-1] = 2
        candidate, _, _ = review.classify(xyz, support, neighbors=10, factor=2.5, floor=.25)
        self.assertTrue(candidate[-2])
        self.assertFalse(candidate[-1])
        self.assertFalse(candidate[:50].any())


if __name__ == '__main__':
    unittest.main()

import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('adaptive', scripts / 'adaptive_outlier_review.py')
adaptive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adaptive)


class AdaptiveReviewTests(unittest.TestCase):
    def test_protected_outlier_is_preserved_and_bad_input_rejected(self):
        xyz = np.array([[0., 0, 0], [1., 0, 0], [2., 0, 0]])
        mask = np.array([False, True, True])
        result, count = adaptive.apply_protection(mask, xyz, np.array([[1., 0, 0, 42.]]))
        np.testing.assert_array_equal(result, [False, False, True])
        self.assertEqual(count, 1)
        with self.assertRaises(ValueError):
            adaptive.apply_protection(mask, xyz, np.array([[3., 0, 0, 42.]]))

    def test_isolated_component_is_separate(self):
        rng = np.random.default_rng(7)
        xyz = np.vstack([rng.normal(0, .03, (120, 3)), [3, 0, 0]])
        flag, components, _, _, size = adaptive.classify(xyz, neighbors=8, local_neighbors=40)
        self.assertTrue(flag[-1])
        self.assertTrue(components[-1])
        self.assertEqual(size[-1], 1)
        self.assertFalse(components[:120].any())

    def test_insufficient_points_rejected(self):
        with self.assertRaises(ValueError):
            adaptive.classify(np.zeros((5, 3)))


if __name__ == '__main__':
    unittest.main()

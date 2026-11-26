import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('cluster_review', scripts / 'cluster_visibility_review.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


class ClusterVisibilityTest(unittest.TestCase):
    def test_only_later_free_evidence_triggers(self):
        self.assertEqual(review.temporal_class([1., 2.], [5., 6., 7.], 2, 3, 1.), 'dynamic_candidate')
        self.assertEqual(review.temporal_class([5., 6.], [1., 2., 3.], 2, 3, 1.), 'uncertain')
        self.assertEqual(review.temporal_class([1.], [5., 6., 7.], 2, 3, 1.), 'uncertain')
        self.assertEqual(review.temporal_class([1., 2.], [5., 6.], 2, 3, 1.), 'uncertain')

    def test_clusters_respect_size_limits(self):
        xyz = np.array([[0., 0, 0], [.01, 0, 0], [5, 0, 0], [5.01, 0, 0], [5.02, 0, 0]])
        labels, sizes, ids = review.cluster_cloud(xyz, .1, 3, 5)
        self.assertEqual(len(ids), 1)
        self.assertEqual(sizes[ids[0]], 3)
        self.assertEqual(labels[2], labels[4])


if __name__ == '__main__':
    unittest.main()

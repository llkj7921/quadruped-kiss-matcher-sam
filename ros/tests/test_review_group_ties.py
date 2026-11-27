import unittest
import numpy as np
from review_group_ties import tie_candidates


class GroupTieTest(unittest.TestCase):
    def test_only_repeated_ties_in_reviewed_occluded_groups(self):
        group = np.array([1, 1, 1, 1, 1, 0], dtype=bool)
        free = np.array([2, 1, 2, 3, 0, 2])
        occupied = np.array([2, 1, 3, 3, 0, 2])
        occluded = np.array([5, 8, 9, 4, 9, 9])
        np.testing.assert_array_equal(tie_candidates(group, free, occupied, occluded), [1, 0, 0, 0, 0, 0])

#!/usr/bin/env python3
"""Experimental density-adaptive SOR and separate small-component review."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from multiframe_outlier_review import load_pcd, write_pcd


def classify(xyz, neighbors=20, local_neighbors=100, sigma_factor=2.5,
             ratio=1.5, floor=0.04, component_radius=0.12, component_max=5):
    """Return adaptive SOR candidates and isolated-component candidates.

    Neighborhoods use the entire cloud (no block-boundary truncation). Local
    medians/MAD estimate density; no map-origin distance is used as range.
    """
    if len(xyz) <= max(neighbors, local_neighbors):
        raise ValueError('not enough points for neighborhood analysis')
    tree = cKDTree(xyz)
    distance, indices = tree.query(xyz, k=local_neighbors + 1, workers=-1)
    spacing = distance[:, 1:neighbors + 1].mean(axis=1)
    adjacent_spacing = spacing[indices[:, 1:]]
    median = np.median(adjacent_spacing, axis=1)
    deviation = 1.4826 * np.median(np.abs(adjacent_spacing - median[:, None]), axis=1)
    threshold = np.maximum(median * ratio, median + np.maximum(sigma_factor * deviation, floor))
    adaptive = spacing > threshold
    pairs = tree.query_pairs(component_radius, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs), dtype=np.uint8), (pairs[:, 0], pairs[:, 1])), shape=(len(xyz), len(xyz))).tocsr()
    _, labels = connected_components(graph, directed=False)
    counts = np.bincount(labels)
    components = counts[labels] <= component_max
    return adaptive, components, spacing, threshold, counts[labels]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--neighbors', type=int, default=20)
    p.add_argument('--local-neighbors', type=int, default=100)
    p.add_argument('--sigma-factor', type=float, default=2.5)
    p.add_argument('--ratio', type=float, default=1.5)
    p.add_argument('--threshold-floor', type=float, default=.04)
    p.add_argument('--component-radius', type=float, default=.12)
    p.add_argument('--component-max-points', type=int, default=5)
    p.add_argument('--display-max-z', type=float, default=1.5)
    p.add_argument('--support-evidence', type=Path, help='Prior multiframe review directory; report support without auto-protecting or deleting')
    a = p.parse_args()
    if a.neighbors < 1 or a.local_neighbors < a.neighbors or a.sigma_factor <= 0 or a.ratio <= 1 or a.threshold_floor <= 0 or a.component_radius <= 0 or a.component_max_points < 1:
        p.error('invalid parameters')
    cloud = load_pcd(a.map)
    adaptive, components, spacing, threshold, size = classify(cloud[:, :3], a.neighbors, a.local_neighbors, a.sigma_factor, a.ratio, a.threshold_floor, a.component_radius, a.component_max_points)
    a.output.mkdir(parents=True, exist_ok=True)
    # Components are review-only, never silently removed.
    for name, mask in [('kept', ~adaptive), ('adaptive_candidates', adaptive), ('small_component_candidates', components)]:
        write_pcd(a.output / f'{name}.pcd', cloud[mask])
        write_pcd(a.output / f'{name}_display.pcd', cloud[mask & (cloud[:, 2] <= a.display_max_z)])
    np.savez_compressed(a.output / 'evidence.npz', adaptive=adaptive, components=components, spacing=spacing, threshold=threshold, component_size=size)
    params = vars(a).copy()
    params.update(map=str(a.map.resolve()), output=str(a.output.resolve()), support_evidence=str(a.support_evidence.resolve()) if a.support_evidence else None)
    report = {'input': str(a.map.resolve()), 'points': len(cloud), 'kept': int((~adaptive).sum()), 'adaptive_candidates': int(adaptive.sum()), 'small_component_candidates_review_only': int(components.sum()), 'overlap_candidates': int((adaptive & components).sum()), 'parameters': params, 'note': 'Small components are review only, not removed. No point labels or accuracy claims.'}
    if a.support_evidence:
        previous = json.loads((a.support_evidence / 'report.json').read_text())
        if Path(previous['map']).resolve() != a.map.resolve() or previous['points'] != len(cloud):
            raise ValueError('support evidence does not match this input PCD')
        support = np.load(a.support_evidence / 'evidence.npz')['support']
        if len(support) != len(cloud):
            raise ValueError('support evidence has incorrect size')
        report['multiframe_support'] = {name: {'zero_or_one': int(((support <= 1) & mask).sum()), 'two_or_more': int(((support >= 2) & mask).sum())} for name, mask in [('adaptive_candidates', adaptive), ('small_components', components)]}
    (a.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

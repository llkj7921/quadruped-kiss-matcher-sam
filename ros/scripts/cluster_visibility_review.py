#!/usr/bin/env python3
"""Offline cluster-level approximate visibility evidence; not a semantic detector."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from multiframe_outlier_review import load_pcd, write_pcd


def temporal_class(occupied, free, min_occupied, min_free, separation):
    """Require observed occupation before later free-space evidence."""
    if len(occupied) < min_occupied:
        return 'uncertain'
    first_occupied = sorted(occupied)[min_occupied - 1]
    later_free = sum(t >= first_occupied + separation for t in free)
    return 'dynamic_candidate' if later_free >= min_free else 'uncertain'


def bidirectional_class(occupied, free, positions, min_occupied, separation, baseline, time_gap):
    """Require one free observation on each side from separated poses/times.

    This is additional candidate evidence, not proof of a visible background.
    """
    if len(occupied) < min_occupied:
        return 'uncertain', []
    before = [t for t in free if t <= min(occupied) - separation]
    after = [t for t in free if t >= max(occupied) + separation]
    for first in before:
        for second in after:
            if second - first >= time_gap and np.linalg.norm(positions[first] - positions[second]) >= baseline:
                return 'dynamic_candidate', [first, second]
    return 'uncertain', []


def separated_forward_class(occupied, free, positions, min_occupied, min_free, separation, baseline, time_gap):
    if len(occupied) < min_occupied:
        return 'uncertain', []
    selected = []
    start = sorted(occupied)[min_occupied - 1] + separation
    for t in sorted(t for t in free if t >= start):
        if all(abs(t - other) >= time_gap and np.linalg.norm(positions[t] - positions[other]) >= baseline for other in selected):
            selected.append(t)
    return ('dynamic_candidate' if len(selected) >= min_free else 'uncertain'), selected


def cluster_cloud(xyz, radius, min_points, max_points):
    tree = cKDTree(xyz)
    pairs = tree.query_pairs(radius, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs), dtype=np.uint8), (pairs[:, 0], pairs[:, 1])), shape=(len(xyz), len(xyz))).tocsr()
    _, labels = connected_components(graph, directed=False)
    sizes = np.bincount(labels)
    valid_ids = np.flatnonzero((sizes >= min_points) & (sizes <= max_points))
    return labels, sizes, valid_ids


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', required=True, type=Path)
    p.add_argument('--keyframes', required=True, type=Path)
    p.add_argument('--trajectory', required=True, type=Path)
    p.add_argument('--timestamps', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--radius', type=float, default=.18)
    p.add_argument('--min-points', type=int, default=10)
    p.add_argument('--max-points', type=int, default=2000)
    p.add_argument('--sample-count', type=int, default=40)
    p.add_argument('--angular-resolution', type=float, default=.01)
    p.add_argument('--range-margin', type=float, default=.4)
    p.add_argument('--max-range', type=float, default=30.)
    p.add_argument('--min-valid-rays', type=int, default=3)
    p.add_argument('--minimum-coverage', type=float, default=.2)
    p.add_argument('--free-fraction', type=float, default=.65)
    p.add_argument('--occupied-fraction', type=float, default=.5)
    p.add_argument('--min-occupied-frames', type=int, default=2)
    p.add_argument('--min-free-frames', type=int, default=3)
    p.add_argument('--separation', type=float, default=1.)
    p.add_argument('--temporal-mode', choices=['forward', 'bidirectional'], default='forward')
    p.add_argument('--min-viewpoint-separation', type=float, default=.5)
    p.add_argument('--min-evidence-time-gap', type=float, default=2.)
    p.add_argument('--display-max-z', type=float, default=1.5)
    a = p.parse_args()
    if not (a.radius > 0 and 1 <= a.min_points <= a.max_points and a.sample_count >= a.min_valid_rays >= 1 and 0 < a.angular_resolution <= .1 and a.range_margin > 0 and a.max_range > 0 and 0 < a.minimum_coverage <= 1 and .5 < a.free_fraction <= 1 and 0 < a.occupied_fraction <= 1 and a.min_occupied_frames >= 1 and a.min_free_frames >= 1 and a.separation >= 0 and a.min_viewpoint_separation > 0 and a.min_evidence_time_gap > 0):
        p.error('invalid parameters')
    cloud = load_pcd(a.map)
    poses = np.loadtxt(a.trajectory, comments='#', ndmin=2)
    stamps = np.loadtxt(a.timestamps, ndmin=2)
    if poses.shape[1] != 8 or stamps.shape[1] != 2 or len(poses) != len(stamps) or not np.array_equal(stamps[:, 0], np.arange(len(stamps))) or not np.allclose(poses[:, 0], stamps[:, 1], atol=1e-3, rtol=0):
        raise ValueError('trajectory and timestamps mismatch')
    labels, sizes, ids = cluster_cloud(cloud[:, :3], a.radius, a.min_points, a.max_points)
    order = np.argsort(labels, kind='stable')
    offsets = np.r_[0, np.cumsum(sizes)]
    sample_index, sample_owner = [], []
    for n, cluster_id in enumerate(ids):
        indices = order[offsets[cluster_id]:offsets[cluster_id + 1]]
        selected = indices[np.linspace(0, len(indices)-1, min(a.sample_count, len(indices)), dtype=int)]
        sample_index.extend(selected)
        sample_owner.extend([n] * len(selected))
    sample_index = np.asarray(sample_index, dtype=int)
    owners = np.asarray(sample_owner, dtype=int)
    sample_sizes = np.bincount(owners, minlength=len(ids))
    occupied = [[] for _ in ids]
    free = [[] for _ in ids]
    occluded_counts = np.zeros(len(ids), dtype=int)
    if len(sample_index):
        world = cloud[sample_index, :3].astype(np.float64)
        chord_limit = 2 * np.sin(a.angular_resolution / 2)
        for i, pose in enumerate(poses):
            frame = load_pcd(a.keyframes / f'{i:06d}.pcd')
            returns = frame[:, :3].astype(np.float64)
            ranges = np.linalg.norm(returns, axis=1)
            usable = (ranges >= .1) & (ranges <= a.max_range)
            if not usable.any():
                continue
            returns, ranges = returns[usable], ranges[usable]
            rays = cKDTree(returns / ranges[:, None])
            local = Rotation.from_quat(pose[4:8]).inv().apply(world - pose[1:4])
            dist = np.linalg.norm(local, axis=1)
            valid = (dist >= .1) & (dist <= a.max_range)
            directions = np.divide(local, dist[:, None], out=np.zeros_like(local), where=dist[:, None] > 0)
            difference, ray = rays.query(directions, distance_upper_bound=chord_limit, workers=-1)
            valid &= np.isfinite(difference) & (ray < len(ranges))
            matched = np.zeros(len(dist))
            matched[valid] = ranges[ray[valid]]
            margin = a.range_margin + dist * a.angular_resolution
            nvalid = np.bincount(owners[valid], minlength=len(ids))
            nfree = np.bincount(owners[valid & (matched > dist + margin)], minlength=len(ids))
            nocc = np.bincount(owners[valid & (np.abs(matched - dist) <= margin)], minlength=len(ids))
            nblocked = np.bincount(owners[valid & (matched < dist - margin)], minlength=len(ids))
            enough = (nvalid >= a.min_valid_rays) & (nvalid >= np.ceil(sample_sizes * a.minimum_coverage))
            fmask = enough & (nfree >= a.free_fraction * nvalid)
            omask = enough & (nocc >= a.occupied_fraction * nvalid)
            for idx in np.flatnonzero(fmask): free[idx].append(float(pose[0]))
            for idx in np.flatnonzero(omask): occupied[idx].append(float(pose[0]))
            occluded_counts += enough & (nblocked > nvalid / 2)
    dynamic = np.zeros(len(cloud), dtype=bool)
    uncertain = np.zeros(len(cloud), dtype=bool)
    rows = []
    frame_evidence = []
    positions = {float(pose[0]): pose[1:4] for pose in poses}
    for n, cluster_id in enumerate(ids):
        indices = order[offsets[cluster_id]:offsets[cluster_id + 1]]
        cls = temporal_class(occupied[n], free[n], a.min_occupied_frames, a.min_free_frames, a.separation)
        pair = []
        forward_evidence = []
        if a.temporal_mode == 'bidirectional':
            cls, forward_evidence = separated_forward_class(occupied[n], free[n], positions, a.min_occupied_frames, a.min_free_frames, a.separation, a.min_viewpoint_separation, a.min_evidence_time_gap)
            bidir, pair = bidirectional_class(occupied[n], free[n], positions, a.min_occupied_frames, a.separation, a.min_viewpoint_separation, a.min_evidence_time_gap)
            if bidir == 'dynamic_candidate':
                cls = bidir
        (dynamic if cls == 'dynamic_candidate' else uncertain)[indices] = True
        centroid = cloud[indices, :3].mean(axis=0)
        rows.append({'cluster_id': int(cluster_id), 'points': len(indices), 'x': float(centroid[0]), 'y': float(centroid[1]), 'z': float(centroid[2]), 'occupied_frames': len(occupied[n]), 'free_frames': len(free[n]), 'occluded_frames': int(occluded_counts[n]), 'first_occupied': min(occupied[n]) if occupied[n] else '', 'last_free': max(free[n]) if free[n] else '', 'classification': cls})
        frame_evidence.append({'cluster_id': int(cluster_id), 'occupied_timestamps': occupied[n], 'free_timestamps': free[n], 'bidirectional_pair': pair, 'separated_forward_evidence': forward_evidence})
    a.output.mkdir(parents=True, exist_ok=True)
    for name, mask in [('proposed_kept', ~dynamic), ('dynamic_candidates', dynamic), ('uncertain_clusters', uncertain)]:
        write_pcd(a.output / f'{name}.pcd', cloud[mask])
        write_pcd(a.output / f'{name}_display.pcd', cloud[mask & (cloud[:, 2] <= a.display_max_z)])
    with (a.output / 'clusters.csv').open('w', newline='') as stream:
        fields = ['cluster_id', 'points', 'x', 'y', 'z', 'occupied_frames', 'free_frames', 'occluded_frames', 'first_occupied', 'last_free', 'classification']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    (a.output / 'frame_evidence.json').write_text(json.dumps(frame_evidence, indent=2) + '\n')
    params = {k: str(v.resolve()) if isinstance(v, Path) else v for k, v in vars(a).items()}
    report = {'input_points': len(cloud), 'frames': len(poses), 'reviewed_clusters': len(ids), 'reviewed_points': int((dynamic | uncertain).sum()), 'dynamic_candidate_clusters': sum(r['classification'] == 'dynamic_candidate' for r in rows), 'dynamic_candidate_points': int(dynamic.sum()), 'uncertain_cluster_points': int(uncertain.sum()), 'unreviewed_points': int((~(dynamic | uncertain)).sum()), 'parameters': params, 'warning': 'Approximate ray matching and cluster connectivity can misclassify. Proposed deletion is experimental and must be inspected against raw frames; unreviewed and uncertain points are retained.'}
    (a.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

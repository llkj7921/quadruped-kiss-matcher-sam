#!/usr/bin/env python3
"""Reconstruct source-return provenance; compare label order without masking observations."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from multiframe_outlier_review import load_pcd, write_pcd
from capture_visibility_evidence import sha256_files, source_hashes


def voxel_keys(xyz, voxel):
    # PCL VoxelGrid uses float point coordinates and inverse leaf size.
    cells = np.floor(np.asarray(xyz[:, :3], np.float32) * np.float32(1. / voxel)).astype('<i8')
    return np.ascontiguousarray(cells).view(np.dtype((np.void, 24))).ravel()


def decide_orders(point_labels, cluster_labels):
    """Fixed-label monotone union: no stage alters raw observations."""
    point_then_cluster = point_labels.copy()
    point_then_cluster |= cluster_labels
    cluster_then_point = cluster_labels.copy()
    cluster_then_point |= point_labels
    return point_then_cluster, cluster_then_point


def voxelized(points, voxel):
    if not len(points):
        return np.empty((0, 4), dtype=np.float32)
    keys = voxel_keys(points, voxel)
    _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    sums = np.column_stack([np.bincount(inverse, weights=points[:, i], minlength=len(counts)) for i in range(4)])
    return np.asarray(sums / counts[:, None], dtype=np.float32)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', required=True, type=Path)
    p.add_argument('--keyframes', required=True, type=Path)
    p.add_argument('--trajectory', required=True, type=Path)
    p.add_argument('--timestamps', required=True, type=Path)
    p.add_argument('--point-evidence', required=True, type=Path)
    p.add_argument('--cluster-review', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--voxel', type=float, default=.05)
    p.add_argument('--display-max-z', type=float, default=1.5)
    a = p.parse_args()
    if a.voxel <= 0 or a.output.exists():
        p.error('voxel must be positive and output must not already exist')
    manifest = json.loads((a.point_evidence.parent / 'manifest.json').read_text())
    if (Path(manifest['session']).resolve() != a.keyframes.parent.resolve() or
        manifest['source_hashes'] != source_hashes(a.keyframes.parent) or
        manifest['evidence_sha256_name_and_bytes'] != sha256_files([a.point_evidence])):
        raise ValueError('point evidence provenance does not match the unchanged input session')
    group_report = json.loads((a.cluster_review / 'report.json').read_text())
    params = group_report['parameters']
    if any(Path(params[k]).resolve() != path.resolve() for k, path in [('map', a.map), ('keyframes', a.keyframes), ('trajectory', a.trajectory), ('timestamps', a.timestamps)]):
        raise ValueError('cluster review product does not match inputs')
    raw = load_pcd(a.map)
    poses = np.loadtxt(a.trajectory, comments='#', ndmin=2)
    stamps = np.loadtxt(a.timestamps, ndmin=2)
    if poses.shape[1] != 8 or stamps.shape[1] != 2 or len(poses) != len(stamps) or not np.array_equal(stamps[:, 0], np.arange(len(stamps))) or not np.allclose(poses[:, 0], stamps[:, 1], atol=.001, rtol=0):
        raise ValueError('trajectory and timestamps mismatch')
    clouds, offsets = [], [0]
    for i, pose in enumerate(poses):
        frame = load_pcd(a.keyframes / f'{i:06d}.pcd')
        world = (Rotation.from_quat(pose[4:8]).apply(frame[:, :3]) + pose[1:4]).astype(np.float32)
        clouds.append(np.column_stack((world, frame[:, 3])))
        offsets.append(offsets[-1] + len(frame))
    world = np.concatenate(clouds)
    offsets = np.asarray(offsets, dtype=np.int64)
    evidence = np.loadtxt(a.point_evidence, dtype=np.uint32, comments='#', ndmin=2)
    expected_frame = np.repeat(np.arange(len(poses), dtype=np.uint32), np.diff(offsets))
    expected_index = np.arange(len(world), dtype=np.uint32) - np.repeat(offsets[:-1], np.diff(offsets)).astype(np.uint32)
    if evidence.shape != (len(world), 5) or not np.array_equal(evidence[:, 0], expected_frame) or not np.array_equal(evidence[:, 1], expected_index):
        raise ValueError('point evidence does not match keyframe point ordering')
    map_keys = voxel_keys(raw, a.voxel)
    if len(np.unique(map_keys)) != len(map_keys):
        raise ValueError('raw map contains duplicate voxel keys; mapping is ambiguous')
    sorter = np.argsort(map_keys)
    sorted_keys = map_keys[sorter]
    source_keys = voxel_keys(world, a.voxel)
    loc = np.searchsorted(sorted_keys, source_keys)
    matched = loc < len(sorted_keys)
    matched[matched] &= sorted_keys[loc[matched]] == source_keys[matched]
    source_voxel = np.full(len(world), -1, dtype=np.int32)
    source_voxel[matched] = sorter[loc[matched]]
    if not matched.all():
        raise ValueError(f'{int((~matched).sum())} keyframe returns cannot be mapped to saved raw voxels; refusing approximate matching')
    counts = np.bincount(source_voxel, minlength=len(raw))
    if np.any(counts == 0):
        raise ValueError(f'{int((counts == 0).sum())} saved map voxels have no mapped source')
    centroid = np.column_stack([np.bincount(source_voxel, weights=world[:, i], minlength=len(raw))/counts for i in range(3)])
    error = np.linalg.norm(centroid - raw[:, :3], axis=1)
    if error.max(initial=0) > 1e-3:
        raise ValueError(f'raw map centroids disagree with provenance: max={error.max()}')
    cluster = load_pcd(a.cluster_review / 'dynamic_candidates.pcd')
    group_keys = voxel_keys(cluster, a.voxel)
    group_map = np.isin(map_keys, group_keys)
    if int(group_map.sum()) != len(cluster):
        raise ValueError('cluster candidates cannot be mapped to raw map without ambiguity')
    free, occupied, occluded = evidence[:, 2], evidence[:, 3], evidence[:, 4]
    # Reuse the point-level filter's existing threshold and keep cluster-only
    # proposals conservative at original-return level. Occupied/free are
    # approximate and correlated; labels remain candidates, not truth.
    point_candidate = (free >= 3) & (free >= 2 * np.maximum(1, occupied))
    group_membership = group_map[source_voxel]
    cluster_candidate = group_membership & (free >= 1) & (free > occupied)
    first, second = decide_orders(point_candidate, cluster_candidate)
    if not np.array_equal(first, second):
        raise AssertionError('order-specific label results diverged')
    provenance_sort = np.argsort(source_voxel, kind='stable')
    voxel_offsets = np.r_[0, np.cumsum(counts, dtype=np.int64)]
    a.output.mkdir(parents=True)
    np.savez_compressed(a.output / 'source_provenance.npz', frame_offsets=offsets, source_voxel=source_voxel, voxel_offsets=voxel_offsets, voxel_source_indices=provenance_sort, point_candidate=point_candidate, cluster_membership=group_membership, cluster_candidate=cluster_candidate, free=free, occupied=occupied, occluded=occluded)
    # A separate query product for a label-aware clustering experiment. It is
    # not substituted for keyframes or their ray evidence.
    write_pcd(a.output / 'point_stage_query.pcd', voxelized(world[~point_candidate], a.voxel))
    final = voxelized(world[~first], a.voxel)
    final_reverse = voxelized(world[~second], a.voxel)
    if not np.array_equal(final, final_reverse):
        raise AssertionError('equal return labels produced different voxel maps')
    write_pcd(a.output / 'kept.pcd', final)
    write_pcd(a.output / 'point_then_cluster.pcd', final)
    write_pcd(a.output / 'cluster_then_point.pcd', final_reverse)
    write_pcd(a.output / 'kept_display.pcd', final[final[:, 2] <= a.display_max_z])
    removed = voxelized(world[first], a.voxel)
    write_pcd(a.output / 'removed_return_voxels.pcd', removed)
    # Preserve retained group members without sufficient per-return deletion
    # evidence from the final outlier filter, using exact output map points.
    protected_keys = np.unique(source_keys[group_membership & ~first])
    protected = final[np.isin(voxel_keys(final, a.voxel), protected_keys)]
    write_pcd(a.output / 'uncertain_group_protected.pcd', protected)
    # Removed and retained voxels can overlap when source labels disagree.
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    report = {'map_points': len(raw), 'keyframe_returns': len(world), 'frames': len(poses), 'mapped_returns': int(matched.sum()), 'maximum_centroid_error_m': float(error.max(initial=0)), 'point_candidate_returns': int(point_candidate.sum()), 'cluster_membership_returns': int(group_membership.sum()), 'cluster_candidate_returns': int(cluster_candidate.sum()), 'candidate_overlap_returns': int((point_candidate & cluster_candidate).sum()), 'removed_returns': int(first.sum()), 'retained_returns': int((~first).sum()), 'output_voxels': len(final), 'removed_return_voxels': len(removed), 'protected_group_voxels': len(protected), 'ordering_differences': int(np.count_nonzero(first != second)), 'inputs': {k: str(v.resolve()) for k, v in vars(a).items() if isinstance(v, Path)}, 'source_hashes': manifest['source_hashes'], 'evidence_sha256': digest(a.point_evidence), 'map_sha256': digest(a.map), 'cluster_candidates_sha256': digest(a.cluster_review / 'dynamic_candidates.pcd'), 'voxel': a.voxel, 'policy': 'fixed source labels; monotone union; cluster member additionally requires free>=1 and free>occupied', 'warning': 'Provenance is reconstructed from saved keyframes and optimized poses, not captured during original SLAM. It is validated against saved map centroids but floating-point boundaries may still matter. A cluster voxel may contain static and dynamic returns. Point and cluster labels use correlated observations, and cluster refinement also uses point votes. Both label orders share fixed evidence and a commutative policy; equality is expected and does not prove map accuracy.'}
    (a.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

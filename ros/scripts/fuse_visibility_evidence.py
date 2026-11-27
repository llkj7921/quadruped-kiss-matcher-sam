#!/usr/bin/env python3
"""Conservative offline evidence fusion; experimental, not a ground-truth labeler."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from multiframe_outlier_review import load_pcd, write_pcd


def sha256_files(paths):
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.name.encode('utf-8') + b'\0')
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1048576), b''):
                digest.update(block)
    return digest.hexdigest()


def voxel_keys(points, leaf):
    cells = np.floor(np.asarray(points[:, :3], dtype=np.float32) / np.float32(leaf)).astype('<i8')
    return np.ascontiguousarray(cells).view(np.dtype((np.void, 24))).ravel()


def decide(point_prior, cluster_prior, free_views, occupied_views, fine_free, coarse_free, occupied_span):
    """Use both priors without treating occupation alone as proven static."""
    considered = point_prior | cluster_prior
    dynamic = considered & (free_views >= 2) & (fine_free >= 2) & (coarse_free >= 2) & (free_views > occupied_views)
    restored = considered & ~dynamic & (free_views == 0) & (occupied_views >= 3) & (occupied_span >= 20.)
    uncertain = considered & ~dynamic & ~restored
    return dynamic, restored, uncertain


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', required=True, type=Path)
    p.add_argument('--point-candidates', required=True, type=Path)
    p.add_argument('--cluster-candidates', required=True, type=Path)
    p.add_argument('--keyframes', required=True, type=Path)
    p.add_argument('--trajectory', required=True, type=Path)
    p.add_argument('--timestamps', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--voxel', type=float, default=.05)
    p.add_argument('--fine-angle', type=float, default=.01)
    p.add_argument('--coarse-angle', type=float, default=.02)
    p.add_argument('--range-margin', type=float, default=.4)
    p.add_argument('--max-range', type=float, default=30.)
    p.add_argument('--view-distance', type=float, default=.5)
    p.add_argument('--view-time', type=float, default=2.)
    p.add_argument('--display-max-z', type=float, default=1.5)
    a = p.parse_args()
    if not (a.voxel > 0 and 0 < a.fine_angle < a.coarse_angle <= .1 and a.range_margin > 0 and a.max_range > 0 and a.view_distance > 0 and a.view_time > 0):
        p.error('invalid parameters')
    if a.output.exists():
        raise ValueError('output already exists; choose a new directory')
    point_report = yaml.safe_load((a.point_candidates.parent / 'report.yaml').read_text())
    cluster_report = json.loads((a.cluster_candidates.parent / 'report.json').read_text())
    cluster_params = cluster_report['parameters']
    session = Path(point_report['input_session']).resolve()
    if ((session / 'keyframes').resolve() != a.keyframes.resolve() or
        (session / 'optimized_trajectory_tum.txt').resolve() != a.trajectory.resolve() or
        (session / 'timestamps.txt').resolve() != a.timestamps.resolve() or
        not np.isclose(float(point_report['output_voxel_resolution_m']), a.voxel) or
        Path(cluster_params['map']).resolve() != a.map.resolve() or
        Path(cluster_params['keyframes']).resolve() != a.keyframes.resolve() or
        Path(cluster_params['trajectory']).resolve() != a.trajectory.resolve() or
        Path(cluster_params['timestamps']).resolve() != a.timestamps.resolve()):
        raise ValueError('candidate provenance does not match map product, trajectory or voxel resolution')
    cloud = load_pcd(a.map)
    map_keys = voxel_keys(cloud, a.voxel)
    point = np.isin(map_keys, voxel_keys(load_pcd(a.point_candidates), a.voxel))
    cluster = np.isin(map_keys, voxel_keys(load_pcd(a.cluster_candidates), a.voxel))
    candidate_ids = np.flatnonzero(point | cluster)
    if not len(candidate_ids):
        raise ValueError('no candidate voxels matched input map')
    world = cloud[candidate_ids, :3].astype(float)
    poses = np.loadtxt(a.trajectory, comments='#', ndmin=2)
    stamps = np.loadtxt(a.timestamps, ndmin=2)
    if poses.shape[1] != 8 or stamps.shape[1] != 2 or len(poses) != len(stamps) or not np.array_equal(stamps[:, 0], np.arange(len(stamps))) or not np.allclose(poses[:, 0], stamps[:, 1], atol=1e-3, rtol=0):
        raise ValueError('trajectory and timestamps mismatch')
    count = len(world)
    free_views = np.zeros(count, np.uint16)
    occ_views = np.zeros(count, np.uint16)
    fine_free = np.zeros(count, np.uint16)
    coarse_free = np.zeros(count, np.uint16)
    blocked = np.zeros(count, np.uint16)
    unmatched = np.zeros(count, np.uint16)
    last_free = np.full((count, 3), np.nan)
    last_occ = np.full((count, 3), np.nan)
    free_time = np.full(count, -np.inf)
    occ_time = np.full(count, -np.inf)
    first_occ = np.full(count, np.nan)
    last_occ_stamp = np.full(count, np.nan)
    for i, pose in enumerate(poses):
        frame = load_pcd(a.keyframes / f'{i:06d}.pcd')[:, :3].astype(float)
        ranges = np.linalg.norm(frame, axis=1)
        good = (ranges >= .1) & (ranges <= a.max_range)
        frame, ranges = frame[good], ranges[good]
        if not len(frame):
            continue
        rays = cKDTree(frame / ranges[:, None])
        local = Rotation.from_quat(pose[4:8]).inv().apply(world - pose[1:4])
        distance = np.linalg.norm(local, axis=1)
        direction = np.divide(local, distance[:, None], out=np.zeros_like(local), where=distance[:, None] > 0)
        delta, index = rays.query(direction, distance_upper_bound=2*np.sin(a.coarse_angle/2), workers=-1)
        in_range = (distance >= .1) & (distance <= a.max_range)
        matched = in_range & np.isfinite(delta) & (index < len(ranges))
        unmatched += in_range & ~matched
        fine = matched & (delta <= 2*np.sin(a.fine_angle/2))
        measured = np.zeros(count)
        measured[matched] = ranges[index[matched]]
        fine_margin = a.range_margin + distance*a.fine_angle
        coarse_margin = a.range_margin + distance*a.coarse_angle
        is_free = fine & (measured > distance + fine_margin)
        is_occupied = fine & (np.abs(measured-distance) <= fine_margin)
        is_coarse_free = matched & (measured > distance + coarse_margin)
        blocked += fine & (measured < distance - fine_margin)
        fine_free += is_free
        coarse_free += is_coarse_free
        def accept(mask, last_pos, last_t):
            separation = np.linalg.norm(last_pos-pose[1:4], axis=1)
            chosen = mask & ((~np.isfinite(last_t)) | ((pose[0]-last_t >= a.view_time) & (separation >= a.view_distance)))
            last_pos[chosen] = pose[1:4]
            last_t[chosen] = pose[0]
            return chosen
        new_free = accept(is_free, last_free, free_time)
        new_occ = accept(is_occupied, last_occ, occ_time)
        free_views += new_free
        occ_views += new_occ
        first_occ[new_occ & ~np.isfinite(first_occ)] = pose[0]
        last_occ_stamp[new_occ] = pose[0]
    span = np.nan_to_num(last_occ_stamp-first_occ, nan=0.)
    dynamic, restored, uncertain = decide(point[candidate_ids], cluster[candidate_ids], free_views, occ_views, fine_free, coarse_free, span)
    flags = np.zeros(len(cloud), dtype=bool)
    flags[candidate_ids[dynamic]] = True
    static_ids = candidate_ids[restored]
    unknown_ids = candidate_ids[uncertain]
    a.output.mkdir(parents=True)
    products = [('kept', cloud[~flags]), ('dynamic_candidates', cloud[flags]), ('restored_static_candidates', cloud[static_ids]), ('uncertain_candidates', cloud[unknown_ids])]
    for name, points in products:
        write_pcd(a.output / f'{name}.pcd', points)
        write_pcd(a.output / f'{name}_display.pcd', points[points[:, 2] <= a.display_max_z])
    np.savez_compressed(a.output / 'evidence.npz', candidate_indices=candidate_ids, point_prior=point[candidate_ids], cluster_prior=cluster[candidate_ids], free_views=free_views, occupied_views=occ_views, fine_free=fine_free, coarse_free=coarse_free, occluded=blocked, unmatched=unmatched, occupied_span=span, dynamic=dynamic, restored=restored, uncertain=uncertain)
    params = {k: str(v.resolve()) if isinstance(v, Path) else v for k, v in vars(a).items()}
    hashes = {name: sha256_files([path]) for name, path in [('map', a.map), ('point_candidates', a.point_candidates), ('cluster_candidates', a.cluster_candidates), ('trajectory', a.trajectory), ('timestamps', a.timestamps)]}
    hashes['keyframe_sequence'] = sha256_files([a.keyframes / f'{i:06d}.pcd' for i in range(len(poses))])
    report = {'input': len(cloud), 'point_prior': int(point.sum()), 'cluster_prior': int(cluster.sum()), 'prior_overlap': int((point & cluster).sum()), 'candidate_union': count, 'dynamic': int(dynamic.sum()), 'restored_static_candidates': int(restored.sum()), 'uncertain_retained': int(uncertain.sum()), 'kept': int((~flags).sum()), 'params': params, 'input_sha256_name_and_bytes': hashes, 'warning': 'Experimental; voxel association and nearest rays are approximate. Fine and coarse ray evidence is correlated. View grouping is greedy, not statistical independence; static recovery can preserve a stationary person. No labels are ground truth.'}
    (a.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

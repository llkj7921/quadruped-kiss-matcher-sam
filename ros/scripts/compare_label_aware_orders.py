#!/usr/bin/env python3
"""Compare label-aware cluster query order using unchanged source observations."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from capture_visibility_evidence import source_hashes
from multiframe_outlier_review import load_pcd, write_pcd
from provenance_order_experiment import voxel_keys, voxelized


def labels(point, group_raw, group_after_point, free, occupied):
    cluster_first = point | (group_raw & (free >= 1) & (free > occupied))
    point_first = point | (group_after_point & (free >= 1) & (free > occupied))
    return point_first, cluster_first


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--provenance', required=True, type=Path)
    p.add_argument('--group-after-point', required=True, type=Path)
    p.add_argument('--group-before-point', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    if a.output.exists():
        p.error('output directory exists')
    report = json.loads((a.provenance / 'report.json').read_text())
    inp = {k: Path(v) for k, v in report['inputs'].items()}
    if hashlib.sha256(inp['map'].read_bytes()).hexdigest() != report['map_sha256'] or source_hashes(inp['keyframes'].parent) != report['source_hashes']:
        raise ValueError('original inputs differ from the provenance snapshot')
    raw_report = json.loads((a.group_before_point / 'report.json').read_text())
    post_report = json.loads((a.group_after_point / 'report.json').read_text())
    raw_cfg, post_cfg = raw_report['parameters'], post_report['parameters']
    if Path(raw_cfg['map']).resolve() != inp['map'].resolve() or Path(post_cfg['map']).resolve() != (a.provenance / 'point_stage_query.pcd').resolve():
        raise ValueError('cluster maps do not match expected stages')
    for key in raw_cfg:
        if key not in ('map', 'output') and raw_cfg[key] != post_cfg.get(key):
            raise ValueError(f'cluster parameter/input differs between orders: {key}')
    raw = load_pcd(inp['map'])
    data = np.load(a.provenance / 'source_provenance.npz')
    source_voxel, point = data['source_voxel'], data['point_candidate']
    free, occupied = data['free'], data['occupied']
    raw_membership = data['cluster_membership']
    if hashlib.sha256((a.group_before_point / 'dynamic_candidates.pcd').read_bytes()).hexdigest() != report['cluster_candidates_sha256']:
        raise ValueError('group-before candidate file differs from provenance snapshot')
    voxel = float(report['voxel'])
    raw_keys = voxel_keys(raw, voxel)
    after = load_pcd(a.group_after_point / 'dynamic_candidates.pcd')
    after_keys = voxel_keys(after, voxel)
    if not np.isin(after_keys, raw_keys).all():
        raise ValueError('group-after candidate contains an unknown source voxel')
    after_membership = np.isin(raw_keys[source_voxel], after_keys)
    point_first, cluster_first = labels(point, raw_membership, after_membership, free, occupied)
    poses = np.loadtxt(inp['trajectory'], comments='#', ndmin=2)
    world = []
    for i, pose in enumerate(poses):
        frame = load_pcd(inp['keyframes'] / f'{i:06d}.pcd')
        xyz = (Rotation.from_quat(pose[4:8]).apply(frame[:, :3]) + pose[1:4]).astype(np.float32)
        world.append(np.column_stack((xyz, frame[:, 3])))
    world = np.concatenate(world)
    if len(world) != len(point):
        raise ValueError('source point count differs from sidecar')
    if not np.array_equal(voxelized(world[~point], voxel), load_pcd(a.provenance / 'point_stage_query.pcd')):
        raise ValueError('point-stage query does not match the source labels')
    a.output.mkdir(parents=True)
    result = {}
    for name, selected, membership in [('point_first', point_first, after_membership), ('cluster_first', cluster_first, raw_membership)]:
        retained = voxelized(world[~selected], voxel)
        removed = voxelized(world[selected], voxel)
        protected_keys = np.unique(voxel_keys(world[membership & ~selected], voxel))
        protected = retained[np.isin(voxel_keys(retained, voxel), protected_keys)]
        for suffix, values in [('kept', retained), ('removed_return_voxels', removed), ('uncertain_group_protected', protected)]:
            write_pcd(a.output / f'{name}_{suffix}.pcd', values)
        result[name] = {'candidate_returns': int(selected.sum()), 'retained_return_count': int((~selected).sum()), 'kept_voxels': len(retained), 'protected_voxels': len(protected), 'candidate_group_source_returns': int(membership.sum())}
    np.savez_compressed(a.output / 'order_labels.npz', point_first=point_first, cluster_first=cluster_first, point_candidates=point, group_before=raw_membership, group_after=after_membership)
    result.update(different_source_labels=int(np.count_nonzero(point_first != cluster_first)), point_only=int((point_first & ~cluster_first).sum()), cluster_only=int((cluster_first & ~point_first).sum()), warning='Both orders use the original unmodified keyframes/rays and the same parameters. The point-first cluster query excludes point-labelled returns but does not remove their observation evidence. Grouping changes with the query. Candidate source labels are approximate, not semantic truth.')
    result['input_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (a.provenance / 'point_stage_query.pcd', a.group_before_point / 'dynamic_candidates.pcd', a.group_after_point / 'dynamic_candidates.pcd')}
    (a.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

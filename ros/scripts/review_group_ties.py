#!/usr/bin/env python3
"""Experimental source-level tie handling for pre-reviewed dynamic groups."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multiframe_outlier_review import load_pcd, write_pcd
from provenance_order_experiment import voxel_keys, voxelized
from capture_visibility_evidence import source_hashes


def tie_candidates(group, free, occupied, occluded, minimum_free=2, minimum_occluded=5):
    """Promote only tied, repeatedly observed and heavily occluded group returns."""
    return group & (free >= minimum_free) & (free == occupied) & (occluded >= minimum_occluded)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--provenance', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('output exists')
    report = json.loads((a.provenance / 'report.json').read_text())
    inp = {k: Path(v) for k, v in report['inputs'].items()}
    if source_hashes(inp['keyframes'].parent) != report['source_hashes']:
        raise ValueError('source snapshot changed')
    data = np.load(a.provenance / 'source_provenance.npz')
    free, occupied, occluded = data['free'], data['occupied'], data['occluded']
    group = data['cluster_membership']
    base = data['point_candidate'] | data['cluster_candidate']
    added = tie_candidates(group, free, occupied, occluded) & ~base
    selected = base | added
    poses = np.loadtxt(inp['trajectory'], comments='#', ndmin=2)
    clouds = []
    for i, pose in enumerate(poses):
        frame = load_pcd(inp['keyframes'] / f'{i:06d}.pcd')
        xyz = (Rotation.from_quat(pose[4:8]).apply(frame[:, :3]) + pose[1:4]).astype(np.float32)
        clouds.append(np.column_stack((xyz, frame[:, 3])))
    world = np.concatenate(clouds)
    if len(world) != len(selected):
        raise ValueError('source count mismatch')
    voxel = float(report['voxel'])
    retained = voxelized(world[~selected], voxel)
    uncertain = np.unique(voxel_keys(world[group & ~selected], voxel))
    protected = retained[np.isin(voxel_keys(retained, voxel), uncertain)]
    a.output.mkdir(parents=True)
    for name, points in [('kept', retained), ('added_removed_return_voxels', voxelized(world[added], voxel)), ('uncertain_group_protected', protected)]:
        write_pcd(a.output / (name + '.pcd'), points)
    np.savez_compressed(a.output / 'labels.npz', added=added, selected=selected)
    result = {'source_returns_added_to_candidates': int(added.sum()), 'retained_voxels': len(retained), 'protected_voxels': len(protected), 'rule': 'existing dynamic-candidate group and free == occupied >= 2 and occluded >= 5', 'warning': 'Experimental correlated evidence, occlusion is not proof of motion. Review all added candidates for static structure before use.'}
    (a.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Conservative offline candidates; keyframe support is not ground truth."""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def load_pcd(path):
    with Path(path).open('rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f'incomplete PCD: {path}')
            text = line.decode('ascii').strip()
            header.append(text)
            if text.startswith('DATA '):
                break
        meta = {line.split()[0]: line.split()[1:] for line in header if line and not line.startswith('#')}
        if meta['FIELDS'] != ['x', 'y', 'z', 'intensity'] or meta['SIZE'] != ['4'] * 4 or meta['TYPE'] != ['F'] * 4 or meta.get('COUNT', ['1'] * 4) != ['1'] * 4:
            raise ValueError(f'unsupported PCD fields: {path}')
        count = int(meta['POINTS'][0])
        if meta['DATA'][0] == 'binary':
            cloud = np.frombuffer(stream.read(count * 16), dtype='<f4').reshape(-1, 4).copy()
        elif meta['DATA'][0] == 'ascii':
            cloud = np.loadtxt(stream, dtype=np.float32, ndmin=2)
        elif meta['DATA'][0] == 'binary_compressed':
            with tempfile.TemporaryDirectory(prefix='outlier_pcd_') as temporary:
                converted = Path(temporary) / 'converted.pcd'
                subprocess.run(['pcl_convert_pcd_ascii_binary', str(path), str(converted), '1'], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return load_pcd(converted)
        else:
            raise ValueError(f'unsupported PCD encoding: {path}')
    if cloud.shape != (count, 4) or not np.isfinite(cloud).all():
        raise ValueError(f'invalid PCD data: {path}')
    return cloud


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 4)
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(points), len(points)))
    with Path(path).open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())


def classify(xyz, support, neighbors=10, factor=2.5, floor=0.25, max_support=1):
    if len(xyz) <= neighbors:
        raise ValueError('not enough points for neighborhood analysis')
    tree = cKDTree(xyz)
    distances, indices = tree.query(xyz, k=neighbors + 1, workers=-1)
    spacing = distances[:, 1:].mean(axis=1)
    local = np.median(spacing[indices[:, 1:]], axis=1)
    candidate = (support <= max_support) & (spacing > floor) & (spacing > factor * local)
    return candidate, spacing, local


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--keyframes', required=True, type=Path)
    parser.add_argument('--trajectory', required=True, type=Path)
    parser.add_argument('--timestamps', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--support-radius', type=float, default=0.12)
    parser.add_argument('--neighbors', type=int, default=10)
    parser.add_argument('--local-factor', type=float, default=2.5)
    parser.add_argument('--minimum-spacing', type=float, default=0.25)
    parser.add_argument('--maximum-support', type=int, default=1)
    parser.add_argument('--display-max-z', type=float, default=1.5)
    args = parser.parse_args()
    if not (args.support_radius > 0 and args.neighbors >= 1 and args.local_factor > 1 and args.minimum_spacing > 0 and args.maximum_support >= 0):
        parser.error('invalid filter parameters')
    cloud = load_pcd(args.map)
    poses = np.loadtxt(args.trajectory, comments='#', ndmin=2)
    stamps = np.loadtxt(args.timestamps, ndmin=2)
    if poses.shape[1] != 8 or stamps.shape[1] != 2 or len(poses) != len(stamps) or not np.array_equal(stamps[:, 0], np.arange(len(stamps))) or not np.allclose(poses[:, 0], stamps[:, 1], atol=1e-3, rtol=0):
        raise ValueError('trajectory and keyframe timestamps do not match')
    tree = cKDTree(cloud[:, :3])
    support = np.zeros(len(cloud), dtype=np.uint32)
    for i, pose in enumerate(poses):
        frame = load_pcd(args.keyframes / f'{i:06d}.pcd')
        world = Rotation.from_quat(pose[4:8]).apply(frame[:, :3]) + pose[1:4]
        # Each keyframe votes at most once per map point, even with many nearby returns.
        matches = tree.query_ball_point(world, args.support_radius, workers=-1, return_sorted=False)
        nonempty = [np.asarray(x, dtype=np.int64) for x in matches if len(x)]
        if nonempty:
            support[np.unique(np.concatenate(nonempty))] += 1
    candidate, spacing, local = classify(cloud[:, :3], support, args.neighbors, args.local_factor, args.minimum_spacing, args.maximum_support)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, mask in [('kept', ~candidate), ('removed_candidates', candidate)]:
        write_pcd(args.output / f'{name}.pcd', cloud[mask])
        write_pcd(args.output / f'{name}_display.pcd', cloud[mask & (cloud[:, 2] <= args.display_max_z)])
    np.savez_compressed(args.output / 'evidence.npz', support=support, spacing=spacing, local_spacing=local, candidate=candidate)
    report = {'map': str(args.map.resolve()), 'keyframes': len(poses), 'points': len(cloud), 'kept': int((~candidate).sum()), 'candidates': int(candidate.sum()), 'support_radius_m': args.support_radius, 'neighbors': args.neighbors, 'local_factor': args.local_factor, 'minimum_spacing_m': args.minimum_spacing, 'maximum_support': args.maximum_support, 'display_max_z': args.display_max_z, 'support_zero': int((support == 0).sum()), 'support_one': int((support == 1).sum()), 'note': 'Candidate classification is not ground truth; zero support can reflect registration or voxel mismatch.'}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

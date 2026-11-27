#!/usr/bin/env python3
"""Review sampled angular-ray matches for one cluster in selected keyframes."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from cluster_visibility_review import cluster_cloud
from multiframe_outlier_review import load_pcd

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--map', required=True, type=Path)
p.add_argument('--keyframes', required=True, type=Path)
p.add_argument('--trajectory', required=True, type=Path)
p.add_argument('--review', required=True, type=Path)
p.add_argument('--cluster-id', required=True, type=int)
p.add_argument('--frames', required=True, type=int, nargs='+')
a = p.parse_args()
params = json.loads((a.review / 'report.json').read_text())['parameters']
if Path(params['map']).resolve() != a.map.resolve():
    raise ValueError('input map does not match review')
cloud = load_pcd(a.map)
poses = np.loadtxt(a.trajectory, comments='#', ndmin=2)
labels, sizes, ids = cluster_cloud(cloud[:, :3], params['radius'], params['min_points'], params['max_points'])
if a.cluster_id not in ids:
    raise ValueError('cluster not reviewed')
indices = np.flatnonzero(labels == a.cluster_id)
indices = indices[np.linspace(0, len(indices)-1, min(params['sample_count'], len(indices)), dtype=int)]
world = cloud[indices, :3].astype(float)
center = world.mean(axis=0)
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(1, len(a.frames), figsize=(4 * len(a.frames), 5), squeeze=False, constrained_layout=True)
stats = []
for ax, i in zip(axes[0], a.frames):
    pose = poses[i]
    frame = load_pcd(a.keyframes / f'{i:06d}.pcd')[:, :3].astype(float)
    ranges = np.linalg.norm(frame, axis=1)
    ok = (ranges >= .1) & (ranges <= params['max_range'])
    frame, ranges = frame[ok], ranges[ok]
    rotation = Rotation.from_quat(pose[4:8])
    local = rotation.inv().apply(world - pose[1:4])
    dist = np.linalg.norm(local, axis=1)
    direction = np.divide(local, dist[:, None], out=np.zeros_like(local), where=dist[:, None] > 0)
    diff, ray = cKDTree(frame / ranges[:, None]).query(direction, distance_upper_bound=2 * np.sin(params['angular_resolution'] / 2))
    valid = np.isfinite(diff) & (ray < len(ranges)) & (dist >= .1) & (dist <= params['max_range'])
    matched = np.zeros(len(dist)); matched[valid] = ranges[ray[valid]]
    margin = params['range_margin'] + dist * params['angular_resolution']
    free = valid & (matched > dist + margin)
    occupied = valid & (np.abs(matched-dist) <= margin)
    blocked = valid & (matched < dist-margin)
    world_frame = rotation.apply(frame) + pose[1:4]
    near = (np.abs(world_frame[:, 0]-center[0]) < 4) & (np.abs(world_frame[:, 1]-center[1]) < 5)
    ax.scatter(world_frame[near, 0], world_frame[near, 1], c='#b4b8bc', s=.4, rasterized=True)
    ax.scatter(world[:, 0], world[:, 1], c='#ff8f00', s=10, label='目标样本')
    if free.any():
        hit = world_frame[ray[free]]
        ax.scatter(hit[:, 0], hit[:, 1], c='#008ed1', s=12, label='判为空闲的命中')
        for target, end in zip(world[free], hit):
            ax.plot([target[0], end[0]], [target[1], end[1]], c='#008ed1', alpha=.4, lw=.5)
    ax.scatter([pose[1]], [pose[2]], marker='^', c='red', s=30, label='雷达位置')
    ax.set_xlim(center[0]-4, center[0]+4); ax.set_ylim(center[1]-5, center[1]+5); ax.set_aspect('equal')
    ax.set_title(f'关键帧 {i}\n匹配 {valid.sum()}/{len(world)}，空闲 {free.sum()}，占据 {occupied.sum()}'); ax.grid(alpha=.15)
    ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
    stats.append({'frame': i, 'relative_time_s': float(pose[0]-poses[0, 0]), 'pose_xyz': pose[1:4].tolist(), 'samples': len(world), 'matched': int(valid.sum()), 'free': int(free.sum()), 'occupied': int(occupied.sum()), 'occluded': int(blocked.sum()), 'free_hit_world': world_frame[ray[free]].tolist(), 'free_sample_world': world[free].tolist()})
axes[0, 0].legend(loc='best', fontsize=7)
fig.suptitle('点团样本射线俯视核查：匹配点不保证属于真实背景；灰色为关键帧点云', fontsize=12)
fig.savefig(a.review / f'点团{a.cluster_id}_关键帧射线检查.png', dpi=160)
(a.review / f'cluster_{a.cluster_id}_ray_inspection.json').write_text(json.dumps(stats, indent=2) + '\n')

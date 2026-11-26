#!/usr/bin/env python3
"""Render matched-height comparison of offline outlier experiments."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from scipy.spatial import cKDTree
from multiframe_outlier_review import load_pcd

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--map', type=Path, required=True)
p.add_argument('--sor', type=Path, required=True)
p.add_argument('--multiframe', type=Path, required=True)
p.add_argument('--adaptive', type=Path, required=True)
a = p.parse_args()
source = load_pcd(a.map)
outputs = [('原始输入', source, None), ('全局 SOR', load_pcd(a.sor / 'sor_kept.pcd'), load_pcd(a.sor / 'sor_removed.pcd')), ('多帧保守方案', load_pcd(a.multiframe / 'kept.pcd'), load_pcd(a.multiframe / 'removed_candidates.pcd')), ('局部自适应方案', load_pcd(a.adaptive / 'kept.pcd'), load_pcd(a.adaptive / 'adaptive_candidates.pcd'))]
tree = cKDTree(source[:, :3])
for name, kept, removed in outputs[1:]:
    if len(kept) + len(removed) != len(source):
        raise ValueError(f'{name}: counts do not match input')
    for points in (kept, removed):
        dist, _ = tree.query(points[:, :3], workers=-1)
        if np.max(dist, initial=0) > 1e-4:
            raise ValueError(f'{name}: not the same input')
region = lambda q: (q[:, 1] > 25) & (np.abs(q[:, 0]) < 10)
summary = {name: {'total': len(points), 'far_region': int(region(points).sum()), 'display_z_le_1_5': int((points[:, 2] <= 1.5).sum())} for name, points, _ in outputs}
summary['far_region_definition'] = 'y > 25 m and |x| < 10 m, all heights'
summary['warning'] = 'Quantities only; candidates are not ground truth. Small connected components are review only.'
(a.adaptive / 'comparison.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(summary, ensure_ascii=False, indent=2))
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(2, 2, figsize=(15, 12), constrained_layout=True)
for ax, (name, points, deleted) in zip(axes.flat, outputs):
    shown = points[points[:, 2] <= 1.5]
    ax.scatter(shown[:, 0], shown[:, 1], s=.18, c='#414951', rasterized=True)
    if deleted is not None:
        marked = deleted[deleted[:, 2] <= 1.5]
        ax.scatter(marked[:, 0], marked[:, 1], s=1.4, c='#dd4639', alpha=.75, label='删除候选', rasterized=True)
        ax.legend(loc='lower right', markerscale=4)
    ax.set_title(name); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
    ax.set_aspect('equal', adjustable='box'); ax.set_xlim(-5, 31); ax.set_ylim(-14, 36); ax.grid(alpha=.15)
fig.suptitle('离群点方案对比：俯视，地图坐标 z ≤ 1.5 米\n红点为删除候选而非真值；小连通块仅供单独检查', fontsize=15)
fig.savefig(a.adaptive / '自适应离群点对比.png', dpi=180)

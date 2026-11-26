#!/usr/bin/env python3
"""Compare one candidate result to a baseline; no ground-truth accuracy claims."""
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
p.add_argument('--input', required=True, type=Path)
p.add_argument('--sor-kept', required=True, type=Path)
p.add_argument('--sor-removed', required=True, type=Path)
p.add_argument('--review', required=True, type=Path)
args = p.parse_args()
source = load_pcd(args.input)
sor = load_pcd(args.sor_kept)
sor_removed = load_pcd(args.sor_removed)
kept = load_pcd(args.review / 'kept.pcd')
removed = load_pcd(args.review / 'removed_candidates.pcd')
dist, _ = cKDTree(source[:, :3]).query(kept[:, :3], k=1, workers=-1)
if len(kept) + len(removed) != len(source) or np.max(dist, initial=0) > 1e-4:
    raise ValueError('candidate output does not match input')
region = lambda p: (p[:, 1] > 25) & (np.abs(p[:, 0]) < 10)
report = {'input': len(source), 'sor_kept': len(sor), 'sor_removed': len(sor_removed), 'candidate_kept': len(kept), 'candidates_removed': len(removed), 'far_region_definition': 'y > 25 m and |x| < 10 m; all heights', 'far_input': int(region(source).sum()), 'far_sor_kept': int(region(sor).sum()), 'far_candidate_kept': int(region(kept).sum()), 'far_candidates': int(region(removed).sum()), 'warning': 'Point counts do not establish correctness; review removed and retained points.'}
(args.review / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(2, 2, figsize=(15, 12), constrained_layout=True)
for ax, points, title, deleted in [(axes[0, 0], source, '输入：可见性过滤后', None), (axes[0, 1], sor, '原 SOR：保留点', sor_removed), (axes[1, 0], kept, '多帧支持与局部密度：保留点', removed), (axes[1, 1], source, '远端局部：删除候选叠加', None)]:
    shown = points[points[:, 2] <= 1.5]
    ax.scatter(shown[:, 0], shown[:, 1], s=.16, c='#404850', rasterized=True)
    if deleted is not None:
        crop = deleted[deleted[:, 2] <= 1.5]
        ax.scatter(crop[:, 0], crop[:, 1], s=1, c='#e34b3e', alpha=.8, rasterized=True, label='删除候选')
        ax.legend(loc='lower right', markerscale=5)
    ax.set_title(title)
    ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
    ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.15)
ax = axes[1, 1]
for points, color, label in [(sor_removed, '#e34b3e', 'SOR 删除'), (removed, '#1380cc', '多帧方法删除')]:
    shown = points[(points[:, 2] <= 1.5) & region(points)]
    ax.scatter(shown[:, 0], shown[:, 1], s=4, c=color, label=label, rasterized=True)
ax.set_xlim(-10, 10); ax.set_ylim(25, max(30, float(source[:, 1].max()) + 1)); ax.legend(loc='upper right', markerscale=2)
fig.suptitle('离群点处理对比：俯视、地图坐标 z ≤ 1.5 米\n红蓝点为删除候选，非人工标注真值；远端统计使用完整高度', fontsize=15)
fig.savefig(args.review / '离群点过滤对比.png', dpi=180)

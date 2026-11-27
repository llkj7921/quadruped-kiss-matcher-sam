#!/usr/bin/env python3
"""Visualize fixed-label order and weak-outlier results; no accuracy claims."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from multiframe_outlier_review import load_pcd

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--raw', type=Path, required=True)
p.add_argument('--experiment', type=Path, required=True)
p.add_argument('--outlier', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
maps = [('原始融合图', load_pcd(a.raw)), ('先点标签→团标签', load_pcd(a.experiment / 'point_then_cluster.pcd')), ('先团标签→点标签', load_pcd(a.experiment / 'cluster_then_point.pcd')), ('共同结果 + 轻度离群', load_pcd(a.outlier / 'kept.pcd'))]
if not np.array_equal(maps[1][1], maps[2][1]):
    raise ValueError('point/group order maps are not identical')
far = lambda x: (x[:, 1] > 25) & (np.abs(x[:, 0]) < 10)
target = lambda x: (x[:, 0] >= 2.68) & (x[:, 0] <= 3.57) & (x[:, 1] >= 13.81) & (x[:, 1] <= 15.70) & (x[:, 2] >= -.80) & (x[:, 2] <= .69)
report = {name: {'all': len(x), 'display': int((x[:, 2] <= 1.5).sum()), 'far_all_heights': int(far(x).sum()), 'target_bbox_all_heights': int(target(x).sum())} for name, x in maps}
report['warning'] = 'Equal order results follow from the commutative fixed-label policy, not proof of accuracy. Target box can contain static points.'
if a.output.exists():
    raise ValueError('output directory already exists')
a.output.mkdir(parents=True)
(a.output / 'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(report, ensure_ascii=False, indent=2))
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(2, 4, figsize=(20, 10), constrained_layout=True)
for col, (name, points) in enumerate(maps):
    shown = points[points[:, 2] <= 1.5]
    for row in range(2):
        ax = axes[row, col]
        ax.scatter(shown[:, 0], shown[:, 1], c=shown[:, 2], cmap='viridis', vmin=-1.5, vmax=1.5, s=.5 if row == 0 else 3, rasterized=True)
        ax.set_aspect('equal'); ax.grid(alpha=.15); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
        if row == 0: ax.set_title(name); ax.set_xlim(-5, 31); ax.set_ylim(-14, 36)
        else: ax.set_title('走廊目标附近'); ax.set_xlim(1, 5); ax.set_ylim(13, 17)
fig.suptitle('固定观测和标签的顺序验证：同一位姿、地图坐标 z ≤ 1.5 米', fontsize=15)
fig.savefig(a.output / '来源追踪_顺序对比.png', dpi=160)
base = maps[-1][1]; base = base[base[:, 2] <= 1.5]
fig2, ax = plt.subplots(figsize=(10, 11), constrained_layout=True)
ax.scatter(base[:, 0], base[:, 1], s=.2, c='#a5abb0', rasterized=True, label='最终保留地图')
for path, color, label in [(a.experiment / 'removed_return_voxels.pcd', '#e43232', '候选删除的来源点体素'), (a.outlier / 'adaptive_candidates.pcd', '#078ace', '离群候选')]:
    points = load_pcd(path); points = points[points[:, 2] <= 1.5]
    ax.scatter(points[:, 0], points[:, 1], s=2, c=color, alpha=.75, rasterized=True, label=label)
ax.set_aspect('equal'); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）'); ax.grid(alpha=.15)
ax.set_title('被删点检查：同体素中可能同时有被删和保留的来源点\n红蓝点不是人工标注真值'); ax.legend(markerscale=4)
fig2.savefig(a.output / '来源追踪_删除候选.png', dpi=160)

#!/usr/bin/env python3
"""Visual and point-count comparison; none of the inputs are ground truth."""
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
p.add_argument('--point-first', type=Path, required=True)
p.add_argument('--cluster-first', type=Path, required=True)
p.add_argument('--joint', type=Path, required=True)
p.add_argument('--dynamic', type=Path, required=True)
p.add_argument('--outliers', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
maps = [('原始融合图', load_pcd(a.raw)), ('点→离群→团', load_pcd(a.point_first)), ('团→点→离群', load_pcd(a.cluster_first)), ('联合证据 + 弱离群', load_pcd(a.joint))]
far = lambda x: (x[:, 1] > 25) & (np.abs(x[:, 0]) < 10)
target = lambda x: (x[:, 0] >= 2.68) & (x[:, 0] <= 3.57) & (x[:, 1] >= 13.81) & (x[:, 1] <= 15.70) & (x[:, 2] >= -.80) & (x[:, 2] <= .69)
report = {name: {'all': len(x), 'display': int((x[:, 2] <= 1.5).sum()), 'far_all_heights': int(far(x).sum()), 'target_bbox_all_heights': int(target(x).sum())} for name, x in maps}
report['note'] = 'Counts do not measure accuracy. The bounding box may contain static points. All results are experimental.'
a.output.mkdir(parents=True, exist_ok=True)
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
        else: ax.set_xlim(1, 5); ax.set_ylim(13, 17); ax.set_title('走廊目标附近')
fig.suptitle('同一回放的离线处理对比：地图坐标 z ≤ 1.5 米；无人工真值', fontsize=15)
fig.savefig(a.output / '联合证据_地图对比.png', dpi=160)
background = maps[-1][1]
background = background[background[:, 2] <= 1.5]
fig2, ax = plt.subplots(figsize=(10, 11), constrained_layout=True)
ax.scatter(background[:, 0], background[:, 1], s=.2, c='#b3b8bd', rasterized=True, label='保留地图')
for path, color, label in [(a.dynamic, '#e03636', '动态删除候选'), (a.outliers, '#168ac5', '轻度离群候选')]:
    points = load_pcd(path); points = points[points[:, 2] <= 1.5]
    ax.scatter(points[:, 0], points[:, 1], s=2, c=color, alpha=.75, rasterized=True, label=label)
ax.set_aspect('equal'); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）'); ax.grid(alpha=.15)
ax.set_title('删除候选检查：红/蓝点均非人工标注真值'); ax.legend(markerscale=4)
fig2.savefig(a.output / '联合证据_删除候选.png', dpi=160)

#!/usr/bin/env python3
"""Compare label-aware orders using shared observation evidence, not ground truth."""
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
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
maps = [('原始融合图', load_pcd(a.raw)), ('点标签→团标签→轻度离群', load_pcd(a.point_first)), ('团标签→点标签→轻度离群', load_pcd(a.cluster_first))]
far = lambda x: (x[:, 1] > 25) & (np.abs(x[:, 0]) < 10)
target = lambda x: (x[:, 0] >= 2.68) & (x[:, 0] <= 3.57) & (x[:, 1] >= 13.81) & (x[:, 1] <= 15.70) & (x[:, 2] >= -.80) & (x[:, 2] <= .69)
report = {name: {'all': len(x), 'display': int((x[:, 2] <= 1.5).sum()), 'far_all_heights': int(far(x).sum()), 'target_bbox_all_heights': int(target(x).sum())} for name, x in maps}
report['warning'] = 'Group queries differ by preceding point labels; raw ray observations remain unchanged. Counts are not ground truth, and the target box may contain static points.'
if a.output.exists():
    raise ValueError('output directory already exists')
a.output.mkdir(parents=True)
(a.output / 'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(report, ensure_ascii=False, indent=2))
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(2, 3, figsize=(15, 10), constrained_layout=True)
for col, (name, points) in enumerate(maps):
    shown = points[points[:, 2] <= 1.5]
    for row in range(2):
        ax = axes[row, col]
        ax.scatter(shown[:, 0], shown[:, 1], c=shown[:, 2], cmap='viridis', vmin=-1.5, vmax=1.5, s=.5 if row == 0 else 3, rasterized=True)
        ax.set_aspect('equal'); ax.grid(alpha=.15); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
        if row == 0: ax.set_title(name); ax.set_xlim(-5, 31); ax.set_ylim(-14, 36)
        else: ax.set_title('走廊目标附近'); ax.set_xlim(1, 5); ax.set_ylim(13, 17)
fig.suptitle('同一原始观测、相同阈值：标签驱动的处理顺序对比\n团级候选集合不同，关键帧射线证据均未修改', fontsize=14)
fig.savefig(a.output / '标签驱动_顺序对比.png', dpi=180)

#!/usr/bin/env python3
"""Draw cluster review locations without asserting candidate labels are truth."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from multiframe_outlier_review import load_pcd

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--map', required=True, type=Path)
p.add_argument('--review', required=True, type=Path)
a = p.parse_args()
cloud = load_pcd(a.map)
with (a.review / 'clusters.csv').open() as stream:
    rows = list(csv.DictReader(stream))
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
shown = cloud[cloud[:, 2] <= 1.5]
fig, ax = plt.subplots(figsize=(10, 12), constrained_layout=True)
ax.scatter(shown[:, 0], shown[:, 1], s=.18, c='#9ba2a8', rasterized=True)
for cls, color, label in [('uncertain', '#e8a72c', '证据不足：保留'), ('dynamic_candidate', '#e43232', '动态候选：需复核')]:
    selected = [r for r in rows if r['classification'] == cls]
    ax.scatter([float(r['x']) for r in selected], [float(r['y']) for r in selected], s=22 if cls == 'uncertain' else 50, c=color, edgecolors='black', linewidths=.3, label=label, zorder=3)
    if cls == 'dynamic_candidate':
        for r in selected:
            ax.annotate(r['cluster_id'], (float(r['x']), float(r['y'])), xytext=(3, 3), textcoords='offset points', fontsize=7)
ax.set_aspect('equal', adjustable='box'); ax.set_xlabel('地图 X（米）'); ax.set_ylabel('地图 Y（米）')
ax.set_title('点团可见性检查：编号对应 clusters.csv\n候选不等于真值；未参与检查的点保持灰色')
ax.grid(alpha=.15); ax.legend(loc='best'); fig.savefig(a.review / '点团可见性检查.png', dpi=180)

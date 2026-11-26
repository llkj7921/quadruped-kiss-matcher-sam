# Offline multi-keyframe outlier review (experimental)

This optional offline review starts with the visibility-filtered PCD and the
same run's saved keyframes and optimized trajectory. It does not alter SLAM,
dynamic filtering, or source maps. Each map point receives at most one vote
per keyframe when an original keyframe return lies within the support radius
after transformation by the optimized pose. A candidate must have no more than
the configured number of supporting frames, mean distance to its k nearest
neighbors greater than the absolute spacing floor, and greater than a factor
times the median of those neighbors' mean distances. The result is deliberately
conservative; candidate points are not labeled ground truth.

```
python3 ros/scripts/multiframe_outlier_review.py \
  --map /path/to/visibility/clean_map.pcd \
  --keyframes /path/to/map/keyframes \
  --trajectory /path/to/map/optimized_trajectory_tum.txt \
  --timestamps /path/to/map/timestamps.txt \
  --output /path/to/new/review
```

Dependencies: Python 3, NumPy, SciPy, and `pcl_convert_pcd_ascii_binary` for
compressed PCDs. The comparison script additionally needs Matplotlib and a
Chinese font. See `--help` for the experimental parameters. Outputs include
full-height retained and candidate PCDs, display-clipped copies, per-point
evidence, and a JSON report. Do not overwrite existing datasets.

Support is proximity evidence, not an independent observation: a stationary
person can receive many votes; keyframes can overlap in time and location;
pose error, occlusion, voxelization and radius choice affect votes. The local
spacing test does not guarantee preservation of thin objects or removal of
clustered noise. This is not a substitute for free-space visibility or semantic
dynamic-object filtering. Validate against labeled data and inspect both
retained and removed points before navigation use.

Run synthetic checks with:

```
python3 -m unittest discover -s ros/tests -p test_multiframe_outlier_review.py
```

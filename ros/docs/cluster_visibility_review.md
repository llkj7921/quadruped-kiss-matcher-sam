# Experimental cluster-level visibility review

Offline review only. Starting from an existing filtered PCD, this groups
points with a radius-connected graph and reviews components within a point-count
interval. Points outside those components remain untouched. Sampled cluster
points are transformed into every optimized keyframe. As in the existing
visibility filter, a nearby angular ray and a range-dependent margin determine
whether a sample appears occupied, free, or occluded. Frame-level votes require
minimum sample coverage. A proposed dynamic candidate requires multiple
occupied frames followed by multiple free-space frames after a time gap.
`proposed_kept.pcd` excludes candidates for comparison only; uncertain and
unreviewed points remain. Nothing modifies the original map or online SLAM.

```
python3 ros/scripts/cluster_visibility_review.py \
  --map /path/to/kept.pcd --keyframes /path/to/keyframes \
  --trajectory /path/to/optimized_trajectory_tum.txt \
  --timestamps /path/to/timestamps.txt --temporal-mode bidirectional \
  --output /path/to/new/review
python3 ros/scripts/plot_cluster_visibility_review.py \
  --map /path/to/kept.pcd --review /path/to/new/review
```

The default `forward` mode preserves the original behavior. Opt-in
`bidirectional` mode accepts either (a) three later free frame votes after
greedy temporal and spatial baseline separation or (b) a free vote before
and another after the occupied interval, separated by both the configured
time gap and sensor-origin distance. The rules are experimental: a different
sensor position does not prove statistically independent viewpoints, and the
occupied-frame count itself is not deduplicated. Occluded and unobserved
samples are never counted as free. The selected timestamps are recorded in
`frame_evidence.json`; inspect their raw keyframes before using a proposed
deletion. The separate `inspect_cluster_rays.py` script draws point/nearest-ray
matches for selected keyframes, but a 2-D projection is not proof of a valid
3-D line of sight.

Outputs include full-height and clipped proposed/uncertain/candidate PCDs,
`clusters.csv` with centroid, vote counts and class, `frame_evidence.json`
with voting timestamps, and `report.json`. The plot uses map z <= 1.5 m.
Requires NumPy, SciPy, PCL converter for compressed PCD, and Matplotlib for
the plot. The radius graph can require substantial memory for dense maps.

Important limitations: proximity connected components can merge a person into
a wall or split a person into fragments; points outside size limits are never
reviewed. Point ordering biases the deterministic samples. Angular nearest-ray
matching is approximate, can confuse adjacent objects, and does not prove
visibility. Nearby frames are correlated and counts are not independent;
relative timestamps do not guarantee a valid line of sight. Dynamic candidate
labels can therefore be wrong, and lack of evidence does not prove an object is
static. This tool is not semantic detection and cannot identify the person in a
screenshot without a map coordinate. Review candidates against raw keyframes
before deletion or navigation use; do not equate an empty return with free
space or use this output as a safety-certified map.

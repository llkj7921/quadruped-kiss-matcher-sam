# Online back-end dynamic point filtering

The optional filter is applied to keyframe clouds when building Scan Context
descriptors and immediately before loop-closure registration. Original
keyframes, map products and FAST-LIO inputs are unchanged. It does not clean
the published map or improve the front-end odometry.

For each keyframe, at most `history_frames` earlier keyframes are compared in
their own sensor coordinate systems using their optimized poses. A point is
removed only when at least `minimum_free_observations` scans have a return
farther away in the same angular bin, and no scan has a nearby return. Missing
and occluded measurements are unknown and retained. The filter uses the first
return in a bin. It is a conservative approximation of ray visibility, not a
full occupancy or semantic model. Binning, pose error, occlusion, sparse
Mid-360 coverage, and sensor extrinsics can cause errors; a stationary person
can be retained. Scan Context descriptors are cached when first created and
are not recomputed after later pose-graph corrections. Registration uses the
latest available optimized poses. The two results may therefore differ.

The feature is disabled by default. Set `dynamic_filter.enable: true` in
`slam_config.yaml` to evaluate it. `angular_resolution_rad` controls ray bin
size and `range_margin_m` provides tolerance for alignment and measurement
error. Validate on representative recordings before deployment, measuring loop
acceptance, trajectory accuracy, static point loss and callback timing. The
offline filter remains available for final map cleanup.

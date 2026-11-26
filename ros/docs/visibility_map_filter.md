# Offline approximate visibility filter

`visibility_map_filter MAP_PRODUCT NEW_OUTPUT_DIRECTORY` compares each saved
keyframe point with other optimized keyframes in their local sensor frames.
It counts nearby angular returns behind the candidate as free-space evidence,
nearby returns at the candidate range as occupied evidence, and returns in
front as occlusion. Missing returns are unknown. Only candidates with at least
three free observations and twice as many free as occupied observations are
removed; all other points remain in the output. Nearby occupied evidence
therefore preserves/reverts candidates but is not a full Removert algorithm.

The tool writes a new `clean_map.pcd`, `removed_points.pcd` and
`uncertain_points.pcd` (when nonempty), plus `report.yaml`. It refuses an
existing output directory and never changes the input product. Counts in the
report refer to original keyframe points before output voxelization.

This is an approximate angular-neighbour test on saved, downsampled clouds,
not calibrated raw-beam ray tracing. Nearest angular samples may represent
different physical surfaces, especially at depth discontinuities. FAST-LIO
publishes cloud data in its configured frame and its extrinsics need to be
validated before interpreting the pose origin as the LiDAR origin. Pose error,
motion distortion, sparse solid-state LiDAR coverage, moving objects that stay
in one place, and unobserved backgrounds can all cause classification errors.
Output must be inspected against both dynamic objects and static surfaces.
This does not alter the online map, the front-end, or original keyframes.

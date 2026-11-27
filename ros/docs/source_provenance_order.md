# Saved-keyframe source provenance and fixed-label order experiment

This offline experiment reconstructs the relationship between a saved fused
PCL VoxelGrid map and the saved keyframe returns used to build it. It does not
change raw observations, rerun SLAM or update a relocalization product. This is
**not** raw Livox return provenance: FAST-LIO/keyframe sampling already occurred.
The mapping is reconstructed after the original mapping run, not automatically
recorded by the online map writer. It is rejected unless all saved keyframe
returns map to unique saved-map voxels and the reconstructed voxel centroids
agree with the saved map within 0.001 m. The method assumes the same PCL voxel
leaf, saved optimized poses, input frame coordinates and map product. Persisted
pose rounding, boundaries or different PCL conventions can cause rejection.

`visibility_evidence_export` calls the existing visibility implementation with
optional evidence capture and writes one row per saved keyframe point:
`frame_id point_id free occupied occluded`. Default existing callers do not
capture this extra data. `capture_visibility_evidence.py` records source file
and exporter hashes before/after running it; the experiment checks the manifest
before consuming it. Hashes establish that files match the captured snapshot,
not that the initial map product was correct. Cluster review must be generated
from the same raw map, keyframes and optimized poses.

The NPZ sidecar gives a bidirectional one-to-many mapping:

- `frame_offsets[f] + j` identifies saved keyframe `f` point index `j`.
- `source_voxel[source_index]` is the row index of the original fused PCD voxel.
- `voxel_source_indices[voxel_offsets[v]:voxel_offsets[v+1]]` lists all saved
  keyframe source indices contributing to voxel `v`.
- The same source index indexes point votes, point labels, cluster membership
  and cluster-refined labels.

The fixed point label reproduces the existing visibility threshold. A source
return in a cluster-candidate map voxel receives a cluster candidate label only
if it has at least one free vote and free votes exceed occupied votes. The group
label is spatial association to a map cluster plus source-level evidence, not
proof the source return belongs to the same semantic object. A single voxel may
contain both static and dynamic returns; the tool stores both rather than
deleting every return within it. Labels and votes are correlated and approximate.

In this experiment both order paths consume identical fixed labels and combine
them using monotone union; therefore point→group and group→point are identical
**by design**. This verifies reproducibility and no evidence mutation; it does
not establish that one ordering is superior or that the candidate labels are
accurate. `removed_return_voxels.pcd` can overlap retained voxels because
different contributing returns in one voxel can receive different labels.
The final weak outlier step can protect retained uncertain group voxels using
`adaptive_outlier_review.py --protected`; this does not prove they are static.

## Label-aware order comparison

`point_stage_query.pcd` is generated from the unlabelled source returns for
clustering only. It does not modify keyframes or the observations used by ray
tests. Run `cluster_visibility_review.py` on this query using the same
parameters and original keyframes as the original-map cluster review, then run:

```
python3 ros/scripts/compare_label_aware_orders.py \
  --provenance /path/to/new/order_test \
  --group-before-point /path/to/same_raw_map_cluster_review \
  --group-after-point /path/to/point_stage_cluster_review \
  --output /path/to/new/label_aware_comparison
```

This compares group-first on the original map with point-first on the
point-labelled query. Both use the original ray evidence and the same label
rules; grouping can differ because the candidate query differs. Apply the same
outlier parameters to each retained map, using its corresponding protected
file. Candidate labels and group membership are not semantic ground truth.
Review removed returns and outliers as well as retained structure. The two
orders are experimental definitions, not a general comparison of all possible
ordering algorithms.

```
# After rebuilding the ROS package with visibility_evidence_export installed:
python3 ros/scripts/capture_visibility_evidence.py --session /path/to/map_product \
  --exporter /path/to/visibility_evidence_export --output /path/to/new/evidence
python3 ros/scripts/provenance_order_experiment.py \
  --map /path/to/map_product/sequence_map.pcd \
  --keyframes /path/to/map_product/keyframes \
  --trajectory /path/to/map_product/optimized_trajectory_tum.txt \
  --timestamps /path/to/map_product/timestamps.txt \
  --point-evidence /path/to/new/evidence/point_evidence.tsv \
  --cluster-review /path/to/same_raw_map_cluster_review \
  --output /path/to/new/order_test
python3 ros/scripts/adaptive_outlier_review.py --map /path/to/new/order_test/kept.pcd \
  --output /path/to/new/outlier --sigma-factor 4 --ratio 2 --threshold-floor .08 \
  --protected /path/to/new/order_test/uncertain_group_protected.pcd
```

All outputs are experimental; do not use them as navigation truth without
reviewing false removals and residuals against raw data and ground truth.

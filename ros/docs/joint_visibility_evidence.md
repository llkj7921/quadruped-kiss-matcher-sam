# Joint offline visibility evidence (experimental)

`fuse_visibility_evidence.py` reuses one immutable raw map, the precomputed
point-level visibility candidates, cluster-level candidates computed from the
same raw product, and the original keyframes and optimized poses. It does not
modify input keyframes or the online SLAM pipeline. Candidate PCDs are mapped
to the raw 0.05 m map via voxel keys; this is approximate spatial association,
not per-return provenance. Inputs must all come from the exact same run,
poses, map, voxel size and coordinate frame. The command checks paths against
the original filter reports and records SHA-256 digests of input names/bytes;
it cannot prove that prior reports or files were not altered before the run.

For the union of candidate voxels it obtains approximate nearest-ray evidence
from unmodified keyframes at two angular thresholds. A dynamic candidate
requires >=2 greedily separated free observations, >=2 fine and >=2 coarse
free-frame matches, and more separated free than occupied observations.
Occlusion/unmatched observations are not free. A retained 'static recovery
candidate' requires >=3 separated occupied observations spanning >=20 seconds
and no free group. All other candidates are retained as uncertain. Greedy view
groups compare against the previous selected pose/time, not every group;
fine/coarse rays are correlated and are not independent sensors. Repeated
occupation can be a stationary person, and lack of free evidence is not proof
of static structure. These experimental thresholds are not calibrated.

Requires PyYAML in addition to the dependencies of the other offline tools.

```
python3 ros/scripts/fuse_visibility_evidence.py \
  --map /path/to/lab_map.pcd \
  --point-candidates /path/to/visibility/removed_points.pcd \
  --cluster-candidates /path/to/cluster/dynamic_candidates.pcd \
  --keyframes /path/to/keyframes \
  --trajectory /path/to/optimized_trajectory_tum.txt \
  --timestamps /path/to/timestamps.txt --output /path/to/new/fusion

python3 ros/scripts/adaptive_outlier_review.py \
  --map /path/to/new/fusion/kept.pcd --output /path/to/new/light_outlier \
  --sigma-factor 4 --ratio 2 --threshold-floor .08 \
  --protected /path/to/new/fusion/restored_static_candidates.pcd \
  --protected /path/to/new/fusion/uncertain_candidates.pcd
```

The final filter applies weak density cleanup, and optional `--protected` PCDs
prevent previously retained uncertain/recovery candidates from being deleted.
The protected PCDs must match the exact input and are checked spatially. Small
components remain review-only. Keep all intermediate candidate PCDs, evidence
NPZ and reports. `compare_evidence_fusion.py` creates comparable overview,
target-area and deleted-candidate plots from explicitly supplied inputs.

This is not Removert and does not reconstruct exact original laser rays or
automatically correct registration error. Existing point and group candidates
already use approximate filtered/voxelized views, and cluster boundaries can
merge static and dynamic structure. Outputs are for visual/labelled-data
evaluation, not ground truth or navigation deployment; rejected and retained
points must be checked in multiple regions. It does not regenerate cleaned
keyframes for the relocalizer.

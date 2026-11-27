# Local density-adaptive outlier review (experimental)

`adaptive_outlier_review.py` operates offline on an existing XYZ-intensity
PCD. Each point's average distance to its nearest k neighbors is compared to
the local median and robust MAD of neighbor spacing, with a ratio and absolute
floor. Spatial neighborhoods are queried globally rather than partitioned into
blocks. This is **not** PCL SOR, a range-based filter, or a dynamic-object
detector. Map-origin distance is never treated as measurement range.

The script also builds a fixed-radius connected graph and exports small
components for review only. It never automatically deletes those components;
thin structures, distant surfaces, and isolated obstacles may all be real.
The radius graph can consume substantial memory on large or very dense clouds;
do not use unbounded inputs without profiling. Multi-frame support evidence
from the previous review can optionally be summarized, but does not change
the decision and is not ground truth. Points are not matched by index unless
the exact same input path and point count are recorded in the prior report;
do not modify the input PCD in place.

Optional repeatable `--protected` arguments preserve exact matching points
from later outlier removal; this is useful for uncertain or recovery-candidate
points produced by the joint evidence review. Protection does not establish
that those points are genuinely static.

```
python3 ros/scripts/adaptive_outlier_review.py \
  --map /path/to/visibility/clean_map.pcd \
  --support-evidence /path/to/multiframe_review \
  --output /path/to/new/adaptive_review

python3 ros/scripts/compare_adaptive_outlier.py \
  --map /path/to/visibility/clean_map.pcd --sor /path/to/sor_review \
  --multiframe /path/to/multiframe_review --adaptive /path/to/adaptive_review
```

Requirements are Python 3, NumPy, SciPy, PCL converter for compressed PCD,
and Matplotlib plus a Chinese font for the comparison. Thresholds are trial
parameters, not general recommendations. Outputs contain full-height and
display-clipped PCDs, separate candidate sets, evidence, and a report.
Retained and removed points must be assessed in several locations using raw
observations and suitable ground truth before any navigation deployment.

# Offline dynamic-point map filtering

`offline_dynamic_map_filter` rebuilds a clean map from a completed KISS-SAM
map product. It never overwrites `final_map.pcd` or the original keyframes.

The filter transforms every keyframe with its optimized pose, quantizes points
into observation voxels, and counts each voxel at most once per keyframe. A
voxel is retained only when it is observed by enough distinct keyframes over a
long enough time span. The retained points are then voxelized at the requested
output resolution.

This is a geometric persistence filter. It removes many walking-person trails
and other short-lived objects, but it is not semantic human detection. A person
standing still for long enough can be retained, while a real surface observed
only briefly can be removed.

## Usage

```bash
source /opt/ros/humble/setup.bash
source /path/to/fastlio2_ws/install/setup.bash

ros2 run kiss_matcher_ros offline_dynamic_map_filter \
  /absolute/path/to/map_product \
  --observation-voxel 0.30 \
  --output-voxel 0.10 \
  --min-observations 3 \
  --min-duration 0.5
```

Default outputs inside the map-product directory:

```text
clean_map.pcd
clean_map_report.yaml
```

Recommended initial values:

| Scene | Observation voxel | Output voxel | Min observations | Min duration |
| --- | ---: | ---: | ---: | ---: |
| Indoor | 0.30 m | 0.10 m | 3 | 0.5 s |
| Outdoor | 0.50 m | 0.20 m | 3 | 1.0 s |

If the result loses walls or map boundaries, first reduce `min-observations`
to 2. If walking trails remain, increase `min-duration` before increasing the
observation count. Always inspect `clean_map.pcd` before using it downstream.

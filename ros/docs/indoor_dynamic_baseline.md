# 室内离线动态过滤基线：点 → 离群 → 团

这是对室内 0.05 m 地图实验的复现入口，不改变在线建图、回环、定位或导航。使用原始保存的关键帧、优化轨迹和时间戳；此处“原始”是指保存的关键帧，已经过前端采样，并非原始雷达回波。

1. 点级：`visibility_map_filter`，输出体素 0.05 m；空闲观测至少 3 次且空闲/占据比例至少 2；角分辨率 0.01 rad、距离余量 0.4 m、最大距离 30 m、最小时间间隔 0.5 s。工具默认参数必须与记录一致，否则入口停止。
2. 离群：自适应 SOR，20 邻居、100 局部邻居、sigma 2.5、ratio 1.5、floor 0.04 m。小连通块仅供审查，不自动删除；本基线不启用后续实验的不确定团保护。
3. 团级：输入上一步保留地图，用**未修改的原始保存关键帧**计算双向时序证据，参数见运行清单。只从地图删除动态候选团，不修改关键帧或原观测。

依赖 ROS 2 下编译的 `visibility_map_filter`、Python 3、NumPy 和 SciPy。入口复用现有离线工具，记录命令、输入关键帧摘要和各工具摘要，要求输出目录不存在且不与输入互相包含。失败时留下标记为 failed 的目录供检查；不覆盖输入。

```bash
source /opt/ros/humble/setup.bash
python3 ros/scripts/run_indoor_dynamic_baseline.py \
  --session /path/to/map_product \
  --visibility-executable /path/to/visibility_map_filter \
  --output /path/to/new_baseline_output
```

结果：`01_point/clean_map.pcd`、`02_outlier/kept.pcd`、`03_cluster/proposed_kept.pcd`（最终完整地图）、`03_cluster/proposed_kept_display.pcd`（地图坐标系 z≤1.5 m 的展示副本）。各阶段各自保存删除候选、报告和中间结果；`manifest.json` 为复现清单。地图 z 截取不是估计真实天花板。

原始实验中各阶段点数分别为 232897、228138、227870；实际结果还取决于输入、编译依赖和实现版本。图像整洁或点数少不能说明真实静态结构被保留；务必检查删除候选和残留，特别是远端与薄结构。本流程是离线实验基线，不是实时过滤或可直接投入机器人导航的安全认证。

“团→点→离群”对照曾使用体素掩码修改关键帧；本基线没有这一掩码步骤。不要把两条路径混淆。

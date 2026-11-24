# Quadruped KISS-SAM

面向机器狗的 ROS 2 SLAM 后端，基于
[MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) 二次开发。
本仓库不是 MIT-SPARK 官方发行版本。

推荐从统一交付仓库
[`llkj7921/quadruped-slam`](https://github.com/llkj7921/quadruped-slam)
安装和启动，不要单独组合未经验证的组件版本。

## 主要功能

- FAST-LIO 关键帧接入、Scan Context 回环候选和 KISS-Matcher 配准。
- 位姿图优化、完整地图产品和增量建图。
- 已有地图重定位，连续平移与旋转一致性确认。
- 常规定位、建图、增量建图和重定位模式控制。
- 雷达断流或定位异常后的自动恢复、人工协助和确认闭环。
- `map -> odom -> base` 坐标系契约及强类型 ROS 2 控制接口。
- ROS 2 跨节点恢复测试和核心纯 C++ 单元测试。

## 系统关系

```text
Livox Mid-360 + IMU
         |
         v
quadruped-spark-fast-lio
         |  /odometry + /cloud_registered
         v
Quadruped KISS-SAM
   |          |              |
   v          v              v
pose graph  relocalizer  system mode/recovery
```

本仓库依赖：

- [`quadruped-spark-fast-lio`](https://github.com/llkj7921/quadruped-spark-fast-lio)
- [`slam-system-interfaces`](https://github.com/llkj7921/slam-system-interfaces)

兼容版本由总仓库的 `slam.repos` 锁定。

## 安装

```bash
git clone https://github.com/llkj7921/quadruped-slam.git
cd quadruped-slam
./scripts/install.sh
./scripts/build.sh
cp config/robot.env.example config/robot.env
./scripts/doctor.sh
```

## 启动

```bash
./scripts/start.sh
```

默认由统一启动文件同时启动 FAST-LIO、KISS-SAM 和模式管理器。Livox 驱动
可以独立运行，也可以通过 `config/robot.env` 启用。

常用控制：

```bash
./scripts/mode.sh mapping
./scripts/mode.sh incremental
./scripts/mode.sh exit
./scripts/mode.sh status
./scripts/mode.sh recovery
./scripts/mode.sh acknowledge
```

特殊模式之间不能直接切换，必须先退出到常规定位。

建图完成后可生成不覆盖原图的离线清理地图：

```bash
ros2 run kiss_matcher_ros offline_dynamic_map_filter \
  /absolute/path/to/map_product \
  --observation-voxel 0.30 --output-voxel 0.10 \
  --min-observations 3 --min-duration 0.5
```

默认输出 `clean_map.pcd` 和 `clean_map_report.yaml`。该功能按跨关键帧
体素持久性清除短暂目标，不是语义人体检测；详细说明见
[离线动态点过滤](ros/docs/offline_dynamic_filter.md)。

## 恢复状态

```bash
ros2 topic echo /recovery/state
```

恢复消息提供：

- `motion_allowed`：是否允许运动；当前保留人工遥控能力。
- `map_writes_allowed`：是否允许更新地图。
- `autonomy_allowed`：是否允许自主导航。
- `reason`：断流、定位质量或重定位失败原因。

导航/任务控制必须订阅 `/recovery/state` 并执行 `autonomy_allowed`；SLAM
发布权限并不会自动阻止另一个控制节点发送速度。

## 独立构建与测试

建议仍在包含三个兼容组件的 colcon 工作区中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to kiss_matcher_ros --symlink-install
colcon test --packages-select kiss_matcher_ros
colcon test-result --verbose
```

## 项目来源与许可证

本仓库保留上游 Git 历史，所以 GitHub Contributors 中会显示原项目贡献者。
这是正常的来源记录，不代表本仓库是上游官方版本。

- 上游项目及论文引用：[docs/UPSTREAM_README.md](docs/UPSTREAM_README.md)
- 许可证：[LICENSE](LICENSE)（MIT）
- 上游项目：[`MIT-SPARK/KISS-Matcher`](https://github.com/MIT-SPARK/KISS-Matcher)

<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
</div>

<p align="center">
  <img src="assets/images/demo1.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo2.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/demo3.png" width="100%"/>
</p>
<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。仿真走 `~/go2_sim_ws` Gazebo 物理闭环，由 `run.launch.py` 统一启动。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成。



## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

仿真依赖独立工作空间 `~/go2_sim_ws`（Gazebo Classic + CHAMP + Velodyne）。旧 kinematic / mockamap / local_sensing 仿真包已移除。

## 快速启动

```bash
source /opt/ros/humble/setup.bash
source ~/3d_nav/install/setup.bash
source ~/go2_sim_ws/install/setup.bash

# 终端 1
ros2 launch scan_planner rviz.launch.py use_sim_time:=true

# 终端 2 — Gazebo + 桥 + 规划
ros2 launch scan_planner run.launch.py navi_mode:=1

# 真机
ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=1
```

### 导航模式说明
- `navi_mode:=1`：RViz 2D Goal Pose
- `navi_mode:=2`：`keypoints_file` 预设路径点
- `navi_mode:=3`：`/initial_path` 或 `reference_path_file`

控制器固定为闭环（`cmd_vel` → CHAMP / 真机底盘）。真机 LIO/驱动为外部依赖，话题默认 `/LIO/odom_vehicle`、`/LIO/clouds_lidar`、`/cmd_vel`。

## 配置与接口

- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/gazebo_sim.yaml` / `gazebo_bridge.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

参考路径演示发布器也可以独立运行：

```bash
ros2 run scan_planner reference_path_publisher.py --ros-args \
  --params-file src/planner/plan_manage/config/reference_path.map.yaml \
  -r body_pose:=/quad_0/body_pose -r initial_path:=/initial_path
```

## Gazebo 仿真说明

导航闭环使用 `~/go2_sim_ws`（Gazebo Classic + CHAMP + Velodyne），见 `使用说明.md`。  
本仓库内 `go2_description` 主要用于真机 RViz 的 URDF；另可选：

```bash
ros2 launch go2_description go2_rviz.launch.py
```


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。

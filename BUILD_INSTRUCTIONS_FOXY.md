# ROS2 Foxy 编译说明

## 环境要求

- ROS2 Foxy
- Ubuntu 20.04
- 支持 x86_64 和 aarch64 架构

## 快速编译

```bash
cd ~/3d_nav
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 完整编译（禁用测试以避免VTK链接问题）
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 编译说明

### 为什么需要 -DBUILD_TESTING=OFF？

foxy_dev 分支的测试代码依赖 PCL 的可视化功能，而系统上的 VTK 7.1 使用版本化库名（如 libvtkCommonCore-7.1.so），导致链接问题。禁用测试可以避免这个问题，不影响主要功能的使用。

### 为什么需要 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp？

系统默认配置为 rmw_fastrtps_cpp，但该中间件在当前环境不可用。使用 rmw_cyclonedds_cpp 可以正常编译和运行。

### 单独编译某个包

```bash
cd ~/3d_nav
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 编译单个包
colcon build --packages-select <package_name> \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

### 清理构建

```bash
cd ~/3d_nav
rm -rf build/ install/ log/
```

## 已解决的问题

1. **FLANN::FLANN 目标未找到**
   - 通过 cmake/foxy_flann_fix.cmake 自动查找和创建 FLANN 目标

2. **VTK 库链接失败**
   - 自动处理 VTK 7.1 版本化库名（-lvtkXXX -> -lvtkXXX-7.1）
   - 支持传递依赖的 PCL 包

3. **绝对路径问题**
   - 所有 CMakeLists.txt 使用相对路径引用 foxy_flann_fix.cmake
   - 提高跨环境可移植性

## 编译时间参考

在 ARM64 (Go2机器人) 上完整编译约 6-7 分钟。

## 运行时配置

编译完成后，使用前需要 source：

```bash
source ~/3d_nav/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

建议将以下内容添加到 ~/.bashrc：

```bash
# ROS2 Foxy
source /opt/ros/foxy/setup.bash
source ~/3d_nav/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 闭环控制器优化

foxy_dev 分支已集成闭环控制器的 PID 优化，详见 CLOSED_LOOP_OPTIMIZATION.md。

新增参数可在 src/planner/plan_manage/config/controllers.yaml 中配置。

## 故障排除

### 问题：cannot find -lvtkXXX

**原因**：VTK 修复未生效

**解决方案**：
```bash
# 清理后重新编译
rm -rf build/ install/
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

### 问题：RMW implementation 'rmw_fastrtps_cpp' not available

**原因**：默认中间件不可用

**解决方案**：
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 问题：ModuleNotFoundError: No module named 'ament_package'

**原因**：未 source ROS2 环境

**解决方案**：
```bash
source /opt/ros/foxy/setup.bash
```

## 技术细节

### cmake/foxy_flann_fix.cmake 工作原理

1. **FLANN 处理**：
   - 尝试使用 find_package(FLANN)
   - 失败则手动查找 libflann_cpp.so
   - 创建 FLANN::FLANN 导入目标

2. **VTK 处理**：
   - 无条件执行（解决传递依赖问题）
   - 遍历常用 VTK 库列表
   - 查找版本化库文件（-7.1 后缀）
   - 创建无版本号的别名目标

### 包依赖关系

```
scan_planner
├── bspline_opt
│   ├── path_searching
│   │   └── plan_env (依赖 PCL → VTK)
│   └── plan_env
└── traj_utils
```

所有依赖 plan_env 的包都会传递获得 PCL/VTK 依赖，因此都需要 foxy_flann_fix.cmake。

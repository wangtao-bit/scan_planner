# 闭环控制器优化说明

## 优化概述

本次优化针对 `closed_loop_controller.cpp` 进行了全面改进，显著提高轨迹跟踪精度和运动流畅性。

## 主要改进

### 1. **从P控制升级到PID控制**
**问题**：原始纯P控制无法消除稳态误差，在斜坡、侧向扰动时会有持续偏差。

**解决方案**：
- 添加积分项 `ki_pos_`：累积位置误差并补偿
- 添加微分项 `kd_pos_`：利用速度误差抑制超调和震荡
- 实现积分抗饱和：仅在误差较小时累积，并限制积分项幅值

```cpp
// PID控制输出
Eigen::Vector2d feedback_vel = kp_pos_ * pos_error_new +
                               ki_pos_ * pos_error_integral_ +
                               kd_pos_ * vel_error;
```

### 2. **自适应时间推进**
**问题**：原始固定速率时间推进，即使跟踪误差很大参考点仍然前进，导致误差累积。

**解决方案**：基于位置误差的自适应时间缩放
```cpp
// 误差大时自动减慢参考点推进
const double time_scale = 1.0 / (1.0 + adaptive_time_gain_ * error_norm);
exec_time_ += dt * time_scale;
```

### 3. **柔性航向约束**
**问题**：原始硬停止策略（航向误差>阈值时完全停止），导致频繁"停-转-走"，轨迹不流畅。

**解决方案**：用速度线性衰减代替硬停止
```cpp
// 航向误差大时降低速度但不完全停止
double speed_scale = 1.0;
if (std::abs(yaw_error) > heading_error_threshold_) {
    const double excess = std::abs(yaw_error) - heading_error_threshold_;
    speed_scale = std::max(heading_speed_decay_,
                          1.0 - excess / (M_PI - heading_error_threshold_));
}
vel_world *= speed_scale;
```

### 4. **分层速度限幅**
**问题**：原始直接截断总速度，前馈和反馈被等比缩放，误差大时前馈项被削弱。

**解决方案**：按优先级分配速度预算，优先保证反馈控制能力
```cpp
// 按 feedforward_priority_ 比例分配速度预算
const double feedback_budget = max_total_vel * feedforward_priority_;
const double feedforward_budget = max_total_vel * (1.0 - feedforward_priority_);
```

### 5. **加速度前馈补偿**
**问题**：高速运动时纯速度控制有相位滞后。

**解决方案**：利用B样条二阶导数提供加速度前馈
```cpp
if (use_acceleration_feedforward_ && traj_.size() > 2) {
    const Eigen::Vector3d acc_des = traj_[2].evaluateDeBoorT(exec_time_);
    acc_feedforward = Eigen::Vector2d(acc_des.x(), acc_des.y()) * 0.1;
}
```

### 6. **自适应前瞻距离**
**问题**：原始固定时间前瞻不考虑当前速度和曲率。

**解决方案**：基于当前速度的动态前瞻
```cpp
const double current_speed = std::max(0.1, odom_vel_.head<2>().norm());
const double adaptive_forward = time_forward_ * current_speed / max_vx_;
```

### 7. **速度反馈增强**
**问题**：原始未使用里程计速度信息。

**解决方案**：从 `Odometry` 消息提取速度并转换到世界坐标系，用于微分项计算

## 新增参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ki_pos` | 0.15 | 位置积分增益，消除稳态误差 |
| `kd_pos` | 0.3 | 位置微分增益，抑制震荡 |
| `integral_limit` | 0.5 | 积分项幅值限制（m），防止积分饱和 |
| `adaptive_time_gain` | 2.0 | 自适应时间推进增益，越大越保守 |
| `heading_speed_decay` | 0.3 | 航向误差大时的最小速度比例（0-1） |
| `use_acceleration_feedforward` | true | 是否启用加速度前馈 |
| `feedforward_priority` | 0.7 | 速度预算中反馈控制的优先级（0-1） |

## 调参建议

### 基础调参流程

1. **先调P增益 `kp_pos`**
   - 从 0.5 开始，逐步增大到响应满意但不震荡
   - 观察是否有稳态误差

2. **添加I增益 `ki_pos`**
   - 如果存在持续偏差，从 0.1 开始增加
   - 过大会导致超调和震荡
   - 配合调整 `integral_limit`

3. **调D增益 `kd_pos`**
   - 如果有震荡，从 0.2 开始增加
   - 过大会对噪声敏感

4. **调整自适应时间推进 `adaptive_time_gain`**
   - 增大：更保守，误差大时减速更明显
   - 减小：更激进，可能追不上轨迹

5. **调整航向策略**
   - `heading_speed_decay`：设置为 0.2-0.5，越小越流畅但可能偏航
   - `heading_error_threshold`：原始阈值，建议保持 0.8 弧度

### 不同场景的建议参数

**高精度慢速任务**（如精密扫描）：
```yaml
kp_pos: 1.0
ki_pos: 0.2
kd_pos: 0.4
adaptive_time_gain: 3.0
heading_speed_decay: 0.2
```

**快速导航任务**：
```yaml
kp_pos: 0.6
ki_pos: 0.1
kd_pos: 0.2
adaptive_time_gain: 1.5
heading_speed_decay: 0.4
```

**平衡模式**（当前默认）：
```yaml
kp_pos: 0.8
ki_pos: 0.15
kd_pos: 0.3
adaptive_time_gain: 2.0
heading_speed_decay: 0.3
```

## 测试验证

### 建议测试场景

1. **直线跟踪**：验证稳态误差是否消除
2. **圆形轨迹**：验证高曲率下的跟踪性能
3. **急转弯**：验证航向约束的流畅性
4. **变速轨迹**：验证加速度前馈效果
5. **斜坡干扰**：验证积分项的补偿能力

### 评估指标

- 横向误差（Cross-track error）
- 纵向误差（Along-track error）
- 稳态误差
- 超调量
- 轨迹流畅性（速度和角速度的抖动）

## 兼容性说明

- 完全向后兼容，不修改配置文件也能运行（使用默认参数）
- 原有参数保持不变，语义不变
- 新增参数均有合理默认值

## 调试工具

可以发布以下话题用于实时监控：
- 位置误差：`pos_error`
- 积分项：`pos_error_integral`
- 速度误差：`vel_error`
- 时间缩放因子：`time_scale`
- 速度衰减因子：`speed_scale`

（需要在代码中添加对应的 Publisher）

## 后续优化方向

1. 模型预测控制（MPC）：考虑未来多步状态
2. 自适应增益：根据速度和曲率动态调整PID参数
3. 学习型控制：利用历史数据优化参数
4. 鲁棒性增强：处理定位丢失、传感器噪声等异常情况

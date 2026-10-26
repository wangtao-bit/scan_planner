#!/bin/bash
# 一键启动所有 ROS2 节点 (在 192.168.8.160 板子上运行)
# 窗口0: livox 雷达驱动
# 窗口1: cmd_vel_to_go2 底盘桥
# 窗口2: lightning 建图/定位
# 窗口3: scan_planner 导航（交互选择 Mode 1/2/3）

if ! command -v tmux &> /dev/null; then
    echo "请先安装 tmux: sudo apt install tmux"
    exit 1
fi

SESSION="ros2_all"

setup_env() {
  conda deactivate 2>/dev/null || true
  conda deactivate 2>/dev/null || true
  export PATH=/usr/bin:/bin:$PATH
  source /opt/ros/foxy/setup.bash
  source /home/unitree/cyclonedds_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=30
  export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/dds.xml
}

tmux kill-session -t "$SESSION" 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n "livox" bash -lc "
  $(declare -f setup_env)
  setup_env
  cd /home/unitree/go2_tools && source install/setup.bash
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py
  echo '=== livox 已退出 ==='
  exec bash
"

tmux new-window -t "$SESSION" -n "cmd_vel" bash -lc "
  $(declare -f setup_env)
  setup_env
  export CYCLONEDDS_URI=file:///home/unitree/go2_tools/install/cmd_vel_to_go2/share/cmd_vel_to_go2/config/cyclonedds_eth0_eth1.xml
  cd /home/unitree/go2_tools && source install/setup.bash
  ros2 run cmd_vel_to_go2 cmd_vel_to_go2_sdk --ros-args \
    --params-file /home/unitree/go2_tools/src/cmd_vel_to_go2/config/cmd_vel_to_go2.yaml
  echo '=== cmd_vel_to_go2 已退出 ==='
  exec bash
"

tmux new-window -t "$SESSION" -n "mapping" bash -lc "
  $(declare -f setup_env)
  setup_env
  cd /home/unitree/location && source install/setup.bash
  ros2 launch lightning online_mapping_rviz.launch.py config:=livox_mid360_livo.yaml
  echo '=== lightning 已退出 ==='
  exec bash
"

tmux new-window -t "$SESSION" -n "navigation" bash -lc "
  $(declare -f setup_env)
  setup_env
  cd /home/unitree/3d_nav && source install/setup.bash
  echo '============================================================'
  echo ' 窗口3: scan_planner 导航模式选择'
  echo '  1: Mode 1 手动目标点 (RViz/外部发布 move_base_simple/goal)'
  echo '  2: Mode 2 预设航点巡航 (keypoints_file)'
  echo '  3: Mode 3 参考路径跟踪 (/initial_path)'
  echo '  n: 跳过并留在 shell'
  echo '============================================================'
  mode=''
  while true; do
    printf '请选择导航模式 [1/2/3/n] '
    read -r ans || exit 0
    case \"\$ans\" in
      1|2|3)
        mode=\$ans
        break
        ;;
      n|N|no|NO)
        break
        ;;
      *)
        echo '请输入 1、2、3 或 n'
        ;;
    esac
  done
  if [ -z \"\$mode\" ]; then
    echo '[SKIP] 未启动导航'
  else
    extra_args=''
    if [ \"\$mode\" = 2 ]; then
      kp='/home/unitree/3d_nav/src/scan_planner/src/planner/plan_manage/config/keypoints.example.yaml'
      printf 'Mode 2 keypoints 文件路径 [回车=默认 %s]: ' \"\$kp\"
      read -r kp_in || kp_in=''
      [ -n \"\$kp_in\" ] && kp=\"\$kp_in\"
      if [ ! -f \"\$kp\" ]; then
        echo \"[ERROR] keypoints 文件不存在: \$kp\"
        mode=''
      else
        echo \"[OK] keypoints: \$kp\"
        extra_args=\"keypoints_file:=\$kp\"
      fi
    elif [ \"\$mode\" = 3 ]; then
      rp='/home/unitree/3d_nav/src/scan_planner/src/planner/plan_manage/config/reference_path.map.yaml'
      printf 'Mode 3 参考路径来源 [回车=launch 内置发布器(%s) / e=外部节点发布 /initial_path]: ' \"\$rp\"
      read -r rp_in || rp_in=''
      if [ \"\$rp_in\" = e ] || [ \"\$rp_in\" = E ]; then
        echo '[OK] 使用外部节点发布 /initial_path'
      else
        [ -n \"\$rp_in\" ] && rp=\"\$rp_in\"
        if [ ! -f \"\$rp\" ]; then
          echo \"[ERROR] reference_path 文件不存在: \$rp\"
          mode=''
        else
          echo \"[OK] 参考路径: \$rp\"
          extra_args=\"reference_path_file:=\$rp\"
        fi
      fi
    fi
  fi
  if [ -n \"\$mode\" ]; then
    echo \"[OK] 启动 scan_planner Mode \$mode ...\"
    mkdir -p /home/unitree/3d_nav/logs
    ts=\$(date +%F_%H%M%S)
    log=/home/unitree/3d_nav/logs/planner_\${ts}.log
    echo \"[LOG] 日志保存到: \$log\"
    PYTHONUNBUFFERED=1 ros2 launch scan_planner run.launch.py \\
      is_real_world:=true \\
      navi_mode:=\$mode \\
      sensor_type:=lidar \\
      controller_mode:=closed_loop \$extra_args 2>&1 | tee -a \"\$log\"
    echo '=== scan_planner 已退出 ==='
  fi
  exec bash
"

echo "============================================"
echo " 节点已在 tmux session '$SESSION' 中启动"
echo " 0:livox 1:cmd_vel 2:mapping 3:navigation"
echo " 杀掉全部: tmux kill-session -t $SESSION"
echo "============================================"

tmux select-window -t "$SESSION:3"
tmux attach -t "$SESSION"

"""SCAN-Planner entry: Gazebo physics sim OR real robot (closed-loop only).

  # Gazebo (default) — needs ~/go2_sim_ws sourced so go2_sim_bringup is visible
  source ~/3d_nav/install/setup.bash
  source ~/go2_sim_ws/install/setup.bash
  ros2 launch scan_planner run.launch.py
  ros2 launch scan_planner rviz.launch.py use_sim_time:=true

  # Real robot — bring up LIO / drivers first
  ros2 launch scan_planner run.launch.py is_real_world:=true
"""

import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _bridge_ready_topic(gazebo_config):
    """rl_sar + auto_nav publishes /go2_sim/ready once the dog stands and follows /cmd_vel."""
    with open(gazebo_config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    rl = cfg.get("rl_sar") or {}
    if cfg.get("controller", "champ") == "rl_sar" and rl.get("auto_nav", True):
        return "/go2_sim/ready"
    return ""


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    bridge_yaml = os.path.join(scan_share, "config", "gazebo_bridge.yaml")
    default_gazebo_cfg = os.path.join(scan_share, "config", "gazebo_sim.yaml")

    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    gazebo_config = LaunchConfiguration("gazebo_config").perform(context) or default_gazebo_cfg
    start_gazebo = _as_bool(LaunchConfiguration("start_gazebo").perform(context))

    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    use_sim_time = not is_real
    common = {"use_sim_time": use_sim_time}

    if is_real:
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        cmd_vel = "/cmd_vel"
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = (
            "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        )
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = False
        need_extrinsic = False
        cmd_vel = "/quad_0/cmd_vel"
        intrinsics = {}

    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }

    actions = []

    if not is_real:
        if start_gazebo:
            try:
                bringup_share = get_package_share_directory("go2_sim_bringup")
            except PackageNotFoundError:
                actions.append(
                    LogInfo(
                        msg=(
                            "[run] ERROR: go2_sim_bringup not found. "
                            "source ~/go2_sim_ws/install/setup.bash (after 3d_nav), "
                            "or start_gazebo:=false and launch Gazebo yourself."
                        )
                    )
                )
                return actions
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_share, "launch", "sim.launch.py")
                    ),
                    launch_arguments={"config": gazebo_config}.items(),
                )
            )
        actions.extend(
            [
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_odom_static_tf",
                    output="screen",
                    arguments=[
                        "--x", "0", "--y", "0", "--z", "0",
                        "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        "--frame-id", "world",
                        "--child-frame-id", "odom",
                    ],
                    parameters=[common],
                ),
                Node(
                    package="scan_planner",
                    executable="gazebo_sim_bridge.py",
                    name="gazebo_sim_bridge",
                    output="screen",
                    parameters=[bridge_yaml, common,
                                {"ready_topic": _bridge_ready_topic(gazebo_config)}],
                ),
            ]
        )

    actions.append(
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml]
            + ([keypoints_file] if keypoints_file else [])
            + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    )

    if is_real:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                os.path.join(go2_share, "xacro", "robot.xacro"),
                                " use_gazebo:=false",
                            ]
                        )
                    },
                ],
            )
        )

    actions.append(
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[controllers_yaml, common]
            + ([] if is_real else [{"odom_twist_in_world": True}]),
            remappings=[
                ("body_pose", body_pose),
                ("cmd_vel", cmd_vel),
            ],
        )
    )

    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("initial_path", "/initial_path"),
                ],
            )
        )

    # 实机模式：以坐标轴 Marker 显示机器人位姿（100Hz 里程计降频到 10Hz 发布）
    if is_real:
        actions.append(
            Node(
                package="scan_planner",
                executable="robot_pose_visualizer",
                name="robot_pose_visualizer",
                output="screen",
                parameters=[
                    common,
                    {
                        "frame_id": "world",
                        "publish_rate": 10.0,
                        "axis_length": 0.3,
                        "axis_width": 0.02,
                    },
                ],
                remappings=[
                    ("body_pose", body_pose),
                    ("robot_axes", "/robot_axes"),
                ],
            )
        )

    mode = "real robot" if is_real else "Gazebo sim"
    actions.append(
        LogInfo(msg=f"[run] mode={mode} navi_mode={navi_mode} controller=closed_loop")
    )
    return actions


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    default_gazebo_cfg = os.path.join(scan_share, "config", "gazebo_sim.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "is_real_world",
                default_value="false",
                description="false=Gazebo+bridge; true=real robot (no Gazebo).",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="When sim: also launch go2_sim_bringup.",
            ),
            DeclareLaunchArgument(
                "gazebo_config",
                default_value=default_gazebo_cfg,
                description="YAML for go2_sim_bringup (lidar/world/spawn).",
            ),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("reference_path_file", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )

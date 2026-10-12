"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
Package: vbgc_robot
File: ros2_ws/src/vbgc_robot/launch/full_system.launch.py
Description: Universal production launch file. Discovers repository root
             dynamically, parameterizes hardware vs simulation targets, and
             spawns all vision, controller, and bridge nodes reliably.
===============================================================================
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import xacro


def find_repo_root(start_dir):
    """Walk up from the current directory to locate the git repository root."""
    current = os.path.abspath(start_dir)
    while current != os.path.dirname(current):
        if os.path.exists(os.path.join(current, ".git")) or os.path.exists(
            os.path.join(current, "vision", "cc_ll.py")
        ):
            return current
        current = os.path.dirname(current)
    # Default fallback to parent of ros2_ws
    return os.path.abspath(os.path.join(start_dir, "..", ".."))


def generate_launch_description():
    # Discover package share directory
    package_dir = get_package_share_directory("vbgc_robot")

    # Dynamically find repo root
    repo_root = find_repo_root(os.getcwd())
    vision_dir = os.path.join(repo_root, "vision")
    cc_ll_script = os.path.join(vision_dir, "cc_ll.py")

    # ========================================================
    # LAUNCH CONFIGURATIONS & ARGUMENTS
    # ========================================================
    use_sim = LaunchConfiguration("use_sim")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    camera_url = LaunchConfiguration("camera_url")
    python_exec = LaunchConfiguration("python_exec")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether to launch Gazebo simulator or physical rover hardware",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for ESP32 motor bridge",
    )

    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Baud rate for ESP32 serial communication",
    )

    declare_camera_url = DeclareLaunchArgument(
        "camera_url",
        default_value="0",
        description="Camera video source (e.g., '0' for local webcam or 'http://IP:8080/video')",
    )

    declare_python_exec = DeclareLaunchArgument(
        "python_exec",
        default_value=sys.executable,
        description="Python interpreter to invoke cc_ll.py (defaults to current runtime)",
    )

    # ========================================================
    # ROBOT DESCRIPTION (URDF via XACRO)
    # ========================================================
    xacro_file = os.path.join(package_dir, "description", "rover.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ========================================================
    # GAZEBO SIMULATION NODES (Active only if use_sim:=true)
    # ========================================================
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    gazebo_launch = os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
    world_file = os.path.join(package_dir, "worlds", "rover_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=IfCondition(use_sim),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "vbgc_rover",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.15",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # ========================================================
    # PHYSICAL ROVER HARDWARE NODES (Active if use_sim:=false)
    # ========================================================
    esp32_motor_bridge = Node(
        package="vbgc_robot",
        executable="esp32_motor_bridge",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "command_timeout": 0.5,
            }
        ],
        condition=UnlessCondition(use_sim),
    )

    # ========================================================
    # ROS 2 CORE CONTROL NODES
    # ========================================================
    gesture_publisher = Node(
        package="vbgc_robot",
        executable="gesture_publisher",
        output="screen",
    )

    camera_target_publisher = Node(
        package="vbgc_robot",
        executable="camera_target_publisher",
        output="screen",
    )

    following_controller = Node(
        package="vbgc_robot",
        executable="following_controller",
        output="screen",
    )

    # ========================================================
    # STANDALONE VISION PROCESS (cc_ll.py in vision/)
    # ========================================================
    vision_process = ExecuteProcess(
        cmd=[
            python_exec,
            cc_ll_script,
            "--camera",
            camera_url,
        ],
        cwd=vision_dir,
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim,
            declare_serial_port,
            declare_baud_rate,
            declare_camera_url,
            declare_python_exec,
            LogInfo(msg=f"[INFO] Discovered Repository Root: {repo_root}"),
            LogInfo(msg=f"[INFO] Launching Vision Controller from: {vision_dir}"),
            # Core ROS 2 nodes
            robot_state_publisher,
            gesture_publisher,
            camera_target_publisher,
            following_controller,
            # Hardware branch
            esp32_motor_bridge,
            # Simulation branch
            gazebo,
            spawn_robot,
            gz_bridge,
            # Vision process
            vision_process,
        ]
    )

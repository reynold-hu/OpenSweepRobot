#!/usr/bin/env python3
"""Bring up the benchmark: Gazebo Classic, the fixed scene, and the robot.

    ros2 launch sim/launch/coverage_test.launch.py

Run from the repository root. Paths are resolved from this file, so no package
install and no ament workspace is needed — but that also means `ros2 launch` must
be given the file path, not a package and launch-file name.

Spawns the robot at the scene's fixed start pose. That pose is not a preference:
the scene and the scoring tool both assume it, and moving it makes a run
incomparable with every other run.

Neither the world nor the robot description has ever been loaded by a simulator.
Expect this to need debugging on the first run; see sim/README.md.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
URDF = os.path.join(SIM, "description", "urdf", "robot.urdf.xacro")
WORLD = os.path.join(SIM, "worlds", "coverage_test.world")

# Must match START_POSE in sim/tools/check_scene.py.
START_X, START_Y, START_YAW = "-3.00", "0.60", "0.0"

# Dropped from slightly above the floor rather than placed exactly on it: the
# wheels settle on contact, and spawning flush tends to start the run with the
# body interpenetrating the floor.
START_Z = "0.05"


def generate_launch_description():
    # xacro args, exposed so a run can be repeated with a different odometry
    # source without editing files.
    odom_source = LaunchConfiguration("odom_source")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_imu = LaunchConfiguration("enable_imu")

    robot_description = ParameterValue(
        Command([
            "xacro ", URDF,
            " odom_source:=", odom_source,
            " enable_lidar:=", enable_lidar,
            " enable_imu:=", enable_imu,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom_source", default_value="ground_truth",
                              description="which source owns /odom and /tf"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="set false for a headless run"),
        DeclareLaunchArgument("verbose", default_value="false"),

        # Gazebo Classic. The world carries no plugins of its own, so it is just
        # geometry plus physics settings.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.environ.get("GAZEBO_ROS_SHARE", "/opt/ros/humble/share/gazebo_ros"),
                    "launch", "gazebo.launch.py",
                )
            ),
            launch_arguments={
                "world": WORLD,
                "gui": LaunchConfiguration("gui"),
                "verbose": LaunchConfiguration("verbose"),
            }.items(),
        ),

        # Publishes TF for every fixed joint and both wheels. The plugin file
        # does not publish base_footprint -> base_link itself; robot_state_publisher
        # does, from the URDF.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}],
        ),

        # spawn_entity waits for the service itself, but Gazebo still needs a
        # moment to finish loading the world before it will accept the model.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-topic", "robot_description",
                        "-entity", "oomwoo_one",
                        "-x", START_X, "-y", START_Y, "-z", START_Z,
                        "-Y", START_YAW,
                    ],
                )
            ],
        ),
    ])

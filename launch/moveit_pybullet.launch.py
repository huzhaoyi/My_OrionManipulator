#!/usr/bin/env python3
# MoveIt2 规划 + PyBullet 执行（VM 友好，无 Gazebo）。
# PyBullet 节点提供 orion_controller/follow_joint_trajectory 并发布 /joint_states。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("orion")
    urdf_path = os.path.join(pkg_share, "urdf", "orion.urdf")
    srdf_path = os.path.join(pkg_share, "config", "orion.srdf")
    controllers_path = os.path.join(pkg_share, "config", "moveit_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder("orion", package_name="orion")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = os.path.join(pkg_share, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    pybullet_node = Node(
        package="orion",
        executable="pybullet_follow_joint_trajectory_server.py",
        name="pybullet_follow_joint_trajectory_server",
        output="screen",
        parameters=[
            {"urdf_path": urdf_path},
            {"use_gui": LaunchConfiguration("pybullet_gui", default="true")},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pybullet_gui",
            default_value="true",
            description="Set true to show PyBullet GUI window.",
        ),
        move_group_node,
        rviz_node,
        static_tf,
        robot_state_publisher,
        pybullet_node,
    ])

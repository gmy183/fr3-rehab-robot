from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("parent_frame", default_value="fr3_link8"),
        DeclareLaunchArgument("xhand_xyz", default_value="0 0 0.081"),
        DeclareLaunchArgument("xhand_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("camera_xyz", default_value="-0.090 0 0.051"),
        DeclareLaunchArgument("camera_rpy", default_value="0 0 0"),
    ]

    xhand_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="xhand_grasp_center_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.081",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", LaunchConfiguration("parent_frame"),
            "--child-frame-id", "xhand_grasp_center",
        ],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435i_mount_tf",
        arguments=[
            "--x", "-0.090", "--y", "0", "--z", "0.051",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", LaunchConfiguration("parent_frame"),
            "--child-frame-id", "d435i_mount",
        ],
    )

    return LaunchDescription(args + [xhand_tf, camera_tf])

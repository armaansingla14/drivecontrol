from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("drivecontrol_ros"), "config", "params.yaml"])
    return LaunchDescription([
        Node(package="drivecontrol_ros", executable="drivecontrol_node", name="drivecontrol",
             parameters=[params], output="screen"),
    ])

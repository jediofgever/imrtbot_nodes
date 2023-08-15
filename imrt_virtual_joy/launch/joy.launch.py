from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    imrt_virtual_joy_node = Node(
        package="imrt_virtual_joy", executable="virtual_gamepad", name="virtual_gamepad"
    )

    ld.add_action(imrt_virtual_joy_node)

    return ld

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    imrt_teleop_node = Node(
        package="imrt_teleop",
        executable="imrt_teleop_node",
        name="imrt_teleop_node",
        remappings=[("teleop/cmd_vel", "/AGV0/vox_nav/cmd_vel")],
    )

    imrt_virtual_joy_node = Node(
        package="imrt_virtual_joy", executable="virtual_gamepad", name="virtual_gamepad"
    )

    ld.add_action(imrt_teleop_node)
    ld.add_action(imrt_virtual_joy_node)

    return ld

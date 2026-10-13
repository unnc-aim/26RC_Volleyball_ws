import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('volleyball_controller'), 'config', 'dm_volleyball.yaml')

    chassis_node = Node(
        package='chassis_controllers',
        executable='dji_full_wheel_chassis_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    stewart_node = Node(
        package='volleyball_controller',
        executable='dm_stewart_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    hub_node = Node(
        package='volleyball_controller',
        executable='volleyball_hub_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([
        chassis_node,
        stewart_node,
        hub_node
    ])

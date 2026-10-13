from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('soem_bringup'),
        'config',
        'config-vol-lk.yaml'
    )

    return LaunchDescription([
        Node(
            package='soem_wrapper',
            executable='soem_backend',
            name='soem_backend',
            parameters=[{
                'interface': "enp196s0",
                'rt_cpu': 19,
                'non_rt_cpus': "0-18",
                'config_file': config_file
            }],
            output='screen'
        )
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution

def generate_launch_description():
    pkg_soem_bringup = get_package_share_directory('soem_bringup')
    pkg_vision = get_package_share_directory('volleyball_vision')
    pkg_tracker = get_package_share_directory('volleyball_tracker')
    pkg_planner = get_package_share_directory('volleyball_planner')
    pkg_controller = get_package_share_directory('volleyball_controller')

    # Launch configurations
    launch_vision = LaunchConfiguration('launch_vision')
    vision_engine = LaunchConfiguration('vision_engine')
    launch_tracker = LaunchConfiguration('launch_tracker')
    launch_planner = LaunchConfiguration('launch_planner')
    launch_controller = LaunchConfiguration('launch_controller')
    launch_soem = LaunchConfiguration('launch_soem')

    # Declare arguments
    declare_vision_arg = DeclareLaunchArgument(
        'launch_vision',
        default_value='true',
        description='Whether to launch the volleyball vision node'
    )
    declare_vision_engine_arg = DeclareLaunchArgument(
        'vision_engine',
        default_value='python',
        description='Vision engine: "python" (recommended 67 FPS PyTorch) or "cpp" (C++ OpenCV DNN)'
    )
    declare_tracker_arg = DeclareLaunchArgument(
        'launch_tracker',
        default_value='true',
        description='Whether to launch the volleyball EKF tracker node'
    )
    declare_planner_arg = DeclareLaunchArgument(
        'launch_planner',
        default_value='true',
        description='Whether to launch the volleyball trajectory planner node'
    )
    declare_controller_arg = DeclareLaunchArgument(
        'launch_controller',
        default_value='true',
        description='Whether to launch the robot chassis & stewart controllers'
    )
    declare_soem_arg = DeclareLaunchArgument(
        'launch_soem',
        default_value='false',
        description='Whether to launch the EtherCAT SOEM backend (enable when connected to hardware)'
    )

    # 1. Vision Engine selection
    python_script_path = '/home/aerialstewart/26_RC_Volleyball/ros2_stereo_vision_node.py'
    python_vision = ExecuteProcess(
        cmd=['python3', python_script_path],
        output='screen',
        condition=IfCondition(EqualsSubstitution(vision_engine, 'python'))
    )
    cpp_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vision, 'launch', 'vision.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(vision_engine, 'cpp'))
    )
    vision_group = GroupAction(
        actions=[python_vision, cpp_vision],
        condition=IfCondition(launch_vision)
    )

    # 2. Tracker launch
    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tracker, 'launch', 'tracker.launch.py')
        ),
        condition=IfCondition(launch_tracker)
    )

    # 3. Planner launch
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_planner, 'launch', 'planner.launch.py')
        ),
        condition=IfCondition(launch_planner)
    )

    # 4. Controller launch (chassis, stewart, hub)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, 'launch', 'dm_volleyball.launch.py')
        ),
        condition=IfCondition(launch_controller)
    )

    # 5. SOEM EtherCAT backend launch
    soem_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_soem_bringup, 'launch', 'bringup.launch.py')
        ),
        condition=IfCondition(launch_soem)
    )

    return LaunchDescription([
        declare_vision_arg,
        declare_vision_engine_arg,
        declare_tracker_arg,
        declare_planner_arg,
        declare_controller_arg,
        declare_soem_arg,
        LogInfo(msg="====================================================="),
        LogInfo(msg="  Starting Volleyball Robot Full Autonomy Stack...   "),
        LogInfo(msg="====================================================="),
        vision_group,
        tracker_launch,
        planner_launch,
        controller_launch,
        soem_launch
    ])

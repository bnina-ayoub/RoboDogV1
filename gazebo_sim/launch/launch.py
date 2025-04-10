import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    
    # Set Gazebo model paths
    models_path = os.path.join(pkg_path, 'models')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path
    )
    ld.add_action(gazebo_model_path)
    
    # For Gazebo Ignition/Fortress
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )
    ld.add_action(gz_sim_resource_path)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Использовать симуляционное время'
    )

    ld.add_action(declare_use_sim_time)

    world_file = os.path.join(pkg_path, 'world', 'cafe.world') 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gazebo)

    pause = ExecuteProcess(
        cmd=['sleep', '6'],
        output='screen'
    )
    ld.add_action(pause)

    # Добавляем запуск gazebo_multi_nav2_world.launch.py после ожидания
    multi_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo_multi_nav2_world.launch.py')
        )
    )

    launch_after_pause = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pause,
            on_exit=[multi_nav2_launch]
        )
    )

    ld.add_action(launch_after_pause)

    return ld

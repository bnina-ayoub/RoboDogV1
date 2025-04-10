from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='robot1',
        description='Namespace for the robot to control'
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    ps5_teleop_node = Node(
        package='ps5_controller',
        executable='ps5_teleop',
        name='ps5_teleop',
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
        }]
    )
    
    return LaunchDescription([
        namespace_arg,
        joy_node,
        ps5_teleop_node
    ])

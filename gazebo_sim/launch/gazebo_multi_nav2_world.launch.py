import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap, ComposableNodeContainer
from launch.event_handlers import OnProcessExit
import xacro, yaml


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    robots = yaml_data['robots']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Использовать симуляционное время'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_use_sim_time)

    remappings_initial = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/odom", "odometry/filtered")
    ]
    
    map_server = Node(package='nav2_map_server',
                      executable='map_server',
                      name='map_server',
                      output='screen',
                      parameters=[{'yaml_filename': os.path.join(pkg_path, 'maps', 'cambridge.yaml'),
                                   }, ],
                      remappings=remappings_initial)

    map_server_lifecycle = Node(package='nav2_lifecycle_manager',
                                executable='lifecycle_manager',
                                name='lifecycle_manager_map_server',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time},
                                            {'autostart': True},
                                            {'node_names': ['map_server']}])

    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecycle)


    remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan", "scan"),
            ("/odom", "odometry/filtered")
        ]
    

    bridge_params = os.path.join(pkg_path,'config','gz_bridge.yaml')
    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ld.add_action(ros_gz_bridge_clock)   

    
    last_action = None

    for i, robot in enumerate(robots):
        namespace = robot['name']
        robot_name = robot['name']
        xacro_file = os.path.join(os.path.join(get_package_share_directory('go2_description')), 'xacro', 'robot.xacro') ## CHANGE ME!!!!
        robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name}).toxml()
        params_robot_state_publisher = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params_robot_state_publisher],
            remappings=remappings
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-topic', f'/{namespace}/robot_description',
                '-name', f'{namespace}_my_bot',
                '-allow_renaming', 'true',
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', robot['z_pose'],
                # '-Y', robot['Y_pose']
            ],
            output='screen'
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                f'/{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                f'/{namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/{namespace}/color/image_rect@sensor_msgs/msg/Image@gz.msgs.Image',
                # f'/{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ]
        )
        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            namespace=namespace,
            arguments=['color/image_raw', 'color/image_rect'],
            output='screen',
        )


        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_state_broadcaster',
            arguments=['joint_state_broadcaster'],
            output='screen',
            remappings=remappings
        )

        joint_group_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_group_controller',
            arguments=['joint_group_controller'],
            output='screen',
            remappings=remappings
        )

        controller = Node(
            package='quadropted_controller',
            executable='robot_controller_gazebo.py',
            name='quadruped_controller',
            namespace=namespace,
            output='screen',
            remappings=remappings
        )

        # apriltag_launch_file = os.path.join(get_package_share_directory('yahboom_rosmaster_docking'), 'launch', 'apriltag_dock_pose_publisher.launch.py')
        # aprilTag = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(apriltag_launch_file),
        #     launch_arguments={
        #         'camera_namespace': namespace,
        #         'camera_frame_type': 'camera_face'
        #     }.items()
        # )



        odom = Node(
            package='quadropted_controller',
            executable='QuadrupedOdometryNode.py',
            name='odom',
            namespace=namespace,
            output='screen',
            parameters=[{
                "verbose": False,
                'publish_rate': 100,
                'open_loop': False,
                'has_imu_heading': True,
                'is_gazebo': True,
                'imu_topic': f'/{namespace}/imu',
                'base_frame_id': "base_link",
                'odom_frame_id': "odom",
                'clock_topic': f'/clock',
                'enable_odom_tf': True,
            }],
            remappings=remappings
        )

        nav2_launch_file = os.path.join(pkg_path, 'launch', 'nav2', 'bringup_launch.py')
        map_yaml_file = os.path.join(pkg_path, 'maps', 'cafe_world_map.yaml')
        params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

        message = f"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {robot['x_pose']}, y: {robot['y_pose']}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, }} }}"

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                f'/{namespace}/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_yaml_file,
                'use_namespace': 'True',
                'namespace': namespace,
                'params_file': params_file,  
                'autostart': 'true',
                'use_sim_time': 'true',
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'rviz', 'nav2_default_view.rviz')

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items(),
            condition=IfCondition(enable_rviz)
        )

        cmd_vel_pub = Node(
            package='quadropted_controller',
            executable='cmd_vel_pub.py',
            namespace=namespace,
            name='cmd_vel_pub',
            output='screen',
            remappings=remappings
        )

        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', f'/{namespace}/battery_state', 'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
                '-r', '1'
            ],
            output='log'
        )
        robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
        # Start robot localization using an Extended Kalman filter
        start_robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[robot_localization_file_path, 
            {'use_sim_time': use_sim_time}],
            remappings=remappings)

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            namespace=namespace,
            name='smoother_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        )

        robot_control = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            joint_state_broadcaster,
            joint_group_controller,
            controller,
            cmd_vel_pub,
            odom,
            start_robot_localization_cmd,
            # aprilTag,
            fake_bms,
        ])
        test_action = Node(
            package='gazebo_sim',
            executable='test_action.py',
            namespace=namespace,
            name='test_action',
            output='screen',
            remappings=remappings
        )

        # Группировка всех действий для робота
        robot_group = GroupAction([
            node_robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            start_gazebo_ros_image_bridge_cmd,
            robot_control,
            nav2_actions,
            rviz,
            test_action,
            smoother_server
        ])

        if last_action is None:
            ld.add_action(robot_group)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_group]
                )
            )
            ld.add_action(spawn_robot_event)

        last_action = joint_group_controller

    return ld
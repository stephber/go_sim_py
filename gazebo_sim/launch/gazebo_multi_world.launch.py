import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.event_handlers import OnProcessExit
import xacro, yaml


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # Load robots from YAML
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots = yaml_data['robots']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable RViz launch'
    )

    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_use_sim_time)

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/odom", "odometry/filtered")
    ]

    # ROS-Gazebo bridge for clock
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

        xacro_file = os.path.join(get_package_share_directory('go2_description'), 'xacro', 'robot.xacro')
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
            ]
        )

        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            namespace=namespace,
            arguments=['color/image_raw', 'color/image_rect'],
            output='screen',
        )

        # Controllers
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

        cmd_vel_pub = Node(
            package='quadropted_controller',
            executable='cmd_vel_pub.py',
            namespace=namespace,
            name='cmd_vel_pub',
            output='screen',
            remappings=remappings
        )

        odom = Node(
            package='quadropted_controller',
            executable='QuadrupedOdometryNode.py',
            name='odom',
            namespace=namespace,
            output='screen',
            parameters=[{
                "verbose": False,
                'publish_rate': 50,
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

        # Fake BMS
        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', f'/{namespace}/battery_state', 'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
                '-r', '1'
            ],
            output='log'
        )

        robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
        start_robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}],
            remappings=remappings
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
            fake_bms,
        ])

        # RViz
        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'rviz', 'nav2_default_view.rviz')
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name=f'{namespace}_rviz',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen'
        )

        robot_group = GroupAction([
            node_robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            start_gazebo_ros_image_bridge_cmd,
            robot_control,
            # Uncomment next line to launch RViz per robot
            # rviz
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

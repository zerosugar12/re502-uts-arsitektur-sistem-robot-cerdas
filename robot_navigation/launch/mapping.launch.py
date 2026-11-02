import os

from ament_index_python.packages import get_package_share_directory

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

START_X, START_Y, START_Z, START_YAW = '1.4', '-1.2', '0.08', '1.5708'
HEADLESS = False


def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_description_dir = get_package_share_directory('robot_description')

    world = os.path.join(robot_description_dir, 'worlds', 'track.sdf')
    controllers_yaml = os.path.join(robot_navigation_dir, 'config', 'controllers.yaml')
    bridge_yaml = os.path.join(robot_navigation_dir, 'config', 'bridge.yaml')
    rviz_config = os.path.join(robot_navigation_dir, 'rviz', 'navigation.rviz')
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf')

    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        robot_description_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    with open(urdf_file) as f:
        robot_description = f.read()

    robot_description_spawn = (
        robot_description
        .replace('package://robot_description', 'file://' + robot_description_dir)
        .replace('__CONTROLLERS_YAML__', controllers_yaml))

    gz_args = (world + ' -s -r') if HEADLESS else (world + ' -r')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-string', robot_description_spawn,
            '-x', START_X,
            '-y', START_Y,
            '-z', START_Z,
            '-Y', START_YAW,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-ros-args', '-r ~/odom:=/odom',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner],
    )

    with open(bridge_yaml) as f:
        bridge_config = yaml.safe_load(f)

    direction = {'GZ_TO_ROS': '[', 'ROS_TO_GZ': ']', 'BIDIRECTIONAL': '@'}
    bridge_args = []
    remap = []
    for entry in bridge_config:
        bridge_args.append(
            f"{entry['gz_topic_name']}@{entry['ros_type_name']}"
            f"{direction[entry['direction']]}{entry['gz_type_name']}")
        if entry['ros_topic_name'] != entry['gz_topic_name']:
            remap += ['-r', f"{entry['gz_topic_name']}:={entry['ros_topic_name']}"]
    if remap:
        bridge_args += ['--ros-args', *remap]

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
    )

    return LaunchDescription([
        use_rviz,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        spawners,
        parameter_bridge,
        rviz,
    ])

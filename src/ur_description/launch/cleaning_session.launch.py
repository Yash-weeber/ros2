# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command
# from launch_ros.actions import Node
#
# def generate_launch_description():
#     ur_description_path = get_package_share_directory('ur_description')
#     xacro_file = os.path.join(ur_description_path, 'urdf', 'ur_table_assembly.urdf.xacro')
#
#     robot_description = {'robot_description': Command(['xacro ', xacro_file])}
#
#     # Essential Clock Bridge for Jazzy
#     node_gz_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
#         output='screen'
#     )
#
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description, {'use_sim_time': True}]
#     )
#
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
#         launch_arguments={'gz_args': '-r empty.sdf'}.items(),
#     )
#
#     gz_spawn_entity = Node(
#         package='ros_gz_sim',
#         executable='create',
#         output='screen',
#         arguments=['-topic', 'robot_description', '-name', 'ur5e_mop', '-z', '0.0'],
#     )
#
#     jsb_spawner = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
#     jtc_spawner = Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller'])
#
#     # The Home Mover Script
#     home_mover_node = Node(
#         package='ur_description',
#         executable='test.py',
#         output='screen',
#         parameters=[{'use_sim_time': True}]
#     )
#
#     return LaunchDescription([
#         node_robot_state_publisher,
#         gz_sim,
#         gz_spawn_entity,
#         node_gz_bridge,
#
#         TimerAction(period=5.0, actions=[jsb_spawner]),
#         TimerAction(period=7.0, actions=[jtc_spawner]),
#         TimerAction(period=12.0, actions=[home_mover_node]),
#     ])
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    ur_description_path = get_package_share_directory('ur_description')
    # xacro_file = os.path.join(ur_description_path, 'urdf', 'ur_table_assembly.urdf.xacro')

    xacro_file = os.path.join(ur_description_path, 'urdf', 'ur_mop_standalone.urdf.xacro')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file,' use_gazebo:=true']),
            value_type=str
        )
    }

    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'ur_table_assembly', '-z', '0.0'],
    )

    # Spawn Controllers
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    ur1_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller']
    )

    ur2_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_2_controller']
    )
    # ur2_gripper_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['ur_2_gripper_controller']
    # )
    # NOTE: test.py is REMOVED from this list
    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        node_gz_bridge,
        TimerAction(period=5.0, actions=[jsb_spawner]),
        TimerAction(period=7.0, actions=[ur1_spawner, ur2_spawner]),#,ur2_gripper_spawner]),
    ])
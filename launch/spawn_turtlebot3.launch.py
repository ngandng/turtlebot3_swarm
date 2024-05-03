import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    # Launch configuration variables specific to simulation
    robot_prefix_arg = DeclareLaunchArgument('robot_prefix', default_value='')
    robot_prefix = LaunchConfiguration('robot_prefix')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='')
    use_sim_time = LaunchConfiguration('use_sim_time')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    x_pose = LaunchConfiguration('x_pose')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose')


    # Obtain urdf from xacro files.
    turtlebot3_swarm_pkg_dir = get_package_share_directory('turtlebot3_swarm')
    xacro_file_path = os.path.join(turtlebot3_swarm_pkg_dir, 'urdf', 'turtlebot3_waffle.urdf.xacro')
    robot_desc = Command(['xacro ', str(xacro_file_path), ' frame_prefix:=', robot_prefix, ' topic_prefix:=', robot_prefix])

    # Robot state publisher
    # This node will take the urdf description and:
    # - publish the transforms using the prefix set by the frame_prefix parameter.
    # - publish the robot description under the set namespace.
    # - subscribe to joint states under the set namespace.
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot_prefix,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix':
                    PythonExpression(["'", LaunchConfiguration('robot_prefix'), "/'"])
            }],
        )

    # Spawn robot
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', PathJoinSubstitution([robot_prefix, 'waffle']),
            '-topic', PathJoinSubstitution([robot_prefix, 'robot_description']),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(robot_prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher)

    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld

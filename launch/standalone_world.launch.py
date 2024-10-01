import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

# x, y, theta
# STARTS = [[-0.5, 0.0, 0.0],
#           [-0.5, 1.5, 0.0],
#           [-1.0, 1.0, 0.0]]

STARTS = [[0.0, 0.0, 0.0]]
use_rviz = True

pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
pkg_turtlebot3_swarm = get_package_share_directory('turtlebot3_swarm')

def generate_launch_description():
    ld = LaunchDescription()

    #######################
    ## Environment setup ##
    #######################
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false',
                          description='Open Gazebo in verbose mode.')
    verbose = LaunchConfiguration('verbose')

    world_name = LaunchConfiguration('world_name')
    world_name_arg = DeclareLaunchArgument(
          'world_name',
          default_value='empty_world.world',
          description='SDF world file name. [empty_world.world or turtlebot3_world.world]')

    # Includes gazebo_ros launch for gazebo
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': PathJoinSubstitution([pkg_turtlebot3_swarm,'worlds', world_name]),
              'verbose': verbose,
              'gui': 'true',
          }.items()
    )

    # Add the commands to the launch description
    ld.add_action(verbose_arg)
    ld.add_action(world_name_arg)
    ld.add_action(include_gazebo)

    #######################
    ## Multi robot spawn ##
    #######################
    # Obtain urdf from xacro files.
    xacro_file_path = os.path.join(pkg_turtlebot3_swarm, 'urdf', 'turtlebot3_waffle.urdf.xacro')

    # Spawn robot
    for index in range(len(STARTS)):
        x_pose = STARTS[index][0]
        y_pose = STARTS[index][1]
        robot_prefix = "robot_{}".format(index)

        robot_desc = Command(['xacro ', str(xacro_file_path), ' frame_prefix:=', robot_prefix, ' topic_prefix:=', robot_prefix])
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot_prefix,
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
                'frame_prefix': robot_prefix + "/"
            }],
        )

        # Spawn robot
        start_gazebo_ros_spawner_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', PathJoinSubstitution([robot_prefix, 'waffle']),
                '-topic', PathJoinSubstitution([robot_prefix, 'robot_description']),
                '-x', str(x_pose),
                '-y', str(y_pose),
                '-z', '0.01'
            ],
            output='screen',
        )

        broadcaster = Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='broadcaster'+str(index),
            arguments = ["0", "0", "0", "0", "0", "0", "world", "{}/odom".format(robot_prefix)]
        )

        # Controller
        controller = Node(
            package='turtlebot3_swarm',
            executable='move_to_target_node',
            name='controller'+str(index),
            parameters=[{
                'name': robot_prefix
            }],
        )

        # Add the commands to the launch description
        ld.add_action(robot_state_publisher)
        ld.add_action(start_gazebo_ros_spawner_cmd)
        ld.add_action(broadcaster)
        ld.add_action(controller)

    if use_rviz:
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', [os.path.join(pkg_turtlebot3_swarm, 'rviz', 'config.rviz')]]
        )
        ld.add_action(rviz)

    return ld

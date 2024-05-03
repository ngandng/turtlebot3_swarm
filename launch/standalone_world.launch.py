import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PathJoinSubstitution

pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
pkg_turtlebot3_swarm = get_package_share_directory('turtlebot3_swarm')

def generate_launch_description():
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

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(verbose_arg)
    ld.add_action(world_name_arg)
    ld.add_action(include_gazebo)
    return ld

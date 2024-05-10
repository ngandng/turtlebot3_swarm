# ROS2 Turtlebot 3 Waffle on Gazebo

### ROS2 Topic
- `<robot_prefix>/robot_description`
- `<robot_prefix>/joint_states`
- `<robot_prefix>/cmd_vel`
- `<robot_prefix>/odom`
- `<robot_prefix>/scan`

## Prerequisites
```
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-pkgs
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-turtlebot3-msgs
```

## Installation
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone git@github.com:duynamrcv/turtlebot3_swarm.git
cd ~/turtlebot3_ws/
colcon build --symlink-install
```

## Demo
* Launch the empty environment with 3 Waffle robots
```
ros2 launch turtlebot3_swarm sample_swarm.launch.py 
```

* Send a sample command
```
ros2 run turtlebot3_swarm turtlebot3_controller.py --ros-args -p name:=robot_0
```
## Custom your space

- Launch the standalone simulation, no robot is spawned
```
ros2 launch turtlebot3_swarm standalone_world.launch.py
```

Note: By default the empty world is used, you can also use `world_name:=turtlebot3_world.world`.

- Spawn a robot using a `robot_prefix` in a particular `x_pose` and `y_pose`.
```
ros2 launch turtlebot3_swarm spawn_turtlebot3.launch.py robot_prefix:=robot1 x_pose:=0.5 y_pose:=0.5
```
The `robot_prefix` will affect both the ros namespace and the tf prefix for that robot

- Spawn another robot in a different position

```
ros2 launch turtlebot3_swarm spawn_turtlebot3.launch.py robot_prefix:=robot2 x_pose:=-0.5 y_pose:=-0.5
```

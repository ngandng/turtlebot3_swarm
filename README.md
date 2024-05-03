# ROS2 Turtlebot 3 Waffle on Gazebo

### ROS2 Topic
- `<robot_prefix>/robot_description`
- `<robot_prefix>/joint_states`
- `<robot_prefix>/cmd_vel`
- `<robot_prefix>/odom`
- `<robot_prefix>/scan`


## Demo
* Launch the empty environment with 3 Waffle robots
```
ros2 launch turtlebot3_swarm sample_swarm.launch.py 
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

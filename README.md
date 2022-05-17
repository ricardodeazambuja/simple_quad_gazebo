# simple_quad_gazebo
Very simple quadcopter for Gazebo using ROS2 Galactic 

```
colcon build --symlink-install
```

```
. install/setup.bash
```

```
ros2 launch simple_quad display.launch.py
```

Move to Z=0.5:
```
ros2 topic pub /simple_quad/cmd_pos geometry_msgs/msg/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.5
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" -1
```

Spin around Z with velocity 0.5rad/s:
```
ros2 topic pub /simple_quad/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -1
```
# simple_quad_gazebo
Very simple quadcopter for Gazebo using ROS2 Galactic. It's just a very cheap and dirty adaptation of a pre-existent Gazebo plugin called "hand of god", but now it looks like a quadcopter and you can send velocity commands too. In addition to that, it makes the quad **look** a little bit more realistic by **faking** the pitch and roll angles according to the acceleration. All settings are easy to understand, just check the [simple_quad.urdf](src/simple_quad/src/description/simple_quad.urdf).     
I wrote this because I wanted something faster than the quad plugins available for Gazebo, but I wrote the code without worried about writing an optimized code and I never tested it against anything to confirm it's really faster than those plugins... LOL.

The easiest way to test it is by using docker and [this script](https://github.com/ricardodeazambuja/ros2-playground/blob/main/launch_ros2_desktop.sh):
```
launch_ros2_desktop.sh -g -d . --image ricardodeazambuja/ros2-galactic-desktop
```


```
colcon build --symlink-install
```

```
. install/setup.bash
```

```
ros2 launch simple_quad display.launch.py
```

```
ros2 launch simple_quad slam.launch.py 
```

```
ros2 launch simple_quad navigation.launch.py
```

Move to Z=0.5:
```
ros2 topic pub /cmd_pos geometry_msgs/msg/Pose "position:
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
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -1
```

Control it using the keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r teleop_twist_keyboard:cmd_vel:=/cmd_vel
```

## Notes
* ExecuteProcess has a very useful parameter called `respawn` that relaunches a process that abnormally died (and you can control the delay with `respawn_delay`). I added that to te `display.launch.py` and it helped launching Gazebo more reliably.
* Next step is to use the (Event Handlers)[https://docs.ros.org/en/rolling/Tutorials/Launch/Using-Event-Handlers.html]

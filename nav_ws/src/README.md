# Wormhole Navigation

## Requirements

### OS & Middleware

- Ubuntu 22.04
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

### ROS 2 Packages

``` bash
sudo apt install \
  ros-humble-turtlebot3* \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

### Python Packages

```sudo apt install python3-sqlite3```

---

## Running the code

- Copy the package into workspace source directory
- ```colcon build```
- in case of any error try ```rosdep install --from-paths src --ignore-src -r -y```
- For gazebo ```export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py```
- ```source ./install/setup.bash && ros2 launch anscer_robotics multi_map_nav.launch.py```
- for testing ```source ./install/setup.bash && ros2 run anscer_robotics test_navigator```

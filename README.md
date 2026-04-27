# ros2_project_sc23ww

In this, the robot should go around hte map and explore and as soon as it sees the blue box and it fills up a certain amount of the screen, the robot stops.

# Starting up

cd ~/ros2_2proj
colcon build --packages-select ros2_project_sc23ww
source install/setup.bash


## Starting Gazebo

cd ~/ros2_2proj
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py

## Start Nav2 with the map

cd ~/ros2_2proj
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_2proj/map/map.yaml

## node running
ros2 run ros2_project_sc23ww project_node
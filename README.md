
# Task-2

Turtlebot3 navigation using a custom rrt global planner pluggin.



## Deployment

 - create a workspace and a src folder in it go to the src directory and git clone the repo

```bash
  git clone https://github.com/avirupghoshiitmandi/task-2-.git

```
- build your workspace
```bash
 cd ..
 catkin_make
 ```
 - launch the gazebo world
 ```bash
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
  ```

- now the already generated map is in the turtlebot3_navigation/maps directory which can be used for navigation and also make sure the move_base global planner is set to the right class->  global_planner/RRTGlobalPlanner

- launch the navigation launch file
```bash
 roslaunch turtlebot3_navigation turtlebot3_navigation.launch
 ```
- check on the topics and see if the global_planner/rrt-global_planner topic is there(the exact name of the topic would be something similar) now using the 2d nav goal in rviz select a point you will see the global_planner generate a path which is followed by the local DWA planner.
- ***info***  
  the planner_frequency in move_base launch file is set to 0.0 so only one global_planner path is created for the entire navigation untill new 2d goal is given .
## Acknowledgements

 - [rrt-global-planner(used for header and cpp files and optimized accordingly)](https://github.com/mech0ctopus/rrt-global-planner)


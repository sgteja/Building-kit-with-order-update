# Building the kit with order update.

# Todo

- Conveyer not used
- Parts are in bins
- No flipped parts 
- No dropped parts
- No sensor blackout

- Specific AGV for delivery.
- Faulty parts are present.
- Run in competition node.




## Running the package:

- Unpack the zip file "group3_rwa4.zip" into "group3_rwa4" folder into the workspace src directory.
- Open two terminals.
- In terminal-1:
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
roslaunch group3_rwa4 group3_rwa4.launch
```
- In terminal-2:
```
source /opt/ros/melodic/setup.bash
source ~/ariac_ws/install/setup.bash (The ariac workspace where moveit package is present)
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```
- In terminal-3:
```
source /opt/ros/melodic/setup.bash
catkin_make --only-pkg-with-deps group3_rwa4
source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
rosrun group3_rwa4 ariac_example_node
```

	

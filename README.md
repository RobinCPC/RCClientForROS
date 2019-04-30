# RCClientForROS
MiniBOT Remote Control Client for ROS

## To simulate minibot in Gazebo, run the following command in order
``` bash
# To display in Gazebo Simulation.
roslaunch minibot_gazebo minibot_world.launch
# Connect with ros control
roslaunch minibot_control minibot_control.launch
# a function publish joint control message
rosrun minibot_control simple_mover
```

### TODO:
* Check the range and direction of joint (J2 - 5)


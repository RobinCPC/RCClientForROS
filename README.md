# RCClientForROS
MiniBOT Remote Control Client for ROS

## To simulate MiniBOT in Gazebo, run the following command in order
``` bash
# To display in Gazebo Simulation.
roslaunch minibot_gazebo minibot_world.launch
# Connect with ros control
roslaunch minibot_control minibot_control.launch
```

## To operate real MiniBOT, run the following command in order
``` bash
# To connect with real MiniBOT and open Rviz
roslaunch subscriber_mini_bot start_rviz_demo.launch
# After connecting and homing, ready to send joint command
rosrun minibot_control simple_mover
```

## To simulate MiniBOT in Gazebo with Moveit!
``` bash
roslaunch minibot_control simulate_with_moveit.launch
```

### TODO:
* Check the range and direction of joint (J2 - 5)
* Update urdf mesh models


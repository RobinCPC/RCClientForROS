# RCClientForROS
MiniBOT Remote Control Client for ROS

## How to build
``` bash
# Got to your workspace's src and clone the repo. For instance
cd $HOME/catkin_ws/src
git clone https://github.com/RobinCPC/RCClientForROS.git
# use rosdep install dependent packages
cd $HOME/catkin_ws/
rosdep install --from-paths src --ignore-src --rosdistro kinetic # or melodic
# use catkin_tools to build packages in RCClientForROS
cd $HOME/catkin_ws/
catkin build  # may take a while
# after buld complete, source the devel folder
source ./devel/setup.bash  # or setup.zsh if you use zsh
```

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


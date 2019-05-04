#include <vector>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

// define a lambda function to expand vector to string
auto glambda = [](std::vector<double>& v) -> std::string{
  std::string tmp="";
  for(auto i : v)
  {
    tmp += std::to_string(i) + " ";
  }
  return tmp;
};

void send_command(ros::Publisher& pub, ros::Rate& r)
{
  const double deg = 0.017453293;
  std::vector<double> jnt_vals = {0., 0., 0., 0., -90*deg, 0.}; // home position
  std::vector<std::vector<double>> jnt_arr = {};
  //jnt_arr.push_back(jnt_vals);
  // move through 4 points
  jnt_arr.push_back({-21.841*deg, (85.523 - 90)*deg, 5.052*deg, 0.149*deg, -90.203*deg, -21.836*deg});
  jnt_arr.push_back({21.841*deg, (85.523 - 90)*deg, 5.052*deg, -0.149*deg, -90.203*deg, 21.841*deg});
  jnt_arr.push_back({21.841*deg, (80.521 - 90)*deg, -34.90*deg, -0.201*deg, -45.243*deg, 21.989*deg});
  jnt_arr.push_back({-21.841*deg, (80.521 -90)*deg, -34.906*deg, 0.201*deg, -45.242*deg, -21.985*deg});
  std_msgs::Float64MultiArray msg;

  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = jnt_vals.size();
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "Joint";

  int iter = 0;
  while(ros::ok())
  {
    iter %= jnt_arr.size();
    jnt_vals = jnt_arr[iter];
    //copy in the data
    ROS_INFO_STREAM("Command joint values:");
    ROS_INFO_STREAM( glambda(jnt_vals) + "\n");
    msg.data.clear();
    msg.data.insert(msg.data.end(), jnt_vals.begin(), jnt_vals.end());
    pub.publish(msg);

    ++iter;
    r.sleep();
  }

  return;
}

int main(int argc, char *argv[])
{
  // initialize
  ros::init(argc, argv, "simple_mover");
  ros::NodeHandle nh;
  ros::Rate rate = ros::Rate(0.25);

  ros::Publisher pub_group = nh.advertise<std_msgs::Float64MultiArray>
    ("/minibot/joint_position_controller/command",  5);
  send_command(pub_group, rate);

  ros::spin();

  return 0;
}

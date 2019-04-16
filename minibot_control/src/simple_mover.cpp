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
  std::vector<double> jnt_vals = {M_PI*0.25, -1.57, 1., 1., 0.5, fabs(M_PI*-0.25)};
  std_msgs::Float64MultiArray msg;

  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = jnt_vals.size();
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "Joint";

  while(ros::ok())
  {
    //copy in the data
    ROS_INFO_STREAM("Command joint values:");
    ROS_INFO_STREAM( glambda(jnt_vals) + "\n");
    msg.data.clear();
    msg.data.insert(msg.data.end(), jnt_vals.begin(), jnt_vals.end());
    pub.publish(msg);

    for(auto& j : jnt_vals)
    {
      j *= -1;
    }
    r.sleep();
  }

  return;
}

int main(int argc, char *argv[])
{
  // initialize
  ros::init(argc, argv, "simple_mover");
  ros::NodeHandle nh;
  ros::Rate rate = ros::Rate(0.1);

  ros::Publisher pub_group = nh.advertise<std_msgs::Float64MultiArray>
    ("/minibot/joint_position_controller/command",  5);
  send_command(pub_group, rate);

  ros::spin();

  return 0;
}

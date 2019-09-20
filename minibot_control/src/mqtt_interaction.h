// Author: Robin Chen

#ifndef  MQTT_INTERACTION_H
#define  MQTT_INTERACTION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/StopMotion.h>

#include <vector>
#include <string>

class MQTTInteraction
{
public:
  MQTTInteraction(ros::NodeHandle nh);    // Constructor
  ~MQTTInteraction();                     // Destructor

private:
  ros::NodeHandle nh_;
  ros::Subscriber jnt_sub;
  std::vector<ros::Publisher> jnt_pubs;

  ros::Subscriber halt_sub;
  ros::ServiceClient halt_client;

  ros::Subscriber motion_sub;
  ros::Publisher jnt_cmd_pub;

  ros::Publisher state_pub;
  ros::Publisher time_pub;

  ros::Time start_time;
  ros::Time update_time;
  int num_count = 0;
  int num_mv_jnt = 0;   // check if any joint is moving
  const int num_jnt = 6;
  const double r2d = 57.295779513;
  const double d2r = 0.017453293;
  std::vector<double> jnt_vals = {720., 720., 720., 720., 720., 720.};
  std_msgs::Float64MultiArray ptp_msg;


  void jnt_state_callback(const sensor_msgs::JointState &jnt_msg);
  void halt_msg_callback(const std_msgs::String &str_msg);
  void motion_msg_callback(const std_msgs::String &str_msg);

  std::string& trim(std::string &str, const std::string& end_word="\t\n\v\f\r ");
  std::vector<std::string> split(std::string cmd, std::string delimiter);

};
#endif //MQTT_INTERACTION_H

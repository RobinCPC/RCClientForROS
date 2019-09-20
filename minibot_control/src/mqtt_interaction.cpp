#include "mqtt_interaction.h"

MQTTInteraction::MQTTInteraction(ros::NodeHandle nh)
  : nh_(nh)
{
  start_time = ros::Time::now();
  update_time = start_time;
  // Subscribe to joint_states
  jnt_sub = nh_.subscribe("/joint_states", 5,   // only keep newer joint states
      &MQTTInteraction::jnt_state_callback, this);

  // Publish joint state to mqtt
  for(int i=0; i < num_jnt; ++i)
  {
    std::string topic = "/mqtt_j" + std::to_string(i+1);
    jnt_pubs.push_back(
        nh_.advertise<std_msgs::String>(topic, 5)
        );
  }


  // Catch halt command from mqtt
  halt_sub = nh_.subscribe("/mq_halt", 5,
      &MQTTInteraction::halt_msg_callback, this);

  // Catch motion command from mqtt
  motion_sub = nh_.subscribe("/mq_motion", 5,
      &MQTTInteraction::motion_msg_callback, this);

  jnt_cmd_pub = nh_.advertise<std_msgs::Float64MultiArray>
    ("/minibot/joint_position_controller/command", 5);

  ptp_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ptp_msg.layout.dim[0].size = num_jnt;
  ptp_msg.layout.dim[0].stride = 1;
  ptp_msg.layout.dim[0].label = "Joint";

  // Publish total connecting time and state of robot
  time_pub = nh_.advertise<std_msgs::String>("/mqtt_time", 1);
  state_pub = nh_.advertise<std_msgs::String>("/mqtt_state", 1);

}

MQTTInteraction::~MQTTInteraction(){}

void MQTTInteraction::jnt_state_callback(const sensor_msgs::JointState &jnt_msg)
{
  ros::Duration time_diff = (jnt_msg.header.stamp - update_time);
  if (time_diff.toSec() >= 1.0)
  {
    std_msgs::String str_msg;
    ROS_DEBUG_STREAM("count in one sec: " << num_count );
    ros::Duration power_on_time = jnt_msg.header.stamp - start_time;
    double cur_time = power_on_time.toSec();
    ROS_DEBUG_STREAM("current time" << cur_time );
    int hr = (int) cur_time/3600;
    int mn = (int) cur_time/60;
    int sc = (int) cur_time % 60;
    std::string hms = std::to_string(hr) + ":" + std::to_string(mn) + ":"
      + std::to_string(sc);
    str_msg.data = hms;
    time_pub.publish(str_msg);
    update_time = jnt_msg.header.stamp;

    num_count = 0;
    num_mv_jnt = 0;
    for (int i=0; i < jnt_msg.position.size(); ++i)
    {
      double tmp_deg = jnt_msg.position[i] *r2d;
      if( i == 1) tmp_deg += 90.0;

      if( fabs(jnt_vals[i] - tmp_deg) > 0.1 )
      {
        jnt_vals[i] = tmp_deg;
        ++num_mv_jnt;
      }else
      {
        continue;
      }

      str_msg.data = std::to_string(jnt_vals[i]);
      jnt_pubs[i].publish(str_msg);
    }

    if(num_mv_jnt > 0)
    {
      str_msg.data = "MOVING";
      state_pub.publish(str_msg);
    }
    else
    {
      str_msg.data = "STAND_STILL";
      state_pub.publish(str_msg);
    }
  }
  else
  {
    //ROS_DEBUG_STREAM("time diff: " << time_diff.toSec() );
    num_count++;
  }

  return;
}

void MQTTInteraction::halt_msg_callback(const std_msgs::String &str_msg)
{
  // Make sure "stop_motion" service is on
  ros::service::waitForService("stop_motion", 5);

  halt_client = nh_.serviceClient<industrial_msgs::StopMotion>("stop_motion");
  industrial_msgs::StopMotion halt_srv;
  if(halt_client.call(halt_srv))
  {
    ROS_INFO_STREAM("Halt command send succeed! Response: " << halt_srv.response.code);
  }
  else
  {
    ROS_ERROR_STREAM("Halt command send failed! Response: " << halt_srv.response.code);
  }
  return;
}

void MQTTInteraction::motion_msg_callback(const std_msgs::String &str_msg)
{
  ROS_INFO_STREAM("Receive motion command from AWS:\n" << str_msg.data);
  std::string msg = str_msg.data;
  this->trim(msg);
  std::vector<std::string> tokens = this->split(msg, ":");
  std::string cmd;
  std::vector<std::string> params{};
  if(tokens.size() == 1)
  {
    cmd = this->trim(tokens[0]);
  }
  else if ( tokens.size() == 2)
  {
    cmd = this->trim(tokens[0]);
    this->trim(tokens[1]);
    params = this->split(tokens[1], " ");
  }

  if (cmd.compare("ptp") == 0)
  {
    if(params.size() != 6)
    {
      ROS_INFO_STREAM("ptp command need 6 parameters, only input " << params.size());
      return;
    }
    // Publish ptp command to RC Client
    ptp_msg.data.clear();
    for(int p=0; p < params.size(); ++p)
    {
      double deg = std::stod(params[p]);
      if (p == 1) deg -= 90.0;
      ptp_msg.data.push_back(deg * d2r);
    }
    jnt_cmd_pub.publish(ptp_msg);

  }

  return;
}

std::string& MQTTInteraction::trim(std::string &str, const std::string& end_word)
{
  // left trim
  str.erase(0, str.find_first_not_of(end_word));
  // right trim
  str.erase(str.find_last_not_of(end_word) + 1);
  return str;
}

std::vector<std::string> MQTTInteraction::split(std::string cmd, std::string delimiter)
{
  std::vector<std::string> list;
  std::size_t start = 0;
  std::size_t end = 0;
  std::string token;
  while ((end = cmd.find(delimiter, start)) != std::string::npos)
  {
    std::size_t len = end - start;
    token = cmd.substr(start, len);
    list.emplace_back(token);
    start += len + delimiter.length();
  }
  // catch last one
  token = cmd.substr(start, end - start);
  list.emplace_back(token);
  return list;
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "mqtt_interaction");
  ros::NodeHandle nh;

  MQTTInteraction mqtt_interactor(nh);

  ros::spin();
  return 0;
}

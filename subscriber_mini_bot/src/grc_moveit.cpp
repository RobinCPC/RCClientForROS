// C++ standard headers
#include <exception>
#include <string>
#include <memory>

// ROS headers
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>


class MoveItAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  ros::Publisher jnt_path_pub_;

public:
  MoveItAction(std::string name) :
    as_(nh_, name, boost::bind(&MoveItAction::executeCB, this, _1), false),
    action_name_(name)
  {
    jnt_path_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 50);
    as_.start();
  }

  ~MoveItAction(void)
  {
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    // helper variables
    ros::Rate r(50);
    //bool success = true;

    jnt_path_pub_.publish(goal->trajectory);

    int num_point = goal->trajectory.points.size();
    ROS_INFO_STREAM("Number of trajectory points: " << num_point );
    //for (auto pnt : goal->trajectory.points)
    //{
    //  std::string jnt_val_str = "";
    //  for (auto v : pnt.positions)
    //  {
    //    //std::cout << v << " ";
    //    jnt_val_str += std::to_string(v) + " ";
    //  }
    //  //std::cout << std::endl;
    //  jnt_val_str += "\n";
    //  ROS_INFO_STREAM(jnt_val_str);
    //}

    try
    {
      //ros::Rate::sleep(goal->trajectory.points.end()->time_from_start);
      //goal->trajectory.points.end()->time_from_start.sleep();   // <- cuase run-time error: Duration is out of dual 32-bit range
      ros::Rate(1./goal->trajectory.points[num_point-1].time_from_start.toSec()).sleep();
      ROS_INFO_STREAM( "sleep over!");

      result_.error_code = result_.SUCCESSFUL;
    }
    catch(ros::Exception e)
    {
      ROS_ERROR("ROS Error occurs: %s", e.what());
      result_.error_code = result_.OLD_HEADER_TIMESTAMP;
    }
    as_.setSucceeded(result_);

    return;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grc_client_moveit");

  MoveItAction moveit_sever(ros::this_node::getName());

  //// Start a ROS spining thread
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  ros::spin();

  return 0;
}

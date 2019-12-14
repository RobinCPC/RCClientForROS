// Reference: 
// http://wiki.ros.org/Robots/TIAGo/Tutorials/trajectory_controller
// C++ standard headers
#include <exception>
#include <string>
#include <memory>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// the Action interface type for moving robot arm, provided as typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef std::shared_ptr<arm_control_client> arm_control_client_Ptr;

// Create a ROS action client to move minibot
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/minibot/arm_controller/follow_joint_trajectory"));

  int iterations = 0, max_iteration = 3;
  // wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iteration)
  {
    ROS_DEBUG("Wait for the arm_controller_action server to come up");
    ++iterations;
  }

  if(iterations == max_iteration)
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

// Generate a simple trajectory with two waypoints to move minibot
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("Joint1");
  goal.trajectory.joint_names.push_back("Joint2");
  goal.trajectory.joint_names.push_back("Joint3");
  goal.trajectory.joint_names.push_back("Joint4");
  goal.trajectory.joint_names.push_back("Joint5");
  goal.trajectory.joint_names.push_back("Joint6");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  const double d2r = 0.017453293;
  int index = 0;
  goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = 21.841 * d2r;
  goal.trajectory.points[index].positions[1] = (85.84-90) * d2r;
  goal.trajectory.points[index].positions[2] = 5.052 * d2r;
  goal.trajectory.points[index].positions[3] = -0.149 * d2r;
  goal.trajectory.points[index].positions[4] = -90.203 * d2r;
  goal.trajectory.points[index].positions[5] = 21.841 * d2r;
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for(int j=0; j < 6; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // to be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = -21.841 * d2r;
  goal.trajectory.points[index].positions[1] = (125.84-90) * d2r;
  goal.trajectory.points[index].positions[2] = -34.90 * d2r;
  goal.trajectory.points[index].positions[3] = -0.201 * d2r;
  goal.trajectory.points[index].positions[4] = 45.243 * d2r;
  goal.trajectory.points[index].positions[5] = -21.985 * d2r;
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for(int j=0; j < 6; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // to be reached 4 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_traj_control");

  ROS_INFO("Start simple_traj_control application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if(!ros::Time::waitForValid(ros::WallDuration(10.0))) // Note: Important when using simulated clock
  {
    ROS_FATAL("Time-out wait for valid time.");
    return EXIT_FAILURE;
  }

  // Start a ROS spining thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create an arm controller action client to move minibot
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generate the goal for the minibot
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_goal(arm_goal);

  // Send the command to start the given trajectory is from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

  // wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}

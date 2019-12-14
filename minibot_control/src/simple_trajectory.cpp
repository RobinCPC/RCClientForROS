// Reference:
// http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action#Creating_the_node
// C++ standard headers
#include <exception>
#include <string>
//#include <memory>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <ros/topic.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for the action server to come up
  RobotArm()
  {
    // tell the action client that we want to spin a thread by default.
    traj_client_ = new TrajClient("/minibot/arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up.
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Wait for the joint_trajectory_action server.");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! send command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the given trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtentionTrajectory()
  {
    // Our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint name, which apply to all waypoints.
    goal.trajectory.joint_names.push_back("Joint1");
    goal.trajectory.joint_names.push_back("Joint2");
    goal.trajectory.joint_names.push_back("Joint3");
    goal.trajectory.joint_names.push_back("Joint4");
    goal.trajectory.joint_names.push_back("Joint5");
    goal.trajectory.joint_names.push_back("Joint6");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    const double d2r = 0.017453293;
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 21.841 * d2r;
    goal.trajectory.points[ind].positions[1] = (85.84-90) * d2r;
    goal.trajectory.points[ind].positions[2] = 5.052 * d2r;
    goal.trajectory.points[ind].positions[3] = -0.149 * d2r;
    goal.trajectory.points[ind].positions[4] = -90.203 * d2r;
    goal.trajectory.points[ind].positions[5] = 21.841 * d2r;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(6);
    for(int j=0; j < 6; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // to be reached 2 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);


    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = -21.841 * d2r;
    goal.trajectory.points[ind].positions[1] = (125.84-90) * d2r;
    goal.trajectory.points[ind].positions[2] = -34.90 * d2r;
    goal.trajectory.points[ind].positions[3] = -0.201 * d2r;
    goal.trajectory.points[ind].positions[4] = 45.243 * d2r;
    goal.trajectory.points[ind].positions[5] = -21.985 * d2r;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(6);
    for(int j=0; j < 6; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // to be reached 4 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    // we are done, return goal.
    return goal;
  }

  //! Return the current state of action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};

int main(int argc, char** argv)
{
  // Initial ROS node
  ros::init(argc, argv, "robot_driver");

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

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtentionTrajectory());
  // Wait for trajectory complete
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }


  return EXIT_SUCCESS;
}

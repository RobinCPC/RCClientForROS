#include "planned_path_client.h"


using namespace std;

//Grobal parameter
ROSData_T          gROSData;
I32_T              gSockfd;
struct sockaddr_in gClientInfo;

std::mutex gMutex;
void robotStatePublishWorker(ros::NodeHandle& nh, int rate);


void SendMsgs();


int main( int argc, char **argv )
{
  // Create ROS node first.
  ros::init( argc, argv, "planned_path"  );

  ros::NodeHandle n;

  // load configuration (ip, prot, etc...)
  std::string server_ip = "";
  int server_port = 0;
  n.param<std::string>("robot_ip", server_ip, "10.10.1.81");
  n.param<int>("robot_port", server_port, 27015);


  //Creat client socket
  gSockfd = socket( AF_INET, SOCK_STREAM, 0 );
  if( gSockfd == -1 )
  {
      printf( "Fail to create a socket.\n" );
  }

  struct sockaddr_in *pClintInfo = &gClientInfo;
  bzero( pClintInfo, sizeof( struct sockaddr_in ) );
  pClintInfo->sin_family = PF_INET;
  pClintInfo->sin_addr.s_addr = inet_addr(server_ip.c_str());
  pClintInfo->sin_port = htons( server_port );

  // Show config ip and port
  ROS_INFO_STREAM("Current IP is " << server_ip << " and port is " << server_port);

  int err = connect( gSockfd, (struct sockaddr *)pClintInfo, sizeof( struct sockaddr_in ) );
  if( err == -1 )
  {
      printf( "\nConnection fail.\n\n" );
  }
  else
  {
      printf( "\nConnection success.\n\n" );
  }

  ROSJointParam_T *pJointParam = &gROSData.jointParam;
  GetJointParam( n, pJointParam );

  ros::Subscriber motionReq   = n.subscribe( "/move_group/motion_plan_request", 10000, SubscribeVelAccScale );

  ros::Subscriber plannedPath = n.subscribe( "/move_group/display_planned_path", 10000, SubscribePath );

  std::thread robotStatePublishThread(robotStatePublishWorker, std::ref(n), 50);

  ros::spin();

  printf( "Close socket.\n" );
  close( gSockfd );

  return 0;
}

void SubscribeVelAccScale( const moveit_msgs::MotionPlanRequest::ConstPtr &PScale )
{
  ROSSpeedRatio_T *pSpeedRatio = &gROSData.speedRatio;

  pSpeedRatio->velRatio = PScale->max_velocity_scaling_factor;
  pSpeedRatio->accRatio = PScale->max_acceleration_scaling_factor;

}

void GetJointParam( ros::NodeHandle Handle, ROSJointParam_T *PRetJointParam )
{
  //get joint1~6 Acc limit parameters
  Handle.getParam("/robot_description_planning/joint_limits/Joint1/has_acceleration_limits", PRetJointParam->accFlag[0] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint2/has_acceleration_limits", PRetJointParam->accFlag[1] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint3/has_acceleration_limits", PRetJointParam->accFlag[2] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint4/has_acceleration_limits", PRetJointParam->accFlag[3] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint5/has_acceleration_limits", PRetJointParam->accFlag[4] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint6/has_acceleration_limits", PRetJointParam->accFlag[5] );

  //get joint1~6 Vel limit parameters
  Handle.getParam("/robot_description_planning/joint_limits/Joint1/has_velocity_limits", PRetJointParam->velFlag[0] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint2/has_velocity_limits", PRetJointParam->velFlag[1] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint3/has_velocity_limits", PRetJointParam->velFlag[2] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint4/has_velocity_limits", PRetJointParam->velFlag[3] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint5/has_velocity_limits", PRetJointParam->velFlag[4] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint6/has_velocity_limits", PRetJointParam->velFlag[5] );

  //get joint1~6 max Acc
  Handle.getParam("/robot_description_planning/joint_limits/Joint1/max_acceleration", PRetJointParam->accLimit[0] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint2/max_acceleration", PRetJointParam->accLimit[1] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint3/max_acceleration", PRetJointParam->accLimit[2] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint4/max_acceleration", PRetJointParam->accLimit[3] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint5/max_acceleration", PRetJointParam->accLimit[4] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint6/max_acceleration", PRetJointParam->accLimit[5] );

  //get joint1~6 max Vel
  Handle.getParam("/robot_description_planning/joint_limits/Joint1/max_velocity", PRetJointParam->velLimit[0] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint2/max_velocity", PRetJointParam->velLimit[1] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint3/max_velocity", PRetJointParam->velLimit[2] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint4/max_velocity", PRetJointParam->velLimit[3] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint5/max_velocity", PRetJointParam->velLimit[4] );
  Handle.getParam("/robot_description_planning/joint_limits/Joint6/max_velocity", PRetJointParam->velLimit[5] );
}

void SubscribePath( const moveit_msgs::DisplayTrajectory::ConstPtr &PMsg )
{
  ROSData_T *pROSData   = &gROSData;
  I32_T      totalPoint = PMsg->trajectory[0].joint_trajectory.points.size();
  pROSData->totalPoint  = totalPoint;

  ROSPath_T *pPath;
  pPath = ( ROSPath_T * )malloc( sizeof( ROSPath_T ) * totalPoint );
  for( I32_T pointIndex = 0; pointIndex < totalPoint; pointIndex++ )
  {
    pPath[pointIndex].points[0] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[0];
    pPath[pointIndex].points[1] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[1];
    pPath[pointIndex].points[2] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[2];
    pPath[pointIndex].points[3] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[3];
    pPath[pointIndex].points[4] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[4];
    pPath[pointIndex].points[5] = PMsg->trajectory[0].joint_trajectory.points[pointIndex].positions[5];
  }

  pROSData->pPathData = pPath;

  SendMsgs();
  
  free( pPath );
}

void SendMsgs()
{
  ROSData_T   *pROSData = &gROSData;
  RCPackage_T  pkg;

  //Transfer the velocity limits
  pkg.cmd = RCSVR_CMD_SET_PARAMETER_VEL;
  pkg.pointData.data[0] = pROSData->speedRatio.velRatio;
  pkg.pointData.data[1] = pROSData->jointParam.velLimit[0];
  pkg.pointData.data[2] = pROSData->jointParam.velLimit[1];
  pkg.pointData.data[3] = pROSData->jointParam.velLimit[2];
  pkg.pointData.data[4] = pROSData->jointParam.velLimit[3];
  pkg.pointData.data[5] = pROSData->jointParam.velLimit[4];
  pkg.pointData.data[6] = pROSData->jointParam.velLimit[5];

  send( gSockfd, &pkg, sizeof( RCPackage_T ), 0 );

  //Transfer the acceleration limits
  pkg.cmd = RCSVR_CMD_SET_PARAMETER_ACC;
  pkg.pointData.data[0] = pROSData->speedRatio.accRatio;
  pkg.pointData.data[1] = pROSData->jointParam.accLimit[0];
  pkg.pointData.data[2] = pROSData->jointParam.accLimit[1];
  pkg.pointData.data[3] = pROSData->jointParam.accLimit[2];
  pkg.pointData.data[4] = pROSData->jointParam.accLimit[3];
  pkg.pointData.data[5] = pROSData->jointParam.accLimit[4];
  pkg.pointData.data[6] = pROSData->jointParam.accLimit[5];

  send( gSockfd, &pkg, sizeof( RCPackage_T ), 0 );

  //Transfer the number of total points
  pkg.cmd = RCSVR_CMD_SAVE_TOTAL_POINT;
  pkg.pointData.index = pROSData->totalPoint;

  send( gSockfd, &pkg, sizeof( RCPackage_T ), 0 );

  //Transfer the data of path
  pkg.cmd = RCSVR_CMD_SAVE_POINT_DATA;
  for( I32_T pointIndex = 0; pointIndex < pROSData->totalPoint; pointIndex++ )
  {
    pkg.pointData.index   = ( pointIndex + 1 );
    pkg.pointData.data[0] = pROSData->pPathData[pointIndex].points[0];
    pkg.pointData.data[1] = pROSData->pPathData[pointIndex].points[1];
    pkg.pointData.data[2] = pROSData->pPathData[pointIndex].points[2];
    pkg.pointData.data[3] = pROSData->pPathData[pointIndex].points[3];
    pkg.pointData.data[4] = pROSData->pPathData[pointIndex].points[4];
    pkg.pointData.data[5] = pROSData->pPathData[pointIndex].points[5];

    send( gSockfd, &pkg, sizeof( RCPackage_T ), 0 );
    sleep( 0.1 );
  }

  I32_T ret;
  ret = recv( gSockfd, &pkg, sizeof( RCPackage_T ), 0 );
  
  if( ret != -1 )
    ROS_INFO("Mini Bot motion success!!\n");
}

void robotStatePublishWorker(ros::NodeHandle& nh, int rate)
{
    ros::Publisher jointPublisher = nh.advertise<sensor_msgs::JointState>("joint_states", rate);
    ros::Rate sleeper = ros::Rate(rate);
    RCPackage_T pkg;

    while(ros::ok())
    {
        // Critical Section
        std::lock_guard<std::mutex> mLock(gMutex);

        // Ask server to get current joint position
        pkg.cmd = RCSVR_CMD_GET_ACTUAL_POS;
        pkg.pointData.index = 0;
        send( gSockfd, &pkg, sizeof( RCPackage_T), 0);
        I32_T ret = recv( gSockfd, &pkg, sizeof( RCPackage_T ), 0);

        if( ret != -1 && pkg.cmd == RCSVR_CMD_SEND_ACTUAL_POS )
        {
            ROS_DEBUG("MiniBOT send actual joint values!\n");
            std::string tmp = "";
            for(int i=0; i < 6; ++i)
            {
                tmp += std::to_string(pkg.pointData.data[i]) + " ";
            }
            ROS_DEBUG_STREAM(tmp);
            pkg.pointData.data[1] -= 90.0;
        }
        else
        {
            ROS_INFO("MiniBOT fail to send !\n");
            sleep(1);
            continue;
        }

        std::vector<std::string> jointNames = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
        //publish joint state
        sensor_msgs::JointState jointStateMsg;
        jointStateMsg.header.stamp = ros::Time::now();
        jointStateMsg.header.frame_id = "base_link";
        jointStateMsg.name = jointNames;
        std::vector<double> jointVals = {pkg.pointData.data[0] * DegreeToRadian,
                                         pkg.pointData.data[1] * DegreeToRadian,
                                         pkg.pointData.data[2] * DegreeToRadian,
                                         pkg.pointData.data[3] * DegreeToRadian,
                                         pkg.pointData.data[4] * DegreeToRadian,
                                         pkg.pointData.data[5] * DegreeToRadian,
                                        };
        jointStateMsg.position = jointVals;

        jointPublisher.publish(jointStateMsg);
        sleeper.sleep();

    }

    return;
}

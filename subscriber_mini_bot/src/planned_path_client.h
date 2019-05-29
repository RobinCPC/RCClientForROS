// include header from C++ std library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

// include header from ROS library
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <sensor_msgs/JointState.h>


#define MINIBOTAXES    6
#define DegreeToRadian 0.017453292519943

//Nex type
typedef int      I32_T;
typedef double   F64_T;
typedef int      BOOL_T;


//cmd
#define RCSVR_CMD_SAVE_TOTAL_POINT  ( 0x1 )
#define RCSVR_CMD_SAVE_POINT_DATA   ( 0x2 )
#define RCSVR_CMD_MOTION_DONE       ( 0x3 )
#define RCSVR_CMD_SET_PARAMETER_VEL ( 0x4 )
#define RCSVR_CMD_SET_PARAMETER_ACC ( 0x5 )
#define RCSVR_CMD_GET_ACTUAL_POS    ( 0x6 )
#define RCSVR_CMD_SEND_ACTUAL_POS   ( 0x7 )
#define RCSVR_CMD_MOTION_HALT       ( 0x8 )

//package
#define MAX_POINT_DATA_SIZE (8)
typedef struct
{
    I32_T index;
    F64_T data[MAX_POINT_DATA_SIZE];

} RCPointData_T;

typedef struct
{
    I32_T         cmd;
    RCPointData_T pointData;

} RCPackage_T;

//data structure
typedef struct
{
    F64_T velRatio;
    F64_T accRatio;

} ROSSpeedRatio_T;

typedef struct
{
    BOOL_T accFlag[MINIBOTAXES];
    BOOL_T velFlag[MINIBOTAXES];
    F64_T  accLimit[MINIBOTAXES];
    F64_T  velLimit[MINIBOTAXES];

} ROSJointParam_T;

typedef struct
{
    F64_T points[MINIBOTAXES];

} ROSPath_T;

typedef struct
{
    ROSSpeedRatio_T speedRatio;
    ROSJointParam_T jointParam;
    I32_T           totalPoint;
    ROSPath_T       *pPathData;

} ROSData_T;

//Function
void SubscribeVelAccScale( const moveit_msgs::MotionPlanRequest::ConstPtr &PScale );
void GetJointParam( ros::NodeHandle Handle, ROSJointParam_T *PRetJointParam );
void SubscribePath( const moveit_msgs::DisplayTrajectory::ConstPtr &PMsg );

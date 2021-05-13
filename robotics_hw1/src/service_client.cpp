#include "ros/ros.h"
#include "robotics_hw1/SetZeroPose.h"
#include "robotics_hw1/SetPose.h"
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");
  ros::NodeHandle n;
  ros::ServiceClient client;
  robotics_hw1::SetPose srv_custom;
  robotics_hw1::SetZeroPose srv_zero;

  if (argc == 1)
  {
    ROS_ERROR("Missed arguments: [set-zero | <x> <y> <theta>]");
    return 1;
  }
  if (argc == 2)
  {
    client = n.serviceClient<robotics_hw1::SetZeroPose>("set_zero_pose");
    if (client.call(srv_zero)) {
      ROS_INFO("%s", srv_zero.response.ack.c_str());
    } else {
      ROS_ERROR("Failed to call SetZeroPose service");
    }
  }

  if (argc == 4)
  {
    client = n.serviceClient<robotics_hw1::SetPose>("set_pose");
    srv_custom.request.x = atof(argv[1]);
    srv_custom.request.y = atof(argv[2]);
    srv_custom.request.theta = atof(argv[3]);
    if (client.call(srv_custom)) {
      ROS_INFO("%s", srv_custom.response.ack.c_str());
    } else {
      ROS_ERROR("Failed to call SetPose service");
    }
  }
  
  ros::spinOnce();

  return 0;
}

#include "ros/ros.h"
#include "robotics_hw1/SetZeroPose.h"
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

bool set_zero(robotics_hw1::SetZeroPose::Request &req, robotics_hw1::SetZeroPose::Response &res) {
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);
    res.pose = pose;
    ROS_INFO("Request: set pose to (0,0,0)");
    ROS_INFO("Sending back response");
    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_zero_pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("set_zero_pose", set_zero);

  ROS_INFO("Ready to set the odometry pose to (0,0)");
  ros::spin();

  return 0;
}

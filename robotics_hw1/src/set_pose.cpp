#include "ros/ros.h"
#include "robotics_hw1/SetPose.h"
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

bool set_pose(robotics_hw1::SetPose::Request &req, robotics_hw1::SetPose::Response &res) {
    geometry_msgs::Pose pose;
    pose.position.x = req.x;
    pose.position.y = req.y;
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(req.theta);
    res.pose = pose;
    ROS_INFO("Request: set pose to (%f,%f,%f)", (float)req.x, (double)req.y, (double)req.theta);
    ROS_INFO("Sending back response");
    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("set_pose", set_pose);

  ROS_INFO("Ready to set the odometry pose to custom one given");
  ros::spin();

  return 0;
}

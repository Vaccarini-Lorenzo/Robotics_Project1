#include "ros/ros.h"
#include <sstream>
#include <fstream>
#include <nav_msgs/Odometry.h>

std::fstream f("odom/given_odom.txt");

void callback(const nav_msgs::Odometry::ConstPtr& msg)
              {
              if(f.is_open()){
                ROS_INFO("Writing on file...\n");

                f << msg-> header.seq;
                f << "\n";
                f << msg -> pose.pose.position.x;
                f << "\n";
                f << msg -> pose.pose.position.y;
                f << "\n";
                f << msg -> pose.pose.orientation.z;
                f << "\n";
                f << msg -> pose.pose.orientation.w;
                f << "\n";
                f << msg -> twist.twist.linear.x;
                f << "\n";
                f << msg -> twist.twist.angular.z;
                f << "\n";
              }
              else{
                ROS_INFO("File error");
              }

              };

void close_file(const ros::TimerEvent&){
  f.close();
  ROS_INFO("File closed\n");
  ros::shutdown();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "file_write");
  ros::NodeHandle n;
  ros::Subscriber odom_sub;
  ros::Timer timer;
  ROS_INFO("main...\n");
  odom_sub = n.subscribe("/scout_odom", 1000, &callback);
  timer = n.createTimer(ros::Duration(285), &close_file);
  ros::spin();

  return 0;

}

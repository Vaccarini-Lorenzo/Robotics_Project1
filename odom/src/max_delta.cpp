#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <cmath>

std::fstream max_delta("odom/max_delta.txt");
std::fstream given_odom("odom/given_odom.txt");
std::fstream error_log("odom/error_log.txt");
std::string line;
double x_delta = 0;
double y_delta = 0;
double z_delta = 0;
double w_delta = 0;
double linear_x_delta = 0;
double angular_z_delta = 0;

void close_files(){
  max_delta.close();
  given_odom.close();
  error_log.close();
  ROS_INFO("Closing files...\n");
  ros::shutdown;
}

void confronter(const nav_msgs::Odometry::ConstPtr& msg)
              {
              if(max_delta.is_open() && given_odom.is_open() && std::getline(given_odom, line)){
                double seq = msg-> header.seq;
                double x = msg -> pose.pose.position.x;
                double y = msg -> pose.pose.position.y;
                double z = msg -> pose.pose.orientation.z;
                double w = msg -> pose.pose.orientation.w;
                double linear_x = msg -> twist.twist.linear.x;
                double angular_z = msg -> twist.twist.angular.z;
                if(seq == stoi(line)){
                  getline(given_odom, line);
                  x_delta += abs(stoi(line) - x);
                  getline(given_odom, line);
                  y_delta += abs(stoi(line) - y);
                  getline(given_odom, line);
                  z_delta += abs(stoi(line) - z);
                  getline(given_odom, line);
                  w_delta += abs(stoi(line) - w);
                  getline(given_odom, line);
                  linear_x_delta += abs(stoi(line) - linear_x);
                  getline(given_odom, line);
                  angular_z_delta += abs(stoi(line) - angular_z);
                }
                else if(line == "ENDFILE"){
                  close_files();
                }
                else{
                  if(error_log.is_open()){
                    error_log << "Packet missing:";
                    error_log << seq;
                    error_log << "\n";
                  }
                  else{
                    ROS_INFO("error_log file not found\n");
                  }
                }
              }
              else{
                ROS_INFO("File error");
              }

              };

int main(int argc, char** argv){
  ros::init(argc, argv, "confronter");
  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/odom_output", 1000, confronter);
  ros::spin();

  return 0;

}

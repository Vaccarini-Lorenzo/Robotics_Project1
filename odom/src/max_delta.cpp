#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <cmath>

std::fstream max_delta("odom/max_delta.txt", std::ios_base::app);
std::fstream given_odom("odom/given_odom.txt");
std::fstream error_log("odom/error_log.txt");
std::string line;
double x_delta = 0;
double y_delta = 0;
double z_delta = 0;
double w_delta = 0;
double linear_x_delta = 0;
double angular_z_delta = 0;
ros::NodeHandle n;

void close_files(int incr){
  max_delta << "LOOP:";
  max_delta << incr;
  max_delta << "\nx_delta = ";
  max_delta << x_delta;
  max_delta << "\ny_delta = ";
  max_delta << y_delta;
  max_delta << "\nz_delta = ";
  max_delta << z_delta;
  max_delta << "\nw_delta = ";
  max_delta << w_delta;
  max_delta << "\nlinear_x_delta = ";
  max_delta << linear_x_delta;
  max_delta << "\nangualr_z_delta = ";
  max_delta << angular_z_delta;
  max_delta << "\n";
  max_delta.close();
  given_odom.close();
  error_log.close();
  ROS_INFO("Closing files...\n");
  ros::shutdown();
}

void confronter(const nav_msgs::Odometry::ConstPtr& msg){

              int incremental_var_base_line;
              n.getParam("incremental_var_base_line", incremental_var_base_line);

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
                  close_files(incremental_var_base_line);
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
  ros::Subscriber odom_sub = n.subscribe("/odom_output", 1000, confronter);
  ros::spin();

  return 0;

}

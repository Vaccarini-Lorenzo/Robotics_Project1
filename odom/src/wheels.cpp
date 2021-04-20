#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <odom/MotorSpeed.h>

void callback(const odom::MotorSpeed::ConstPtr& fr_msg,
              const odom::MotorSpeed::ConstPtr& fl_msg,
              const odom::MotorSpeed::ConstPtr& rr_msg,
              const odom::MotorSpeed::ConstPtr& rl_msg){
              ROS_INFO("Received 4 values\n");
              ROS_INFO("fr: rpm value = [%f]\n", fr_msg -> rpm);
              ROS_INFO("fl: rpm value = [%f]\n", fl_msg -> rpm);
              ROS_INFO("rr: rpm value = [%f]\n", rr_msg -> rpm);
              ROS_INFO("rl: rpm value = [%f]\n", rl_msg -> rpm);
              };


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");

  ros::NodeHandle n;

  message_filters::Subscriber<odom::MotorSpeed> fr_sub(n, "/motor_speed_fr", 1);
  message_filters::Subscriber<odom::MotorSpeed> fl_sub(n, "/motor_speed_fl", 1);
  message_filters::Subscriber<odom::MotorSpeed> rr_sub(n, "/motor_speed_rr", 1);
  message_filters::Subscriber<odom::MotorSpeed> rl_sub(n, "/motor_speed_rl", 1);

  typedef message_filters::sync_policies
      ::ApproximateTime<odom::MotorSpeed, odom::MotorSpeed, odom::MotorSpeed, odom::MotorSpeed> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fr_sub, fl_sub, rr_sub, rl_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}

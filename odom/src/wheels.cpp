#include "ros/ros.h"
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <robotics_hw1/MotorSpeed.h>

class wheel_sync {
public:
  robotics_hw1::MotorSpeed fr;
  robotics_hw1::MotorSpeed fl;
  robotics_hw1::MotorSpeed rr;
  robotics_hw1::MotorSpeed rl;

private:
  ros::NodeHandle n;

public:
  wheel_sync() {
    message_filters::Subscriber<robotics_hw1::MotorSpeed> fr_sub(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> fl_sub(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rr_sub(n, "motor_speed_rr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rl_sub(n, "motor_speed_rl", 1);
  }
};

void callback(const robotics_hw1::MotorSpeed& fr_sub,
              const robotics_hw1::MotorSpeed& fl_sub,
              const robotics_hw1::MotorSpeed& rr_sub,
              const robotics_hw1::MotorSpeed& rl_sub){
  ROS_INFO("Received messages: header_seq_fr : %u, Received value : %f\n", fr_sub->header.seq, fr_sub -> rpm);
  ROS_INFO("header_seq_fl : %u, Received value : %f\n", fl_sub->header.seq, fl_sub->rpm);
  ROS_INFO("header_seq_rr : %u, Received value : %f\n", rr_sub->header.seq, rr_sub->rpm);
  ROS_INFO("header_seq_rl : %u, Received value : %f\n", rl_sub->hader.seq, rl_sub->rpm);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_sync");
  typedef message_filters::sync_policies
            ::ApproximateTime<robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,
                              robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fr_sub, fl_sub, rr_sub, rl_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  wheel_sync my_wheel_sync;
  ros::spin();
  return 0;
}

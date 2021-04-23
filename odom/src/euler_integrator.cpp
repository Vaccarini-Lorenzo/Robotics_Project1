#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <odom/WheelSpeed.h>

/*
  Nodo che ascolta il topic su cui pubblica wheels e computa
  un integrazione di eulero
*/
void integrator(const odom::WheelSpeed::ConstPtr& velocities){
  double v_r = (velocities.v_fr + velocities._rr) / 2;
  double v_l = (velocities.v_fl + velocities.v_rl) / 2;
}

int main(int argc char** argv){
  ros::init(argc, argv, "euler_integration");
  ros::NodeHandel n;
  ros::Subscriber sub = n.subscribe("/wheels_velocity", 1000, integrator);


  return 0;
}

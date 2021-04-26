
#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <odom/MotorSpeed.h>
#include <odom/WheelSpeed.h>
#include <geometry_msgs/TwistStamped.h>

#define RADIUS 0.1575
#define RAD_CONVERT 0.10472
#define GEAR_RATIO 0.027027027
#define APPARENT_BASELINE 0.9875

/*
    Aggiunta classe InputConverter e custom message WheelSpeed:
    Pubblicare 4 topic (uno per ogni ruota) mi sembrava un po' superfluo: Il custom
    message WheelSpeed ha gli header e i rpm (convertiti in velocità lineare) di ogni
    ruota in modo da poter pubblicare un singolo messaggio su un singolo topic.
    La classe InputConverter si occupa di fare l'update/publish del messaggio w_speed
    (custom WheelSpeed).
    InputConverter viene instanziato nel main e passato al callback subscriber nel
    quale vengono chiamati i metodi update (angular v -> linear v) e publ.
    La conversione in velocità lineare è stata fatta assumento un gear ratio di 1:37
    e un raggio di 0.1575 m.
    ---
    revolution/min -> radians/sec
*/

class InputConverter {
public:
  double vl, vr;
  geometry_msgs::TwistStamped twist;

private:
  ros::Publisher pub;
  ros::NodeHandle n;

public:
  InputConverter() {
    pub = n.advertise<geometry_msgs::TwistStamped>("/wheels_velocity", 1000); // Il costruttore inizializza il topic su cui pubblichiamo
  }

  void update(const odom::MotorSpeed::ConstPtr& fr_w, const odom::MotorSpeed::ConstPtr& fl_w,
              const odom::MotorSpeed::ConstPtr& rr_w, const odom::MotorSpeed::ConstPtr& rl_w) {
    vr = (fr_w -> rpm * GEAR_RATIO * RADIUS * RAD_CONVERT + rr_w -> rpm * GEAR_RATIO * RADIUS * RAD_CONVERT)/2;
    vl = (fl_w -> rpm * GEAR_RATIO * RADIUS * RAD_CONVERT + rl_w -> rpm * GEAR_RATIO * RADIUS * RAD_CONVERT)/2;

    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "wheels";
    twist.twist.linear.x = (vl + vr) / 2;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    twist.twist.angular.z = (- vl + vr) / APPARENT_BASELINE;
  }

  void publish_message() {
    pub.publish(twist);
  }
};

void callback(const odom::MotorSpeed::ConstPtr& fr_msg,
              const odom::MotorSpeed::ConstPtr& fl_msg,
              const odom::MotorSpeed::ConstPtr& rr_msg,
              const odom::MotorSpeed::ConstPtr& rl_msg,
              InputConverter converter) {
  // ROS_INFO("Received 4 values\n");
  // ROS_INFO("fr: rpm value = [%f]\n", fr_msg -> rpm);
  // ROS_INFO("fl: rpm value = [%f]\n", fl_msg -> rpm);
  // ROS_INFO("rr: rpm value = [%f]\n", rr_msg -> rpm);
  // ROS_INFO("rl: rpm value = [%f]\n", rl_msg -> rpm);
  converter.update(fr_msg, fl_msg, rr_msg, rl_msg);
  converter.publish_message();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  InputConverter input_converter;

  message_filters::Subscriber<odom::MotorSpeed> fr_sub(n, "/motor_speed_fr", 1);
  message_filters::Subscriber<odom::MotorSpeed> fl_sub(n, "/motor_speed_fl", 1);
  message_filters::Subscriber<odom::MotorSpeed> rr_sub(n, "/motor_speed_rr", 1);
  message_filters::Subscriber<odom::MotorSpeed> rl_sub(n, "/motor_speed_rl", 1);

  typedef message_filters::sync_policies
      ::ApproximateTime<odom::MotorSpeed, odom::MotorSpeed, odom::MotorSpeed, odom::MotorSpeed> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fr_sub, fl_sub, rr_sub, rl_sub);

  // Binding 4 topics to instance of InputConverter
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, input_converter));

  ros::spin();

  return 0;

}

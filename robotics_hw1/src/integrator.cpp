#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


class integrator_pub_sub {

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;

  ros::Subscriber sub;
  ros::Publisher pub;

  double x;
  double y;
  double th;
  ros::Time last_time;
  ros::Time current_time;
  char integration_method; // e => euler; r => runge-kutta

public:
  integrator_pub_sub() {
    sub = n.subscribe("/wheels_velocity", 1, &integrator_pub_sub::callback, this);
    pub = n.advertise<nav_msgs::Odometry>("/my_odom", 1);
    last_time = ros::Time::now();
    current_time = ros::Time::now();
    integration_method = 'r';
  }

  void init_odometry(double x, double y, double th) {
    this->x = x;
    this->y = y;
    this->th = th;
  }

  void set_integration_method(char method) {
    this->integration_method = method;
  }

  void euler(const geometry_msgs::TwistStamped::ConstPtr& v, double dt) {
    x += v->twist.linear.x * dt * cos(th);
    y += v->twist.linear.y * dt * sin(th);
    th += v->twist.angular.z * dt;
  }

  void runge_kutta(const geometry_msgs::TwistStamped::ConstPtr& v, double dt) {
    x += v->twist.linear.x * dt * cos(th + (v->twist.angular.z * dt)/2);
    y += v->twist.linear.x * dt * sin(th + (v->twist.angular.z * dt)/2);
    th += v->twist.angular.z * dt;
  }

  void callback(const geometry_msgs::TwistStamped::ConstPtr& v){
    current_time = ros::Time::now();
    double delta_t = (current_time - last_time).toSec();

    tf::Transform transform;
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    if (integration_method == 'e') {
      euler(v, delta_t);
    } else {
      runge_kutta(v, delta_t);
    }

    /*
    - convert the pose to a transform
    - publish the transform
    */
    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion orientation;
    orientation.setRPY(0, 0, th);
    transform.setRotation(orientation);
    broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v->twist.linear.x;
    odom.twist.twist.angular.z = v->twist.angular.z;

    last_time = current_time;
    pub.publish(odom);
    ROS_INFO("\npose:\n x= %f\n y= %f\n theta= %f", x, y, th);
  }
};



int main(int argc, char** argv){
  ros::init(argc, argv, "euler_integration");
  integrator_pub_sub my_integrator;
  my_integrator.init_odometry(0, 0, 0);
  my_integrator.set_integration_method('r');
  ros::spin();
  return 0;
}

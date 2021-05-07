#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class integrator_pub_sub {

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber sub;
  ros::Subscriber pose_sub;
  ros::Publisher pub;
  geometry_msgs::Pose curr_pose;
  double th;
  ros::Time last_time;
  ros::Time current_time;
  char integration_method; // e => euler; r => runge-kutta
public:
  integrator_pub_sub() {
    sub = n.subscribe("/wheels_velocity", 1, &integrator_pub_sub::callback, this);
    pose_sub = n.subscribe("/reset_odom", 1, &integrator_pub_sub::pose_callback, this);
    pub = n.advertise<nav_msgs::Odometry>("/my_odom", 1);

    curr_pose.position.x = 0;
    curr_pose.position.y = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    curr_pose.orientation = tf2::toMsg(q);

    last_time = ros::Time::now();
    current_time = ros::Time::now();
    integration_method = 'r';
  }

  void euler(const geometry_msgs::TwistStamped::ConstPtr& v, double dt) {
    curr_pose.position.x += v->twist.linear.x * dt * cos(th);
    curr_pose.position.y += v->twist.linear.y * dt * sin(th);
    th += v->twist.angular.z * dt;
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    curr_pose.orientation = tf2::toMsg(q);
  }

  void runge_kutta(const geometry_msgs::TwistStamped::ConstPtr& v, double dt) {
    curr_pose.position.x += v->twist.linear.x * dt * cos(th + (v->twist.angular.z * dt)/2);
    curr_pose.position.y += v->twist.linear.x * dt * sin(th + (v->twist.angular.z * dt)/2);
    th += v->twist.angular.z * dt;
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    curr_pose.orientation = tf2::toMsg(q);
  }

  void callback(const geometry_msgs::TwistStamped::ConstPtr& v){
    current_time = ros::Time::now();
    double delta_t = (current_time - last_time).toSec();

    tf::Transform transform;
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    switch (integration_method) {
      case 'e':
        euler(v, delta_t);
        break;
      case 'r':
        runge_kutta(v, delta_t);
        break;
      default:
        runge_kutta(v, delta_t);
        break;
    }

    transform.setOrigin(tf::Vector3(curr_pose.position.x, curr_pose.position.y, 0));
    tf::Quaternion orientation;
    orientation.setRPY(0, 0, th);
    transform.setRotation(orientation);
    broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));

    odom.pose.pose = curr_pose;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v->twist.linear.x;
    odom.twist.twist.angular.z = v->twist.angular.z;

    last_time = current_time;
    pub.publish(odom);
    ROS_INFO("pose:\n\tx= %f\n\ty= %f\n\ttheta= %f", curr_pose.position.x, curr_pose.position.y, th);
  }

  void pose_callback(const geometry_msgs::Pose::ConstPtr& p) {
    // tf::Transform transform;
    tf::Quaternion orientation( p->orientation.x,
                                p->orientation.y,
                                p->orientation.z,
                                p->orientation.w);
    // nav_msgs::Odometry odom;
    //
    // current_time = ros::Time::now();
    curr_pose = *p;
    this->th = tf::getYaw(orientation);

    // odom.header.stamp = current_time;
    // odom.header.frame_id = "odom";
    // odom.pose.pose = curr_pose;
    // odom.child_frame_id = "base_link";
    //
    // transform.setOrigin(tf::Vector3(curr_pose.position.x, curr_pose.position.y, 0));
    // transform.setRotation(orientation);
    // broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
    //
    // last_time = current_time;
    // pub.publish(odom);
    ROS_INFO("Odometry reset triggered");
  }
};


int main(int argc, char** argv){

  ros::init(argc, argv, "integrator");

  integrator_pub_sub my_integrator;

  ros::spin();

  return 0;
}

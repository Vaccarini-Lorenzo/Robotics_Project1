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
#include <dynamic_reconfigure/server.h>
#include "robotics_hw1/CustomOdom.h"
#include <robotics_hw1/integrationConfig.h>
#include <sstream>
#include "robotics_hw1/SetZeroPose.h"
#include "robotics_hw1/SetPose.h"

char integration_method; // e => euler; r => runge-kutta

void parameterCallback(robotics_hw1::integrationConfig &config, uint32_t level){
  ROS_INFO("got parameter = %d", config.integration_method);
  if(config.integration_method == 0){
    integration_method = 'e';
  }
  if(config.integration_method == 1){
    integration_method = 'r';
  }
}

geometry_msgs::Pose parseParam(std::string pose_str, double *th) {
  geometry_msgs::Pose pose;
  double theta;
  std::vector<std::string> v;
  std::stringstream ss(pose_str);

  while (ss.good()) {
    std::string substring;
    getline(ss, substring, ',');
    v.push_back(substring);
  }

  pose.position.x = atof(v[0].c_str());
  pose.position.y = atof(v[1].c_str());
  theta = atof(v[2].c_str());
  *th = theta; // DANGEROUS
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  pose.orientation = tf2::toMsg(q);

  return pose;
}


class integrator_pub_sub {

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::ServiceServer service_zero;
  ros::ServiceServer service_custom;
  geometry_msgs::Pose curr_pose;
  double th;
  std::string initial_pose_str;
  ros::Time last_time;
  ros::Time current_time;

public:
  integrator_pub_sub() {
    sub = n.subscribe("/wheels_velocity", 1, &integrator_pub_sub::callback, this);
    pub = n.advertise<nav_msgs::Odometry>("/my_odom", 1);
    pub1 = n.advertise<robotics_hw1::CustomOdom>("/custom_odom", 1);

    n.getParam("/initial_pose", initial_pose_str);
    curr_pose = parseParam(initial_pose_str, &th);
    ROS_INFO("Position from static parameter: %.3f %.3f %.3f", curr_pose.position.x, curr_pose.position.y, th);

    service_zero = n.advertiseService("set_zero_pose", &integrator_pub_sub::srvcall_zero, this);
    service_custom = n.advertiseService("set_pose", &integrator_pub_sub::srvcall_custom, this);

    last_time = ros::Time::now();
    current_time = ros::Time::now();
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
    robotics_hw1::CustomOdom custom_odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
//    custom_odom.odom.header.stamp  = current_time;
//    custom_odom.odom.header.frame_id = "custom_odom";

    switch (integration_method) {
      case 'e':
        custom_odom.integration_method = "Euler";
        euler(v, delta_t);
        break;
      case 'r':
        custom_odom.integration_method = "Runge-Kutta";
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
    custom_odom.odom = odom;

    last_time = current_time;
    pub.publish(odom);
    pub1.publish(custom_odom);
    ROS_INFO("pose:\n\tx= %.3f\n\ty= %.3f\n\ttheta= %.3f", curr_pose.position.x, curr_pose.position.y, th);
  }

  bool srvcall_zero(robotics_hw1::SetZeroPose::Request &req, robotics_hw1::SetZeroPose::Response &res) {
    curr_pose.position.x = 0;
    curr_pose.position.y = 0;
    curr_pose.position.z = 0;
    curr_pose.orientation = tf::createQuaternionMsgFromYaw(0);
    th = 0;
    res.ack = "Odometry reset to 0,0,0: SUCCESS";
    ROS_INFO("Odometry reset x y th : [%.2f,%.2f,%.2f]", curr_pose.position.x, curr_pose.position.y, th);
    return true;
  }

  bool srvcall_custom(robotics_hw1::SetPose::Request &req, robotics_hw1::SetPose::Response &res) {
    curr_pose.position.x = req.x;
    curr_pose.position.y = req.y;
    curr_pose.position.z = 0;
    curr_pose.orientation = tf::createQuaternionMsgFromYaw(req.theta);
    th = req.theta;
    std::stringstream ssack;
    ssack << "Odometry reset to " << req.x << " " << req.y << " " << req.theta << ": SUCCESS";
    res.ack = ssack.str();
    ROS_INFO("Odometry reset x y th : [%.2f,%.2f,%.2f]", curr_pose.position.x, curr_pose.position.y, th);
    return true;
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "integrator");

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<robotics_hw1::integrationConfig> server;
  dynamic_reconfigure::Server<robotics_hw1::integrationConfig>::CallbackType parameterFunction;
  parameterFunction = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(parameterFunction);

  /* integrator instance */
  integrator_pub_sub my_integrator;

  ros::Rate loop_rate(0.1); // 1 Hz
  ros::spin();

  return 0;
}

#include "ros/ros.h"
#include "robotics_hw1/SetZeroPose.h"
#include "robotics_hw1/SetPose.h"
#include <geometry_msgs/Pose.h>


/*
TODO:
  La classe commentata era la prima versione che avevo provato (incapsulavo client e Publisher
  nella classe).
  La versione con tutto nel main continua a non funzionare.
  Se creo un nodo che fa solo da publisher e pubblico una pose sul topic funziona tutto, integrator
  la legge e setta correttamente l'odometry, se cerco di fare sia service client che publisher nello
  stesso nodo per qualche strano motivo la pose non viene pubblicata sul topic, quindi integrator non vede
  nulla quando viene chiamato il service lato server
*/

// class ServiceClient {
// private:
//   ros::NodeHandle n;
//   ros::Publisher pub;
//   ros::ServiceClient client;
//   robotics_hw1::SetPose srv_custom;
//   robotics_hw1::SetZeroPose srv_zero;
//   geometry_msgs::Pose pose;
// public:
//   ServiceClient(int argc, char **argv) {
//     pub = n.advertise<geometry_msgs::Pose>("reset_odom", 1);
//     switch (argc) {
//       case 2:
//         client = n.serviceClient<robotics_hw1::SetZeroPose>("set_zero_pose");
//         if (client.call(srv_zero)) {
//           pose = srv_zero.response.pose;
//         } else {
//           ROS_ERROR("Failed to call SetZeroPose service");
//         }
//         break;
//
//       case 4:
//         client = n.serviceClient<robotics_hw1::SetPose>("set_pose");
//         srv_custom.request.x = atof(argv[1]);
//         srv_custom.request.y = atof(argv[2]);
//         srv_custom.request.theta = atof(argv[3]);
//         if (client.call(srv_custom)) {
//           pose = srv_custom.response.pose;
//           ROS_INFO("RECEIVED POSE: x[%f] y[%f]", pose.position.x, pose.position.y);
//         } else {
//           ROS_ERROR("Failed to call SetPose service");
//         }
//         break;
//
//       default:
//         ROS_ERROR("Failed to call service:");
//         ROS_ERROR("Insert arguments to call service");
//         break;
//     }
//   }
//
//   void send_pose() {
//     pub.publish(pose);
//     ROS_INFO("Client has published");
//   }
// };

int main(int argc, char **argv)
{
  if (argc == 1) {return 1;}
  ros::init(argc, argv, "service_client");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("reset_odom", 1);
  ros::ServiceClient client;
  robotics_hw1::SetPose srv_custom;
  robotics_hw1::SetZeroPose srv_zero;
  geometry_msgs::Pose p;

  if (argc == 2)
  {
    client = n.serviceClient<robotics_hw1::SetZeroPose>("set_zero_pose");
    if (client.call(srv_zero)) {
      p = srv_zero.response.pose;
      pub.publish(p);
    } else {
      ROS_ERROR("Failed to call SetZeroPose service");
    }
  }

  if (argc == 4)
  {
    client = n.serviceClient<robotics_hw1::SetPose>("set_pose");
    srv_custom.request.x = atof(argv[1]);
    srv_custom.request.y = atof(argv[2]);
    srv_custom.request.theta = atof(argv[3]);
    if (client.call(srv_custom)) {
      p = srv_custom.response.pose;
      pub.publish(p);
    } else {
      ROS_ERROR("Failed to call SetPose service");
    }
  }

  //pub.publish(p);
  ros::spinOnce();


  return 0;
}

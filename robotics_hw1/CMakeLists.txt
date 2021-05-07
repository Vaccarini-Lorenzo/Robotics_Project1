cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_filters
  tf
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorSpeed.msg
  CustomOdom.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetPose.srv
  SetZeroPose.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  nav_msgs
  message_filters
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Wheels
add_executable(wheels src/wheels.cpp)
target_link_libraries(wheels ${catkin_LIBRARIES})
add_dependencies(wheels robotics_hw1_generate_messages_cpp)

## integrator
add_executable(integrator src/integrator.cpp)
target_link_libraries(integrator ${catkin_LIBRARIES})
add_dependencies(integrator
  robotics_hw1_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Service client
add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client ${catkin_LIBRARIES})
add_dependencies(service_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Services:
## SetPose
add_executable(set_pose src/set_pose.cpp)
target_link_libraries(set_pose ${catkin_LIBRARIES})
add_dependencies(set_pose ${catkin_EXPORTED_TARGETS})

## SetZeroPose
add_executable(set_zero_pose src/set_zero_pose.cpp)
target_link_libraries(set_zero_pose ${catkin_LIBRARIES})
add_dependencies(set_zero_pose ${catkin_EXPORTED_TARGETS})

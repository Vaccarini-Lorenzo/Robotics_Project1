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
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorSpeed.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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

## wheels node
add_executable(wheels src/wheels.cpp)
target_link_libraries(wheels ${catkin_LIBRARIES})
add_dependencies(wheels robotics_hw1_generate_messages_cpp)

## integrator node
add_executable(integrator src/integrator.cpp)
target_link_libraries(integrator ${catkin_LIBRARIES})
add_dependencies(integrator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

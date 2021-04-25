cmake_minimum_required(VERSION 3.0.2)
project(odom)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    genmsg
    message_generation
    nav_msgs
    std_msgs
    message_filters
)

add_message_files(
  FILES
    MotorSpeed.msg
    WheelSpeed.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wheels src/wheels.cpp)
target_link_libraries(wheels ${catkin_LIBRARIES})
add_dependencies(wheels ${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_executable(file_write src/t.cpp)
target_link_libraries(file_write ${catkin_LIBRARIES})
add_dependencies(file_write ${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_executable(max_delta src/max_delta.cpp)
target_link_libraries(max_delta ${catkin_LIBRARIES})
add_dependencies(max_delta ${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 3.0.2)
project(motpy_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
 rospy
 darknet_ros_msgs
 message_generation
)

# add_message_files(
#   FILES
#   motpy_bbox.msg
# )
add_service_files(
  FILES
  motpy_bbox.srv
)
generate_messages(
  DEPENDENCIES
  darknet_ros_msgs
)
catkin_package(
  CATKIN_DEPENDS
  rospy
  message_generation
)
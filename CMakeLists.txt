cmake_minimum_required(VERSION 3.0.2)
project(coopmrm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CoopData.msg
  TopologyInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES coopmrm
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


set_source_files_properties(PROPERTIES LANGUAGE CXX)

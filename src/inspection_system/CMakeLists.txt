cmake_minimum_required(VERSION 3.0.2)
project(inspection_system)

find_package(catkin REQUIRED COMPONENTS 
  actionlib_msgs 
  actionlib 
  std_msgs 
  rospy 
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES YoloDetection.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib std_msgs rospy message_runtime
)


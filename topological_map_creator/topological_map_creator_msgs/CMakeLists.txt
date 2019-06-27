cmake_minimum_required(VERSION 2.8.3)
project(topological_map_creator_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs)

add_message_files(
  FILES
  AddDoor.msg
  AddWp.msg
)

add_service_files(
   FILES
   SaveMap.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime
)

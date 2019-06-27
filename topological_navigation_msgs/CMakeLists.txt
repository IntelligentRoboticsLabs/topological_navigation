cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  GetLocation.srv
  SetLocation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topological_navigation_msgs
   CATKIN_DEPENDS geometry_msgs message_generation
#  DEPENDS system_lib
)

cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  bica_planning
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  bica
  move_base_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  costmap_2d
  image_transport
  roslint
  pepper_basic_capabilities_msgs
  topological_navigation_msgs
)


catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES vizbox
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
#    DEPENDS system_lib
)

include_directories(
 	  include
    ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

add_library(TopologicalNav
  src/TopologicalNav.cpp
)
target_link_libraries(TopologicalNav ${catkin_LIBRARIES})
add_dependencies(TopologicalNav ${catkin_EXPORTED_TARGETS})

add_executable(topological_navigation_node
  src/topological_nav_node.cpp
)

add_dependencies(topological_navigation_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(topological_navigation_node ${catkin_LIBRARIES} TopologicalNav)

add_executable(rp_move src/RP_move.cpp)
add_executable(rp_cross src/RP_cross.cpp)
add_executable(rp_navigate src/RP_navigate.cpp)

add_dependencies(rp_move ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_cross ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_navigate ${catkin_EXPORTED_TARGETS} )


target_link_libraries(rp_move ${catkin_LIBRARIES} )
target_link_libraries(rp_cross ${catkin_LIBRARIES} )
target_link_libraries(rp_navigate ${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS
    rp_move
    rp_cross
    rp_navigate
    topological_navigation_node
    TopologicalNav
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY pddl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pddl/
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test test/test_TopologicalNav.launch test/test_TopologicalNav.cpp)

  target_link_libraries(
    ${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    TopologicalNav
  )

endif()

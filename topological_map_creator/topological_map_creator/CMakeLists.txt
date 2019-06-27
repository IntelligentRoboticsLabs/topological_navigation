cmake_minimum_required(VERSION 2.8.3)
project(topological_map_creator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  tf
  rqt_gui
  rqt_gui_cpp
  interactive_markers
  roslint
  topological_map_creator_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tm_creator
  CATKIN_DEPENDS std_msgs roscpp rqt_gui_cpp interactive_markers
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

set(tm_creator_SRCS
  src/tm_creator/TMCreator.cpp
)

set(tm_creator_HDRS
  include/tm_creator/TMCreator.h
)

set(tm_creator_UIS
  src/tm_creator/tm_creator.ui
)

qt5_wrap_cpp(tm_creator_MOCS ${tm_creator_HDRS})
qt5_wrap_ui(tm_creator_UIS_H ${tm_creator_UIS})



add_library(tm_creator
    ${tm_creator_SRCS}
    ${tm_creator_MOCS}
    ${tm_creator_UIS_H}
    )

target_link_libraries(tm_creator
      ${catkin_LIBRARIES}
      Qt5::Widgets
      Qt5::Xml
    )

add_library(tm_creator_lib
  src/tm_creator_lib.cpp
)
target_link_libraries(tm_creator_lib ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})
add_dependencies(tm_creator_lib ${catkin_EXPORTED_TARGETS})

add_executable(tm_creator_node src/tm_creator_node.cpp)
add_dependencies(tm_creator_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tm_creator_node
  tm_creator_lib
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/tm_creator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    tm_creator_lib
    tm_creator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test test/test_TopologicalMapCreator.launch test/test_TopologicalMapCreator.cpp)
  target_link_libraries(
    ${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    tm_creator_lib
  )
endif()

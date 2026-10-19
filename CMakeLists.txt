cmake_minimum_required(VERSION 3.0.2)
project(waypoint_recorder)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs visualization_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_node src/waypoint_node.cpp)
target_link_libraries(waypoint_node
  ${catkin_LIBRARIES}
)

add_executable(waypoint_teleop src/waypoint_teleop.cpp)
target_link_libraries(waypoint_teleop
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(recording_test test/recording.test test/recording_test.cpp)
  target_link_libraries(recording_test ${catkin_LIBRARIES})
  add_dependencies(recording_test waypoint_node)
endif()

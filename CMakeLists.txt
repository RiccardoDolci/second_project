cmake_minimum_required(VERSION 2.8.3)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  message_generation
  actionlib
  actionlib_msgs
  move_base_msgs
  roslib
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf tf2_ros
                 actionlib actionlib_msgs move_base_msgs message_runtime roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odomtotf src/odomtotf.cpp)
add_executable(laserfilternode src/LaserFilterNode.cpp)
add_executable(goal_sender_node src/goal_sender_node.cpp)

target_link_libraries(odomtotf ${catkin_LIBRARIES})
target_link_libraries(laserfilternode ${catkin_LIBRARIES})
target_link_libraries(goal_sender_node ${catkin_LIBRARIES})

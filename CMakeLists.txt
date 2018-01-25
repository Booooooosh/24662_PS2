cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  pluginlib
  tf
  tf_conversions
  std_msgs
  geometry_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

add_message_files(
  FILES
  JointState.msg
)
add_service_files(
  FILES
  MarkState.srv
  ClearStates.srv
  DeleteState.srv
  ShowStates.srv
  ExecuteTrajectory.srv
  GoTo.srv
  Translation.srv
  Upload.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  std_msgs
  geometry_msgs
  message_runtime
)

## build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(denso_robot_interface src/denso_robot_interface.cpp)
add_dependencies(denso_robot_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(denso_robot_interface yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

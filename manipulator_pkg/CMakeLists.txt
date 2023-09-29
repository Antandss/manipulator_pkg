cmake_minimum_required(VERSION 3.0.2)
project(manipulator_pkg)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
    ias_msgs/GoTo.action
    ias_msgs/Pick.action
    ias_msgs/Place.action
   )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulator_pkg
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

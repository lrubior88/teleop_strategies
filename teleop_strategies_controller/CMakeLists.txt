cmake_minimum_required(VERSION 2.8.3)
project(teleop_strategies_controller)

find_package(catkin REQUIRED COMPONENTS 
  visualization_msgs
  geometry_msgs
  teleop_msgs
)

catkin_package(CATKIN_DEPENDS
  smach
  smach_ros
  visualization_msgs
  geometry_msgs
  teleop_msgs
)

catkin_python_setup() 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

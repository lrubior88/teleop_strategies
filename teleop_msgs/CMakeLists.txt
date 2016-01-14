cmake_minimum_required(VERSION 2.8.3)
project(teleop_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation geometry_msgs sensor_msgs std_msgs)

add_message_files(FILES 
		Button5.msg
		Button2.msg
		)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_cleaner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot_cleaner_node1 src/robot_cleaner.cpp)
target_link_libraries(robot_cleaner_node1 ${catkin_LIBRARIES})
add_dependencies(robot_cleaner_node1 beginner_tutorials_gencpp)

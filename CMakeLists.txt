cmake_minimum_required(VERSION 2.8.3)
project(manatee_PID)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manatee_PID
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid src/pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid manatee_PID_generate_messages_cpp)

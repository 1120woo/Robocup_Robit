cmake_minimum_required(VERSION 2.8.3)
project(ik_walk)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy 
  std_msgs 
  msg_generate 
)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ik_walk
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
 add_library(solve_kinematics
	src/solve_kinematics.cpp
 )
 add_library(walk_pattern
   src/walk_pattern.cpp
 )
 add_library(imu_position_control
	src/imu_position_control.cpp
 )
 add_library(zmp_position_control
	src/zmp_position_control.cpp
 )


add_executable(ik_walk src/ik_walk.cpp)

add_dependencies(ik_walk ${catkin_EXPORTED_TARGETS})

 target_link_libraries(ik_walk
   ${catkin_LIBRARIES}
solve_kinematics
walk_pattern
imu_position_control
pid_control_float
zmp_position_control
 )



cmake_minimum_required(VERSION 2.8.3)
project(pong_ball)

########################################################################
## Debug. Lets find these BUGS!
########################################################################
# set(ROS_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Debug)

########################################################################
## Use c++11
########################################################################
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")
endif()

########################################################################
## Find catkin and any catkin packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	message_generation
	roscpp
	rospy
	std_msgs
)

###################################
## Generate messages in the 
## 'msg' folder
###################################
add_message_files(
  FILES
  Pose.msg
)
###################################
## Generate services in the 
## 'srv' folder
###################################
add_service_files(
  FILES
  Reset.srv
  SetPose.srv
)
###################################
## Generate added messages and 
## services with any dependencies 
## listed here
###################################
generate_messages(DEPENDENCIES std_msgs)

########################################################################
#### Build 
########################################################################
	
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime std_msgs
  DEPENDS
)

###################################
## Modify path to the right location
###################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SrcFiles
  src/${PROJECT_NAME}_node.cpp
  src/${PROJECT_NAME}.cpp
)

add_executable(${PROJECT_NAME} ${SrcFiles})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

cmake_minimum_required(VERSION 2.8.3)
project(epos_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  EposLib
)

generate_dynamic_reconfigure_options(
  cfg/Motor.cfg
)

add_message_files(
  FILES
  EPOSControl.msg
  GroupEPOSControl.msg
  GroupMotorInfo.msg
  MotorInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp dynamic_reconfigure message_runtime EposLib
  DEPENDS
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(epos_manager src/EposManager.cpp src/EPOS.cpp)
target_link_libraries(epos_manager
  ${catkin_LIBRARIES}
)


add_dependencies(epos_manager ${PROJECT_NAME}_gencfg)
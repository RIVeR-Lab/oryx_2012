cmake_minimum_required(VERSION 2.8.3)
project(oryx_manager)

find_package(catkin REQUIRED COMPONENTS roscpp oryx_msgs dynamic_reconfigure sensor_msgs epos_manager tf geometry_msgs nav_msgs)

generate_dynamic_reconfigure_options(cfg/ArmManager.cfg cfg/DriveManager.cfg cfg/IMU.cfg cfg/OryxManager.cfg cfg/PTZManager.cfg)


catkin_package(CATKIN_DEPENDS roscpp oryx_msgs dynamic_reconfigure sensor_msgs epos_manager tf geometry_msgs nav_msgs)

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS})

add_executable(drive_manager src/Motors/DriveManager.cpp)
target_link_libraries(drive_manager ${catkin_LIBRARIES})

add_executable(battery_manager src/Internals/BatteryManager.cpp)
target_link_libraries(battery_manager ${catkin_LIBRARIES})

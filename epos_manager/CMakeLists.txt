cmake_minimum_required(VERSION 2.8.3)
project(epos_manager)

find_package(catkin REQUIRED COMPONENTS roscpp oryx_msgs dynamic_reconfigure message_generation)

add_message_files(DIRECTORY msg FILES EPOSControl.msg GroupEPOSControl.msg GroupMotorInfo.msg MotorInfo.msg)
generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(cfg/Motor.cfg)

catkin_package(CATKIN_DEPENDS roscpp oryx_msgs dynamic_reconfigure message_runtime)

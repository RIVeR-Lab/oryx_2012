cmake_minimum_required(VERSION 2.8.3)
project(oryx_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_message_files(DIRECTORY msg FILES Battery.msg Blob.msg BlobList.msg Heartbeat.msg SoftwareStop.msg SwervePositions.msg Temperature.msg WheelVelocities.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy message_runtime)

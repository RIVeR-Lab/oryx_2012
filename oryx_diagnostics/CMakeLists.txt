cmake_minimum_required(VERSION 2.8.3)
project(oryx_diagnostics)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation oryx_msgs)
find_package(Boost REQUIRED COMPONENTS)

add_service_files(DIRECTORY srv FILES DiagnosticsCommand.srv DiagnosticsRegistration.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime oryx_msgs)

include_directories(include src/nodes src/helpers ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/helpers/NodeManager.cpp)
add_dependencies(${PROJECT_NAME} oryx_diagnostics_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(oryx_diagnostics_manager src/nodes/oryx_diagnostics_manager.cpp src/helpers/NodeManager.cpp src/helpers/DiagnosticsManager.cpp)
add_dependencies(oryx_diagnostics_manager oryx_diagnostics_gencpp)
target_link_libraries(oryx_diagnostics_manager ${catkin_LIBRARIES})

add_executable(oryx_diagnostics_client src/nodes/oryx_diagnostics_client.cpp src/helpers/NodeManager.cpp)
add_dependencies(oryx_diagnostics_client oryx_diagnostics_gencpp)
target_link_libraries(oryx_diagnostics_client ${catkin_LIBRARIES})

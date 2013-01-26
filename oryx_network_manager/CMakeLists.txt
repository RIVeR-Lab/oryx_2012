cmake_minimum_required(VERSION 2.8.3)
project(oryx_network_manager)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_package(PkgConfig)

add_message_files(DIRECTORY msg FILES ActiveNetworkConnection.msg ActiveNetworkConnections.msg NetworkConnection.msg NetworkConnections.msg NetworkDevice.msg NetworkDevices.msg)
add_service_files(DIRECTORY srv FILES ActivateConnection.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp message_runtime)

pkg_check_modules(LIB_NM REQUIRED dbus-1 glib-2.0 dbus-glib-1 libnm-glib libnm-util)
include_directories(${catkin_INCLUDE_DIRS} ${LIB_NM_INCLUDE_DIRS})
link_directories(${LIB_NM_LIBRARY_DIRS})

add_executable(server src/NetworkManagerServer.cpp src/NetworkDeviceManager.cpp src/NetworkConnectionManager.cpp src/ActiveNetworkConnectionManager.cpp src/ActivateConnectionService.cpp src/NetworkConfigurationManager.cpp)
target_link_libraries(server ${LIB_NM_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(server oryx_network_manager_gencpp)

add_executable(cli src/NetworkManagerCLI.cpp)
target_link_libraries(cli ${catkin_LIBRARIES})
add_dependencies(cli oryx_network_manager_gencpp)
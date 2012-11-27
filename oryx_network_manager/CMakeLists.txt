cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_genmsg()
rosbuild_gensrv()


rosbuild_add_executable(server src/NetworkManagerServer.cpp)
pkg_check_modules(LIB_NM REQUIRED dbus-1 glib-2.0 dbus-glib-1 libnm-glib libnm-util)
include_directories(${LIB_NM_INCLUDE_DIRS})
link_directories(${LIB_NM_LIBRARY_DIRS})
target_link_libraries(server ${LIB_NM_LIBRARIES})


#rosbuild_add_executable(ui src/NetworkManagerUI.cpp)
#target_link_libraries(ui ${QT_LIBRARIES})

rosbuild_add_executable(cli src/NetworkManagerCLI.cpp)

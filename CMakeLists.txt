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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include_directories(include ${catkin_INCLUDE_DIRS})

set(DRONE_LIB_SOURCES
	src/final_project/drone_main.cpp
	src/final_project/qr_reader.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_executable(qr_reader ${DRONE_LIB_SOURCES})
target_link_libraries(qr_reader ${catkin_LIBRARIES} -lzbar)


rosbuild_add_executable(drone_fly_2 src/final_project/drone_fly_2.cpp)
rosbuild_add_executable(drone_fly_4 src/final_project/drone_fly_4.cpp)
rosbuild_add_executable(circle_finder src/final_project/circle_finder.cpp)
#rosbuild_add_executable(square_finder src/final_project/square_finder.cpp)
rosbuild_add_executable(circle_finder_Martin src/Martins_fun/circle_finder2.cpp)
rosbuild_add_executable(martin_fun src/Martins_fun/martin_fun.cpp)
rosbuild_add_executable(test_off_alti src/Martins_fun/test_off_alti.cpp)

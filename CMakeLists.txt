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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(Display_Images src/test/Image/image.cpp)
rosbuild_add_executable(qr_reader src/final_project/qr_reader.cpp)
target_link_libraries(qr_reader ${catkin_LIBRARIES} -lzbar)
#rosbuild_add_executable(drone_fly src/final_project/drone_fly.cpp)
#rosbuild_add_executable(flypath1 src/test/FlyPath/FlyPath1.cpp)
rosbuild_add_executable(drone_fly_2 src/final_project/drone_fly_2.cpp)
rosbuild_add_executable(circle_finder src/final_project/circle_finder.cpp)
#rosbuild_add_executable(square_finder src/final_project/square_finder.cpp)

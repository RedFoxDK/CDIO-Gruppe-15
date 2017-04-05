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
rosbuild_add_executable(example1_a src/exampler/example1_a.cpp)
rosbuild_add_executable(example1_b src/exampler/example1_b.cpp)
rosbuild_add_executable(example2_a src/exampler/example2_a.cpp)
rosbuild_add_executable(example2_b src/exampler/example2_b.cpp)
rosbuild_add_executable(example3_a src/exampler/example3_a.cpp)
rosbuild_add_executable(example3_b src/exampler/example3_b.cpp)
rosbuild_add_executable(get_battery src/test/Get_Battery/get_battery.cpp)
rosbuild_add_executable(Fly1234 src/test/Fly/fly.cpp)
rosbuild_add_executable(multithred src/test/Multithred/multithred.cpp)
rosbuild_add_executable(flypath1 src/test/FlyPath/FlyPath1.cpp)
rosbuild_add_executable(ARDrone_manual_test src/test/FromTheInternet/ARDrone_manual_test.cpp)
rosbuild_add_executable(flypath2 src/test/FlyPath/FlyPath2.cpp)
rosbuild_add_executable(CDIO-Gruppe-15 src/Final_Project/main.cpp src/Final_Project/Drone_controllor.cpp)

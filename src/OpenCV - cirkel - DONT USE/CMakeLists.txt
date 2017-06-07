cmake_minimum_required(VERSION 2.8)
project( test )
find_package( OpenCV REQUIRED )
add_executable( Track my_tracking.cpp )
target_link_libraries( Track ${OpenCV_LIBS} )


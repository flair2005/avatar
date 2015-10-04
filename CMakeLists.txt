cmake_minimum_required(VERSION 2.8)
project( eyes_builder )
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE Debug)
find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS system filesystem)

add_executable( eyes_builder eyes_builder.cpp timer.h)
target_link_libraries( eyes_builder ${OpenCV_LIBS} ${Boost_LIBRARIES})

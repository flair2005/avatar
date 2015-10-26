cmake_minimum_required(VERSION 2.8)
project( eyes_builder )
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE Debug)
find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS system filesystem)

add_executable(avatar           avatar.cpp avatar.h timer.h)
target_link_libraries(avatar    ${OpenCV_LIBS} ${Boost_LIBRARIES})

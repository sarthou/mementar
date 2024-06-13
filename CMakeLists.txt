cmake_minimum_required(VERSION 3.8)
project(mementar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# link_libraries(stdc++fs)

# Do we really need boost??
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

if($ENV{ROS_VERSION} STREQUAL "1")
    include(cmake/BuildROS1.cmake)
elseif($ENV{ROS_VERSION} STREQUAL "2")
    include(cmake/BuildROS2.cmake)
endif()


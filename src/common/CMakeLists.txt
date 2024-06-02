cmake_minimum_required(VERSION 3.15)
project(common)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DMANIF_NO_DEBUG")

include_directories(${CMAKE_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include(thirdparty/thirdparty.cmake)

add_subdirectory(common)
add_subdirectory(test)

ament_export_include_directories(include)
ament_export_libraries(g3log g3logrotate g3syslog)

ament_package()

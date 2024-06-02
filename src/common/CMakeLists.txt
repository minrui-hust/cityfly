cmake_minimum_required(VERSION 3.15)
project(common)

set(CMAKE_CXX_STANDARD 17)

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

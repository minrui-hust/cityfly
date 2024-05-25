cmake_minimum_required(VERSION 3.15)
project(sensor)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)

include_directories(${CMAKE_SOURCE_DIR} ${rclcpp_INCLUDE_DIRS}
                    ${common_INCLUDE_DIRS})

add_subdirectory(sensor)
add_subdirectory(node)

ament_package()

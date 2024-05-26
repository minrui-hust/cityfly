cmake_minimum_required(VERSION 3.15)
project(sensor)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(common REQUIRED)
find_package(depthai REQUIRED)

add_subdirectory(sensor)
add_subdirectory(node)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

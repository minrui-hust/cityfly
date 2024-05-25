cmake_minimum_required(VERSION 3.15)
project(common)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR} ${rclcpp_INCLUDE_DIRS})

add_subdirectory(common)
add_subdirectory(test)

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

ament_package()

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} depthai::opencv)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs cv_bridge
                          image_transport common)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ # source directory
  DESTINATION include/${PROJECT_NAME} # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.h" # select header files
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

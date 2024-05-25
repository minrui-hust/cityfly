file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ # source directory
  DESTINATION include/${PROJECT_NAME} # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.h" # select header files
)

add_executable(oak_ffc_4p oak_ffc_4p.cpp)
ament_target_dependencies(oak_ffc_4p rclcpp std_msgs)
target_link_libraries(${PROJECT_NAME})
install(TARGETS oak_ffc_4p DESTINATION lib/${PROJECT_NAME})

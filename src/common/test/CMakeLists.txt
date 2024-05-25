add_executable(node_test node_test.cpp)
ament_target_dependencies(node_test rclcpp std_msgs)
install(TARGETS node_test DESTINATION lib/${PROJECT_NAME})

add_executable(oak_ffc_4p oak_ffc_4p.cpp)
target_link_libraries(oak_ffc_4p ${PROJECT_NAME})
install(TARGETS oak_ffc_4p DESTINATION lib/${PROJECT_NAME})

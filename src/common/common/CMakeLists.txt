install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ # source directory
  DESTINATION include/${PROJECT_NAME} # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.h" # select header files
)

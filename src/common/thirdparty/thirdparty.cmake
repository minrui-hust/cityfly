include(ExternalProject)

set(thirdparty_dir ${CMAKE_SOURCE_DIR}/thirdparty)

# g3log
set(g3log_binary_dir ${CMAKE_BINARY_DIR}/thirdparty/g3log)
ExternalProject_Add(
  g3log
  SOURCE_DIR ${thirdparty_dir}/g3log
  BINARY_DIR ${g3log_binary_dir}/build
  CMAKE_ARGS -DUSE_DYNAMIC_LOGGING_LEVELS=ON -DADD_G3LOG_UNIT_TEST=OFF
             -DCMAKE_INSTALL_PREFIX=${g3log_binary_dir}/install)
include_directories(${g3log_binary_dir}/install/include)
install(DIRECTORY ${g3log_binary_dir}/install/include/g3log DESTINATION include)
install(DIRECTORY ${g3log_binary_dir}/install/lib/ DESTINATION lib)

# g3sinks
set(g3sinks_binary_dir ${CMAKE_BINARY_DIR}/thirdparty/g3sinks)
ExternalProject_Add(
  g3sinks
  DEPENDS g3log
  SOURCE_DIR ${thirdparty_dir}/g3sinks
  BINARY_DIR ${g3sinks_binary_dir}/build
  CMAKE_ARGS -DCHOICE_BUILD_TESTS=OFF -DCHOICE_BUILD_EXAMPLES=OFF
             -DCMAKE_PREFIX_PATH=${g3log_binary_dir}/install
             -DCMAKE_INSTALL_PREFIX=${g3sinks_binary_dir}/install)
include_directories(${g3sinks_binary_dir}/install/include)
install(DIRECTORY ${g3sinks_binary_dir}/install/include/g3sinks
        DESTINATION include)
install(DIRECTORY ${g3sinks_binary_dir}/install/lib/ DESTINATION lib)

# msync
include_directories(${thirdparty_dir}/msync)
install(
  DIRECTORY ${thirdparty_dir}/msync/msync # source directory
  DESTINATION include # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.h" # select header files
)

cmake_minimum_required(VERSION 3.15)
project(docker)

find_package(ament_cmake REQUIRED)

install(
  FILES ${CMAKE_SOURCE_DIR}/entrypoint.bash
  DESTINATION share
  PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  OWNER_READ GROUP_READ WORLD_READ
)

ament_package()

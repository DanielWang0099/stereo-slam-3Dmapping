cmake_minimum_required(VERSION 3.16)
project(nvblox_mapping)

find_package(ament_cmake REQUIRED)
include(FetchContent)

FetchContent_Declare(
  magic_enum
  GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
  GIT_TAG v0.9.5
)

FetchContent_MakeAvailable(magic_enum)



install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

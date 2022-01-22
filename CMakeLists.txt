cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(my_pcl_tutorial)

find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

message("include dir: ${PCL_INCLUDE_DIRS}")
message("library dir: ${PCL_LIBRARY_DIRS}")

add_executable(main src/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})

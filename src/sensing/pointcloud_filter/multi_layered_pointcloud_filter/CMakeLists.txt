cmake_minimum_required(VERSION 3.8)
project(multi_layered_pointcloud_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(multi_layered_pointcloud_filter SHARED
  src/multi_layered_pointcloud_filter.cpp
)

ament_auto_package()

cmake_minimum_required(VERSION 3.5)
project(wheel_stuck_common_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(test_wheel_stuck_common_utils test/test_wheel_stuck_common_utils.cpp)
  file(GLOB TEST_SOURCES "test/**/*.cpp")
  target_sources(test_wheel_stuck_common_utils PRIVATE ${TEST_SOURCES})
endif()

ament_auto_package()

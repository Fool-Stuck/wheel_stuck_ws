cmake_minimum_required(VERSION 3.5)
project(ddt_motor_driver)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

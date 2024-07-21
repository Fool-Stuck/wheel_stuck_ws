cmake_minimum_required(VERSION 3.5)
project(joy2twist)

# C++の標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# コンパイラの警告オプションを追加
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを見つける
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# インクルードディレクトリを追加
include_directories(include)

# joy2twistノードのライブラリを追加
ament_auto_add_library(joy2twist SHARED
  src/joy2twist.cpp
)

# joy2twistノードの登録
rclcpp_components_register_node(joy2twist
  PLUGIN "joy2twist::Joy2Twist"
  EXECUTABLE joy2twist_node_exec
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)

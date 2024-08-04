cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocessor)

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

# pointcloud_preprocessorノードのライブラリを追加
ament_auto_add_library(pointcloud_preprocessor SHARED
  src/pointcloud_preprocessor.cpp
)

# pointcloud_preprocessorノードの登録
rclcpp_components_register_node(pointcloud_preprocessor
  PLUGIN "pointcloud_preprocessor::pointcloud_preprocessor"
  EXECUTABLE pointcloud_preprocessor_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)

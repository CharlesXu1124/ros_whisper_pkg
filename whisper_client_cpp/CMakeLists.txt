cmake_minimum_required(VERSION 3.18)
project(whisper_client_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/whisper_client_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN whisper_client::WhisperClientNode
  EXECUTABLE whisper_client_node_exe
)

target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)

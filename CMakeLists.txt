cmake_minimum_required(VERSION 3.5)
project(relaxed_ik_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(NLopt REQUIRED)

generate_parameter_library(
  relaxed_ik_parameters
  src/relaxed_ik_parameters.yaml
)

# MoveIt plugin
add_library(relaxed_ik_plugin SHARED
  src/relaxed_ik_plugin.cpp
  src/objective.cpp
)

target_include_directories(relaxed_ik_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/relaxed_ik>
)

ament_target_dependencies(relaxed_ik_plugin PUBLIC
  moveit_core
  pluginlib
  rclcpp
  tf2_eigen
)
target_link_libraries(relaxed_ik_plugin PUBLIC
  nlopt
  relaxed_ik_parameters
)

pluginlib_export_plugin_description_file(moveit_core relaxed_ik_plugin_description.xml)

install(
  TARGETS relaxed_ik_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

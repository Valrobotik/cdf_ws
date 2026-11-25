cmake_minimum_required(VERSION 3.16)
project(cdf_vacuum_gripper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

add_library(cdf_vacuum_gripper SHARED
  src/vacuum_gripper_plugin.cpp
)
target_include_directories(cdf_vacuum_gripper PRIVATE include)
target_compile_features(cdf_vacuum_gripper PUBLIC cxx_std_17)
target_link_libraries(cdf_vacuum_gripper
  gz-sim::gz-sim
  gz-plugin::register
  rclcpp::rclcpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
)

install(TARGETS cdf_vacuum_gripper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/cdf_vacuum_gripper.sh")

ament_package()

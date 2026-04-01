cmake_minimum_required(VERSION 3.8)
project(mini_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_library(mini_hardware SHARED
  src/diffbot_system.cpp
)

target_include_directories(mini_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(mini_hardware
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(mini_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  mini_hardware.xml
)

install(
  TARGETS mini_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mini_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.8)
project(gripper_rviz_panel)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/gripper_rviz_panel/gripper_panel.hpp
)

add_library(gripper_panel SHARED
  src/gripper_panel.cpp
  ${MOC_FILES}
)

target_include_directories(gripper_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_panel
  pluginlib
  rviz_common
  rclcpp
  std_msgs
)

target_link_libraries(gripper_panel
  Qt5::Widgets
)

install(TARGETS gripper_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(
  rviz_common
  gripper_rviz_panel.xml
)

ament_package()

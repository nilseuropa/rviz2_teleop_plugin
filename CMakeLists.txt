cmake_minimum_required(VERSION 3.8)
project(rviz2_teleop_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(rviz2_teleop_plugin SHARED
  src/teleop_panel.cpp
)

target_include_directories(rviz2_teleop_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rviz2_teleop_plugin
  Qt5::Widgets
)

ament_target_dependencies(rviz2_teleop_plugin
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  sensor_msgs
)

pluginlib_export_plugin_description_file(rviz_common rviz2_teleop_plugin.xml)

install(TARGETS rviz2_teleop_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz2_teleop_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

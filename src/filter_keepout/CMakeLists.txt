cmake_minimum_required(VERSION 3.5)
project(filter_keepout)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(SQLite3 REQUIRED)

include_directories(
  include
)

add_library(keepout_filter_plugin SHARED
  src/keepout_filter.cpp
)

ament_target_dependencies(keepout_filter_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  std_msgs
  tf2_ros
  rclcpp_components
)

target_link_libraries(keepout_filter_plugin SQLite::SQLite3)

target_include_directories(keepout_filter_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_costmap_2d keepout_filter.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS keepout_filter_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES keepout_filter.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY sql/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

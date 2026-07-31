cmake_minimum_required(VERSION 3.8)
project(map_coordinates_edit_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(BUILD_STANDALONE "Build without ament/colcon (standalone cmake install)" ON)

if(NOT BUILD_STANDALONE)
  find_package(ament_cmake REQUIRED)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Network)
find_package(SQLite3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geographic_msgs_INCLUDE_DIRS}
  ${robot_localization_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
)

add_executable(map_coordinates_edit_gui_node
  src/main.cpp
  src/qnode.cpp
  src/ros_bridge.cpp
  src/keepout_db.cpp
  src/pgm_io.cpp
  src/speed_plot.cpp
  src/pgm_assist.cpp
)

target_include_directories(map_coordinates_edit_gui_node PRIVATE include)
qt5_wrap_cpp(MOC_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/qnode.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/speed_plot.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/pgm_assist.hpp
)
target_sources(map_coordinates_edit_gui_node PRIVATE
  ${MOC_SRCS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/qnode.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/speed_plot.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/map_coordinates_edit_gui/pgm_assist.hpp
)

if(NOT BUILD_STANDALONE)
  ament_target_dependencies(map_coordinates_edit_gui_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    geographic_msgs
    robot_localization
    tf2
  )
  target_link_libraries(map_coordinates_edit_gui_node
    Qt5::Widgets
    Qt5::Network
    SQLite::SQLite3
  )
  install(TARGETS
    map_coordinates_edit_gui_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/
    DESTINATION include
  )
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/
  )
  ament_package()
else()
  target_link_libraries(map_coordinates_edit_gui_node
    Qt5::Widgets
    Qt5::Network
    SQLite::SQLite3
    rclcpp::rclcpp
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geographic_msgs_LIBRARIES}
    ${robot_localization_LIBRARIES}
    ${tf2_LIBRARIES}
  )
  install(TARGETS map_coordinates_edit_gui_node RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
endif()

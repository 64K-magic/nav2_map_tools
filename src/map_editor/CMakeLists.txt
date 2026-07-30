cmake_minimum_required(VERSION 3.16)
project(pgm_map_editor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
include_directories(${rclcpp_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${rcl_interfaces_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${rcl_interfaces_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${pcl_conversions_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS} ${tf2_geometry_msgs_LIBRARY_DIRS})

add_executable(pgm_map_editor
    src/main.cpp
)

set_target_properties(pgm_map_editor PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
)

target_link_libraries(pgm_map_editor Qt5::Widgets ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${PCL_LIBRARIES} ${pcl_conversions_LIBRARIES} ${tf2_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

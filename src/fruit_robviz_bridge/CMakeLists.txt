cmake_minimum_required(VERSION 3.8)
project(fruit_robviz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fruit_arm_interfaces REQUIRED)
find_package(fruit_hardware_interfaces REQUIRED)
find_package(fruit_perception REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4.5.4 EXACT REQUIRED
  PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4
  NO_DEFAULT_PATH
  COMPONENTS core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(fruit_robviz_bridge_component SHARED
  src/image_bridge_component.cpp)
target_compile_features(fruit_robviz_bridge_component PUBLIC cxx_std_17)
target_include_directories(fruit_robviz_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fruit_robviz_bridge_component ${OpenCV_LIBRARIES})
ament_target_dependencies(fruit_robviz_bridge_component
  cv_bridge
  fruit_arm_interfaces
  fruit_hardware_interfaces
  fruit_perception
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs)
rclcpp_components_register_node(
  fruit_robviz_bridge_component
  PLUGIN "fruit_robviz_bridge::ImageBridgeComponent"
  EXECUTABLE fruit_robviz_image_bridge)

install(TARGETS fruit_robviz_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/robviz_config_client.py
  scripts/check_preview.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

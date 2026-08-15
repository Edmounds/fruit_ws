cmake_minimum_required(VERSION 3.8)
project(fruit_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(
  OpenCV 4.5.4 EXACT REQUIRED
  PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4
  NO_DEFAULT_PATH
  COMPONENTS core imgproc
)
find_package(cv_bridge REQUIRED)
find_package(OpenVINO 2024.6 EXACT REQUIRED COMPONENTS Runtime)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FruitTypes.msg"
  DEPENDENCIES std_msgs
)

add_library(
  detection_parser
  src/detection_parser.cpp
  src/image_processing.cpp
  src/target_localization.cpp
)
target_include_directories(detection_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(detection_parser PUBLIC cxx_std_17)
target_link_libraries(detection_parser ${OpenCV_LIBRARIES})
ament_target_dependencies(detection_parser sensor_msgs)

add_library(fruit_perception_component SHARED src/fruit_perception_node.cpp)
target_include_directories(fruit_perception_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(fruit_perception_component PUBLIC cxx_std_17)
target_link_libraries(
  fruit_perception_component
  detection_parser
  openvino::runtime
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(
  fruit_perception_component
  ament_index_cpp
  cv_bridge
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(fruit_perception_component "${cpp_typesupport_target}")
rclcpp_components_register_node(
  fruit_perception_component
  PLUGIN "fruit_perception::FruitPerceptionNode"
  EXECUTABLE fruit_perception_node
)

install(TARGETS detection_parser fruit_perception_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch model
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_detection_parser test/test_detection_parser.cpp)
  target_link_libraries(test_detection_parser detection_parser)

  ament_add_gtest(test_image_processing test/test_image_processing.cpp)
  target_link_libraries(test_image_processing detection_parser ${OpenCV_LIBRARIES})
  target_include_directories(test_image_processing PRIVATE ${OpenCV_INCLUDE_DIRS})

  ament_add_gtest(test_target_localization test/test_target_localization.cpp)
  target_link_libraries(test_target_localization detection_parser ${OpenCV_LIBRARIES})
  target_include_directories(test_target_localization PRIVATE ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_target_localization sensor_msgs)

  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

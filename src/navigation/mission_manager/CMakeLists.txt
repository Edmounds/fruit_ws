cmake_minimum_required(VERSION 3.8)
project(mission_manager)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rc2026_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMission.srv"
  "srv/LoadMission.srv"
  "srv/UpdateMissionPoint.srv"
  DEPENDENCIES geometry_msgs
)

add_library(mission_io SHARED
  src/mission_io.cpp
)
set_target_properties(mission_io PROPERTIES
  OUTPUT_NAME mission_manager_mission_io
)
target_include_directories(mission_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_io
  yaml-cpp
)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
)
target_include_directories(mission_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_manager_node
  mission_io
)
ament_target_dependencies(mission_manager_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(mission_manager_panel SHARED
  src/mission_manager_panel.cpp
  include/mission_manager/mission_manager_panel.hpp
)
target_include_directories(mission_manager_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_manager_panel
  Qt5::Widgets
)
ament_target_dependencies(mission_manager_panel
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rc2026_interfaces
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  std_srvs
  tf2
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mission_manager_node "${cpp_typesupport_target}")
target_link_libraries(mission_manager_panel "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS mission_io mission_manager_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch maps missions rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_io
    test/test_mission_io.cpp
  )
  if(TARGET test_mission_io)
    target_link_libraries(test_mission_io
      mission_io
    )
  endif()
endif()

ament_package()

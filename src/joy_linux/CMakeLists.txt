cmake_minimum_required(VERSION 3.5)

project(joy_linux)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  # Look for <linux/joystick.h>
  include(CheckIncludeFiles)
  check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

  if(HAVE_LINUX_JOYSTICK_H)
    include_directories(msg/cpp ${catkin_INCLUDE_DIRS})
    include_directories(include)

    # Component Library
    add_library(joy_linux_component SHARED
      src/joy_linux_node.cpp
    )
    target_include_directories(joy_linux_component PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(joy_linux_component PUBLIC
      ${diagnostic_msgs_TARGETS}
      diagnostic_updater::diagnostic_updater
      rclcpp::rclcpp
      rclcpp_components::component
      ${sensor_msgs_TARGETS}
    )
    rclcpp_components_register_nodes(joy_linux_component "joy_linux::JoyLinux")

    # Standalone Executable
    add_executable(joy_linux_node src/main.cpp)
    target_link_libraries(joy_linux_node PRIVATE
      joy_linux_component
      rclcpp::rclcpp
    )

    install(TARGETS joy_linux_component joy_linux_node
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    install(DIRECTORY include/ DESTINATION include)
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

    add_library(joy_convert_component SHARED src/joy_convert_node.cpp)
    target_include_directories(joy_convert_component PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(joy_convert_component PRIVATE
      rclcpp::rclcpp
      rclcpp_components::component
      ${sensor_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
    )
    rclcpp_components_register_node(joy_convert_component PLUGIN "joy_conventor::JoyConvertNode" EXECUTABLE joy_convert_node)

    install(TARGETS joy_convert_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      find_package(ament_lint_auto REQUIRED)
      set(ament_cmake_copyright_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()

      ament_add_gtest(test_joy_convert_node test/test_joy_convert_node.cpp)
      target_link_libraries(test_joy_convert_node
        joy_convert_component
      )
      ament_target_dependencies(test_joy_convert_node rclcpp sensor_msgs geometry_msgs)
    endif()

  else()
    message(FATAL_ERROR "no <linux/joystick.h>; can't build joy node")
  endif()
endif()


ament_package()

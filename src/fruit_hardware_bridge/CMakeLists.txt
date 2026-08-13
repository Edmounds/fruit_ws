cmake_minimum_required(VERSION 3.8)
project(fruit_hardware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fruit_perception REQUIRED)

add_library(fruit_result_mapping src/fruit_result_mapping.cpp)
target_compile_features(fruit_result_mapping PUBLIC cxx_std_17)
target_include_directories(fruit_result_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(fruit_result_bridge src/fruit_result_bridge.cpp)
target_compile_features(fruit_result_bridge PUBLIC cxx_std_17)
target_link_libraries(fruit_result_bridge fruit_result_mapping)
ament_target_dependencies(fruit_result_bridge fruit_perception rclcpp std_msgs)

install(TARGETS fruit_result_mapping fruit_result_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_fruit_result_mapping test/test_fruit_result_mapping.cpp)
  target_link_libraries(test_fruit_result_mapping fruit_result_mapping)
endif()

ament_package()

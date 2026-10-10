cmake_minimum_required(VERSION 3.8)
project(fruit_robviz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/robviz_config_client.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/check_preview.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_preview)
install(PROGRAMS scripts/check_arm_preview.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_arm_preview)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  fruit_robviz_bridge/video_gateway.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fruit_robviz_image_bridge)
install(PROGRAMS
  fruit_robviz_bridge/waypoint_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_recorder_node)
install(PROGRAMS
  fruit_robviz_bridge/waypoint_editor_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_editor_server)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_recorder test/test_recorder.py)
endif()

ament_package()

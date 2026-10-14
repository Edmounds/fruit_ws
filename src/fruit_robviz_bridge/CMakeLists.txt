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
install(PROGRAMS
  scripts/fruit_runtime.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fruit-runtime)
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
  fruit_robviz_bridge/robviz_web_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robviz_web_server)
install(PROGRAMS
  fruit_robviz_bridge/runtime_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fruit_runtime_server)

add_library(foxglove_service_timeout SHARED
  src/foxglove_service_timeout.cpp
)
target_link_libraries(foxglove_service_timeout ${CMAKE_DL_LIBS})

install(TARGETS foxglove_service_timeout
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_recorder test/test_recorder.py)
endif()

ament_package()

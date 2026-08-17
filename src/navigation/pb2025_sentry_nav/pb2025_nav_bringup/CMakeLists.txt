cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

install(PROGRAMS
  scripts/map_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_initial_pose
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
  ament_add_pytest_test(test_map_initial_pose test/test_map_initial_pose.py)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  behavior_trees
  rviz
)
